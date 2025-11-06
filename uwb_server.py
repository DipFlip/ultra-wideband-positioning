#!/usr/bin/env python3
"""
UWB Server - Up to 8 Anchors with 3D Position Calculation
Reads positions 5, 7, 9, 11, 13, 15, 17, 19 for Anchor IDs 0-7
Calculates 3D position using multilateration and Kalman filtering
"""
import serial
import struct
import time
import json
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
from position_solver import PositionSolver
import numpy as np

latest_data = {
    'timestamp': 0,
    'anchors': {},
    'packet_count': 0,
    'device_port': '',
    'format': 'CmdM:4[ - Auto-detect anchors',
    'position': {
        'x': None,
        'y': None,
        'z': None,
        'success': False,
        'num_anchors': 0,
        'residual': None,
        'velocity': {'x': 0, 'y': 0, 'z': 0}
    },
    'imu': {
        'acc_x': 0,
        'acc_y': 0,
        'acc_z': 0,
        'angle': 0
    }
}

# Global position solver
position_solver = None

# Track last seen time for each anchor
anchor_last_seen = {}
ANCHOR_TIMEOUT = 2.0  # Seconds before considering anchor disconnected

# Calibration data
calibration_config = {}

def load_calibration():
    """Load calibration data from anchor_config.json"""
    global calibration_config
    try:
        with open('anchor_config.json', 'r') as f:
            config = json.load(f)
            calibration_config = config.get('calibration', {
                'per_anchor': {},
                'device_parameters': {}
            })
    except Exception as e:
        print(f"Warning: Could not load calibration config: {e}")
        calibration_config = {
            'per_anchor': {},
            'device_parameters': {}
        }

def save_calibration():
    """Save calibration data to anchor_config.json"""
    try:
        with open('anchor_config.json', 'r') as f:
            config = json.load(f)

        config['calibration'] = calibration_config

        with open('anchor_config.json', 'w') as f:
            json.dump(config, f, indent=2)

        return True
    except Exception as e:
        print(f"Error saving calibration: {e}")
        return False

def apply_calibration(anchor_id, raw_distance):
    """Apply per-anchor calibration correction"""
    if not calibration_config:
        return raw_distance

    per_anchor = calibration_config.get('per_anchor', {})
    anchor_cal = per_anchor.get(str(anchor_id), {})

    scale = anchor_cal.get('scale_factor', 1.0)
    offset = anchor_cal.get('offset_mm', 0.0)

    # Apply correction: corrected = scale * raw + offset_meters
    corrected = scale * raw_distance + (offset / 1000.0)

    return corrected

def fit_calibration(anchor_id):
    """
    Fit linear calibration model for an anchor using collected measurements
    Returns (scale_factor, offset_mm, r_squared, error_message)
    """
    per_anchor = calibration_config.get('per_anchor', {})
    anchor_cal = per_anchor.get(str(anchor_id), {})
    measurements = anchor_cal.get('measurements', [])

    if len(measurements) < 2:
        return None, None, None, "Need at least 2 measurements"

    # Extract true and measured distances
    true_distances = np.array([m['true_distance_m'] for m in measurements])
    measured_distances = np.array([m['measured_distance_m'] for m in measurements])

    # Perform linear regression: true = scale * measured + offset
    # Solve: [measured, 1] * [scale, offset] = true
    A = np.column_stack([measured_distances, np.ones(len(measured_distances))])
    params, residuals, rank, s = np.linalg.lstsq(A, true_distances, rcond=None)

    scale_factor = params[0]
    offset_m = params[1]
    offset_mm = offset_m * 1000.0

    # Calculate R-squared
    ss_res = np.sum((true_distances - (scale_factor * measured_distances + offset_m)) ** 2)
    ss_tot = np.sum((true_distances - np.mean(true_distances)) ** 2)
    r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0

    return scale_factor, offset_mm, r_squared, None

def get_all_anchors_status(current_anchors):
    """
    Build status for all 8 anchors (IDs 0-7)
    Returns dict with anchor data and connection status
    """
    global anchor_last_seen
    current_time = time.time()

    # Update last seen times for anchors with current data
    for anchor_id, distance in current_anchors.items():
        anchor_last_seen[anchor_id] = current_time

    # Build complete anchor status
    all_anchors = {}
    for anchor_id in ['0', '1', '2', '3', '4', '5', '6', '7']:
        if anchor_id in anchor_last_seen:
            time_since_seen = current_time - anchor_last_seen[anchor_id]
            is_connected = time_since_seen < ANCHOR_TIMEOUT

            all_anchors[anchor_id] = {
                'distance': current_anchors.get(anchor_id, None),
                'connected': is_connected,
                'last_seen': anchor_last_seen[anchor_id]
            }
        else:
            # Never seen this anchor
            all_anchors[anchor_id] = {
                'distance': None,
                'connected': False,
                'last_seen': None
            }

    return all_anchors

def parse_uwb_packet(data):
    """Parse CmdM:4[ format - extract up to 8 anchor distances"""
    header = b'CmdM:4['
    if header not in data:
        return None

    start = data.find(header)
    end = data.find(b'\r\n', start)
    if end == -1:
        return None

    packet = data[start:end]
    payload = packet[7:]

    # Parse 16-bit LE values
    values = []
    for i in range(0, len(payload) - 1, 2):
        if i + 1 < len(payload):
            value = struct.unpack('<H', payload[i:i+2])[0]
            values.append(value)

    # Extract anchor distances based on pattern
    # Position 4 is counter/timestamp (SKIP)
    # Anchor positions increment by 2: 5, 7, 9, 11, 13, 15, 17, 19
    anchors = {}

    anchor_positions = {
        '0': 5,
        '1': 7,
        '2': 9,
        '3': 11,
        '4': 13,
        '5': 15,
        '6': 17,
        '7': 19
    }

    for anchor_id, pos in anchor_positions.items():
        if len(values) > pos and 100 < values[pos] < 10000:
            raw_distance = values[pos] / 1000.0
            # Software calibration disabled - calibration applied at device level via AT+SETDEV
            # calibrated_distance = apply_calibration(anchor_id, raw_distance)
            anchors[anchor_id] = raw_distance

    return anchors if anchors else None

def parse_imu_data(response):
    """Parse AT+GETSENSOR response to extract IMU data
    NOTE: This only works on the TTL port, not the USB port of the BU03 module."""
    imu_data = {'acc_x': 0, 'acc_y': 0, 'acc_z': 0, 'angle': 0}

    try:
        lines = response.split('\n')
        acc_values = []

        for line in lines:
            line = line.strip()
            if 'acc_x:' in line or 'acc_y:' in line or 'acc_z:' in line:
                try:
                    value = float(line.split(':')[1].strip())
                    acc_values.append(value)
                except:
                    pass
            elif 'angle:' in line:
                try:
                    imu_data['angle'] = float(line.split(':')[1].strip())
                except:
                    pass

        # Assign acc values in order (x, y, z)
        if len(acc_values) >= 3:
            imu_data['acc_x'] = acc_values[0]
            imu_data['acc_y'] = acc_values[1]
            imu_data['acc_z'] = acc_values[2]

    except Exception as e:
        print(f"Error parsing IMU data: {e}")

    return imu_data

def read_imu_data(ser):
    """Send AT+GETSENSOR command and read IMU data
    NOTE: This only works on the TTL port, not the USB port of the BU03 module."""
    global latest_data

    while True:
        try:
            # Send AT+GETSENSOR command
            ser.write(b'AT+GETSENSOR\r\n')
            time.sleep(0.1)

            # Read response
            response = b''
            start_time = time.time()
            while time.time() - start_time < 0.5:
                if ser.in_waiting > 0:
                    response += ser.read(ser.in_waiting)
                    time.sleep(0.05)
                elif response and b'OK' in response:
                    break

            # Parse IMU data
            response_str = response.decode('utf-8', errors='ignore')
            if 'acc' in response_str:
                imu_data = parse_imu_data(response_str)
                latest_data['imu'] = imu_data

        except Exception as e:
            print(f"Error reading IMU data: {e}")

        # Read IMU data every 0.5 seconds
        time.sleep(0.5)

def read_uwb_data(port='/dev/ttyACM0'):
    """Read UWB data with auto-reconnect on disconnection"""
    global latest_data

    print(f"Starting UWB data reader on {port}")

    ser = None
    buffer = b''
    packet_count = 0
    detected_anchors = set()
    reconnect_delay = 2.0  # seconds

    def connect_serial():
        """Attempt to connect to serial port"""
        try:
            s = serial.Serial(port, 115200, timeout=0.1)
            time.sleep(0.5)
            print(f"✓ Connected to {port}")
            return s
        except serial.SerialException as e:
            print(f"Could not open {port}: {e}")
            return None

    # Initial connection
    ser = connect_serial()
    if ser is None:
        print("\nPlease specify the correct port. Example:")
        print(f"  python3 uwb_server.py /dev/ttyACM1 8080")
        import sys
        sys.exit(1)

    while True:
        try:
            # Check if port is still open
            if ser is None or not ser.is_open:
                print(f"Serial port disconnected. Attempting to reconnect...")
                time.sleep(reconnect_delay)
                ser = connect_serial()
                if ser is None:
                    time.sleep(reconnect_delay)
                    continue
                buffer = b''  # Clear buffer on reconnect

            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                buffer += chunk

                # Process UWB packets
                while b'CmdM:4[' in buffer and b'\r\n' in buffer:
                    start = buffer.find(b'CmdM:4[')
                    end = buffer.find(b'\r\n', start)

                    if start != -1 and end != -1:
                        packet = buffer[start:end+2]
                        buffer = buffer[end+2:]

                        anchors = parse_uwb_packet(packet)

                        if anchors:
                            packet_count += 1

                            # Track newly detected anchors
                            new_anchors = set(anchors.keys()) - detected_anchors
                            if new_anchors:
                                detected_anchors.update(new_anchors)
                                print(f"✓ Detected anchors: {sorted(detected_anchors, key=int)}")

                            # Get full status for all 8 anchors
                            all_anchors_status = get_all_anchors_status(anchors)

                            # Calculate 3D position if solver is available
                            position_data = {
                                'x': None,
                                'y': None,
                                'z': None,
                                'success': False,
                                'num_anchors': 0,
                                'residual': None,
                                'velocity': {'x': 0, 'y': 0, 'z': 0}
                            }

                            if position_solver:
                                result = position_solver.solve(anchors)
                                position_data['success'] = result['success']
                                position_data['num_anchors'] = result['num_anchors']
                                position_data['residual'] = result['residual']

                                if result['success'] and result['position']:
                                    position_data['x'] = result['position'][0]
                                    position_data['y'] = result['position'][1]
                                    position_data['z'] = result['position'][2]

                                    if result['velocity']:
                                        position_data['velocity']['x'] = result['velocity'][0]
                                        position_data['velocity']['y'] = result['velocity'][1]
                                        position_data['velocity']['z'] = result['velocity'][2]

                            latest_data.update({
                                'timestamp': time.time(),
                                'anchors': all_anchors_status,
                                'packet_count': packet_count,
                                'device_port': port,
                                'format': f'CmdM:4[ - {len(anchors)} Anchors',
                                'position': position_data
                            })

                if len(buffer) > 5000:
                    buffer = buffer[-2000:]

            time.sleep(0.01)

        except (serial.SerialException, OSError) as e:
            # Handle both serial exceptions and I/O errors (e.g., errno 5 when cable unplugged)
            print(f"Serial/IO error: {e}")
            if ser:
                try:
                    ser.close()
                except:
                    pass
            ser = None
            buffer = b''  # Clear buffer on reconnect
            print(f"Will attempt reconnect in {reconnect_delay}s...")
            time.sleep(reconnect_delay)

        except Exception as e:
            print(f"Error reading UWB data: {e}")
            time.sleep(0.1)

class UWBRequestHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/data':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(latest_data).encode())

        elif self.path == '/anchor_config.json':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            try:
                with open('anchor_config.json', 'rb') as f:
                    self.wfile.write(f.read())
            except FileNotFoundError:
                self.wfile.write(b'{}')

        elif self.path == '/calibration/status':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(calibration_config).encode())

        elif self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            with open('uwb_dashboard.html', 'rb') as f:
                self.wfile.write(f.read())
        else:
            super().do_GET()

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)

        if self.path == '/update_params':
            try:
                params = json.loads(post_data.decode('utf-8'))

                # Update position solver parameters
                if position_solver:
                    if 'process_noise' in params:
                        position_solver.kalman.update_process_noise(params['process_noise'])
                        print(f"✓ Updated process_noise: {params['process_noise']}")

                    if 'measurement_noise' in params:
                        position_solver.kalman.update_measurement_noise(params['measurement_noise'])
                        print(f"✓ Updated measurement_noise: {params['measurement_noise']}")

                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'ok'}).encode())

            except Exception as e:
                print(f"Error updating parameters: {e}")
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())

        elif self.path == '/calibration/add_measurement':
            try:
                data = json.loads(post_data.decode('utf-8'))
                anchor_id = str(data['anchor_id'])
                true_distance = float(data['true_distance_m'])

                # Get current measured distance from latest_data
                current_distance = latest_data['anchors'].get(anchor_id, {}).get('distance')

                if current_distance is None:
                    raise ValueError(f"No distance measurement available for anchor {anchor_id}")

                # Ensure calibration structure exists
                if 'per_anchor' not in calibration_config:
                    calibration_config['per_anchor'] = {}
                if anchor_id not in calibration_config['per_anchor']:
                    calibration_config['per_anchor'][anchor_id] = {
                        'scale_factor': 1.0,
                        'offset_mm': 0.0,
                        'measurements': []
                    }

                # Add measurement
                measurement = {
                    'true_distance_m': true_distance,
                    'measured_distance_m': current_distance,
                    'timestamp': time.time()
                }
                calibration_config['per_anchor'][anchor_id]['measurements'].append(measurement)

                # Save to config file
                save_calibration()

                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'status': 'ok',
                    'measurement': measurement,
                    'total_measurements': len(calibration_config['per_anchor'][anchor_id]['measurements'])
                }).encode())

            except Exception as e:
                print(f"Error adding calibration measurement: {e}")
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())

        elif self.path == '/calibration/fit':
            try:
                data = json.loads(post_data.decode('utf-8'))
                anchor_id = str(data['anchor_id'])

                scale, offset_mm, r_squared, error = fit_calibration(anchor_id)

                if error:
                    raise ValueError(error)

                # Update calibration parameters
                calibration_config['per_anchor'][anchor_id]['scale_factor'] = float(scale)
                calibration_config['per_anchor'][anchor_id]['offset_mm'] = float(offset_mm)

                # Save to config
                save_calibration()

                print(f"✓ Fitted calibration for anchor {anchor_id}: scale={scale:.4f}, offset={offset_mm:.2f}mm, R²={r_squared:.4f}")

                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'status': 'ok',
                    'scale_factor': scale,
                    'offset_mm': offset_mm,
                    'r_squared': r_squared
                }).encode())

            except Exception as e:
                print(f"Error fitting calibration: {e}")
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())

        elif self.path == '/calibration/clear':
            try:
                data = json.loads(post_data.decode('utf-8'))
                anchor_id = str(data['anchor_id'])

                if anchor_id in calibration_config.get('per_anchor', {}):
                    calibration_config['per_anchor'][anchor_id]['measurements'] = []
                    calibration_config['per_anchor'][anchor_id]['scale_factor'] = 1.0
                    calibration_config['per_anchor'][anchor_id]['offset_mm'] = 0.0
                    save_calibration()

                print(f"✓ Cleared calibration for anchor {anchor_id}")

                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'ok'}).encode())

            except Exception as e:
                print(f"Error clearing calibration: {e}")
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())

        elif self.path == '/calibration/read_device':
            try:
                from bu03_util import BU03Device

                # Read current device parameters
                with BU03Device() as device:
                    response = device.get_device_params()

                # Parse the response if possible
                # Response format unclear from docs, so just return raw
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'status': 'ok',
                    'response': response
                }).encode())

            except Exception as e:
                print(f"Error reading device parameters: {e}")
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())

        elif self.path == '/calibration/reset_device':
            try:
                from bu03_util import BU03Device

                # Reset to neutral calibration (raw measurements)
                with BU03Device() as device:
                    device.set_device_params(
                        label_rate=5,
                        antenna_delay=16336,
                        kalman_enable=1,
                        kalman_q=0.018,
                        kalman_r=0.642,
                        correction_a=1.0,    # No scale correction
                        correction_b=0.0,    # No offset correction
                        positioning_enable=0,
                        positioning_dim=0
                    )

                print(f"✓ Reset device calibration to a=1.0, b=0.0 (raw measurements)")

                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'ok'}).encode())

            except Exception as e:
                print(f"Error resetting device calibration: {e}")
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())

        elif self.path == '/calibration/apply_device':
            try:
                from bu03_util import BU03Device

                # Calculate average calibration across all anchors with data
                per_anchor = calibration_config.get('per_anchor', {})

                # Collect all fitted calibrations
                scale_factors = []
                offsets = []

                for anchor_id in range(8):
                    anchor_cal = per_anchor.get(str(anchor_id), {})
                    measurements = anchor_cal.get('measurements', [])

                    if len(measurements) >= 2:  # Only use anchors with fitted calibration
                        scale = anchor_cal.get('scale_factor', 1.0)
                        offset = anchor_cal.get('offset_mm', 0.0)

                        # Only include if calibrated (not default values)
                        if scale != 1.0 or offset != 0.0:
                            scale_factors.append(scale)
                            offsets.append(offset)

                # Use average if we have calibrated anchors, otherwise defaults
                if scale_factors:
                    avg_scale = sum(scale_factors) / len(scale_factors)
                    avg_offset = sum(offsets) / len(offsets)
                    print(f"Using average calibration: scale={avg_scale:.4f}, offset={avg_offset:.2f}mm")
                    print(f"  (averaged across {len(scale_factors)} calibrated anchors)")
                else:
                    avg_scale = 1.0
                    avg_offset = 0.0
                    print("No calibrated anchors found, using defaults")

                # Connect to device and apply parameters
                with BU03Device() as device:
                    device.set_device_params(
                        label_rate=5,           # Default tag refresh rate
                        antenna_delay=16336,    # Default antenna delay
                        kalman_enable=1,        # Enable Kalman filter
                        kalman_q=0.018,         # Default process noise
                        kalman_r=0.642,         # Default measurement noise
                        correction_a=avg_scale, # Unitless scale factor
                        correction_b=avg_offset, # Offset in millimeters (not meters!)
                        positioning_enable=0,   # Disable on-device positioning
                        positioning_dim=0
                    )

                print(f"✓ Applied calibration to device: a={avg_scale:.4f}, b={avg_offset:.2f}mm")

                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'status': 'ok',
                    'scale_factor': avg_scale,
                    'offset_mm': avg_offset,
                    'num_anchors_used': len(scale_factors)
                }).encode())

            except Exception as e:
                print(f"Error applying device calibration: {e}")
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())

        elif self.path == '/calibration/write_device':
            try:
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)
                params = json.loads(post_data.decode('utf-8'))

                from bu03_util import BU03Device

                # Extract all 9 parameters from request
                label_rate = params.get('label_rate', 5)
                antenna_delay = params.get('antenna_delay', 16336)
                kalman_enable = params.get('kalman_enable', 1)
                kalman_q = params.get('kalman_q', 0.018)
                kalman_r = params.get('kalman_r', 0.642)
                correction_a = params.get('correction_a', 1.0)
                correction_b = params.get('correction_b', 0.0)
                positioning_enable = params.get('positioning_enable', 0)
                positioning_dim = params.get('positioning_dim', 0)

                print(f"Writing device parameters:")
                print(f"  X1 (Label Rate): {label_rate}")
                print(f"  X2 (Antenna Delay): {antenna_delay}")
                print(f"  X3 (Kalman Enable): {kalman_enable}")
                print(f"  X4 (Kalman Q): {kalman_q}")
                print(f"  X5 (Kalman R): {kalman_r}")
                print(f"  X6 (Correction A): {correction_a}")
                print(f"  X7 (Correction B): {correction_b} mm")
                print(f"  X8 (Positioning Enable): {positioning_enable}")
                print(f"  X9 (Positioning Dim): {positioning_dim}")

                # Connect to device and write all parameters
                with BU03Device() as device:
                    device.set_device_params(
                        label_rate=label_rate,
                        antenna_delay=antenna_delay,
                        kalman_enable=kalman_enable,
                        kalman_q=kalman_q,
                        kalman_r=kalman_r,
                        correction_a=correction_a,
                        correction_b=correction_b,
                        positioning_enable=positioning_enable,
                        positioning_dim=positioning_dim
                    )

                print(f"✓ Parameters written to device (saved and rebooting)")

                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'status': 'ok',
                    'params': params
                }).encode())

            except Exception as e:
                print(f"Error writing device parameters: {e}")
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())

        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass

def run_server(port=8080):
    try:
        server = HTTPServer(('0.0.0.0', port), UWBRequestHandler)
        print(f"\nServer running on http://localhost:{port}")
        server.serve_forever()
    except OSError as e:
        if e.errno == 98:  # Address already in use
            print(f"\nError: Port {port} is already in use!")
            print("\nOptions:")
            print(f"  1. Kill the existing server:")
            print(f"     pkill -f 'uwb_server'")
            print(f"  2. Use a different port:")
            print(f"     python3 uwb_server.py /dev/ttyACM0 {port + 1}")
            import sys
            sys.exit(1)
        else:
            raise

if __name__ == "__main__":
    import sys
    import os

    # Parse command line arguments
    http_port = int(sys.argv[2]) if len(sys.argv) > 2 else 8080

    # Auto-detect UWB port if not specified
    if len(sys.argv) > 1:
        uwb_port = sys.argv[1]
    else:
        # Try ACM0 first, then ACM1
        uwb_port = None
        for port_candidate in ['/dev/ttyACM0', '/dev/ttyACM1']:
            if os.path.exists(port_candidate):
                uwb_port = port_candidate
                print(f"Auto-detected device at {uwb_port}")
                break

        if uwb_port is None:
            print("Error: No UWB device found!")
            print("\nSearched for: /dev/ttyACM0, /dev/ttyACM1")
            print("\nPlease connect a device or specify the port manually:")
            print("  python3 uwb_server.py /dev/ttyACM0 8080")
            sys.exit(1)

    # Initialize position solver
    print("\n" + "="*50)
    print("Initializing 3D Position Solver")
    print("="*50)
    try:
        position_solver = PositionSolver('anchor_config.json')
        print("✓ Position solver ready")
    except Exception as e:
        print(f"Warning: Could not initialize position solver: {e}")
        print("Continuing without 3D positioning...")
        position_solver = None

    # Load calibration data
    print("\n" + "="*50)
    print("Loading Calibration Data")
    print("="*50)
    load_calibration()
    print("✓ Calibration data loaded")

    uwb_thread = threading.Thread(target=read_uwb_data, args=(uwb_port,), daemon=True)
    uwb_thread.start()

    time.sleep(1)
    run_server(http_port)
