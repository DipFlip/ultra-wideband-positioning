#!/usr/bin/env python3
"""
UWB Server - Up to 8 Anchors
Reads positions 5, 7, 9, 11, 13, 15, 17, 19 for Anchor IDs 0-7
Only displays anchors with valid distance data
"""
import serial
import struct
import time
import json
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading

latest_data = {
    'timestamp': 0,
    'anchors': {},
    'packet_count': 0,
    'device_port': '',
    'format': 'CmdM:4[ - Auto-detect anchors'
}

# Track last seen time for each anchor
anchor_last_seen = {}
ANCHOR_TIMEOUT = 2.0  # Seconds before considering anchor disconnected

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
            anchors[anchor_id] = values[pos] / 1000.0

    return anchors if anchors else None

def read_uwb_data(port='/dev/ttyACM0'):
    """Read UWB data"""
    global latest_data

    print(f"Starting UWB data reader on {port}")

    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(0.5)
    except serial.SerialException as e:
        print(f"\nError: Could not open {port}: {e}")
        print("\nPlease specify the correct port. Example:")
        print(f"  python3 uwb_server.py /dev/ttyACM1 8080")
        import sys
        sys.exit(1)

    buffer = b''
    packet_count = 0
    detected_anchors = set()

    while True:
        try:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                buffer += chunk

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

                            latest_data = {
                                'timestamp': time.time(),
                                'anchors': all_anchors_status,
                                'packet_count': packet_count,
                                'device_port': port,
                                'format': f'CmdM:4[ - {len(anchors)} Anchors'
                            }

                if len(buffer) > 5000:
                    buffer = buffer[-2000:]

            time.sleep(0.01)

        except Exception as e:
            print(f"Error: {e}")
            time.sleep(1)

class UWBRequestHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/data':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(latest_data).encode())

        elif self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            with open('uwb_dashboard.html', 'rb') as f:
                self.wfile.write(f.read())
        else:
            super().do_GET()

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

    uwb_thread = threading.Thread(target=read_uwb_data, args=(uwb_port,), daemon=True)
    uwb_thread.start()

    time.sleep(1)
    run_server(http_port)
