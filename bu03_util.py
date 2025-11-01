#!/usr/bin/env python3
"""
Simple BU03 UWB Module Utility
Handles the quirks of the BU03 (like auto-reboot after AT+SAVE)
"""
import serial
import time
import sys

PORT = '/dev/ttyUSB0'
BAUD = 115200

class BU03Device:
    def __init__(self, port=PORT, baud=BAUD):
        self.port = port
        self.baud = baud
        self.ser = None
        self.connect()

    def connect(self):
        """Connect to the device"""
        if self.ser and self.ser.is_open:
            self.ser.close()

        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False
        )
        time.sleep(0.5)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def send(self, cmd, wait=0.5):
        """Send command and return response"""
        self.ser.reset_input_buffer()
        self.ser.write((cmd + '\r\n').encode())
        self.ser.flush()
        time.sleep(wait)

        response = b''
        start = time.time()
        while time.time() - start < 2:
            if self.ser.in_waiting > 0:
                response += self.ser.read(self.ser.in_waiting)
                time.sleep(0.1)
            elif response:
                break
            else:
                time.sleep(0.05)

        return response.decode('utf-8', errors='replace').strip()

    def send_with_reboot(self, cmd, reboot_wait=3):
        """Send command that causes device to reboot (like AT+SAVE)"""
        print(f"Sending: {cmd}")
        response = self.send(cmd)
        print(f"Response: {response}")

        if "OK" in response:
            print(f"Command successful. Device will reboot...")
            print(f"Waiting {reboot_wait} seconds for reboot to complete...")
            time.sleep(reboot_wait)

            # Read and discard boot messages
            if self.ser.in_waiting > 0:
                boot_msg = self.ser.read(self.ser.in_waiting)
                print(f"Boot messages received ({len(boot_msg)} bytes)")

            # Reconnect to ensure clean state
            self.connect()
            print("Reconnected to device")

        return response

    def get_config(self):
        """Get current configuration"""
        response = self.send("AT+GETCFG")
        print(f"Current configuration:\n{response}")
        return response

    def set_config(self, device_id, role, channel, rate, save=True):
        """
        Set configuration and optionally save

        device_id: 0-10
        role: 0=tag, 1=base station
        channel: 0=channel 9, 1=channel 5
        rate: 0=850K, 1=6.8M
        save: automatically save config
        """
        cmd = f"AT+SETCFG={device_id},{role},{channel},{rate}"
        response = self.send(cmd)
        print(f"Set config: {response}")

        if save and "OK" in response:
            print("\nSaving configuration (device will reboot)...")
            self.send_with_reboot("AT+SAVE")

        return response

    def get_version(self):
        """Get software version"""
        return self.send("AT+GETVER")

    def get_distance(self):
        """Get distance measurement"""
        response = self.send("AT+DISTANCE")
        # Parse distance value
        try:
            # Response format: "distance: 0.340000"
            for line in response.split('\n'):
                if 'distance:' in line:
                    dist = float(line.split(':')[1].strip())
                    return dist
        except:
            pass
        return None

    def get_sensor(self):
        """Get accelerometer data"""
        return self.send("AT+GETSENSOR")

    def restart(self):
        """Restart the device"""
        print("Restarting device...")
        self.send_with_reboot("AT+RESTART")

    def close(self):
        """Close connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


def interactive():
    """Interactive command-line interface"""
    print("BU03 UWB Module - Interactive Mode")
    print("="*50)

    with BU03Device() as bu03:
        # Show initial info
        print("\n" + bu03.get_version())
        print("\n" + bu03.get_config())

        print("\nCommands:")
        print("  cfg              - Show current configuration")
        print("  set <id> <role> <ch> <rate> - Set config (auto-saves)")
        print("  ver              - Show version")
        print("  dist             - Get distance")
        print("  sensor           - Get sensor data")
        print("  restart          - Restart device")
        print("  <any AT cmd>     - Send raw AT command")
        print("  quit             - Exit")

        while True:
            try:
                cmd = input("\n> ").strip()

                if not cmd:
                    continue

                if cmd.lower() in ['quit', 'exit', 'q']:
                    break

                elif cmd == 'cfg':
                    bu03.get_config()

                elif cmd.startswith('set '):
                    parts = cmd.split()
                    if len(parts) == 5:
                        _, dev_id, role, ch, rate = parts
                        bu03.set_config(int(dev_id), int(role), int(ch), int(rate))
                    else:
                        print("Usage: set <device_id> <role> <channel> <rate>")
                        print("  device_id: 0-10")
                        print("  role: 0=tag, 1=base")
                        print("  channel: 0=ch9, 1=ch5")
                        print("  rate: 0=850K, 1=6.8M")

                elif cmd == 'ver':
                    print(bu03.get_version())

                elif cmd == 'dist':
                    dist = bu03.get_distance()
                    if dist is not None:
                        print(f"Distance: {dist:.3f} m")
                    else:
                        print("Could not read distance")

                elif cmd == 'sensor':
                    print(bu03.get_sensor())

                elif cmd == 'restart':
                    bu03.restart()

                else:
                    # Raw AT command
                    response = bu03.send(cmd if cmd.startswith('AT') else f'AT+{cmd}')
                    print(response)

            except KeyboardInterrupt:
                print("\nUse 'quit' to exit")
            except Exception as e:
                print(f"Error: {e}")

    print("\nGoodbye!")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        # Command-line usage
        cmd = sys.argv[1].lower()

        with BU03Device() as bu03:
            if cmd == 'info':
                print(bu03.get_version())
                print(bu03.get_config())

            elif cmd == 'config':
                bu03.get_config()

            elif cmd == 'set' and len(sys.argv) == 6:
                _, _, dev_id, role, ch, rate = sys.argv
                bu03.set_config(int(dev_id), int(role), int(ch), int(rate))

            elif cmd == 'distance':
                dist = bu03.get_distance()
                if dist:
                    print(f"{dist:.3f}")

            else:
                print("Usage:")
                print("  python3 bu03_util.py             # Interactive mode")
                print("  python3 bu03_util.py info        # Show version and config")
                print("  python3 bu03_util.py config      # Show config")
                print("  python3 bu03_util.py set <id> <role> <ch> <rate>")
                print("  python3 bu03_util.py distance    # Get distance")
    else:
        # Interactive mode
        interactive()
