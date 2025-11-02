# BU03 UWB Development Board

Python utility for configuring and working with BU03/BU04 UWB (Ultra-Wideband) positioning modules.

## Quick Start

```bash
# Interactive mode
python3 bu03_util.py

# Command-line
python3 bu03_util.py info                    # Show version and config
python3 bu03_util.py set <id> <role> <ch> <rate>  # Configure device
```

## Connection

- **Port:** `/dev/ttyUSB0`
- **Baud:** 115200
- **Format:** 8N1

## Device Configuration

Configure at least one device as a **tag** (moving point) and at least one as a **base station** (anchor).

**Important:** Tag and first base station must have the same ID. Additional base stations must have different IDs.

### Configuration Parameters

| Parameter | Values |
|-----------|--------|
| **ID** | 0-10 |
| **Role** | 0 = Tag (moving), 1 = Base Station (anchor) |
| **Channel** | 0 = Channel 9, 1 = Channel 5 |
| **Rate** | 0 = 850K, 1 = 6.8M |

### Example: 1 Tag + 3 Base Stations

| Device | ID | Role | Channel | Rate | Description |
|--------|----|----|---------|------|-------------|
| Tag | 0 | 0 (Tag) | 1 (Ch 5) | 1 (6.8M) | Moving device to track |
| Base 1 | 0 | 1 (Base) | 1 (Ch 5) | 1 (6.8M) | Primary anchor (same ID as tag) |
| Base 2 | 1 | 1 (Base) | 1 (Ch 5) | 1 (6.8M) | 2nd anchor |
| Base 3 | 2 | 1 (Base) | 1 (Ch 5) | 1 (6.8M) | 3rd anchor |

**Configuration commands:**
```bash
# Tag (ID:0)
python3 bu03_util.py set 0 0 1 1

# Base Station 1 (ID:0, same as tag)
python3 bu03_util.py set 0 1 1 1

# Base Station 2 (ID:1)
python3 bu03_util.py set 1 1 1 1

# Base Station 3 (ID:2)
python3 bu03_util.py set 2 1 1 1
```

## Interactive Commands

```
cfg                 # Show current configuration
set <id> <role> <ch> <rate>  # Set and save configuration
ver                 # Show firmware version
dist                # Get distance measurement
sensor              # Get accelerometer data
restart             # Restart device
quit                # Exit
```

## Notes

- **AT+SAVE triggers automatic reboot** (3 second wait)
- "IIC Error 3" on boot is normal (I2C device not connected)
- Chinese characters in boot messages are system info (expected)
- Configuration persists after reboot

## Reference

See `references/` for official AT command documentation.

## IMU/Accelerometer Data Limitations

**Note:** IMU (accelerometer) data can **only** be read from the **"TTL"** USB port using AT commands (e.g., `AT+GETSENSOR`). It is **NOT available** on the **"USB"** port where distance measurements are streamed.

- **USB Port** (shows as `/dev/ttyACM0`): Streams UWB distance data in binary `CmdM:4[` packets (~20 Hz), but does **not** respond to AT commands
- **TTL Port** (shows as `/dev/ttyUSB0` or `/dev/ttyUSB1`): Responds to AT commands including `AT+GETSENSOR` for IMU data (~26 Hz polling rate), but does **not** stream UWB distance data

We attempted to use both USB cables simultaneously to read both data streams, but the device does not support this mode - only one port can be active at a time.

**Potential Future Improvement:** Someone with firmware modification capabilities could potentially modify the device firmware to output both acceleration data and distance measurements on the USB port simultaneously.
