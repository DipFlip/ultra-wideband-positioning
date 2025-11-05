# UWB Distance Calibration Guide

## Overview

This guide explains how to calibrate distance measurements for the BU03 UWB positioning system. Proper calibration can improve ranging accuracy to sub-centimeter precision.

## Calibration Theory

### Why Calibrate?

UWB distance measurements can have systematic errors due to:
- **Antenna delay**: Internal chip propagation delays (varies chip-to-chip)
- **PCB effects**: Circuit board and antenna design impacts
- **Environmental factors**: Temperature, humidity
- **Signal power effects**: Received signal strength affects timing

### Calibration Model

The system uses a linear correction model:

```
corrected_distance = scale_factor × measured_distance + offset
```

Where:
- **scale_factor**: Corrects proportional errors (typical: 0.95-1.05)
- **offset**: Corrects constant bias in mm (typical: ±50mm)

## Calibration Process

### Prerequisites

1. UWB system running with at least one active anchor
2. Access to the web dashboard (http://localhost:8080)
3. Known reference distances (use laser measure or tape measure)
4. Stable environment (avoid moving objects during measurements)

### Step-by-Step Procedure

#### 1. Select Anchor to Calibrate

In the dashboard's "Distance Calibration" panel:
- Choose the anchor from the dropdown (e.g., "Anchor 0")
- Verify the "Current Measured Distance" shows a green value (connected)

#### 2. Collect Measurement Points

For best results, collect measurements at **3-5 different distances** spanning your operating range:

**Example distances**: 0.5m, 1.0m, 2.0m, 4.0m, 8.0m

For each distance:

1. **Position the tag/anchor** at a precisely measured distance
   - Use a laser measure or tape measure
   - Keep line-of-sight clear
   - Wait 2-3 seconds for readings to stabilize

2. **Record the measurement**:
   - Enter true distance in the "True Distance" field (e.g., 1.0)
   - Click **"Add Measurement"**
   - Verify the measurement appears in the list with error (Δ)

3. **Repeat** at different distances

#### 3. Fit Calibration

Once you have 2+ measurements:

1. Click **"Fit Calibration"**
2. Review the results:
   - **Scale Factor**: Should be close to 1.0 (e.g., 0.98-1.02)
   - **Offset**: Typical range ±100mm
   - **R² (fit quality)**: Higher is better (>0.99 excellent, >0.95 good)

If R² is low (<0.90):
- Check for measurement errors
- Ensure stable positioning during measurements
- Add more measurement points
- Check for environmental interference

#### 4. Verify Calibration

After fitting:
- The calibration is immediately applied to live measurements
- Observe the corrected distances in the anchor display
- Verify accuracy at a new test distance

#### 5. Apply to Device (Optional)

To store calibration in device firmware:

1. **Connect TTL port** (`/dev/ttyUSB0`) for AT commands
2. Click **"Apply to Device (AT+SETDEV)"**
3. Confirm the action
4. Device will reboot with new calibration

**Note**: This requires the TTL serial port. If only USB port is connected, calibration is applied in software only.

## Best Practices

### Measurement Tips

1. **Coverage**: Measure across your full operating range
2. **Accuracy**: Use precise reference measurements (laser preferred)
3. **Stability**: Wait for readings to stabilize (~2 seconds)
4. **Environment**: Avoid obstacles, moving objects, and metal surfaces
5. **Minimum points**: 3-5 measurements recommended (2 minimum)

### Recommended Distances

For a typical indoor system (0-10m range):

| Distance | Purpose |
|----------|---------|
| 0.5m     | Near-field behavior |
| 1.0m     | Close range accuracy |
| 2.0m     | Mid-range anchor spacing |
| 4.0m     | Typical room diagonal |
| 8.0m     | Far-field accuracy |

### Interpreting Results

**Good Calibration:**
```
Scale Factor: 1.0125
Offset: -23.45 mm
R²: 0.9987
```
- Scale near 1.0 (within ±5%)
- Offset reasonable (±100mm)
- R² > 0.99

**Poor Calibration:**
```
Scale Factor: 0.8542
Offset: 234.56 mm
R²: 0.8234
```
- Scale far from 1.0 (>10% error)
- Large offset suggests systematic issue
- Low R² indicates measurement problems

## Troubleshooting

### "No distance measurement available"
- Verify anchor is connected (green indicator)
- Check if anchor is enabled in anchor_config.json
- Ensure tag is powered and within range

### Low R² after fitting
- **Re-measure**: Verify reference distances with laser measure
- **Stability**: Ensure no movement during measurements
- **Environment**: Check for multipath reflections (metal, walls)
- **Outliers**: Remove bad measurements and re-fit

### Calibration not applied
- Check browser console for errors (F12)
- Verify server logs for API errors
- Reload dashboard to refresh calibration data

### Device calibration fails
- Ensure TTL port (`/dev/ttyUSB0`) is connected
- Check if other program is using the port
- Try manual AT commands: `python3 bu03_util.py`

## Technical Details

### Calibration Storage

Calibration data is stored in `anchor_config.json`:

```json
"calibration": {
  "per_anchor": {
    "0": {
      "scale_factor": 1.0125,
      "offset_mm": -23.45,
      "measurements": [
        {
          "true_distance_m": 1.0,
          "measured_distance_m": 1.023,
          "timestamp": 1699564123.456
        }
      ]
    }
  },
  "device_parameters": {
    "antenna_delay": 16336,
    "correction_a": 1.0,
    "correction_b": 0.0
  }
}
```

### API Endpoints

- `POST /calibration/add_measurement` - Add measurement point
- `POST /calibration/fit` - Fit linear model
- `POST /calibration/clear` - Clear calibration
- `POST /calibration/apply_device` - Send to device via AT+SETDEV
- `GET /calibration/status` - Get current calibration

### AT Command Details

The `AT+SETDEV` command sets device parameters:

```
AT+SETDEV=5,16336,1,0.018,0.642,1.0000,0.00,0,0
```

Parameters:
- x1: Label refresh rate (5)
- x2: **Antenna delay** (16336)
- x3: Kalman filter enable (1)
- x4: Kalman Q (0.018)
- x5: Kalman R (0.642)
- x6: **Correction factor a** (1.0000)
- x7: **Correction offset b** (0.00)
- x8: Positioning enable (0)
- x9: Positioning dimension (0)

## References

### UWB Calibration Papers

1. "Data-Driven Antenna Delay Calibration for UWB Devices" (IEEE)
2. Qorvo APS011: DW1000 Antenna Delay Calibration
3. Makerfabs: ESP32 UWB Antenna Delay Calibrating

### Device Documentation

- BU03 AT Commands: `references/bu03_at_commands.pdf`
- Device interface: `bu03_util.py`
- Server implementation: `uwb_server.py`

## Quick Reference

| Action | Location |
|--------|----------|
| Open dashboard | http://localhost:8080 |
| Calibration panel | Bottom of page |
| View calibration data | anchor_config.json |
| Manual AT commands | `python3 bu03_util.py` |
| Server logs | Terminal running uwb_server.py |

---

**Note**: Calibration is per-anchor. Each anchor may need individual calibration due to hardware variations.
