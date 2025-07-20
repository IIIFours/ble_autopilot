# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an ESP32 BLE intermediary that bridges communication between:
- A Teensy 4.0 microcontroller running a custom autopilot (which has no BLE capability)
- An iOS device running a custom app for sending PID controller values

The ESP32 acts as a transparent bridge, forwarding data between BLE and UART serial interfaces.

## Development Commands

### Build and Upload
```bash
# Build the project
pio run

# Build and upload to ESP32
pio run -t upload

# Monitor serial output
pio device monitor

# Build, upload and monitor in one command
pio run -t upload && pio device monitor
```

### Development Workflow
```bash
# Clean build files
pio run -t clean

# Update dependencies
pio pkg update
```

## Architecture

### Hardware Configuration
- **Board**: Adafruit QT Py ESP32-C3
- **Serial Communication**: 115200 baud rate for both USB debug and Teensy communication
- **BLE Service**: Custom service with two characteristics for telemetry and PID updates

### Data Flow Architecture
1. **Telemetry Flow (Teensy → iOS)**:
   - Teensy sends `AutopilotTelemetry` struct via serial (wrapped with 0x02/0x03 markers)
   - ESP32 validates and unwraps the data
   - ESP32 broadcasts data as BLE notifications
   
2. **Control Flow (iOS → Teensy)**:
   - iOS app writes PID values to BLE characteristic
   - ESP32 receives the write callback
   - ESP32 forwards data to Teensy via serial

### Key Data Structure
The `AutopilotTelemetry` struct (25 fields) is the central data format containing:
- PID parameters (kp, ki, kd)
- Navigation data (GPS coordinates, heading, bearing)
- Control outputs (rudder angle, motor position)
- State information (homing status)

### BLE Service Details
- **Service UUID**: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`
- **Telemetry Characteristic**: `beb5483e-36e1-4688-b7f5-ea07361b26a8` (notify)
- **PID Characteristic**: `98ab29d2-2b95-497d-9df7-f064e5ac05a5` (write)

## Common Tasks

### Adding New Telemetry Fields
1. Update the `AutopilotTelemetry` struct in src/main.cpp
2. Ensure the struct size remains consistent between ESP32 and Teensy
3. Update the iOS app to handle the new fields

### Debugging Serial Communication
- Monitor USB serial for debug output (115200 baud)
- Check for "Packet too large" errors if struct size mismatches
- Verify start (0x02) and end (0x03) markers in serial data

### Known Issue
There's a compilation error on line 144: The code references `bearingPositionToDestinationWaypoint` but the struct field is named `bearingToWaypoint`. This needs to be fixed for successful compilation.