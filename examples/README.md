# Robot Host Examples

Comprehensive examples demonstrating the full robot_host system for controlling ESP32-based robots.

## Prerequisites

1. **Hardware**: ESP32 with robot_host firmware flashed
2. **Connection**: USB cable (serial) or WiFi (TCP)
3. **Python packages**: Install with `pip install pyserial`

## Quick Start

```bash
cd robot_host/examples

# Find your serial port
python 01_serial_connection.py

# Connect to a specific port
python 01_serial_connection.py /dev/ttyUSB0

# Or connect via TCP/WiFi
python 02_tcp_connection.py 192.168.1.100
```

## Examples Overview

| Example | Description | Requires Hardware |
|---------|-------------|-------------------|
| 01 | Serial connection and handshake | ESP32 via USB |
| 02 | TCP/WiFi connection | ESP32 with WiFi |
| 03 | Command basics (ARM, ACKs) | ESP32 |
| 04 | Telemetry streaming | ESP32 with sensors |
| 05 | GPIO control (LEDs, buttons) | ESP32 + LED |
| 06 | Motor control and motion | ESP32 + motors |
| 07 | Encoder feedback | ESP32 + encoders |
| 08 | Session recording | ESP32 |
| 09 | Full robot control | Complete robot |

## Examples in Detail

### 01: Serial Connection
Basic USB/serial connection to ESP32:
- Automatic port detection
- Version handshake
- Connection monitoring

```bash
python 01_serial_connection.py
python 01_serial_connection.py /dev/ttyUSB0
python 01_serial_connection.py COM3  # Windows
```

### 02: TCP Connection
WiFi/network connection:
- TCP transport with auto-reconnect
- Works over local network

```bash
python 02_tcp_connection.py 192.168.1.100
python 02_tcp_connection.py 192.168.1.100 8080  # custom port
```

### 03: Command Basics
Sending commands and handling responses:
- Fire-and-forget commands
- Reliable commands with ACK
- Robot state machine (ARM/ACTIVATE/DISARM)
- Error handling

```bash
python 03_command_basics.py /dev/ttyUSB0
python 03_command_basics.py tcp:192.168.1.100
```

### 04: Telemetry Stream
Receiving sensor data:
- IMU (accelerometer, gyroscope)
- Encoders
- Motor data
- Ultrasonic sensors

```bash
python 04_telemetry_stream.py /dev/ttyUSB0
```

### 05: GPIO Control
Controlling GPIO pins:
- Write (LEDs, relays)
- Read (buttons, switches)
- Toggle
- Pattern generation (SOS)

```bash
python 05_gpio_control.py /dev/ttyUSB0
```

### 06: Motor Control
**WARNING: Robot will move!**

Motor and motion control:
- ARM/ACTIVATE sequence
- Velocity commands
- Velocity ramping
- Emergency stop (ESTOP)
- Arc motion

```bash
python 06_motor_control.py /dev/ttyUSB0
```

### 07: Encoder Feedback
Reading quadrature encoders:
- Attach encoders to pins
- Read tick counts
- Compute velocity
- Real-time display

```bash
python 07_encoder_feedback.py /dev/ttyUSB0
```

### 08: Session Recording
Recording and replaying sessions:
- Record all events to JSONL
- Analyze recorded sessions
- Replay at different speeds

```bash
python 08_session_recording.py /dev/ttyUSB0
# Creates recordings in ./recordings/
```

### 09: Full Robot Control
**WARNING: Robot will move!**

Complete control application:
- Connection management
- State machine
- Real-time telemetry
- Safety monitoring
- Demo trajectories (square, figure-8)
- Session recording

```bash
python 09_full_robot_control.py /dev/ttyUSB0
```

## Connection Argument Format

All examples accept a connection argument:

```bash
# Serial (USB)
python example.py /dev/ttyUSB0        # Linux
python example.py /dev/tty.usbserial-XXXX  # macOS
python example.py COM3                 # Windows

# TCP (WiFi)
python example.py tcp:192.168.1.100   # Default port 8080
python example.py tcp:192.168.1.100:9000  # Custom port
```

## Output Files

Some examples create output files:

- `recordings/` - Session recordings (example 08)
- `control_logs/` - Control session logs (example 09)
- `*.png` - Generated plots

## Troubleshooting

### Serial Connection Issues

1. **Port not found**:
   - Check USB cable is connected
   - Run `ls /dev/tty*` (Linux/macOS) or check Device Manager (Windows)

2. **Permission denied** (Linux):
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

3. **Port busy**:
   - Close other programs using the port
   - Unplug and replug USB cable

### TCP Connection Issues

1. **Connection refused**:
   - Verify ESP32 IP address
   - Check ESP32 serial output for IP
   - Ensure both on same network

2. **Timeout**:
   - Check firewall settings
   - Try pinging the ESP32 IP

### Handshake Failures

1. **Protocol version mismatch**:
   - Update firmware or host to match versions

2. **Timeout**:
   - Reset ESP32
   - Check baud rate (115200)

## Safety Notes

- Examples 06 and 09 move motors - **ensure safe environment**
- Always have an emergency stop ready (unplug power)
- Start with low velocities when testing
- The ESTOP command immediately stops all motion

## See Also

- `robot_host/research/examples/` - Research and analysis examples
- `robot_host/runners/` - Additional utility scripts
