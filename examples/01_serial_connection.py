#!/usr/bin/env python3
"""
Example 01: Serial Connection

Demonstrates:
- Finding available serial ports
- Connecting to ESP32 via USB serial
- Performing version handshake
- Basic lifecycle (start/stop)

Prerequisites:
- ESP32 with robot_host firmware connected via USB
- pyserial installed: pip install pyserial

Usage:
    python 01_serial_connection.py
    python 01_serial_connection.py /dev/ttyUSB0  # specify port
"""
import asyncio
import sys
from pathlib import Path

# Add parent to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from robot_host.transport.serial_transport import SerialTransport
from robot_host.command.client import AsyncRobotClient


def find_serial_ports():
    """List available serial ports."""
    import serial.tools.list_ports

    ports = serial.tools.list_ports.comports()
    usb_ports = []

    print("Available serial ports:")
    for port in ports:
        # Filter for likely ESP32 ports
        is_esp32 = any(x in port.description.lower() for x in ["usb", "uart", "serial", "cp210", "ch340", "ftdi"])
        marker = " <-- likely ESP32" if is_esp32 else ""
        print(f"  {port.device}: {port.description}{marker}")
        if is_esp32:
            usb_ports.append(port.device)

    return usb_ports


async def main():
    # Find or use specified port
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        ports = find_serial_ports()
        if not ports:
            print("\nNo USB serial ports found. Connect ESP32 and try again.")
            return
        port = ports[0]
        print(f"\nUsing first detected port: {port}")

    print(f"\n{'='*50}")
    print(f"Connecting to ESP32 on {port}")
    print(f"{'='*50}")

    # Create transport and client
    transport = SerialTransport(port=port, baudrate=115200)
    client = AsyncRobotClient(
        transport=transport,
        require_version_match=True,  # Verify protocol compatibility
        handshake_timeout_s=5.0,     # Wait up to 5s for handshake
    )

    try:
        # Start client (opens serial, performs handshake)
        print("\nStarting client...")
        await client.start()

        # Display connection info
        print(f"\nConnection established!")
        print(f"  Robot name: {client.robot_name}")
        print(f"  Board: {client.board}")
        print(f"  Firmware version: {client.firmware_version}")
        print(f"  Protocol version: {client.protocol_version}")
        print(f"  Connected: {client.is_connected}")

        # Keep running for a few seconds to see heartbeats
        print("\nMonitoring connection for 5 seconds...")
        for i in range(5):
            await asyncio.sleep(1)
            print(f"  [{i+1}s] Connected: {client.is_connected}, "
                  f"Time since msg: {client.connection.time_since_last_message:.2f}s")

        print("\nConnection test successful!")

    except Exception as e:
        print(f"\nError: {e}")

    finally:
        # Clean shutdown
        print("\nStopping client...")
        await client.stop()
        print("Done.")


if __name__ == "__main__":
    asyncio.run(main())
