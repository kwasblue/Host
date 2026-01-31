#!/usr/bin/env python3
"""
Example 02: TCP Connection

Demonstrates:
- Connecting to ESP32 via WiFi/TCP
- Auto-reconnection on disconnect
- Version handshake over network

Prerequisites:
- ESP32 with robot_host firmware and WiFi enabled
- ESP32 and host on same network
- Know the ESP32's IP address (check serial output or router)

Usage:
    python 02_tcp_connection.py 192.168.1.100
    python 02_tcp_connection.py 192.168.1.100 8080  # custom port
"""
import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from robot_host.transport.tcp_transport import AsyncTcpTransport
from robot_host.command.client import AsyncRobotClient


async def main():
    # Parse arguments
    if len(sys.argv) < 2:
        print("Usage: python 02_tcp_connection.py <ip_address> [port]")
        print("Example: python 02_tcp_connection.py 192.168.1.100")
        return

    host = sys.argv[1]
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 8080

    print(f"{'='*50}")
    print(f"TCP Connection Example")
    print(f"{'='*50}")
    print(f"Connecting to {host}:{port}")

    # Create TCP transport with auto-reconnect
    transport = AsyncTcpTransport(
        host=host,
        port=port,
        reconnect_delay=5.0,  # Wait 5s between reconnect attempts
    )

    # Create client
    client = AsyncRobotClient(
        transport=transport,
        require_version_match=True,
        handshake_timeout_s=10.0,  # Longer timeout for network
    )

    # Track telemetry
    telemetry_count = 0

    def on_telemetry(data):
        nonlocal telemetry_count
        telemetry_count += 1

    client.bus.subscribe("telemetry", on_telemetry)

    try:
        print("\nStarting client...")
        await client.start()

        print(f"\nConnected to {client.robot_name}!")
        print(f"  Firmware: {client.firmware_version}")
        print(f"  Protocol: {client.protocol_version}")

        # Monitor for 10 seconds
        print("\nMonitoring for 10 seconds...")
        print("(Unplug network to test reconnect behavior)\n")

        for i in range(10):
            await asyncio.sleep(1)
            status = "Connected" if client.is_connected else "DISCONNECTED"
            print(f"  [{i+1:2d}s] Status: {status}, "
                  f"Telemetry packets: {telemetry_count}")

        print("\nTest complete!")

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    except Exception as e:
        print(f"\nConnection error: {e}")
        print("\nTroubleshooting:")
        print("  1. Check ESP32 is powered and WiFi connected")
        print("  2. Verify IP address (check ESP32 serial output)")
        print("  3. Ensure both devices on same network")
        print("  4. Check firewall settings")

    finally:
        print("\nStopping client...")
        await client.stop()
        print("Done.")


if __name__ == "__main__":
    asyncio.run(main())
