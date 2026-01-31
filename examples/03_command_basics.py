#!/usr/bin/env python3
"""
Example 03: Command Basics

Demonstrates:
- Sending commands to the ESP32
- Reliable command delivery with ACKs
- Fire-and-forget commands
- Error handling
- Robot state machine (ARM/ACTIVATE/DISARM)

Prerequisites:
- ESP32 connected via USB or TCP

Usage:
    python 03_command_basics.py /dev/ttyUSB0
    python 03_command_basics.py tcp:192.168.1.100
"""
import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from robot_host.transport.serial_transport import SerialTransport
from robot_host.transport.tcp_transport import AsyncTcpTransport
from robot_host.command.client import AsyncRobotClient


def create_transport(arg: str):
    """Create transport from command line argument."""
    if arg.startswith("tcp:"):
        host = arg[4:]
        port = 8080
        if ":" in host:
            host, port_str = host.rsplit(":", 1)
            port = int(port_str)
        return AsyncTcpTransport(host=host, port=port)
    else:
        return SerialTransport(port=arg, baudrate=115200)


async def main():
    if len(sys.argv) < 2:
        print("Usage: python 03_command_basics.py <port_or_tcp>")
        print("Examples:")
        print("  python 03_command_basics.py /dev/ttyUSB0")
        print("  python 03_command_basics.py tcp:192.168.1.100")
        return

    transport = create_transport(sys.argv[1])
    client = AsyncRobotClient(transport=transport)

    print("="*50)
    print("Command Basics Example")
    print("="*50)

    try:
        await client.start()
        print(f"Connected to {client.robot_name}\n")

        # -------------------------------------------------------
        # 1. Fire-and-forget command (no ACK waiting)
        # -------------------------------------------------------
        print("1. Fire-and-forget command (CMD_PING)")
        await client.send_json_cmd("CMD_PING", {})
        print("   Sent CMD_PING (no ACK waiting)\n")
        await asyncio.sleep(0.2)

        # -------------------------------------------------------
        # 2. Reliable command with ACK
        # -------------------------------------------------------
        print("2. Reliable command with ACK (CMD_WHOAMI)")
        success, error = await client.send_reliable("CMD_WHOAMI", {})
        if success:
            print("   CMD_WHOAMI acknowledged successfully")
        else:
            print(f"   CMD_WHOAMI failed: {error}")
        print()

        # -------------------------------------------------------
        # 3. Robot state machine commands
        # -------------------------------------------------------
        print("3. Robot state machine")

        # Start in safe state
        print("   Ensuring safe baseline...")
        await client.send_reliable("CMD_DISARM", {})
        await asyncio.sleep(0.1)

        # ARM the robot
        print("   Arming robot...")
        success, error = await client.arm()
        if success:
            print("   Robot ARMED")
        else:
            print(f"   ARM failed: {error}")

        # ACTIVATE the robot
        print("   Activating robot...")
        success, error = await client.activate()
        if success:
            print("   Robot ACTIVATED - motors enabled")
        else:
            print(f"   ACTIVATE failed: {error}")

        await asyncio.sleep(0.5)

        # DEACTIVATE
        print("   Deactivating robot...")
        success, error = await client.deactivate()
        if success:
            print("   Robot DEACTIVATED")
        else:
            print(f"   DEACTIVATE failed: {error}")

        # DISARM
        print("   Disarming robot...")
        success, error = await client.disarm()
        if success:
            print("   Robot DISARMED")
        else:
            print(f"   DISARM failed: {error}")
        print()

        # -------------------------------------------------------
        # 4. Command with payload
        # -------------------------------------------------------
        print("4. Command with payload (CMD_SET_VEL)")
        success, error = await client.set_vel(vx=0.0, omega=0.0)
        if success:
            print("   SET_VEL(0, 0) acknowledged")
        else:
            print(f"   SET_VEL failed: {error}")
        print()

        # -------------------------------------------------------
        # 5. Error handling - command that should fail
        # -------------------------------------------------------
        print("5. Error handling - invalid state transition")
        # Try to activate without arming (should fail)
        success, error = await client.activate()
        if success:
            print("   Unexpected success!")
            await client.deactivate()
        else:
            print(f"   Expected failure: {error}")
        print()

        # -------------------------------------------------------
        # 6. Batch commands
        # -------------------------------------------------------
        print("6. Batch commands (rapid fire)")
        for i in range(5):
            # Use fire-and-forget for high-rate streaming
            await client.send_stream("CMD_PING", {}, request_ack=False)
        print("   Sent 5 PINGs (fire-and-forget)")
        await asyncio.sleep(0.3)
        print()

        # -------------------------------------------------------
        # 7. Command statistics
        # -------------------------------------------------------
        print("7. Command statistics")
        stats = client.get_stats()
        print(f"   Connected: {stats['connected']}")
        print(f"   Commands sent: {stats.get('total_sent', 'N/A')}")
        print(f"   Commands acked: {stats.get('total_acked', 'N/A')}")
        print(f"   Retries: {stats.get('total_retries', 'N/A')}")
        print()

        print("All tests complete!")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        await client.stop()


if __name__ == "__main__":
    asyncio.run(main())
