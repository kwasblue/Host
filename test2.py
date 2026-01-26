# debug_state.py
import asyncio
from robot_host.transports.serial_transport import SerialTransport
from robot_host.core.client import BaseAsyncRobotClient

async def test():
    transport = SerialTransport(port="/dev/tty.usbserial-0001", baudrate=115200)
    
    client = BaseAsyncRobotClient(
        transport=transport,
        heartbeat_interval_s=0.2,
        connection_timeout_s=5.0,
        require_version_match=True,
        handshake_timeout_s=5.0,
    )
    
    # Subscribe to state changes
    client.bus.subscribe("state.changed", lambda d: print(f"STATE: {d}"))
    client.bus.subscribe("cmd.ARM_ACK", lambda d: print(f"ARM_ACK: {d}"))
    client.bus.subscribe("cmd.HEARTBEAT_ACK", lambda d: print(f"HEARTBEAT_ACK: {d}"))
    
    await client.start()
    print(f"Version verified: {client.version_verified}")
    
    # Wait and watch heartbeat ACKs
    print("Waiting for heartbeats to establish IDLE state...")
    await asyncio.sleep(2.0)
    
    # Try arm
    print("Attempting arm...")
    ok, err = await client.arm()
    print(f"arm: ok={ok}, err={err}")
    
    await client.stop()

asyncio.run(test())