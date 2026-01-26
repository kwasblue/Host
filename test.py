# quick_version_test.py
import asyncio
from robot_host.transports.serial_transport import SerialTransport
from robot_host.core import protocol

async def test():
    transport = SerialTransport(port="/dev/tty.usbserial-0001", baudrate=115200)
    
    received = []
    def on_frame(body):
        print(f"Received: msg_type={body[0]:02x}, payload={body[1:]}")
        received.append(body)
    
    transport.set_frame_handler(on_frame)
    transport.start()
    
    await asyncio.sleep(0.5)  # Let it connect
    
    # Send VERSION_REQUEST
    frame = protocol.encode(protocol.MSG_VERSION_REQUEST, b"")
    transport._send_bytes(frame)
    
    await asyncio.sleep(1.0)  # Wait for response
    
    transport.stop()
    
    if received:
        print("Got response!")
    else:
        print("No response - firmware needs update")

asyncio.run(test())