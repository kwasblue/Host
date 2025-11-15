import time
from client import RobotClient
from transport import SerialTransport


def main():
    # TODO: set this to your actual serial port
    # macOS:   "/dev/tty.usbserial-XXXX" or "/dev/tty.usbmodemXXXX"
    # Linux:   "/dev/ttyUSB0" or "/dev/ttyACM0"
    # Windows: "COM5" etc.
    port = "/dev/cu.usbserial-0001"
    serial_port = SerialTransport(port, baudrate=115200)
    client = RobotClient(serial_port)

    # subscribe to events
    client.bus.subscribe("heartbeat", lambda data: print(f"[Host] HEARTBEAT {data}"))
    client.bus.subscribe("pong", lambda data: print(f"[Host] PONG {data}"))

    client.start()

    try:
        last_ping = 0.0
        while True:
            now = time.time()
            # send PING every 5 seconds
            if now - last_ping >= 5.0:
                client.send_ping()
                last_ping = now
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[Host] Ctrl+C, stopping...")
    finally:
        client.stop()


if __name__ == "__main__":
    main()
