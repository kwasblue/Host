

from client import RobotClient
from tcp_transport import TcpTransport

def main():
    # AP mode example: often 192.168.4.1
    host = "192.168.4.1"  # or whatever IP printed by the ESP32
    host2 = "10.0.0.107"
    port = 3333

    client = RobotClient(TcpTransport(host2, port))

    client.bus.subscribe("heartbeat", lambda d: print("[TCP] HEARTBEAT", d))
    client.bus.subscribe("pong",      lambda d: print("[TCP] PONG", d))
    client.bus.subscribe("hello", lambda info: print("[Bus] HELLO:", info))
    
    

    client.start()
    
    try:
        import time
        last_ping = 0.0
        while True:
            now = time.time()
            if now - last_ping >= 5.0:
                client.send_ping()
                client.send_led_on()
                client.send_whoami()
                time.sleep(2)
                client.send_led_off()
                last_ping = now
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[TCP] Stopping...")
    finally:
        client.stop()

if __name__ == "__main__":
    main()
