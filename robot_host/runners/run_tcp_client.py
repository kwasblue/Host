# robot_host/runners/run_tcp_client_async.py

import asyncio

from robot_host.core.client import AsyncRobotClient
from robot_host.transports.tcp_transport import AsyncTcpTransport


async def main() -> None:
    # AP mode IP (RobotAP): 192.168.4.1
    # STA mode IP (home WiFi): whatever the ESP32 prints, e.g. 10.0.0.107
    host_ap  = "192.168.4.1"
    host_sta = "10.0.0.107"
    port = 3333

    transport = AsyncTcpTransport(host_sta, port)
    client = AsyncRobotClient(transport)

    # Subscribe to events
    client.bus.subscribe("heartbeat", lambda d: print("[TCP] HEARTBEAT", d))
    client.bus.subscribe("pong",      lambda d: print("[TCP] PONG", d))
    client.bus.subscribe("hello",     lambda info: print("[Bus] HELLO:", info))
    client.bus.subscribe("json",      lambda obj: print("[Bus] JSON:", obj))

    await client.start()

    try:
        last_ping = 0.0
        loop = asyncio.get_running_loop()
        while True:
            now = loop.time()
            if now - last_ping >= 5.0:
                await client.send_ping()
                await client.send_led_on()
                await client.send_whoami()
                await asyncio.sleep(2.0)
                await client.send_led_off()
                last_ping = now
            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[TCP] Stopping...")
    finally:
        await client.stop()


if __name__ == "__main__":
    asyncio.run(main())
