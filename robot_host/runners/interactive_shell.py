# robot_host/runners/interactive_shell.py

import asyncio
from typing import Callable

from robot_host.core.client import AsyncRobotClient
from robot_host.transports.tcp_transport import AsyncTcpTransport


async def ainput(prompt: str = "") -> str:
    """
    Async-friendly input() so Ctrl-C is handled correctly.
    """
    loop = asyncio.get_running_loop()
    try:
        return await loop.run_in_executor(None, lambda: input(prompt))
    except KeyboardInterrupt:
        # Propagate so main loop can catch it
        raise


def _print_event(tag: str) -> Callable[[dict], None]:
    def _handler(data: dict) -> None:
        print(f"[{tag}] {data}")
    return _handler


async def main() -> None:
    host_sta = "10.0.0.107"
    port = 3333

    transport = AsyncTcpTransport(host_sta, port)
    client = AsyncRobotClient(transport)

    # Subscribe to events
    client.bus.subscribe("heartbeat", _print_event("HEARTBEAT"))
    client.bus.subscribe("pong",      _print_event("PONG"))
    client.bus.subscribe("hello",     _print_event("HELLO"))
    client.bus.subscribe("json",      _print_event("JSON"))
    client.bus.subscribe("raw_frame", _print_event("RAW"))

    await client.start()

    print("")
    print("=== Robot Interactive Shell ===")
    print("Available commands:")
    print("  ping              - send ping")
    print("  whoami            - ask robot identity")
    print("  led on            - turn LED on")
    print("  led off           - turn LED off")
    print("  q / quit / exit   - quit")
    print("  servo attach       - attach servo on channel 0")
    print("  servo angle <deg>  - move servo 0 to angle")

    print("")

    try:
        while True:
            try:
                line = (await ainput("robot> ")).strip()
            except KeyboardInterrupt:
                print("\n[Shell] KeyboardInterrupt — exiting...")
                break

            if not line:
                continue

            lower = line.lower()

            if lower in ("quit", "exit", "q"):
                print("[Shell] Exiting...")
                break

            elif lower == "ping":
                await client.send_ping()

            elif lower == "whoami":
                await client.send_whoami()

            elif lower == "led on":
                await client.send_led_on()

            elif lower == "led off":
                await client.send_led_off()
            
            # --- NEW SERVO COMMANDS ---
            elif lower == "servo attach":
                await client.send_servo_attach(servo_id=0)

            elif lower.startswith("servo angle"):
                parts = lower.split()
                if len(parts) != 3:
                    print("Usage: servo angle <deg>")
                else:
                    try:
                        angle = float(parts[2])
                    except ValueError:
                        print("Angle must be a number")
                    else:
                        await client.send_servo_angle(servo_id=0, angle_deg=angle)
            
            elif lower.startswith("servo sweep"):
                # e.g. "servo sweep 0 180"
                parts = lower.split()
                if len(parts) != 4:
                    print("Usage: servo sweep <start_deg> <end_deg>")
                else:
                    try:
                        start = float(parts[2])
                        end = float(parts[3])
                    except ValueError:
                        print("Angles must be numbers")
                    else:
                        await client.smooth_servo_move(servo_id=0, start_deg=start, end_deg=end)
            elif lower.startswith("mode "):
                # e.g. "mode active"
                _, mode_str = lower.split(maxsplit=1)
                mode = mode_str.upper()  # IDLE/ARMED/ACTIVE/CALIB
                await client.cmd_set_mode(mode=mode)
                print(f"[Shell] Requested mode: {mode}")



            else:
                print(f"[Shell] Unknown command: {line}")
                print("  Commands: ping | whoami | led on | led off | quit")

    finally:
        # Ensure transport shuts down cleanly
        print("[Shell] Shutting down client...")
        await client.stop()
        print("[Shell] Done.")


if __name__ == "__main__":
    # Protect asyncio.run() from showing nasty tracebacks on Ctrl-C
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("[Shell] Force-quit.")
