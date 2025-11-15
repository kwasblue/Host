import time
from typing import Any

from event_bus import EventBus
from transport import SerialTransport
import protocol


class RobotClient:
    def __init__(self, transport: Any) -> None:
        self.bus = EventBus()
        self.transport = transport
        self.transport.set_frame_handler(self._on_frame)

    def start(self) -> None:
        print("[RobotClient] Starting transport...")
        self.transport.start()

    def stop(self) -> None:
        print("[RobotClient] Stopping transport...")
        self.transport.stop()


    # === incoming from MCU ===

    def _on_frame(self, body: bytes) -> None:
        if not body:
            return
        msg_type = body[0]
        payload = body[1:]

        now = time.time()

        if msg_type == protocol.MSG_HEARTBEAT:
            self.bus.publish("heartbeat", {"ts": now, "raw": payload})
        elif msg_type == protocol.MSG_PING:
            self.bus.publish("ping", {"ts": now, "raw": payload})
        elif msg_type == protocol.MSG_PONG:
            self.bus.publish("pong", {"ts": now, "raw": payload})
        else:
            self.bus.publish("unknown", {"ts": now, "msg_type": msg_type, "raw": payload})

    # === outgoing commands ===

    def send_ping(self) -> None:
        print("[RobotClient] Sending PING")
        self.transport.send_frame(protocol.MSG_PING)

    def send_pong(self) -> None:
        print("[RobotClient] Sending PONG")
        self.transport.send_frame(protocol.MSG_PONG)
