import asyncio
import json
from dataclasses import dataclass
from typing import Callable, Optional, Any

from robot_host.core import protocol


def _decode_frames_from_bytes(blob: bytes) -> list[bytes]:
    """
    Uses host protocol.extract_frames to decode one or more frames from bytes.
    Returns list of bodies: body[0]=msg_type, body[1:]=payload
    """
    buf = bytearray(blob)
    bodies: list[bytes] = []
    protocol.extract_frames(buf, lambda body: bodies.append(body))
    return bodies


@dataclass
class PublishedEvent:
    topic: str
    data: Any


class CapturingBus:
    """
    Wraps your EventBus-like interface: publish(topic, data).
    Useful for asserting what got published.
    """
    def __init__(self) -> None:
        self.events: list[PublishedEvent] = []
        self.subscribers: dict[str, list[Callable[[Any], None]]] = {}

    def subscribe(self, topic: str, handler: Callable[[Any], None]) -> None:
        self.subscribers.setdefault(topic, []).append(handler)

    def publish(self, topic: str, data: Any) -> None:
        self.events.append(PublishedEvent(topic, data))
        for h in self.subscribers.get(topic, []):
            h(data)

    def topics(self) -> list[str]:
        return [e.topic for e in self.events]

    def last(self, topic: str) -> Optional[PublishedEvent]:
        for e in reversed(self.events):
            if e.topic == topic:
                return e
        return None


class FakeAsyncTransport:
    """
    Minimal transport that matches HasSendBytes in your BaseAsyncRobotClient.

    - send_bytes collects outgoing frames
    - can auto-respond with ACKs to reliable commands
    - can inject inbound frames to the client via set_frame_handler callback
    """
    def __init__(self, *, auto_ack: bool = True) -> None:
        self._on_frame: Optional[Callable[[bytes], None]] = None
        self.sent: list[bytes] = []
        self.started = False
        self.stopped = False
        self.auto_ack = auto_ack

    def set_frame_handler(self, handler: Callable[[bytes], None]) -> None:
        self._on_frame = handler

    def start(self) -> object:
        self.started = True
        return None

    def stop(self) -> object:
        self.stopped = True
        return None

    async def send_bytes(self, data: bytes) -> None:
        self.sent.append(data)

        if not self.auto_ack:
            return

        # Decode outgoing frames and respond to CMD_JSON with an ACK
        bodies = _decode_frames_from_bytes(data)
        for body in bodies:
            msg_type = body[0]
            payload = body[1:]

            if msg_type != protocol.MSG_CMD_JSON:
                continue

            try:
                obj = json.loads(payload.decode("utf-8"))
            except Exception:
                continue

            # Heartbeats are "fire and forget" in your client, so no ack required,
            # but acking doesn't hurt. We'll skip by default.
            if obj.get("type") == "CMD_HEARTBEAT":
                continue

            seq = obj.get("seq", 0)
            cmd_type = obj.get("type", "")

            # Generic ack naming: CMD_SET_VEL -> SET_VEL_ACK
            ack_cmd = None
            if isinstance(cmd_type, str) and cmd_type.startswith("CMD_"):
                ack_cmd = cmd_type.replace("CMD_", "", 1) + "_ACK"

            if not ack_cmd:
                continue

            resp = {"src": "mcu", "cmd": ack_cmd, "ok": True, "seq": seq}
            await self._inject_json_from_mcu(resp)

    async def _inject_json_from_mcu(self, obj: dict) -> None:
        if not self._on_frame:
            return
        payload = json.dumps(obj).encode("utf-8")
        frame = protocol.encode(protocol.MSG_CMD_JSON, payload)
        # Deliver like a decoded frame body: [msg_type][payload...]
        bodies = _decode_frames_from_bytes(frame)
        for b in bodies:
            self._on_frame(b)

    async def inject_pong(self) -> None:
        if not self._on_frame:
            return
        frame = protocol.encode(protocol.MSG_PONG, b"")
        bodies = _decode_frames_from_bytes(frame)
        for b in bodies:
            self._on_frame(b)

    async def inject_heartbeat(self) -> None:
        if not self._on_frame:
            return
        frame = protocol.encode(protocol.MSG_HEARTBEAT, b"")
        bodies = _decode_frames_from_bytes(frame)
        for b in bodies:
            self._on_frame(b)

    async def inject_raw(self, msg_type: int, payload: bytes = b"") -> None:
        if not self._on_frame:
            return
        frame = protocol.encode(msg_type, payload)
        bodies = _decode_frames_from_bytes(frame)
        for b in bodies:
            self._on_frame(b)
