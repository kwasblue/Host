# robot_host/core/client_async.py

from __future__ import annotations

import json
import inspect
from typing import Optional, Protocol, Callable

from .event_bus import EventBus
from . import protocol
from .messages import MsgType  # kept for external imports, even if unused
from robot_host.config.client_commands import RobotCommandsMixin


class HasSendBytes(Protocol):
    """Minimal transport interface used by the async robot client."""

    async def send_bytes(self, data: bytes) -> None: ...
    def set_frame_handler(self, handler: Callable[[bytes], None]) -> None: ...
    def start(self) -> object: ...
    def stop(self) -> object: ...


# Re-export / alias protocol constants for convenience
MSG_PING      = protocol.MSG_PING
MSG_PONG      = protocol.MSG_PONG
MSG_HEARTBEAT = protocol.MSG_HEARTBEAT
MSG_WHOAMI    = protocol.MSG_WHOAMI
MSG_CMD_JSON  = protocol.MSG_CMD_JSON


class BaseAsyncRobotClient:
    """
    Core async host-side client:

      - Owns the transport and framing (protocol.encode / decode).
      - Publishes decoded events onto an EventBus.
      - Provides generic helpers: send_ping(), send_whoami(), send_json_cmd().
      - Handles generic JSON/telemetry routing.

    This class is intentionally *robot-agnostic* 
    """

    def __init__(self, transport: HasSendBytes, bus: Optional[EventBus] = None) -> None:
        self.transport = transport
        self.bus = bus or EventBus()
        self._running = False
        self._seq = 0  # for JSON command sequencing

        # Transport will call _on_frame(body) where:
        #   body[0] = msg_type
        #   body[1:] = payload bytes
        self.transport.set_frame_handler(self._on_frame)

    # ---------- Lifecycle ----------

    async def start(self) -> None:
        """
        Start the transport.

        Compatible with both sync and async transport.start().
        """
        if self.transport is None:
            return

        start_fn = getattr(self.transport, "start", None)
        if start_fn is None:
            raise RuntimeError("Transport has no start() method")

        result = start_fn()
        if inspect.isawaitable(result):
            await result

        self._running = True
        print("[RobotClient] Started")

    async def stop(self) -> None:
        """
        Stop the client and underlying transport.

        Compatible with both sync and async transport.stop().
        """
        if self.transport is None:
            return

        stop_fn = getattr(self.transport, "stop", None)
        if stop_fn is None:
            return

        result = stop_fn()
        if inspect.isawaitable(result):
            await result

    # ---------- Incoming data path ----------

    def _on_frame(self, body: bytes) -> None:
        """
        Called by the transport whenever a complete framed message arrives.

        body[0] = msg_type
        body[1:] = payload bytes
        """
        if not body:
            return

        msg_type = body[0]
        payload = body[1:]

        if msg_type == MSG_PONG:
            self.bus.publish("pong", {})
        elif msg_type == MSG_HEARTBEAT:
            self.bus.publish("heartbeat", {})
        elif msg_type == MSG_CMD_JSON:
            self._handle_json_payload(payload)
        else:
            # Unknown / raw frame
            self.bus.publish(
                "raw_frame",
                {"msg_type": msg_type, "payload": payload},
            )

    def _handle_json_payload(self, payload: bytes) -> None:
        """
        Handle a JSON-encoded payload from the robot.
        """
        try:
            text = payload.decode("utf-8")
            obj = json.loads(text)
        except Exception as e:
            print(f"[RobotClient] Failed to decode JSON payload: {e!r}")
            self.bus.publish(
                "json_error",
                {"error": str(e), "raw": payload},
            )
            return

        type_str = obj.get("type", "")
        cmd_str  = obj.get("cmd", "")

        # --- HELLO handshake ---
        if type_str == "HELLO":
            self.bus.publish("hello", obj)
            return

        # --- Telemetry (generic) ---
        if type_str == "TELEMETRY":
            # Raw telemetry feed for host modules
            self.bus.publish("telemetry.raw", obj)

            # Existing generic event (if anything expects this)
            self.bus.publish("telemetry", obj)

            return

        # --- Command ACKs / other cmd-based messages ---
        if cmd_str:
            # e.g. cmd="ULTRASONIC_READ_ACK" -> "cmd.ULTRASONIC_READ_ACK"
            self.bus.publish(f"cmd.{cmd_str}", obj)
            return

        # Fallback: generic JSON
        self.bus.publish("json", obj)

    # ---------- Outgoing commands ----------

    def _next_seq(self) -> int:
        self._seq += 1
        return self._seq

    async def _send_frame(self, msg_type: int, payload: bytes = b"") -> None:
        """
        Helper: encode and send a framed message via the transport.
        Uses async transport.send_bytes(...) so it works with TCP and Serial.
        """
        frame = protocol.encode(msg_type, payload)
        await self.transport.send_bytes(frame)

    async def send_ping(self) -> None:
        """Send a simple PING frame (no payload)."""
        await self._send_frame(MSG_PING, b"")

    async def send_whoami(self) -> None:
        """Ask the MCU 'who are you?'."""
        await self._send_frame(MSG_WHOAMI, b"")

    async def send_json_cmd(self, type_str: str, payload: Optional[dict] = None) -> None:
        """
        Generic helper to send a high-level JSON command to the robot.

        RobotCommandsMixin relies on this method to implement cmd_* helpers.
        """
        cmd_obj = {
            "kind": "cmd",
            "type": type_str,
            "seq": self._next_seq(),
            "payload": payload or {},
        }
        data = json.dumps(cmd_obj).encode("utf-8")
        await self._send_frame(MSG_CMD_JSON, data)


class AsyncRobotClient(BaseAsyncRobotClient, RobotCommandsMixin):
    """
    Full async client:

      - Inherits all core transport / framing / telemetry behavior from
        BaseAsyncRobotClient.
      - Adds the autogenerated cmd_* JSON command helpers from RobotCommandsMixin.

    All module- or hardware-specific helpers (servo, GPIO, DC, IMU processing, etc.)
    should live in their respective HostModule classes, not here.
    """
    pass
