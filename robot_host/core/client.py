# robot_host/core/client_async.py

import asyncio
import json
from typing import Optional

from .event_bus import EventBus
from . import protocol, messages
from .messages import MsgType 

# Re-export / alias protocol constants for convenience
MSG_PING       = protocol.MSG_PING
MSG_PONG       = protocol.MSG_PONG
MSG_HEARTBEAT  = protocol.MSG_HEARTBEAT
MSG_WHOAMI     = protocol.MSG_WHOAMI
MSG_CMD_JSON   = protocol.MSG_CMD_JSON


class AsyncRobotClient:
    """
    Async host-side client:
      - Uses an async transport (e.g. AsyncTcpTransport)
      - Frames are produced by protocol.encode(...) on the way out
        and by transport + protocol.extract_frames(...) on the way in.
      - Publishes events to EventBus
      - Provides async helpers like send_ping(), send_led_on(), etc.
    """

    def __init__(self, transport, bus: Optional[EventBus] = None) -> None:
        self.transport = transport
        self.bus = bus or EventBus()
        self._running = False
        self._seq = 0  # for JSON command sequencing if desired

        # Transport will call _on_frame(body) where:
        #   body[0] = msg_type
        #   body[1:] = payload bytes
        self.transport.set_frame_handler(self._on_frame)

    # ---------- Lifecycle ----------

    async def start(self) -> None:
        """
        Start the transport (connect/reconnect loop).
        """
        await self.transport.start()
        self._running = True
        print("[RobotClient] Started")

    async def stop(self) -> None:
        self._running = False
        await self.transport.stop()
        print("[RobotClient] Stopped")

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

        # Debug (optional)
        # print(f"[RobotClient] Frame: type=0x{msg_type:02X}, len={len(payload)}")

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

        The MCU side (CommandHandler / other modules) will typically send things like:
          {"kind":"event","type":"HELLO","payload":{...}, "seq": N}
        or telemetry/resp variants.
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

        kind = obj.get("kind", "")
        type_str = obj.get("type", "")

        # Example: Identity / hello events
        if type_str == "HELLO":
            # Your IdentityModule on the MCU can build this.
            self.bus.publish("hello", obj)
        else:
            # Generic JSON channel
            self.bus.publish("json", obj)

    # ---------- Outgoing commands ----------

    async def send_ping(self) -> None:
        """
        Send a simple PING frame (no payload).
        """
        await self.transport.send_frame(MSG_PING, b"")

    async def send_whoami(self) -> None:
        """
        Ask the MCU 'who are you?'. IdentityModule should respond with
        some HELLO/identity JSON via MSG_CMD_JSON.
        """
        await self.transport.send_frame(MSG_WHOAMI, b"")

    def _next_seq(self) -> int:
        self._seq += 1
        return self._seq

    async def send_json_cmd(self, type_str: str, payload: Optional[dict] = None) -> None:
        """
        Generic helper to send a high-level JSON command to the robot.

        This matches what your ESP32 CommandHandler expects:
          {
            "kind": "cmd",
            "type": "<type_str>",         # e.g. "CMD_LED_ON"
            "seq":  <int>,
            "payload": { ... }
          }
        wrapped in a MSG_CMD_JSON frame.
        """
        cmd_obj = {
            "kind": "cmd",
            "type": type_str,
            "seq": self._next_seq(),
            "payload": payload or {},
        }
        data = json.dumps(cmd_obj).encode("utf-8")
        await self.transport.send_frame(MSG_CMD_JSON, data)

    # Convenience helpers matching your MCU CommandHandler commands:

    async def send_led_on(self) -> None:
        # MCU side: case CmdType::LED_ON
        await self.send_json_cmd("CMD_LED_ON")

    async def send_led_off(self) -> None:
        # MCU side: case CmdType::LED_OFF
        await self.send_json_cmd("CMD_LED_OFF")

    # You can add more:
    # async def send_set_mode(self, mode: str) -> None:
    #     await self.send_json_cmd("CMD_SET_MODE", {"mode": mode})
    #
    # async def send_set_vel(self, vx: float, omega: float) -> None:
    #     await self.send_json_cmd("CMD_SET_VEL", {"vx": vx, "omega": omega})

        # ---- Outgoing ----

    async def send_ping(self) -> None:
        frame = protocol.encode(protocol.MSG_PING)
        await self.transport.send_bytes(frame)

    async def send_whoami(self) -> None:
        frame = protocol.encode(protocol.MSG_WHOAMI)
        await self.transport.send_bytes(frame)

    async def send_led_on(self) -> None:
        """
        Send a JSON command that your MCU CommandHandler understands,
        framed as MSG_CMD_JSON + JSON payload.
        """
        cmd_obj = {
            "kind": "cmd",
            "type": "CMD_LED_ON",
            "seq": 0,
            "payload": {},
        }
        payload = json.dumps(cmd_obj).encode("utf-8")
        await self.transport.send_frame(protocol.MSG_CMD_JSON, payload)

    async def send_led_off(self) -> None:
        cmd_obj = {
            "kind": "cmd",
            "type": "CMD_LED_OFF",
            "seq": 0,
            "payload": {},
        }
        payload = json.dumps(cmd_obj).encode("utf-8")
        await self.transport.send_frame(protocol.MSG_CMD_JSON, payload)
    
    async def send_servo_attach(self, servo_id: int = 0) -> None:
        cmd = {
            "kind": "cmd",
            "type": "CMD_SERVO_ATTACH",
            "seq": 0,
            "payload": {
                "servo_id": servo_id,
                "channel": 0,
                "min_us": 1000,
                "max_us": 2000,
            },
        }
        payload = json.dumps(cmd).encode("utf-8")
        frame = protocol.encode(protocol.MSG_CMD_JSON, payload)
        await self.transport.send_bytes(frame)

    async def send_servo_angle(self, servo_id: int, angle_deg: float) -> None:
        cmd = {
            "kind": "cmd",
            "type": "CMD_SERVO_SET_ANGLE",
            "seq": 0,
            "payload": {
                "servo_id": servo_id,
                "angle_deg": angle_deg,
            },
        }
        payload = json.dumps(cmd).encode("utf-8")
        frame = protocol.encode(protocol.MSG_CMD_JSON, payload)
        await self.transport.send_bytes(frame)