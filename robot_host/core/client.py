# robot_host/core/client_async.py

from __future__ import annotations

import asyncio
import json
import inspect
from typing import Optional, Protocol, Callable
import numpy as np
from .event_bus import EventBus
from . import protocol
from .messages import MsgType
from robot_host.config.client_commands import RobotCommandsMixin

# Optional: a simple structural type for transports
class HasSendBytes(Protocol):
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


class AsyncRobotClient(RobotCommandsMixin):
    """
    Async host-side client:
      - Uses an async-ish transport (AsyncTcpTransport, SerialTransport via StreamTransport, etc.)
      - Transport exposes async send_bytes(data: bytes)
      - Frames are produced by protocol.encode(...) on the way out
        and by transport + protocol.extract_frames(...) on the way in.
      - Publishes events to EventBus
      - Provides async helpers like send_ping(), send_led_on(), servo commands, etc.
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
        self._accel_bias = np.zeros(3, dtype=float)
        self._gyro_bias = np.zeros(3, dtype=float)


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

        if type_str == "TELEMETRY":
            # NEW: raw telemetry feed for host modules
            self.bus.publish("telemetry.raw", obj)

            # existing generic event (keep for now so nothing breaks)
            self.bus.publish("telemetry", obj)

            data = obj.get("data", {})

            # Convenience: existing ultrasonic fan-out (raw dict)
            ultra = data.get("ultrasonic")
            if ultra is not None:
                self.bus.publish("telemetry.ultrasonic", ultra)

            # IMU telemetry: feed into your math helper
            imu = data.get("imu")
            if imu is not None:
                ts_ms = obj.get("ts_ms")
                self._process_imu_telemetry(imu, ts_ms)

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
        """
        cmd_obj = {
            "kind": "cmd",
            "type": type_str,
            "seq": self._next_seq(),
            "payload": payload or {},
        }
        data = json.dumps(cmd_obj).encode("utf-8")
        await self._send_frame(MSG_CMD_JSON, data)

    # Convenience helpers matching your MCU CommandHandler commands:

    async def send_led_on(self) -> None:
        # MCU side: case CmdType::LED_ON
        await self.send_json_cmd("CMD_LED_ON")

    async def send_led_off(self) -> None:
        # MCU side: case CmdType::LED_OFF
        await self.send_json_cmd("CMD_LED_OFF")

    async def send_servo_attach(self, servo_id: int = 0) -> None:
        cmd = {
            "servo_id": servo_id,
            "channel": 0,
            "min_us": 500,
            "max_us": 2500,
        }
        await self.send_json_cmd("CMD_SERVO_ATTACH", cmd)

    async def send_servo_angle(self, servo_id: int, angle_deg: float) -> None:
        cmd = {
            "servo_id": servo_id,
            "angle_deg": angle_deg,
        }
        await self.send_json_cmd("CMD_SERVO_SET_ANGLE", cmd)

    async def smooth_servo_move(
        self,
        servo_id: int,
        start_deg: float,
        end_deg: float,
        steps: int = 30,
        total_time: float = 1,
    ) -> None:
        """
        Convenience helper to sweep a servo from start_deg to end_deg
        in 'steps' increments over 'total_time' seconds.
        """
        if steps <= 0:
            await self.send_servo_angle(servo_id=servo_id, angle_deg=end_deg)
            return

        step = (end_deg - start_deg) / steps
        delay = total_time / steps

        angle = start_deg
        for _ in range(steps + 1):
            await self.send_servo_angle(servo_id=servo_id, angle_deg=angle)
            angle += step
            await asyncio.sleep(delay)

    def _process_imu_telemetry(self, imu: dict, ts_ms: Optional[int]) -> None:
        online = imu.get("online", False)
        ok = imu.get("ok", False)

        if not online or not ok:
            self.bus.publish(
                "telemetry.imu",
                {"ts_ms": ts_ms, "online": online, "ok": ok},
            )
            return

        # Make a vector
        acc = np.array([
            float(imu.get("ax_g", 0.0)),
            float(imu.get("ay_g", 0.0)),
            float(imu.get("az_g", 0.0)),
        ])

        gyro = np.array([
            float(imu.get("gx_dps", 0.0)),
            float(imu.get("gy_dps", 0.0)),
            float(imu.get("gz_dps", 0.0)),
        ])

        ax, ay, az = acc
        gx, gy, gz = gyro

        acc_mag = np.linalg.norm(acc)

        roll_rad = np.arctan2(ay, az)
        pitch_rad = np.arctan2(-ax, np.sqrt(ay * ay + az * az))

        roll_deg = np.degrees(roll_rad)
        pitch_deg = np.degrees(pitch_rad)

        temp_c = float(imu.get("temp_c", 0.0))

        out = {
            "ts_ms": ts_ms,
            "online": online,
            "ok": ok,
            "ax_g": float(ax),
            "ay_g": float(ay),
            "az_g": float(az),
            "gx_dps": float(gx),
            "gy_dps": float(gy),
            "gz_dps": float(gz),
            "temp_c": temp_c,
            "acc_mag_g": float(acc_mag),
            "roll_rad": float(roll_rad),
            "pitch_rad": float(pitch_rad),
            "roll_deg": float(roll_deg),
            "pitch_deg": float(pitch_deg),
        }

        self.bus.publish("telemetry.imu", out)
    
    def set_imu_biases(self, accel_bias: np.ndarray, gyro_bias: np.ndarray) -> None:
        self._accel_bias = np.asarray(accel_bias, dtype=float)
        self._gyro_bias = np.asarray(gyro_bias, dtype=float)
