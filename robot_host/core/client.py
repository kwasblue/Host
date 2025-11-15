import time
from typing import Any
import struct
import json

from robot_host.core.event_bus import EventBus
from robot_host.core import protocol
from robot_host.core.messages import MsgType
from robot_host.transports.base_transport import BaseTransport


class RobotClient:
    def __init__(self, transport: BaseTransport) -> None:
        self.bus = EventBus()
        self.transport = transport
        self.transport.set_frame_handler(self._on_frame)

    def start(self) -> None:
        print("[RobotClient] Starting transport...")
        self.transport.start()

    def stop(self) -> None:
        print("[RobotClient] Stopping transport...")
        self.transport.stop()

    def send_whoami(self):
        # Empty payload
        self.transport.send_frame(msg_type=MsgType.WHOAMI, payload=b"")
    
    def send_json_cmd(self, cmd_dict: dict):
        """
        Send a high-level JSON command to the robot.
        cmd_dict should look like:
        {
          "kind": "cmd",
          "type": "CMD_LED_ON",
          "payload": {...}
        }
        """
        payload = json.dumps(cmd_dict).encode("utf-8")
        # ✅ msg_type must be an int, not bytes
        self.transport.send_frame(protocol.MSG_CMD_JSON, payload)
    
    def send_led_on(self):
        self.send_json_cmd({
            "kind": "cmd",
            "type": "CMD_LED_ON",
            "payload": {}
        })

    def send_led_off(self):
        self.send_json_cmd({
            "kind": "cmd",
            "type": "CMD_LED_OFF",
            "payload": {}
        })


    # === incoming from MCU ===

    def _on_frame(self, body: bytes) -> None:
        if not body:
            return
        msg_type = body[0]
        payload = body[1:]

        now = time.time()

        mt = MsgType(msg_type)

        if mt == MsgType.PONG:
            self.bus.publish("pong", {"raw": payload, "ts": time.time()})
        elif mt == MsgType.HEARTBEAT:
            self.bus.publish("heartbeat", {"raw": payload, "ts": time.time()})
        elif mt == MsgType.HELLO:
            info = self._parse_hello(payload)
            self.bus.publish("hello", info)
            print(f"[Host] Connected to {info['name']} "
                  f"(fw {info['fw']}, proto v{info['protocol_version']}, caps=0x{info['caps']:08X})")

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

    def _parse_hello(self, payload: bytes) -> dict:
        # uint8 ver, major, minor, patch, uint32 robot_id
        if len(payload) < 4 + 4 + 1:
            return {"error": "HELLO payload too short", "raw": payload}

        ver, maj, minor, patch, robot_id = struct.unpack_from("<BBBBI", payload, 0)
        offset = 4 + 4  # 4 u8 + 1 u32

        name_len = payload[offset]
        offset += 1
        name = payload[offset:offset + name_len].decode("utf-8", errors="ignore")
        offset += name_len

        (caps,) = struct.unpack_from("<I", payload, offset)

        return {
            "protocol_version": ver,
            "fw": f"{maj}.{minor}.{patch}",
            "robot_id": robot_id,
            "name": name,
            "caps": caps,
            "ts": time.time(),
        }