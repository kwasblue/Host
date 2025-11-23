from typing import Any
from robot_host.core.client import AsyncRobotClient
from robot_host.core.event_bus import EventBus


class ServoHostModule:
    """Servo attach/set helpers."""

    def __init__(self, bus: EventBus, client: AsyncRobotClient) -> None:
        self._bus = bus
        self._client = client

    async def attach(self, servo_id: int = 0, min_us: int = 1000, max_us: int = 2000) -> None:
        payload: dict[str, Any] = {
            "servo_id": int(servo_id),
            "min_us": int(min_us),
            "max_us": int(max_us),
        }
        await self._client.send_json_cmd("CMD_SERVO_ATTACH", payload)

    async def set_angle(self, servo_id: int, angle_deg: float, duration_ms: int = 0) -> None:
        payload: dict[str, Any] = {
            "servo_id": int(servo_id),
            "angle_deg": float(angle_deg),
            "duration_ms": int(duration_ms),
        }
        await self._client.send_json_cmd("CMD_SERVO_SET_ANGLE", payload)
