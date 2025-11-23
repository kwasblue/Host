from typing import Any
from robot_host.core.client import AsyncRobotClient
from robot_host.core.event_bus import EventBus


class StepperHostModule:
    """Stepper move/enable helpers."""

    def __init__(self, bus: EventBus, client: AsyncRobotClient) -> None:
        self._bus = bus
        self._client = client

    async def move_rel(self, motor_id: int, steps: int, speed_steps_s: float) -> None:
        payload: dict[str, Any] = {
            "motor_id": int(motor_id),
            "steps": int(steps),
            "speed_steps_s": float(speed_steps_s),
        }
        await self._client.send_json_cmd("CMD_STEPPER_MOVE_REL", payload)

    async def enable(self, motor_id: int, enable: bool = True) -> None:
        payload: dict[str, Any] = {
            "motor_id": int(motor_id),
            "enable": bool(enable),
        }
        await self._client.send_json_cmd("CMD_STEPPER_ENABLE", payload)
