from typing import Any
from robot_host.core.client import AsyncRobotClient
from robot_host.core.event_bus import EventBus


class MotionHostModule:
    """High-level motion helpers (SET_VEL / STOP / ESTOP)."""

    def __init__(self, bus: EventBus, client: AsyncRobotClient) -> None:
        self._bus = bus
        self._client = client

    async def set_velocity(self, vx: float, omega: float) -> None:
        payload: dict[str, Any] = {"vx": float(vx), "omega": float(omega)}
        await self._client.send_json_cmd("CMD_SET_VEL", payload)

    async def stop(self) -> None:
        await self._client.send_json_cmd("CMD_STOP", {})

    async def estop(self) -> None:
        await self._client.send_json_cmd("CMD_ESTOP", {})

    async def clear_estop(self) -> None:
        await self._client.send_json_cmd("CMD_CLEAR_ESTOP", {})
