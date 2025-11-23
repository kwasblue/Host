from typing import Any
from robot_host.core.client import AsyncRobotClient
from robot_host.core.event_bus import EventBus


class ModeHostModule:
    """Wrapper for CMD_SET_MODE (IDLE / ARMED / ACTIVE / ESTOP)."""

    def __init__(self, bus: EventBus, client: AsyncRobotClient) -> None:
        self._bus = bus
        self._client = client

    async def set_mode(self, mode: str) -> None:
        payload: dict[str, Any] = {"mode": mode}
        await self._client.send_json_cmd("CMD_SET_MODE", payload)
