from typing import Any
from robot_host.core.client import AsyncRobotClient
from robot_host.core.event_bus import EventBus


class LoggingControlModule:
    """Host-side helper for CMD_SET_LOG_LEVEL."""

    def __init__(self, bus: EventBus, client: AsyncRobotClient) -> None:
        self._bus = bus
        self._client = client

    async def set_level(self, level: str = "info") -> None:
        payload: dict[str, Any] = {"level": level}
        await self._client.send_json_cmd("CMD_SET_LOG_LEVEL", payload)
