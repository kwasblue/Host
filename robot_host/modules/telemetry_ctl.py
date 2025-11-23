from typing import Any
from robot_host.core.client import AsyncRobotClient
from robot_host.core.event_bus import EventBus


class TelemetryControlModule:
    """Host-side helper for CMD_TELEM_SET_INTERVAL."""

    def __init__(self, bus: EventBus, client: AsyncRobotClient) -> None:
        self._bus = bus
        self._client = client

    async def set_interval(self, interval_ms: int) -> None:
        payload: dict[str, Any] = {"interval_ms": interval_ms}
        await self._client.send_json_cmd("CMD_TELEM_SET_INTERVAL", payload)
