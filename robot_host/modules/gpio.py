from typing import Any
from robot_host.core.client import AsyncRobotClient
from robot_host.core.event_bus import EventBus


class GpioHostModule:
    """GPIO write/read/toggle helpers."""

    def __init__(self, bus: EventBus, client: AsyncRobotClient) -> None:
        self._bus = bus
        self._client = client

    async def write(self, channel: int, value: int) -> None:
        payload: dict[str, Any] = {"channel": int(channel), "value": int(value)}
        await self._client.send_json_cmd("CMD_GPIO_WRITE", payload)

    async def read(self, channel: int) -> None:
        payload: dict[str, Any] = {"channel": int(channel)}
        await self._client.send_json_cmd("CMD_GPIO_READ", payload)

    async def toggle(self, channel: int) -> None:
        payload: dict[str, Any] = {"channel": int(channel)}
        await self._client.send_json_cmd("CMD_GPIO_TOGGLE", payload)
