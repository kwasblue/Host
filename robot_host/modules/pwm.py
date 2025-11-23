from typing import Any
from robot_host.core.client import AsyncRobotClient
from robot_host.core.event_bus import EventBus


class PwmHostModule:
    """PWM_SET helper."""

    def __init__(self, bus: EventBus, client: AsyncRobotClient) -> None:
        self._bus = bus
        self._client = client

    async def set(self, channel: int, duty: float, freq_hz: float | None = None) -> None:
        payload: dict[str, Any] = {
            "channel": int(channel),
            "duty": float(duty),
        }
        if freq_hz is not None:
            payload["freq_hz"] = float(freq_hz)
        await self._client.send_json_cmd("CMD_PWM_SET", payload)
