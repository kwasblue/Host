from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from robot_host.core.client import AsyncRobotClient
from robot_host.core.event_bus import EventBus


@dataclass
class EncoderDefaults:
    encoder_id: int = 0
    pin_a: int = 32
    pin_b: int = 33


class EncoderHostModule:
    """Host-side wrapper around encoder commands.

    Expects CMD_ENCODER_ATTACH / CMD_ENCODER_READ / CMD_ENCODER_RESET
    to be implemented on the MCU.
    """

    def __init__(
        self,
        bus: EventBus,
        client: AsyncRobotClient,
        defaults: EncoderDefaults | None = None,
    ) -> None:
        self._bus = bus
        self._client = client
        self._defaults = defaults or EncoderDefaults()

    async def attach(
        self,
        encoder_id: int | None = None,
        pin_a: int | None = None,
        pin_b: int | None = None,
    ) -> None:
        eid = encoder_id if encoder_id is not None else self._defaults.encoder_id
        payload: dict[str, Any] = {"encoder_id": eid}
        payload["pin_a"] = pin_a if pin_a is not None else self._defaults.pin_a
        payload["pin_b"] = pin_b if pin_b is not None else self._defaults.pin_b

        await self._client.send_json_cmd("CMD_ENCODER_ATTACH", payload)

    async def read(self, encoder_id: int | None = None) -> None:
        eid = encoder_id if encoder_id is not None else self._defaults.encoder_id
        payload: dict[str, Any] = {"encoder_id": eid}
        await self._client.send_json_cmd("CMD_ENCODER_READ", payload)

    async def reset(self, encoder_id: int | None = None) -> None:
        eid = encoder_id if encoder_id is not None else self._defaults.encoder_id
        payload: dict[str, Any] = {"encoder_id": eid}
        await self._client.send_json_cmd("CMD_ENCODER_RESET", payload)
