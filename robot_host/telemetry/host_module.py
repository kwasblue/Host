# telemetry/host_module.py
from __future__ import annotations

from typing import Optional, Dict, Any

from robot_host.core.event_bus import EventBus
from .parser import parse_telemetry
from .models import TelemetryPacket


class TelemetryHostModule:
    def __init__(self, bus: EventBus, source_topic: str = "telemetry.raw") -> None:
        self._bus = bus
        self._latest: Optional[TelemetryPacket] = None
        bus.subscribe(source_topic, self._on_raw)

    @property
    def latest(self) -> Optional[TelemetryPacket]:
        return self._latest

    def _on_raw(self, msg: Dict[str, Any]) -> None:
        if not isinstance(msg, dict):
            return
        if msg.get("type") != "TELEMETRY":
            return

        pkt = parse_telemetry(msg)
        self._latest = pkt

        # Full packet
        self._bus.publish("telemetry.packet", pkt)

        # Typed streams
        if pkt.imu is not None:
            self._bus.publish("telemetry.imu", pkt.imu)

        if pkt.ultrasonic is not None:
            self._bus.publish("telemetry.ultrasonic", pkt.ultrasonic)

        if pkt.lidar is not None:
            self._bus.publish("telemetry.lidar", pkt.lidar)

        if pkt.encoder0 is not None:
            self._bus.publish("telemetry.encoder0", pkt.encoder0)

        if pkt.stepper0 is not None:
            self._bus.publish("telemetry.stepper0", pkt.stepper0)

        if pkt.dc_motor0 is not None:
            self._bus.publish("telemetry.dc_motor0", pkt.dc_motor0)
