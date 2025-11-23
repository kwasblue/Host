# telemetry/host_module.py
from robot_host.core.event_bus import EventBus
from typing import Optional, Dict, Any
from .parser import parse_telemetry
from .models import TelemetryPacket, ImuTelemetry, UltrasonicTelemetry

class TelemetryHostModule:
    def __init__(self, bus: EventBus, source_topic: str = "telemetry.raw") -> None:
        self._bus = bus
        self._latest: Optional[TelemetryPacket] = None
        bus.subscribe(source_topic, self._on_raw)

    def _on_raw(self, msg: Dict[str, Any]) -> None:
        if not isinstance(msg, dict):
            return
        if msg.get("type") != "TELEMETRY":
            return

        pkt = parse_telemetry(msg)
        self._latest = pkt

        self._bus.publish("telemetry.packet", pkt)

        if pkt.imu is not None:
            self._bus.publish("telemetry.imu", pkt.imu)

        if pkt.ultrasonic is not None:
            self._bus.publish("telemetry.ultrasonic", pkt.ultrasonic)
        
        if pkt.lidar is not None:
            self._bus.publish("telemetry.lidar", pkt.lidar)
