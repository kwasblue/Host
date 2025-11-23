# robot_host/telemetry/file_logger.py
from __future__ import annotations

import csv
from pathlib import Path
from typing import Optional

from robot_host.core.event_bus import EventBus
from .models import ImuTelemetry, UltrasonicTelemetry


class TelemetryFileLogger:
    def __init__(self, bus: EventBus, log_dir: Path) -> None:
        self._log_dir = log_dir
        self._imu_fp = None
        self._ultra_fp = None
        self._imu_writer: Optional[csv.writer] = None
        self._ultra_writer: Optional[csv.writer] = None

        bus.subscribe("telemetry.imu", self._on_imu)
        bus.subscribe("telemetry.ultrasonic.structured", self._on_ultra)

    def start(self) -> None:
        self._log_dir.mkdir(parents=True, exist_ok=True)

        imu_path = self._log_dir / "imu.csv"
        ultra_path = self._log_dir / "ultrasonic.csv"

        self._imu_fp = imu_path.open("w", newline="")
        self._ultra_fp = ultra_path.open("w", newline="")

        self._imu_writer = csv.writer(self._imu_fp)
        self._ultra_writer = csv.writer(self._ultra_fp)

        self._imu_writer.writerow(
            ["ax_g", "ay_g", "az_g",
             "gx_dps", "gy_dps", "gz_dps",
             "temp_c", "ok", "online"]
        )

        self._ultra_writer.writerow(
            ["sensor_id", "attached", "ok", "distance_cm"]
        )

    def stop(self) -> None:
        if self._imu_fp:
            self._imu_fp.close()
        if self._ultra_fp:
            self._ultra_fp.close()

    def _on_imu(self, imu: ImuTelemetry) -> None:
        if not self._imu_writer:
            return
        self._imu_writer.writerow([
            imu.ax_g, imu.ay_g, imu.az_g,
            imu.gx_dps, imu.gy_dps, imu.gz_dps,
            imu.temp_c, imu.ok, imu.online,
        ])

    def _on_ultra(self, ultra: UltrasonicTelemetry) -> None:
        if not self._ultra_writer:
            return
        self._ultra_writer.writerow([
            ultra.sensor_id, ultra.attached, ultra.ok, ultra.distance_cm
        ])
