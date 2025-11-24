from typing import Any, Dict, Optional

from .models import TelemetryPacket, ImuTelemetry, UltrasonicTelemetry, LidarTelemetry, StepperTelemetry


def _float_or_none(v: Any) -> Optional[float]:
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def parse_telemetry(msg: Dict[str, Any]) -> TelemetryPacket:
    """
    Parse a TELEMETRY JSON message from the MCU into typed models.

    Matches payloads like:
      {
        "src": "mcu",
        "type": "TELEMETRY",
        "ts_ms": 769001,
        "data": {
          "ultrasonic": {"sensor_id": 0, "attached": false, ...},
          "imu": {"online": false, "ok": false, ...},
          "lidar": {...}
        }
      }
    """
    ts_ms = int(msg.get("ts_ms", 0))
    data = msg.get("data", {}) or {}

    # --- IMU ---
    imu = None
    imu_raw = data.get("imu")
    if isinstance(imu_raw, dict):
        imu = ImuTelemetry(
            online=bool(imu_raw.get("online", False)),
            ok=bool(imu_raw.get("ok", False)),
            ax_g=_float_or_none(imu_raw.get("ax_g")),
            ay_g=_float_or_none(imu_raw.get("ay_g")),
            az_g=_float_or_none(imu_raw.get("az_g")),
            gx_dps=_float_or_none(imu_raw.get("gx_dps")),
            gy_dps=_float_or_none(imu_raw.get("gy_dps")),
            gz_dps=_float_or_none(imu_raw.get("gz_dps")),
            temp_c=_float_or_none(imu_raw.get("temp_c")),
        )

    # --- Ultrasonic ---
    ultrasonic = None
    ultra_raw = data.get("ultrasonic")
    if isinstance(ultra_raw, dict):
        ultrasonic = UltrasonicTelemetry(
            sensor_id=int(ultra_raw.get("sensor_id", 0)),
            attached=bool(ultra_raw.get("attached", False)),
            ok=ultra_raw.get("ok"),  # may be None right now
            distance_cm=_float_or_none(ultra_raw.get("distance_cm")),
            ts_ms=ts_ms,
        )
    # --- Lidar ---
    lidar = None
    lidar_raw = data.get("lidar")
    if isinstance(lidar_raw, dict):
        lidar = LidarTelemetry(
            online=bool(lidar_raw.get("online", False)),
            ok=bool(lidar_raw.get("ok", False)),
            distance_m=_float_or_none(lidar_raw.get("distance_m")),
            signal=_float_or_none(lidar_raw.get("signal")),
            ts_ms=ts_ms,
        )

        # --- 🔹 NEW: Stepper0 ---
        if "stepper0" in data:
            s_raw = data["stepper0"] or {}
            step = StepperTelemetry(
                ts_ms=ts_ms,
                motor_id=s_raw.get("motor_id"),
                attached=s_raw.get("attached"),
                enabled=s_raw.get("enabled"),
                moving=s_raw.get("moving"),
                dir_forward=s_raw.get("dir_forward"),
                last_cmd_steps=s_raw.get("last_cmd_steps"),
                last_cmd_speed=s_raw.get("last_cmd_speed"),
            )
    return TelemetryPacket(
        ts_ms=ts_ms,
        raw=msg,
        imu=imu,
        ultrasonic=ultrasonic,
        lidar=lidar
    )
