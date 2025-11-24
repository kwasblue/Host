from dataclasses import dataclass
from typing import Any, Dict, Optional


@dataclass
class ImuTelemetry:
    online: bool
    ok: bool
    ax_g: Optional[float] = None
    ay_g: Optional[float] = None
    az_g: Optional[float] = None
    gx_dps: Optional[float] = None
    gy_dps: Optional[float] = None
    gz_dps: Optional[float] = None
    temp_c: Optional[float] = None


@dataclass
class UltrasonicTelemetry:
    sensor_id: int
    attached: bool
    ok: Optional[bool] = None
    distance_cm: Optional[float] = None
    ts_ms: Optional[int] = None


@dataclass
class LidarTelemetry:
    online: bool
    ok: bool
    distance_m: Optional[float] = None
    signal: Optional[float] = None
    ts_ms: Optional[int] = None


@dataclass
class TelemetryPacket:
    ts_ms: int
    raw: Dict[str, Any]
    imu: Optional[ImuTelemetry] = None
    ultrasonic: Optional[UltrasonicTelemetry] = None
    lidar: Optional[LidarTelemetry] = None


@dataclass
class StepperTelemetry:
    name: str = "stepper0"

    ts_ms: Optional[int] = None

    motor_id: Optional[int] = None
    attached: Optional[bool] = None

    enabled: Optional[bool] = None
    moving: Optional[bool] = None
    dir_forward: Optional[bool] = None

    last_cmd_steps: Optional[int] = None
    last_cmd_speed: Optional[float] = None