# telemetry/binary_parser.py
from __future__ import annotations

import struct
from typing import Dict, Any

from .models import (
    TelemetryPacket,
    ImuTelemetry,
    UltrasonicTelemetry,
    LidarTelemetry,
    EncoderTelemetry,
    StepperTelemetry,
    DcMotorTelemetry,
)

# Section IDs (MUST match MCU registerBinProvider(section_id, ...))
TELEM_IMU         = 1
TELEM_ULTRASONIC  = 2
TELEM_LIDAR       = 3
TELEM_ENCODER0    = 4
TELEM_STEPPER0    = 5
TELEM_DC_MOTOR0   = 6

# Outer "sectioned packet" header (LE):
# u8 version, u16 seq, u32 ts_ms, u8 section_count
# (this matches your C++: put_u8, put_u16, put_u32, put_u8)
_PKT_HDR = struct.Struct("<BHI B".replace(" ", ""))  # "<BHIB"
# Explicit: "<BHIB" == u8, u16, u32, u8

def _make_empty(ts_ms: int, raw_len: int, meta: Dict[str, Any]) -> TelemetryPacket:
    return TelemetryPacket(
        ts_ms=ts_ms,
        raw=meta | {"bin": True, "len": raw_len},
        imu=None,
        ultrasonic=None,
        lidar=None,
        encoder0=None,
        stepper0=None,
        dc_motor0=None,
    )

def parse_telemetry_bin(payload: bytes) -> TelemetryPacket:
    """
    Parse a *sectioned* binary telemetry payload (the bytes after MSG_TELEMETRY_BIN).
    Format (LE):
      u8  version (=1)
      u16 seq
      u32 ts_ms
      u8  section_count
      repeat:
        u8  section_id
        u16 section_len
        u8[] section_bytes
    """
    if len(payload) < _PKT_HDR.size:
        return _make_empty(0, len(payload), {"error": "short_header"})

    ver, seq, ts_ms, section_count = _PKT_HDR.unpack_from(payload, 0)
    off = _PKT_HDR.size

    pkt = _make_empty(ts_ms, len(payload), {"ver": int(ver), "seq": int(seq), "sections": int(section_count)})

    # Parse each section
    for _ in range(int(section_count)):
        if off + 3 > len(payload):
            pkt.raw["error"] = "short_section_header"
            return pkt

        section_id = payload[off]
        section_len = int.from_bytes(payload[off + 1 : off + 3], "little")
        off += 3

        if off + section_len > len(payload):
            pkt.raw["error"] = "short_section_body"
            pkt.raw["bad_section_id"] = int(section_id)
            pkt.raw["needed"] = section_len
            pkt.raw["have"] = len(payload) - off
            return pkt

        body = payload[off : off + section_len]
        off += section_len

        # ---- Section decoders (v1) ----

        # IMU v1:
        # online(u8), ok(u8),
        # ax_mg(i16), ay_mg(i16), az_mg(i16),
        # gx_mdps(i16), gy_mdps(i16), gz_mdps(i16),
        # temp_c_centi(i16)
        if section_id == TELEM_IMU:
            if len(body) >= struct.calcsize("<BB7h"):
                online, ok, ax_mg, ay_mg, az_mg, gx_mdps, gy_mdps, gz_mdps, temp_c_centi = struct.unpack_from("<BB7h", body, 0)
                pkt.imu = ImuTelemetry(
                    online=bool(online),
                    ok=bool(ok),
                    ax_g=ax_mg / 1000.0,
                    ay_g=ay_mg / 1000.0,
                    az_g=az_mg / 1000.0,
                    gx_dps=gx_mdps / 1000.0,
                    gy_dps=gy_mdps / 1000.0,
                    gz_dps=gz_mdps / 1000.0,
                    temp_c=temp_c_centi / 100.0,
                )
            continue

        # Ultrasonic v1:
        # sensor_id(u8), attached(u8), ok(u8), dist_mm(u16)
        if section_id == TELEM_ULTRASONIC:
            if len(body) >= struct.calcsize("<BBBH"):
                sensor_id, attached, ok, dist_mm = struct.unpack_from("<BBBH", body, 0)
                pkt.ultrasonic = UltrasonicTelemetry(
                    sensor_id=int(sensor_id),
                    attached=bool(attached),
                    ok=bool(ok),
                    distance_cm=(dist_mm / 10.0) if dist_mm != 0 else None,
                    ts_ms=ts_ms,
                )
            continue

        # LiDAR v1:
        # online(u8), ok(u8), dist_mm(u16), signal(u16)
        if section_id == TELEM_LIDAR:
            if len(body) >= struct.calcsize("<BBHH"):
                online, ok, dist_mm, signal = struct.unpack_from("<BBHH", body, 0)
                pkt.lidar = LidarTelemetry(
                    online=bool(online),
                    ok=bool(ok),
                    distance_m=(dist_mm / 1000.0) if dist_mm != 0 else None,
                    signal=(signal if signal != 0 else None),
                    ts_ms=ts_ms,
                )
            continue

        # Encoder0 v1:
        # ticks(i32)
        if section_id == TELEM_ENCODER0:
            if len(body) >= struct.calcsize("<i"):
                (ticks,) = struct.unpack_from("<i", body, 0)
                pkt.encoder0 = EncoderTelemetry(ts_ms=ts_ms, encoder_id=0, ticks=int(ticks))
            continue

        # Stepper0 v1:
        # motor_id(i8), attached(u8), enabled(u8), moving(u8), dir_forward(u8),
        # last_cmd_steps(i32), last_cmd_speed_centi(i16)
        if section_id == TELEM_STEPPER0:
            if len(body) >= struct.calcsize("<bBBBBih"):
                motor_id, attached, enabled, moving, dir_forward, last_steps, speed_centi = struct.unpack_from("<bBBBBih", body, 0)
                pkt.stepper0 = StepperTelemetry(
                    ts_ms=ts_ms,
                    motor_id=int(motor_id),
                    attached=bool(attached),
                    enabled=bool(enabled),
                    moving=bool(moving),
                    dir_forward=bool(dir_forward),
                    last_cmd_steps=int(last_steps),
                    last_cmd_speed=speed_centi / 100.0,
                )
            continue

        # DC Motor0 v1 (minimal):
        # attached(u8), speed_centi(i16)
        if section_id == TELEM_DC_MOTOR0:
            if len(body) >= struct.calcsize("<Bh"):
                attached, speed_centi = struct.unpack_from("<Bh", body, 0)
                pkt.dc_motor0 = DcMotorTelemetry(
                    ts_ms=ts_ms,
                    motor_id=0,
                    attached=bool(attached),
                    in1_pin=None, in2_pin=None, pwm_pin=None, ledc_channel=None,
                    gpio_ch_in1=None, gpio_ch_in2=None, pwm_ch=None,
                    speed=speed_centi / 100.0,
                    freq_hz=None,
                    resolution_bits=None,
                )
            continue

        # Unknown section_id -> ignore (forward-compatible)
        # You can store stats if you want:
        # pkt.raw.setdefault("unknown_sections", []).append(int(section_id))

    return pkt
