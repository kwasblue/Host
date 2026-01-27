import struct

from robot_host.core.client import BaseAsyncRobotClient
from robot_host.core import protocol
from helpers import CapturingBus
from fakes.fake_async_transport import FakeAsyncTransport


# These should match your MCU section IDs + your python binary parser expectations.
SECTION_IMU = 1


def _build_sectioned_payload(ts_ms: int = 1234, seq: int = 7) -> bytes:
    """
    Matches your MCU TelemetryModule format:
      u8  version (=1)
      u16 seq
      u32 ts_ms
      u8  section_count
      repeat:
        u8  section_id
        u16 section_len
        u8[] section_bytes
    """
    version = 1

    # IMU section bytes (example layout used earlier):
    # online(u8), ok(u8),
    # ax_mg(i16), ay_mg(i16), az_mg(i16),
    # gx_mdps(i16), gy_mdps(i16), gz_mdps(i16),
    # temp_c_centi(i16)
    imu_bytes = struct.pack(
        "<BBhhhhhhh",
        1,  # online
        1,  # ok
        1000,   # ax_mg = 1.0 g
        -500,   # ay_mg = -0.5 g
        0,      # az_mg
        250,    # gx_mdps = 0.25 dps
        -250,   # gy_mdps
        0,      # gz_mdps
        2500,   # temp_c_centi = 25.00 C
    )

    payload = bytearray()
    payload += struct.pack("<BHI", version, seq & 0xFFFF, ts_ms & 0xFFFFFFFF)
    payload += struct.pack("<B", 1)  # section_count
    payload += struct.pack("<BH", SECTION_IMU, len(imu_bytes))
    payload += imu_bytes
    return bytes(payload)


def test_client_routes_telemetry_bin_to_packet_topic():
    bus = CapturingBus()
    transport = FakeAsyncTransport()
    client = BaseAsyncRobotClient(transport=transport, bus=bus, require_version_match=False)

    telem_payload = _build_sectioned_payload(ts_ms=1234, seq=7)

    # _on_frame expects: body = [msg_type][payload...]
    body = bytes([protocol.MSG_TELEMETRY_BIN]) + telem_payload

    # Fake transport injects a complete decoded "body" into the frame handler
    transport._inject_body(body)

    evt = bus.last("telemetry.packet")
    assert evt is not None, "Expected telemetry.packet publication for MSG_TELEMETRY_BIN"
    packet = evt.data

    # Light sanity checks (don’t overfit)
    assert packet.ts_ms == 1234
    assert packet.imu is not None
    assert packet.imu.online is True
