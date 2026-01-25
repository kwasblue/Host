import pytest
from robot_host.core import protocol


def test_encode_decode_roundtrip_single_frame():
    payload = b"hello"
    frame = protocol.encode(protocol.MSG_PING, payload)

    buf = bytearray(frame)
    bodies = []
    protocol.extract_frames(buf, lambda body: bodies.append(body))

    assert len(bodies) == 1
    body = bodies[0]
    assert body[0] == protocol.MSG_PING
    assert body[1:] == payload
    assert buf == bytearray()  # consumed


def test_extract_multiple_frames_with_noise_and_partial():
    frames = [
        protocol.encode(protocol.MSG_PING, b"a"),
        protocol.encode(protocol.MSG_PONG, b""),
        protocol.encode(protocol.MSG_CMD_JSON, b'{"x":1}'),
    ]

    noisy = b"\x00\x01\x02" + frames[0] + b"\x99" + frames[1] + frames[2]
    buf = bytearray(noisy)

    bodies = []
    protocol.extract_frames(buf, lambda body: bodies.append(body))

    assert [b[0] for b in bodies] == [protocol.MSG_PING, protocol.MSG_PONG, protocol.MSG_CMD_JSON]
    assert bodies[0][1:] == b"a"
    assert bodies[2][1:] == b'{"x":1}'
    assert len(buf) == 0


def test_bad_checksum_resync():
    good = protocol.encode(protocol.MSG_PING, b"abc")
    # corrupt one byte in payload
    bad = bytearray(good)
    bad[4] ^= 0xFF  # flip first payload byte
    bad = bytes(bad)

    buf = bytearray(bad + good)
    bodies = []
    protocol.extract_frames(buf, lambda body: bodies.append(body))

    # should skip bad one and still parse good
    assert len(bodies) == 1
    assert bodies[0][0] == protocol.MSG_PING
    assert bodies[0][1:] == b"abc"
