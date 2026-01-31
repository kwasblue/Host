from typing import Callable

HEADER = 0xAA

MSG_HEARTBEAT        = 0x01
MSG_PING             = 0x02
MSG_PONG             = 0x03
MSG_VERSION_REQUEST  = 0x04
MSG_VERSION_RESPONSE = 0x05
MSG_WHOAMI           = 0x10
MSG_TELEMETRY_BIN    = 0x30
MSG_CMD_JSON         = 0x50
MSG_CMD_BIN          = 0x51  # Binary command for high-rate streaming

_MAX_LEN = 65535  # 16-bit length

def encode(msg_type: int, payload: bytes = b"") -> bytes:
    """
    Encode a frame as:
        [HEADER][len_hi][len_lo][msg_type][payload...][checksum]

    where:
        length   = 1 + len(payload)  # msg_type + payload
        checksum = (length + msg_type + sum(payload)) & 0xFF
    """
    length = 1 + len(payload)
    if length <= 0 or length > _MAX_LEN:
        raise ValueError(f"Invalid frame length: {length}")

    len_hi = (length >> 8) & 0xFF
    len_lo = length & 0xFF

    checksum = (length + msg_type + sum(payload)) & 0xFF

    # Pre-allocate exact frame size (optimization: avoid multiple appends)
    payload_len = len(payload)
    frame = bytearray(5 + payload_len)  # header + len_hi + len_lo + msg_type + payload + checksum
    frame[0] = HEADER
    frame[1] = len_hi
    frame[2] = len_lo
    frame[3] = msg_type
    if payload_len > 0:
        frame[4:4 + payload_len] = payload
    frame[4 + payload_len] = checksum
    return bytes(frame)


_HEADER_BYTE = bytes([HEADER])  # Pre-allocated for find()

def extract_frames(buffer: bytearray, on_frame: Callable[[bytes], None]) -> None:
    """
    Parse as many frames as possible from buffer.

    Calls:
        on_frame(body)
    where:
        body[0] = msg_type
        body[1:] = payload

    Mutates buffer, removing consumed bytes.
    """

    i = 0
    n = len(buffer)

    # need at least HEADER + len_hi + len_lo + msg_type + checksum
    MIN_FRAME_HEADER = 1 + 2 + 1 + 1  # header + len_hi/lo + msg_type + checksum

    while i + MIN_FRAME_HEADER <= n:
        # Vectorized search for HEADER byte (faster than byte-by-byte)
        if buffer[i] != HEADER:
            idx = buffer.find(_HEADER_BYTE, i)
            if idx == -1:
                # No header found in rest of buffer
                i = n
                break
            i = idx
            if i + MIN_FRAME_HEADER > n:
                break

        # need at least HEADER + len_hi + len_lo
        if i + 3 > n:
            break

        len_hi = buffer[i + 1]
        len_lo = buffer[i + 2]
        length = (len_hi << 8) | len_lo  # length of [msg_type][payload...]

        # sanity check
        if length < 1 or length > _MAX_LEN:
            # bogus length -> treat as false header, resync
            i += 1
            continue

        # total frame size:
        #   HEADER(1) + len_hi(1) + len_lo(1) + body(length) + checksum(1)
        frame_total = 1 + 2 + length + 1

        if i + frame_total > n:
            # not enough data yet
            break

        # body = [msg_type][payload...]
        body_start = i + 3
        body_end   = body_start + length
        body = buffer[body_start:body_end]

        if not body:
            # should not happen if length >= 1, but be defensive
            i += 1
            continue

        msg_type = body[0]
        payload  = body[1:]

        recv_checksum = buffer[body_end]

        checksum = (length + msg_type + sum(payload)) & 0xFF

        if checksum == recv_checksum:
            # good frame
            on_frame(bytes(body))  # msg_type + payload
            i += frame_total
        else:
            # bad frame, skip this HEADER and resync
            i += 1

        n = len(buffer)

    if i > 0:
        del buffer[:i]
