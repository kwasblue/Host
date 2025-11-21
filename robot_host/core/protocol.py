from typing import Callable

HEADER = 0xAA

MSG_HEARTBEAT = 0x01
MSG_PING      = 0x02
MSG_PONG      = 0x03
MSG_CMD_JSON  = 0x50
MSG_WHOAMI    = 0x10

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

    frame = bytearray()
    frame.append(HEADER)
    frame.append(len_hi)
    frame.append(len_lo)
    frame.append(msg_type)
    frame.extend(payload)
    frame.append(checksum)
    return bytes(frame)


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
        # search for HEADER
        if buffer[i] != HEADER:
            i += 1
            continue

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
