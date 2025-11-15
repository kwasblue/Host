from typing import Callable

HEADER = 0xAA

MSG_HEARTBEAT = 0x01
MSG_PING = 0x02
MSG_PONG = 0x03
MSG_CMD_JSON  = 0x50


def encode(msg_type: int, payload: bytes = b"") -> bytes:
    """
    Encode a frame as:
    [HEADER][len][msg_type][payload...][checksum]
    where len = 1 + payload_len, checksum = (len + msg_type + sum(payload)) & 0xFF
    """
    length = 1 + len(payload)  # msg_type + payload
    checksum = (length + msg_type + sum(payload)) & 0xFF

    frame = bytearray()
    frame.append(HEADER)
    frame.append(length)
    frame.append(msg_type)
    frame.extend(payload)
    frame.append(checksum)
    return bytes(frame)


def extract_frames(buffer: bytearray, on_frame: Callable[[bytes], None]) -> None:
    """
    Parse as many frames as possible from buffer.
    Calls on_frame(body) where body[0] = msg_type, body[1:] = payload.
    Mutates buffer, removing consumed bytes.
    """
    i = 0
    while i + 3 <= len(buffer):
        if buffer[i] != HEADER:
            i += 1
            continue

        if i + 2 >= len(buffer):
            break
        length = buffer[i + 1]
        frame_total = 2 + length + 1  # header + len + [len bytes] + checksum

        if i + frame_total > len(buffer):
            break

        body = buffer[i + 2 : i + 2 + length]
        msg_type = body[0]
        payload = body[1:]
        recv_checksum = buffer[i + 2 + length]

        checksum = (length + msg_type + sum(payload)) & 0xFF

        if checksum == recv_checksum:
            on_frame(bytes(body))  # msg_type + payload
            i += frame_total
        else:
            # bad frame, skip this header and resync
            i += 1

    if i > 0:
        del buffer[:i]
