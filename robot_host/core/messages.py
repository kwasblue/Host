from enum import IntEnum

class MsgType(IntEnum):
    PING      = 0x01
    PONG      = 0x02
    HEARTBEAT = 0x03
    WHOAMI    = 0x10   # host -> robot
    HELLO     = 0x11   # robot -> host
