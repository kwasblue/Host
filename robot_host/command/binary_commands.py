# core/binary_commands.py
"""
Binary command encoder for high-rate streaming.

Use binary commands for real-time control loops (50+ Hz).
Use JSON commands for setup/configuration.

Binary format is ~10x smaller than equivalent JSON:
  SET_VEL binary: 9 bytes
  SET_VEL JSON:   ~50 bytes

Example:
    from robot_host.command.binary_commands import BinaryStreamer
    from robot_host.core.protocol import encode, MSG_CMD_BIN

    streamer = BinaryStreamer()

    # Send velocity command
    payload = streamer.encode_set_vel(vx=0.2, omega=0.1)
    frame = encode(MSG_CMD_BIN, payload)
    transport.send(frame)

    # Send multiple signals
    payload = streamer.encode_set_signals([
        (100, 1.5),  # signal_id=100, value=1.5
        (101, 0.0),  # signal_id=101, value=0.0
    ])
    frame = encode(MSG_CMD_BIN, payload)
    transport.send(frame)
"""

from __future__ import annotations

import struct
from typing import List, Tuple


class Opcode:
    """Binary command opcodes (must match BinaryCommands.h on MCU)."""
    SET_VEL     = 0x10  # Set velocity: vx(f32), omega(f32)
    SET_SIGNAL  = 0x11  # Set signal: id(u16), value(f32)
    SET_SIGNALS = 0x12  # Set multiple signals: count(u8), [id(u16), value(f32)]*
    HEARTBEAT   = 0x20  # Heartbeat (no payload)
    STOP        = 0x21  # Emergency stop (no payload)


class BinaryStreamer:
    """
    Encodes binary commands for high-rate streaming.

    All multi-byte values are little-endian to match ESP32.
    """

    def encode_set_vel(self, vx: float, omega: float) -> bytes:
        """
        Encode SET_VEL command.

        Args:
            vx: Linear velocity (m/s)
            omega: Angular velocity (rad/s)

        Returns:
            Binary payload (9 bytes): opcode + vx(f32) + omega(f32)
        """
        return struct.pack("<Bff", Opcode.SET_VEL, vx, omega)

    def encode_set_signal(self, signal_id: int, value: float) -> bytes:
        """
        Encode SET_SIGNAL command for a single signal.

        Args:
            signal_id: Signal ID (0-65535)
            value: Signal value

        Returns:
            Binary payload (7 bytes): opcode + id(u16) + value(f32)
        """
        return struct.pack("<BHf", Opcode.SET_SIGNAL, signal_id, value)

    def encode_set_signals(self, signals: List[Tuple[int, float]]) -> bytes:
        """
        Encode SET_SIGNALS command for multiple signals.

        Args:
            signals: List of (signal_id, value) tuples

        Returns:
            Binary payload: opcode + count(u8) + [id(u16) + value(f32)] * count
        """
        count = min(len(signals), 255)  # Max 255 signals per packet
        data = struct.pack("<BB", Opcode.SET_SIGNALS, count)
        for i in range(count):
            signal_id, value = signals[i]
            data += struct.pack("<Hf", signal_id, value)
        return data

    def encode_heartbeat(self) -> bytes:
        """Encode HEARTBEAT command (1 byte)."""
        return struct.pack("<B", Opcode.HEARTBEAT)

    def encode_stop(self) -> bytes:
        """Encode STOP command (1 byte)."""
        return struct.pack("<B", Opcode.STOP)


__all__ = ["Opcode", "BinaryStreamer"]
