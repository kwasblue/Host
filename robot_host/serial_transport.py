from __future__ import annotations
from typing import Optional

import serial

from robot_host.stream_transport import StreamTransport


class SerialTransport(StreamTransport):
    """
    Generic serial transport for USB/UART or Bluetooth-Serial devices.
    'port' is something like:
      - macOS USB:   /dev/tty.usbserial-XXXX  or  /dev/tty.usbmodemXXXX
      - macOS BT:    /dev/tty.YOUR_BT_DEVICE-XXX
      - Linux USB:   /dev/ttyUSB0, /dev/ttyACM0
      - Linux BT:    /dev/rfcomm0
      - Windows:     COM3, COM5, ...
    """

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self._ser: Optional[serial.Serial] = None

    def _open(self) -> None:
        self._ser = serial.Serial(self.port, self.baudrate, timeout=0.05)

    def _close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    def _read_raw(self, n: int) -> bytes:
        if not self._ser:
            return b""
        return self._ser.read(n)

    def _send_bytes(self, data: bytes) -> None:
        if not self._ser or not self._ser.is_open:
            raise RuntimeError("Serial not open")
        self._ser.write(data)
