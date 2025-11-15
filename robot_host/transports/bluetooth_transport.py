from __future__ import annotations
from robot_host.transports.serial_transport import SerialTransport


class BluetoothSerialTransport(SerialTransport):
    """
    Bluetooth Classic SPP transport, built on top of SerialTransport.
    From Python's perspective, it's still just a serial port.
    """

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        super().__init__(port, baudrate)
        # later you could add BT-specific init or helpers here
