import threading
import time
from typing import Callable, Optional
import serial
import protocol


class SerialTransport:
    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self.port = port
        self.baudrate = baudrate

        self._ser: Optional[serial.Serial] = None
        self._on_frame: Optional[Callable[[bytes], None]] = None

        self._rx_buffer = bytearray()
        self._stop = False
        self._thread: Optional[threading.Thread] = None

    def set_frame_handler(self, handler: Callable[[bytes], None]) -> None:
        self._on_frame = handler

    def start(self) -> None:
        self._ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
        self._stop = False
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop = True
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._ser and self._ser.is_open:
            self._ser.close()

    def _reader_loop(self) -> None:
        assert self._ser is not None
        while not self._stop:
            try:
                data = self._ser.read(256)
                if data:
                    self._rx_buffer.extend(data)
                    if self._on_frame:
                        protocol.extract_frames(
                            self._rx_buffer,
                            lambda body: self._on_frame(body),
                        )
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"[SerialTransport] error: {e}")
                time.sleep(0.5)

    def send_frame(self, msg_type: int, payload: bytes = b"") -> None:
        if not self._ser or not self._ser.is_open:
            raise RuntimeError("Serial not open")
        frame = protocol.encode(msg_type, payload)
        self._ser.write(frame)
