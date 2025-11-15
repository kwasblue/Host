# host/robot_host/tcp_transport.py
import socket
import threading
import time
from typing import Callable, Optional

from . import protocol


class TcpTransport:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port

        self._sock: Optional[socket.socket] = None
        self._on_frame: Optional[Callable[[bytes], None]] = None
        self._rx_buffer = bytearray()
        self._stop = False
        self._thread: Optional[threading.Thread] = None

    def set_frame_handler(self, handler: Callable[[bytes], None]) -> None:
        self._on_frame = handler

    def start(self) -> None:
        self._sock = socket.create_connection((self.host, self.port))
        self._stop = False
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop = True
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass

    def _reader_loop(self) -> None:
        assert self._sock is not None
        sock = self._sock
        sock.settimeout(0.1)
        while not self._stop:
            try:
                data = sock.recv(256)
                if not data:
                    time.sleep(0.05)
                    continue
                self._rx_buffer.extend(data)
                if self._on_frame:
                    protocol.extract_frames(
                        self._rx_buffer,
                        lambda body: self._on_frame(body),
                    )
            except (socket.timeout, BlockingIOError):
                continue
            except OSError as e:
                print(f"[TcpTransport] error: {e}")
                time.sleep(0.5)

    def send_frame(self, msg_type: int, payload: bytes = b"") -> None:
        if not self._sock:
            raise RuntimeError("TCP socket not connected")
        frame = protocol.encode(msg_type, payload)
        self._sock.sendall(frame)
