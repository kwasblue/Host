# robot_host/transports/stream_transport.py
from __future__ import annotations
import asyncio
import threading
import time
from abc import ABC, abstractmethod
from typing import Optional

from robot_host.core import protocol
from robot_host.transports.base_transport import BaseTransport


class StreamTransport(BaseTransport, ABC):
    """
    Base for transports that expose a byte stream (read/write bytes).
    Handles:
      - background reader thread
      - rx buffer
      - protocol.extract_frames()

    Subclasses must implement:
      - _open()
      - _close()
      - _read_raw(n) -> bytes
      - _send_bytes(data: bytes) -> None
    """

    def __init__(self) -> None:
        super().__init__()
        self._rx_buffer = bytearray()
        self._stop = False
        self._thread: Optional[threading.Thread] = None

    # ---- subclass hooks ----

    @abstractmethod
    def _open(self) -> None:
        ...

    @abstractmethod
    def _close(self) -> None:
        ...

    @abstractmethod
    def _read_raw(self, n: int) -> bytes:
        ...

    @abstractmethod
    def _send_bytes(self, data: bytes) -> None:
        """
        Blocking write of raw bytes to the underlying stream.
        Implemented by SerialTransport, TcpTransport, etc.
        """
        ...

    # ---- sync lifecycle ----

    def start(self) -> None:
        """
        Open the stream and start the background reader thread.
        AsyncRobotClient.start() will call this; it handles sync/async.
        """
        self._open()
        self._stop = False
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """
        Stop the background reader and close the stream.
        """
        self._stop = True
        if self._thread:
            self._thread.join(timeout=1.0)
        self._close()

    # ---- background reader ----

    def _reader_loop(self) -> None:
        while not self._stop:
            try:
                data = self._read_raw(256)
                if data:
                    self._rx_buffer.extend(data)
                    protocol.extract_frames(
                        self._rx_buffer,
                        lambda body: self._handle_body(body),
                    )
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"[StreamTransport] error: {e}")
                time.sleep(0.5)

    # ---- async-friendly write API ----

    async def send_bytes(self, data: bytes) -> None:
        """
        Async-friendly wrapper so AsyncRobotClient can `await transport.send_bytes(...)`.
        Offloads the (potentially blocking) _send_bytes() call to a thread
        so we don't block the asyncio event loop.
        """
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._send_bytes, data)
