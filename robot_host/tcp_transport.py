# host/robot_host/tcp_transport.py
from __future__ import annotations
import socket
from typing import Optional

from stream_transport import StreamTransport


class TcpTransport(StreamTransport):
    """
    TCP transport built on top of StreamTransport.

    ESP32 runs a TCP server, Python connects as a client:
      - host: IP address of ESP32 (e.g. "192.168.4.1" in AP mode or LAN IP)
      - port: TCP port (must match WifiTransport on MCU side)
    """

    def __init__(self, host: str, port: int) -> None:
        super().__init__()
        self.host = host
        self.port = port

        self._sock: Optional[socket.socket] = None

    # === StreamTransport hooks ===

    def _open(self) -> None:
        # Create a blocking socket; StreamTransport's _reader_loop
        # will handle timeouts by reading small chunks repeatedly.
        self._sock = socket.create_connection((self.host, self.port))
        # Optional: small timeout to avoid hanging forever in recv
        self._sock.settimeout(0.1)

    def _close(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None

    def _read_raw(self, n: int) -> bytes:
        if not self._sock:
            return b""
        try:
            data = self._sock.recv(n)
            # If remote closed, recv() returns b""
            return data
        except socket.timeout:
            return b""
        except BlockingIOError:
            return b""
        except OSError as e:
            print(f"[TcpTransport] recv error: {e}")
            return b""

    def _send_bytes(self, data: bytes) -> None:
        if not self._sock:
            raise RuntimeError("TCP socket not connected")
        try:
            self._sock.sendall(data)
        except OSError as e:
            print(f"[TcpTransport] send error: {e}")
            raise
