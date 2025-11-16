import asyncio
from typing import Callable, Optional
from robot_host.core import protocol


class AsyncTcpTransport:
    """
    Async TCP transport:
      - Maintains a connection to (host, port)
      - Reconnects on failure
      - Reads BYTES from the socket, then uses protocol.extract_frames()
        to turn them into framed messages.
      - Calls a frame handler with `body` where:
            body[0] = msg_type
            body[1:] = payload

      - Also exposes send_bytes() so higher-level code (AsyncRobotClient)
        can send fully-encoded frames directly.
    """

    def __init__(
        self,
        host: str,
        port: int,
        reconnect_delay: float = 5.0,
    ) -> None:
        self.host = host
        self.port = port
        self.reconnect_delay = reconnect_delay

        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        self._running: bool = False

        # Frame handler: gets "body" (msg_type + payload), as produced by protocol.extract_frames
        self._frame_handler: Callable[[bytes], None] = lambda frame: None

        self._task: Optional[asyncio.Task] = None
        self._rx_buffer = bytearray()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_frame_handler(self, handler: Callable[[bytes], None]) -> None:
        """
        RobotClient will register its callback here.

        handler(body: bytes):
            body[0] = msg_type (int 0–255)
            body[1:] = payload bytes
        """
        self._frame_handler = handler

    async def start(self) -> None:
        """Start connection/reconnect loop in the background."""
        if self._task is None:
            self._running = True
            self._task = asyncio.create_task(self._run())

    async def stop(self) -> None:
        """Stop background loop and close the socket."""
        self._running = False

        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None

        if self._writer:
            self._writer.close()
            try:
                await self._writer.wait_closed()
            except Exception:
                pass
            self._writer = None
            self._reader = None

    async def send_frame(self, msg_type: int, payload: bytes = b"") -> None:
        """
        Encode a frame using your existing protocol and send it.
        Typically used if you want the transport to handle framing.
        """
        if not self._writer:
            print("[TcpTransport] send_frame called while not connected")
            return

        frame = protocol.encode(msg_type, payload)
        self._writer.write(frame)
        await self._writer.drain()

    async def send_bytes(self, data: bytes) -> None:
        """
        Send already-encoded bytes over the socket.

        This is what AsyncRobotClient expects to call, since it often
        builds the full frame itself (e.g. protocol.encode_json_cmd(...)).
        """
        if not self._writer:
            print("[TcpTransport] send_bytes called while not connected")
            return

        self._writer.write(data)
        await self._writer.drain()

    # Optional alias if you ever called `send()` elsewhere
    async def send(self, data: bytes) -> None:
        await self.send_bytes(data)

    # ------------------------------------------------------------------
    # Internal async loop
    # ------------------------------------------------------------------

    async def _run(self) -> None:
        """Main reconnect + read loop."""
        while self._running:
            try:
                print(f"[TcpTransport] Connecting to {self.host}:{self.port} ...")
                self._reader, self._writer = await asyncio.open_connection(
                    self.host, self.port
                )
                print("[TcpTransport] Connected")

                # Clear buffer on (re)connect
                self._rx_buffer.clear()

                # Read loop
                while self._running:
                    data = await self._reader.read(1024)
                    if not data:
                        print("[TcpTransport] Connection closed by peer")
                        break

                    # Accumulate and let protocol.extract_frames parse frames.
                    self._rx_buffer.extend(data)

                    # This will call self._frame_handler(body) for each frame found.
                    protocol.extract_frames(self._rx_buffer, self._frame_handler)

            except Exception as e:
                print(f"[TcpTransport] Error: {e}")

            # Cleanup and reconnect delay
            if self._writer:
                self._writer.close()
                try:
                    await self._writer.wait_closed()
                except Exception:
                    pass
                self._writer = None
                self._reader = None

            if self._running:
                print(f"[TcpTransport] Reconnecting in {self.reconnect_delay}s ...")
                await asyncio.sleep(self.reconnect_delay)
