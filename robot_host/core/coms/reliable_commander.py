# robot_host/core/reliable_commander.py

import asyncio
import time
from dataclasses import dataclass
from typing import Callable, Optional, Dict, Any, Awaitable
from enum import Enum


class CommandStatus(Enum):
    PENDING = "pending"
    ACKED = "acked"
    FAILED = "failed"
    TIMEOUT = "timeout"


@dataclass
class PendingCommand:
    seq: int
    cmd_type: str
    payload: Dict[str, Any]
    sent_at: float
    retries: int = 0
    future: Optional[asyncio.Future] = None


class ReliableCommander:
    """
    Tracks pending commands and handles retries on timeout.
    
    Async version that integrates with your existing client.
    """
    
    def __init__(
        self,
        send_func: Callable[[str, Dict[str, Any], Optional[int]], Awaitable[int]],
        timeout_s: float = 0.25,
        max_retries: int = 3,
       
    ):
        """
        Args:
            send_func: Async function that sends command and returns sequence number.
                       Signature: async send_func(cmd_type, payload) -> seq
            timeout_s: Time to wait for ack before retry
            max_retries: Number of retries before giving up
        """
        self.send_func = send_func
        self.timeout_s = timeout_s
        self.max_retries = max_retries
        
        self._pending: Dict[int, PendingCommand] = {}
        self._update_task: Optional[asyncio.Task] = None
        
        # Stats
        self.commands_sent = 0
        self.acks_received = 0
        self.timeouts = 0
        self.retries = 0
    
    async def send(
        self,
        cmd_type: str,
        payload: Optional[Dict[str, Any]] = None,
        wait_for_ack: bool = True,
    ) -> tuple[bool, Optional[str]]:
        """
        Send a command and optionally wait for ack.
        
        Args:
            cmd_type: Command type string (e.g., "SET_VEL")
            payload: Command payload dict
            wait_for_ack: If True, wait for ack and return result
            
        Returns:
            (success, error_msg) if wait_for_ack, else (True, None)
        """
        payload = payload or {}
        seq = await self.send_func(cmd_type, payload, None)
        
        if not wait_for_ack:
            self.commands_sent += 1
            return True, None
        
        future: asyncio.Future[tuple[bool, Optional[str]]] = asyncio.get_event_loop().create_future()
        
        self._pending[seq] = PendingCommand(
            seq=seq,
            cmd_type=cmd_type,
            payload=payload,
            sent_at=time.monotonic(),
            future=future,
        )
        self.commands_sent += 1
        
        try:
            return await future
        except asyncio.CancelledError:
            self._pending.pop(seq, None)
            return False, "CANCELLED"
    
    async def send_fire_and_forget(
        self,
        cmd_type: str,
        payload: Optional[Dict[str, Any]] = None,
    ) -> int:
        """Send without tracking. Use for heartbeats etc."""
        payload = payload or {}
        seq = await self.send_func(cmd_type, payload)
        self.commands_sent += 1
        return seq
    
    def on_ack(self, seq: int, ok: bool, error: Optional[str] = None) -> None:
        """Call when ack received from firmware."""
        cmd = self._pending.pop(seq, None)
        if cmd:
            self.acks_received += 1
            if cmd.future and not cmd.future.done():
                cmd.future.set_result((ok, error))
    
    async def start_update_loop(self, interval_s: float = 0.05) -> None:
        """Start background task to handle retries."""
        self._update_task = asyncio.create_task(self._update_loop(interval_s))
    
    async def stop_update_loop(self) -> None:
        """Stop background task."""
        if self._update_task:
            self._update_task.cancel()
            try:
                await self._update_task
            except asyncio.CancelledError:
                pass
            self._update_task = None
    
    async def _update_loop(self, interval_s: float) -> None:
        while True:
            await self._update()
            await asyncio.sleep(interval_s)
    
    async def _update(self) -> None:
        """Handle retries and timeouts."""
        now = time.monotonic()
        
        timed_out = []
        to_retry = []
        
        for seq, cmd in list(self._pending.items()):
            if now - cmd.sent_at > self.timeout_s:
                if cmd.retries < self.max_retries:
                    to_retry.append(cmd)
                else:
                    timed_out.append(seq)
        
        # Handle retries
        for cmd in to_retry:
            cmd.retries += 1
            cmd.sent_at = now
            self.retries += 1

            # resend with SAME seq so the ACK matches the pending entry
            await self.send_func(cmd.cmd_type, cmd.payload, cmd.seq)
        
        # Handle final timeouts
        for seq in timed_out:
            cmd = self._pending.pop(seq)
            self.timeouts += 1
            if cmd.future and not cmd.future.done():
                cmd.future.set_result((False, "TIMEOUT"))
    
    def pending_count(self) -> int:
        return len(self._pending)
    
    def clear_pending(self) -> None:
        """Clear all pending commands (e.g., on disconnect)."""
        for cmd in self._pending.values():
            if cmd.future and not cmd.future.done():
                cmd.future.set_result((False, "CLEARED"))
        self._pending.clear()
    
    def stats(self) -> Dict[str, int]:
        return {
            "commands_sent": self.commands_sent,
            "acks_received": self.acks_received,
            "timeouts": self.timeouts,
            "retries": self.retries,
            "pending": self.pending_count(),
        }