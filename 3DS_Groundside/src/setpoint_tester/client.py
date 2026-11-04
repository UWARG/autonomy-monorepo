"""Blocking TCP client for the airside setpoint link."""

from __future__ import annotations

import select
import socket
from typing import Any, BinaryIO

from src.setpoint_tester.protocol import Reply, encode, parse_reply

# Replies are short; this only bounds memory if the peer misbehaves.
MAX_REPLY_BYTES = 64 * 1024


class LinkError(Exception):
    """The command could not be confirmed. The message says whether it may still
    have reached the drone."""


class SetpointClient:
    """Sends commands to the drone and waits for each reply.

    Connects on first use and reconnects automatically on the next command after
    the link drops. After any failure the connection is closed, so a late reply
    can never be mistaken for the answer to a later command.
    """

    def __init__(self, host: str, port: int, timeout: float = 3.0) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self._sock: socket.socket | None = None
        self._reader: BinaryIO | None = None

    @property
    def address(self) -> str:
        return f"{self.host}:{self.port}"

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def connect(self) -> None:
        """(Re)connect to the drone. Raises LinkError if it is unreachable."""
        self.close()
        try:
            self._sock = socket.create_connection(
                (self.host, self.port), timeout=self.timeout
            )
        except OSError as error:
            raise LinkError(f"cannot connect to {self.address}: {error}") from None
        self._reader = self._sock.makefile("rb")

    def close(self) -> None:
        if self._reader is not None:
            self._reader.close()
            self._reader = None
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def send(self, command: dict[str, Any]) -> Reply:
        """Send one command and return the drone's reply.

        Raises LinkError if the reply cannot be confirmed.
        """
        data = encode(command)
        if self._sock is None or self._peer_has_closed():
            self.connect()
        assert self._sock is not None and self._reader is not None

        try:
            self._sock.sendall(data)
        except OSError as error:
            self.close()
            raise LinkError(
                f"connection lost while sending ({error}); command probably NOT "
                "received"
            ) from None

        try:
            line = self._reader.readline(MAX_REPLY_BYTES)
        except socket.timeout:
            self.close()
            raise LinkError(
                f"no reply within {self.timeout:g}s; the command MAY have been "
                "received"
            ) from None
        except OSError as error:
            self.close()
            raise LinkError(
                f"connection lost waiting for the reply ({error}); the command MAY "
                "have been received"
            ) from None

        if not line.endswith(b"\n"):
            self.close()
            if len(line) >= MAX_REPLY_BYTES:
                raise LinkError("reply too long; the command MAY have been received")
            raise LinkError(
                "connection closed before the reply; the command MAY have been "
                "received"
            )
        try:
            return parse_reply(line)
        except (ValueError, TypeError) as error:
            self.close()
            raise LinkError(str(error)) from None

    def _peer_has_closed(self) -> bool:
        """Whether the drone has closed or reset the connection since the last
        command (e.g. because the airside node restarted).

        The drone only ever sends replies, so a connection that is readable
        between commands is at EOF, reset, or out of sync: all reasons to
        reconnect before sending.
        """
        assert self._sock is not None
        try:
            readable, _, _ = select.select([self._sock], [], [], 0)
        except (OSError, ValueError):
            return True
        return bool(readable)
