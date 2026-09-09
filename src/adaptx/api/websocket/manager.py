"""WebSocket connection registry.

Tracks live telemetry subscribers so a future producer can broadcast to all of
them. Kept deliberately small: it holds connections and nothing else.
"""

from __future__ import annotations

import asyncio
from typing import Any

from fastapi import WebSocket

from adaptx.core.logging import get_logger

logger = get_logger(__name__)


class ConnectionManager:
    """Registry of connected telemetry clients."""

    def __init__(self, max_connections: int = 32) -> None:
        self._max_connections = max_connections
        self._connections: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    @property
    def connection_count(self) -> int:
        """Number of currently connected clients."""
        return len(self._connections)

    async def connect(self, websocket: WebSocket) -> bool:
        """Accept and register a client.

        Returns False without accepting when the connection limit is reached,
        letting the caller close with an explicit policy code.
        """
        async with self._lock:
            if len(self._connections) >= self._max_connections:
                return False
            await websocket.accept()
            self._connections.add(websocket)
        return True

    async def disconnect(self, websocket: WebSocket) -> None:
        """Deregister a client. Safe to call more than once."""
        async with self._lock:
            self._connections.discard(websocket)

    async def broadcast(self, message: dict[str, Any]) -> None:
        """Send ``message`` to every connected client, dropping dead ones."""
        async with self._lock:
            targets = list(self._connections)
        stale: list[WebSocket] = []
        for connection in targets:
            try:
                await connection.send_json(message)
            except Exception:  # A failed send means a dead peer.
                stale.append(connection)
        if stale:
            async with self._lock:
                for connection in stale:
                    self._connections.discard(connection)
            logger.debug(
                "dropped disconnected telemetry clients",
                extra={"context": {"dropped": len(stale)}},
            )
