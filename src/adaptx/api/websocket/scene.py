"""Live scene channel (Phase 12).

``/ws/scene`` pushes the latest :class:`~adaptx.models.scene.SceneSnapshot`
whenever the scene service's sequence changes. It is a push of frame geometry
- tracks, paths, assessments, tiles and a point sample - and is kept apart
from ``/ws/telemetry``, which stays the compact status channel it always was.

Message envelope, the same shape as telemetry::

    {"type": "hello" | "scene", "sequence": <int>, "timestamp": <ISO-8601>, "data": {...}}

A ``hello`` carries the current sequence and whether a scene exists; a
``scene`` carries the snapshot. The channel polls the sequence at a short,
fixed interval rather than being woken by the producer, because the
producers are synchronous request handlers and this keeps the coupling to a
single integer (ADR-055).
"""

from __future__ import annotations

import asyncio
from contextlib import suppress
from typing import Any

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from starlette.websockets import WebSocketState

from adaptx.core.lifecycle import ApplicationContext
from adaptx.core.logging import get_logger
from adaptx.models.common import utc_now

logger = get_logger(__name__)

router = APIRouter()

#: How often the channel checks for a new snapshot. Twenty-five checks a
#: second bounds the push latency at 40 ms; a scenario ticks at 20 Hz.
POLL_INTERVAL_S = 0.04

_POLICY_VIOLATION = 1008


def _envelope(message_type: str, sequence: int, data: dict[str, Any]) -> dict[str, Any]:
    return {
        "type": message_type,
        "sequence": sequence,
        "timestamp": utc_now().isoformat(),
        "data": data,
    }


@router.websocket("/ws/scene")
async def scene_channel(websocket: WebSocket) -> None:
    """Push the latest scene to a dashboard client whenever it changes."""
    app = websocket.app
    context: ApplicationContext = app.state.context
    manager = app.state.connections

    if not await manager.connect(websocket):
        await websocket.close(code=_POLICY_VIOLATION, reason="connection limit reached")
        return

    try:
        sequence, latest = context.scene.latest()
        await websocket.send_json(
            _envelope(
                "hello",
                sequence,
                {"has_scene": latest is not None, "poll_interval_s": POLL_INTERVAL_S},
            )
        )
        if latest is not None:
            current = _envelope("scene", sequence, latest.model_dump(mode="json"))
            current["skipped"] = 0
            await websocket.send_json(current)
        sent = sequence
        while True:
            await asyncio.sleep(POLL_INTERVAL_S)
            sequence, latest = context.scene.latest()
            if sequence == sent:
                continue
            # Latest-only delivery: whatever was published between two polls
            # is superseded, and the client is told how many frames that was
            # rather than left to believe it saw every one.
            skipped = max(0, sequence - sent - 1)
            sent = sequence
            if latest is None:
                await websocket.send_json(_envelope("cleared", sequence, {"has_scene": False}))
                continue
            envelope = _envelope("scene", sequence, latest.model_dump(mode="json"))
            envelope["skipped"] = skipped
            await websocket.send_json(envelope)
    except WebSocketDisconnect:
        logger.debug("scene client disconnected")
    finally:
        await manager.disconnect(websocket)
        if websocket.application_state is WebSocketState.CONNECTED:
            with suppress(RuntimeError):
                await websocket.close()
