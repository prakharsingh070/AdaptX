"""Real-time telemetry channel.

``/ws/telemetry`` is the foundation of the dashboard's live feed. It carries
only what the backend actually knows: the aggregated system status, the
measured runtime metrics, and **summaries** of detection, tracking,
prediction, mapping, risk and adaptive mapping. The payload's ``provides`` and
``not_yet_available`` fields say what exists, and a stream is removed from
``not_yet_available`` only once something genuinely produces it.

The tracking summary carries counts, track identifiers and configuration -
not per-track history. The prediction summary follows the same rule: counts,
horizon, model and the ids that received a trajectory, never the trajectory
points themselves. Mapping follows it too: dimensions, counts and an
occupancy ratio, never the grid - a 0.5 m map over the default bounds is
57,600 cells. Risk carries scene counts and a handful of the most concerning
objects, never every assessment. Adaptive mapping follows it too: the region
count, the level distribution and how many regions changed, never the tiles -
a single region at the finest level holds ten thousand cells. Full
trajectories, map cells, assessments and region decisions are returned by
``POST /api/v1/lidar/predict``, ``/map``, ``/risk`` and ``/adaptive-map``;
putting them on every tick would push frame geometry down a status channel.

Message envelope::

    {
      "type": "hello" | "telemetry",
      "sequence": <int>,
      "timestamp": "<ISO-8601 UTC>",
      "data": { ... }
    }
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

#: Close code used when the connection limit is reached (policy violation).
_POLICY_VIOLATION = 1008

#: Streams the dashboard will eventually consume but which produce nothing yet.
_NOT_YET_AVAILABLE = [
    # Object-level risk exists (Phase 7) and is summarised as "risk". What does
    # not exist is a spatial risk *field*: no per-cell risk formulation is
    # implemented, so that stream stays listed as unavailable.
    #
    # "adaptive_map" was removed in Phase 8: a resolution controller now
    # allocates detail by region, and the stream is summarised as
    # "adaptive_mapping". Note what is summarised and what is not - the region
    # decisions and level distribution travel here; the tiles themselves never
    # do.
    "risk_field",
]


def _envelope(message_type: str, sequence: int, data: dict[str, Any]) -> dict[str, Any]:
    return {
        "type": message_type,
        "sequence": sequence,
        "timestamp": utc_now().isoformat(),
        "data": data,
    }


def build_telemetry_payload(context: ApplicationContext) -> dict[str, Any]:
    """Assemble one telemetry payload from live backend state."""
    status = context.system.status()
    metrics = context.metrics.snapshot()
    return {
        "system": status.model_dump(mode="json"),
        "metrics": metrics.model_dump(mode="json"),
        "detection": _detection_summary(context),
        "tracking": context.tracking.summary(),
        "prediction": context.prediction.summary(),
        "mapping": context.mapping.summary(),
        "risk": context.risk.summary(),
        "adaptive_mapping": context.adaptive_mapping.summary(),
        "provides": [
            "system",
            "metrics",
            "detection",
            "tracking",
            "prediction",
            "mapping",
            "risk",
            "adaptive_mapping",
        ],
        "not_yet_available": _NOT_YET_AVAILABLE,
    }


def _detection_summary(context: ApplicationContext) -> dict[str, Any]:
    """What the detector is configured to do, without any point data.

    Deliberately a summary: streaming detections would mean streaming whatever
    the last frame produced, and the telemetry channel is not the place to push
    per-frame geometry - let alone raw point arrays, which would swamp it.
    Detections are returned by ``POST /api/v1/lidar/detect`` instead.
    """
    detector = context.detector
    return {
        "detector": detector.name,
        "is_baseline": detector.is_baseline,
        "classifier": "geometric_bands_v1",
        "configuration": detector.configuration.model_dump(mode="json"),
        "note": (
            "Detection runs per request, not continuously; this channel carries "
            "no per-frame object data."
        ),
    }


@router.websocket("/ws/telemetry")
async def telemetry(websocket: WebSocket) -> None:
    """Stream backend status and measured metrics to a dashboard client.

    The client receives a ``hello`` message on connect, then a ``telemetry``
    message every ``ADAPTX_WEBSOCKET__TELEMETRY_INTERVAL_S`` seconds.
    """
    app = websocket.app
    context: ApplicationContext = app.state.context
    manager = app.state.connections
    interval = context.settings.websocket.telemetry_interval_s

    if not await manager.connect(websocket):
        await websocket.close(code=_POLICY_VIOLATION, reason="telemetry connection limit reached")
        return

    sequence = 0
    try:
        await websocket.send_json(
            _envelope(
                "hello",
                sequence,
                {
                    "name": context.settings.app.name,
                    "environment": context.settings.app.environment.value,
                    "interval_s": interval,
                    "provides": ["system", "metrics"],
                    "not_yet_available": _NOT_YET_AVAILABLE,
                },
            )
        )
        while True:
            sequence += 1
            await websocket.send_json(
                _envelope("telemetry", sequence, build_telemetry_payload(context))
            )
            await asyncio.sleep(interval)
    except WebSocketDisconnect:
        logger.debug("telemetry client disconnected")
    finally:
        await manager.disconnect(websocket)
        # Only close when this side has not already sent a close frame.
        # `application_state` tracks what the server has sent; `client_state`
        # does not, and closing twice raises inside Starlette.
        if websocket.application_state is WebSocketState.CONNECTED:
            with suppress(RuntimeError):
                await websocket.close()
