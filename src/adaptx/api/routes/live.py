"""Live simulation session endpoints (post-Phase-12 extension).

High-level session controls only: start a catalogue scenario, pause, resume,
stop, reset. There is deliberately **no** endpoint that spawns or destroys
an actor, moves one, ticks the world or applies a control to the ego; those
belong to the scenario layer and the controller inside the process, and the
route audit in ``tests/integration/test_dashboard_api.py`` allows exactly
this list (ADR-056). The controls can be switched off with
``ADAPTX_LIVE__CONTROLS_ENABLED=false``; the status, scenario list, events
and camera reads stay available.
"""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, Query, Response, status

from adaptx.api.dependencies import ContextDep
from adaptx.api.schemas import ErrorResponse
from adaptx.live.models import LiveEvent, LiveScenarioSummary, LiveStatus
from adaptx.live.scenarios import summaries
from adaptx.models.common import AdaptXModel

router = APIRouter(prefix="/live", tags=["live"])

_CONTROL_ERRORS: dict[int | str, dict[str, Any]] = {
    409: {"model": ErrorResponse, "description": "The session is not in a state for this"},
    503: {"model": ErrorResponse, "description": "CARLA is disabled or unreachable"},
}


class LiveStartRequest(AdaptXModel):
    """Which catalogue scenario to run, and with which seed."""

    scenario_id: str | None = None
    seed: int | None = None


class LiveEventList(AdaptXModel):
    """Events newer than a sequence, oldest first."""

    events: list[LiveEvent]
    latest_sequence: int


@router.get("/status", response_model=LiveStatus, summary="Live session state")
def live_status(context: ContextDep) -> LiveStatus:
    """The session as it is now: state, scenario, seed, ego, control, timing."""
    return context.live.status()


@router.get(
    "/scenarios", response_model=list[LiveScenarioSummary], summary="Live scenario catalogue"
)
def live_scenarios() -> list[LiveScenarioSummary]:
    """The scenarios a live session can run; each has a default seed."""
    return summaries()


@router.get("/events", response_model=LiveEventList, summary="Recent live events")
def live_events(
    context: ContextDep,
    since: int = Query(default=0, ge=0, description="Return events after this sequence."),
    limit: int = Query(default=50, ge=1, le=500),
) -> LiveEventList:
    """What the loop noticed: detections, lost tracks, risk changes, control, collisions."""
    events = context.live.events(since=since, limit=limit)
    return LiveEventList(events=events, latest_sequence=context.live.status().event_sequence)


@router.get(
    "/camera",
    summary="Latest ego camera frame (PNG)",
    responses={404: {"description": "No camera frame is available"}},
    response_class=Response,
)
def live_camera(context: ContextDep) -> Response:
    """The forward RGB camera's newest frame. Visualisation only."""
    png = context.live.camera_png()
    if png is None:
        return Response(status_code=404, content=b"", media_type="image/png")
    return Response(content=png, media_type="image/png", headers={"Cache-Control": "no-store"})


@router.post(
    "/start",
    response_model=LiveStatus,
    status_code=status.HTTP_202_ACCEPTED,
    responses=_CONTROL_ERRORS,
    summary="Start a live scenario",
)
def live_start(request: LiveStartRequest, context: ContextDep) -> LiveStatus:
    """Open a CARLA session, spawn the scenario and run the loop until stopped."""
    return context.live.start(request.scenario_id, request.seed)


@router.post("/pause", response_model=LiveStatus, responses=_CONTROL_ERRORS, summary="Pause")
def live_pause(context: ContextDep) -> LiveStatus:
    """Stop ticking the simulator; the world freezes until resumed."""
    return context.live.pause()


@router.post("/resume", response_model=LiveStatus, responses=_CONTROL_ERRORS, summary="Resume")
def live_resume(context: ContextDep) -> LiveStatus:
    """Continue ticking after a pause."""
    return context.live.resume()


@router.post("/stop", response_model=LiveStatus, responses=_CONTROL_ERRORS, summary="Stop")
def live_stop(context: ContextDep) -> LiveStatus:
    """End the session: every actor destroyed, world settings restored."""
    return context.live.stop()


@router.post("/reset", response_model=LiveStatus, responses=_CONTROL_ERRORS, summary="Reset")
def live_reset(context: ContextDep) -> LiveStatus:
    """Stop and start the same scenario with the same seed."""
    return context.live.reset()
