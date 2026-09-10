"""Tracking endpoints (Phase 4).

Tracking is **stateful**: a track exists because of what earlier frames
contained, so these endpoints behave differently from the stateless processing
and detection routes.

These are the tracking *state* operations. Frame-level tracking lives on the
LiDAR router as ``POST /api/v1/lidar/track``, because it consumes a point cloud
exactly as the other LiDAR endpoints do.
"""

from __future__ import annotations

from fastapi import APIRouter, status

from adaptx.api.dependencies import ContextDep
from adaptx.api.schemas import TrackingResetResponse, TrackingStatusResponse

router = APIRouter(prefix="/tracking", tags=["tracking"])


@router.post(
    "/reset",
    response_model=TrackingResetResponse,
    status_code=status.HTTP_200_OK,
    summary="Drop all tracks and start again",
)
def reset_tracking(context: ContextDep) -> TrackingResetResponse:
    """Clear every track and restart identifier allocation.

    Call this between unrelated sequences. Without it, the first frame of a new
    scene is associated against tracks from the old one, and objects that
    happen to fall inside the association gate inherit the wrong identity.
    """
    cleared = context.tracking.live_track_count
    context.tracking.reset()
    return TrackingResetResponse(cleared_track_count=cleared)


@router.get(
    "/status",
    response_model=TrackingStatusResponse,
    status_code=status.HTTP_200_OK,
    summary="Current tracking state",
)
def tracking_status(context: ContextDep) -> TrackingStatusResponse:
    """Report how many frames have been tracked and what is currently alive."""
    return TrackingStatusResponse(summary=context.tracking.summary())
