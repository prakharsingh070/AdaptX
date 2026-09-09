"""LiDAR frame ingestion endpoint."""

from __future__ import annotations

from fastapi import APIRouter, status

from adaptx.api.dependencies import LiDARServiceDep
from adaptx.api.schemas import ErrorResponse, LiDARFrameRequest, LiDARFrameResponse
from adaptx.core.exceptions import InvalidPointCloudError

router = APIRouter(prefix="/lidar", tags=["lidar"])


@router.post(
    "/frame",
    response_model=LiDARFrameResponse,
    status_code=status.HTTP_202_ACCEPTED,
    responses={422: {"model": ErrorResponse, "description": "Malformed point cloud"}},
    summary="Submit a point-cloud frame",
)
def submit_frame(payload: LiDARFrameRequest, service: LiDARServiceDep) -> LiDARFrameResponse:
    """Validate a frame, account for it and return its summary.

    Phase 1 performs structural validation, point counting and bounds only. The
    frame is not filtered, detected on, tracked or mapped; those modules do not
    exist yet.

    Returns 422 when the point array is ragged, non-numeric, wrongly shaped, or
    outside the configured point-count limits.
    """
    try:
        frame = payload.to_frame()
    except ValueError as exc:
        raise InvalidPointCloudError(str(exc)) from exc

    summary = service.ingest(frame)
    return LiDARFrameResponse(accepted=True, summary=summary)
