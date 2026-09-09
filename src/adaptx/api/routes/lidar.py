"""LiDAR frame ingestion endpoint."""

from __future__ import annotations

from fastapi import APIRouter, status

from adaptx.api.dependencies import ContextDep, LiDARServiceDep
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
def submit_frame(
    payload: LiDARFrameRequest, service: LiDARServiceDep, context: ContextDep
) -> LiDARFrameResponse:
    """Validate a frame, account for it and return its summary.

    With ``preprocess: false`` (the default) the behaviour is unchanged: the
    frame is validated structurally and against the configured point-count
    limits, and any non-finite value is rejected with 422.

    With ``preprocess: true`` the frame runs through the Phase 2A pipeline
    first - NaN/Inf removal, ROI filtering and range filtering - and the
    response carries the measured per-stage counts and duration in
    ``processing``. Points are only ever *removed*; none are repaired or
    invented.

    Neither mode performs detection, tracking or mapping; those modules do not
    exist yet.

    Returns 422 when the point array is ragged, non-numeric, wrongly shaped, or
    outside the configured point-count limits.
    """
    if payload.preprocess:
        try:
            raw_frame = payload.to_raw_frame()
        except ValueError as exc:
            raise InvalidPointCloudError(str(exc)) from exc

        result = context.preprocessor.run(raw_frame)
        # The pipeline already enforced the configured point-count limits on the
        # raw input, and its measured duration is carried into the ingest
        # metrics so reported latency covers the real work.
        summary = service.ingest(
            result.frame,
            pre_validated=True,
            upstream_duration_s=result.metrics.duration_ms / 1000.0,
        )
        return LiDARFrameResponse(
            accepted=True,
            summary=summary,
            processing=result.metrics,
            input_summary=result.input_summary,
        )

    try:
        frame = payload.to_frame()
    except ValueError as exc:
        raise InvalidPointCloudError(str(exc)) from exc

    summary = service.ingest(frame)
    return LiDARFrameResponse(accepted=True, summary=summary)
