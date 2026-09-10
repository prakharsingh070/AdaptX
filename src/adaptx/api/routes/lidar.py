"""LiDAR frame ingestion endpoint."""

from __future__ import annotations

from fastapi import APIRouter, status

from adaptx.api.dependencies import ContextDep, LiDARServiceDep
from adaptx.api.schemas import (
    ErrorResponse,
    LiDARDetectionResponse,
    LiDARFrameRequest,
    LiDARFrameResponse,
    LiDARTrackingResponse,
)
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
            ground_summary=result.ground_summary,
        )

    try:
        frame = payload.to_frame()
    except ValueError as exc:
        raise InvalidPointCloudError(str(exc)) from exc

    summary = service.ingest(frame)
    return LiDARFrameResponse(accepted=True, summary=summary)


@router.post(
    "/detect",
    response_model=LiDARDetectionResponse,
    status_code=status.HTTP_200_OK,
    responses={422: {"model": ErrorResponse, "description": "Malformed point cloud"}},
    summary="Process a frame and detect objects in it",
)
def detect_objects(
    payload: LiDARFrameRequest, service: LiDARServiceDep, context: ContextDep
) -> LiDARDetectionResponse:
    """Run the LiDAR pipeline, then detect objects in the non-ground points.

    The frame is always preprocessed here regardless of the request's
    ``preprocess`` flag, because detection consumes the pipeline's non-ground
    output — clustering a raw cloud would return the road surface as one
    enormous object. Ground segmentation must therefore be enabled in
    configuration for this endpoint to be useful; with it off, the whole scene
    reaches the detector and is rejected by the footprint filter.

    Detections are **geometric clusters classified by size**. There is no
    trained model, no tracking and no velocity: a single frame cannot show
    motion, so ``velocity`` is always null.

    Returns 422 when the point array is ragged, non-numeric, wrongly shaped, or
    outside the configured point-count limits.
    """
    try:
        raw_frame = payload.to_raw_frame()
    except ValueError as exc:
        raise InvalidPointCloudError(str(exc)) from exc

    processed = context.preprocessor.run(raw_frame)
    detection = context.detector.detect(processed.frame)

    service.ingest(
        processed.frame,
        pre_validated=True,
        upstream_duration_s=(processed.metrics.duration_ms + detection.duration_ms) / 1000.0,
    )

    return LiDARDetectionResponse(
        accepted=True,
        detection=detection,
        processing=processed.metrics,
        summary=processed.output_summary,
        ground_summary=processed.ground_summary,
    )


@router.post(
    "/track",
    response_model=LiDARTrackingResponse,
    status_code=status.HTTP_200_OK,
    responses={422: {"model": ErrorResponse, "description": "Malformed point cloud"}},
    summary="Process a frame, detect objects, and track them across frames",
)
def track_frame(
    payload: LiDARFrameRequest, service: LiDARServiceDep, context: ContextDep
) -> LiDARTrackingResponse:
    """Run processing, detection and tracking for one frame.

    **Stateful**, unlike every other endpoint here. Each call advances the
    tracker by one frame, so frames must be posted in temporal order, and
    ``POST /api/v1/tracking/reset`` should be called between unrelated
    sequences - otherwise the first frame of a new scene is associated against
    tracks from the old one.

    The request ``timestamp`` matters: velocity is measured from the interval
    between successive frames. Omitting it makes every frame "now", so the
    measured interval becomes the wall-clock gap between HTTP requests rather
    than the gap between scans.

    Velocity is ``null`` on a track's first frame. One observation cannot show
    motion, and reporting zero would claim a measured standstill.

    No trajectory prediction is performed. ``predicted_position`` on a track is
    the tracker's own association guess, not a forecast.

    Returns 422 for the same malformed input as the other LiDAR endpoints.
    """
    try:
        raw_frame = payload.to_raw_frame()
    except ValueError as exc:
        raise InvalidPointCloudError(str(exc)) from exc

    processed = context.preprocessor.run(raw_frame)
    detection = context.detector.detect(processed.frame)
    tracking = context.tracking.update(
        detection.objects,
        processed.frame.timestamp,
        frame_id=processed.frame.frame_id,
        sensor_id=processed.frame.sensor_id,
    )

    service.ingest(
        processed.frame,
        pre_validated=True,
        upstream_duration_s=(
            processed.metrics.duration_ms + detection.duration_ms + tracking.duration_ms
        )
        / 1000.0,
    )

    return LiDARTrackingResponse(
        accepted=True,
        tracking=tracking,
        detection=detection,
        processing=processed.metrics,
        summary=processed.output_summary,
        ground_summary=processed.ground_summary,
    )
