"""LiDAR frame ingestion endpoint."""

from __future__ import annotations

from fastapi import APIRouter, status

from adaptx.api.dependencies import ContextDep, LiDARServiceDep
from adaptx.api.schemas import (
    ErrorResponse,
    LiDARAdaptiveMapRequest,
    LiDARAdaptiveMapResponse,
    LiDARDetectionResponse,
    LiDARFrameRequest,
    LiDARFrameResponse,
    LiDARMapRequest,
    LiDARMapResponse,
    LiDARPredictionResponse,
    LiDARRiskRequest,
    LiDARRiskResponse,
    LiDARTrackingResponse,
)
from adaptx.core.exceptions import InvalidPointCloudError
from adaptx.models.resolution import ResolutionDecision
from adaptx.models.scene import PointStage
from adaptx.services.scene_service import build_snapshot

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


@router.post(
    "/predict",
    response_model=LiDARPredictionResponse,
    status_code=status.HTTP_200_OK,
    responses={422: {"model": ErrorResponse, "description": "Malformed point cloud"}},
    summary="Process, detect, track, and predict future trajectories",
)
def predict_frame(
    payload: LiDARFrameRequest, service: LiDARServiceDep, context: ContextDep
) -> LiDARPredictionResponse:
    """Run processing, detection, tracking and trajectory prediction for one frame.

    **Stateful**, for the same reason ``/track`` is: prediction consumes tracks,
    and a track exists only because of the frames that came before. Every
    caveat on ``POST /api/v1/lidar/track`` applies here unchanged - post frames
    in temporal order, send explicit timestamps, and call
    ``POST /api/v1/tracking/reset`` between unrelated sequences.

    Prediction itself carries no state between frames: the same tracks and
    timestamp always produce the same trajectories.

    The predictor is a **deterministic constant-velocity baseline**. It
    extrapolates each track's measured velocity and models nothing else - no
    acceleration, no turning, no lane geometry, no interaction between objects.
    Its accuracy is **unmeasured**, because no labelled trajectories exist.

    A track's first frame has no measured velocity, so no trajectory is
    produced for it; it appears in ``prediction.skipped`` with
    ``insufficient_velocity``. A *measured* standstill is different and yields
    a stationary trajectory. Predicted positions appear only in
    ``prediction``; nothing in ``tracking`` is overwritten with a forecast.

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
    prediction = context.prediction.predict_from_tracking(tracking)

    service.ingest(
        processed.frame,
        pre_validated=True,
        upstream_duration_s=(
            processed.metrics.duration_ms
            + detection.duration_ms
            + tracking.duration_ms
            + prediction.duration_ms
        )
        / 1000.0,
    )

    return LiDARPredictionResponse(
        accepted=True,
        prediction=prediction,
        tracking=tracking,
        detection=detection,
        processing=processed.metrics,
        summary=processed.output_summary,
        ground_summary=processed.ground_summary,
    )


@router.post(
    "/map",
    response_model=LiDARMapResponse,
    status_code=status.HTTP_200_OK,
    responses={422: {"model": ErrorResponse, "description": "Malformed point cloud or resolution"}},
    summary="Process a frame and build a 2.5D spatial map from it",
)
def map_frame(
    payload: LiDARMapRequest, service: LiDARServiceDep, context: ContextDep
) -> LiDARMapResponse:
    """Run the LiDAR pipeline, then bin the processed points into a 2.5D grid.

    **Stateless and frame-local.** Each call builds a complete map from the
    frame it is given; nothing accumulates and nothing carries over from an
    earlier request. This is not a persistent world map, and there is no
    reset to call.

    The mapper is a **deterministic fixed-resolution baseline**: one cell size
    applies across the whole map. Adaptive, risk-aware resolution is not
    implemented - the mapper is handed a resolution and never chooses one.

    Occupancy is binary: a cell is occupied when it contains at least one
    point. A cell with no points reports **null** height rather than zero,
    because zero is a real height in this frame.

    Every input point is accounted for: ``input_point_count`` always equals
    ``mapped_point_count + out_of_bounds_point_count``. A point outside the
    configured bounds is counted, not silently dropped.

    The dense grid is never returned. ``map`` carries dimensions, bounds,
    resolution and accounting; set ``include_cells`` to receive occupied cells
    only, capped by ``max_cells`` with any truncation reported.

    Returns 422 when the point array is malformed, or when the requested
    resolution is outside the configured limits or would exceed the cell
    budget.
    """
    try:
        raw_frame = payload.to_raw_frame()
    except ValueError as exc:
        raise InvalidPointCloudError(str(exc)) from exc

    processed = context.preprocessor.run(raw_frame)
    resolution = (
        ResolutionDecision.override(
            payload.resolution_m,
            reason="requested per API call",
            requested_by="api",
        )
        if payload.resolution_m is not None
        else None
    )
    spatial_map = context.mapping.build(processed.frame, resolution)

    service.ingest(
        processed.frame,
        pre_validated=True,
        upstream_duration_s=(processed.metrics.duration_ms + spatial_map.duration_ms) / 1000.0,
    )

    cells = None
    truncated = False
    if payload.include_cells:
        cells, truncated = spatial_map.to_adaptive_map(max_cells=payload.max_cells)

    return LiDARMapResponse(
        accepted=True,
        map=spatial_map.summary(),
        processing=processed.metrics,
        summary=processed.output_summary,
        ground_summary=processed.ground_summary,
        cells=cells,
        cells_truncated=truncated,
    )


@router.post(
    "/risk",
    response_model=LiDARRiskResponse,
    status_code=status.HTTP_200_OK,
    responses={422: {"model": ErrorResponse, "description": "Malformed point cloud"}},
    summary="Process, detect, track, predict, map, and assess risk",
)
def assess_risk(
    payload: LiDARRiskRequest, service: LiDARServiceDep, context: ContextDep
) -> LiDARRiskResponse:
    """Run the whole perception chain and assess the risk of each tracked object.

    **Stateful**, because it consumes tracks: post frames in temporal order,
    send explicit timestamps, and call ``POST /api/v1/tracking/reset`` between
    unrelated sequences. Risk assessment itself carries no state between frames.

    **Risk is a deterministic engineering heuristic.** Three factors -
    proximity, rate of approach, and how close the predicted path passes -
    combined as a weighted mean over the factors actually available. A factor
    that cannot be computed is dropped and the remaining weights renormalise; it
    is never treated as zero. It is **not** a probability of collision, is not
    calibrated, and has never been validated against labelled risk data.

    **Uncertainty is reported separately, never folded into the score.** An
    object can be low-risk and poorly observed, and that combination is exactly
    what a later resolution policy would need to see.

    A track whose velocity was never measured keeps its proximity factor but
    loses the approach factor, and its uncertainty rises. A track that is lost,
    or for which nothing could be computed, is reported ``UNKNOWN`` with a
    **null score** rather than a fabricated number.

    Map context never lowers risk. A cell with no returns is *unobserved*, not
    free space.

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
    prediction = context.prediction.predict_from_tracking(tracking)
    spatial_map = context.mapping.build(processed.frame) if payload.include_map_context else None
    risk = context.risk.assess_from_pipeline(
        tracking, prediction=prediction, spatial_map=spatial_map
    )

    upstream_ms = (
        processed.metrics.duration_ms
        + detection.duration_ms
        + tracking.duration_ms
        + prediction.duration_ms
        + risk.duration_ms
        + (0.0 if spatial_map is None else spatial_map.duration_ms)
    )
    service.ingest(processed.frame, pre_validated=True, upstream_duration_s=upstream_ms / 1000.0)

    return LiDARRiskResponse(
        accepted=True,
        risk=risk,
        tracking=tracking,
        detection=detection,
        processing=processed.metrics,
        prediction_summary={
            "predictor": prediction.predictor,
            "considered_tracks": prediction.considered_track_count,
            "predicted_tracks": prediction.predicted_track_count,
            "skipped_tracks": prediction.skipped_track_count,
            "counts_by_status": prediction.counts_by_status(),
        },
        map_summary=None if spatial_map is None else spatial_map.summary(),
        summary=processed.output_summary,
    )


@router.post(
    "/adaptive-map",
    response_model=LiDARAdaptiveMapResponse,
    status_code=status.HTTP_200_OK,
    responses={422: {"model": ErrorResponse, "description": "Malformed point cloud"}},
    summary="Process, detect, track, predict, assess risk, and map at adaptive resolution",
)
def adaptive_map_frame(
    payload: LiDARAdaptiveMapRequest, service: LiDARServiceDep, context: ContextDep
) -> LiDARAdaptiveMapResponse:
    """Run the whole chain, then allocate spatial detail by region.

    **Stateful in two ways**, and both matter. It consumes tracks, so frames
    must be posted in temporal order with explicit timestamps. It *also*
    stabilises resolution across frames: each region remembers the level it
    held and how long a coarser one has been proposed, so a region does not
    give up detail the instant a score dips. Call
    ``POST /api/v1/tracking/reset`` and
    ``POST /api/v1/map/adaptive/reset`` between unrelated sequences, or a new
    scene inherits the history of the old one.

    **Resolution is decided per region, never per point.** The map extent is
    partitioned into fixed-size regions and each is given its own cell size,
    so one map genuinely holds several resolutions at once. Regions partition
    the extent exactly: no point falls in two of them, and none falls in none.

    **The detail priority is an engineering prioritisation score.** It combines
    risk, uncertainty, predicted-motion relevance, object density, proximity
    and measured motion as a weighted mean over the factors actually
    available. A factor that cannot be computed is dropped and the remaining
    weights renormalise; it is never treated as zero. The score is **not** a
    probability of collision, not a safety margin, not calibrated, and has
    never been validated against labelled data.

    **Uncertainty raises detail on its own.** A quiet but badly observed region
    can earn a finer level than a confidently quiet one, which is the whole
    reason risk and uncertainty are kept apart.

    **An object whose risk could not be scored does not make a region quiet.**
    Its risk factor is dropped rather than counted as zero, and the region
    takes a conservative floor level. Unknown is not low.

    Occupancy is binary and an unobserved cell reports **null** height, not
    zero. Every input point is accounted for as mapped or out of bounds.

    Set ``include_fixed_comparison`` to build the Phase 6 fixed-resolution map
    over the same frame and receive both workloads - what each spent, and where
    the adaptive cells went. That is the measurement the adaptive claim rests
    on, and it costs a second full map.

    Returns 422 for the same malformed input as the other LiDAR endpoints, and
    when the plan would exceed the configured cell ceiling.
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
    prediction = context.prediction.predict_from_tracking(tracking)

    # The fixed map is built whenever risk needs spatial context, and reused
    # for the comparison rather than built twice.
    spatial_map = (
        context.mapping.build(processed.frame)
        if payload.include_map_context or payload.include_fixed_comparison
        else None
    )
    risk = context.risk.assess_from_pipeline(
        tracking, prediction=prediction, spatial_map=spatial_map
    )
    adaptive_map = context.adaptive_mapping.run_from_pipeline(
        processed.frame, risk, tracking, trajectories=prediction.trajectories
    )

    upstream_ms = (
        processed.metrics.duration_ms
        + detection.duration_ms
        + tracking.duration_ms
        + prediction.duration_ms
        + risk.duration_ms
        + adaptive_map.plan.duration_ms
        + adaptive_map.duration_ms
        + (0.0 if spatial_map is None else spatial_map.duration_ms)
    )
    service.ingest(processed.frame, pre_validated=True, upstream_duration_s=upstream_ms / 1000.0)

    plan = adaptive_map.plan
    decisions = None
    decisions_truncated = False
    if payload.include_decisions:
        decisions = plan.decisions[: payload.max_decisions]
        decisions_truncated = len(plan.decisions) > payload.max_decisions

    cells = None
    cells_truncated = False
    if payload.include_cells:
        cells, cells_truncated = adaptive_map.to_adaptive_map(max_cells=payload.max_cells)

    comparison = (
        context.adaptive_mapping.compare_with_fixed(adaptive_map, spatial_map)
        if payload.include_fixed_comparison and spatial_map is not None
        else None
    )

    # The dashboard's live scene: what this request produced, handed over
    # for display after the fact. Nothing above depends on it (ADR-055).
    context.scene.publish(
        build_snapshot(
            frame_id=processed.frame.frame_id,
            sensor_id=processed.frame.sensor_id,
            source=processed.frame.source,
            frame_timestamp=processed.frame.timestamp,
            origin="api",
            points=processed.frame.points,
            point_stage=PointStage.PROCESSED,
            detection=detection,
            tracking=tracking,
            prediction=prediction,
            risk=risk,
            plan=plan,
            adaptive_map=adaptive_map.summary(controller_duration_ms=plan.duration_ms),
            fixed_map=None if spatial_map is None else spatial_map.summary(),
            comparison=comparison,
            processing_ms=processed.metrics.duration_ms,
            max_points=context.settings.dashboard.scene_max_points,
        )
    )

    return LiDARAdaptiveMapResponse(
        accepted=True,
        map=adaptive_map.summary(controller_duration_ms=plan.duration_ms),
        plan_summary={
            "controller": plan.controller,
            "policy_model": plan.policy_model,
            "is_baseline": plan.is_baseline,
            "frame_index": plan.frame_index,
            "tile_count": plan.tile_count,
            "changed_tile_count": plan.changed_tile_count,
            "considered_assessments": plan.considered_assessment_count,
            "influencing_assessments": plan.influencing_assessment_count,
            "excluded_assessments": len(plan.excluded),
            "tiles_by_level": plan.counts_by_level(),
            "cells_by_level": plan.cells_by_level(),
            "highest_priority": plan.highest_priority,
            "budget": plan.budget.model_dump(mode="json"),
            "duration_ms": plan.duration_ms,
        },
        decisions=decisions,
        decisions_truncated=decisions_truncated,
        risk=risk,
        processing=processed.metrics,
        summary=processed.output_summary,
        comparison=comparison,
        cells=cells,
        cells_truncated=cells_truncated,
    )
