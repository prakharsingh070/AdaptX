"""HTTP request and response schemas.

Domain contracts live in :mod:`adaptx.models`. This module holds only the
shapes that exist because of HTTP: JSON-friendly request bodies and response
envelopes. Internal implementation details are not exposed.
"""

from __future__ import annotations

from datetime import datetime
from typing import Any

from pydantic import Field

from adaptx.models.common import (
    AdaptXModel,
    CoordinateFrame,
    DataSource,
    TimestampedModel,
    utc_now,
)
from adaptx.models.detection import DetectionResult
from adaptx.models.map import AdaptiveMap, ResolutionLevel
from adaptx.models.point_cloud import (
    PointCloudFrame,
    PointCloudSummary,
    RawPointCloudFrame,
)
from adaptx.models.prediction import PredictionStatus
from adaptx.models.prediction_result import PredictionResult
from adaptx.models.processing import ProcessingMetrics
from adaptx.models.resolution import ResolutionSource
from adaptx.models.risk import RiskLevel
from adaptx.models.risk_assessment import RiskAssessmentResult
from adaptx.models.spatial_map import (
    DEFAULT_MAX_PROJECTED_CELLS,
    MapBounds,
    SpatialMapSummary,
)
from adaptx.models.system import ComponentStatus
from adaptx.models.tracking_result import TrackingResult

#: Hard ceiling on the number of points accepted in a single JSON request,
#: independent of the configurable ``ADAPTX_LIDAR__MAX_POINTS`` limit. It bounds
#: request parsing before configuration is consulted.
MAX_REQUEST_POINTS = 1_000_000


class HealthResponse(AdaptXModel):
    """Liveness probe payload."""

    status: str = Field(description="Always 'ok' when the process is serving requests.")
    version: str
    name: str
    timestamp: datetime = Field(default_factory=utc_now)


class ErrorDetail(AdaptXModel):
    """Body of an error response."""

    code: str
    message: str
    details: dict[str, Any] = Field(default_factory=dict)


class ErrorResponse(AdaptXModel):
    """Envelope returned for every handled error."""

    error: ErrorDetail


class LiDARFrameRequest(AdaptXModel):
    """A point-cloud frame submitted over HTTP.

    ``source`` is required: the caller must declare the provenance of the data
    so simulated, replayed and synthetic frames are never recorded as sensor
    measurements.
    """

    frame_id: int = Field(ge=0)
    sensor_id: str = Field(min_length=1)
    source: DataSource = Field(description="Provenance of these points. Required.")
    points: list[list[float]] = Field(
        max_length=MAX_REQUEST_POINTS,
        description="Rows of [x, y, z] or [x, y, z, intensity], in metres.",
    )
    timestamp: datetime | None = Field(
        default=None, description="Frame time (UTC, timezone-aware). Defaults to now."
    )
    coordinate_frame: CoordinateFrame = CoordinateFrame.LIDAR
    preprocess: bool = Field(
        default=False,
        description=(
            "Run the Phase 2A preprocessing pipeline (NaN/Inf removal, ROI and "
            "range filtering) before ingest. Default false preserves the "
            "original behaviour, where any non-finite value is rejected."
        ),
    )

    def to_raw_frame(self) -> RawPointCloudFrame:
        """Convert to the raw contract, which tolerates NaN and infinities.

        Used only when ``preprocess`` is true; the pipeline is what removes
        and counts those points.
        """
        return RawPointCloudFrame.from_sequence(
            self.points,
            frame_id=self.frame_id,
            sensor_id=self.sensor_id,
            source=self.source,
            coordinate_frame=self.coordinate_frame,
            timestamp=self.timestamp if self.timestamp is not None else utc_now(),
        )

    def to_frame(self) -> PointCloudFrame:
        """Convert to the domain contract.

        Raises:
            ValueError: the points are ragged, non-numeric or otherwise violate
                the point-cloud contract.
        """
        return PointCloudFrame.from_sequence(
            self.points,
            frame_id=self.frame_id,
            sensor_id=self.sensor_id,
            source=self.source,
            coordinate_frame=self.coordinate_frame,
            timestamp=self.timestamp if self.timestamp is not None else utc_now(),
        )


class LiDARFrameResponse(AdaptXModel):
    """Result of accepting a frame.

    ``summary`` always describes the frame that was ingested: the frame as
    submitted when ``preprocess`` was false, or the processed frame when it was
    true. ``processing`` is null unless preprocessing ran.
    """

    accepted: bool
    summary: PointCloudSummary
    detail: str = Field(
        default=(
            "Frame validated and accounted for. No detection, mapping or tracking is performed."
        )
    )
    processing: ProcessingMetrics | None = Field(
        default=None,
        description="Measured preprocessing counts and duration; null when not requested.",
    )
    input_summary: PointCloudSummary | None = Field(
        default=None,
        description="Metadata of the frame as submitted; null when not preprocessed.",
    )
    ground_summary: PointCloudSummary | None = Field(
        default=None,
        description=(
            "Metadata of the points classified as ground; null unless ground "
            "segmentation ran. `summary` then describes the non-ground points."
        ),
    )


class LiDARDetectionResponse(AdaptXModel):
    """Result of running the processing pipeline and then object detection.

    ``processing`` describes what the pipeline did to the frame; ``detection``
    describes what the detector found in the non-ground points that survived.
    Raw point arrays are deliberately absent: the payload carries summaries and
    object geometry, not the cloud itself.
    """

    accepted: bool
    detection: DetectionResult
    processing: ProcessingMetrics
    summary: PointCloudSummary = Field(
        description="Metadata of the non-ground frame the detector consumed."
    )
    ground_summary: PointCloudSummary | None = Field(
        default=None, description="Metadata of the separated ground points, if any."
    )
    detail: str = Field(
        default=(
            "Objects are geometric clusters classified by size alone. This is a "
            "baseline, not trained recognition, and no tracking or prediction is "
            "performed."
        )
    )


class LiDARTrackingResponse(AdaptXModel):
    """Result of processing a frame, detecting objects and tracking them.

    Carries all three stages so a caller can see what each contributed. Raw
    point arrays are absent throughout: summaries and object geometry only.
    """

    accepted: bool
    tracking: TrackingResult
    detection: DetectionResult
    processing: ProcessingMetrics
    summary: PointCloudSummary = Field(
        description="Metadata of the non-ground frame the detector consumed."
    )
    ground_summary: PointCloudSummary | None = Field(
        default=None, description="Metadata of the separated ground points, if any."
    )
    detail: str = Field(
        default=(
            "Tracking is stateful: post frames in temporal order and reset "
            "between unrelated sequences. Velocity is measured from frame "
            "timestamps and is null until a track has two observations. No "
            "trajectory prediction is performed."
        )
    )


class LiDARPredictionResponse(AdaptXModel):
    """Result of processing a frame, detecting, tracking and predicting.

    Carries all four stages so a caller can see what each contributed. Raw
    point arrays are absent throughout: summaries, object geometry and
    trajectories only.

    Predicted positions live in ``prediction`` and nowhere else. Nothing in
    ``tracking`` is ever overwritten with a forecast.
    """

    accepted: bool
    prediction: PredictionResult
    tracking: TrackingResult
    detection: DetectionResult
    processing: ProcessingMetrics
    summary: PointCloudSummary = Field(
        description="Metadata of the non-ground frame the detector consumed."
    )
    ground_summary: PointCloudSummary | None = Field(
        default=None, description="Metadata of the separated ground points, if any."
    )
    detail: str = Field(
        default=(
            "Prediction is a deterministic constant-velocity baseline with "
            "heuristic uncertainty. Positions are extrapolations, not "
            "measurements, and their accuracy is unmeasured: no labelled "
            "trajectories exist. Tracks without a measured velocity are "
            "reported in prediction.skipped rather than assumed stationary."
        )
    )


class LiDARMapRequest(LiDARFrameRequest):
    """A frame to be mapped, plus map-specific options.

    Extends the standard frame request rather than replacing it, so a caller
    that can post to ``/detect`` can post here unchanged.
    """

    resolution_m: float | None = Field(
        default=None,
        gt=0.0,
        description=(
            "Cell edge length for this request only. Omit to use the "
            "configured value. Recorded on the map with source 'override' so a "
            "one-off resolution is never mistaken for the configured baseline."
        ),
    )
    include_cells: bool = Field(
        default=False,
        description=(
            "Include occupied cells in the response. Off by default: a 0.5 m "
            "map over the default bounds is 57,600 cells, and returning the "
            "empty ones would say nothing at great length."
        ),
    )
    max_cells: int = Field(
        default=DEFAULT_MAX_PROJECTED_CELLS,
        ge=1,
        le=DEFAULT_MAX_PROJECTED_CELLS,
        description="Upper bound on returned cells. Truncation is reported, never silent.",
    )


class LiDARMapResponse(AdaptXModel):
    """Result of processing a frame and mapping it into a 2.5D grid.

    The grid itself is **not** returned as a dense array. ``map`` carries the
    dimensions, bounds, resolution and full point accounting; ``cells`` carries
    occupied cells only, and only when asked for.
    """

    accepted: bool
    map: SpatialMapSummary
    processing: ProcessingMetrics
    summary: PointCloudSummary = Field(
        description="Metadata of the processed frame the mapper consumed."
    )
    ground_summary: PointCloudSummary | None = Field(
        default=None, description="Metadata of the separated ground points, if any."
    )
    cells: AdaptiveMap | None = Field(
        default=None,
        description="Occupied cells only, when requested. Empty cells are omitted.",
    )
    cells_truncated: bool = Field(
        default=False,
        description="True when the cell projection hit the requested limit.",
    )
    detail: str = Field(
        default=(
            "Mapping is a deterministic, frame-local, fixed-resolution baseline. "
            "One cell size applies everywhere; adaptive resolution is not "
            "implemented. Occupancy is binary (a cell holds a point or it does "
            "not), and an unobserved cell reports null height rather than zero. "
            "Nothing accumulates between frames: this is not a persistent world "
            "map."
        )
    )


class LiDARRiskRequest(LiDARFrameRequest):
    """A frame to be processed, tracked, predicted, mapped and risk-assessed."""

    include_map_context: bool = Field(
        default=True,
        description=(
            "Build the 2.5D map and use it for spatial context. Turning it off "
            "raises reported uncertainty rather than hiding the absence."
        ),
    )


class LiDARRiskResponse(AdaptXModel):
    """Result of running the whole chain and assessing risk.

    Carries the risk assessments and stage summaries. Raw point arrays, full
    trajectories and map cells are absent throughout - those come from their own
    endpoints.
    """

    accepted: bool
    risk: RiskAssessmentResult
    tracking: TrackingResult
    detection: DetectionResult
    processing: ProcessingMetrics
    prediction_summary: dict[str, Any] = Field(
        default_factory=dict, description="Counts from the prediction pass, not trajectories."
    )
    map_summary: SpatialMapSummary | None = Field(
        default=None, description="Map dimensions and accounting; null when not built."
    )
    summary: PointCloudSummary = Field(
        description="Metadata of the processed frame the chain consumed."
    )
    detail: str = Field(
        default=(
            "Risk is a deterministic engineering heuristic, not a probability of "
            "collision. It is not calibrated and has never been validated against "
            "labelled risk data, because none exists. Thresholds are baseline "
            "engineering values, not safety-certified limits. Uncertainty is "
            "reported separately from risk, never folded into the score. An "
            "object that could not be assessed is reported UNKNOWN with a null "
            "score rather than a fabricated number. Risk does not decide spatial "
            "resolution - that is a separate decision, not implemented."
        )
    )


class TrackingResetResponse(AdaptXModel):
    """Confirmation that tracking state was cleared."""

    reset: bool = True
    cleared_track_count: int = Field(
        ge=0, description="Tracks that were alive immediately before the reset."
    )
    detail: str = Field(default="All tracks dropped and identifier allocation restarted from zero.")


class TrackingStatusResponse(TimestampedModel):
    """Current tracking state, without per-track geometry."""

    summary: dict[str, Any] = Field(description="Counts, identifiers and effective configuration.")


class ModuleStatusResponse(TimestampedModel):
    """Status of a single subsystem, plus its effective configuration."""

    component: ComponentStatus
    configuration: dict[str, Any] = Field(
        default_factory=dict,
        description="Effective configuration for this subsystem (no secrets).",
    )


class MapStatusResponse(ModuleStatusResponse):
    """2.5D map status."""

    resolution_levels: dict[ResolutionLevel, float] = Field(
        description="Cell edge length in metres configured for each resolution level."
    )
    range_m: float
    active_cells: int = Field(
        default=0, ge=0, description="Occupied cells in the most recent map; 0 before the first."
    )
    mapper: str = Field(
        default="", description="Identifier of the configured mapper; empty when none exists."
    )
    is_adaptive: bool = Field(
        default=False,
        description=(
            "True only when a risk-aware mapper is configured. False for the "
            "Phase 6 fixed-resolution baseline."
        ),
    )
    adaptive_resolution_implemented: bool = Field(
        default=False,
        description="Whether a resolution controller exists. False in Phase 6.",
    )
    lifecycle: str = Field(
        default="frame_local",
        description="'frame_local': each map covers one frame and nothing accumulates.",
    )
    resolution_m: float | None = Field(
        default=None, gt=0.0, description="Cell edge length the mapper applies."
    )
    resolution_source: ResolutionSource | None = Field(
        default=None, description="Where that resolution came from."
    )
    bounds: MapBounds | None = Field(default=None, description="Extent actually mapped.")
    width: int | None = Field(default=None, ge=1, description="Cells along x.")
    height: int | None = Field(default=None, ge=1, description="Cells along y.")
    total_cells: int | None = Field(default=None, ge=1, description="width * height.")
    last_map_timestamp: datetime | None = Field(
        default=None, description="Source time of the most recent map; null before the first."
    )
    summary: dict[str, Any] = Field(
        default_factory=dict, description="Counts from the most recent map, if any."
    )


class RiskStatusResponse(ModuleStatusResponse):
    """Risk engine status."""

    engine: str = Field(description="Identifier of the currently configured engine.")
    is_baseline: bool = Field(
        description="True when the configured engine is a comparison baseline."
    )
    risk_levels: list[RiskLevel]
    thresholds: dict[str, float] = Field(
        description=(
            "Lower bound of each **scored** level on the normalised [0, 1] scale. "
            "UNKNOWN is absent by design: it means nothing was scored, so it has "
            "no threshold."
        )
    )
    scoring_model: str = Field(
        default="", description="Identifier of the scoring formulation in use."
    )
    score_is_heuristic: bool = Field(
        default=True,
        description=(
            "True: the score is a deterministic engineering heuristic, not a "
            "calibrated probability."
        ),
    )
    is_collision_probability: bool = Field(
        default=False,
        description="False: no collision-probability model exists in this project.",
    )
    uncertainty_is_heuristic: bool = Field(
        default=True,
        description="True: uncertainty is a documented heuristic, reported separately from risk.",
    )
    decides_resolution: bool = Field(
        default=False,
        description=(
            "False: the risk engine never chooses spatial resolution. That is a "
            "separate decision belonging to a resolution controller, which is "
            "not implemented."
        ),
    )
    baseline_engine: str = Field(
        default="",
        description="Identifier of the proximity-only comparison baseline retained alongside.",
    )
    frames_assessed: int = Field(default=0, ge=0)
    last_assessment_timestamp: datetime | None = Field(
        default=None,
        description="Source time of the most recent assessment; null before the first.",
    )
    summary: dict[str, Any] = Field(
        default_factory=dict, description="Counts from the most recent assessment, if any."
    )


class PredictionStatusResponse(ModuleStatusResponse):
    """Trajectory prediction status."""

    predictor: str = Field(description="Identifier of the currently configured predictor.")
    model_name: str = Field(description="Motion model applied. Not a learned model.")
    is_baseline: bool = Field(
        description="True while the configured predictor is a deterministic baseline."
    )
    horizon_s: float = Field(gt=0.0, description="Supported prediction horizon in seconds.")
    interval_s: float = Field(gt=0.0, description="Spacing between trajectory points.")
    points_per_trajectory: int = Field(
        ge=1, description="Points a full-horizon trajectory contains, t+0 inclusive."
    )
    uncertainty_model: str = Field(description="Identifier of the uncertainty model applied.")
    uncertainty_is_heuristic: bool = Field(
        default=True,
        description=(
            "True: uncertainty is a documented heuristic, not a calibrated "
            "sigma, probability or confidence interval."
        ),
    )
    prediction_statuses: list[PredictionStatus] = Field(
        description="Every outcome a track can receive, produced or skipped."
    )
    summary: dict[str, Any] = Field(
        default_factory=dict, description="Counts from the most recent prediction, if any."
    )
