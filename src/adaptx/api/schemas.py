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
from adaptx.models.map import ResolutionLevel
from adaptx.models.point_cloud import (
    PointCloudFrame,
    PointCloudSummary,
    RawPointCloudFrame,
)
from adaptx.models.prediction import PredictionStatus
from adaptx.models.prediction_result import PredictionResult
from adaptx.models.processing import ProcessingMetrics
from adaptx.models.risk import RiskLevel
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
    """Adaptive 2.5D map status."""

    resolution_levels: dict[ResolutionLevel, float] = Field(
        description="Cell edge length in metres configured for each resolution level."
    )
    range_m: float
    active_cells: int = Field(
        default=0, ge=0, description="Cells in the current map; 0 until a mapper exists."
    )


class RiskStatusResponse(ModuleStatusResponse):
    """Risk engine status."""

    engine: str = Field(description="Identifier of the currently configured engine.")
    is_baseline: bool = Field(
        description="True when the configured engine is a comparison baseline."
    )
    risk_levels: list[RiskLevel]
    thresholds: dict[str, float] = Field(
        description="Lower bound of each risk level on the normalised [0, 1] scale."
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
