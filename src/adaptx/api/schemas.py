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
from adaptx.models.map import ResolutionLevel
from adaptx.models.point_cloud import (
    PointCloudFrame,
    PointCloudSummary,
    RawPointCloudFrame,
)
from adaptx.models.processing import ProcessingMetrics
from adaptx.models.risk import RiskLevel
from adaptx.models.system import ComponentStatus

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
