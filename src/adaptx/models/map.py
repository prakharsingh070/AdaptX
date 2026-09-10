"""Adaptive 2.5D map contracts.

A 2.5D map indexes space by ``x``/``y`` and summarises ``z`` as height
information rather than storing a full volumetric grid
(``docs/knowledge-base/05_2.5d-mapping.md``).

Phase 1 defines the data structures and the resolution vocabulary only. The
adaptive resolution algorithm - deciding which level a region should use - is
not implemented; see :class:`adaptx.mapping.interfaces.ResolutionController`.
"""

from __future__ import annotations

from enum import StrEnum

from pydantic import Field, model_validator

from adaptx.models.common import (
    AdaptXModel,
    CoordinateFrame,
    DataSource,
    TimestampedModel,
    Vector3,
)


class ResolutionLevel(StrEnum):
    """Spatial detail assigned to a region.

    Coarse to fine. The metre value of each level is configuration
    (``ADAPTX_MAP__RESOLUTION_*_M``), not a constant of the model.
    """

    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


class OccupancyState(StrEnum):
    """Discrete occupancy interpretation of a cell."""

    UNKNOWN = "unknown"
    FREE = "free"
    OCCUPIED = "occupied"


class AdaptiveMapCell(TimestampedModel):
    """One cell of the adaptive 2.5D map.

    ``x``/``y`` are cell-centre coordinates in metres; ``resolution_m`` is the
    cell edge length, which varies across the map by design.
    """

    x: float
    y: float
    resolution_m: float = Field(gt=0.0)
    resolution_level: ResolutionLevel = ResolutionLevel.LOW
    occupancy: float = Field(ge=0.0, le=1.0, description="Occupancy probability in [0, 1].")
    occupancy_state: OccupancyState = OccupancyState.UNKNOWN
    height_m: float | None = Field(
        default=None, description="Representative surface height; null when unobserved."
    )
    height_min_m: float | None = None
    height_max_m: float | None = None
    point_count: int = Field(default=0, ge=0)
    confidence: float = Field(default=0.0, ge=0.0, le=1.0)
    risk_score: float = Field(default=0.0, ge=0.0, le=1.0)
    uncertainty: float = Field(default=0.0, ge=0.0, le=1.0)
    coordinate_frame: CoordinateFrame = CoordinateFrame.EGO
    source: DataSource = DataSource.LIVE_SENSOR

    @model_validator(mode="after")
    def _check_height_range(self) -> AdaptiveMapCell:
        if (
            self.height_min_m is not None
            and self.height_max_m is not None
            and self.height_min_m > self.height_max_m
        ):
            raise ValueError("height_min_m must be <= height_max_m")
        return self


class ResolutionContext(AdaptXModel):
    """Inputs available when deciding the resolution of one region.

    This is the contract behind the central ADAPT-X question: *given the
    environment and risk around this region, how much spatial detail should it
    receive?* Consumed by
    :class:`~adaptx.mapping.interfaces.ResolutionController`, implemented in
    Phase 8.

    Nothing here is invented
    -----------------------
    Every quantity a region may lack is nullable, and ``None`` means *not
    computed* - never zero. ``risk_score`` in particular is ``None`` whenever
    the influencing objects could not be scored, because coercing that to 0.0
    would hand the coarsest representation to the objects the system
    understands least (ADR-032, ADR-038).
    """

    x: float
    y: float
    distance_from_ego_m: float = Field(ge=0.0)
    risk_score: float | None = Field(
        default=None,
        ge=0.0,
        le=1.0,
        description=(
            "Strongest influencing risk score, or null when no influencing "
            "object could be scored. Null is NOT low risk - never coerce it "
            "to 0.0 (ADR-032)."
        ),
    )
    predicted_risk_score: float | None = Field(
        default=None,
        ge=0.0,
        le=1.0,
        description="Risk expected from predicted motion, or null when unavailable.",
    )
    uncertainty: float | None = Field(
        default=None,
        ge=0.0,
        le=1.0,
        description=(
            "Strongest influencing heuristic uncertainty, or null when nothing "
            "influenced the region. Reported beside risk, never summed into it "
            "(ADR-033)."
        ),
    )
    object_density: float | None = Field(
        default=None,
        ge=0.0,
        description="Objects per square metre near the region; null when unknown.",
    )
    max_object_speed_mps: float | None = Field(
        default=None,
        ge=0.0,
        description="Fastest measured influencing speed; null when no speed was ever measured.",
    )
    trajectory_relevance: float | None = Field(
        default=None,
        ge=0.0,
        le=1.0,
        description=(
            "How strongly a predicted path passes through this region, weighted "
            "towards the near future. Null when no trajectory reaches it. This "
            "is predicted-motion relevance, NOT a collision prediction."
        ),
    )
    object_count: int = Field(
        default=0, ge=0, description="Objects whose influence reaches this region."
    )
    has_unknown_risk: bool = Field(
        default=False,
        description=(
            "True when at least one influencing object could not be scored. "
            "Such a region is treated conservatively rather than as quiet."
        ),
    )
    in_ego_path: bool = False
    current_level: ResolutionLevel | None = Field(
        default=None, description="Level in force now, for hysteresis and dwell-time rules."
    )


class AdaptiveMap(TimestampedModel):
    """A 2.5D map snapshot.

    ``is_adaptive`` distinguishes an ADAPT-X map from the fixed-resolution
    baseline required by ADR-003, so benchmark results can never be mixed up.
    """

    frame_id: int = Field(ge=0)
    is_adaptive: bool
    origin: Vector3 = Field(
        default_factory=Vector3, description="Map origin in `coordinate_frame`."
    )
    range_m: float = Field(gt=0.0)
    cells: list[AdaptiveMapCell] = Field(default_factory=list)
    coordinate_frame: CoordinateFrame = CoordinateFrame.EGO
    source: DataSource = DataSource.LIVE_SENSOR

    @property
    def cell_count(self) -> int:
        """Number of active cells in the snapshot."""
        return len(self.cells)

    @property
    def average_resolution_m(self) -> float | None:
        """Mean cell edge length, or ``None`` when the map is empty."""
        if not self.cells:
            return None
        return sum(cell.resolution_m for cell in self.cells) / len(self.cells)
