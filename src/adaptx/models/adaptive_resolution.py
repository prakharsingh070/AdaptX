"""Adaptive resolution decision contracts (Phase 8).

What this layer answers
-----------------------
*How much spatial detail should this region receive?*

It does **not** answer *how concerning is this object* - that is Phase 7
(ADR-036) - and it does not represent space. It produces a plan; the mapper
applies it.

::

    RiskAssessment[] + PredictedTrajectory[]
        -> ResolutionContext (per tile)
        -> [ResolutionController]
        -> TileResolutionDecision[]  (a ResolutionPlan)
        -> RegionAdaptiveMapper

Everything here is heuristic
----------------------------
``detail_priority`` is a **deterministic engineering prioritisation score**. It
answers "how much detail does this region deserve relative to the others", and
it is not a probability of collision, not a safety margin, not calibrated and
not validated against any labelled dataset - none exists. Its weights and
thresholds are baseline engineering parameters, never tuned against outcomes.

Missing information is never invented
-------------------------------------
A factor that could not be computed is **absent** from ``factors`` and null in
``factor_scores``; it is dropped from the weighted mean and the remaining
weights renormalise, exactly as ADR-032 requires of risk. A tile that no object
influences has ``detail_priority = None`` - not 0.0 - and takes the configured
base level with a recorded reason.
"""

from __future__ import annotations

import math
from enum import StrEnum

from pydantic import Field, field_validator, model_validator

from adaptx.models.common import AdaptXModel, TimestampedModel
from adaptx.models.map import ResolutionLevel
from adaptx.models.resolution import ResolutionDecision, ResolutionSource
from adaptx.models.spatial_map import MapBounds

#: Identifier of the scoring formulation, recorded on every plan.
DETAIL_POLICY_MODEL = "heuristic_weighted_detail_priority"

#: Levels counted against the fine-tile budget: those finer than MEDIUM.
FINE_LEVELS: tuple[ResolutionLevel, ...] = (ResolutionLevel.HIGH, ResolutionLevel.CRITICAL)

#: Levels ordered coarse to fine. The ordering is a property of the vocabulary,
#: not of any cell size, so it holds however the levels are configured.
LEVEL_ORDER: tuple[ResolutionLevel, ...] = (
    ResolutionLevel.LOW,
    ResolutionLevel.MEDIUM,
    ResolutionLevel.HIGH,
    ResolutionLevel.CRITICAL,
)

#: Rank of each level, coarse (0) to fine (3).
LEVEL_RANK: dict[ResolutionLevel, int] = {level: rank for rank, level in enumerate(LEVEL_ORDER)}


def is_finer(candidate: ResolutionLevel, reference: ResolutionLevel) -> bool:
    """Whether ``candidate`` asks for more detail than ``reference``."""
    return LEVEL_RANK[candidate] > LEVEL_RANK[reference]


def finer_of(first: ResolutionLevel, second: ResolutionLevel) -> ResolutionLevel:
    """The more detailed of two levels."""
    return first if LEVEL_RANK[first] >= LEVEL_RANK[second] else second


def coarser_by_one(level: ResolutionLevel) -> ResolutionLevel | None:
    """One step coarser, or ``None`` when already coarsest."""
    rank = LEVEL_RANK[level]
    return None if rank == 0 else LEVEL_ORDER[rank - 1]


class DetailFactorName(StrEnum):
    """A named contribution that was actually computed for a tile.

    A factor absent from a decision list was **not computed**, which is
    different from computed-and-zero; ``DetailFactors`` distinguishes the two.
    """

    RISK = "risk"
    UNCERTAINTY = "uncertainty"
    PROXIMITY = "proximity"
    TRAJECTORY = "trajectory"
    DENSITY = "density"
    MOTION = "motion"


class DetailFactors(AdaptXModel):
    """Normalised value of each detail factor, for attribution.

    Every field is optional because a factor that could not be computed is
    reported as null rather than zero. ``uncertainty`` is a **separate input**
    from ``risk``, never summed into it (ADR-033): a poorly observed region can
    deserve detail precisely because it is poorly observed.
    """

    risk: float | None = Field(
        default=None,
        ge=0.0,
        le=1.0,
        description="Strongest influencing risk score. Null when no influencing object was scored.",
    )
    uncertainty: float | None = Field(
        default=None, ge=0.0, le=1.0, description="Strongest influencing heuristic uncertainty."
    )
    proximity: float | None = Field(
        default=None, ge=0.0, le=1.0, description="Nearness of the tile to the ego reference."
    )
    trajectory: float | None = Field(
        default=None,
        ge=0.0,
        le=1.0,
        description=(
            "Predicted-motion relevance: a predicted path passes through this "
            "tile within the horizon. NOT a statement that anything will arrive."
        ),
    )
    density: float | None = Field(
        default=None, ge=0.0, le=1.0, description="Saturating count of influencing objects."
    )
    motion: float | None = Field(
        default=None,
        ge=0.0,
        le=1.0,
        description="Fastest measured influencing speed. Null when no speed was ever measured.",
    )


class TileDecisionReason(StrEnum):
    """Why a tile ended at the level it did.

    Reasons accumulate: a tile can be scored, floored for unknown risk and then
    demoted by the cell budget, and all three stay visible.
    """

    SCORED = "scored"
    NO_OBJECT_INFLUENCE = "no_object_influence"
    UNKNOWN_RISK_FLOOR = "unknown_risk_floor"
    HYSTERESIS_HOLD = "hysteresis_hold"
    DWELL_HOLD = "dwell_hold"
    BUDGET_DEMOTED = "budget_demoted"
    FINE_TILE_LIMIT = "fine_tile_limit"
    ADAPTATION_DISABLED = "adaptation_disabled"


class ExclusionReason(StrEnum):
    """Why an assessment contributed to no tile."""

    OUT_OF_BOUNDS = "out_of_bounds"
    TRACK_LOST = "track_lost"
    LIMIT_EXCEEDED = "limit_exceeded"
    MISSING_POSITION = "missing_position"


class ExcludedAssessment(AdaptXModel):
    """One assessment the controller declined to use, and why.

    Present so a plan accounts for every assessment it was handed, the same
    rule detection, tracking and prediction already follow (ADR-022/024/027).
    """

    track_id: int = Field(ge=0)
    reason: ExclusionReason
    distance_m: float | None = Field(
        default=None, ge=0.0, description="Planar distance to the ego reference, when known."
    )


class TileResolutionDecision(AdaptXModel):
    """The resolution chosen for one tile, and everything behind that choice.

    Carries enough for mapping, telemetry, a future dashboard, debugging and
    benchmarking: what was decided, what it replaced, what evidence produced
    it, and which tracks contributed.
    """

    tile_index: int = Field(ge=0, description="Row-major index of the tile within the map.")
    tile_row: int = Field(ge=0)
    tile_column: int = Field(ge=0)
    bounds: MapBounds = Field(description="Extent of the tile, clipped to the map bounds.")

    level: ResolutionLevel
    resolution: ResolutionDecision = Field(
        description="Cell size in force for this tile, with source ADAPTIVE."
    )
    previous_level: ResolutionLevel | None = Field(
        default=None, description="Level in force before this frame; null on the first frame."
    )
    changed: bool = Field(description="Whether this frame changed the level of the tile.")

    detail_priority: float | None = Field(
        default=None,
        ge=0.0,
        le=1.0,
        description=(
            "Heuristic prioritisation score in [0, 1], or null when no object "
            "influenced the tile. NOT a probability and NOT a safety margin."
        ),
    )
    factors: list[DetailFactorName] = Field(
        default_factory=list, description="Factors actually computed for this tile."
    )
    factor_scores: DetailFactors = Field(default_factory=DetailFactors)
    reasons: list[TileDecisionReason] = Field(default_factory=list)
    reason: str = Field(
        min_length=1,
        description="Explanation generated from the computed factors, never free-form text.",
    )

    influencing_track_ids: list[int] = Field(
        default_factory=list, description="Tracks whose influence reached this tile, ascending."
    )
    unknown_risk_track_ids: list[int] = Field(
        default_factory=list,
        description=(
            "Influencing tracks whose risk could not be scored. Their risk is "
            "dropped from the mean, never counted as 0.0 (ADR-032)."
        ),
    )

    cell_width: int = Field(ge=1, description="Cells along x within this tile.")
    cell_height: int = Field(ge=1, description="Cells along y within this tile.")

    @field_validator("detail_priority")
    @classmethod
    def _require_finite(cls, value: float | None) -> float | None:
        if value is not None and not math.isfinite(value):
            raise ValueError("detail_priority must be finite")
        return value

    @model_validator(mode="after")
    def _check_adaptive_source(self) -> TileResolutionDecision:
        if self.resolution.source is not ResolutionSource.ADAPTIVE:
            raise ValueError(
                "a tile decision must carry an ADAPTIVE resolution, got "
                f"{self.resolution.source.value}"
            )
        if not self.reasons:
            raise ValueError("a tile decision must record at least one reason")
        return self

    @property
    def cell_count(self) -> int:
        """Cells this tile allocates at its chosen level."""
        return self.cell_width * self.cell_height

    @property
    def resolution_m(self) -> float:
        """Cell edge length in metres."""
        return self.resolution.resolution_m

    @property
    def is_fine(self) -> bool:
        """Whether this tile counts against the fine-tile budget."""
        return self.level in FINE_LEVELS


class ResolutionBudget(AdaptXModel):
    """What the plan allocated against the configured ceilings.

    Reported rather than assumed: when a scene wants more detail than the
    budget allows, the shortfall is visible instead of being silently absorbed
    or silently over-allocated.
    """

    tile_count: int = Field(ge=0)
    max_tiles: int = Field(ge=1)
    total_cell_count: int = Field(ge=0)
    max_total_cells: int = Field(ge=1)
    fine_tile_count: int = Field(ge=0)
    max_fine_tiles: int = Field(ge=0)
    demoted_tile_count: int = Field(
        default=0, ge=0, description="Tiles coarsened to fit a ceiling."
    )
    within_budget: bool = Field(
        description="False when a ceiling could not be met even after demotion."
    )

    @property
    def cell_utilisation(self) -> float:
        """Fraction of the cell ceiling used."""
        return self.total_cell_count / self.max_total_cells


class AdaptiveResolutionConfiguration(AdaptXModel):
    """Effective controller configuration that produced a plan.

    Carried on every plan so a record is self-describing - the same rule every
    other module follows. All values are **baseline engineering parameters**,
    not validated or tuned limits.
    """

    tile_size_m: float
    base_level: ResolutionLevel
    unknown_risk_min_level: ResolutionLevel
    level_cell_sizes: dict[ResolutionLevel, float]

    threshold_medium: float
    threshold_high: float
    threshold_critical: float

    weight_risk: float
    weight_uncertainty: float
    weight_proximity: float
    weight_trajectory: float
    weight_density: float
    weight_motion: float

    proximity_near_m: float
    proximity_far_m: float
    density_saturation_objects: int
    motion_saturation_mps: float

    influence_radius_m: float
    uncertainty_radius_gain_m: float
    trajectory_corridor_m: float

    hysteresis_margin: float
    min_dwell_frames: int

    max_tiles: int
    max_total_cells: int
    max_fine_tiles: int
    max_influencing_objects: int


class ResolutionPlan(TimestampedModel):
    """Everything one controller pass produced.

    Every tile of the map appears in ``decisions`` - including those no object
    influenced, which carry a null ``detail_priority``. "Nothing needs detail
    here" and "nothing could be evaluated here" stay distinguishable.

    Every assessment handed to the controller is accounted for: it either
    influenced at least one tile, or appears in ``excluded`` with a reason.
    """

    frame_id: int = Field(ge=0)
    sensor_id: str = Field(min_length=1)
    controller: str = Field(min_length=1, description="Identifier of the controller that ran.")
    is_baseline: bool = Field(
        default=True,
        description="True while the policy is a deterministic heuristic, not a learned one.",
    )
    policy_model: str = Field(
        default=DETAIL_POLICY_MODEL,
        description="Identifier of the prioritisation formulation. Heuristic, uncalibrated.",
    )

    decisions: list[TileResolutionDecision] = Field(default_factory=list)
    excluded: list[ExcludedAssessment] = Field(default_factory=list)
    considered_assessment_count: int = Field(ge=0)
    influencing_assessment_count: int = Field(ge=0)

    frame_index: int = Field(
        ge=0,
        description="Controller frames seen since the last reset, for stabilisation traceability.",
    )
    budget: ResolutionBudget
    duration_ms: float = Field(ge=0.0, description="Whole controller pass, measured.")
    configuration: AdaptiveResolutionConfiguration

    @model_validator(mode="after")
    def _check_accounting(self) -> ResolutionPlan:
        accounted = self.influencing_assessment_count + len(self.excluded)
        if self.considered_assessment_count != accounted:
            raise ValueError(
                f"considered_assessment_count ({self.considered_assessment_count}) must equal "
                f"influencing ({self.influencing_assessment_count}) + excluded "
                f"({len(self.excluded)})"
            )
        indices = [decision.tile_index for decision in self.decisions]
        if indices != sorted(indices):
            raise ValueError("tile decisions must be ordered by tile_index")
        if len(set(indices)) != len(indices):
            raise ValueError("a tile may appear at most once in a plan")
        return self

    @property
    def tile_count(self) -> int:
        """Tiles the plan covers."""
        return len(self.decisions)

    @property
    def changed_tile_count(self) -> int:
        """Tiles whose level changed this frame."""
        return sum(1 for decision in self.decisions if decision.changed)

    @property
    def total_cell_count(self) -> int:
        """Cells the plan allocates across every tile."""
        return sum(decision.cell_count for decision in self.decisions)

    def counts_by_level(self) -> dict[str, int]:
        """Tiles at each level, including levels with none."""
        counts = {level.value: 0 for level in LEVEL_ORDER}
        for decision in self.decisions:
            counts[decision.level.value] += 1
        return counts

    def cells_by_level(self) -> dict[str, int]:
        """Cells allocated at each level."""
        cells = {level.value: 0 for level in LEVEL_ORDER}
        for decision in self.decisions:
            cells[decision.level.value] += decision.cell_count
        return cells

    @property
    def finest_resolution_m(self) -> float | None:
        """Smallest cell size in the plan, or ``None`` when it has no tiles."""
        if not self.decisions:
            return None
        return min(decision.resolution_m for decision in self.decisions)

    @property
    def coarsest_resolution_m(self) -> float | None:
        """Largest cell size in the plan, or ``None`` when it has no tiles."""
        if not self.decisions:
            return None
        return max(decision.resolution_m for decision in self.decisions)

    @property
    def highest_priority(self) -> float | None:
        """Largest computed priority, or ``None`` when nothing was scored."""
        scored = [d.detail_priority for d in self.decisions if d.detail_priority is not None]
        return max(scored) if scored else None

    def decision_for(self, tile_index: int) -> TileResolutionDecision | None:
        """The decision for ``tile_index``, or ``None`` when the plan omits it."""
        for decision in self.decisions:
            if decision.tile_index == tile_index:
                return decision
        return None
