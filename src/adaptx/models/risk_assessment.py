"""Risk and uncertainty assessment contracts (Phase 7).

What this layer answers
-----------------------
*How concerning is this object right now, and how sure are we?*

It does **not** answer *how much spatial detail should this region receive* -
that is the resolution policy of Phase 8, which consumes these assessments
(ADR-036). Nothing here carries a cell size, a resolution level or a
:class:`~adaptx.models.resolution.ResolutionDecision`.

Risk and uncertainty are separate quantities
--------------------------------------------
The pre-existing contract in :mod:`adaptx.models.risk` states it plainly: an
object can be low-risk but uncertain, or high-risk and well observed. Phase 7
keeps them separate rather than folding uncertainty into the score (ADR-033).

That separation is load-bearing for Phase 8. A poorly observed region may
deserve finer perception *precisely because* it is poorly observed, even when
its computed risk is low. Summing uncertainty into risk would destroy the
signal a resolution controller most needs.

Everything here is heuristic
----------------------------
The score is a **deterministic engineering heuristic**, not a probability of
collision, not calibrated, and not validated against any labelled risk dataset -
none exists. Thresholds are baseline engineering values, not safety-certified
limits.
"""

from __future__ import annotations

import math
from enum import StrEnum

from pydantic import Field, field_validator, model_validator

from adaptx.models.common import AdaptXModel, DataSource, TimestampedModel
from adaptx.models.risk import RiskFactors, RiskLevel
from adaptx.models.tracking import TrackStatus


class RiskFactorName(StrEnum):
    """A named contribution that was actually computed for an assessment.

    Only factors the engine genuinely calculates appear here. A factor absent
    from an assessment's list was **not computed**, which is different from
    computed-and-zero; the numeric fields distinguish the two.
    """

    PROXIMITY = "proximity"
    CLOSING_SPEED = "closing_speed"
    PREDICTED_PROXIMITY = "predicted_proximity"


class UncertaintyReason(StrEnum):
    """Why an assessment is less trustworthy than a fully observed one.

    These are the *sources* of uncertainty, kept visible alongside the scalar
    so a consumer can tell a stale track from an unconfirmed one.
    """

    UNKNOWN_VELOCITY = "unknown_velocity"
    STALE_OBSERVATION = "stale_observation"
    LOW_TRACK_CONFIDENCE = "low_track_confidence"
    NO_PREDICTION = "no_prediction"
    WIDE_PREDICTION_UNCERTAINTY = "wide_prediction_uncertainty"
    TENTATIVE_TRACK = "tentative_track"
    COASTING_TRACK = "coasting_track"
    UNOBSERVED_MAP_CONTEXT = "unobserved_map_context"
    NO_MAP = "no_map"


class AssessmentStatus(StrEnum):
    """Outcome of attempting to assess one track.

    ``ASSESSED``
        A score was computed from at least one available factor.
    ``INSUFFICIENT_DATA``
        Nothing could be computed. Reported as ``RiskLevel.UNKNOWN`` with
        ``risk_score = None`` rather than a fabricated number.
    ``TRACK_LOST``
        The track is terminated. A stale track is not an active concern, and
        scoring it would present history as a present threat.
    """

    ASSESSED = "assessed"
    INSUFFICIENT_DATA = "insufficient_data"
    TRACK_LOST = "track_lost"


class MapObservation(StrEnum):
    """What the 2.5D map says about the cell an object occupies.

    The distinction that matters most is the last two: Phase 6 records where
    LiDAR returns landed, and **an empty cell is not free space** (ADR-031). A
    cell may be empty because nothing is there, or because something occluded
    it, and the map cannot tell the difference.

    ``OBSERVED_OCCUPIED``
        The cell holds at least one measured return.
    ``OBSERVED_EMPTY``
        The cell is inside the map and holds no return. **Not** a statement
        that the space is clear.
    ``OUT_OF_BOUNDS``
        The object lies outside the mapped extent, so the map says nothing.
    ``NO_MAP``
        No map was supplied to the assessment.
    """

    OBSERVED_OCCUPIED = "observed_occupied"
    OBSERVED_EMPTY = "observed_empty"
    OUT_OF_BOUNDS = "out_of_bounds"
    NO_MAP = "no_map"


class TrajectoryRelevance(AdaptXModel):
    """How close a predicted path comes to the ego reference point.

    **This is not a collision detector and does not prove a collision.** It is
    a spatial-temporal relevance signal derived from the Phase 5
    constant-velocity extrapolation, whose own uncertainty is a documented
    heuristic (ADR-026). A predicted approach means the extrapolation passes
    near the ego origin - nothing more.
    """

    min_distance_m: float = Field(
        ge=0.0, description="Smallest planar distance to the ego reference over the path."
    )
    time_to_min_distance_s: float = Field(
        ge=0.0, description="Time offset at which that minimum occurs."
    )
    uncertainty_at_min_m: float = Field(
        ge=0.0,
        description=(
            "Heuristic positional uncertainty of the trajectory at that point, "
            "carried through from Phase 5. Not a calibrated sigma."
        ),
    )
    horizon_s: float = Field(gt=0.0, description="Horizon the trajectory covered.")
    is_approaching: bool = Field(
        description="True when the closest predicted point is nearer than the current position."
    )

    @field_validator("min_distance_m", "time_to_min_distance_s", "uncertainty_at_min_m")
    @classmethod
    def _require_finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("trajectory relevance values must be finite")
        return value


class MapContext(AdaptXModel):
    """What the spatial map records where an object currently is.

    Deliberately narrow. It reports an observation, and the engine never uses
    it to *lower* risk: absence of returns is unobserved, not safe (ADR-034).
    """

    observation: MapObservation
    point_count: int | None = Field(
        default=None,
        ge=0,
        description="Measured returns in the object's cell; null when no map or out of bounds.",
    )
    max_height_m: float | None = Field(
        default=None, description="Greatest measured height in that cell; null when unobserved."
    )

    @property
    def is_observed(self) -> bool:
        """Whether the map actually saw the object's cell."""
        return self.observation is MapObservation.OBSERVED_OCCUPIED


class UncertaintyBreakdown(AdaptXModel):
    """How much the engine distrusts an assessment, and why.

    ``score`` is a **heuristic** on ``[0, 1]``: 0 means every input the engine
    wanted was present and fresh, 1 means essentially none were. It is not a
    variance, not a probability and not calibrated - no labelled data exists to
    calibrate it against (ADR-033).

    ``reasons`` keeps the contributors visible so a consumer can act on the
    cause rather than the number.
    """

    score: float = Field(ge=0.0, le=1.0, description="Heuristic uncertainty in [0, 1].")
    reasons: list[UncertaintyReason] = Field(default_factory=list)
    observation_age_s: float = Field(
        ge=0.0, description="Measured seconds since the track was last observed."
    )
    is_stale: bool = Field(
        description="True when the observation is older than the configured staleness bound."
    )
    velocity_known: bool = Field(
        description="False when velocity was never measured. Null velocity is not zero (ADR-023)."
    )
    prediction_available: bool
    track_confidence: float = Field(
        ge=0.0,
        le=1.0,
        description=(
            "Carried through unchanged from Phase 3/4. A **geometric fit score**, "
            "not a probability that the classification is correct (ADR-021)."
        ),
    )

    @property
    def is_confident(self) -> bool:
        """Whether nothing reduced trust in this assessment."""
        return not self.reasons


class RiskConfiguration(AdaptXModel):
    """Effective risk configuration that produced a result.

    Carried on every result so a record is self-describing - the same rule
    processing, detection, tracking, prediction and mapping already follow.
    All values are **baseline engineering values**, not validated safety limits.
    """

    proximity_near_m: float
    proximity_far_m: float
    closing_speed_high_mps: float
    stale_observation_s: float
    low_track_confidence: float
    threshold_medium: float
    threshold_high: float
    threshold_critical: float
    weight_proximity: float
    weight_closing_speed: float
    weight_predicted_proximity: float


class RiskAssessment(TimestampedModel):
    """Deterministic heuristic risk and uncertainty for one tracked object.

    ``risk_score`` is ``None`` exactly when ``risk_level`` is
    :attr:`~adaptx.models.risk.RiskLevel.UNKNOWN`. Nothing is invented to fill
    the gap.
    """

    track_id: int = Field(ge=0)
    status: AssessmentStatus
    risk_level: RiskLevel
    risk_score: float | None = Field(
        default=None,
        ge=0.0,
        le=1.0,
        description=(
            "Heuristic engineering score in [0, 1], or null when nothing could "
            "be computed. NOT a probability of collision."
        ),
    )

    distance_m: float = Field(ge=0.0, description="Planar distance to the ego reference.")
    closing_speed_mps: float | None = Field(
        default=None,
        description=(
            "Rate of approach along the line to the ego reference; negative "
            "means receding. Null when velocity was never measured."
        ),
    )
    speed_mps: float | None = Field(
        default=None, ge=0.0, description="Track speed, or null when unmeasured."
    )

    track_status: TrackStatus
    object_class: str = Field(min_length=1)

    trajectory: TrajectoryRelevance | None = Field(
        default=None, description="Null when the track had no predicted trajectory."
    )
    map_context: MapContext
    uncertainty: UncertaintyBreakdown

    factors: list[RiskFactorName] = Field(
        default_factory=list, description="Factors actually computed for this assessment."
    )
    factor_scores: RiskFactors = Field(
        description="Normalised value of each computed factor, for attribution."
    )
    reason: str = Field(
        min_length=1,
        description="Explanation generated from the computed factors, never free-form text.",
    )
    source: DataSource = DataSource.LIVE_SENSOR

    @model_validator(mode="after")
    def _check_score_matches_level(self) -> RiskAssessment:
        unknown = self.risk_level is RiskLevel.UNKNOWN
        if unknown and self.risk_score is not None:
            raise ValueError("an UNKNOWN assessment must not carry a risk_score")
        if not unknown and self.risk_score is None:
            raise ValueError(f"a {self.risk_level.value} assessment requires a risk_score")
        if unknown and self.status is AssessmentStatus.ASSESSED:
            raise ValueError("an assessed track cannot have risk_level UNKNOWN")
        return self

    @property
    def is_assessed(self) -> bool:
        """Whether a score was actually produced."""
        return self.status is AssessmentStatus.ASSESSED


class RiskAssessmentResult(TimestampedModel):
    """Everything one risk pass produced, plus the scene-level aggregate.

    Every track handed to the engine appears in ``assessments`` - including
    those that could not be scored, which carry ``UNKNOWN``. "Nothing is risky"
    and "nothing could be assessed" stay distinguishable.
    """

    frame_id: int = Field(ge=0)
    sensor_id: str = Field(min_length=1)
    engine: str = Field(min_length=1, description="Identifier of the engine that ran.")
    is_baseline: bool = Field(
        default=True,
        description="True while risk is a deterministic heuristic rather than a learned model.",
    )
    scoring_model: str = Field(
        default="heuristic_weighted_factors",
        description="Identifier of the scoring formulation. Heuristic, uncalibrated.",
    )

    assessments: list[RiskAssessment] = Field(default_factory=list)
    considered_track_count: int = Field(ge=0)
    duration_ms: float = Field(ge=0.0, description="Whole risk pass, measured.")
    configuration: RiskConfiguration

    @model_validator(mode="after")
    def _check_accounting(self) -> RiskAssessmentResult:
        if self.considered_track_count != len(self.assessments):
            raise ValueError(
                f"considered_track_count ({self.considered_track_count}) must equal "
                f"the number of assessments ({len(self.assessments)})"
            )
        return self

    def counts_by_level(self) -> dict[str, int]:
        """How many tracks landed on each level, including ``unknown``."""
        counts: dict[str, int] = {}
        for assessment in self.assessments:
            key = assessment.risk_level.value
            counts[key] = counts.get(key, 0) + 1
        return counts

    @property
    def scored_assessments(self) -> list[RiskAssessment]:
        """Assessments that produced a score."""
        return [a for a in self.assessments if a.risk_score is not None]

    @property
    def highest_risk_level(self) -> RiskLevel:
        """The most concerning **scored** level present.

        Aggregation is a **maximum, never a mean** (ADR-035). Averaging would
        let one critical object vanish behind ten quiet ones, which is the
        opposite of what a scene-level signal is for.

        Returns ``UNKNOWN`` when nothing was scored - which is itself
        informative, and is not the same as ``LOW``.
        """
        scored = self.scored_assessments
        if not scored:
            return RiskLevel.UNKNOWN
        order = {
            RiskLevel.LOW: 0,
            RiskLevel.MEDIUM: 1,
            RiskLevel.HIGH: 2,
            RiskLevel.CRITICAL: 3,
        }
        return max((a.risk_level for a in scored), key=lambda level: order[level])

    @property
    def highest_risk_score(self) -> float | None:
        """The largest score present, or ``None`` when nothing was scored."""
        scores = [a.risk_score for a in self.scored_assessments if a.risk_score is not None]
        return max(scores) if scores else None

    @property
    def max_uncertainty(self) -> float | None:
        """The largest heuristic uncertainty present, or ``None`` with no tracks.

        Reported alongside risk rather than inside it: a quiet but poorly
        observed scene is a different situation from a quiet, well observed one.
        """
        if not self.assessments:
            return None
        return max(a.uncertainty.score for a in self.assessments)

    @property
    def unknown_count(self) -> int:
        """Tracks that could not be scored."""
        return sum(1 for a in self.assessments if a.risk_score is None)
