"""Evaluation result contracts (Phase 11).

What an evaluation *concluded* from a run record, and - just as important -
what it could not conclude and why.

Status, not zero
----------------
Every section carries a :class:`MetricStatus`. A quantity that could not be
computed is ``None`` with a reason beside it; it is never 0, 0.0 or a level.
The distinction Phase 7 drew for risk (``UNKNOWN`` is not ``LOW``, ADR-035)
applies to every number here: an absent measurement that reads as a
measurement is worse than no measurement at all (ADR-051).

Simulation evidence
-------------------
Everything here is computed from a CARLA run. It describes how the pipeline
behaved in a controlled simulation and nothing else - not physical LiDAR, not
real traffic, not safety. Every report carries ``source=SIMULATION`` and a
fixed list of limitations so that the boundary travels with the numbers.
"""

from __future__ import annotations

import math
from datetime import datetime
from enum import StrEnum

import numpy as np
from pydantic import Field, field_validator, model_validator

from adaptx.models.common import AdaptXModel, DataSource, TimestampedModel, Vector3
from adaptx.models.map import ResolutionLevel
from adaptx.models.risk import RiskLevel
from adaptx.scenarios.models import ScenarioState
from adaptx.scenarios.result import SensorConfiguration

#: Below this many samples a 95th percentile is the maximum in disguise, so
#: it is not reported.
MIN_SAMPLES_FOR_P95 = 20

#: Statements that accompany every report. They are the scientific boundary
#: of the whole phase and are not configurable.
LIMITATIONS: tuple[str, ...] = (
    "Simulation evidence only: every figure is computed from a CARLA run and says "
    "nothing about physical LiDAR, real traffic or real-world behaviour.",
    "Not safety validation: nothing here is a safety claim or a certification input.",
    "Not collision-probability validation: the risk score is a heuristic prioritisation "
    "score, not a probability, and no scenario contains a collision.",
    "Not real-world validation: no figure transfers to a deployed vehicle.",
    "Ground-truth actors exclude static scene geometry, so a track without a match is "
    "unlabelled, not a false positive, and no precision figure is reported.",
    "Mapping occupancy accuracy is not evaluated: the record holds no map grid and actor "
    "ground truth is not an occupancy reference.",
    "Timings were measured on the machine that ran the scenario and are not real-time claims.",
)


class MetricStatus(StrEnum):
    """Whether a section of the report holds a measurement.

    ``MEASURED``
        Computed from the evidence as defined.
    ``PARTIAL``
        Computed, but some of the inputs were missing; the reason and the
        counts say how much.
    ``UNAVAILABLE``
        Could not be computed from this record; the reason says why.
    ``NOT_APPLICABLE``
        Does not apply to this scenario - a velocity error with no moving
        actor, for example.
    """

    MEASURED = "measured"
    PARTIAL = "partial"
    UNAVAILABLE = "unavailable"
    NOT_APPLICABLE = "not_applicable"


class Distribution(AdaptXModel):
    """Summary of a set of measured values.

    ``p95`` is NumPy's linear-interpolation percentile and is reported only
    when ``count`` reaches :data:`MIN_SAMPLES_FOR_P95`; otherwise it is
    ``None`` and ``p95_note`` says so. An empty set has ``count=0`` and every
    statistic ``None`` - a distribution of nothing has no mean.
    """

    count: int = Field(ge=0)
    mean: float | None = None
    median: float | None = None
    rmse: float | None = None
    min: float | None = None
    max: float | None = None
    p95: float | None = None
    p95_note: str | None = None

    @classmethod
    def of(cls, values: list[float]) -> Distribution:
        """Summarise ``values``; every value must be finite."""
        if not values:
            return cls(count=0)
        array = np.asarray(values, dtype=np.float64)
        if not np.all(np.isfinite(array)):
            raise ValueError("a distribution cannot summarise non-finite values")
        p95: float | None
        note: str | None
        if array.size >= MIN_SAMPLES_FOR_P95:
            p95 = float(np.percentile(array, 95.0))
            note = None
        else:
            p95 = None
            note = f"needs at least {MIN_SAMPLES_FOR_P95} samples, got {array.size}"
        return cls(
            count=int(array.size),
            mean=float(np.mean(array)),
            median=float(np.median(array)),
            rmse=float(math.sqrt(float(np.mean(array * array)))),
            min=float(np.min(array)),
            max=float(np.max(array)),
            p95=p95,
            p95_note=note,
        )


class EvaluationConfiguration(AdaptXModel):
    """Every threshold the evaluation applies, recorded on every report.

    All values are **baseline engineering choices**, not validated limits, and
    none was chosen by looking at a result. A metric that depends on one is
    reported at every gate in ``match_gates_m`` so no headline rests on a
    single choice (docs/EVALUATION.md §3).
    """

    match_gates_m: list[float] = Field(
        default=[1.0, 2.0, 4.0],
        min_length=1,
        description="Planar gates at which matching is evaluated, metres.",
    )
    primary_gate_m: float = Field(
        default=2.0,
        gt=0.0,
        description="The gate continuity, prediction and risk metrics are computed at.",
    )
    include_map_actors: bool = Field(
        default=False,
        description=(
            "Treat every non-ego simulator actor (traffic signs and lights included) as "
            "eligible ground truth, not only the actors the scenario spawned."
        ),
    )
    proximity_event_m: float = Field(
        default=20.0,
        gt=0.0,
        description="An actor within this planar ground-truth distance is a proximity event.",
    )
    alert_level: RiskLevel = Field(
        default=RiskLevel.HIGH, description="Risk level at or above which a track is alerting."
    )
    refined_level: ResolutionLevel = Field(
        default=ResolutionLevel.MEDIUM,
        description="Level at or above which a tile counts as refined.",
    )
    is_baseline: bool = Field(
        default=True, description="True while these thresholds are untuned baseline values."
    )

    @field_validator("match_gates_m")
    @classmethod
    def _positive_gates(cls, value: list[float]) -> list[float]:
        if any(gate <= 0.0 or not math.isfinite(gate) for gate in value):
            raise ValueError("every match gate must be a positive finite distance")
        if len(set(value)) != len(value):
            raise ValueError("match gates must be distinct")
        return sorted(value)

    @model_validator(mode="after")
    def _primary_is_a_gate(self) -> EvaluationConfiguration:
        if self.primary_gate_m not in self.match_gates_m:
            raise ValueError(
                f"primary_gate_m {self.primary_gate_m} must be one of match_gates_m "
                f"{self.match_gates_m}"
            )
        return self


class Section(AdaptXModel):
    """Common header of every report section."""

    status: MetricStatus
    reason: str | None = Field(
        default=None, description="Why the status is not MEASURED, when it is not."
    )
    warnings: list[str] = Field(default_factory=list)


# ---------------------------------------------------------------------------
# detection and tracking
# ---------------------------------------------------------------------------
class GateMetrics(AdaptXModel):
    """Matching outcome at one gate, pooled over every eligible (actor, frame)."""

    gate_m: float = Field(gt=0.0)
    eligible_pairs: int = Field(ge=0, description="(actor, frame) pairs that could be matched.")
    matched_pairs: int = Field(ge=0)
    match_rate: float | None = Field(
        default=None, ge=0.0, le=1.0, description="matched / eligible; null with no eligible pair."
    )
    class_agreement_rate: float | None = Field(
        default=None,
        ge=0.0,
        le=1.0,
        description="Share of matched pairs whose perceived class equals the ground-truth class.",
    )
    position_error_planar_m: Distribution = Field(description="Over matched pairs, xy only.")
    position_error_3d_m: Distribution = Field(
        description=(
            "Over matched pairs, in the sensor frame. Includes the offset between the "
            "actor's mesh origin and the point-cloud centroid, which is inherent."
        )
    )


class TrackingGateMetrics(GateMetrics):
    """Tracking adds velocity, which detection does not have."""

    velocity_error_mps: Distribution = Field(
        description="|v_track - v_reference| over matched pairs where both exist."
    )
    velocity_null_pairs: int = Field(
        ge=0,
        description=(
            "Matched pairs where the track had no measured velocity. Null is not zero "
            "(ADR-023): these are counted here and excluded from the error, never scored."
        ),
    )
    velocity_reference_missing_pairs: int = Field(
        ge=0, description="Matched pairs on frame 0, where no finite-difference reference exists."
    )


class DetectionEvaluation(Section):
    """Question A, detector alone: is there a detection where the actor is?"""

    gates: list[GateMetrics] = Field(default_factory=list)


class ActorContinuity(AdaptXModel):
    """How consistently one ground-truth actor was represented over the run."""

    actor_id: str = Field(min_length=1, description="Scenario-level name, or the type id.")
    simulator_actor_id: int = Field(ge=0)
    frames_present: int = Field(ge=0)
    frames_eligible: int = Field(ge=0, description="Present, in sensor range and in map bounds.")
    frames_matched: int = Field(ge=0)
    coverage: float | None = Field(
        default=None, ge=0.0, le=1.0, description="matched / eligible; null with no eligible frame."
    )
    track_ids: list[int] = Field(default_factory=list, description="Distinct ids, in order met.")
    id_switches: int = Field(ge=0, description="Distinct track ids minus one, floored at zero.")
    fragments: int = Field(ge=0, description="Maximal runs of consecutive matched frames.")
    longest_fragment_frames: int = Field(ge=0)
    matched_status_counts: dict[str, int] = Field(
        default_factory=dict, description="Lifecycle status of the matched track, per frame."
    )


class TrackingEvaluation(Section):
    """Question A: how well do tracks correspond to ground-truth actors?"""

    gates: list[TrackingGateMetrics] = Field(default_factory=list)
    continuity_gate_m: float | None = None
    continuity: list[ActorContinuity] = Field(default_factory=list)
    unlabelled_track_frames: int = Field(
        ge=0,
        description=(
            "Track-frames with no ground-truth match at the primary gate. Unlabelled, "
            "not false: static scene geometry is not in the ground truth."
        ),
    )
    unlabelled_track_ids: int = Field(ge=0, description="Distinct track ids never matched.")


# ---------------------------------------------------------------------------
# prediction
# ---------------------------------------------------------------------------
class PredictionEvaluation(Section):
    """Question B: ADE / FDE of predicted trajectories against later ground truth."""

    gate_m: float | None = None
    trajectories_in_record: int = Field(ge=0)
    trajectories_evaluated: int = Field(ge=0)
    skipped: dict[str, int] = Field(
        default_factory=dict, description="Trajectories not evaluated, by reason."
    )
    predictor_skips: dict[str, int] = Field(
        default_factory=dict,
        description="Tracks the predictor itself declined, by its own status, from the record.",
    )
    ade_m: Distribution = Field(description="Mean planar displacement per trajectory.")
    fde_m: Distribution = Field(description="Planar displacement at the last aligned point.")
    horizon_coverage: Distribution = Field(
        description="Aligned points / points in the trajectory, per evaluated trajectory."
    )
    error_by_offset_m: dict[str, Distribution] = Field(
        default_factory=dict, description="Planar error at each time offset, pooled."
    )
    ego_stationary: bool | None = Field(
        default=None,
        description=(
            "Whether the ego's ground-truth position was constant over the run. Ego-"
            "relative ground truth on a later frame is only comparable when it was."
        ),
    )


# ---------------------------------------------------------------------------
# risk
# ---------------------------------------------------------------------------
class ActorRiskMetrics(AdaptXModel):
    """Risk behaviour of the track matched to one actor, against proximity events."""

    actor_id: str = Field(min_length=1)
    simulator_actor_id: int = Field(ge=0)
    event_frames: int = Field(ge=0, description="Frames with the actor inside the proximity band.")
    matched_event_frames: int = Field(ge=0, description="Event frames with a matched track.")
    alert_frames_in_event: int = Field(ge=0)
    alert_recall: float | None = Field(
        default=None,
        ge=0.0,
        le=1.0,
        description="alerting / matched event frames; null with no matched event frame.",
    )
    first_event_time_s: float | None = None
    first_alert_time_s: float | None = None
    lead_time_s: float | None = Field(
        default=None,
        description=(
            "first_event_time_s - first_alert_time_s. Positive: the alert preceded the "
            "event. Negative: it followed. Null when either never happened."
        ),
    )
    early_alert_frames: int = Field(
        ge=0, description="Alerting while the actor was outside the band. Not false alerts."
    )
    unknown_risk_frames: int = Field(
        ge=0,
        description="Matched frames whose assessment had no score. Reported, never converted.",
    )
    scored_frames: int = Field(ge=0)
    ordering_pairs: int = Field(ge=0, description="Frame pairs with two scores and two distances.")
    ordering_concordance: float | None = Field(
        default=None,
        ge=0.0,
        le=1.0,
        description=(
            "Share of frame pairs where the higher score coincided with the smaller "
            "distance. 1.0 = perfectly ordered, 0.5 = unrelated. Null below two scored frames."
        ),
    )
    risk_level_counts: dict[str, int] = Field(default_factory=dict)


class RiskEvaluation(Section):
    """Question D, against the only label ground truth supports: proximity."""

    proximity_event_m: float | None = None
    alert_level: str | None = None
    collision_labels_present: bool = Field(
        default=False, description="Always false for the catalogue: no scenario collides."
    )
    actors: list[ActorRiskMetrics] = Field(default_factory=list)
    unlabelled_alert_track_frames: int = Field(
        ge=0, description="Alerting track-frames with no matched actor. Count only."
    )


# ---------------------------------------------------------------------------
# mapping
# ---------------------------------------------------------------------------
class MapWorkload(AdaptXModel):
    """Per-frame workload of one map variant, summarised over the run."""

    variant: str = Field(min_length=1)
    total_cells: Distribution
    occupied_cells: Distribution
    occupancy_ratio: Distribution
    mapped_points: Distribution
    out_of_bounds_points: Distribution
    grid_bytes: Distribution
    duration_ms: Distribution = Field(description="Grid construction only.")


class MappingEvaluation(Section):
    """Question C. Accuracy is explicitly not evaluated (docs/EVALUATION.md §4.5)."""

    accuracy_status: MetricStatus = MetricStatus.UNAVAILABLE
    accuracy_reason: str = Field(min_length=1)
    bounds_m: dict[str, float] = Field(default_factory=dict)
    fixed_resolution_m: float | None = None
    fixed: MapWorkload | None = None
    adaptive: MapWorkload | None = None


# ---------------------------------------------------------------------------
# adaptive resolution
# ---------------------------------------------------------------------------
class TileRefinement(AdaptXModel):
    """One (actor, tile) entry and when, if ever, the tile was refined."""

    actor_id: str = Field(min_length=1)
    tile_index: int = Field(ge=0)
    entered_frame: int = Field(ge=0)
    refined_frame: int | None = Field(
        default=None, description="First frame the tile was at or above the refined level."
    )
    lead_frames: int | None = Field(
        default=None,
        description=(
            "entered_frame - refined_frame. Positive: refined before the actor arrived. "
            "Negative: after. Null: never refined during the run."
        ),
    )


class ChurnMetrics(AdaptXModel):
    """How much the allocation moved from frame to frame."""

    frames: int = Field(ge=0)
    changed_tiles_per_frame: Distribution
    frames_with_any_change: int = Field(ge=0)
    total_transitions: int = Field(ge=0, description="Tile level changes over the run.")
    tiles_ever_changed: int = Field(ge=0)
    reversals: int = Field(
        ge=0,
        description=(
            "Transitions that returned a tile to a level it had left within the "
            "configured dwell window: the signature of oscillation."
        ),
    )
    hysteresis_holds: int = Field(ge=0, description="Decisions the record marks hysteresis_hold.")
    dwell_holds: int = Field(ge=0, description="Decisions the record marks dwell_hold.")
    min_dwell_frames: int | None = None


class AdaptiveResolutionEvaluation(Section):
    """Question E: does Phase 8 change detail as its policy intends, and at what cost?"""

    tile_count: int | None = None
    tile_size_m: float | None = None
    level_cell_sizes_m: dict[str, float] = Field(default_factory=dict)
    tile_resolution_m: Distribution = Field(description="Pooled over every tile of every frame.")
    unique_resolutions_per_frame: Distribution
    area_weighted_resolution_m: Distribution = Field(description="Per frame.")
    fixed_cells: Distribution
    adaptive_cells: Distribution
    cell_ratio: Distribution = Field(description="adaptive / fixed cells, per frame.")
    byte_ratio: Distribution = Field(description="adaptive / fixed grid bytes, per frame.")
    mapping_duration_ratio: Distribution = Field(
        description="adaptive grid construction / fixed grid construction, per frame."
    )
    total_duration_ratio: Distribution = Field(
        description="(controller + adaptive mapping) / fixed mapping, per frame."
    )
    actor_tile_levels: dict[str, int] = Field(
        default_factory=dict, description="Level of the tile holding an actor, per (actor, frame)."
    )
    other_tile_levels: dict[str, int] = Field(
        default_factory=dict, description="Level of every tile holding no actor, per frame."
    )
    actor_tile_resolution_m: Distribution
    other_tile_resolution_m: Distribution
    actor_tile_resolution_by_risk_m: dict[str, Distribution] = Field(
        default_factory=dict,
        description="Actor-tile cell size grouped by the matched track's risk level.",
    )
    refined_level: str | None = None
    refinements: list[TileRefinement] = Field(default_factory=list)
    refinement_lead_frames: Distribution = Field(description="Over entries that were refined.")
    entries_never_refined: int = Field(ge=0)
    churn: ChurnMetrics | None = None
    frames_over_budget: int = Field(ge=0)
    demoted_tiles_total: int = Field(ge=0)


# ---------------------------------------------------------------------------
# resource
# ---------------------------------------------------------------------------
class ResourceEvaluation(Section):
    """Measured cost on the machine that ran the scenario. Not a real-time claim."""

    stage_ms: dict[str, Distribution] = Field(default_factory=dict)
    pipeline_ms: Distribution = Field(description="Sum of every stage, per frame.")
    fixed_grid_bytes: Distribution
    adaptive_grid_bytes: Distribution
    peak_memory_status: MetricStatus = MetricStatus.UNAVAILABLE
    peak_memory_reason: str = Field(min_length=1)


# ---------------------------------------------------------------------------
# the report
# ---------------------------------------------------------------------------
class EvaluationReport(TimestampedModel):
    """Everything one evaluation concluded, with everything needed to redo it."""

    evaluation_id: str = Field(min_length=1)
    scenario_id: str = Field(min_length=1)
    scenario_name: str = Field(min_length=1)
    seed: int = Field(ge=0)
    run_state: ScenarioState
    run_timestamp: datetime
    map_name: str | None = None
    simulator_version: str = Field(min_length=1)
    fixed_delta_seconds: float = Field(gt=0.0)
    frame_count: int = Field(ge=0)
    planned_frame_count: int = Field(ge=0)
    evaluated_frame_count: int = Field(ge=0, description="Frames carrying pipeline outputs.")
    sensor: SensorConfiguration | None = None
    sensor_mount: Vector3 | None = None
    adaptx_version: str = Field(min_length=1)
    git_commit: str | None = Field(
        default=None, description="Commit of the evaluating checkout, when one could be read."
    )
    configuration: EvaluationConfiguration
    pipeline_configuration: dict[str, dict[str, object]] = Field(
        default_factory=dict,
        description="Effective stage configurations, from the first evaluated frame's outputs.",
    )
    scenario_actors: list[str] = Field(default_factory=list)

    detection: DetectionEvaluation
    tracking: TrackingEvaluation
    prediction: PredictionEvaluation
    risk: RiskEvaluation
    mapping: MappingEvaluation
    adaptive_resolution: AdaptiveResolutionEvaluation
    resource: ResourceEvaluation

    warnings: list[str] = Field(default_factory=list)
    limitations: list[str] = Field(default_factory=lambda: list(LIMITATIONS))
    source: DataSource = Field(
        default=DataSource.SIMULATION,
        description="Always SIMULATION: an evaluation of a simulated run is simulated evidence.",
    )

    @property
    def sections(self) -> dict[str, Section]:
        """Every section by name, for rendering and comparison."""
        return {
            "detection": self.detection,
            "tracking": self.tracking,
            "prediction": self.prediction,
            "risk": self.risk,
            "mapping": self.mapping,
            "adaptive_resolution": self.adaptive_resolution,
            "resource": self.resource,
        }


__all__ = [
    "LIMITATIONS",
    "MIN_SAMPLES_FOR_P95",
    "ActorContinuity",
    "ActorRiskMetrics",
    "AdaptiveResolutionEvaluation",
    "ChurnMetrics",
    "DetectionEvaluation",
    "Distribution",
    "EvaluationConfiguration",
    "EvaluationReport",
    "GateMetrics",
    "MapWorkload",
    "MappingEvaluation",
    "MetricStatus",
    "PredictionEvaluation",
    "ResourceEvaluation",
    "RiskEvaluation",
    "Section",
    "TileRefinement",
    "TrackingEvaluation",
    "TrackingGateMetrics",
]
