"""Deterministic heuristic risk and uncertainty engine (Phase 7).

Consumes the Phase 4 tracks, the Phase 5 trajectories and the Phase 6 map, and
answers one question per object:

    *How concerning is this object right now, and how sure are we?*

    tracks + trajectories + map -> factors -> weighted score -> level
                                -> uncertainty breakdown -> RiskAssessment

**This is a deterministic engineering heuristic.** It is not a probability of
collision, not a calibrated model, not validated against any labelled risk
dataset, and not a safety certification. The thresholds are baseline
engineering values (ADR-032).

What this module must never do
------------------------------
It does **not** choose spatial resolution, and it never will. It imports no
resolution type and produces no ``ResolutionDecision``. Risk says *how
concerning*; a resolution controller decides *how much detail* - two different
decisions, kept in two different phases (ADR-036).

It does read ``SpatialMap.resolution_m``, for one purpose: finding which cell an
object falls in. That is reading the geometry of a map it was handed, not
selecting a cell size, and no risk value influences it.

Scoring
-------
Three factors, each normalised to ``[0, 1]``, combined as a weighted mean over
the ones actually available::

    risk_score = sum(w_i * f_i) / sum(w_i)   over available i only

A factor that could not be computed is **dropped and the remaining weights
renormalise** - it is never silently treated as zero (ADR-032). A missing
velocity means unknown motion, not a standstill; scoring it as zero would make
an unmeasured object look calm.

If no factor at all can be computed, the object is reported ``UNKNOWN`` with
``risk_score = None`` rather than given a fabricated number.

Uncertainty is reported, not scored
-----------------------------------
Uncertainty is **not** a term in the risk score. The pre-existing contract in
:mod:`adaptx.models.risk` keeps the two quantities separate, and Phase 8 needs
them separate: a poorly observed region may deserve finer perception precisely
because it is poorly observed, even when its computed risk is low (ADR-033).
"""

from __future__ import annotations

import math
import time
from datetime import datetime

from adaptx.config.settings import RiskSettings
from adaptx.core.logging import get_logger
from adaptx.models.common import DataSource, ObjectClass, Vector3, utc_now
from adaptx.models.prediction import PredictedTrajectory
from adaptx.models.risk import ObjectRisk, RiskFactors, RiskField, RiskLevel
from adaptx.models.risk_assessment import (
    AssessmentStatus,
    MapContext,
    MapObservation,
    RiskAssessment,
    RiskAssessmentResult,
    RiskConfiguration,
    RiskFactorName,
    TrajectoryRelevance,
    UncertaintyBreakdown,
    UncertaintyReason,
)
from adaptx.models.spatial_map import SpatialMap
from adaptx.models.tracking import TrackedObject, TrackStatus
from adaptx.models.vehicle import VehicleState
from adaptx.risk.interfaces import RiskEngine

logger = get_logger(__name__)

#: Identifier of the scoring formulation, recorded on every result.
SCORING_MODEL = "heuristic_weighted_factors"

#: Weight each uncertainty reason contributes before the total is clamped to
#: ``[0, 1]``. Baseline engineering values: an unmeasured velocity is the
#: single largest blind spot, so it dominates.
_UNCERTAINTY_WEIGHTS: dict[UncertaintyReason, float] = {
    UncertaintyReason.UNKNOWN_VELOCITY: 0.35,
    UncertaintyReason.NO_PREDICTION: 0.20,
    UncertaintyReason.STALE_OBSERVATION: 0.15,
    UncertaintyReason.COASTING_TRACK: 0.15,
    UncertaintyReason.LOW_TRACK_CONFIDENCE: 0.10,
    UncertaintyReason.TENTATIVE_TRACK: 0.10,
    UncertaintyReason.WIDE_PREDICTION_UNCERTAINTY: 0.10,
    UncertaintyReason.UNOBSERVED_MAP_CONTEXT: 0.05,
    UncertaintyReason.NO_MAP: 0.05,
}


class HeuristicRiskEngine(RiskEngine):
    """Scores tracked objects by proximity, approach rate and predicted approach."""

    name = "heuristic_risk_v1"
    #: True: a deterministic heuristic, not a learned or calibrated model.
    is_baseline = True

    def __init__(self, settings: RiskSettings) -> None:
        self._settings = settings

    # -- state -------------------------------------------------------------
    @property
    def configuration(self) -> RiskConfiguration:
        """Snapshot of the settings that shape this engine's behaviour."""
        s = self._settings
        return RiskConfiguration(
            proximity_near_m=s.proximity_near_m,
            proximity_far_m=s.proximity_far_m,
            closing_speed_high_mps=s.closing_speed_high_mps,
            stale_observation_s=s.stale_observation_s,
            low_track_confidence=s.low_track_confidence,
            threshold_medium=s.threshold_medium,
            threshold_high=s.threshold_high,
            threshold_critical=s.threshold_critical,
            weight_proximity=s.weight_proximity,
            weight_closing_speed=s.weight_closing_speed,
            weight_predicted_proximity=s.weight_predicted_proximity,
        )

    def classify(self, risk_score: float) -> RiskLevel:
        """Partition ``[0, 1]`` using the configured thresholds.

        Implements the pre-existing :class:`~adaptx.risk.interfaces.RiskEngine`
        contract, and uses the same thresholds as the proximity baseline so the
        two engines remain directly comparable.
        """
        if not 0.0 <= risk_score <= 1.0:
            raise ValueError(f"risk_score must be in [0, 1], got {risk_score}")
        if risk_score >= self._settings.threshold_critical:
            return RiskLevel.CRITICAL
        if risk_score >= self._settings.threshold_high:
            return RiskLevel.HIGH
        if risk_score >= self._settings.threshold_medium:
            return RiskLevel.MEDIUM
        return RiskLevel.LOW

    # -- contract compatibility --------------------------------------------
    def evaluate(
        self,
        tracks: list[TrackedObject],
        *,
        ego_state: VehicleState | None = None,
        trajectories: list[PredictedTrajectory] | None = None,
        timestamp: datetime | None = None,
    ) -> RiskField:
        """Return a :class:`RiskField`, satisfying the Phase 1 contract.

        Provided so this engine and
        :class:`~adaptx.risk.baseline.BaselineProximityRiskEngine` can be
        swapped and compared through one interface. It projects the richer
        assessment down to the older per-object contract; callers wanting the
        full detail use :meth:`assess_many`.

        Produces no ``cells``: a spatial risk field would need a per-cell
        formulation this phase does not implement.
        """
        result = self.assess_many(
            tracks,
            trajectories=trajectories,
            ego_state=ego_state,
            timestamp=timestamp,
        )
        return RiskField(
            timestamp=result.timestamp,
            engine=self.name,
            is_baseline=self.is_baseline,
            cells=[],
            object_risks=[
                ObjectRisk(
                    timestamp=assessment.timestamp,
                    track_id=assessment.track_id,
                    # The contract requires a score; UNKNOWN assessments are
                    # omitted rather than given an invented one.
                    risk_score=assessment.risk_score or 0.0,
                    risk_level=assessment.risk_level,
                    uncertainty=assessment.uncertainty.score,
                    factors=assessment.factor_scores,
                    source=assessment.source,
                )
                for assessment in result.assessments
                if assessment.risk_score is not None
            ],
            source=(ego_state.source if ego_state is not None else DataSource.UNAVAILABLE),
        )

    # -- assessment --------------------------------------------------------
    def assess(
        self,
        track: TrackedObject,
        *,
        trajectory: PredictedTrajectory | None = None,
        spatial_map: SpatialMap | None = None,
        ego_state: VehicleState | None = None,
        timestamp: datetime | None = None,
    ) -> RiskAssessment:
        """Assess one track. Deterministic: same inputs, same assessment."""
        settings = self._settings
        origin = ego_state.position if ego_state is not None else Vector3()
        now = timestamp if timestamp is not None else track.timestamp

        distance = _planar_distance(track.position, origin)
        age_s = _observation_age_s(track, now)
        is_stale = age_s > settings.stale_observation_s
        velocity = track.velocity
        speed = track.speed_mps

        map_context = self._map_context(track, spatial_map)
        uncertainty = self._uncertainty(
            track,
            trajectory=trajectory,
            map_context=map_context,
            age_s=age_s,
            is_stale=is_stale,
        )

        # A terminated track is history, not a present concern. Scoring it
        # would report a stale position as an active threat.
        if track.status is TrackStatus.LOST:
            return self._unassessed(
                track,
                status=AssessmentStatus.TRACK_LOST,
                distance=distance,
                speed=speed,
                map_context=map_context,
                uncertainty=uncertainty,
                reason="Not assessed: the track is terminated and is not an active object.",
                now=now,
            )

        proximity = _linear_falloff(distance, settings.proximity_near_m, settings.proximity_far_m)
        closing_speed = _closing_speed(track.position, velocity, origin)
        closing_factor = (
            None
            if closing_speed is None
            else _clamp(max(0.0, closing_speed) / settings.closing_speed_high_mps)
        )
        relevance = self._trajectory_relevance(trajectory, origin, distance)
        predicted_factor = (
            None
            if relevance is None
            else _linear_falloff(
                relevance.min_distance_m,
                settings.predicted_proximity_near_m,
                settings.proximity_far_m,
            )
        )

        weighted: list[tuple[RiskFactorName, float, float]] = []
        if proximity is not None:
            weighted.append((RiskFactorName.PROXIMITY, proximity, settings.weight_proximity))
        if closing_factor is not None:
            weighted.append(
                (RiskFactorName.CLOSING_SPEED, closing_factor, settings.weight_closing_speed)
            )
        if predicted_factor is not None:
            weighted.append(
                (
                    RiskFactorName.PREDICTED_PROXIMITY,
                    predicted_factor,
                    settings.weight_predicted_proximity,
                )
            )

        # Only factors with a non-zero weight can contribute to the mean.
        contributing = [entry for entry in weighted if entry[2] > 0.0]
        if not contributing:
            return self._unassessed(
                track,
                status=AssessmentStatus.INSUFFICIENT_DATA,
                distance=distance,
                speed=speed,
                map_context=map_context,
                uncertainty=uncertainty,
                reason=(
                    "Not assessed: no risk factor could be computed from the "
                    "available track, prediction and map data."
                ),
                now=now,
            )

        total_weight = sum(weight for _, _, weight in contributing)
        score = _clamp(sum(value * weight for _, value, weight in contributing) / total_weight)
        level = self.classify(score)
        factor_names = [name for name, _, _ in contributing]

        return RiskAssessment(
            timestamp=now,
            track_id=track.track_id,
            status=AssessmentStatus.ASSESSED,
            risk_level=level,
            risk_score=score,
            distance_m=distance,
            closing_speed_mps=closing_speed,
            speed_mps=speed,
            track_status=track.status,
            object_class=track.object_class.value,
            trajectory=relevance,
            map_context=map_context,
            uncertainty=uncertainty,
            factors=factor_names,
            factor_scores=RiskFactors(
                proximity=proximity,
                relative_velocity=closing_factor,
                trajectory_overlap=predicted_factor,
                uncertainty=uncertainty.score,
            ),
            reason=_explain(
                level=level,
                distance=distance,
                closing_speed=closing_speed,
                relevance=relevance,
                uncertainty=uncertainty,
                map_context=map_context,
                object_class=track.object_class,
            ),
            source=track.source,
        )

    def assess_many(
        self,
        tracks: list[TrackedObject],
        *,
        trajectories: list[PredictedTrajectory] | None = None,
        spatial_map: SpatialMap | None = None,
        ego_state: VehicleState | None = None,
        timestamp: datetime | None = None,
        frame_id: int = 0,
        sensor_id: str = "unknown",
    ) -> RiskAssessmentResult:
        """Assess every track and aggregate the scene.

        Trajectories are matched to tracks by ``track_id``; a track without one
        is assessed without the predicted-proximity factor rather than skipped.

        Args:
            tracks: Live tracks from a tracking update, in any order.
            trajectories: Phase 5 output, matched by ``track_id``.
            spatial_map: Phase 6 output, for map context. Optional.
            ego_state: Ego reference. Defaults to the coordinate-frame origin.
            timestamp: Assessment time, used for staleness. Defaults to each
                track's own timestamp.
            frame_id: Frame identifier recorded on the result.
            sensor_id: Sensor identifier recorded on the result.
        """
        started = time.perf_counter()
        by_track = {trajectory.track_id: trajectory for trajectory in (trajectories or [])}
        limit = self._settings.max_assessed_tracks

        assessments = [
            self.assess(
                track,
                trajectory=by_track.get(track.track_id),
                spatial_map=spatial_map,
                ego_state=ego_state,
                timestamp=timestamp,
            )
            for track in tracks[:limit]
        ]
        duration_ms = (time.perf_counter() - started) * 1000.0

        # Anchor the result to the frame that produced the tracks. With no
        # timestamp and no tracks there is nothing to anchor to, so the
        # assessment time is now - an empty scene assessed at the current
        # moment, which is exactly what happened.
        if timestamp is not None:
            resolved_timestamp = timestamp
        elif tracks:
            resolved_timestamp = tracks[0].timestamp
        else:
            resolved_timestamp = utc_now()

        result = RiskAssessmentResult(
            timestamp=resolved_timestamp,
            frame_id=frame_id,
            sensor_id=sensor_id,
            engine=self.name,
            is_baseline=self.is_baseline,
            scoring_model=SCORING_MODEL,
            assessments=assessments,
            considered_track_count=len(assessments),
            duration_ms=duration_ms,
            configuration=self.configuration,
        )

        logger.debug(
            "risk assessed",
            extra={
                "context": {
                    "frame_id": frame_id,
                    "tracks": len(tracks),
                    "assessed": len(assessments),
                    "highest": result.highest_risk_level.value,
                    "duration_ms": round(duration_ms, 3),
                }
            },
        )
        return result

    # -- internals ---------------------------------------------------------
    def _unassessed(
        self,
        track: TrackedObject,
        *,
        status: AssessmentStatus,
        distance: float,
        speed: float | None,
        map_context: MapContext,
        uncertainty: UncertaintyBreakdown,
        reason: str,
        now: datetime,
    ) -> RiskAssessment:
        """An assessment that deliberately carries no score."""
        return RiskAssessment(
            timestamp=now,
            track_id=track.track_id,
            status=status,
            risk_level=RiskLevel.UNKNOWN,
            risk_score=None,
            distance_m=distance,
            closing_speed_mps=None,
            speed_mps=speed,
            track_status=track.status,
            object_class=track.object_class.value,
            trajectory=None,
            map_context=map_context,
            uncertainty=uncertainty,
            factors=[],
            factor_scores=RiskFactors(uncertainty=uncertainty.score),
            reason=reason,
            source=track.source,
        )

    def _uncertainty(
        self,
        track: TrackedObject,
        *,
        trajectory: PredictedTrajectory | None,
        map_context: MapContext,
        age_s: float,
        is_stale: bool,
    ) -> UncertaintyBreakdown:
        """Collect the reasons this assessment is less than fully trustworthy."""
        settings = self._settings
        reasons: list[UncertaintyReason] = []

        if track.velocity is None:
            reasons.append(UncertaintyReason.UNKNOWN_VELOCITY)
        if is_stale:
            reasons.append(UncertaintyReason.STALE_OBSERVATION)
        if track.confidence <= settings.low_track_confidence:
            reasons.append(UncertaintyReason.LOW_TRACK_CONFIDENCE)
        if track.status is TrackStatus.TENTATIVE:
            reasons.append(UncertaintyReason.TENTATIVE_TRACK)
        if track.status is TrackStatus.COASTING:
            reasons.append(UncertaintyReason.COASTING_TRACK)

        if trajectory is None:
            reasons.append(UncertaintyReason.NO_PREDICTION)
        else:
            widest = trajectory.points[-1].position_uncertainty_m
            # "Wide" means the heuristic uncertainty has grown past the
            # near-proximity distance, i.e. the prediction can no longer
            # distinguish near from not-near.
            if widest > settings.predicted_proximity_near_m:
                reasons.append(UncertaintyReason.WIDE_PREDICTION_UNCERTAINTY)

        if map_context.observation is MapObservation.NO_MAP:
            reasons.append(UncertaintyReason.NO_MAP)
        elif map_context.observation in (
            MapObservation.OBSERVED_EMPTY,
            MapObservation.OUT_OF_BOUNDS,
        ):
            # No returns where the object is said to be. That is a reason to
            # trust the picture less, never a reason to call the space free.
            reasons.append(UncertaintyReason.UNOBSERVED_MAP_CONTEXT)

        score = _clamp(sum(_UNCERTAINTY_WEIGHTS[reason] for reason in reasons))
        return UncertaintyBreakdown(
            score=score,
            reasons=reasons,
            observation_age_s=age_s,
            is_stale=is_stale,
            velocity_known=track.velocity is not None,
            prediction_available=trajectory is not None,
            track_confidence=track.confidence,
        )

    def _trajectory_relevance(
        self,
        trajectory: PredictedTrajectory | None,
        origin: Vector3,
        current_distance: float,
    ) -> TrajectoryRelevance | None:
        """Closest approach of a predicted path to the ego reference.

        **Not a collision test.** It reports where a constant-velocity
        extrapolation passes closest, and how uncertain that extrapolation had
        become by then.
        """
        if trajectory is None:
            return None

        best = min(
            trajectory.points,
            key=lambda point: (_planar_distance(point.position, origin), point.time_offset_s),
        )
        minimum = _planar_distance(best.position, origin)
        return TrajectoryRelevance(
            min_distance_m=minimum,
            time_to_min_distance_s=best.time_offset_s,
            uncertainty_at_min_m=best.position_uncertainty_m,
            horizon_s=trajectory.horizon_s,
            is_approaching=minimum < current_distance,
        )

    @staticmethod
    def _map_context(track: TrackedObject, spatial_map: SpatialMap | None) -> MapContext:
        """What the map recorded where this object is.

        Never used to reduce risk. An empty cell means no returns landed there,
        which may be because nothing is present or because something occluded
        it - Phase 6 cannot tell the difference (ADR-031, ADR-034).
        """
        if spatial_map is None:
            return MapContext(observation=MapObservation.NO_MAP)

        bounds = spatial_map.bounds
        x, y = track.position.x, track.position.y
        if not bounds.contains(x, y):
            return MapContext(observation=MapObservation.OUT_OF_BOUNDS)

        # Mirrors the mapper's half-open indexing (ADR-028). `int` is exact
        # here rather than merely close to `floor`, because the bounds check
        # above guarantees the offsets are non-negative. The clamp guards the
        # same ceil-derived edge cell the mapper clips.
        size = spatial_map.resolution_m
        column = min(int((x - bounds.min_x) / size), spatial_map.width - 1)
        row = min(int((y - bounds.min_y) / size), spatial_map.height - 1)
        count = int(spatial_map.point_count[row, column])
        if count == 0:
            return MapContext(observation=MapObservation.OBSERVED_EMPTY, point_count=0)

        height = float(spatial_map.max_height_m[row, column])
        return MapContext(
            observation=MapObservation.OBSERVED_OCCUPIED,
            point_count=count,
            max_height_m=None if math.isnan(height) else height,
        )


def _planar_distance(position: Vector3, origin: Vector3) -> float:
    """Distance in the xy plane, matching the proximity baseline's convention."""
    return math.hypot(position.x - origin.x, position.y - origin.y)


def _closing_speed(position: Vector3, velocity: Vector3 | None, origin: Vector3) -> float | None:
    """Rate of approach along the line to the ego reference, in m/s.

    Positive means closing, negative means receding. ``None`` when velocity was
    never measured - unknown motion is not a standstill (ADR-023), and reporting
    zero here would make an unmeasured object look calm.

    ``None`` also when the object is exactly at the reference point, where the
    direction of approach is undefined rather than zero.
    """
    if velocity is None:
        return None
    dx, dy = position.x - origin.x, position.y - origin.y
    distance = math.hypot(dx, dy)
    if distance == 0.0:
        return None
    closing = -((dx * velocity.x + dy * velocity.y) / distance)
    # Normalise the negative zero that a measured standstill produces, so a
    # stationary object never serialises as "-0.0".
    return closing + 0.0 if closing else 0.0


def _linear_falloff(value: float, near: float, far: float) -> float:
    """Map ``value`` to ``[0, 1]``: 1 at or below ``near``, 0 at or above ``far``.

    Continuous and monotonic between the two, so a small change in distance can
    never produce a jump in risk.
    """
    if value <= near:
        return 1.0
    if value >= far:
        return 0.0
    return (far - value) / (far - near)


def _observation_age_s(track: TrackedObject, now: datetime) -> float:
    """Measured seconds since the track was last observed.

    Zero when the track was seen at the assessment time, when it has never been
    seen, or when its timestamp is in the future - a negative age would describe
    an observation that has not happened.
    """
    if track.last_seen is None:
        return 0.0
    age = (now - track.last_seen).total_seconds()
    if not math.isfinite(age) or age <= 0.0:
        return 0.0
    return age


def _clamp(value: float) -> float:
    """Confine a score to ``[0, 1]``."""
    return max(0.0, min(1.0, value))


def _explain(
    *,
    level: RiskLevel,
    distance: float,
    closing_speed: float | None,
    relevance: TrajectoryRelevance | None,
    uncertainty: UncertaintyBreakdown,
    map_context: MapContext,
    object_class: ObjectClass,
) -> str:
    """Build the explanation from the values that were actually computed.

    Every clause is generated from a real number or a real flag. Nothing is
    asserted that the engine did not calculate.
    """
    parts = [f"{level.value.capitalize()} risk: {object_class.value} at {distance:.1f} m"]

    if closing_speed is None:
        parts.append("velocity was never measured, so approach rate is unknown")
    elif closing_speed > 0.0:
        parts.append(f"closing at {closing_speed:.1f} m/s")
    else:
        parts.append(f"receding at {abs(closing_speed):.1f} m/s")

    if relevance is not None:
        parts.append(
            f"predicted to pass within {relevance.min_distance_m:.1f} m at "
            f"t+{relevance.time_to_min_distance_s:.2f}s "
            f"(heuristic uncertainty {relevance.uncertainty_at_min_m:.1f} m)"
        )
    else:
        parts.append("no predicted trajectory available")

    if map_context.observation is MapObservation.OBSERVED_EMPTY:
        parts.append("no map returns at its cell, which is unobserved rather than clear")
    elif map_context.observation is MapObservation.OUT_OF_BOUNDS:
        parts.append("outside the mapped extent")

    if uncertainty.reasons:
        parts.append(
            "uncertainty "
            f"{uncertainty.score:.2f} from "
            + ", ".join(reason.value.replace("_", " ") for reason in uncertainty.reasons)
        )

    return "; ".join(parts) + "."


def build_risk_engine(settings: RiskSettings) -> HeuristicRiskEngine:
    """Construct the configured heuristic risk engine."""
    return HeuristicRiskEngine(settings)
