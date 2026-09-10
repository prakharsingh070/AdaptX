"""Deterministic heuristic resolution control (Phase 8).

Consumes Phase 7 risk assessments, the tracks that produced them and the
Phase 5 trajectories, and decides how much spatial detail each **region** of
the map deserves::

    assessments + tracks + trajectories
        -> spatial influence -> per-tile ResolutionContext
        -> detail priority   -> level (thresholds + hysteresis)
        -> dwell             -> budget
        -> ResolutionPlan

**This is a deterministic engineering heuristic.** ``detail_priority`` orders
regions by how much detail they deserve relative to each other. It is not a
probability, not a safety margin, not calibrated, and has never been validated
against labelled data - none exists (ADR-038).

What this module must never do
------------------------------
It does not score risk: it consumes ``RiskAssessment`` and never recomputes
one. It does not represent space: it produces a plan and hands it to a mapper,
which is the only component that touches point data (ADR-029, ADR-036).

Three rules it exists to keep
-----------------------------
1. **A missing factor is dropped, never scored zero.** The weighted mean runs
   over the factors actually available and the remaining weights renormalise -
   the ADR-032 rule, one phase later.
2. **Unknown risk is not low risk.** ``risk_score is None`` removes the risk
   factor and raises a floor on the level; it is never coerced to 0.0, which
   would hand the coarsest representation to the objects the system
   understands least (ADR-038).
3. **Uncertainty is a separate input, not a second risk score.** A quiet but
   badly observed region can earn detail on uncertainty alone, which is the
   whole reason Phase 7 kept the two apart (ADR-033).

State
-----
Unlike every other perception component here, the controller is **stateful**:
stabilisation is temporal by definition, so it remembers each region level and
how long a coarser one has been proposed (ADR-039). The state is small, keyed
by tile index, and owned by a service on the application context (ADR-025) -
never a module global.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from datetime import datetime

from adaptx.config.settings import AdaptiveResolutionSettings, MapSettings
from adaptx.core.logging import get_logger
from adaptx.mapping.interfaces import ResolutionController
from adaptx.mapping.tiles import TileGrid
from adaptx.models.adaptive_resolution import (
    DETAIL_POLICY_MODEL,
    AdaptiveResolutionConfiguration,
    DetailFactorName,
    DetailFactors,
    ExcludedAssessment,
    ExclusionReason,
    ResolutionBudget,
    ResolutionPlan,
    TileDecisionReason,
    TileResolutionDecision,
    coarser_by_one,
    finer_of,
    is_finer,
)
from adaptx.models.common import utc_now
from adaptx.models.map import ResolutionContext, ResolutionLevel
from adaptx.models.prediction import PredictedTrajectory
from adaptx.models.resolution import ResolutionDecision, ResolutionSource
from adaptx.models.risk_assessment import AssessmentStatus, RiskAssessment
from adaptx.models.spatial_map import MapBounds
from adaptx.models.tracking import TrackedObject

logger = get_logger(__name__)

#: Component recorded on every decision as its author.
CONTROLLER_NAME = "heuristic_resolution_controller_v1"


@dataclass(slots=True)
class _TileState:
    """What the controller remembers about one region between frames."""

    level: ResolutionLevel
    #: Consecutive frames on which a coarser level has been proposed.
    demote_frames: int = 0


@dataclass(slots=True)
class _Influence:
    """Evidence accumulated for one region during a single pass.

    Every field starts as ``None`` and stays that way unless something was
    genuinely measured, so "not computed" never collapses into "computed zero".
    """

    risk: float | None = None
    uncertainty: float | None = None
    trajectory: float | None = None
    speed_mps: float | None = None
    track_ids: list[int] = field(default_factory=list)
    unknown_risk_track_ids: list[int] = field(default_factory=list)

    def note_track(self, track_id: int, *, risk_known: bool) -> None:
        """Record that a track influences this region."""
        if track_id not in self.track_ids:
            self.track_ids.append(track_id)
        if not risk_known and track_id not in self.unknown_risk_track_ids:
            self.unknown_risk_track_ids.append(track_id)


@dataclass(slots=True)
class DetailPriority:
    """A region priority, and the factors that produced it.

    ``value`` is ``None`` when nothing influenced the region: no evidence is
    not the same as evidence of quiet, and the difference decides whether the
    base level was chosen or merely defaulted to.
    """

    value: float | None
    factors: list[DetailFactorName]
    scores: DetailFactors


class HeuristicResolutionController(ResolutionController):
    """Chooses a resolution level per region from risk, uncertainty and motion."""

    name = CONTROLLER_NAME
    #: True while the policy is a deterministic heuristic rather than a learned one.
    is_baseline = True

    def __init__(self, map_settings: MapSettings, settings: AdaptiveResolutionSettings) -> None:
        self._map_settings = map_settings
        self._settings = settings
        self._levels = map_settings.level_cell_sizes
        self._state: dict[int, _TileState] = {}
        self._frame_index = 0

    # -- configuration -----------------------------------------------------
    @property
    def bounds(self) -> MapBounds:
        """Configured map extent, shared with the fixed-resolution mapper."""
        return MapBounds(
            min_x=self._map_settings.min_x_m,
            max_x=self._map_settings.max_x_m,
            min_y=self._map_settings.min_y_m,
            max_y=self._map_settings.max_y_m,
        )

    def tile_grid(self) -> TileGrid:
        """The region partition this controller decides over.

        Raises:
            InvalidPointCloudError: the tiling would exceed ``max_tiles``.
        """
        return TileGrid.over(
            self.bounds,
            self._settings.tile_size_m,
            max_tiles=self._settings.max_tiles,
        )

    @property
    def configuration(self) -> AdaptiveResolutionConfiguration:
        """Snapshot of the settings that shape one planning pass."""
        settings = self._settings
        return AdaptiveResolutionConfiguration(
            tile_size_m=settings.tile_size_m,
            base_level=settings.base_level,
            unknown_risk_min_level=settings.unknown_risk_min_level,
            level_cell_sizes=dict(self._levels),
            threshold_medium=settings.threshold_medium,
            threshold_high=settings.threshold_high,
            threshold_critical=settings.threshold_critical,
            weight_risk=settings.weight_risk,
            weight_uncertainty=settings.weight_uncertainty,
            weight_proximity=settings.weight_proximity,
            weight_trajectory=settings.weight_trajectory,
            weight_density=settings.weight_density,
            weight_motion=settings.weight_motion,
            proximity_near_m=settings.proximity_near_m,
            proximity_far_m=settings.proximity_far_m,
            density_saturation_objects=settings.density_saturation_objects,
            motion_saturation_mps=settings.motion_saturation_mps,
            influence_radius_m=settings.influence_radius_m,
            uncertainty_radius_gain_m=settings.uncertainty_radius_gain_m,
            trajectory_corridor_m=settings.trajectory_corridor_m,
            hysteresis_margin=settings.hysteresis_margin,
            min_dwell_frames=settings.min_dwell_frames,
            max_tiles=settings.max_tiles,
            max_total_cells=settings.max_total_cells,
            max_fine_tiles=settings.max_fine_tiles,
            max_influencing_objects=settings.max_influencing_objects,
        )

    @property
    def frame_index(self) -> int:
        """Frames planned since the last reset."""
        return self._frame_index

    def levels(self) -> dict[ResolutionLevel, float]:
        """Cell edge length for each level, from map configuration."""
        return dict(self._levels)

    def current_level(self, tile_index: int) -> ResolutionLevel | None:
        """Level in force for one region, or ``None`` before its first decision."""
        state = self._state.get(tile_index)
        return None if state is None else state.level

    def reset(self) -> None:
        """Forget every region level and dwell counter.

        The one piece of accumulated state in the adaptive path. Call it
        between unrelated sequences, or a new scene inherits the stabilisation
        history of the old one.
        """
        self._state.clear()
        self._frame_index = 0
        logger.info("resolution controller state cleared")

    # -- the ResolutionController contract ---------------------------------
    def cell_size_m(self, level: ResolutionLevel) -> float:
        """Cell edge length in metres for ``level``, from map configuration."""
        return self._levels[level]

    def select_resolution(self, context: ResolutionContext) -> ResolutionLevel:
        """Return the level for the region described by ``context``.

        The stateless half of the policy: priority, thresholds, the unknown-risk
        floor and the hysteresis margin, all derived from the context alone.
        Minimum dwell time and the cell budget need history and a whole-map
        view, so :meth:`plan` applies those.
        """
        return self._select(context)[0]

    # -- scoring -----------------------------------------------------------
    def evaluate(self, context: ResolutionContext) -> DetailPriority:
        """Score how much detail a region deserves, and show the working.

        The weighted mean runs over the factors that could actually be
        computed. A factor that could not is absent from the result and its
        weight is not counted, so an unmeasurable quantity never drags a region
        towards coarse (ADR-032, ADR-038).
        """
        settings = self._settings
        if context.object_count <= 0:
            # Nothing influences this region. Proximity to the ego alone is not
            # evidence of complexity, so no score is invented for it.
            return DetailPriority(value=None, factors=[], scores=DetailFactors())

        weights: list[float] = []
        values: list[float] = []
        names: list[DetailFactorName] = []
        scores = DetailFactors()

        def contribute(name: DetailFactorName, value: float | None, weight: float) -> float | None:
            if value is None:
                return None
            clamped = _clamp(value)
            names.append(name)
            values.append(clamped)
            weights.append(weight)
            return clamped

        scores.risk = contribute(DetailFactorName.RISK, context.risk_score, settings.weight_risk)
        scores.uncertainty = contribute(
            DetailFactorName.UNCERTAINTY, context.uncertainty, settings.weight_uncertainty
        )
        scores.proximity = contribute(
            DetailFactorName.PROXIMITY,
            _inverse_falloff(
                context.distance_from_ego_m, settings.proximity_near_m, settings.proximity_far_m
            ),
            settings.weight_proximity,
        )
        scores.trajectory = contribute(
            DetailFactorName.TRAJECTORY, context.trajectory_relevance, settings.weight_trajectory
        )
        scores.density = contribute(
            DetailFactorName.DENSITY,
            min(1.0, context.object_count / settings.density_saturation_objects),
            settings.weight_density,
        )
        scores.motion = contribute(
            DetailFactorName.MOTION,
            (
                None
                if context.max_object_speed_mps is None
                else min(1.0, context.max_object_speed_mps / settings.motion_saturation_mps)
            ),
            settings.weight_motion,
        )

        total_weight = sum(weights)
        if total_weight <= 0.0:
            # Every available factor carries zero configured weight. Reporting a
            # score computed from nothing would be an invention.
            return DetailPriority(value=None, factors=names, scores=scores)

        weighted = sum(value * weight for value, weight in zip(values, weights, strict=True))
        return DetailPriority(value=_clamp(weighted / total_weight), factors=names, scores=scores)

    def band(self, priority: float) -> ResolutionLevel:
        """The level a priority falls in, ignoring history."""
        settings = self._settings
        if priority >= settings.threshold_critical:
            return ResolutionLevel.CRITICAL
        if priority >= settings.threshold_high:
            return ResolutionLevel.HIGH
        if priority >= settings.threshold_medium:
            return ResolutionLevel.MEDIUM
        return ResolutionLevel.LOW

    def _lower_bound(self, level: ResolutionLevel) -> float:
        """Smallest priority that still belongs to ``level``."""
        settings = self._settings
        return {
            ResolutionLevel.LOW: 0.0,
            ResolutionLevel.MEDIUM: settings.threshold_medium,
            ResolutionLevel.HIGH: settings.threshold_high,
            ResolutionLevel.CRITICAL: settings.threshold_critical,
        }[level]

    def _select(
        self, context: ResolutionContext
    ) -> tuple[ResolutionLevel, DetailPriority, list[TileDecisionReason]]:
        """Level, priority and reasons for one region, without dwell or budget."""
        settings = self._settings
        reasons: list[TileDecisionReason] = []
        priority = self.evaluate(context)

        if not settings.enabled:
            return settings.base_level, priority, [TileDecisionReason.ADAPTATION_DISABLED]

        if priority.value is None:
            target = settings.base_level
            reasons.append(TileDecisionReason.NO_OBJECT_INFLUENCE)
        else:
            target = self.band(priority.value)
            reasons.append(TileDecisionReason.SCORED)

        # Unknown risk is not low risk. A region influenced by an object that
        # could not be scored is floored, never left at the coarsest level.
        if context.has_unknown_risk:
            floored = finer_of(target, settings.unknown_risk_min_level)
            if floored is not target:
                reasons.append(TileDecisionReason.UNKNOWN_RISK_FLOOR)
            target = floored

        current = context.current_level
        if current is None or target is current:
            return target, priority, reasons

        if is_finer(target, current):
            # Refinement is immediate: detail is what the system needs when
            # something changes, and delaying it is the expensive mistake.
            return target, priority, reasons

        # Coarsening is resisted. The priority must fall clear of the band the
        # region currently holds, by the configured margin, before a coarser
        # level is even proposed (ADR-039).
        value = 0.0 if priority.value is None else priority.value
        if value >= self._lower_bound(current) - settings.hysteresis_margin:
            reasons.append(TileDecisionReason.HYSTERESIS_HOLD)
            return current, priority, reasons
        return target, priority, reasons

    # -- planning ----------------------------------------------------------
    def plan(
        self,
        assessments: list[RiskAssessment],
        *,
        tracks: list[TrackedObject] | None = None,
        trajectories: list[PredictedTrajectory] | None = None,
        timestamp: datetime | None = None,
        frame_id: int = 0,
        sensor_id: str = "unknown",
    ) -> ResolutionPlan:
        """Decide a resolution for every region of the map.

        Args:
            assessments: Phase 7 risk assessments for the current frame.
            tracks: The tracks those assessments describe. Positions come from
                here: a ``RiskAssessment`` carries a distance, not a location,
                and Phase 7 is deliberately left unchanged (ADR-036).
            trajectories: Phase 5 trajectories, used to raise the priority of
                regions a predicted path crosses. Absent trajectories lower no
                priority; they simply remove that factor.
            timestamp: Source time of the frame. Defaults to now.
            frame_id: Frame the plan belongs to.
            sensor_id: Sensor the frame came from.

        Returns:
            A plan covering every tile, accounting for every assessment.

        Raises:
            InvalidPointCloudError: the tiling would exceed ``max_tiles``.
        """
        started = time.perf_counter()
        grid = self.tile_grid()
        self._frame_index += 1

        influences, excluded, influencing = self._accumulate(
            grid, assessments, tracks, trajectories
        )
        decisions = self._decide(grid, influences)
        budget = self._apply_budget(grid, decisions)
        self._commit(decisions)

        duration_ms = (time.perf_counter() - started) * 1000.0
        plan = ResolutionPlan(
            timestamp=timestamp if timestamp is not None else utc_now(),
            frame_id=frame_id,
            sensor_id=sensor_id,
            controller=self.name,
            is_baseline=self.is_baseline,
            policy_model=DETAIL_POLICY_MODEL,
            decisions=[entry.to_model() for entry in decisions],
            excluded=excluded,
            considered_assessment_count=len(assessments),
            influencing_assessment_count=influencing,
            frame_index=self._frame_index,
            budget=budget,
            duration_ms=duration_ms,
            configuration=self.configuration,
        )
        logger.debug(
            "resolution plan built",
            extra={
                "context": {
                    "frame_id": frame_id,
                    "tiles": plan.tile_count,
                    "changed": plan.changed_tile_count,
                    "cells": budget.total_cell_count,
                    "duration_ms": round(duration_ms, 3),
                }
            },
        )
        return plan

    def _accumulate(
        self,
        grid: TileGrid,
        assessments: list[RiskAssessment],
        tracks: list[TrackedObject] | None,
        trajectories: list[PredictedTrajectory] | None,
    ) -> tuple[dict[int, _Influence], list[ExcludedAssessment], int]:
        """Project every assessment onto the regions it influences.

        Assessments are processed in ascending ``track_id`` so the pass does
        not depend on the order they arrived in. Every one of them ends up
        either influencing a region or recorded in ``excluded``.
        """
        settings = self._settings
        by_track = {track.track_id: track for track in (tracks or [])}
        paths = {trajectory.track_id: trajectory for trajectory in (trajectories or [])}

        influences: dict[int, _Influence] = {}
        excluded: list[ExcludedAssessment] = []
        influencing = 0

        ordered = sorted(assessments, key=lambda assessment: assessment.track_id)
        for position, assessment in enumerate(ordered):
            if position >= settings.max_influencing_objects:
                excluded.append(
                    ExcludedAssessment(
                        track_id=assessment.track_id,
                        reason=ExclusionReason.LIMIT_EXCEEDED,
                        distance_m=assessment.distance_m,
                    )
                )
                continue

            if assessment.status is AssessmentStatus.TRACK_LOST:
                # A terminated track is history. Spending detail on where it
                # used to be would represent the past as the present.
                excluded.append(
                    ExcludedAssessment(
                        track_id=assessment.track_id,
                        reason=ExclusionReason.TRACK_LOST,
                        distance_m=assessment.distance_m,
                    )
                )
                continue

            track = by_track.get(assessment.track_id)
            if track is None:
                excluded.append(
                    ExcludedAssessment(
                        track_id=assessment.track_id,
                        reason=ExclusionReason.MISSING_POSITION,
                        distance_m=assessment.distance_m,
                    )
                )
                continue

            weights = self._influence_weights(grid, assessment, track, paths.get(track.track_id))
            if not weights:
                excluded.append(
                    ExcludedAssessment(
                        track_id=assessment.track_id,
                        reason=ExclusionReason.OUT_OF_BOUNDS,
                        distance_m=assessment.distance_m,
                    )
                )
                continue

            influencing += 1
            risk_known = assessment.risk_score is not None
            for tile_index in sorted(weights):
                weight, trajectory_weight = weights[tile_index]
                influence = influences.setdefault(tile_index, _Influence())
                influence.note_track(assessment.track_id, risk_known=risk_known)

                if assessment.risk_score is not None:
                    influence.risk = _maximum(influence.risk, assessment.risk_score * weight)
                influence.uncertainty = _maximum(
                    influence.uncertainty, assessment.uncertainty.score * weight
                )
                if trajectory_weight is not None:
                    influence.trajectory = _maximum(influence.trajectory, trajectory_weight)
                if assessment.speed_mps is not None:
                    influence.speed_mps = _maximum(influence.speed_mps, assessment.speed_mps)

        return influences, excluded, influencing

    def _influence_weights(
        self,
        grid: TileGrid,
        assessment: RiskAssessment,
        track: TrackedObject,
        trajectory: PredictedTrajectory | None,
    ) -> dict[int, tuple[float, float | None]]:
        """How strongly one object reaches each region, and via what.

        Two sources, combined by taking the stronger:

        * **Where it is.** Influence falls linearly from 1.0 at the region
          holding the object to 0.0 at the influence radius, widened by the
          object own uncertainty - a vaguer object is relevant over a wider
          area.
        * **Where it is predicted to go.** Every trajectory point marks a
          corridor, weighted towards the near future. This is predicted-motion
          relevance and nothing more: it does not claim the object will arrive.

        Returns:
            ``{tile_index: (weight, trajectory_weight)}``, trajectory weight
            ``None`` where no predicted path reached the region.
        """
        settings = self._settings
        weights: dict[int, tuple[float, float | None]] = {}

        radius = settings.influence_radius_m + (
            settings.uncertainty_radius_gain_m * assessment.uncertainty.score
        )
        x, y = track.position.x, track.position.y
        for tile_index in grid.indices_within(x, y, radius):
            distance = grid.distance_to(tile_index, x, y)
            weights[tile_index] = (_clamp(1.0 - distance / radius) if radius > 0 else 1.0, None)

        if trajectory is not None:
            horizon = trajectory.horizon_s
            for point in trajectory.points:
                # Nearer in time weighs more: a region a predicted path reaches
                # in half a second is a stronger argument for detail now than
                # one it reaches at the end of the horizon.
                time_weight = _clamp(1.0 - (point.time_offset_s / horizon)) if horizon > 0 else 1.0
                if time_weight <= 0.0:
                    continue
                corridor = settings.trajectory_corridor_m + point.position_uncertainty_m
                for tile_index in grid.indices_within(point.position.x, point.position.y, corridor):
                    existing, existing_trajectory = weights.get(tile_index, (0.0, None))
                    weights[tile_index] = (
                        max(existing, time_weight),
                        _maximum(existing_trajectory, time_weight),
                    )

        return weights

    def _decide(self, grid: TileGrid, influences: dict[int, _Influence]) -> list[_Decision]:
        """Choose a level for every region, applying thresholds and dwell."""
        decisions: list[_Decision] = []

        for tile_index in range(grid.tile_count):
            influence = influences.get(tile_index, _Influence())
            context = self._context(grid, tile_index, influence)
            target, priority, reasons = self._select(context)
            previous = context.current_level
            level, reasons = self._apply_dwell(tile_index, previous, target, reasons)

            decisions.append(
                _Decision(
                    tile_index=tile_index,
                    grid=grid,
                    level=level,
                    previous_level=previous,
                    priority=priority,
                    reasons=reasons,
                    influence=influence,
                    cell_size_m=self.cell_size_m(level),
                )
            )
        return decisions

    def _context(self, grid: TileGrid, tile_index: int, influence: _Influence) -> ResolutionContext:
        """Assemble the decision inputs for one region."""
        centre_x, centre_y = grid.tile_centre(tile_index)
        bounds = grid.tile_bounds(tile_index)
        area = bounds.size_x_m * bounds.size_y_m
        count = len(influence.track_ids)

        return ResolutionContext(
            x=centre_x,
            y=centre_y,
            distance_from_ego_m=math.hypot(centre_x, centre_y),
            risk_score=influence.risk,
            # No predicted-risk formulation exists: Phase 7 folds predicted
            # approach into its score rather than publishing a second one, so
            # inventing a value here would be fabricating a measurement.
            predicted_risk_score=None,
            uncertainty=influence.uncertainty,
            object_density=(count / area if count and area > 0 else None),
            max_object_speed_mps=influence.speed_mps,
            trajectory_relevance=influence.trajectory,
            object_count=count,
            has_unknown_risk=bool(influence.unknown_risk_track_ids),
            # No ego planned path exists in this project, so this is never set
            # true rather than being guessed from heading.
            in_ego_path=False,
            current_level=self.current_level(tile_index),
        )

    def _apply_dwell(
        self,
        tile_index: int,
        previous: ResolutionLevel | None,
        target: ResolutionLevel,
        reasons: list[TileDecisionReason],
    ) -> tuple[ResolutionLevel, list[TileDecisionReason]]:
        """Hold a coarser level back until it has been proposed long enough.

        Refinement applies on the frame it is asked for; coarsening must be
        proposed on ``min_dwell_frames`` consecutive frames. Together with the
        hysteresis margin this is what stops a region on a threshold boundary
        flipping every frame (ADR-039).
        """
        state = self._state.get(tile_index)
        if state is None or previous is None:
            return target, reasons

        if is_finer(target, state.level):
            state.demote_frames = 0
            return target, reasons

        if target is state.level:
            state.demote_frames = 0
            return target, reasons

        state.demote_frames += 1
        if state.demote_frames >= self._settings.min_dwell_frames:
            state.demote_frames = 0
            return target, reasons
        return state.level, [*reasons, TileDecisionReason.DWELL_HOLD]

    def _apply_budget(self, grid: TileGrid, decisions: list[_Decision]) -> ResolutionBudget:
        """Coarsen the least important regions until the plan fits its ceilings.

        Deterministic by construction: candidates are ordered by priority
        ascending with ``tile_index`` breaking ties, and a region with no
        priority at all sorts first, because a region nothing influences is the
        cheapest place to give up detail.
        """
        settings = self._settings
        demoted: set[int] = set()

        def order(decision: _Decision) -> tuple[float, int]:
            # -1.0 sorts an unscored region ahead of every scored one.
            value = -1.0 if decision.priority.value is None else decision.priority.value
            return value, decision.tile_index

        candidates = sorted(decisions, key=order)

        fine = [decision for decision in candidates if decision.level in _FINE_LEVELS]
        excess = len(fine) - settings.max_fine_tiles
        for decision in fine[: max(0, excess)]:
            decision.demote_to(ResolutionLevel.MEDIUM, self.cell_size_m(ResolutionLevel.MEDIUM))
            decision.reasons.append(TileDecisionReason.FINE_TILE_LIMIT)
            demoted.add(decision.tile_index)

        total = sum(decision.cell_count() for decision in decisions)
        if total > settings.max_total_cells:
            for decision in candidates:
                if total <= settings.max_total_cells:
                    break
                coarser = coarser_by_one(decision.level)
                while coarser is not None and total > settings.max_total_cells:
                    before = decision.cell_count()
                    decision.demote_to(coarser, self.cell_size_m(coarser))
                    total -= before - decision.cell_count()
                    if decision.tile_index not in demoted:
                        decision.reasons.append(TileDecisionReason.BUDGET_DEMOTED)
                        demoted.add(decision.tile_index)
                    coarser = coarser_by_one(decision.level)

        total = sum(decision.cell_count() for decision in decisions)
        fine_count = sum(1 for decision in decisions if decision.level in _FINE_LEVELS)
        return ResolutionBudget(
            tile_count=len(decisions),
            max_tiles=settings.max_tiles,
            total_cell_count=total,
            max_total_cells=settings.max_total_cells,
            fine_tile_count=fine_count,
            max_fine_tiles=settings.max_fine_tiles,
            demoted_tile_count=len(demoted),
            within_budget=(
                total <= settings.max_total_cells
                and fine_count <= settings.max_fine_tiles
                and len(decisions) <= settings.max_tiles
            ),
        )

    def _commit(self, decisions: list[_Decision]) -> None:
        """Record the applied levels so the next frame can stabilise against them."""
        for decision in decisions:
            state = self._state.get(decision.tile_index)
            if state is None:
                self._state[decision.tile_index] = _TileState(level=decision.level)
            else:
                state.level = decision.level


#: Levels that count against the fine-tile ceiling.
_FINE_LEVELS = (ResolutionLevel.HIGH, ResolutionLevel.CRITICAL)


@dataclass(slots=True)
class _Decision:
    """A tile decision under construction, before the budget has settled it."""

    tile_index: int
    grid: TileGrid
    level: ResolutionLevel
    previous_level: ResolutionLevel | None
    priority: DetailPriority
    reasons: list[TileDecisionReason]
    influence: _Influence
    cell_size_m: float

    def demote_to(self, level: ResolutionLevel, cell_size_m: float) -> None:
        """Coarsen this decision to ``level``."""
        self.level = level
        self.cell_size_m = cell_size_m

    def cell_count(self) -> int:
        """Cells this decision would allocate."""
        height, width = self.grid.cell_shape(self.tile_index, self.cell_size_m)
        return height * width

    def to_model(self) -> TileResolutionDecision:
        """Freeze into the published contract."""
        row, column = self.grid.row_column(self.tile_index)
        height, width = self.grid.cell_shape(self.tile_index, self.cell_size_m)
        explanation = _explain(self.level, self.priority, self.reasons, self.influence)
        return TileResolutionDecision(
            tile_index=self.tile_index,
            tile_row=row,
            tile_column=column,
            bounds=self.grid.tile_bounds(self.tile_index),
            level=self.level,
            resolution=ResolutionDecision(
                resolution_m=self.cell_size_m,
                source=ResolutionSource.ADAPTIVE,
                reason=explanation,
                requested_by=CONTROLLER_NAME,
            ),
            previous_level=self.previous_level,
            changed=self.previous_level is not None and self.previous_level is not self.level,
            detail_priority=self.priority.value,
            factors=list(self.priority.factors),
            factor_scores=self.priority.scores,
            reasons=list(self.reasons),
            reason=explanation,
            influencing_track_ids=sorted(self.influence.track_ids),
            unknown_risk_track_ids=sorted(self.influence.unknown_risk_track_ids),
            cell_width=width,
            cell_height=height,
        )


def _explain(
    level: ResolutionLevel,
    priority: DetailPriority,
    reasons: list[TileDecisionReason],
    influence: _Influence,
) -> str:
    """Generate a decision explanation from the computed values.

    Assembled from numbers and flags, never free-form: the text can only say
    what was actually calculated. It deliberately never contains the words a
    heuristic has no right to - probability, calibrated, validated, safe or
    guaranteed.
    """
    parts: list[str] = []
    if priority.value is None:
        parts.append("no object influence, base level")
    else:
        parts.append(f"detail priority {priority.value:.2f} (heuristic)")

    scores = priority.scores
    detail: list[str] = []
    if scores.risk is not None:
        detail.append(f"risk {scores.risk:.2f}")
    if scores.uncertainty is not None:
        detail.append(f"uncertainty {scores.uncertainty:.2f}")
    if scores.proximity is not None:
        detail.append(f"proximity {scores.proximity:.2f}")
    if scores.trajectory is not None:
        detail.append(f"predicted-motion relevance {scores.trajectory:.2f}")
    if scores.density is not None:
        detail.append(f"density {scores.density:.2f}")
    if scores.motion is not None:
        detail.append(f"motion {scores.motion:.2f}")
    if detail:
        parts.append(", ".join(detail))

    if influence.unknown_risk_track_ids:
        floored = ", level floored" if TileDecisionReason.UNKNOWN_RISK_FLOOR in reasons else ""
        parts.append(
            f"{len(influence.unknown_risk_track_ids)} influencing track(s) unscored; "
            f"risk factor dropped{floored}"
        )
    if TileDecisionReason.HYSTERESIS_HOLD in reasons:
        parts.append("held: priority has not fallen clear of the current band")
    if TileDecisionReason.DWELL_HOLD in reasons:
        parts.append("held: a coarser level has not been proposed for long enough")
    if TileDecisionReason.BUDGET_DEMOTED in reasons:
        parts.append("coarsened to fit the cell budget")
    if TileDecisionReason.FINE_TILE_LIMIT in reasons:
        parts.append("coarsened to fit the fine-region limit")
    if TileDecisionReason.ADAPTATION_DISABLED in reasons:
        parts.append("adaptation disabled by configuration")

    return f"{level.value}: " + "; ".join(parts)


def _maximum(current: float | None, candidate: float) -> float:
    """Largest of a value that may not exist yet and a new one."""
    return candidate if current is None else max(current, candidate)


def _inverse_falloff(value: float, near: float, far: float) -> float:
    """1.0 at or inside ``near``, 0.0 at or beyond ``far``, linear between."""
    if value <= near:
        return 1.0
    if value >= far:
        return 0.0
    return (far - value) / (far - near)


def _clamp(value: float) -> float:
    """Constrain to ``[0, 1]``."""
    return max(0.0, min(1.0, value))


def build_controller(
    map_settings: MapSettings, settings: AdaptiveResolutionSettings
) -> HeuristicResolutionController:
    """Construct the configured resolution controller."""
    return HeuristicResolutionController(map_settings, settings)
