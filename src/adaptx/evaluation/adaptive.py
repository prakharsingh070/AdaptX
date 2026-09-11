"""Adaptive resolution against its own stated policy (Question E).

Fixed and adaptive maps in the record were built from the **same processed
frame** of the **same run**; only the resolution policy differs, so every
ratio here is paired by construction (ADR-053).

The policy's intent, stated as measurements:

* *detail at the actor* - the level of the tile holding each ground-truth
  actor, against the level distribution of every other tile;
* *detail by risk* - that tile's cell size grouped by the matched track's
  risk level, ``UNKNOWN`` kept apart;
* *refinement lead* - for each (actor, tile) pair, counted once, the frame
  the actor entered the tile minus the frame the tile first reached the
  refined level. Positive means the region was refined before the object
  arrived, which is what the project set out to do; negative means after;
  null means never during the run.

And its cost: cells, bytes and construction time against the fixed map,
churn, reversals, and the holds the stabiliser reports.

Nothing here assumes the adaptive map is better. A ratio above one, a lead
time that is positive, a reversal count that is high - each is reported as
measured.
"""

from __future__ import annotations

from collections import Counter

from adaptx.evaluation.dataset import EvaluationDataset
from adaptx.evaluation.matching import FrameMatching
from adaptx.evaluation.models import (
    AdaptiveResolutionEvaluation,
    ChurnMetrics,
    Distribution,
    EvaluationConfiguration,
    MetricStatus,
    TileRefinement,
)
from adaptx.models.adaptive_resolution import (
    LEVEL_RANK,
    ResolutionPlan,
    TileDecisionReason,
    TileResolutionDecision,
)
from adaptx.models.risk import RiskLevel


def tile_for(plan: ResolutionPlan, x: float, y: float) -> TileResolutionDecision | None:
    """The plan's tile containing planar position ``(x, y)``, or ``None`` if outside.

    A tile owns its lower edges and the map's outermost tiles also own the
    upper edge, so a position on an interior boundary belongs to exactly one
    tile and a position on the map edge is not lost.
    """
    candidates = [
        d
        for d in plan.decisions
        if d.bounds.min_x <= x <= d.bounds.max_x and d.bounds.min_y <= y <= d.bounds.max_y
    ]
    if not candidates:
        return None
    # Interior boundaries: prefer the tile for which the point is not on the
    # upper edge, i.e. the one that owns it.
    owners = [
        d
        for d in candidates
        if (x < d.bounds.max_x or _is_map_edge(plan, d, "x"))
        and (y < d.bounds.max_y or _is_map_edge(plan, d, "y"))
    ]
    chosen = owners or candidates
    return min(chosen, key=lambda d: d.tile_index)


def _is_map_edge(plan: ResolutionPlan, tile: TileResolutionDecision, axis: str) -> bool:
    upper = max(getattr(d.bounds, f"max_{axis}") for d in plan.decisions)
    return bool(getattr(tile.bounds, f"max_{axis}") == upper)


def evaluate_adaptive(
    dataset: EvaluationDataset,
    configuration: EvaluationConfiguration,
    matching: dict[int, FrameMatching],
) -> AdaptiveResolutionEvaluation:
    """Every Question E measurement, from the plans and summaries in the record."""
    if not dataset.frames:
        return _unavailable("no evaluated frame")
    first_plan = dataset.frames[0].outputs.plan
    if not first_plan.decisions:
        return _unavailable("the resolution plan covers no tile")

    refined_rank = LEVEL_RANK[configuration.refined_level]
    tile_resolutions: list[float] = []
    unique_per_frame: list[float] = []
    area_weighted: list[float] = []
    fixed_cells: list[float] = []
    adaptive_cells: list[float] = []
    cell_ratio: list[float] = []
    byte_ratio: list[float] = []
    mapping_ratio: list[float] = []
    total_ratio: list[float] = []
    actor_levels: Counter[str] = Counter()
    other_levels: Counter[str] = Counter()
    actor_resolution: list[float] = []
    other_resolution: list[float] = []
    by_risk: dict[str, list[float]] = {}
    changed_per_frame: list[float] = []
    frames_with_change = 0
    transitions_per_tile: Counter[int] = Counter()
    hysteresis_holds = 0
    dwell_holds = 0
    frames_over_budget = 0
    demoted_total = 0
    reversals = 0
    warnings: list[str] = []

    # Per-tile level history for reversals.
    level_history: dict[int, list[tuple[int, str]]] = {}
    # First frame each tile reached the refined level.
    first_refined: dict[int, int] = {}
    # The first frame each actor was seen in each tile. An (actor, tile) pair
    # counts once: an actor sitting on a tile boundary flickers between two
    # tiles with the simulator's sub-centimetre jitter, and counting every
    # flicker as an arrival would multiply one entry into dozens.
    entries: dict[tuple[str, int], int] = {}

    dwell_frames = first_plan.configuration.min_dwell_frames
    reversal_window = max(dwell_frames, 1)
    actors_outside = 0

    for frame in dataset.frames:
        outputs = frame.outputs
        plan = outputs.plan
        comparison = outputs.comparison

        per_frame_res = [d.resolution_m for d in plan.decisions]
        tile_resolutions.extend(per_frame_res)
        unique_per_frame.append(float(len(set(per_frame_res))))
        area_weighted.append(outputs.adaptive_map.area_weighted_resolution_m)
        fixed_cells.append(float(comparison.fixed.total_cell_count))
        adaptive_cells.append(float(comparison.adaptive.total_cell_count))
        cell_ratio.append(comparison.cell_ratio)
        if comparison.fixed.grid_bytes > 0:
            byte_ratio.append(comparison.byte_ratio)
        if comparison.fixed.duration_ms > 0.0:
            mapping_ratio.append(comparison.adaptive.duration_ms / comparison.fixed.duration_ms)
            total_ratio.append(
                (plan.duration_ms + comparison.adaptive.duration_ms) / comparison.fixed.duration_ms
            )

        # Churn, holds and budget from the plan itself.
        changed = plan.changed_tile_count
        changed_per_frame.append(float(changed))
        frames_with_change += int(changed > 0)
        if not plan.budget.within_budget:
            frames_over_budget += 1
        demoted_total += plan.budget.demoted_tile_count
        for decision in plan.decisions:
            if TileDecisionReason.HYSTERESIS_HOLD in decision.reasons:
                hysteresis_holds += 1
            if TileDecisionReason.DWELL_HOLD in decision.reasons:
                dwell_holds += 1
            # ``history`` holds one (start frame, level) entry per run of
            # frames at one level. A reversal is A -> B -> A where the stay at
            # B lasted no longer than the window: the tile went back to the
            # level it had just left.
            history = level_history.setdefault(decision.tile_index, [])
            if decision.changed:
                transitions_per_tile[decision.tile_index] += 1
                if (
                    len(history) >= 2
                    and history[-2][1] == decision.level.value
                    and frame.index - history[-1][0] <= reversal_window
                ):
                    reversals += 1
            if not history or history[-1][1] != decision.level.value:
                history.append((frame.index, decision.level.value))
            if (
                LEVEL_RANK[decision.level] >= refined_rank
                and decision.tile_index not in first_refined
            ):
                first_refined[decision.tile_index] = frame.index

        # Detail at the actor versus elsewhere, and entries for lead time.
        frame_matching = matching[frame.index]
        assessments = {a.track_id: a for a in outputs.risk.assessments}
        actor_tiles: set[int] = set()
        for actor in frame.eligible_actors:
            tile = tile_for(plan, actor.position.x, actor.position.y)
            if tile is None:
                actors_outside += 1
                continue
            actor_tiles.add(tile.tile_index)
            actor_levels[tile.level.value] += 1
            actor_resolution.append(tile.resolution_m)
            match = frame_matching.match_for_actor(actor.simulator_actor_id)
            assessment = None if match is None else assessments.get(match.candidate_id)
            group = "unmatched" if assessment is None else assessment.risk_level.value
            if assessment is not None and assessment.risk_level is RiskLevel.UNKNOWN:
                group = RiskLevel.UNKNOWN.value
            by_risk.setdefault(group, []).append(tile.resolution_m)
            entries.setdefault((actor.actor_id, tile.tile_index), frame.index)
        for decision in plan.decisions:
            if decision.tile_index not in actor_tiles:
                other_levels[decision.level.value] += 1
                other_resolution.append(decision.resolution_m)

    if actors_outside:
        warnings.append(
            f"{actors_outside} actor-frames fell outside every tile and were excluded from "
            "the detail-at-actor figures"
        )

    refinements: list[TileRefinement] = []
    leads: list[float] = []
    never = 0
    for (actor_name, tile_index), entered in sorted(entries.items(), key=lambda e: e[1]):
        refined = first_refined.get(tile_index)
        lead = None if refined is None else entered - refined
        if lead is None:
            never += 1
        else:
            leads.append(float(lead))
        refinements.append(
            TileRefinement(
                actor_id=actor_name,
                tile_index=tile_index,
                entered_frame=entered,
                refined_frame=refined,
                lead_frames=lead,
            )
        )

    return AdaptiveResolutionEvaluation(
        status=MetricStatus.MEASURED,
        warnings=warnings,
        tile_count=first_plan.tile_count,
        tile_size_m=first_plan.configuration.tile_size_m,
        level_cell_sizes_m={
            level.value: size for level, size in first_plan.configuration.level_cell_sizes.items()
        },
        tile_resolution_m=Distribution.of(tile_resolutions),
        unique_resolutions_per_frame=Distribution.of(unique_per_frame),
        area_weighted_resolution_m=Distribution.of(area_weighted),
        fixed_cells=Distribution.of(fixed_cells),
        adaptive_cells=Distribution.of(adaptive_cells),
        cell_ratio=Distribution.of(cell_ratio),
        byte_ratio=Distribution.of(byte_ratio),
        mapping_duration_ratio=Distribution.of(mapping_ratio),
        total_duration_ratio=Distribution.of(total_ratio),
        actor_tile_levels=dict(actor_levels),
        other_tile_levels=dict(other_levels),
        actor_tile_resolution_m=Distribution.of(actor_resolution),
        other_tile_resolution_m=Distribution.of(other_resolution),
        actor_tile_resolution_by_risk_m={
            group: Distribution.of(values) for group, values in sorted(by_risk.items())
        },
        refined_level=configuration.refined_level.value,
        refinements=refinements,
        refinement_lead_frames=Distribution.of(leads),
        entries_never_refined=never,
        churn=ChurnMetrics(
            frames=len(dataset.frames),
            changed_tiles_per_frame=Distribution.of(changed_per_frame),
            frames_with_any_change=frames_with_change,
            total_transitions=sum(transitions_per_tile.values()),
            tiles_ever_changed=len(transitions_per_tile),
            reversals=reversals,
            hysteresis_holds=hysteresis_holds,
            dwell_holds=dwell_holds,
            min_dwell_frames=dwell_frames,
        ),
        frames_over_budget=frames_over_budget,
        demoted_tiles_total=demoted_total,
    )


def _unavailable(reason: str) -> AdaptiveResolutionEvaluation:
    empty = Distribution.of([])
    return AdaptiveResolutionEvaluation(
        status=MetricStatus.UNAVAILABLE,
        reason=reason,
        tile_resolution_m=empty,
        unique_resolutions_per_frame=empty,
        area_weighted_resolution_m=empty,
        fixed_cells=empty,
        adaptive_cells=empty,
        cell_ratio=empty,
        byte_ratio=empty,
        mapping_duration_ratio=empty,
        total_duration_ratio=empty,
        actor_tile_resolution_m=empty,
        other_tile_resolution_m=empty,
        refinement_lead_frames=empty,
        entries_never_refined=0,
        frames_over_budget=0,
        demoted_tiles_total=0,
    )


__all__ = ["evaluate_adaptive", "tile_for"]
