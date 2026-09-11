"""Trajectory prediction against where the actor actually went (Question B).

For a trajectory emitted on frame *i* for a track matched to actor *a*, each
point at offset ``τ`` is compared with *a*'s ground-truth position on frame
``i + τ/Δt`` - and only when ``τ/Δt`` is an integer and that frame exists in
the run. **No interpolation**: a ground-truth position between frames would
be an invention, and with the catalogue's 0.05 s step and 0.25 s interval
every offset lands exactly on a frame anyway.

ADE is the mean planar displacement over the aligned **future** points of
one trajectory (the ``t+0`` point restates the track's current position and
is tracking error, not prediction error, so it is excluded); FDE is the
displacement at the last aligned point. Both are then summarised over
trajectories. A trajectory with no aligned point at all is skipped with the
reason, never scored.
"""

from __future__ import annotations

from collections import Counter

from adaptx.evaluation.dataset import EvaluationDataset
from adaptx.evaluation.matching import FrameMatching, planar_distance
from adaptx.evaluation.models import (
    Distribution,
    EvaluationConfiguration,
    MetricStatus,
    PredictionEvaluation,
)

#: Tolerance when deciding whether an offset is an integer number of frames.
_FRAME_TOLERANCE = 1e-6


def frames_ahead(offset_s: float, timestep_s: float) -> int | None:
    """How many frames ahead ``offset_s`` is, or ``None`` if not a whole number."""
    ratio = offset_s / timestep_s
    nearest = round(ratio)
    if abs(ratio - nearest) > _FRAME_TOLERANCE:
        return None
    return int(nearest)


def evaluate_prediction(
    dataset: EvaluationDataset,
    configuration: EvaluationConfiguration,
    matching: dict[int, FrameMatching],
) -> PredictionEvaluation:
    """ADE / FDE over every trajectory whose track was matched on its source frame."""
    in_record = 0
    predictor_skips: Counter[str] = Counter()
    skipped: Counter[str] = Counter()
    ade: list[float] = []
    fde: list[float] = []
    coverage: list[float] = []
    by_offset: dict[float, list[float]] = {}

    last_index = dataset.frames[-1].index if dataset.frames else -1
    for frame in dataset.frames:
        prediction = frame.outputs.prediction
        in_record += len(prediction.trajectories)
        for skip in prediction.skipped:
            predictor_skips[skip.status.value] += 1
        frame_matching = matching[frame.index]

        for trajectory in prediction.trajectories:
            match = frame_matching.match_for_candidate(trajectory.track_id)
            if match is None:
                skipped["track_unmatched_on_source_frame"] += 1
                continue
            actor_id = match.simulator_actor_id
            errors: list[tuple[float, float]] = []
            beyond_run = 0
            misaligned = 0
            actor_absent = 0
            future_points = [p for p in trajectory.points if p.time_offset_s > 0.0]
            for point in future_points:
                ahead = frames_ahead(point.time_offset_s, dataset.timestep_s)
                if ahead is None:
                    misaligned += 1
                    continue
                target_index = frame.index + ahead
                if target_index > last_index:
                    beyond_run += 1
                    continue
                target = dataset.frame_at(target_index)
                if target is None:
                    actor_absent += 1
                    continue
                future = next((a for a in target.actors if a.simulator_actor_id == actor_id), None)
                if future is None:
                    actor_absent += 1
                    continue
                errors.append(
                    (point.time_offset_s, planar_distance(point.position, future.position))
                )
            if not errors:
                if not future_points:
                    skipped["no_future_point"] += 1
                elif misaligned and misaligned == len(future_points):
                    skipped["offsets_not_on_frame_boundary"] += 1
                elif actor_absent:
                    skipped["actor_absent_on_target_frames"] += 1
                else:
                    skipped["no_ground_truth_within_run"] += 1
                continue
            ade.append(sum(error for _, error in errors) / len(errors))
            fde.append(errors[-1][1])
            coverage.append(len(errors) / len(future_points))
            for offset, error in errors:
                by_offset.setdefault(offset, []).append(error)

    if in_record == 0:
        return PredictionEvaluation(
            status=MetricStatus.NOT_APPLICABLE,
            reason="the record contains no predicted trajectory",
            gate_m=configuration.primary_gate_m,
            trajectories_in_record=0,
            trajectories_evaluated=0,
            predictor_skips=dict(predictor_skips),
            ade_m=Distribution.of([]),
            fde_m=Distribution.of([]),
            horizon_coverage=Distribution.of([]),
            ego_stationary=dataset.ego_stationary,
        )

    status = MetricStatus.MEASURED
    reason: str | None = None
    warnings: list[str] = []
    if not ade:
        status = MetricStatus.UNAVAILABLE
        reason = "no trajectory had a matched track and an aligned future ground-truth frame"
    elif skipped:
        status = MetricStatus.PARTIAL
        reason = f"{sum(skipped.values())} of {in_record} trajectories could not be evaluated"
    if dataset.ego_stationary is False:
        status = MetricStatus.PARTIAL
        warnings.append(
            "the ego moved during the run, so ego-relative ground truth on later frames "
            "is not strictly comparable with a prediction made earlier"
        )
    return PredictionEvaluation(
        status=status,
        reason=reason,
        warnings=warnings,
        gate_m=configuration.primary_gate_m,
        trajectories_in_record=in_record,
        trajectories_evaluated=len(ade),
        skipped=dict(skipped),
        predictor_skips=dict(predictor_skips),
        ade_m=Distribution.of(ade),
        fde_m=Distribution.of(fde),
        horizon_coverage=Distribution.of(coverage),
        error_by_offset_m={
            f"{offset:.2f}": Distribution.of(errors) for offset, errors in sorted(by_offset.items())
        },
        ego_stationary=dataset.ego_stationary,
    )


__all__ = ["evaluate_prediction", "frames_ahead"]
