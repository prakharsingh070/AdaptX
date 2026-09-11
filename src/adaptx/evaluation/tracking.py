"""Detection and tracking against ground truth (Question A).

Both stages are matched with the same rule at the same gates, so the
difference between the two sections is the tracker's own contribution: a
detection that is there but a track that is not, or an identity that swaps.

What is deliberately not here
-----------------------------
No precision, no MOTA. A track without a ground-truth match is unlabelled -
the map's buildings, poles and parked meshes return LiDAR points and are not
CARLA actors - so a false-positive count would be a guess dressed as a
measurement (``docs/EVALUATION.md`` §3.2).
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from itertools import pairwise

from adaptx.evaluation.dataset import EvaluationDataset, EvaluationFrame, ReferenceActor
from adaptx.evaluation.matching import Candidate, FrameMatching, match_frame
from adaptx.evaluation.models import (
    ActorContinuity,
    DetectionEvaluation,
    Distribution,
    EvaluationConfiguration,
    GateMetrics,
    MetricStatus,
    TrackingEvaluation,
    TrackingGateMetrics,
)
from adaptx.models.tracking import TrackedObject


def detection_candidates(frame: EvaluationFrame) -> list[Candidate]:
    """The frame's detections as matcher input."""
    return [
        Candidate(candidate_id=obj.object_id, position=obj.position, object_class=obj.object_class)
        for obj in frame.outputs.detection.objects
    ]


def track_candidates(frame: EvaluationFrame) -> list[Candidate]:
    """The frame's live tracks as matcher input."""
    return [
        Candidate(candidate_id=t.track_id, position=t.position, object_class=t.object_class)
        for t in frame.outputs.tracking.tracks
    ]


def match_tracks(dataset: EvaluationDataset, *, gate_m: float) -> dict[int, FrameMatching]:
    """Track-to-actor matching for every evaluated frame, keyed by frame index.

    Shared by the tracking, prediction, risk and adaptive-resolution sections
    so that all four agree on which track is which actor. Every ground-truth
    actor takes part in matching, eligible or not: a track on an actor that
    is outside the map bounds is still a track on something real, and must
    not be counted as unlabelled. Eligibility decides what enters a *rate*.
    """
    return {
        frame.index: match_frame(track_candidates(frame), list(frame.actors), gate_m=gate_m)
        for frame in dataset.frames
    }


@dataclass
class _GateAccumulator:
    eligible: int = 0
    matched: int = 0
    agreed: int = 0
    planar: list[float] = field(default_factory=list)
    spatial: list[float] = field(default_factory=list)

    def record(self, matching: FrameMatching, eligible: tuple[ReferenceActor, ...]) -> None:
        """Count the eligible actors and the matches that involve one of them."""
        eligible_ids = {actor.simulator_actor_id for actor in eligible}
        self.eligible += len(eligible_ids)
        for match in matching.matches:
            if match.simulator_actor_id not in eligible_ids:
                continue
            self.matched += 1
            self.agreed += int(match.class_agreed)
            self.planar.append(match.distance_planar_m)
            self.spatial.append(match.distance_3d_m)

    def metrics(self, gate_m: float) -> GateMetrics:
        return GateMetrics(
            gate_m=gate_m,
            eligible_pairs=self.eligible,
            matched_pairs=self.matched,
            match_rate=None if self.eligible == 0 else self.matched / self.eligible,
            class_agreement_rate=None if self.matched == 0 else self.agreed / self.matched,
            position_error_planar_m=Distribution.of(self.planar),
            position_error_3d_m=Distribution.of(self.spatial),
        )


def evaluate_detection(
    dataset: EvaluationDataset, configuration: EvaluationConfiguration
) -> DetectionEvaluation:
    """Is there a detection where each eligible actor is, at each gate?"""
    eligible_total = sum(len(frame.eligible_actors) for frame in dataset.frames)
    if eligible_total == 0:
        return DetectionEvaluation(
            status=MetricStatus.UNAVAILABLE,
            reason="no eligible ground-truth actor on any evaluated frame",
        )
    gates: list[GateMetrics] = []
    for gate_m in configuration.match_gates_m:
        accumulator = _GateAccumulator()
        for frame in dataset.frames:
            matching = match_frame(detection_candidates(frame), list(frame.actors), gate_m=gate_m)
            accumulator.record(matching, frame.eligible_actors)
        gates.append(accumulator.metrics(gate_m))
    return DetectionEvaluation(status=MetricStatus.MEASURED, gates=gates)


def evaluate_tracking(
    dataset: EvaluationDataset,
    configuration: EvaluationConfiguration,
    primary_matching: dict[int, FrameMatching],
) -> TrackingEvaluation:
    """Match rate, position and velocity error at every gate; continuity at the primary."""
    eligible_total = sum(len(frame.eligible_actors) for frame in dataset.frames)
    if eligible_total == 0:
        return TrackingEvaluation(
            status=MetricStatus.UNAVAILABLE,
            reason="no eligible ground-truth actor on any evaluated frame",
            unlabelled_track_frames=0,
            unlabelled_track_ids=0,
        )

    gates: list[TrackingGateMetrics] = []
    for gate_m in configuration.match_gates_m:
        matchings = (
            primary_matching
            if gate_m == configuration.primary_gate_m
            else match_tracks(dataset, gate_m=gate_m)
        )
        gates.append(_tracking_gate(dataset, matchings, gate_m))

    continuity = _continuity(dataset, primary_matching)

    unlabelled_frames = 0
    matched_ids: set[int] = set()
    all_ids: set[int] = set()
    for frame in dataset.frames:
        matching = primary_matching[frame.index]
        unlabelled_frames += len(matching.unmatched_candidate_ids)
        all_ids.update(t.track_id for t in frame.outputs.tracking.tracks)
        matched_ids.update(m.candidate_id for m in matching.matches)

    warnings: list[str] = []
    if any(not actor.eligible for frame in dataset.frames for actor in frame.actors):
        warnings.append(
            "some ground-truth actor-frames were ineligible (out of range or bounds) and "
            "were excluded from every rate"
        )

    return TrackingEvaluation(
        status=MetricStatus.MEASURED,
        warnings=warnings,
        gates=gates,
        continuity_gate_m=configuration.primary_gate_m,
        continuity=continuity,
        unlabelled_track_frames=unlabelled_frames,
        unlabelled_track_ids=len(all_ids - matched_ids),
    )


def _tracking_gate(
    dataset: EvaluationDataset, matchings: dict[int, FrameMatching], gate_m: float
) -> TrackingGateMetrics:
    accumulator = _GateAccumulator()
    velocity_errors: list[float] = []
    velocity_null = 0
    reference_missing = 0
    for frame in dataset.frames:
        matching = matchings[frame.index]
        accumulator.record(matching, frame.eligible_actors)
        tracks = {t.track_id: t for t in frame.outputs.tracking.tracks}
        actors = {a.simulator_actor_id: a for a in frame.eligible_actors}
        for match in matching.matches:
            actor = actors.get(match.simulator_actor_id)
            if actor is None:
                continue  # matched an ineligible actor: not part of any rate
            track = tracks[match.candidate_id]
            if track.velocity is None:
                velocity_null += 1
            elif actor.reference_velocity is None:
                reference_missing += 1
            else:
                reference = actor.reference_velocity
                velocity_errors.append(_planar_speed_error(track, reference.x, reference.y))
    base = accumulator.metrics(gate_m)
    return TrackingGateMetrics(
        **base.model_dump(),
        velocity_error_mps=Distribution.of(velocity_errors),
        velocity_null_pairs=velocity_null,
        velocity_reference_missing_pairs=reference_missing,
    )


def _planar_speed_error(track: TrackedObject, ref_x: float, ref_y: float) -> float:
    assert track.velocity is not None
    return ((track.velocity.x - ref_x) ** 2 + (track.velocity.y - ref_y) ** 2) ** 0.5


def _continuity(
    dataset: EvaluationDataset, matchings: dict[int, FrameMatching]
) -> list[ActorContinuity]:
    """Per actor: coverage, identity switches and fragmentation at one gate."""
    present: Counter[int] = Counter()
    eligible: Counter[int] = Counter()
    names: dict[int, str] = {}
    matched_track: dict[int, list[tuple[int, int, str]]] = {}
    for frame in dataset.frames:
        matching = matchings[frame.index]
        tracks = {t.track_id: t for t in frame.outputs.tracking.tracks}
        for actor in frame.actors:
            names[actor.simulator_actor_id] = actor.actor_id
            present[actor.simulator_actor_id] += 1
            if actor.eligible:
                eligible[actor.simulator_actor_id] += 1
            match = matching.match_for_actor(actor.simulator_actor_id)
            if match is not None and actor.eligible:
                matched_track.setdefault(actor.simulator_actor_id, []).append(
                    (frame.index, match.candidate_id, tracks[match.candidate_id].status.value)
                )

    continuity: list[ActorContinuity] = []
    for actor_id in sorted(present):
        history = matched_track.get(actor_id, [])
        ids: list[int] = []
        for _, track_id, _ in history:
            if track_id not in ids:
                ids.append(track_id)
        fragments, longest = _fragments([index for index, _, _ in history])
        continuity.append(
            ActorContinuity(
                actor_id=names[actor_id],
                simulator_actor_id=actor_id,
                frames_present=present[actor_id],
                frames_eligible=eligible[actor_id],
                frames_matched=len(history),
                coverage=None if eligible[actor_id] == 0 else len(history) / eligible[actor_id],
                track_ids=ids,
                id_switches=max(len(ids) - 1, 0),
                fragments=fragments,
                longest_fragment_frames=longest,
                matched_status_counts=dict(Counter(status for _, _, status in history)),
            )
        )
    return continuity


def _fragments(indices: list[int]) -> tuple[int, int]:
    """Count maximal runs of consecutive frame indices and the longest run."""
    if not indices:
        return 0, 0
    fragments = 1
    longest = current = 1
    for earlier, later in pairwise(indices):
        if later == earlier + 1:
            current += 1
        else:
            fragments += 1
            current = 1
        longest = max(longest, current)
    return fragments, longest


__all__ = [
    "detection_candidates",
    "evaluate_detection",
    "evaluate_tracking",
    "match_tracks",
    "track_candidates",
]
