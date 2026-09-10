"""Detection-to-track association (Phase 4).

Method: gated greedy nearest neighbour
--------------------------------------
Every (track, detection) pair is scored by the distance between the track's
expected position and the detection centroid. Pairs outside the gate, or
failing a compatibility check, are discarded outright. The survivors are
considered in ascending distance and claimed greedily, so the closest plausible
pair matches first and neither side is ever claimed twice.

Why greedy rather than optimal
------------------------------
Hungarian assignment minimises total distance and would occasionally do better
where two tracks compete for two detections. It needs `scipy.optimize`, and the
project has repeatedly declined dependencies that the baseline does not
require (ADR-014, ADR-020). Greedy nearest neighbour is explainable line by
line, which matters more here than optimality: when tracking goes wrong during
a close crossing, the reason should be readable.

Determinism
-----------
Ties are broken by ``(distance, track_id, detection index)``, so the result
never depends on dictionary ordering, on the order detections arrive in, or on
which of two equidistant candidates NumPy happened to visit first.

Cost
----
``O(T x D)`` scoring for ``T`` tracks and ``D`` detections, then a sort of the
surviving pairs. Per-detection work is hoisted out of the inner loop and the
gate is tested on squared distance, so the constant factor is small - but the
quadratic term is real. Measured on this machine: 0.06 ms at 5 objects, 0.6 ms
at 25, 8.5 ms at 100, and 206 ms at 500.

That is acceptable at the object counts this pipeline actually produces (the
large benchmark scene yields under a hundred), and it is the wrong shape for
thousands. Bucketing tracks into a spatial grid - the same trick the noise
filter and clusterer use - would make it near-linear, and is the fix to reach
for if object counts ever justify the extra machinery.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from adaptx.config.settings import TrackingSettings
from adaptx.models.common import ObjectClass, Vector3
from adaptx.models.objects import DetectedObject


@dataclass(frozen=True)
class Candidate:
    """A track and a detection that could plausibly be the same object."""

    track_index: int
    track_id: int
    detection_index: int
    distance_m: float

    @property
    def sort_key(self) -> tuple[float, int, int]:
        """Deterministic ordering: closest first, then by stable identifiers."""
        return (self.distance_m, self.track_id, self.detection_index)


@dataclass
class AssociationOutcome:
    """The result of matching one frame's detections against live tracks."""

    matches: list[Candidate] = field(default_factory=list)
    unmatched_track_indices: list[int] = field(default_factory=list)
    unmatched_detection_indices: list[int] = field(default_factory=list)


def _largest_dimension(detection: DetectedObject) -> float:
    return max(detection.extents)


def class_compatible(
    track_class: ObjectClass, detection_class: ObjectClass, settings: TrackingSettings
) -> bool:
    """Whether two classes may describe the same object.

    ``UNKNOWN`` is compatible with everything: the geometric classifier reports
    it whenever a shape is ambiguous, which happens often, and refusing to
    associate on that basis would fragment perfectly good tracks.
    """
    if not settings.require_class_match:
        return True
    if track_class is ObjectClass.UNKNOWN or detection_class is ObjectClass.UNKNOWN:
        return True
    return track_class is detection_class


def size_compatible(
    track_size_m: float | None, detection_size_m: float, settings: TrackingSettings
) -> bool:
    """Whether two objects are close enough in size to be the same one.

    Guards against a pedestrian-sized cluster inheriting a lorry's track when
    it happens to fall inside the distance gate.
    """
    if settings.max_size_ratio is None or track_size_m is None:
        return True
    larger = max(track_size_m, detection_size_m)
    smaller = min(track_size_m, detection_size_m)
    if smaller <= 0.0:
        return True
    return (larger / smaller) <= settings.max_size_ratio


def associate(
    track_states: list[tuple[int, Vector3, ObjectClass, float | None]],
    detections: list[DetectedObject],
    settings: TrackingSettings,
) -> AssociationOutcome:
    """Match detections to tracks.

    Args:
        track_states: One ``(track_id, expected_position, object_class,
            largest_dimension_m)`` per live track, in track order. The expected
            position is the tracker's own - extrapolated or last observed - and
            is never treated as an observation.
        detections: This frame's detections, in whatever order they arrived.
        settings: Gating and compatibility configuration.

    Returns:
        Matches plus the indices left over on each side.
    """
    # Per-detection values are hoisted out of the inner loop. Recomputing an
    # object's largest dimension, and re-reading pydantic attributes, once per
    # (track, detection) pair rather than once per detection was measured as
    # the dominant cost of association at high object counts.
    prepared = [
        (
            detection.position.x,
            detection.position.y,
            detection.position.z,
            detection.object_class,
            _largest_dimension(detection),
        )
        for detection in detections
    ]
    gate = settings.max_association_distance_m
    gate_squared = gate * gate

    candidates: list[Candidate] = []
    for track_index, (track_id, expected, track_class, track_size) in enumerate(track_states):
        expected_x, expected_y, expected_z = expected.x, expected.y, expected.z
        for detection_index, (x, y, z, detection_class, detection_size) in enumerate(prepared):
            if not class_compatible(track_class, detection_class, settings):
                continue
            if not size_compatible(track_size, detection_size, settings):
                continue

            # Squared distance first: the square root only matters for the
            # candidates that survive the gate, and most do not.
            offset_x = expected_x - x
            offset_y = expected_y - y
            offset_z = expected_z - z
            squared = offset_x * offset_x + offset_y * offset_y + offset_z * offset_z
            if not math.isfinite(squared) or squared > gate_squared:
                continue
            candidates.append(
                Candidate(
                    track_index=track_index,
                    track_id=track_id,
                    detection_index=detection_index,
                    distance_m=math.sqrt(squared),
                )
            )

    candidates.sort(key=lambda candidate: candidate.sort_key)

    claimed_tracks: set[int] = set()
    claimed_detections: set[int] = set()
    matches: list[Candidate] = []
    for candidate in candidates:
        if candidate.track_index in claimed_tracks:
            continue
        if candidate.detection_index in claimed_detections:
            continue
        claimed_tracks.add(candidate.track_index)
        claimed_detections.add(candidate.detection_index)
        matches.append(candidate)

    return AssociationOutcome(
        matches=matches,
        unmatched_track_indices=[
            index for index in range(len(track_states)) if index not in claimed_tracks
        ],
        unmatched_detection_indices=[
            index for index in range(len(detections)) if index not in claimed_detections
        ],
    )
