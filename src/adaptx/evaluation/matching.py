"""Matching perceived objects to ground-truth actors, one frame at a time.

Greedy nearest neighbour, planar, gated (``docs/EVALUATION.md`` §3):

1. every (candidate, actor) pair is given its planar distance;
2. pairs are sorted by ``(distance, candidate_id, actor_id)``, so ties break
   on ids and the result never depends on input order;
3. the sorted list is walked and a pair is accepted when neither side is
   taken and the distance is within the gate.

Optimal assignment was considered and not used. The catalogue never puts more
than two scenario actors in a frame, greedy and optimal coincide unless two
actors sit within one gate of one candidate, and no dependency exists or is
justified for the Hungarian method (ADR-052). If a scenario with dense actors
is ever added, this is the module to revisit.

The candidate is anything with an id and a position: a detection or a track.
Matching does not look at class - the classifier is under evaluation too -
but records whether the classes agreed.
"""

from __future__ import annotations

from dataclasses import dataclass

from adaptx.evaluation.dataset import ReferenceActor
from adaptx.models.common import ObjectClass, Vector3


@dataclass(frozen=True)
class Candidate:
    """A perceived object as the matcher sees it."""

    candidate_id: int
    position: Vector3
    object_class: ObjectClass


@dataclass(frozen=True)
class Match:
    """One accepted (candidate, actor) pair."""

    candidate_id: int
    simulator_actor_id: int
    distance_planar_m: float
    distance_3d_m: float
    class_agreed: bool


@dataclass(frozen=True)
class FrameMatching:
    """The outcome of matching one frame at one gate."""

    gate_m: float
    matches: tuple[Match, ...]
    unmatched_candidate_ids: tuple[int, ...]
    unmatched_actor_ids: tuple[int, ...]

    def match_for_actor(self, simulator_actor_id: int) -> Match | None:
        """The match involving ``simulator_actor_id``, if any."""
        for match in self.matches:
            if match.simulator_actor_id == simulator_actor_id:
                return match
        return None

    def match_for_candidate(self, candidate_id: int) -> Match | None:
        """The match involving ``candidate_id``, if any."""
        for match in self.matches:
            if match.candidate_id == candidate_id:
                return match
        return None


def planar_distance(a: Vector3, b: Vector3) -> float:
    """Distance in the xy plane."""
    return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5


def match_frame(
    candidates: list[Candidate], actors: list[ReferenceActor], *, gate_m: float
) -> FrameMatching:
    """Match ``candidates`` to ``actors`` greedily within ``gate_m``.

    Deterministic for a given input: ties in distance are broken by
    ``candidate_id`` and then by actor id.
    """
    if gate_m <= 0.0:
        raise ValueError("the match gate must be positive")

    pairs: list[tuple[float, int, int, float, bool]] = []
    for candidate in candidates:
        for actor in actors:
            planar = planar_distance(candidate.position, actor.position)
            if planar > gate_m:
                continue
            pairs.append(
                (
                    planar,
                    candidate.candidate_id,
                    actor.simulator_actor_id,
                    candidate.position.distance_to(actor.position),
                    candidate.object_class is actor.object_class,
                )
            )
    pairs.sort(key=lambda pair: (pair[0], pair[1], pair[2]))

    taken_candidates: set[int] = set()
    taken_actors: set[int] = set()
    matches: list[Match] = []
    for planar, candidate_id, actor_id, spatial, agreed in pairs:
        if candidate_id in taken_candidates or actor_id in taken_actors:
            continue
        taken_candidates.add(candidate_id)
        taken_actors.add(actor_id)
        matches.append(
            Match(
                candidate_id=candidate_id,
                simulator_actor_id=actor_id,
                distance_planar_m=planar,
                distance_3d_m=spatial,
                class_agreed=agreed,
            )
        )

    return FrameMatching(
        gate_m=gate_m,
        matches=tuple(matches),
        unmatched_candidate_ids=tuple(
            sorted(c.candidate_id for c in candidates if c.candidate_id not in taken_candidates)
        ),
        unmatched_actor_ids=tuple(
            sorted(a.simulator_actor_id for a in actors if a.simulator_actor_id not in taken_actors)
        ),
    )


__all__ = ["Candidate", "FrameMatching", "Match", "match_frame", "planar_distance"]
