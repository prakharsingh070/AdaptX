"""The ego corridor: which objects are in the ego's path, now or by prediction.

One rule, used by the speed governor to pick what governs the target and by
the scene snapshot to label every object for the dashboard, so the two never
disagree. Coordinates are the ADAPT-X sensor/ego frame: +X forward, +Y left,
origin at the LiDAR (ADR-009).

- ``IN_PATH``: the object is ahead (``x > 0``) and within ``half_width_m`` of
  the ego axis now.
- ``CROSSING``: not in the corridor now, but a point of its predicted
  trajectory (Phase 5, constant velocity) is - it is on its way in.
- ``BEHIND``: ``x <= 0`` and not predicted to enter.
- ``OUTSIDE``: ahead but beside the corridor, and not predicted to enter.

The corridor is straight along +X. On a bend, roadside geometry enters it;
that is a known limitation (Experiment 014), not something this module hides.
"""

from __future__ import annotations

from collections.abc import Iterable
from enum import StrEnum

from adaptx.models.common import Vector3


class PathRelation(StrEnum):
    """How an object relates to the ego's straight-ahead corridor."""

    IN_PATH = "IN_PATH"
    CROSSING = "CROSSING"
    BEHIND = "BEHIND"
    OUTSIDE = "OUTSIDE"


def in_corridor(position: Vector3, half_width_m: float) -> bool:
    """Ahead and within the corridor's half-width."""
    return position.x > 0.0 and abs(position.y) <= half_width_m


def path_relation(
    position: Vector3, predicted_points: Iterable[Vector3], half_width_m: float
) -> PathRelation:
    """Classify one object against the corridor from its position and predicted path."""
    if in_corridor(position, half_width_m):
        return PathRelation.IN_PATH
    if any(in_corridor(point, half_width_m) for point in predicted_points):
        return PathRelation.CROSSING
    if position.x <= 0.0:
        return PathRelation.BEHIND
    return PathRelation.OUTSIDE


def governs(relation: PathRelation) -> bool:
    """Whether an object with this relation sets the governor's target speed."""
    return relation in (PathRelation.IN_PATH, PathRelation.CROSSING)


__all__ = ["PathRelation", "governs", "in_corridor", "path_relation"]
