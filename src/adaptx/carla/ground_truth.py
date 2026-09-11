"""Simulator ground truth (Phase 9).

What the simulator *knows*, as opposed to what the sensor *sees*.

This is a deliberately separate path (ADR-045)::

                       +-> RawPointCloudFrame -> perception -> ... -> adaptive map
    CARLA simulation --+
                       +-> GroundTruthFrame   -> evaluation, debugging (Phase 11)

The two never meet during processing. Ground truth is for checking the
pipeline afterwards, and a detector that can see the answer measures nothing:
feeding any of this into detection, tracking, prediction, risk or resolution
would make every later accuracy figure meaningless while still looking like a
result.

Nothing in :mod:`adaptx.perception`, :mod:`adaptx.tracking`,
:mod:`adaptx.prediction`, :mod:`adaptx.mapping` or :mod:`adaptx.risk` imports
this module, and a test asserts that it stays that way.

Why its own contracts
---------------------
These are **not** :class:`~adaptx.models.objects.DetectedObject` or
:class:`~adaptx.models.tracking.TrackedObject`. Those types carry the
uncertainty, confidence and lifecycle of something that was *inferred*; ground
truth has none of that, and reusing a perception type would make a known
quantity indistinguishable from an estimated one at a glance.
"""

from __future__ import annotations

from datetime import datetime

from pydantic import Field

from adaptx.models.common import (
    AdaptXModel,
    CoordinateFrame,
    DataSource,
    Dimensions,
    ObjectClass,
    TimestampedModel,
    Vector3,
)

#: CARLA blueprint prefixes mapped to the classes ADAPT-X reasons about.
#: Used only to make ground truth readable beside perception output; it never
#: influences a perception decision.
_BLUEPRINT_CLASSES: tuple[tuple[str, ObjectClass], ...] = (
    ("walker.", ObjectClass.PEDESTRIAN),
    ("vehicle.bh.crossbike", ObjectClass.CYCLIST),
    ("vehicle.diamondback", ObjectClass.CYCLIST),
    ("vehicle.gazelle", ObjectClass.CYCLIST),
    ("vehicle.yamaha", ObjectClass.CYCLIST),
    ("vehicle.harley", ObjectClass.CYCLIST),
    ("vehicle.kawasaki", ObjectClass.CYCLIST),
    ("vehicle.vespa", ObjectClass.CYCLIST),
    ("vehicle.", ObjectClass.VEHICLE),
    ("static.", ObjectClass.OBSTACLE),
    ("traffic.", ObjectClass.OBSTACLE),
)


def classify_blueprint(type_id: str) -> ObjectClass:
    """Map a CARLA blueprint id to an ADAPT-X object class.

    Returns :attr:`~adaptx.models.common.ObjectClass.UNKNOWN` for anything
    unrecognised rather than guessing. Two-wheelers are matched before the
    general ``vehicle.`` prefix because CARLA files them under it.
    """
    lowered = type_id.lower()
    for prefix, object_class in _BLUEPRINT_CLASSES:
        if lowered.startswith(prefix):
            return object_class
    return ObjectClass.UNKNOWN


class GroundTruthActor(AdaptXModel):
    """One actor as the simulator knows it, in the ego frame.

    Positions and velocities are expressed relative to the ego vehicle in the
    ADAPT-X convention (ADR-009), so they line up directly with what detection
    and tracking produce - which is the comparison Phase 11 will want. The
    world-frame position is kept alongside for scenario analysis.

    Every value is **exact**: the simulator is not estimating. There is no
    confidence field here for that reason.
    """

    actor_id: int = Field(ge=0, description="Simulator actor id, stable for the actor's life.")
    type_id: str = Field(min_length=1, description="CARLA blueprint id, e.g. 'vehicle.audi.tt'.")
    object_class: ObjectClass = Field(
        description="Blueprint mapped to an ADAPT-X class, for readability beside perception."
    )
    is_ego: bool = Field(default=False, description="True for the vehicle carrying the sensor.")

    position: Vector3 = Field(description="Position relative to the ego, ADAPT-X frame.")
    velocity: Vector3 = Field(description="Velocity relative to the ego frame's axes, m/s.")
    heading_rad: float = Field(description="Yaw relative to the ego heading, counter-clockwise.")
    dimensions: Dimensions = Field(description="Exact bounding-box extents from the simulator.")
    world_position: Vector3 = Field(
        description="Absolute position in the map, ADAPT-X frame, for scenario analysis."
    )
    distance_m: float = Field(ge=0.0, description="Planar distance from the ego reference.")

    @property
    def speed_mps(self) -> float:
        """Exact scalar speed. Not an estimate."""
        return self.velocity.magnitude


class GroundTruthFrame(TimestampedModel):
    """Everything the simulator knew at one simulation frame.

    Carries the same ``frame_id`` and ``timestamp`` as the LiDAR frame captured
    on that tick, which is what makes the two joinable later without either
    having to know about the other.
    """

    frame_id: int = Field(ge=0, description="Simulator frame, matching the LiDAR frame.")
    map_name: str = Field(min_length=1)
    actors: list[GroundTruthActor] = Field(default_factory=list)
    ego_actor_id: int | None = Field(
        default=None, description="Null when no ego vehicle was identified."
    )
    coordinate_frame: CoordinateFrame = CoordinateFrame.EGO
    source: DataSource = Field(
        default=DataSource.SIMULATION,
        description="Always SIMULATION. Ground truth cannot come from a real sensor.",
    )

    @property
    def actor_count(self) -> int:
        """Actors recorded, including the ego."""
        return len(self.actors)

    def others(self) -> list[GroundTruthActor]:
        """Every actor except the ego."""
        return [actor for actor in self.actors if not actor.is_ego]

    def by_class(self, object_class: ObjectClass) -> list[GroundTruthActor]:
        """Actors of one class, ego excluded."""
        return [actor for actor in self.others() if actor.object_class is object_class]

    def nearest(self) -> GroundTruthActor | None:
        """The closest non-ego actor, or ``None`` when the ego is alone.

        Ties break on ``actor_id`` so the answer is deterministic.
        """
        others = self.others()
        if not others:
            return None
        return min(others, key=lambda actor: (actor.distance_m, actor.actor_id))


def build_ground_truth_frame(
    *,
    frame_id: int,
    timestamp: datetime,
    map_name: str,
    actors: list[GroundTruthActor],
    ego_actor_id: int | None,
) -> GroundTruthFrame:
    """Assemble a ground-truth frame, ordered deterministically by actor id."""
    return GroundTruthFrame(
        timestamp=timestamp,
        frame_id=frame_id,
        map_name=map_name,
        actors=sorted(actors, key=lambda actor: actor.actor_id),
        ego_actor_id=ego_actor_id,
    )
