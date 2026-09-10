"""Deterministic temporal sequences for tracking tests.

Builds :class:`~adaptx.models.objects.DetectedObject` instances directly rather
than running the detector, so a tracking test exercises tracking and nothing
else. When detection genuinely needs to be in the loop, the integration tests
build real point clouds and run the whole chain.

Timestamps are explicit and exact, which is what makes velocity assertions
meaningful: a known displacement over a known interval has one correct answer.

Coordinate convention (ADR-009): +x forward, +y left, +z up, metres.
"""

from __future__ import annotations

from datetime import UTC, datetime, timedelta

from adaptx.models.common import (
    BoundingBox3D,
    CoordinateFrame,
    DataSource,
    Dimensions,
    ObjectClass,
    Vector3,
)
from adaptx.models.objects import DetectedObject
from adaptx.models.tracking import TrackedObject, TrackStatus

#: Fixed epoch so every sequence is reproducible.
EPOCH = datetime(2026, 1, 1, 12, 0, 0, tzinfo=UTC)

#: Sizes matching the classifier's dimension bands, for realistic scenes.
SIZES: dict[ObjectClass, tuple[float, float, float]] = {
    ObjectClass.VEHICLE: (4.5, 1.9, 1.6),
    ObjectClass.PEDESTRIAN: (0.6, 0.5, 1.75),
    ObjectClass.CYCLIST: (1.8, 0.6, 1.7),
    ObjectClass.OBSTACLE: (0.8, 0.8, 0.6),
    ObjectClass.UNKNOWN: (2.0, 1.0, 1.0),
}


def at(seconds: float) -> datetime:
    """A timestamp ``seconds`` after the fixed epoch."""
    return EPOCH + timedelta(seconds=seconds)


def detection(
    position: tuple[float, float, float],
    *,
    object_id: int = 0,
    frame_id: int = 0,
    object_class: ObjectClass = ObjectClass.VEHICLE,
    timestamp: datetime | None = None,
    confidence: float = 0.8,
    point_count: int = 500,
    size: tuple[float, float, float] | None = None,
) -> DetectedObject:
    """One detection at a known position, with size matching its class."""
    centre = Vector3(x=position[0], y=position[1], z=position[2])
    length, width, height = size if size is not None else SIZES[object_class]
    return DetectedObject(
        timestamp=timestamp if timestamp is not None else EPOCH,
        object_id=object_id,
        frame_id=frame_id,
        object_class=object_class,
        position=centre,
        velocity=None,
        bounding_box=BoundingBox3D(
            center=centre,
            dimensions=Dimensions(length=length, width=width, height=height),
            yaw_rad=0.0,
        ),
        confidence=confidence,
        point_count=point_count,
        distance_m=(position[0] ** 2 + position[1] ** 2 + position[2] ** 2) ** 0.5,
        classifier="geometric_bands_v1",
        coordinate_frame=CoordinateFrame.EGO,
        source=DataSource.SYNTHETIC_TEST,
    )


def linear_motion(
    start: tuple[float, float, float],
    velocity: tuple[float, float, float],
    *,
    frames: int = 5,
    dt: float = 0.1,
    object_class: ObjectClass = ObjectClass.VEHICLE,
) -> list[tuple[list[DetectedObject], datetime]]:
    """A single object moving at constant velocity.

    Returns one ``(detections, timestamp)`` pair per frame, so a test can feed
    the tracker directly and assert the velocity it recovers.
    """
    sequence: list[tuple[list[DetectedObject], datetime]] = []
    for index in range(frames):
        elapsed = index * dt
        position = (
            start[0] + velocity[0] * elapsed,
            start[1] + velocity[1] * elapsed,
            start[2] + velocity[2] * elapsed,
        )
        sequence.append(
            (
                [
                    detection(
                        position,
                        frame_id=index,
                        object_class=object_class,
                        timestamp=at(elapsed),
                    )
                ],
                at(elapsed),
            )
        )
    return sequence


def stationary(
    position: tuple[float, float, float] = (10.0, 0.0, 0.0),
    *,
    frames: int = 5,
    dt: float = 0.1,
    object_class: ObjectClass = ObjectClass.VEHICLE,
) -> list[tuple[list[DetectedObject], datetime]]:
    """An object that does not move. Velocity should measure as zero, not null."""
    return linear_motion(position, (0.0, 0.0, 0.0), frames=frames, dt=dt, object_class=object_class)


def crossing_pair(
    *, frames: int = 7, dt: float = 0.1
) -> list[tuple[list[DetectedObject], datetime]]:
    """Two vehicles on opposite lateral courses that pass close to each other.

    The hardest case for centroid association: at the crossing frame both
    detections are near both tracks, and only the gate and the greedy ordering
    decide which claims which.
    """
    sequence: list[tuple[list[DetectedObject], datetime]] = []
    for index in range(frames):
        elapsed = index * dt
        left = (20.0, -6.0 + 2.0 * elapsed * 10.0 * dt * 5.0, 0.0)
        right = (20.0, 6.0 - 2.0 * elapsed * 10.0 * dt * 5.0, 0.0)
        sequence.append(
            (
                [
                    detection(left, object_id=0, frame_id=index, timestamp=at(elapsed)),
                    detection(right, object_id=1, frame_id=index, timestamp=at(elapsed)),
                ],
                at(elapsed),
            )
        )
    return sequence


def parallel_pair(
    *, frames: int = 5, dt: float = 0.1, separation_m: float = 6.0
) -> list[tuple[list[DetectedObject], datetime]]:
    """Two same-class objects travelling side by side, well apart."""
    sequence: list[tuple[list[DetectedObject], datetime]] = []
    for index in range(frames):
        elapsed = index * dt
        forward = 10.0 + 10.0 * elapsed
        sequence.append(
            (
                [
                    detection(
                        (forward, separation_m / 2, 0.0),
                        object_id=0,
                        frame_id=index,
                        timestamp=at(elapsed),
                    ),
                    detection(
                        (forward, -separation_m / 2, 0.0),
                        object_id=1,
                        frame_id=index,
                        timestamp=at(elapsed),
                    ),
                ],
                at(elapsed),
            )
        )
    return sequence


def track(
    position: tuple[float, float, float] = (10.0, 0.0, 0.0),
    velocity: tuple[float, float, float] | None = (2.0, 0.0, 0.0),
    *,
    track_id: int = 0,
    status: TrackStatus = TrackStatus.CONFIRMED,
    object_class: ObjectClass = ObjectClass.VEHICLE,
    hits: int = 5,
    age_frames: int = 5,
    missed_frames: int = 0,
    last_seen_offset_s: float = 0.0,
    confidence: float = 0.8,
    coordinate_frame: CoordinateFrame = CoordinateFrame.EGO,
    source: DataSource = DataSource.SYNTHETIC_TEST,
) -> TrackedObject:
    """A tracked object with an explicit state, for prediction tests.

    Built directly rather than produced by the tracker: these tests exercise
    prediction, and routing them through association would make a prediction
    failure indistinguishable from a tracking one. The tracker's own tests
    already prove it produces states of this shape.

    ``velocity=None`` is the important case, and it is the caller's job to ask
    for it: it means *not measurable*, never zero (ADR-023).

    ``last_seen_offset_s`` is how long **before** the epoch the track was last
    observed, so a coasting track can be given a measured staleness.
    """
    length, width, height = SIZES[object_class]
    centre = Vector3(x=position[0], y=position[1], z=position[2])
    return TrackedObject(
        timestamp=EPOCH,
        track_id=track_id,
        object_class=object_class,
        status=status,
        position=centre,
        velocity=(
            None if velocity is None else Vector3(x=velocity[0], y=velocity[1], z=velocity[2])
        ),
        bounding_box=BoundingBox3D(
            center=centre,
            dimensions=Dimensions(length=length, width=width, height=height),
            yaw_rad=0.0,
        ),
        point_count=500,
        hits=hits,
        first_seen=EPOCH - timedelta(seconds=age_frames * 0.1),
        confidence=confidence,
        age_frames=age_frames,
        missed_frames=missed_frames,
        last_seen=EPOCH - timedelta(seconds=last_seen_offset_s),
        coordinate_frame=coordinate_frame,
        source=source,
    )
