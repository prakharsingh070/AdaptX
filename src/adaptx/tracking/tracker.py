"""Baseline temporal object tracking (Phase 4).

Consumes the :class:`~adaptx.models.objects.DetectedObject` output of Phase 3
and gives objects persistent identity across frames:

    detections -> association -> update matched -> create new
               -> age unmatched -> retire stale -> TrackedObject[]

**This is a deterministic geometric baseline.** It has no appearance model, no
re-identification and no learned motion model. Tracking quality is bounded by
detection quality: a frame the detector misses is a frame the tracker must
coast through, and a cluster the detector merges is one object as far as the
tracker can tell.

Velocity
--------
Measured, never assumed: ``(position - previous_position) / dt`` with ``dt``
taken from the frame timestamps, so a variable frame interval is handled
correctly. Until a track has two observations there is no velocity, and it is
reported ``None`` rather than a zero vector - zero would claim a measured
standstill (ADR-023). A gap longer than ``max_timestep_s``, or a non-positive
``dt``, also yields ``None``: extrapolating across it would be invention.

Both the raw frame-to-frame value and an exponentially smoothed one are
reported, so smoothing can never hide a jump - the observation that caused it
stays visible in ``observed_velocity``.

Heading is ``atan2(vy, vx)`` only above ``min_speed_for_heading_mps``. Below
that the direction describes measurement noise, not travel, so it is ``None``.

Not implemented here: trajectory prediction, collision reasoning, appearance
features, re-identification after deletion. A track retired for missing too
many frames does not come back; an object reappearing gets a new id.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from datetime import datetime

from adaptx.config.settings import TrackingSettings
from adaptx.core.logging import get_logger
from adaptx.models.common import BoundingBox3D, CoordinateFrame, DataSource, ObjectClass, Vector3
from adaptx.models.objects import DetectedObject
from adaptx.models.tracking import TrackedObject, TrackStatus
from adaptx.models.tracking_result import TrackingConfiguration, TrackingResult
from adaptx.tracking.association import associate
from adaptx.tracking.interfaces import ObjectTracker

logger = get_logger(__name__)


@dataclass
class _Track:
    """Mutable per-track state.

    Kept separate from :class:`~adaptx.models.tracking.TrackedObject`, which is
    the immutable snapshot published to callers. Mixing the two would let a
    consumer mutate tracker state by holding a result.
    """

    track_id: int
    object_class: ObjectClass
    position: Vector3
    first_seen: datetime
    last_seen: datetime
    confidence: float
    point_count: int
    bounding_box: BoundingBox3D | None
    coordinate_frame: CoordinateFrame
    source: DataSource

    previous_position: Vector3 | None = None
    velocity: Vector3 | None = None
    observed_velocity: Vector3 | None = None
    acceleration: Vector3 | None = None
    heading_rad: float | None = None
    predicted_position: Vector3 | None = None

    status: TrackStatus = TrackStatus.TENTATIVE
    hits: int = 1
    age_frames: int = 1
    missed_frames: int = 0
    #: Class seen repeatedly that disagrees with the current one, and how often.
    pending_class: ObjectClass | None = None
    pending_class_hits: int = 0

    @property
    def largest_dimension_m(self) -> float | None:
        """Largest extent of the most recent detection, for the size gate."""
        if self.bounding_box is None:
            return None
        dimensions = self.bounding_box.dimensions
        return max(dimensions.length, dimensions.width, dimensions.height)

    @property
    def was_confirmed(self) -> bool:
        """Whether this track has ever reached CONFIRMED."""
        return self.status in (TrackStatus.CONFIRMED, TrackStatus.COASTING)


class GeometricObjectTracker(ObjectTracker):
    """Tracks detections across frames by gated nearest-neighbour association."""

    name = "geometric_tracker_v1"
    #: True while tracking is a geometric baseline rather than a learned model.
    is_baseline = True

    def __init__(self, settings: TrackingSettings) -> None:
        self._settings = settings
        self._tracks: list[_Track] = []
        self._next_track_id = 0
        self._last_timestamp: datetime | None = None

    # -- state -------------------------------------------------------------
    @property
    def configuration(self) -> TrackingConfiguration:
        """Snapshot of the settings that shape this tracker's behaviour."""
        settings = self._settings
        return TrackingConfiguration(
            max_association_distance_m=settings.max_association_distance_m,
            require_class_match=settings.require_class_match,
            max_size_ratio=settings.max_size_ratio,
            min_hits_to_confirm=settings.min_hits_to_confirm,
            max_missed_frames=settings.max_missed_frames,
            max_missed_frames_tentative=settings.max_missed_frames_tentative,
            class_switch_hits=settings.class_switch_hits,
            velocity_smoothing=settings.velocity_smoothing,
            max_timestep_s=settings.max_timestep_s,
            min_speed_for_heading_mps=settings.min_speed_for_heading_mps,
            use_predicted_position_for_association=(
                settings.use_predicted_position_for_association
            ),
        )

    @property
    def live_track_count(self) -> int:
        """Number of tracks currently alive."""
        return len(self._tracks)

    def reset(self) -> None:
        """Drop all tracks and restart identifier allocation.

        Ids restart from zero, so a reset run is reproducible. Within a single
        run an id is never reused once retired.
        """
        self._tracks = []
        self._next_track_id = 0
        self._last_timestamp = None
        logger.info("tracker reset")

    # -- update ------------------------------------------------------------
    def update(
        self,
        detections: list[DetectedObject],
        timestamp: datetime,
        *,
        frame_id: int = 0,
        sensor_id: str = "unknown",
    ) -> TrackingResult:
        """Advance every track by one frame.

        Args:
            detections: This frame's detections, in any order.
            timestamp: Frame time. Velocity is measured against this.
            frame_id: Frame identifier recorded on the result.
            sensor_id: Sensor identifier recorded on the result.
        """
        started = time.perf_counter()

        for track in self._tracks:
            track.predicted_position = self._expected_position(track, timestamp)

        mark = time.perf_counter()
        outcome = associate(
            [
                (
                    track.track_id,
                    track.predicted_position or track.position,
                    track.object_class,
                    track.largest_dimension_m,
                )
                for track in self._tracks
            ],
            detections,
            self._settings,
        )
        association_ms = (time.perf_counter() - mark) * 1000.0

        for candidate in outcome.matches:
            self._apply_detection(
                self._tracks[candidate.track_index],
                detections[candidate.detection_index],
                timestamp,
            )

        for index in outcome.unmatched_track_indices:
            self._age(self._tracks[index])

        new_ids = [
            self._create(detections[index], timestamp)
            for index in outcome.unmatched_detection_indices
        ]

        deleted_ids = self._retire()
        self._last_timestamp = timestamp
        duration_ms = (time.perf_counter() - started) * 1000.0

        logger.debug(
            "tracker updated",
            extra={
                "context": {
                    "frame_id": frame_id,
                    "detections": len(detections),
                    "matched": len(outcome.matches),
                    "new": len(new_ids),
                    "deleted": len(deleted_ids),
                    "live": len(self._tracks),
                    "duration_ms": round(duration_ms, 3),
                }
            },
        )

        return TrackingResult(
            timestamp=timestamp,
            frame_id=frame_id,
            sensor_id=sensor_id,
            tracker=self.name,
            is_baseline=self.is_baseline,
            tracks=[self._snapshot(track, timestamp) for track in self._tracks],
            new_track_ids=new_ids,
            deleted_track_ids=deleted_ids,
            unmatched_detection_ids=[
                detections[index].object_id for index in outcome.unmatched_detection_indices
            ],
            unmatched_track_ids=[
                self._tracks[index].track_id
                for index in outcome.unmatched_track_indices
                if index < len(self._tracks)
            ],
            detection_count=len(detections),
            association_count=len(outcome.matches),
            duration_ms=duration_ms,
            association_duration_ms=association_ms,
            configuration=self.configuration,
        )

    # -- internals ---------------------------------------------------------
    def _expected_position(self, track: _Track, timestamp: datetime) -> Vector3:
        """Where the track should be now, for gating only.

        Extrapolated from the last measured velocity when one exists. This is
        tracker state, never published as an observation, and emphatically not
        a trajectory prediction - that is a later phase.
        """
        if not self._settings.use_predicted_position_for_association:
            return track.position
        if track.velocity is None:
            return track.position

        elapsed = (timestamp - track.last_seen).total_seconds()
        if elapsed <= 0.0 or elapsed > self._settings.max_timestep_s:
            return track.position
        return Vector3(
            x=track.position.x + track.velocity.x * elapsed,
            y=track.position.y + track.velocity.y * elapsed,
            z=track.position.z + track.velocity.z * elapsed,
        )

    def _apply_detection(
        self, track: _Track, detection: DetectedObject, timestamp: datetime
    ) -> None:
        """Fold an associated detection into a track."""
        elapsed = (timestamp - track.last_seen).total_seconds()
        observed = self._observed_velocity(track.position, detection.position, elapsed)

        if observed is not None:
            previous_velocity = track.velocity
            track.observed_velocity = observed
            track.velocity = self._smooth(previous_velocity, observed)
            track.acceleration = self._acceleration(previous_velocity, track.velocity, elapsed)
            track.heading_rad = self._heading(track.velocity)

        track.previous_position = track.position
        track.position = detection.position
        track.bounding_box = detection.bounding_box
        track.point_count = detection.point_count
        track.confidence = detection.confidence
        track.last_seen = timestamp
        track.hits += 1
        track.age_frames += 1
        track.missed_frames = 0
        self._update_class(track, detection.object_class)

        if track.hits >= self._settings.min_hits_to_confirm:
            track.status = TrackStatus.CONFIRMED
        else:
            track.status = TrackStatus.TENTATIVE

    def _observed_velocity(
        self, previous: Vector3, current: Vector3, elapsed_s: float
    ) -> Vector3 | None:
        """Raw frame-to-frame velocity, or ``None`` when it cannot be measured.

        Returns ``None`` for a non-positive interval (out-of-order or duplicate
        timestamps) and for a gap beyond ``max_timestep_s``, where dividing by
        the interval would produce a number with no physical meaning.
        """
        if not math.isfinite(elapsed_s) or elapsed_s <= 0.0:
            return None
        if elapsed_s > self._settings.max_timestep_s:
            return None
        return Vector3(
            x=(current.x - previous.x) / elapsed_s,
            y=(current.y - previous.y) / elapsed_s,
            z=(current.z - previous.z) / elapsed_s,
        )

    def _smooth(self, previous: Vector3 | None, observed: Vector3) -> Vector3:
        """Exponential moving average of velocity.

        The first observation is taken as-is: there is nothing to blend with,
        and seeding from zero would drag a genuine first measurement toward a
        standstill that was never observed.
        """
        if previous is None:
            return observed
        weight = self._settings.velocity_smoothing
        return Vector3(
            x=weight * observed.x + (1.0 - weight) * previous.x,
            y=weight * observed.y + (1.0 - weight) * previous.y,
            z=weight * observed.z + (1.0 - weight) * previous.z,
        )

    @staticmethod
    def _acceleration(
        previous_velocity: Vector3 | None, velocity: Vector3, elapsed_s: float
    ) -> Vector3 | None:
        """Change in velocity per second, or ``None`` without two velocities."""
        if previous_velocity is None or elapsed_s <= 0.0:
            return None
        return Vector3(
            x=(velocity.x - previous_velocity.x) / elapsed_s,
            y=(velocity.y - previous_velocity.y) / elapsed_s,
            z=(velocity.z - previous_velocity.z) / elapsed_s,
        )

    def _heading(self, velocity: Vector3) -> float | None:
        """Direction of travel, or ``None`` when the speed is too low to mean one."""
        planar_speed = math.hypot(velocity.x, velocity.y)
        if planar_speed < self._settings.min_speed_for_heading_mps:
            return None
        return math.atan2(velocity.y, velocity.x)

    def _update_class(self, track: _Track, observed: ObjectClass) -> None:
        """Adopt a new class only once it has been seen consistently.

        An unknown track takes any class immediately - that is new information.
        A known track changing class needs ``class_switch_hits`` consecutive
        agreeing observations, so one ambiguous frame cannot flip a vehicle
        into a pedestrian and back.
        """
        if observed is track.object_class:
            track.pending_class = None
            track.pending_class_hits = 0
            return

        if track.object_class is ObjectClass.UNKNOWN:
            track.object_class = observed
            track.pending_class = None
            track.pending_class_hits = 0
            return

        if observed is ObjectClass.UNKNOWN:
            # An ambiguous frame is not evidence against a known class.
            return

        if track.pending_class is observed:
            track.pending_class_hits += 1
        else:
            track.pending_class = observed
            track.pending_class_hits = 1

        if track.pending_class_hits >= self._settings.class_switch_hits:
            track.object_class = observed
            track.pending_class = None
            track.pending_class_hits = 0

    @staticmethod
    def _age(track: _Track) -> None:
        """Advance an unmatched track. No detection is fabricated for it."""
        track.missed_frames += 1
        track.age_frames += 1
        if track.status is TrackStatus.CONFIRMED:
            track.status = TrackStatus.COASTING

    def _create(self, detection: DetectedObject, timestamp: datetime) -> int:
        """Start a tentative track from an unmatched detection."""
        track_id = self._next_track_id
        self._next_track_id += 1
        self._tracks.append(
            _Track(
                track_id=track_id,
                object_class=detection.object_class,
                position=detection.position,
                first_seen=timestamp,
                last_seen=timestamp,
                confidence=detection.confidence,
                point_count=detection.point_count,
                bounding_box=detection.bounding_box,
                coordinate_frame=detection.coordinate_frame,
                source=detection.source,
                status=(
                    TrackStatus.CONFIRMED
                    if self._settings.min_hits_to_confirm <= 1
                    else TrackStatus.TENTATIVE
                ),
            )
        )
        return track_id

    def _retire(self) -> list[int]:
        """Drop tracks that have missed too many frames. Ids are not reused."""
        settings = self._settings
        survivors: list[_Track] = []
        deleted: list[int] = []
        for track in self._tracks:
            limit = (
                settings.max_missed_frames
                if track.was_confirmed
                else settings.max_missed_frames_tentative
            )
            if track.missed_frames > limit:
                deleted.append(track.track_id)
            else:
                survivors.append(track)
        self._tracks = survivors
        return deleted

    def _snapshot(self, track: _Track, timestamp: datetime) -> TrackedObject:
        """Immutable published view of a track."""
        return TrackedObject(
            timestamp=timestamp,
            track_id=track.track_id,
            object_class=track.object_class,
            status=track.status,
            position=track.position,
            previous_position=track.previous_position,
            velocity=track.velocity,
            observed_velocity=track.observed_velocity,
            acceleration=track.acceleration,
            heading_rad=track.heading_rad,
            predicted_position=track.predicted_position,
            bounding_box=track.bounding_box,
            point_count=track.point_count,
            hits=track.hits,
            first_seen=track.first_seen,
            confidence=track.confidence,
            age_frames=track.age_frames,
            missed_frames=track.missed_frames,
            last_seen=track.last_seen,
            coordinate_frame=track.coordinate_frame,
            source=track.source,
        )


def build_tracker(settings: TrackingSettings) -> GeometricObjectTracker:
    """Construct the configured baseline tracker."""
    return GeometricObjectTracker(settings)
