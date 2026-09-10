"""Tracking result contracts (Phase 4).

The tracker reports what it matched, what it could not match, and what it
retired. A frame where nothing was tracked is distinguishable from one where
detections arrived but none fell inside the association gate.

Durations are measured with :func:`time.perf_counter`. Nothing is estimated.
"""

from __future__ import annotations

from pydantic import Field, model_validator

from adaptx.models.common import AdaptXModel, TimestampedModel
from adaptx.models.tracking import TrackedObject, TrackStatus


class TrackingConfiguration(AdaptXModel):
    """Effective tracking configuration that produced a result.

    Carried on every result so a record is self-describing. A plain model
    rather than a reference to :class:`~adaptx.config.settings.TrackingSettings`,
    keeping :mod:`adaptx.models` free of any dependency on the configuration
    layer - the same split used for processing and detection.
    """

    max_association_distance_m: float
    require_class_match: bool
    max_size_ratio: float | None
    min_hits_to_confirm: int
    max_missed_frames: int
    max_missed_frames_tentative: int
    class_switch_hits: int
    velocity_smoothing: float
    max_timestep_s: float
    min_speed_for_heading_mps: float
    use_predicted_position_for_association: bool


class TrackingResult(TimestampedModel):
    """Everything one tracker update produced.

    ``tracks`` holds every live track after the update, including those that
    went unmatched this frame and are coasting. Retired tracks appear only as
    ids in ``deleted_track_ids``: their state is gone, and inventing a final
    position for them would be fabricating an observation.
    """

    frame_id: int = Field(ge=0)
    sensor_id: str = Field(min_length=1)
    tracker: str = Field(min_length=1, description="Identifier of the tracker that ran.")
    is_baseline: bool = Field(
        default=True,
        description="True while tracking is a geometric baseline rather than a learned model.",
    )

    tracks: list[TrackedObject] = Field(
        default_factory=list, description="All live tracks after this update."
    )
    new_track_ids: list[int] = Field(
        default_factory=list, description="Tracks created from unmatched detections."
    )
    deleted_track_ids: list[int] = Field(
        default_factory=list, description="Tracks retired this frame. Ids are never reused."
    )
    unmatched_detection_ids: list[int] = Field(
        default_factory=list,
        description="Frame-local detection ids that matched no existing track.",
    )
    unmatched_track_ids: list[int] = Field(
        default_factory=list, description="Live tracks that received no detection this frame."
    )

    detection_count: int = Field(ge=0, description="Detections handed to the tracker.")
    association_count: int = Field(ge=0, description="Detection-to-track matches made.")

    duration_ms: float = Field(ge=0.0, description="Whole tracker update, measured.")
    association_duration_ms: float = Field(default=0.0, ge=0.0)

    configuration: TrackingConfiguration

    @model_validator(mode="after")
    def _check_accounting(self) -> TrackingResult:
        accounted = self.association_count + len(self.unmatched_detection_ids)
        if self.detection_count != accounted:
            raise ValueError(
                f"detection_count ({self.detection_count}) must equal matched "
                f"({self.association_count}) + unmatched "
                f"({len(self.unmatched_detection_ids)})"
            )
        return self

    def _count(self, status: TrackStatus) -> int:
        return sum(1 for track in self.tracks if track.status is status)

    @property
    def active_track_count(self) -> int:
        """Live tracks after this update, in any state."""
        return len(self.tracks)

    @property
    def confirmed_count(self) -> int:
        """Tracks matched this frame and past the confirmation threshold."""
        return self._count(TrackStatus.CONFIRMED)

    @property
    def tentative_count(self) -> int:
        """Tracks not yet seen often enough to trust."""
        return self._count(TrackStatus.TENTATIVE)

    @property
    def coasting_count(self) -> int:
        """Tracks alive but unmatched this frame."""
        return self._count(TrackStatus.COASTING)

    @property
    def moving_track_count(self) -> int:
        """Tracks with measured motion. Excludes tracks whose velocity is unknown."""
        return sum(1 for track in self.tracks if track.is_moving)

    def counts_by_class(self) -> dict[str, int]:
        """Live tracks per object class."""
        counts: dict[str, int] = {}
        for track in self.tracks:
            counts[track.object_class.value] = counts.get(track.object_class.value, 0) + 1
        return counts
