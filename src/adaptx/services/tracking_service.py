"""Stateful tracking service (Phase 4).

Tracking is the first part of ADAPT-X that carries state between requests: a
track only exists because of what earlier frames contained. That state is owned
here and reached through the application context, never through a module-level
global (ADR-025), so it has a defined lifetime, can be reset deliberately, and
does not leak between tests.

Concurrency
-----------
One tracker instance serves the whole process, guarded by a lock. Two clients
posting frames to the same backend therefore feed **one** track set, which is
correct for a single vehicle with one sensor stream and wrong for anything
else. Per-session tracking would need a session concept the API does not have;
until it does, the shared-state behaviour is documented rather than disguised.
"""

from __future__ import annotations

from datetime import datetime
from threading import Lock

from adaptx.config.settings import TrackingSettings
from adaptx.core.logging import get_logger
from adaptx.models.objects import DetectedObject
from adaptx.models.tracking_result import TrackingConfiguration, TrackingResult
from adaptx.tracking.tracker import GeometricObjectTracker

logger = get_logger(__name__)


class TrackingService:
    """Owns the tracker and the state it accumulates across frames."""

    def __init__(
        self, settings: TrackingSettings, tracker: GeometricObjectTracker | None = None
    ) -> None:
        self._settings = settings
        self._tracker = tracker if tracker is not None else GeometricObjectTracker(settings)
        self._lock = Lock()
        self._frames_tracked = 0
        self._last_result: TrackingResult | None = None

    @property
    def tracker(self) -> GeometricObjectTracker:
        """The tracker under management."""
        return self._tracker

    @property
    def configuration(self) -> TrackingConfiguration:
        """Effective tracking configuration."""
        return self._tracker.configuration

    @property
    def frames_tracked(self) -> int:
        """Frames processed since the last reset."""
        with self._lock:
            return self._frames_tracked

    @property
    def live_track_count(self) -> int:
        """Tracks currently alive."""
        with self._lock:
            return self._tracker.live_track_count

    @property
    def last_result(self) -> TrackingResult | None:
        """The most recent update, or ``None`` before the first frame."""
        with self._lock:
            return self._last_result

    def update(
        self,
        detections: list[DetectedObject],
        timestamp: datetime,
        *,
        frame_id: int = 0,
        sensor_id: str = "unknown",
    ) -> TrackingResult:
        """Advance tracking by one frame.

        Held under the lock for the whole update: the tracker mutates its track
        list in place, so two concurrent frames would interleave and corrupt it.
        """
        with self._lock:
            result = self._tracker.update(
                detections, timestamp, frame_id=frame_id, sensor_id=sensor_id
            )
            self._frames_tracked += 1
            self._last_result = result
            return result

    def reset(self) -> None:
        """Drop all tracks and start again from an empty state."""
        with self._lock:
            self._tracker.reset()
            self._frames_tracked = 0
            self._last_result = None
        logger.info("tracking state reset")

    def summary(self) -> dict[str, object]:
        """Compact state summary for status and telemetry.

        Counts and identifiers only. Per-track geometry is returned by the
        tracking endpoint; repeating it on every telemetry tick would push
        frame data down a channel meant for status.
        """
        with self._lock:
            result = self._last_result
            live = self._tracker.live_track_count
            frames = self._frames_tracked

        summary: dict[str, object] = {
            "tracker": self._tracker.name,
            "is_baseline": self._tracker.is_baseline,
            "frames_tracked": frames,
            "active_tracks": live,
            "configuration": self.configuration.model_dump(mode="json"),
        }
        if result is None:
            summary["note"] = "no frame has been tracked since the last reset"
            return summary

        summary.update(
            {
                "confirmed": result.confirmed_count,
                "tentative": result.tentative_count,
                "coasting": result.coasting_count,
                "moving": result.moving_track_count,
                "new_last_frame": len(result.new_track_ids),
                "deleted_last_frame": len(result.deleted_track_ids),
                "track_ids": [track.track_id for track in result.tracks],
                "counts_by_class": result.counts_by_class(),
            }
        )
        return summary
