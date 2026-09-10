"""Tracking-layer contract.

See ``docs/knowledge-base/08_tracking.md``. No implementation exists in
Phase 1.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from datetime import datetime

from adaptx.models.objects import DetectedObject
from adaptx.models.tracking_result import TrackingResult


class ObjectTracker(ABC):
    """Assigns persistent identity to detections across frames.

    Implementations consume :class:`~adaptx.models.objects.DetectedObject`
    output and never re-cluster point clouds: that work belongs to the
    detector and is not repeated here.

    Like the detector contract (ADR-022), ``update`` returns a full
    :class:`~adaptx.models.tracking_result.TrackingResult` rather than a bare
    list. A tracker is the only component that knows what it matched, what it
    could not match and what it retired, and a caller given only the surviving
    tracks cannot tell a quiet scene from one where every detection fell
    outside the association gate.

    An implementation must document its association method, initialisation,
    missed-detection and occlusion handling, and termination rules.
    """

    name: str = "object_tracker"
    #: True for geometric or heuristic baselines, false for a learned tracker.
    is_baseline: bool = True

    @abstractmethod
    def update(
        self,
        detections: list[DetectedObject],
        timestamp: datetime,
        *,
        frame_id: int = 0,
        sensor_id: str = "unknown",
    ) -> TrackingResult:
        """Associate ``detections`` with existing tracks and advance one frame."""

    @abstractmethod
    def reset(self) -> None:
        """Drop all tracks and reset identifier allocation."""
