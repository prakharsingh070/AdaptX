"""Tracking-layer contract.

See ``docs/knowledge-base/08_tracking.md``. No implementation exists in
Phase 1.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from datetime import datetime

from adaptx.models.objects import DetectedObject
from adaptx.models.tracking import TrackedObject


class ObjectTracker(ABC):
    """Assigns persistent identity to detections across frames.

    An implementation must document its association method, initialisation,
    missed-detection and occlusion handling, and termination rules.
    """

    name: str = "object_tracker"

    @abstractmethod
    def update(self, detections: list[DetectedObject], timestamp: datetime) -> list[TrackedObject]:
        """Associate ``detections`` with existing tracks and return all live tracks."""

    @abstractmethod
    def reset(self) -> None:
        """Drop all tracks and reset identifier allocation."""
