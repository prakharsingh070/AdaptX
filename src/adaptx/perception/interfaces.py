"""Perception-layer contracts.

Implementations are plugged in without changing the API, services or models.
See ``docs/knowledge-base/04_lidar-knowledge.md``.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from adaptx.models.objects import DetectedObject
from adaptx.models.point_cloud import PointCloudFrame


class LiDARProcessor(ABC):
    """Transforms a raw point-cloud frame into a processed frame.

    A processor may filter, downsample, remove ground points or transform
    coordinates. It must preserve ``frame_id``, ``sensor_id``, ``timestamp``
    and ``source`` so provenance survives the pipeline, and must declare the
    ``coordinate_frame`` of its output.
    """

    #: Stable identifier used in logs, metrics and experiment records.
    name: str = "lidar_processor"

    @abstractmethod
    def process(self, frame: PointCloudFrame) -> PointCloudFrame:
        """Return a processed copy of ``frame``.

        Raises:
            adaptx.core.exceptions.InvalidPointCloudError: input violates the
                point-cloud contract or the configured limits.
        """


class ObjectDetector(ABC):
    """Produces per-frame object detections from a point cloud.

    Not implemented in Phase 1.
    """

    name: str = "object_detector"

    @abstractmethod
    def detect(self, frame: PointCloudFrame) -> list[DetectedObject]:
        """Return the objects detected in ``frame``.

        The returned detections carry frame-local ``object_id`` values; stable
        identity is assigned later by
        :class:`adaptx.tracking.interfaces.ObjectTracker`.
        """
