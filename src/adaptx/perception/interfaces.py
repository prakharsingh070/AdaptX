"""Perception-layer contracts.

Implementations are plugged in without changing the API, services or models.
See ``docs/knowledge-base/04_lidar-knowledge.md``.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from adaptx.models.detection import DetectionResult
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

    Implementations consume the **non-ground** output of the processing
    pipeline. They do not filter, voxelise or segment: that work belongs to
    :class:`adaptx.perception.pipeline.LiDARProcessingPipeline` and is not
    repeated here.

    The contract returns a full :class:`~adaptx.models.detection.DetectionResult`
    rather than a bare list, because a detector is the only thing that knows its
    own timing, its candidate count and what it rejected. Returning just the
    objects would leave callers unable to tell "found nothing" from "found
    candidates and turned them all down".
    """

    name: str = "object_detector"
    #: True for geometric or heuristic baselines, false for a trained model.
    is_baseline: bool = True

    @abstractmethod
    def detect(self, frame: PointCloudFrame) -> DetectionResult:
        """Detect objects in ``frame``.

        The detections carry frame-local ``object_id`` values; stable identity
        is assigned later by
        :class:`adaptx.tracking.interfaces.ObjectTracker`.
        """
