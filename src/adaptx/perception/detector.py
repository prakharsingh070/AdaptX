"""Geometric object detection (Phase 3).

Consumes the **non-ground** output of
:class:`~adaptx.perception.pipeline.LiDARProcessingPipeline` and turns spatial
clusters into candidate objects:

    non-ground points -> clustering -> geometry -> filtering -> classification

The detector does no filtering, voxelisation or segmentation of its own. Those
belong to the processing pipeline and are not repeated here.

**This is a deterministic geometric baseline**, built to be replaced. It has no
learned model, so it cannot recognise anything - it measures clusters and
compares them against dimension bands. A future ``MLObjectDetector`` implements
the same :class:`~adaptx.perception.interfaces.ObjectDetector` contract and
drops in without the API or service layer changing.

Not implemented here: oriented bounding boxes, velocity (a single frame cannot
show motion), tracking, occlusion reasoning, camera fusion, semantic
segmentation.
"""

from __future__ import annotations

import time

import numpy as np

from adaptx.config.settings import DetectionSettings
from adaptx.core.logging import get_logger
from adaptx.models.common import BoundingBox3D, Dimensions, ObjectClass, Vector3
from adaptx.models.detection import (
    ClusterRejection,
    DetectionConfiguration,
    DetectionResult,
    RejectedCluster,
)
from adaptx.models.objects import DetectedObject
from adaptx.models.point_cloud import PointCloudFrame
from adaptx.perception.classification import GeometricClassifier
from adaptx.perception.clustering import GridConnectedComponentClusterer
from adaptx.perception.interfaces import ObjectDetector

logger = get_logger(__name__)


class _ClusterGeometry:
    """Per-cluster measurements, computed for every cluster at once.

    Held as arrays rather than per-cluster objects: the number of clusters can
    reach thousands and a Python object per cluster would dominate the cost of
    an otherwise vectorised pass.
    """

    __slots__ = ("centroid", "count", "maximum", "minimum")

    def __init__(self, points: np.ndarray, labels: np.ndarray, cluster_count: int) -> None:
        xyz = points[:, :3]
        self.count = np.bincount(labels, minlength=cluster_count)

        # Sorting by label makes each cluster contiguous, so min, max and sum
        # are one reduceat each instead of a loop over clusters.
        order = np.argsort(labels, kind="stable")
        grouped = labels[order]
        start = np.flatnonzero(np.concatenate(([True], grouped[1:] != grouped[:-1])))
        ordered_xyz = xyz[order]

        self.minimum = np.minimum.reduceat(ordered_xyz, start, axis=0)
        self.maximum = np.maximum.reduceat(ordered_xyz, start, axis=0)
        totals = np.add.reduceat(ordered_xyz, start, axis=0)
        self.centroid = totals / self.count[:, None]

    @property
    def extents(self) -> np.ndarray:
        """``(K, 3)`` axis-aligned extents."""
        return self.maximum - self.minimum


class GeometricObjectDetector(ObjectDetector):
    """Detects objects by clustering non-ground points and measuring them."""

    name = "geometric_detector_v1"
    #: True while detection is a geometric baseline rather than a trained model.
    is_baseline = True

    def __init__(self, settings: DetectionSettings) -> None:
        self._settings = settings
        self._clusterer = GridConnectedComponentClusterer(settings)
        self._classifier = GeometricClassifier()
        self._floor_z_m: float | None = None

    @property
    def configuration(self) -> DetectionConfiguration:
        """Snapshot of the settings that shape this detector's behaviour."""
        settings = self._settings
        return DetectionConfiguration(
            cluster_tolerance_m=settings.cluster_tolerance_m,
            min_cluster_points=settings.min_cluster_points,
            max_cluster_points=settings.max_cluster_points,
            min_height_m=settings.min_height_m,
            max_height_m=settings.max_height_m,
            min_footprint_m=settings.min_footprint_m,
            max_footprint_m=settings.max_footprint_m,
            max_bottom_height_m=settings.max_bottom_height_m,
        )

    def detect(self, frame: PointCloudFrame, *, floor_z_m: float | None = None) -> DetectionResult:
        """Detect objects in ``frame``.

        ``frame`` is expected to hold the **non-ground** points produced by the
        processing pipeline. Passing an unsegmented cloud is not an error, but
        the ground surface will then form one enormous cluster and be rejected
        by the footprint filter rather than recognised as ground.

        Args:
            frame: The non-ground points.
            floor_z_m: The road surface height in the frame's own coordinates,
                as the ground stage estimated it (the median z of the points it
                removed). When given, a cluster whose lowest point sits more
                than ``max_bottom_height_m`` above it is rejected as
                ``ELEVATED``: overhead signs, foliage and awnings are not road
                users. ``None`` (no ground stage) disables the rule.
        """
        started = time.perf_counter()
        self._floor_z_m = floor_z_m
        points = frame.points
        input_count = int(points.shape[0])

        mark = time.perf_counter()
        labels = self._clusterer.cluster(points)
        clustering_ms = (time.perf_counter() - mark) * 1000.0

        cluster_count = int(labels.max()) + 1 if labels.size else 0

        mark = time.perf_counter()
        objects, rejected = self._build(frame, points, labels, cluster_count)
        classification_ms = (time.perf_counter() - mark) * 1000.0

        duration_ms = (time.perf_counter() - started) * 1000.0

        logger.debug(
            "detection complete",
            extra={
                "context": {
                    "frame_id": frame.frame_id,
                    "input_points": input_count,
                    "clusters": cluster_count,
                    "objects": len(objects),
                    "rejected": len(rejected),
                    "duration_ms": round(duration_ms, 3),
                }
            },
        )

        return DetectionResult(
            timestamp=frame.timestamp,
            frame_id=frame.frame_id,
            sensor_id=frame.sensor_id,
            detector=self.name,
            is_baseline=self.is_baseline,
            objects=objects,
            rejected=rejected,
            input_point_count=input_count,
            non_ground_point_count=input_count,
            cluster_count=cluster_count,
            duration_ms=duration_ms,
            clustering_duration_ms=clustering_ms,
            classification_duration_ms=classification_ms,
            configuration=self.configuration,
            input_summary=frame.summary(),
        )

    def _build(
        self,
        frame: PointCloudFrame,
        points: np.ndarray,
        labels: np.ndarray,
        cluster_count: int,
    ) -> tuple[list[DetectedObject], list[RejectedCluster]]:
        """Measure every cluster, filter it, and classify what survives."""
        objects: list[DetectedObject] = []
        rejected: list[RejectedCluster] = []
        if cluster_count == 0:
            return objects, rejected

        geometry = _ClusterGeometry(points, labels, cluster_count)
        extents = geometry.extents

        for cluster_id in range(cluster_count):
            count = int(geometry.count[cluster_id])
            extent = extents[cluster_id]
            centroid = geometry.centroid[cluster_id]

            rejection = self._reject(count, extent, float(geometry.minimum[cluster_id][2]))
            if rejection is not None:
                reason, measured, threshold = rejection
                rejected.append(
                    RejectedCluster(
                        cluster_id=cluster_id,
                        reason=reason,
                        point_count=count,
                        centroid=_vector(centroid),
                        extents=_vector(extent),
                        measured_value=measured,
                        threshold=threshold,
                    )
                )
                continue

            object_class, confidence = self._classifier.classify(
                float(extent[0]), float(extent[1]), float(extent[2])
            )
            objects.append(
                self._detected_object(
                    frame, len(objects), centroid, extent, count, object_class, confidence
                )
            )

        return objects, rejected

    def _reject(
        self, count: int, extent: np.ndarray, bottom_z: float
    ) -> tuple[ClusterRejection, float, float] | None:
        """Return why this cluster fails the filter, or ``None`` if it passes.

        Checked in a fixed order so a cluster failing several tests is always
        attributed to the same one, which keeps the rejection record stable.
        """
        settings = self._settings
        height = float(extent[2])
        footprint = float(max(extent[0], extent[1]))
        floor = self._floor_z_m
        if floor is not None and settings.max_bottom_height_m is not None:
            above_floor = bottom_z - floor
            if above_floor > settings.max_bottom_height_m:
                return ClusterRejection.ELEVATED, above_floor, settings.max_bottom_height_m

        if count < settings.min_cluster_points:
            return ClusterRejection.TOO_FEW_POINTS, float(count), float(settings.min_cluster_points)
        if count > settings.max_cluster_points:
            return (
                ClusterRejection.TOO_MANY_POINTS,
                float(count),
                float(settings.max_cluster_points),
            )
        if height < settings.min_height_m:
            return ClusterRejection.TOO_SHORT, height, settings.min_height_m
        if height > settings.max_height_m:
            return ClusterRejection.TOO_TALL, height, settings.max_height_m
        if footprint < settings.min_footprint_m:
            return ClusterRejection.FOOTPRINT_TOO_SMALL, footprint, settings.min_footprint_m
        if footprint > settings.max_footprint_m:
            return ClusterRejection.FOOTPRINT_TOO_LARGE, footprint, settings.max_footprint_m
        return None

    def _detected_object(
        self,
        frame: PointCloudFrame,
        object_id: int,
        centroid: np.ndarray,
        extent: np.ndarray,
        point_count: int,
        object_class: ObjectClass,
        confidence: float,
    ) -> DetectedObject:
        """Assemble one detection, preserving the frame's metadata."""
        position = _vector(centroid)
        return DetectedObject(
            timestamp=frame.timestamp,
            object_id=object_id,
            frame_id=frame.frame_id,
            object_class=object_class,
            position=position,
            # Velocity needs two frames. A single-frame detector cannot know it,
            # and inventing one would be a fabricated measurement.
            velocity=None,
            bounding_box=BoundingBox3D(
                center=position,
                dimensions=Dimensions(
                    # A cluster can be perfectly flat on an axis; Dimensions
                    # requires a positive extent, so clamp to a millimetre.
                    length=max(float(extent[0]), 1e-3),
                    width=max(float(extent[1]), 1e-3),
                    height=max(float(extent[2]), 1e-3),
                ),
                yaw_rad=0.0,  # Axis-aligned: no orientation is estimated.
            ),
            confidence=confidence,
            point_count=point_count,
            distance_m=float(np.linalg.norm(centroid)),
            classifier=self._classifier.name,
            is_baseline_classification=self._classifier.is_baseline,
            coordinate_frame=frame.coordinate_frame,
            source=frame.source,
        )


def _vector(values: np.ndarray) -> Vector3:
    """Convert a 3-element array to the shared vector contract."""
    return Vector3(x=float(values[0]), y=float(values[1]), z=float(values[2]))


def build_detector(settings: DetectionSettings) -> GeometricObjectDetector:
    """Construct the configured geometric detector."""
    return GeometricObjectDetector(settings)
