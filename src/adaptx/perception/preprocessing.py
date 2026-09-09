"""Phase 2A LiDAR preprocessing.

Turns a raw scan into a validated, cleaned, spatially restricted point cloud::

    RawPointCloudFrame
        -> input validation
        -> non-finite (NaN / Inf) removal
        -> ROI filtering
        -> range filtering
        -> PointCloudFrame

Coordinate convention (ADR-009)
-------------------------------
A right-handed frame with the origin at the sensor, in metres:

* **+x forward**, -x behind
* **+y left**, -y right
* **+z up**, -z down

This is the convention Phase 1 already implied, since
:attr:`adaptx.models.common.BoundingBox3D.yaw_rad` and
:attr:`adaptx.models.vehicle.VehicleState.heading_rad` are defined as
counter-clockwise about +z from the +x axis, which only describes a
right-handed frame. It matches ISO 8855 and ROS REP-103.

CARLA uses a **left-handed** frame (+y right). Converting is the CARLA
boundary's job in Phase 9; this module assumes its input is already in the
ADAPT-X convention and does not transform coordinates.

Range convention
----------------
Range is the full 3D Euclidean distance from the sensor origin::

    d = sqrt(x^2 + y^2 + z^2)

not the ground-plane distance ``sqrt(x^2 + y^2)``. Minimum range models the
sensor's blind zone and returns off the ego vehicle itself, which are physical
3D phenomena: a point 0.3 m directly above the sensor is inside the blind zone
even though its planar distance is 0. The implementation compares *squared*
distances against squared bounds so no square root is taken over the array;
that is arithmetically identical for non-negative bounds.

Boundary semantics
------------------
All bounds are **inclusive** on both ends. A point exactly on an ROI face, at
exactly ``min_range_m`` or at exactly ``max_range_m``, is kept.

Not implemented here
--------------------
Voxelisation / downsampling, ground segmentation, statistical outlier removal
and clustering. Those are later work; this module does not approximate them.
"""

from __future__ import annotations

import time

import numpy as np

from adaptx.config.settings import LiDARSettings
from adaptx.core.exceptions import InvalidPointCloudError
from adaptx.core.logging import get_logger
from adaptx.models.point_cloud import BasePointCloudFrame, PointCloudFrame
from adaptx.models.processing import (
    PointCloudProcessingResult,
    ProcessingMetrics,
    ProcessingStage,
    StageMetrics,
)
from adaptx.perception.interfaces import LiDARProcessor

logger = get_logger(__name__)


class PointCloudPreprocessor(LiDARProcessor):
    """Validate, clean and spatially restrict a raw point cloud.

    Stateless and deterministic: the same input and configuration always
    produce the same output, and point order is preserved.
    """

    name = "preprocessing_v1"

    def __init__(self, settings: LiDARSettings) -> None:
        self._settings = settings

    # -- LiDARProcessor contract ------------------------------------------
    def process(self, frame: PointCloudFrame) -> PointCloudFrame:
        """Run the pipeline and return only the processed frame.

        This satisfies the Phase 1 :class:`LiDARProcessor` contract. Use
        :meth:`run` when the per-stage counts and timing are wanted.
        """
        return self.run(frame).frame

    # -- Full pipeline -----------------------------------------------------
    def run(self, frame: BasePointCloudFrame) -> PointCloudProcessingResult:
        """Preprocess ``frame`` and return the result with its measured metrics.

        Accepts a :class:`~adaptx.models.point_cloud.RawPointCloudFrame` (which
        may contain NaN or infinities) or an already-validated
        :class:`~adaptx.models.point_cloud.PointCloudFrame`.

        Raises:
            InvalidPointCloudError: the frame violates the configured
                point-count limits.
        """
        started = time.perf_counter()

        input_summary = frame.summary()
        points = frame.points
        input_count = int(points.shape[0])

        self._validate(input_count)

        # One boolean mask per stage, all evaluated against the original array,
        # then a single compaction at the end. Masks are combined so a point
        # rejected by an earlier stage is never counted again by a later one,
        # which keeps the counts a true partition of the input while allocating
        # only one output array instead of one per stage.
        finite_mask = self._finite_mask(points)
        roi_mask = self._roi_mask(points)
        range_mask = self._range_mask(points)

        surviving_invalid = finite_mask
        surviving_roi = surviving_invalid & roi_mask
        surviving_range = surviving_roi & range_mask

        after_invalid = int(np.count_nonzero(surviving_invalid))
        after_roi = int(np.count_nonzero(surviving_roi))
        after_range = int(np.count_nonzero(surviving_range))

        invalid_rejected = input_count - after_invalid
        roi_rejected = after_invalid - after_roi
        range_rejected = after_roi - after_range

        # `points[mask]` performs the single copy of surviving rows. np.ascontiguousarray
        # is not needed: fancy indexing already returns a fresh contiguous array.
        processed_points = points[surviving_range]

        processed = PointCloudFrame(
            schema_version=frame.schema_version,
            timestamp=frame.timestamp,
            frame_id=frame.frame_id,
            sensor_id=frame.sensor_id,
            points=processed_points,
            fields=frame.fields,
            coordinate_frame=frame.coordinate_frame,
            source=frame.source,
        )

        duration_ms = (time.perf_counter() - started) * 1000.0

        metrics = ProcessingMetrics(
            timestamp=frame.timestamp,
            processor=self.name,
            input_point_count=input_count,
            invalid_point_count=invalid_rejected,
            roi_rejected_count=roi_rejected,
            range_rejected_count=range_rejected,
            output_point_count=after_range,
            duration_ms=duration_ms,
            stages=[
                StageMetrics(
                    stage=ProcessingStage.VALIDATION,
                    input_points=input_count,
                    output_points=input_count,
                    rejected_points=0,
                ),
                StageMetrics(
                    stage=ProcessingStage.INVALID_REMOVAL,
                    input_points=input_count,
                    output_points=after_invalid,
                    rejected_points=invalid_rejected,
                ),
                StageMetrics(
                    stage=ProcessingStage.ROI_FILTER,
                    input_points=after_invalid,
                    output_points=after_roi,
                    rejected_points=roi_rejected,
                ),
                StageMetrics(
                    stage=ProcessingStage.RANGE_FILTER,
                    input_points=after_roi,
                    output_points=after_range,
                    rejected_points=range_rejected,
                ),
            ],
        )

        # One frame-level line, never per point. DEBUG because this runs on the
        # hot path; a frame that loses everything is worth a WARNING.
        context = {
            "frame_id": frame.frame_id,
            "sensor_id": frame.sensor_id,
            "input_points": input_count,
            "invalid": invalid_rejected,
            "roi_rejected": roi_rejected,
            "range_rejected": range_rejected,
            "output_points": after_range,
            "duration_ms": round(duration_ms, 3),
        }
        if input_count > 0 and after_range == 0:
            logger.warning("preprocessing removed every point", extra={"context": context})
        else:
            logger.debug("point cloud preprocessed", extra={"context": context})

        return PointCloudProcessingResult(
            frame=processed, input_summary=input_summary, metrics=metrics
        )

    # -- Stages ------------------------------------------------------------
    def _validate(self, point_count: int) -> None:
        """Enforce the configured point-count limits.

        Array shape, dimensionality, dtype and column layout are already
        guaranteed by the frame contract itself, so this stage only adds the
        limits that are configuration rather than structure.
        """
        if point_count < self._settings.min_points:
            raise InvalidPointCloudError(
                f"frame has {point_count} points, minimum is {self._settings.min_points}",
                details={"point_count": point_count, "min_points": self._settings.min_points},
            )
        if point_count > self._settings.max_points:
            raise InvalidPointCloudError(
                f"frame has {point_count} points, maximum is {self._settings.max_points}",
                details={"point_count": point_count, "max_points": self._settings.max_points},
            )

    @staticmethod
    def _finite_mask(points: np.ndarray) -> np.ndarray:
        """True where every column of the row is finite.

        Intensity is included deliberately: a row with a NaN intensity is not
        trustworthy, and silently keeping it would hide a sensor fault.
        """
        if points.size == 0:
            return np.ones(points.shape[0], dtype=bool)
        return np.isfinite(points).all(axis=1)

    def _roi_mask(self, points: np.ndarray) -> np.ndarray:
        """True for points inside the configured axis-aligned ROI box.

        Bounds are inclusive. Comparisons against NaN are False, so non-finite
        rows are excluded here too; they are attributed to the invalid-removal
        stage rather than to the ROI because the masks are combined in pipeline
        order.
        """
        settings = self._settings
        if points.size == 0:
            return np.ones(points.shape[0], dtype=bool)
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        return (
            (x >= settings.roi_x_min_m)
            & (x <= settings.roi_x_max_m)
            & (y >= settings.roi_y_min_m)
            & (y <= settings.roi_y_max_m)
            & (z >= settings.roi_z_min_m)
            & (z <= settings.roi_z_max_m)
        )

    def _range_mask(self, points: np.ndarray) -> np.ndarray:
        """True for points whose 3D distance from the origin is within bounds.

        Squared distances are compared against squared bounds, which avoids a
        square root over the whole array and is equivalent because both bounds
        are non-negative.
        """
        settings = self._settings
        if points.size == 0:
            return np.ones(points.shape[0], dtype=bool)
        xyz = points[:, :3]
        squared = np.einsum("ij,ij->i", xyz, xyz)
        return (squared >= settings.min_range_m**2) & (squared <= settings.max_range_m**2)


def build_preprocessor(settings: LiDARSettings) -> PointCloudPreprocessor:
    """Construct the configured preprocessing pipeline."""
    return PointCloudPreprocessor(settings)


__all__ = ["PointCloudPreprocessor", "build_preprocessor"]
