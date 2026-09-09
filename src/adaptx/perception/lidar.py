"""Phase 1 LiDAR foundation.

:class:`FrameValidationProcessor` is deliberately minimal. It validates the
frame against the point-cloud contract and the configured limits, preserves
metadata and reports point count and bounds.

It performs **no** geometric processing: no region-of-interest filtering, no
voxelisation, no ground segmentation, no noise removal, no clustering. Those
belong to Phase 2 and are not implemented.
"""

from __future__ import annotations

from adaptx.config.settings import LiDARSettings
from adaptx.core.exceptions import InvalidPointCloudError
from adaptx.models.point_cloud import PointCloudFrame
from adaptx.perception.interfaces import LiDARProcessor


class FrameValidationProcessor(LiDARProcessor):
    """Validate an incoming frame and pass it through unchanged."""

    name = "frame_validation"

    def __init__(self, settings: LiDARSettings) -> None:
        self._settings = settings

    def process(self, frame: PointCloudFrame) -> PointCloudFrame:
        """Validate ``frame`` against the configured limits and return it.

        The frame is returned unchanged: the structural contract is enforced by
        :class:`~adaptx.models.point_cloud.PointCloudFrame` itself, and this
        processor adds only the configurable size limits.

        Raises:
            InvalidPointCloudError: the point count is outside
                ``[min_points, max_points]``.
        """
        self.validate(frame)
        return frame

    def validate(self, frame: PointCloudFrame) -> None:
        """Raise :class:`InvalidPointCloudError` if ``frame`` breaks a limit."""
        count = frame.point_count
        if count < self._settings.min_points:
            raise InvalidPointCloudError(
                f"frame has {count} points, minimum is {self._settings.min_points}",
                details={"point_count": count, "min_points": self._settings.min_points},
            )
        if count > self._settings.max_points:
            raise InvalidPointCloudError(
                f"frame has {count} points, maximum is {self._settings.max_points}",
                details={"point_count": count, "max_points": self._settings.max_points},
            )
