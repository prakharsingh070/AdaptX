"""Perception layer: point-cloud processing and object detection.

Phase 2A implements input validation, non-finite removal, ROI filtering and
range filtering. No detector exists;
:class:`adaptx.perception.interfaces.ObjectDetector` is a contract.
"""

from adaptx.perception.interfaces import LiDARProcessor, ObjectDetector
from adaptx.perception.lidar import FrameValidationProcessor
from adaptx.perception.preprocessing import PointCloudPreprocessor, build_preprocessor

__all__ = [
    "FrameValidationProcessor",
    "LiDARProcessor",
    "ObjectDetector",
    "PointCloudPreprocessor",
    "build_preprocessor",
]
