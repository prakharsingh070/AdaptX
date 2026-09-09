"""Perception layer: point-cloud processing and object detection.

Phase 1 implements frame validation and summarisation only. No detector
exists; :class:`adaptx.perception.interfaces.ObjectDetector` is a contract.
"""

from adaptx.perception.interfaces import LiDARProcessor, ObjectDetector
from adaptx.perception.lidar import FrameValidationProcessor

__all__ = ["FrameValidationProcessor", "LiDARProcessor", "ObjectDetector"]
