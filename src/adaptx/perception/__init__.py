"""Perception layer: point-cloud processing and object detection.

Phase 3 adds geometric object detection on the non-ground output.

Phase 2A implements input validation, non-finite removal, ROI filtering and
range filtering. Phase 2B adds opt-in voxel downsampling, baseline ground
segmentation and baseline noise filtering. No detector exists;
:class:`adaptx.perception.interfaces.ObjectDetector` is a contract.
"""

from adaptx.perception.classification import GeometricClassifier
from adaptx.perception.clustering import GridConnectedComponentClusterer
from adaptx.perception.detector import GeometricObjectDetector, build_detector
from adaptx.perception.ground import GroundSegmenter
from adaptx.perception.interfaces import LiDARProcessor, ObjectDetector
from adaptx.perception.lidar import FrameValidationProcessor
from adaptx.perception.noise import NoiseFilter
from adaptx.perception.pipeline import LiDARProcessingPipeline, build_pipeline
from adaptx.perception.voxel import VoxelDownsampler

__all__ = [
    "FrameValidationProcessor",
    "GeometricClassifier",
    "GeometricObjectDetector",
    "GridConnectedComponentClusterer",
    "GroundSegmenter",
    "LiDARProcessingPipeline",
    "LiDARProcessor",
    "NoiseFilter",
    "ObjectDetector",
    "VoxelDownsampler",
    "build_detector",
    "build_pipeline",
]
