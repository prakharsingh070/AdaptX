"""LiDAR pipeline benchmarking (Phase 2C).

Measures how fast the LiDAR processing pipeline runs on deterministic synthetic
datasets, and defines the conventional fixed-resolution profile that future
adaptive work will be compared against.

Scope: **speed only** - point-cloud processing, and since Phase 3 object
detection as well. It is not the Phase 11 ADAPT-X benchmark, which will
compare fixed-resolution against adaptive-resolution perception on detection
quality, tracking and safety behaviour. Adaptive resolution does not exist,
and no dataset here carries labels, so neither that comparison nor any
accuracy measurement is possible or attempted.
"""

from adaptx.benchmark.baseline import (
    BASELINE_PROFILE,
    BASELINE_VOXEL_SIZE_M,
    DETECTION_PROFILE,
    FILTER_ONLY_PROFILE,
    build_baseline_pipeline,
    filter_only_settings,
    fixed_resolution_settings,
)
from adaptx.benchmark.datasets import (
    EDGE_CASES,
    SIZE_LADDER,
    DatasetDescription,
    DatasetScenario,
    generate_dataset,
)
from adaptx.benchmark.detection import (
    DetectionBenchmarkReport,
    DetectionBenchmarkResult,
    run_detection_benchmark,
)
from adaptx.benchmark.models import (
    BenchmarkReport,
    BenchmarkResult,
    EnvironmentInfo,
    TimingSummary,
)
from adaptx.benchmark.runner import BenchmarkRunner, describe_environment
from adaptx.benchmark.tracking import (
    TrackingBenchmarkCase,
    TrackingBenchmarkReport,
    run_tracking_benchmark,
)

__all__ = [
    "BASELINE_PROFILE",
    "BASELINE_VOXEL_SIZE_M",
    "DETECTION_PROFILE",
    "EDGE_CASES",
    "FILTER_ONLY_PROFILE",
    "SIZE_LADDER",
    "BenchmarkReport",
    "BenchmarkResult",
    "BenchmarkRunner",
    "DatasetDescription",
    "DatasetScenario",
    "DetectionBenchmarkReport",
    "DetectionBenchmarkResult",
    "EnvironmentInfo",
    "TimingSummary",
    "TrackingBenchmarkCase",
    "TrackingBenchmarkReport",
    "build_baseline_pipeline",
    "describe_environment",
    "filter_only_settings",
    "fixed_resolution_settings",
    "generate_dataset",
    "run_detection_benchmark",
    "run_tracking_benchmark",
]
