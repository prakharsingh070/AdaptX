"""ADAPT-X data contracts.

All public models are re-exported here so consumers import from a single
module: ``from adaptx.models import PointCloudFrame``.
"""

from adaptx.models.common import (
    SCHEMA_VERSION,
    AdaptXModel,
    BoundingBox3D,
    CoordinateFrame,
    DataSource,
    Dimensions,
    ObjectClass,
    TimestampedModel,
    Vector3,
    utc_now,
)
from adaptx.models.map import (
    AdaptiveMap,
    AdaptiveMapCell,
    OccupancyState,
    ResolutionContext,
    ResolutionLevel,
)
from adaptx.models.objects import DetectedObject
from adaptx.models.point_cloud import (
    XYZ_FIELDS,
    XYZI_FIELDS,
    BasePointCloudFrame,
    PointCloudBounds,
    PointCloudFrame,
    PointCloudSummary,
    RawPointCloudFrame,
)
from adaptx.models.prediction import PredictedTrajectory, TrajectoryPoint
from adaptx.models.processing import (
    PointCloudProcessingResult,
    ProcessingMetrics,
    ProcessingStage,
    StageMetrics,
)
from adaptx.models.risk import ObjectRisk, RiskCell, RiskFactors, RiskField, RiskLevel
from adaptx.models.system import (
    CarlaConnectionStatus,
    CarlaStatus,
    ComponentReadiness,
    ComponentStatus,
    ImplementationStatus,
    LiDARSourceStatus,
    LiDARStatus,
    SystemMetrics,
    SystemState,
    SystemStatus,
)
from adaptx.models.tracking import TrackedObject, TrackStatus
from adaptx.models.vehicle import VehicleState

__all__ = [
    "SCHEMA_VERSION",
    "XYZI_FIELDS",
    "XYZ_FIELDS",
    "AdaptXModel",
    "AdaptiveMap",
    "AdaptiveMapCell",
    "BasePointCloudFrame",
    "BoundingBox3D",
    "CarlaConnectionStatus",
    "CarlaStatus",
    "ComponentReadiness",
    "ComponentStatus",
    "CoordinateFrame",
    "DataSource",
    "DetectedObject",
    "Dimensions",
    "ImplementationStatus",
    "LiDARSourceStatus",
    "LiDARStatus",
    "ObjectClass",
    "ObjectRisk",
    "OccupancyState",
    "PointCloudBounds",
    "PointCloudFrame",
    "PointCloudProcessingResult",
    "PointCloudSummary",
    "PredictedTrajectory",
    "ProcessingMetrics",
    "ProcessingStage",
    "RawPointCloudFrame",
    "ResolutionContext",
    "ResolutionLevel",
    "RiskCell",
    "RiskFactors",
    "RiskField",
    "RiskLevel",
    "StageMetrics",
    "SystemMetrics",
    "SystemState",
    "SystemStatus",
    "TimestampedModel",
    "TrackStatus",
    "TrackedObject",
    "TrajectoryPoint",
    "Vector3",
    "VehicleState",
    "utc_now",
]
