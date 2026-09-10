"""Application lifecycle and service wiring.

:class:`ApplicationContext` owns every long-lived service and is created once
per process at startup. Routes reach it through FastAPI dependencies rather
than module-level globals, which keeps tests able to build an isolated context.
"""

from __future__ import annotations

from dataclasses import dataclass

from adaptx.config.settings import Settings, get_settings
from adaptx.core.logging import configure_logging, get_logger
from adaptx.perception.detector import GeometricObjectDetector
from adaptx.perception.lidar import FrameValidationProcessor
from adaptx.perception.pipeline import LiDARProcessingPipeline
from adaptx.risk.baseline import BaselineProximityRiskEngine
from adaptx.services.adaptive_mapping_service import AdaptiveMappingService
from adaptx.services.carla_service import CarlaService
from adaptx.services.lidar_service import LiDARIngestService
from adaptx.services.mapping_service import MappingService
from adaptx.services.metrics_service import MetricsService
from adaptx.services.prediction_service import PredictionService
from adaptx.services.risk_service import RiskService
from adaptx.services.system_service import SystemService
from adaptx.services.tracking_service import TrackingService

logger = get_logger(__name__)


@dataclass(slots=True)
class ApplicationContext:
    """Container for the services that live as long as the process."""

    settings: Settings
    metrics: MetricsService
    lidar: LiDARIngestService
    carla: CarlaService
    system: SystemService
    risk_engine: BaselineProximityRiskEngine
    preprocessor: LiDARProcessingPipeline
    detector: GeometricObjectDetector
    tracking: TrackingService
    prediction: PredictionService
    mapping: MappingService
    risk: RiskService
    adaptive_mapping: AdaptiveMappingService


def build_context(settings: Settings | None = None) -> ApplicationContext:
    """Construct the service graph without starting anything."""
    resolved = settings if settings is not None else get_settings()

    metrics = MetricsService(window=resolved.lidar.metrics_window)
    lidar = LiDARIngestService(
        settings=resolved.lidar,
        processor=FrameValidationProcessor(resolved.lidar),
        metrics=metrics,
    )
    carla = CarlaService(resolved.carla)
    system = SystemService(settings=resolved, lidar=lidar, carla=carla)
    return ApplicationContext(
        settings=resolved,
        metrics=metrics,
        lidar=lidar,
        carla=carla,
        system=system,
        risk_engine=BaselineProximityRiskEngine(resolved.risk),
        preprocessor=LiDARProcessingPipeline(resolved.lidar),
        detector=GeometricObjectDetector(resolved.detection),
        tracking=TrackingService(resolved.tracking),
        prediction=PredictionService(resolved.prediction),
        mapping=MappingService(resolved.map),
        risk=RiskService(resolved.risk),
        adaptive_mapping=AdaptiveMappingService(resolved.map, resolved.adaptive),
    )


def startup(context: ApplicationContext) -> None:
    """Run startup work: configure logging and attempt an optional CARLA connect."""
    configure_logging(
        context.settings.logging.level,
        json_format=context.settings.logging.json_format,
    )
    logger.info(
        "ADAPT-X starting",
        extra={
            "context": {
                "environment": context.settings.app.environment.value,
                "debug": context.settings.app.debug,
            }
        },
    )
    if context.settings.carla.enabled:
        context.carla.connect()
    else:
        logger.info("CARLA disabled; reporting DISCONNECTED")


def shutdown(context: ApplicationContext) -> None:
    """Release resources held by the context.

    Tracking state is dropped explicitly: it is the accumulated perception
    state in the process, and leaving it behind would let a restarted context
    inherit tracks from frames it never saw.

    Adaptive mapping is dropped for the same reason as tracking: its resolution
    controller remembers what level each region held and how long it has been
    quiet, and a restarted context must not inherit the stabilisation history
    of a scene it never saw (ADR-039).

    Prediction, mapping and risk are reset too, though none carries perception
    state - only the counters behind their status summaries, which would
    otherwise describe frames a restarted context never processed.
    """
    context.tracking.reset()
    context.prediction.reset()
    context.mapping.reset()
    context.risk.reset()
    context.adaptive_mapping.reset()
    context.carla.disconnect()
    logger.info("ADAPT-X stopped")
