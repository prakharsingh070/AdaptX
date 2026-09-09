"""FastAPI dependencies.

Services are resolved from the application context stored on ``app.state``, so
tests can build an app around an isolated context instead of patching globals.
"""

from __future__ import annotations

from typing import Annotated

from fastapi import Depends, Request

from adaptx.config.settings import Settings
from adaptx.core.lifecycle import ApplicationContext
from adaptx.services.carla_service import CarlaService
from adaptx.services.lidar_service import LiDARIngestService
from adaptx.services.metrics_service import MetricsService
from adaptx.services.system_service import SystemService


def get_context(request: Request) -> ApplicationContext:
    """Return the application context attached at startup."""
    context: ApplicationContext = request.app.state.context
    return context


def get_settings_dep(request: Request) -> Settings:
    """Return the active settings."""
    return get_context(request).settings


def get_system_service(request: Request) -> SystemService:
    """Return the system status service."""
    return get_context(request).system


def get_metrics_service(request: Request) -> MetricsService:
    """Return the metrics service."""
    return get_context(request).metrics


def get_lidar_service(request: Request) -> LiDARIngestService:
    """Return the LiDAR ingest service."""
    return get_context(request).lidar


def get_carla_service(request: Request) -> CarlaService:
    """Return the CARLA service."""
    return get_context(request).carla


ContextDep = Annotated[ApplicationContext, Depends(get_context)]
SettingsDep = Annotated[Settings, Depends(get_settings_dep)]
SystemServiceDep = Annotated[SystemService, Depends(get_system_service)]
MetricsServiceDep = Annotated[MetricsService, Depends(get_metrics_service)]
LiDARServiceDep = Annotated[LiDARIngestService, Depends(get_lidar_service)]
CarlaServiceDep = Annotated[CarlaService, Depends(get_carla_service)]
