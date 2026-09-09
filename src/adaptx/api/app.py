"""FastAPI application factory."""

from __future__ import annotations

from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

from fastapi import APIRouter, FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from adaptx import __version__
from adaptx.api.routes import carla, health, lidar, risk, system
from adaptx.api.routes import map as map_routes
from adaptx.api.websocket import telemetry
from adaptx.api.websocket.manager import ConnectionManager
from adaptx.config.settings import Settings, get_settings
from adaptx.core.exceptions import AdaptXError
from adaptx.core.lifecycle import ApplicationContext, build_context, shutdown, startup
from adaptx.core.logging import get_logger

logger = get_logger(__name__)

API_V1_PREFIX = "/api/v1"

_DESCRIPTION = """
ADAPT-X backend - Adaptive Dynamic Perception and Tracking.

**Implementation status:** this API is the Phase 1 engineering foundation.
It exposes configuration, health, measured runtime metrics, the CARLA
connection boundary and LiDAR frame ingestion with structural validation.

Object detection, tracking, trajectory prediction, 2.5D mapping and the
adaptive resolution algorithm are **not implemented**. Every subsystem reports
its own implementation status through `/api/v1/system/status`.
""".strip()


def _build_lifespan(context: ApplicationContext):  # type: ignore[no-untyped-def]
    @asynccontextmanager
    async def lifespan(app: FastAPI) -> AsyncIterator[None]:
        startup(context)
        app.state.context = context
        app.state.connections = ConnectionManager(
            max_connections=context.settings.websocket.max_connections
        )
        yield
        shutdown(context)

    return lifespan


def create_app(
    settings: Settings | None = None, context: ApplicationContext | None = None
) -> FastAPI:
    """Build the ADAPT-X FastAPI application.

    Args:
        settings: Overrides the process settings; useful in tests.
        context: A pre-built service graph. When omitted, one is constructed
            from ``settings``.
    """
    resolved_settings = settings if settings is not None else get_settings()
    resolved_context = context if context is not None else build_context(resolved_settings)

    app = FastAPI(
        title=resolved_settings.app.name,
        summary=resolved_settings.app.subtitle,
        description=_DESCRIPTION,
        version=__version__,
        root_path=resolved_settings.api.root_path,
        lifespan=_build_lifespan(resolved_context),
    )

    # The context is also attached here so an app used without a lifespan
    # (for example when only inspecting routes) still resolves dependencies.
    app.state.context = resolved_context
    app.state.connections = ConnectionManager(
        max_connections=resolved_settings.websocket.max_connections
    )

    if resolved_settings.api.cors_origins:
        app.add_middleware(
            CORSMiddleware,
            allow_origins=resolved_settings.api.cors_origins,
            allow_credentials=False,
            allow_methods=["GET", "POST", "OPTIONS"],
            allow_headers=["*"],
        )

    @app.exception_handler(AdaptXError)
    async def adaptx_error_handler(_: Request, exc: AdaptXError) -> JSONResponse:
        """Translate a domain error into an explicit response, never a silent 500."""
        logger.warning(
            "request failed",
            extra={"context": {"code": exc.code, "message": exc.message}},
        )
        return JSONResponse(status_code=exc.http_status, content={"error": exc.to_dict()})

    app.include_router(health.router)

    api = APIRouter(prefix=API_V1_PREFIX)
    api.include_router(system.router)
    api.include_router(carla.router)
    api.include_router(lidar.router)
    api.include_router(map_routes.router)
    api.include_router(risk.router)
    app.include_router(api)

    app.include_router(telemetry.router)

    return app
