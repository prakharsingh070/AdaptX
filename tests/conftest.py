"""Shared pytest fixtures.

Every fixture builds an isolated :class:`ApplicationContext` from explicit
settings, so tests never depend on the developer's ``.env`` or on process
environment variables.
"""

from __future__ import annotations

from collections.abc import Iterator

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from adaptx.api.app import create_app
from adaptx.config.settings import Settings
from adaptx.core.lifecycle import ApplicationContext, build_context
from tests.fixtures.point_clouds import make_frame, make_points

__all__ = ["make_frame", "make_points"]


@pytest.fixture
def settings() -> Settings:
    """Deterministic settings for tests: CARLA off, small windows, fast telemetry."""
    return Settings(
        app={"environment": "development", "debug": True},
        api={"cors_origins": []},
        logging={"level": "WARNING"},
        carla={"enabled": False, "use_mock": False},
        lidar={"min_points": 1, "max_points": 1000, "frame_stale_after_s": 60.0},
        websocket={"telemetry_interval_s": 0.05, "max_connections": 4},
    )


@pytest.fixture
def context(settings: Settings) -> ApplicationContext:
    """An application context wired from the test settings."""
    return build_context(settings)


@pytest.fixture
def app(settings: Settings, context: ApplicationContext) -> FastAPI:
    """A FastAPI application bound to the test context."""
    return create_app(settings=settings, context=context)


@pytest.fixture
def client(app: FastAPI) -> Iterator[TestClient]:
    """A TestClient that runs the real startup and shutdown lifespan."""
    with TestClient(app) as test_client:
        yield test_client
