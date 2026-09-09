"""Typed, environment-driven application configuration.

Settings are loaded from (in increasing priority): defaults declared here, a
``.env`` file, then process environment variables. Every variable is prefixed
with ``ADAPTX_`` and nested sections use a ``__`` delimiter, e.g.
``ADAPTX_API__PORT=8000``.

No secrets are declared or defaulted in this module.
"""

from __future__ import annotations

from enum import StrEnum
from functools import lru_cache
from typing import Literal

from pydantic import BaseModel, Field, field_validator, model_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


class Environment(StrEnum):
    """Deployment environment."""

    DEVELOPMENT = "development"
    STAGING = "staging"
    PRODUCTION = "production"


class AppSettings(BaseModel):
    """Application identity and mode."""

    name: str = "ADAPT-X"
    subtitle: str = "Adaptive Dynamic Perception and Tracking"
    environment: Environment = Environment.DEVELOPMENT
    debug: bool = True


class APISettings(BaseModel):
    """HTTP server settings."""

    # Binds all interfaces by default for container/dev use; restrict in production.
    host: str = "0.0.0.0"
    port: int = Field(default=8000, ge=1, le=65535)
    reload: bool = True
    root_path: str = ""
    cors_origins: list[str] = Field(
        default_factory=lambda: ["http://localhost:3000", "http://localhost:5173"]
    )


class LoggingSettings(BaseModel):
    """Structured logging settings."""

    level: Literal["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"] = "INFO"
    json_format: bool = False

    @field_validator("level", mode="before")
    @classmethod
    def _upper(cls, value: object) -> object:
        return value.upper() if isinstance(value, str) else value


class CarlaSettings(BaseModel):
    """CARLA simulator connection settings.

    CARLA is optional. When ``enabled`` is false the backend never attempts a
    connection and reports ``DISCONNECTED``.
    """

    enabled: bool = False
    host: str = "localhost"
    port: int = Field(default=2000, ge=1, le=65535)
    timeout_s: float = Field(default=10.0, gt=0)
    # Development-only in-process fake simulator. Data produced through it is
    # labelled SYNTHETIC_TEST and must never be presented as sensor output.
    use_mock: bool = False


class LiDARSettings(BaseModel):
    """Constraints applied to incoming point-cloud frames."""

    min_points: int = Field(default=1, ge=0)
    max_points: int = Field(default=500_000, gt=0)
    # A frame older than this makes the LiDAR source report DISCONNECTED.
    frame_stale_after_s: float = Field(default=2.0, gt=0)
    # Size of the rolling window used to measure ingest FPS and latency.
    metrics_window: int = Field(default=30, ge=2, le=1000)

    @model_validator(mode="after")
    def _check_bounds(self) -> LiDARSettings:
        if self.min_points > self.max_points:
            raise ValueError("lidar.min_points must be <= lidar.max_points")
        return self


class MapSettings(BaseModel):
    """2.5D map geometry and the cell size bound to each resolution level.

    The adaptive resolution algorithm itself is not implemented in Phase 1;
    these values define the configured meaning of each level.
    """

    range_m: float = Field(default=60.0, gt=0)
    resolution_low_m: float = Field(default=1.0, gt=0)
    resolution_medium_m: float = Field(default=0.5, gt=0)
    resolution_high_m: float = Field(default=0.2, gt=0)
    resolution_critical_m: float = Field(default=0.1, gt=0)

    @model_validator(mode="after")
    def _check_monotonic(self) -> MapSettings:
        sizes = [
            self.resolution_low_m,
            self.resolution_medium_m,
            self.resolution_high_m,
            self.resolution_critical_m,
        ]
        if sizes != sorted(sizes, reverse=True):
            raise ValueError(
                "map resolution cell sizes must decrease from LOW to CRITICAL (coarse -> fine)"
            )
        return self


class RiskSettings(BaseModel):
    """Risk normalisation bounds and level thresholds.

    Risk is normalised to ``[0, 1]``. Thresholds partition that range into the
    LOW / MEDIUM / HIGH / CRITICAL levels.
    """

    max_range_m: float = Field(default=60.0, gt=0)
    threshold_medium: float = Field(default=0.35, ge=0.0, le=1.0)
    threshold_high: float = Field(default=0.60, ge=0.0, le=1.0)
    threshold_critical: float = Field(default=0.85, ge=0.0, le=1.0)

    @model_validator(mode="after")
    def _check_thresholds(self) -> RiskSettings:
        if not (self.threshold_medium < self.threshold_high < self.threshold_critical):
            raise ValueError("risk thresholds must satisfy medium < high < critical")
        return self


class WebSocketSettings(BaseModel):
    """Real-time telemetry channel settings."""

    telemetry_interval_s: float = Field(default=1.0, gt=0)
    max_connections: int = Field(default=32, ge=1)


class Settings(BaseSettings):
    """Root settings object."""

    model_config = SettingsConfigDict(
        env_prefix="ADAPTX_",
        env_nested_delimiter="__",
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
    )

    app: AppSettings = Field(default_factory=AppSettings)
    api: APISettings = Field(default_factory=APISettings)
    logging: LoggingSettings = Field(default_factory=LoggingSettings)
    carla: CarlaSettings = Field(default_factory=CarlaSettings)
    lidar: LiDARSettings = Field(default_factory=LiDARSettings)
    map: MapSettings = Field(default_factory=MapSettings)
    risk: RiskSettings = Field(default_factory=RiskSettings)
    websocket: WebSocketSettings = Field(default_factory=WebSocketSettings)

    @property
    def is_production(self) -> bool:
        """True when running in the production environment."""
        return self.app.environment is Environment.PRODUCTION


@lru_cache(maxsize=1)
def get_settings() -> Settings:
    """Return the process-wide settings singleton."""
    return Settings()


def reload_settings() -> Settings:
    """Clear the settings cache and re-read the environment.

    Intended for tests and for reacting to a configuration change; production
    code should depend on :func:`get_settings`.
    """
    get_settings.cache_clear()
    return get_settings()
