"""Typed, environment-driven application configuration.

Settings are loaded from (in increasing priority): defaults declared here, a
``.env`` file, then process environment variables. Every variable is prefixed
with ``ADAPTX_`` and nested sections use a ``__`` delimiter, e.g.
``ADAPTX_API__PORT=8000``.

No secrets are declared or defaulted in this module.
"""

from __future__ import annotations

import math
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
    """Constraints applied to incoming point-cloud frames, and the Phase 2A
    preprocessing bounds.

    Coordinate convention for every ROI bound below (ADR-009): a right-handed
    frame with **+x forward, +y left, +z up**, origin at the sensor, in metres.

    The default ROI and range values are plausible starting points for an
    automotive roof-mounted scanner, not measured or tuned values. They are
    configuration, not results.
    """

    min_points: int = Field(default=1, ge=0)
    max_points: int = Field(default=500_000, gt=0)
    # A frame older than this makes the LiDAR source report DISCONNECTED.
    frame_stale_after_s: float = Field(default=2.0, gt=0)
    # Size of the rolling window used to measure ingest FPS and latency.
    metrics_window: int = Field(default=30, ge=2, le=1000)

    # --- Range filtering (Euclidean distance from the sensor origin) --------
    min_range_m: float = Field(
        default=0.5,
        ge=0.0,
        description="Points closer than this are dropped (sensor blind zone, ego returns).",
    )
    max_range_m: float = Field(default=100.0, gt=0.0, description="Points beyond this are dropped.")

    # --- Region of interest (axis-aligned box, inclusive bounds) ------------
    roi_x_min_m: float = Field(default=-50.0, description="Behind the sensor is negative x.")
    roi_x_max_m: float = Field(default=80.0, description="Ahead of the sensor is positive x.")
    roi_y_min_m: float = Field(default=-40.0, description="Right of the sensor is negative y.")
    roi_y_max_m: float = Field(default=40.0, description="Left of the sensor is positive y.")
    roi_z_min_m: float = Field(default=-3.0, description="Below the sensor is negative z.")
    roi_z_max_m: float = Field(default=5.0, description="Above the sensor is positive z.")

    # --- Phase 2B stages ----------------------------------------------------
    # All three are opt-in (ADR-012). Voxelisation is lossy and the ground and
    # noise stages are baselines with documented failure modes, so enabling
    # them silently would change what every downstream consumer sees.
    voxel_enabled: bool = Field(
        default=False, description="Enable voxel downsampling after range filtering."
    )
    voxel_size_m: float = Field(
        default=0.10, gt=0.0, description="Cube edge length of one voxel, in metres."
    )

    ground_enabled: bool = Field(default=False, description="Enable baseline ground segmentation.")
    ground_cell_size_m: float = Field(
        default=1.0,
        gt=0.0,
        description="Edge length of the square xy cell used to find the local ground level.",
    )
    ground_height_tolerance_m: float = Field(
        default=0.20,
        ge=0.0,
        description="A point within this height of its cell's lowest point counts as ground.",
    )
    ground_max_height_m: float | None = Field(
        default=None,
        description=(
            "Optional ceiling on where ground may be, in sensor-frame z. A cell whose "
            "lowest point sits above it is treated as containing no ground. Disabled by "
            "default because it assumes a known, level sensor mount."
        ),
    )

    noise_enabled: bool = Field(default=False, description="Enable baseline noise filtering.")
    noise_cell_size_m: float = Field(
        default=0.5,
        gt=0.0,
        description="Edge length of the cell used to count a point's neighbours.",
    )
    noise_min_neighbors: int = Field(
        default=4,
        ge=0,
        description=(
            "A point with fewer neighbours than this in the 3x3x3 block of cells around "
            "it is dropped as an outlier."
        ),
    )

    @model_validator(mode="after")
    def _check_bounds(self) -> LiDARSettings:
        if self.min_points > self.max_points:
            raise ValueError("lidar.min_points must be <= lidar.max_points")
        if self.min_range_m >= self.max_range_m:
            raise ValueError("lidar.min_range_m must be < lidar.max_range_m")
        for axis in ("x", "y", "z"):
            low = getattr(self, f"roi_{axis}_min_m")
            high = getattr(self, f"roi_{axis}_max_m")
            if low >= high:
                raise ValueError(f"lidar.roi_{axis}_min_m must be < lidar.roi_{axis}_max_m")
        return self


class DetectionSettings(BaseModel):
    """Geometric object detection (Phase 3).

    Coordinate convention as everywhere else (ADR-009): +x forward, +y left,
    +z up, metres, origin at the sensor.

    These are **baseline** parameters for a clustering detector, chosen as
    plausible starting points for an automotive scene. None has been tuned or
    validated against labelled data, because no labelled data exists.
    """

    cluster_tolerance_m: float = Field(
        default=0.5,
        gt=0.0,
        description=(
            "Grid cell size for clustering. Points in connected occupied cells "
            "join the same cluster, so this is the effective separation distance."
        ),
    )
    min_cluster_points: int = Field(
        default=10, ge=1, description="Clusters with fewer points are rejected as noise."
    )
    max_cluster_points: int = Field(
        default=50_000,
        ge=1,
        description="Clusters larger than this are rejected as structure, not objects.",
    )

    min_height_m: float = Field(
        default=0.15, ge=0.0, description="Clusters flatter than this are rejected."
    )
    max_height_m: float = Field(
        default=4.5, gt=0.0, description="Clusters taller than this are rejected."
    )
    min_footprint_m: float = Field(
        default=0.10,
        ge=0.0,
        description="Largest horizontal extent must reach this to be an object.",
    )
    max_footprint_m: float = Field(
        default=15.0,
        gt=0.0,
        description="Largest horizontal extent above this is a wall or building, not an object.",
    )

    @model_validator(mode="after")
    def _check_bounds(self) -> DetectionSettings:
        if self.min_cluster_points > self.max_cluster_points:
            raise ValueError("detection.min_cluster_points must be <= max_cluster_points")
        if self.min_height_m >= self.max_height_m:
            raise ValueError("detection.min_height_m must be < max_height_m")
        if self.min_footprint_m >= self.max_footprint_m:
            raise ValueError("detection.min_footprint_m must be < max_footprint_m")
        return self


class TrackingSettings(BaseModel):
    """Temporal object tracking (Phase 4).

    Coordinate convention as everywhere else (ADR-009): +x forward, +y left,
    +z up, metres, origin at the sensor.

    These are **baseline** parameters for a geometric nearest-neighbour tracker.
    None has been tuned or validated against labelled sequences, because no
    labelled sequences exist.
    """

    max_association_distance_m: float = Field(
        default=2.5,
        gt=0.0,
        description=(
            "Gate: a detection further than this from a track's expected "
            "position can never be matched to it."
        ),
    )
    require_class_match: bool = Field(
        default=False,
        description=(
            "When true, a known track class and a different known detection "
            "class cannot match. UNKNOWN matches anything either way."
        ),
    )
    max_size_ratio: float | None = Field(
        default=3.0,
        gt=1.0,
        description=(
            "Gate on the ratio of largest dimensions. None disables the check. "
            "Stops a pedestrian-sized cluster inheriting a lorry's track."
        ),
    )

    min_hits_to_confirm: int = Field(
        default=3,
        ge=1,
        description="Associated detections a tentative track needs before CONFIRMED.",
    )
    max_missed_frames: int = Field(
        default=3,
        ge=0,
        description="Consecutive misses a confirmed track survives before it is dropped.",
    )
    max_missed_frames_tentative: int = Field(
        default=1,
        ge=0,
        description=(
            "Misses an unconfirmed track survives. Lower than the confirmed "
            "limit so a spurious detection does not linger as a ghost track."
        ),
    )
    class_switch_hits: int = Field(
        default=2,
        ge=1,
        description=(
            "Consecutive consistent observations of a different class before a "
            "known track class changes. Stops class flicker frame to frame."
        ),
    )

    velocity_smoothing: float = Field(
        default=0.5,
        ge=0.0,
        le=1.0,
        description=(
            "Exponential moving average weight on the newest observation. "
            "1.0 uses the raw frame-to-frame velocity unsmoothed; 0.0 freezes "
            "the first estimate. The raw value is always reported alongside."
        ),
    )
    max_timestep_s: float = Field(
        default=2.0,
        gt=0.0,
        description=(
            "Frame gaps longer than this make velocity meaningless, so it is "
            "reported unknown rather than computed across the gap."
        ),
    )
    min_speed_for_heading_mps: float = Field(
        default=0.3,
        ge=0.0,
        description=(
            "Below this speed a heading is direction of noise, not of travel, "
            "so it is reported as unknown."
        ),
    )
    use_predicted_position_for_association: bool = Field(
        default=True,
        description=(
            "Gate against where a moving track is expected to be rather than "
            "where it last was. Used for association only - the extrapolation "
            "is never published as an observation or a trajectory."
        ),
    )

    @model_validator(mode="after")
    def _check_bounds(self) -> TrackingSettings:
        if self.max_missed_frames_tentative > self.max_missed_frames:
            raise ValueError("tracking.max_missed_frames_tentative must be <= max_missed_frames")
        return self


class PredictionSettings(BaseModel):
    """Trajectory prediction (Phase 5).

    Coordinate convention as everywhere else (ADR-009): +x forward, +y left,
    +z up, metres, origin at the sensor.

    These are **engineering defaults** for a deterministic constant-velocity
    baseline, not measured real-world limits. None has been tuned or validated
    against labelled trajectories, because no labelled trajectories exist.
    """

    horizon_s: float = Field(
        default=3.0,
        gt=0.0,
        description="How far ahead a trajectory extends, in seconds.",
    )
    interval_s: float = Field(
        default=0.25,
        gt=0.0,
        description=(
            "Spacing between trajectory points. Points run from t+0 to the "
            "horizon inclusive, so the count is horizon/interval + 1."
        ),
    )
    max_tracks: int = Field(
        default=256,
        ge=1,
        description=(
            "Upper bound on tracks predicted in one call. Tracks beyond it are "
            "recorded as skipped rather than silently dropped."
        ),
    )
    max_speed_mps: float = Field(
        default=80.0,
        gt=0.0,
        description=(
            "Sanity bound on measured track speed. A faster track is reported "
            "as invalid and skipped; the velocity is never clipped, because a "
            "clipped value would be a number no sensor produced."
        ),
    )

    base_uncertainty_m: float = Field(
        default=0.5,
        gt=0.0,
        description=(
            "Heuristic uncertainty radius at t+0, standing in for detection "
            "and tracking positional error. Not a calibrated sigma. Strictly "
            "positive: a zero floor would claim a perfectly known position."
        ),
    )
    uncertainty_growth_mps: float = Field(
        default=0.5,
        ge=0.0,
        description=(
            "Metres of additional heuristic uncertainty per second of "
            "extrapolation. Not a calibrated growth rate."
        ),
    )
    confidence_hits_full: int = Field(
        default=3,
        ge=1,
        description=(
            "Associated detections at which a track contributes full evidence "
            "to the trajectory confidence score. Fewer hits scale it down."
        ),
    )

    @model_validator(mode="after")
    def _check_bounds(self) -> PredictionSettings:
        if self.interval_s > self.horizon_s:
            raise ValueError("prediction.interval_s must be <= prediction.horizon_s")
        return self


class MapSettings(BaseModel):
    """2.5D map geometry, resolution vocabulary and Phase 6 mapping bounds.

    The ``resolution_*_m`` values define what each *level* would mean to a
    resolution controller. No controller is implemented, so they are a
    vocabulary rather than a decision.

    ``resolution_m`` and the ``*_x_m`` / ``*_y_m`` bounds are what the Phase 6
    fixed-resolution mapper actually uses. Coordinate convention as everywhere
    else (ADR-009): +x forward, +y left, +z up, metres, origin at the sensor.

    These are **baseline** values chosen as plausible starting points for an
    automotive scene. None has been tuned or validated against labelled data,
    because no labelled data exists.
    """

    range_m: float = Field(
        default=60.0,
        gt=0,
        description=(
            "Advertised mapping range, reported by the status endpoint. The "
            "extent actually mapped is set by the bounds below."
        ),
    )
    resolution_low_m: float = Field(default=1.0, gt=0)
    resolution_medium_m: float = Field(default=0.5, gt=0)
    resolution_high_m: float = Field(default=0.2, gt=0)
    resolution_critical_m: float = Field(default=0.1, gt=0)

    # -- Phase 6 fixed-resolution mapping ---------------------------------
    resolution_m: float = Field(
        default=0.5,
        gt=0.0,
        description=(
            "Cell edge length applied uniformly by the fixed-resolution "
            "mapper. One value everywhere; adaptive allocation is not "
            "implemented (ADR-029)."
        ),
    )
    min_resolution_m: float = Field(
        default=0.05,
        gt=0.0,
        description="Finest cell size a mapping request may ask for.",
    )
    max_resolution_m: float = Field(
        default=5.0,
        gt=0.0,
        description="Coarsest cell size a mapping request may ask for.",
    )

    min_x_m: float = Field(default=-60.0, description="Behind the sensor is negative x.")
    max_x_m: float = Field(default=60.0, description="Ahead of the sensor is positive x.")
    min_y_m: float = Field(default=-60.0, description="Right of the sensor is negative y.")
    max_y_m: float = Field(default=60.0, description="Left of the sensor is positive y.")

    max_cells: int = Field(
        default=4_000_000,
        ge=1,
        description=(
            "Hard ceiling on cells in one grid, checked before any array is "
            "allocated. Stops a fine resolution over wide bounds from "
            "requesting an absurd amount of memory (ADR-028)."
        ),
    )

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

    @model_validator(mode="after")
    def _check_mapping_bounds(self) -> MapSettings:
        if self.min_x_m >= self.max_x_m:
            raise ValueError("map.min_x_m must be < map.max_x_m")
        if self.min_y_m >= self.max_y_m:
            raise ValueError("map.min_y_m must be < map.max_y_m")
        if self.min_resolution_m > self.max_resolution_m:
            raise ValueError("map.min_resolution_m must be <= map.max_resolution_m")
        if not (self.min_resolution_m <= self.resolution_m <= self.max_resolution_m):
            raise ValueError(
                f"map.resolution_m ({self.resolution_m}) must lie within "
                f"[{self.min_resolution_m}, {self.max_resolution_m}]"
            )
        width = math.ceil((self.max_x_m - self.min_x_m) / self.resolution_m)
        height = math.ceil((self.max_y_m - self.min_y_m) / self.resolution_m)
        if width * height > self.max_cells:
            raise ValueError(
                f"map bounds at resolution_m={self.resolution_m} would need "
                f"{width * height} cells, above map.max_cells ({self.max_cells})"
            )
        return self


class RiskSettings(BaseModel):
    """Risk normalisation bounds, level thresholds and Phase 7 heuristics.

    Risk is normalised to ``[0, 1]``. Thresholds partition that range into the
    LOW / MEDIUM / HIGH / CRITICAL levels. ``UNKNOWN`` has no threshold: it
    means nothing was scored, not that a score fell in a band.

    Every value here is a **baseline engineering value**, chosen as a plausible
    starting point for an automotive scene. None has been tuned or validated
    against labelled risk data, because no labelled risk data exists, and none
    is a safety-certified limit.

    Coordinate convention as everywhere else (ADR-009): +x forward, +y left,
    +z up, metres, ego reference at the origin.
    """

    max_range_m: float = Field(default=60.0, gt=0)
    threshold_medium: float = Field(default=0.35, ge=0.0, le=1.0)
    threshold_high: float = Field(default=0.60, ge=0.0, le=1.0)
    threshold_critical: float = Field(default=0.85, ge=0.0, le=1.0)

    # -- Phase 7 heuristic factors ----------------------------------------
    proximity_near_m: float = Field(
        default=5.0,
        gt=0.0,
        description=(
            "At or inside this planar distance the proximity factor is 1.0. "
            "A baseline engineering value, not a validated safety envelope."
        ),
    )
    proximity_far_m: float = Field(
        default=40.0,
        gt=0.0,
        description="At or beyond this distance the proximity factor is 0.0.",
    )
    closing_speed_high_mps: float = Field(
        default=15.0,
        gt=0.0,
        description=(
            "Rate of approach at which the closing-speed factor saturates at "
            "1.0. Objects receding contribute 0.0."
        ),
    )
    stale_observation_s: float = Field(
        default=0.5,
        gt=0.0,
        description=(
            "An observation older than this is treated as stale: it still "
            "informs the assessment but raises uncertainty."
        ),
    )
    low_track_confidence: float = Field(
        default=0.5,
        ge=0.0,
        le=1.0,
        description=(
            "Track confidence at or below this raises uncertainty. The value "
            "is Phase 3's geometric fit score carried through, not a "
            "probability (ADR-021)."
        ),
    )

    # -- Factor weights ----------------------------------------------------
    # Combined as a weighted mean over the factors that are actually
    # available; a factor that could not be computed is dropped and the
    # remaining weights renormalise, never treated as zero (ADR-032).
    weight_proximity: float = Field(
        default=0.5, ge=0.0, description="Weight of the proximity factor."
    )
    weight_closing_speed: float = Field(
        default=0.25, ge=0.0, description="Weight of the closing-speed factor."
    )
    weight_predicted_proximity: float = Field(
        default=0.25, ge=0.0, description="Weight of the predicted-proximity factor."
    )

    #: Objects predicted to pass within this distance count as approaching the
    #: ego region. Not a collision test - the trajectory is a constant-velocity
    #: extrapolation with heuristic uncertainty (ADR-026).
    predicted_proximity_near_m: float = Field(default=5.0, gt=0.0)

    max_assessed_tracks: int = Field(
        default=512,
        ge=1,
        description="Upper bound on tracks assessed in one call, to bound response size.",
    )

    @model_validator(mode="after")
    def _check_thresholds(self) -> RiskSettings:
        if not (self.threshold_medium < self.threshold_high < self.threshold_critical):
            raise ValueError("risk thresholds must satisfy medium < high < critical")
        return self

    @model_validator(mode="after")
    def _check_heuristics(self) -> RiskSettings:
        if self.proximity_near_m >= self.proximity_far_m:
            raise ValueError("risk.proximity_near_m must be < risk.proximity_far_m")
        total = self.weight_proximity + self.weight_closing_speed + self.weight_predicted_proximity
        if total <= 0.0:
            raise ValueError("risk factor weights must not all be zero")
        return self

    @property
    def total_weight(self) -> float:
        """Sum of the factor weights, used to renormalise available factors."""
        return self.weight_proximity + self.weight_closing_speed + self.weight_predicted_proximity


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
    detection: DetectionSettings = Field(default_factory=DetectionSettings)
    tracking: TrackingSettings = Field(default_factory=TrackingSettings)
    prediction: PredictionSettings = Field(default_factory=PredictionSettings)
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
