"""Data contracts local to the CARLA boundary.

These describe the simulator, not the perception domain, so they live beside
the client rather than in :mod:`adaptx.models`.
"""

from __future__ import annotations

from pydantic import Field

from adaptx.models.common import AdaptXModel, Vector3


class CarlaWorldInfo(AdaptXModel):
    """Description of the loaded CARLA world."""

    map_name: str
    synchronous_mode: bool = False
    fixed_delta_seconds: float | None = None
    actor_count: int = Field(default=0, ge=0)
    weather_preset: str | None = None


class CarlaActorRef(AdaptXModel):
    """Handle to an actor spawned in the simulator."""

    actor_id: int = Field(ge=0)
    blueprint_id: str = Field(min_length=1)
    role: str = Field(default="", description="e.g. 'ego', 'traffic', 'pedestrian'.")
    location: Vector3 = Field(default_factory=Vector3)
