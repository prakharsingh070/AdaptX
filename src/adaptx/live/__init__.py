"""Live CARLA simulation session behind the dashboard (post-Phase-12 extension).

Contracts and the scenario catalogue import cleanly; the service that owns
the loop is imported explicitly from :mod:`adaptx.live.service` because it
depends on the application context.
"""

from adaptx.live.models import (
    ActiveActor,
    ActorKind,
    LiveEvent,
    LiveFrameInfo,
    LiveScenarioDefinition,
    LiveScenarioSummary,
    LiveState,
    LiveStatus,
    LiveTiming,
    SpawnEvent,
)

__all__ = [
    "ActiveActor",
    "ActorKind",
    "LiveEvent",
    "LiveFrameInfo",
    "LiveScenarioDefinition",
    "LiveScenarioSummary",
    "LiveState",
    "LiveStatus",
    "LiveTiming",
    "SpawnEvent",
]
