"""Live scenario catalogue and the manager that runs one against a session.

A live scenario is a running scene, not a recording: background traffic
that drives itself under CARLA's Traffic Manager, plus timed appearances of
scripted actors placed relative to where the ego is *at that moment* and
moved in that anchor frame (the Phase 10 placement model, ADR-047/054,
anchored so a driving ego does not drag them along).

Randomness is confined to :func:`resolve_live`: one ``random.Random(seed)``
draws placement jitter, traffic blueprints and traffic spawn points, once,
before the simulator is touched. The same seed gives the same draws; the
Traffic Manager is seeded from the same value (in the session), so traffic
is repeatable to the extent CARLA's own scheduler is.

Nothing in this module reads a pipeline output or ground truth. The manager
spawns, moves and removes actors on the session's clock and that is all.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Any

from adaptx.carla.session import CarlaSimulationSession, WorldAnchor
from adaptx.core.exceptions import AdaptXError, SimulatorUnavailableError
from adaptx.core.logging import get_logger
from adaptx.live.models import (
    ActiveActor,
    ActorKind,
    LiveMotion,
    LiveScenarioDefinition,
    LiveScenarioSummary,
    SpawnEvent,
)

logger = get_logger(__name__)

VEHICLE_BLUEPRINTS = [
    "vehicle.audi.tt",
    "vehicle.tesla.model3",
    "vehicle.lincoln.mkz_2020",
    "vehicle.nissan.patrol",
    "vehicle.mini.cooper_s",
]
PEDESTRIAN = "walker.pedestrian.0001"
CYCLIST = "vehicle.diamondback.century"
PARKED = "vehicle.audi.tt"


class LiveScenarioError(AdaptXError):
    """A live scenario could not be found or is not valid."""

    code = "live_scenario_error"
    http_status = 404


def _catalogue() -> dict[str, LiveScenarioDefinition]:
    obstacle = SpawnEvent(
        actor_id="parked_car",
        kind=ActorKind.OBSTACLE,
        blueprint=PARKED,
        at_s=1.0,
        forward_m=45.0,
        jitter_m=1.0,
        lifetime_s=22.0,
    )
    pedestrian = SpawnEvent(
        actor_id="pedestrian",
        kind=ActorKind.PEDESTRIAN,
        blueprint=PEDESTRIAN,
        at_s=2.0,
        forward_m=32.0,
        # Measured on Town10HD_Opt from spawn point 1: +7 m left at 32 m ahead is
        # inside street furniture and the spawn is refused; +5 m is the pavement.
        left_m=5.0,
        yaw_offset_deg=-90.0,
        jitter_m=0.3,
        motion=[LiveMotion(start_s=0.0, stop_s=10.0, left_mps=-1.2)],
        lifetime_s=16.0,
    )
    cyclist = SpawnEvent(
        actor_id="cyclist",
        kind=ActorKind.CYCLIST,
        blueprint=CYCLIST,
        at_s=2.0,
        forward_m=36.0,
        left_m=-9.0,
        yaw_offset_deg=90.0,
        jitter_m=0.5,
        motion=[LiveMotion(start_s=0.0, stop_s=6.0, left_mps=3.0)],
        lifetime_s=14.0,
    )
    cut_in = SpawnEvent(
        actor_id="cutting_in",
        kind=ActorKind.VEHICLE,
        blueprint="vehicle.lincoln.mkz_2020",
        at_s=1.0,
        forward_m=22.0,
        left_m=3.5,
        jitter_m=0.5,
        motion=[
            LiveMotion(start_s=0.0, stop_s=5.0, forward_mps=4.0, left_mps=-0.7),
            LiveMotion(start_s=5.0, stop_s=40.0, forward_mps=3.0),
        ],
        lifetime_s=40.0,
    )
    roadside = SpawnEvent(
        actor_id="roadside_pedestrian",
        kind=ActorKind.PEDESTRIAN,
        blueprint=PEDESTRIAN,
        at_s=1.0,
        forward_m=48.0,
        left_m=4.0,
        yaw_offset_deg=-90.0,
        jitter_m=0.3,
        # Stands on the pavement for 2 s, steps into the lane over 3 s, waits
        # 5 s, steps back out: an object entering and then leaving the ego
        # path. Timed against the measured ego (cruise 8 m/s from a standing
        # start reaches 48 m in about 9 s).
        motion=[
            LiveMotion(start_s=2.0, stop_s=5.0, left_mps=-1.2),
            LiveMotion(start_s=10.0, stop_s=13.0, left_mps=1.2),
        ],
        lifetime_s=24.0,
    )
    second_parked = SpawnEvent(
        actor_id="parked_car_kerb",
        kind=ActorKind.VEHICLE,
        blueprint="vehicle.mini.cooper_s",
        at_s=1.0,
        forward_m=28.0,
        left_m=-3.5,
        jitter_m=0.5,
        lifetime_s=40.0,
    )
    definitions = [
        LiveScenarioDefinition(
            scenario_id="static_obstacle",
            name="Static Obstacle",
            description=(
                "A parked car appears 45 m ahead in the lane one second in and leaves "
                "after 22 s: the ego must slow, hold, then resume. The obstacle-stop demo."
            ),
            default_seed=42,
            events=[obstacle],
        ),
        LiveScenarioDefinition(
            scenario_id="pedestrian_crossing",
            name="Pedestrian Crossing",
            description="A walker crosses the road from the left, 32 m ahead, at 1.2 m/s.",
            default_seed=42,
            events=[pedestrian],
        ),
        LiveScenarioDefinition(
            scenario_id="cyclist_crossing",
            name="Cyclist Crossing",
            description="A bicycle crosses from the right, 36 m ahead, at 3 m/s.",
            default_seed=42,
            events=[cyclist],
        ),
        LiveScenarioDefinition(
            scenario_id="vehicle_cut_in",
            name="Vehicle Cut-In",
            description="A car in the adjacent lane drifts into the ego lane ahead and slows.",
            default_seed=42,
            events=[cut_in],
        ),
        LiveScenarioDefinition(
            scenario_id="pedestrian_roadside",
            name="Pedestrian at the Roadside",
            description=(
                "A walker stands on the pavement 48 m ahead, steps into the lane at 3 s, "
                "waits, and steps back out at 11 s: an object entering and leaving the ego path."
            ),
            default_seed=42,
            events=[roadside],
        ),
        LiveScenarioDefinition(
            scenario_id="multiple_vehicles",
            name="Multiple Vehicles",
            description=(
                "Eight Traffic-Manager vehicles, a car parked at the kerb 28 m ahead and a "
                "car stopped in the lane at 45 m."
            ),
            default_seed=42,
            traffic_vehicles=8,
            traffic_blueprints=VEHICLE_BLUEPRINTS,
            events=[second_parked, obstacle.model_copy(update={"lifetime_s": 30.0})],
        ),
        LiveScenarioDefinition(
            scenario_id="random_urban_traffic",
            name="Random Urban Traffic",
            description=(
                "Twelve Traffic-Manager vehicles on the map's spawn points, seeded; "
                "no scripted actor."
            ),
            default_seed=42,
            traffic_vehicles=12,
            traffic_blueprints=VEHICLE_BLUEPRINTS,
        ),
        LiveScenarioDefinition(
            scenario_id="mixed_obstacles",
            name="Mixed Urban Scenario",
            description=(
                "Six traffic vehicles, then a parked car at 1 s, a pedestrian crossing "
                "at 28 s and a cyclist at 48 s, each placed ahead of wherever the ego is."
            ),
            default_seed=42,
            traffic_vehicles=6,
            traffic_blueprints=VEHICLE_BLUEPRINTS,
            events=[
                obstacle,
                pedestrian.model_copy(update={"at_s": 28.0}),
                cyclist.model_copy(update={"at_s": 48.0}),
            ],
        ),
    ]
    return {definition.scenario_id: definition for definition in definitions}


LIVE_CATALOGUE: dict[str, LiveScenarioDefinition] = _catalogue()


def live_scenario_ids() -> list[str]:
    """Catalogue ids, in catalogue order."""
    return list(LIVE_CATALOGUE)


def load_live(scenario_id: str) -> LiveScenarioDefinition:
    """A catalogue scenario by id."""
    try:
        return LIVE_CATALOGUE[scenario_id]
    except KeyError as exc:
        raise LiveScenarioError(
            f"unknown live scenario '{scenario_id}'; known: {', '.join(LIVE_CATALOGUE)}",
            details={"scenario_id": scenario_id},
        ) from exc


def summaries() -> list[LiveScenarioSummary]:
    """What the dashboard lists."""
    return [
        LiveScenarioSummary(
            scenario_id=d.scenario_id,
            name=d.name,
            description=d.description,
            default_seed=d.default_seed,
            traffic_vehicles=d.traffic_vehicles,
            event_count=len(d.events),
        )
        for d in LIVE_CATALOGUE.values()
    ]


# ---------------------------------------------------------------------------
# resolution
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class ResolvedSpawn:
    event: SpawnEvent
    base: tuple[float, float]


@dataclass(frozen=True)
class ResolvedLiveScenario:
    definition: LiveScenarioDefinition
    seed: int
    spawns: list[ResolvedSpawn]
    traffic: list[tuple[str, int]] = field(default_factory=list)


def resolve_live(
    definition: LiveScenarioDefinition, seed: int, *, spawn_point_count: int
) -> ResolvedLiveScenario:
    """Draw every randomised value, once, from ``random.Random(seed)``.

    Draw order is fixed - scripted events first, then traffic - so adding a
    traffic vehicle never changes where a scripted actor lands.
    """
    rng = random.Random(seed)
    spawns: list[ResolvedSpawn] = []
    for event in definition.events:
        if event.jitter_m > 0.0:
            d_forward = rng.uniform(-event.jitter_m, event.jitter_m)
            d_left = rng.uniform(-event.jitter_m, event.jitter_m)
        else:
            d_forward, d_left = 0.0, 0.0
        spawns.append(ResolvedSpawn(event, (event.forward_m + d_forward, event.left_m + d_left)))
    traffic: list[tuple[str, int]] = []
    if definition.traffic_vehicles and spawn_point_count > 1:
        indices = list(range(spawn_point_count))
        rng.shuffle(indices)
        for index in indices[: definition.traffic_vehicles]:
            traffic.append((rng.choice(definition.traffic_blueprints), index))
    return ResolvedLiveScenario(definition, seed, spawns, traffic)


# ---------------------------------------------------------------------------
# manager
# ---------------------------------------------------------------------------
@dataclass
class _Live:
    spawn: ResolvedSpawn
    handle: Any
    anchor: WorldAnchor
    spawned_at_s: float
    simulator_actor_id: int


class LiveScenarioManager:
    """Spawns, moves and removes a resolved scenario's actors on the session clock."""

    def __init__(self, resolved: ResolvedLiveScenario, session: CarlaSimulationSession) -> None:
        self._resolved = resolved
        self._session = session
        self._pending = list(resolved.spawns)
        self._active: dict[str, _Live] = {}
        self._traffic: list[Any] = []
        self._notes: list[str] = []

    @property
    def traffic_count(self) -> int:
        """Traffic vehicles actually spawned (best effort)."""
        return len(self._traffic)

    @property
    def active(self) -> list[ActiveActor]:
        """Scripted actors currently in the world."""
        return [
            ActiveActor(
                actor_id=item.spawn.event.actor_id,
                kind=item.spawn.event.kind,
                blueprint=item.spawn.event.blueprint,
                simulator_actor_id=item.simulator_actor_id,
                spawned_at_s=item.spawned_at_s,
            )
            for item in self._active.values()
        ]

    def spawn_traffic(self) -> int:
        """Put the resolved traffic on the map. A refused point is skipped, logged."""
        for blueprint, index in self._resolved.traffic:
            try:
                actor = self._session.spawn_traffic_vehicle(blueprint, index)
            except SimulatorUnavailableError as exc:
                logger.warning("traffic spawn failed", extra={"context": {"error": str(exc)}})
                actor = None
            if actor is not None:
                self._traffic.append(actor)
        return len(self._traffic)

    def advance(self, session_time_s: float) -> list[str]:
        """Apply everything due at ``session_time_s``; returns event descriptions."""
        notes: list[str] = []
        for spawn in list(self._pending):
            if session_time_s + 1e-9 >= spawn.event.at_s:
                self._pending.remove(spawn)
                anchor = self._session.anchor()
                handle, base = self._spawn_with_fallback(spawn, anchor)
                if handle is None:
                    notes.append(f"spawn of {spawn.event.actor_id} refused at every offset tried")
                    continue
                self._session.place_relative_to(
                    handle,
                    anchor,
                    forward_m=base[0],
                    left_m=base[1],
                    up_m=spawn.event.up_m,
                    yaw_offset_deg=spawn.event.yaw_offset_deg,
                )
                self._active[spawn.event.actor_id] = _Live(
                    ResolvedSpawn(spawn.event, base), handle, anchor, session_time_s, int(handle.id)
                )
                shifted = "" if base == spawn.base else " (shifted from the refused placement)"
                notes.append(
                    f"{spawn.event.kind.value} '{spawn.event.actor_id}' appeared "
                    f"{base[0]:.0f} m ahead, {base[1]:+.0f} m left of the ego{shifted}"
                )
        for actor_id, item in list(self._active.items()):
            elapsed = session_time_s - item.spawned_at_s
            lifetime = item.spawn.event.lifetime_s
            if lifetime is not None and elapsed >= lifetime:
                self._session.destroy_actor(item.handle)
                del self._active[actor_id]
                notes.append(f"{item.spawn.event.kind.value} '{actor_id}' left the scene")
                continue
            if item.spawn.event.motion:
                forward, left = item.spawn.event.expected_offset(elapsed, item.spawn.base)
                self._session.place_relative_to(
                    item.handle,
                    item.anchor,
                    forward_m=forward,
                    left_m=left,
                    up_m=item.spawn.event.up_m,
                    yaw_offset_deg=item.spawn.event.yaw_offset_deg,
                )
        return notes

    def _spawn_with_fallback(
        self, spawn: ResolvedSpawn, anchor: WorldAnchor
    ) -> tuple[Any | None, tuple[float, float]]:
        """Spawn at the resolved placement, or at the nearest of a fixed offset ladder.

        A CARLA server refuses a spawn that intersects street furniture. The
        ladder is deterministic (same seed, same outcome on the same server)
        and small - a metre or two along and beside the intended spot - so the
        scene stays the one the scenario described. The offset actually used
        is recorded in the event.
        """
        forward, left = spawn.base
        ladder = [
            (0.0, 0.0),
            (0.0, 1.0),
            (0.0, -1.0),
            (2.0, 0.0),
            (-2.0, 0.0),
            (2.0, 1.0),
            (2.0, -1.0),
        ]
        for d_forward, d_left in ladder:
            base = (forward + d_forward, left + d_left)
            try:
                handle = self._session.spawn_relative_to(
                    anchor,
                    spawn.event.blueprint,
                    forward_m=base[0],
                    left_m=base[1],
                    up_m=spawn.event.up_m,
                )
            except SimulatorUnavailableError:
                continue
            return handle, base
        return None, spawn.base

    def clear(self) -> None:
        """Forget every handle; the session destroys the actors on close."""
        self._active.clear()
        self._traffic.clear()
        self._pending.clear()


__all__ = [
    "LIVE_CATALOGUE",
    "LiveScenarioError",
    "LiveScenarioManager",
    "ResolvedLiveScenario",
    "ResolvedSpawn",
    "live_scenario_ids",
    "load_live",
    "resolve_live",
    "summaries",
]
