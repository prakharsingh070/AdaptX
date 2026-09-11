"""The scenario catalogue (Phase 10).

Four scenarios, each exercising something specific in the existing pipeline.
Deliberately small: a scenario earns its place by testing a behaviour the
others do not, and a catalogue of twenty near-duplicates would make Phase 11
slower without making it more informative.

Every scenario here is a :class:`~adaptx.scenarios.models.ScenarioDefinition`
- data, not code - built by a function only so the definition is validated
at the moment it is requested. None uses placement jitter, so each is exact;
the seed is retained and reported regardless (ADR-046).

Blueprints are CARLA's. That is the one simulator-specific thing a definition
carries, and it is a string the adapter resolves, not an import.
"""

from __future__ import annotations

from collections.abc import Callable

from adaptx.models.common import ObjectClass
from adaptx.scenarios.models import (
    EgoDefinition,
    MotionSegment,
    Placement,
    ScenarioActor,
    ScenarioDefinition,
)

#: Timestep shared by every catalogue scenario. 20 Hz, matching the default
#: LiDAR rotation so each tick delivers one full sweep.
CATALOGUE_TIMESTEP_S = 0.05

#: Seed shared by every catalogue scenario. Retained on every result.
CATALOGUE_SEED = 20260101


def stationary_vehicle() -> ScenarioDefinition:
    """One vehicle, not moving. The simplest scene that produces a detection.

    Exercises: detection of a static object, a track that confirms and holds,
    a **measured standstill** (velocity 0.0, not null), a stationary
    prediction, and a resolution allocation that should settle and not
    change once the dwell time has passed.
    """
    return ScenarioDefinition(
        scenario_id="stationary_vehicle",
        name="Stationary vehicle",
        description=(
            "A single parked vehicle 20 m ahead in the adjacent lane. Establishes "
            "the baseline: static detection, a confirmed track with measured zero "
            "velocity, and a resolution allocation that settles."
        ),
        seed=CATALOGUE_SEED,
        duration_s=2.0,
        fixed_delta_seconds=CATALOGUE_TIMESTEP_S,
        ego=EgoDefinition(),
        actors=[
            ScenarioActor(
                actor_id="parked",
                blueprint="vehicle.audi.tt",
                object_class=ObjectClass.VEHICLE,
                placement=Placement(forward_m=20.0, left_m=3.5),
            )
        ],
        tags=["baseline", "static"],
    )


def vehicle_approach() -> ScenarioDefinition:
    """One vehicle closing head-on in the adjacent lane. The Phase 9 smoke scene.

    Exercises: a measured non-zero velocity, a trajectory that predicts an
    approach, a risk score that rises as distance falls, and a resolution
    controller that should refine the regions the vehicle is entering before
    it arrives.
    """
    return ScenarioDefinition(
        scenario_id="vehicle_approach",
        name="Vehicle approach",
        description=(
            "A vehicle starts 45 m ahead in the adjacent lane and closes at 8 m/s "
            "for the whole run. The scene Phase 9 hard-coded, now as a definition: "
            "measured velocity, predicted approach, rising risk, refinement ahead "
            "of arrival."
        ),
        seed=CATALOGUE_SEED,
        duration_s=3.0,
        fixed_delta_seconds=CATALOGUE_TIMESTEP_S,
        ego=EgoDefinition(),
        actors=[
            ScenarioActor(
                actor_id="approaching",
                blueprint="vehicle.audi.tt",
                object_class=ObjectClass.VEHICLE,
                placement=Placement(forward_m=45.0, left_m=3.5),
                motion=[MotionSegment(start_s=0.0, stop_s=3.0, forward_mps=-8.0)],
            )
        ],
        tags=["dynamic", "approach"],
    )


def pedestrian_crossing() -> ScenarioDefinition:
    """A pedestrian waits, then crosses the ego's path from right to left.

    Exercises: a **timed start** - the object is stationary for the first
    second, so tracking must measure zero and then a change; lateral motion
    across the forward axis, which is what makes the predicted path cross
    the ego line; and a **timed stop** at the far kerb.
    """
    return ScenarioDefinition(
        scenario_id="pedestrian_crossing",
        name="Pedestrian crossing",
        description=(
            "A pedestrian stands 15 m ahead and 4 m to the right for one second, "
            "walks left across the ego's path at 1.4 m/s for four seconds, then "
            "stops on the far side. Timed start, lateral crossing, timed stop."
        ),
        seed=CATALOGUE_SEED,
        duration_s=6.0,
        fixed_delta_seconds=CATALOGUE_TIMESTEP_S,
        ego=EgoDefinition(),
        actors=[
            ScenarioActor(
                actor_id="pedestrian",
                blueprint="walker.pedestrian.0001",
                object_class=ObjectClass.PEDESTRIAN,
                placement=Placement(forward_m=15.0, left_m=-4.0),
                motion=[MotionSegment(start_s=1.0, stop_s=5.0, left_mps=1.4)],
            )
        ],
        tags=["dynamic", "crossing", "timed"],
    )


def cyclist_crossing() -> ScenarioDefinition:
    """A cyclist crosses diagonally while a vehicle waits ahead: two actors.

    Exercises: **two simultaneous tracks** with different classes and speeds,
    diagonal motion (both velocity components non-zero), a moving object
    passing near a stationary one - which is where centroid association is
    weakest (a documented Phase 4 limitation) - and a multi-region allocation.
    """
    return ScenarioDefinition(
        scenario_id="cyclist_crossing",
        name="Cyclist crossing",
        description=(
            "A vehicle waits 25 m ahead. A cyclist starts 12 m ahead and 8 m to "
            "the left and rides diagonally forward-right at 5 m/s, passing near "
            "the vehicle. Two classes, two speeds, one close pass."
        ),
        seed=CATALOGUE_SEED,
        duration_s=4.0,
        fixed_delta_seconds=CATALOGUE_TIMESTEP_S,
        ego=EgoDefinition(),
        actors=[
            ScenarioActor(
                actor_id="waiting_vehicle",
                blueprint="vehicle.audi.tt",
                object_class=ObjectClass.VEHICLE,
                placement=Placement(forward_m=25.0, left_m=0.0),
            ),
            ScenarioActor(
                actor_id="cyclist",
                blueprint="vehicle.diamondback.century",
                object_class=ObjectClass.CYCLIST,
                placement=Placement(forward_m=12.0, left_m=8.0),
                motion=[MotionSegment(start_s=0.5, stop_s=3.5, forward_mps=3.0, left_mps=-4.0)],
            ),
        ],
        tags=["dynamic", "crossing", "multi_actor"],
    )


#: Every catalogue scenario, keyed by id, in a stable order.
CATALOGUE: dict[str, Callable[[], ScenarioDefinition]] = {
    "stationary_vehicle": stationary_vehicle,
    "vehicle_approach": vehicle_approach,
    "pedestrian_crossing": pedestrian_crossing,
    "cyclist_crossing": cyclist_crossing,
}


def scenario_ids() -> list[str]:
    """Catalogue ids, in catalogue order."""
    return list(CATALOGUE)


def load(scenario_id: str) -> ScenarioDefinition:
    """Build the catalogue scenario ``scenario_id``, validated.

    Raises:
        KeyError: no such scenario. The message lists what exists.
    """
    try:
        return CATALOGUE[scenario_id]()
    except KeyError:
        raise KeyError(
            f"no scenario '{scenario_id}'; available: {', '.join(scenario_ids())}"
        ) from None


__all__ = [
    "CATALOGUE",
    "CATALOGUE_SEED",
    "CATALOGUE_TIMESTEP_S",
    "cyclist_crossing",
    "load",
    "pedestrian_crossing",
    "scenario_ids",
    "stationary_vehicle",
    "vehicle_approach",
]
