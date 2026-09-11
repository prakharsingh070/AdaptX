"""Scenario definition contracts (Phase 10).

A scenario is **configuration, not code**: a declarative description of a
scene that produces the same simulation every time it is run. Actors, their
initial placement, their scripted motion, the duration, the timestep and the
seed all live here as data, so a run is reproducible from the description
alone rather than from a git commit (ADR-046).

Nothing in this module knows about CARLA. It imports no simulator, no session
and no adapter, so the same definition could drive a different simulator
tomorrow. The runner (:mod:`adaptx.scenarios.runner`) is where a definition
meets a simulator, and it is the *only* place that does.

Coordinates and time
--------------------
Every position and velocity is in the **ego frame** in the ADAPT-X convention
(ADR-009): +x ahead of the vehicle, +y to its left, metres. A scenario says
"15 m ahead and 3 m to the left"; converting that into a world pose is the
adapter's job (ADR-043).

Every time is **scenario time**: seconds since the first frame, advancing by
exactly ``fixed_delta_seconds`` per frame. Never a wall clock (ADR-044).

Motion is scripted, not simulated
---------------------------------
An actor moves along **timed constant-velocity segments** (ADR-047). Its
position at any scenario time is a closed-form sum, so the scenario can state
where every actor *should* be on every frame without a simulator running.
That expected pose is recorded beside what the simulator reports, which is the
raw material a later evaluation phase needs - but no evaluation happens here.
"""

from __future__ import annotations

import math
from enum import StrEnum

from pydantic import Field, field_validator, model_validator

from adaptx.models.common import AdaptXModel, ObjectClass, Vector3

#: Smallest permitted scenario timestep. Mirrors the CARLA physics ceiling on
#: the upper side; on the lower side this only guards against a zero divide.
MIN_TIMESTEP_S = 1e-3
#: Largest permitted timestep, matching ``CarlaSettings.fixed_delta_seconds``.
MAX_TIMESTEP_S = 0.1


class ScenarioState(StrEnum):
    """Where a scenario run is in its lifecycle.

    ``CREATED``
        Definition constructed; nothing checked against a simulator yet.
    ``VALIDATING``
        Cross-checking the definition and resolving the seed.
    ``READY``
        Resolved and validated; the simulator has been opened and actors
        spawned. Not yet stepped.
    ``RUNNING``
        At least one frame stepped.
    ``COMPLETED``
        Every frame stepped and the simulator closed cleanly.
    ``FAILED``
        Validation, setup or stepping failed. Cleanup has run; the reason is
        on the result.
    """

    CREATED = "CREATED"
    VALIDATING = "VALIDATING"
    READY = "READY"
    RUNNING = "RUNNING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


class Placement(AdaptXModel):
    """Where an actor starts, relative to the ego, in the ADAPT-X frame."""

    forward_m: float = Field(description="Metres ahead of the ego reference.")
    left_m: float = Field(default=0.0, description="Metres to the left of the ego reference.")
    up_m: float = Field(
        default=0.5,
        description=(
            "Metres above the ego origin at which to spawn. Slightly above ground "
            "so a spawned actor settles onto the road instead of clipping into it."
        ),
    )

    @field_validator("forward_m", "left_m", "up_m")
    @classmethod
    def _require_finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("placement values must be finite")
        return value

    @property
    def distance_m(self) -> float:
        """Planar distance from the ego reference."""
        return math.hypot(self.forward_m, self.left_m)


class MotionSegment(AdaptXModel):
    """One timed interval of constant velocity, in the ego frame.

    An actor with no segments is stationary. Segments are ordered, may not
    overlap, and must fit inside the scenario duration - checked by
    :class:`ScenarioActor` and :class:`ScenarioDefinition`.
    """

    start_s: float = Field(ge=0.0, description="Scenario time at which this motion begins.")
    stop_s: float = Field(gt=0.0, description="Scenario time at which it ends.")
    forward_mps: float = Field(default=0.0, description="Velocity along the ego +x axis.")
    left_mps: float = Field(default=0.0, description="Velocity along the ego +y axis.")

    @field_validator("start_s", "stop_s", "forward_mps", "left_mps")
    @classmethod
    def _require_finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("motion values must be finite")
        return value

    @model_validator(mode="after")
    def _check_interval(self) -> MotionSegment:
        if self.stop_s <= self.start_s:
            raise ValueError(
                f"a motion segment must end after it starts (start {self.start_s}, "
                f"stop {self.stop_s})"
            )
        return self

    @property
    def duration_s(self) -> float:
        """How long the segment lasts."""
        return self.stop_s - self.start_s

    @property
    def speed_mps(self) -> float:
        """Scalar speed during the segment."""
        return math.hypot(self.forward_mps, self.left_mps)

    def displacement_at(self, time_s: float) -> tuple[float, float]:
        """Displacement this segment has contributed by ``time_s``.

        Zero before it starts, growing linearly while active, and frozen at
        its full extent after it stops. Summing this over every segment gives
        an actor's position in closed form.
        """
        active = min(max(time_s, self.start_s), self.stop_s) - self.start_s
        return (self.forward_mps * active, self.left_mps * active)


class ScenarioActor(AdaptXModel):
    """One non-ego actor: what it is, where it starts and how it moves.

    ``actor_id`` is the scenario's own stable name for the actor. The
    simulator assigns a separate numeric id when it spawns, and the runner
    records the mapping between the two so ground truth stays traceable to the
    definition.
    """

    actor_id: str = Field(min_length=1, description="Stable scenario-level name, e.g. 'target'.")
    blueprint: str = Field(
        min_length=1, description="Simulator blueprint id, e.g. 'vehicle.audi.tt'."
    )
    object_class: ObjectClass = Field(
        description="What this actor is meant to be, so the intent is on record."
    )
    placement: Placement
    placement_jitter_m: float = Field(
        default=0.0,
        ge=0.0,
        description=(
            "Half-width of a seeded uniform perturbation applied to the initial "
            "forward and left offsets. 0.0 means the placement is exact. This is "
            "the only randomised element a scenario has, and the seed controls it."
        ),
    )
    motion: list[MotionSegment] = Field(
        default_factory=list, description="Ordered, non-overlapping. Empty means stationary."
    )

    @model_validator(mode="after")
    def _check_segments(self) -> ScenarioActor:
        for earlier, later in zip(self.motion, self.motion[1:], strict=False):
            if later.start_s < earlier.stop_s:
                raise ValueError(
                    f"actor '{self.actor_id}': motion segments overlap or are out of order "
                    f"({earlier.start_s}-{earlier.stop_s} then {later.start_s}-{later.stop_s})"
                )
        return self

    @property
    def is_stationary(self) -> bool:
        """Whether the actor never moves."""
        return not self.motion or all(segment.speed_mps == 0.0 for segment in self.motion)

    @property
    def motion_ends_s(self) -> float:
        """Scenario time at which the last segment stops; 0.0 when stationary."""
        return max((segment.stop_s for segment in self.motion), default=0.0)

    def expected_offset(self, time_s: float, base: Placement) -> tuple[float, float]:
        """Where the actor should be at ``time_s``, from a (possibly jittered) base.

        Closed form: the base placement plus every segment's displacement so
        far. No integration, no accumulation, no simulator - which is why the
        same time always yields the same answer.
        """
        forward, left = base.forward_m, base.left_m
        for segment in self.motion:
            d_forward, d_left = segment.displacement_at(time_s)
            forward += d_forward
            left += d_left
        return (forward, left)


class EgoDefinition(AdaptXModel):
    """The vehicle carrying the sensor.

    Phase 10 supports a **stationary** ego only. Ego motion would need either
    physics (non-deterministic) or scripted world-frame transforms that also
    move the sensor, and neither is required to exercise the pipeline; every
    catalogue scenario moves the *targets* instead. Recorded as a field rather
    than assumed so the limitation is visible in every definition.
    """

    blueprint: str = Field(default="vehicle.tesla.model3", min_length=1)
    stationary: bool = Field(
        default=True,
        description="Always true in Phase 10. Present so the constraint is explicit and testable.",
    )

    @field_validator("stationary")
    @classmethod
    def _only_stationary(cls, value: bool) -> bool:
        if not value:
            raise ValueError("ego motion is not supported in Phase 10; the ego must be stationary")
        return value


class ScenarioDefinition(AdaptXModel):
    """A complete, reproducible description of one scene.

    Two definitions with equal fields describe the same scenario, and running
    either with the same simulator yields the same frames. The definition is
    copied onto every result so a run can be reproduced from the result alone.
    """

    scenario_id: str = Field(
        min_length=1,
        pattern=r"^[a-z][a-z0-9_]*$",
        description="Stable machine name, e.g. 'pedestrian_crossing'.",
    )
    name: str = Field(min_length=1, description="Human-readable title.")
    description: str = Field(min_length=1, description="What the scenario is for.")
    seed: int = Field(
        ge=0,
        description=(
            "Controls every randomised element. Retained and reported even when "
            "the scenario has none, so a run always records the seed it used."
        ),
    )
    duration_s: float = Field(gt=0.0, description="Scenario length in simulation seconds.")
    fixed_delta_seconds: float = Field(
        ge=MIN_TIMESTEP_S,
        le=MAX_TIMESTEP_S,
        description="Simulation timestep. Frame count is duration / timestep, rounded down.",
    )
    ego: EgoDefinition = Field(default_factory=EgoDefinition)
    actors: list[ScenarioActor] = Field(default_factory=list)
    town: str | None = Field(
        default=None, description="Map to load; None keeps whatever the simulator has."
    )
    tags: list[str] = Field(default_factory=list, description="Free labels for cataloguing.")

    @field_validator("duration_s", "fixed_delta_seconds")
    @classmethod
    def _require_finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("scenario timing must be finite")
        return value

    @model_validator(mode="after")
    def _check_actors_and_timing(self) -> ScenarioDefinition:
        ids = [actor.actor_id for actor in self.actors]
        duplicates = sorted({actor_id for actor_id in ids if ids.count(actor_id) > 1})
        if duplicates:
            raise ValueError(f"duplicate actor ids: {duplicates}")
        if "ego" in ids:
            raise ValueError("'ego' is reserved for the sensor vehicle and cannot name an actor")

        for actor in self.actors:
            for segment in actor.motion:
                if segment.stop_s > self.duration_s + 1e-9:
                    raise ValueError(
                        f"actor '{actor.actor_id}': motion segment ends at {segment.stop_s}s, "
                        f"after the scenario ends at {self.duration_s}s"
                    )

        if self.frame_count < 2:
            raise ValueError(
                f"duration {self.duration_s}s at {self.fixed_delta_seconds}s per frame gives "
                f"{self.frame_count} frame(s); at least 2 are needed for a measured velocity"
            )
        return self

    @property
    def frame_count(self) -> int:
        """Frames the scenario steps: ``floor(duration / timestep)``.

        Rounded down rather than up so no frame lands past the declared
        duration. A small epsilon absorbs the float error in an exact
        division such as ``3.0 / 0.05``.
        """
        return math.floor(self.duration_s / self.fixed_delta_seconds + 1e-9)

    @property
    def has_randomised_elements(self) -> bool:
        """Whether the seed changes anything about this scenario."""
        return any(actor.placement_jitter_m > 0.0 for actor in self.actors)

    def actor(self, actor_id: str) -> ScenarioActor:
        """Look an actor up by its scenario id."""
        for candidate in self.actors:
            if candidate.actor_id == actor_id:
                return candidate
        raise KeyError(f"scenario '{self.scenario_id}' has no actor '{actor_id}'")

    def scenario_time(self, frame_index: int) -> float:
        """Scenario time of the frame at ``frame_index``, from 0."""
        return frame_index * self.fixed_delta_seconds


class ResolvedActor(AdaptXModel):
    """An actor after the seed has been applied: the placement actually used.

    The definition says "15 m ahead, jitter 0.5 m"; the resolved actor says
    "14.83 m ahead". Every frame's expected pose is computed from this, and it
    is recorded on the result so the run is reproducible without re-drawing.
    """

    actor_id: str = Field(min_length=1)
    blueprint: str = Field(min_length=1)
    object_class: ObjectClass
    base: Placement = Field(description="Initial placement after jitter, if any.")
    jitter_applied_m: tuple[float, float] = Field(
        default=(0.0, 0.0),
        description="The (forward, left) perturbation the seed produced; zero when none.",
    )


class ResolvedScenario(AdaptXModel):
    """A definition with every randomised value drawn and fixed.

    Produced once per run from the seed. Two resolutions of the same
    definition with the same seed are identical; with a different seed they
    may differ wherever the definition allowed it.
    """

    definition: ScenarioDefinition
    actors: list[ResolvedActor]

    def expected_pose(self, actor_id: str, time_s: float) -> Vector3:
        """Where the scenario says ``actor_id`` should be at ``time_s``."""
        resolved = next(a for a in self.actors if a.actor_id == actor_id)
        forward, left = self.definition.actor(actor_id).expected_offset(time_s, resolved.base)
        return Vector3(x=forward, y=left, z=resolved.base.up_m)
