"""The evaluation view of a run record.

A :class:`~adaptx.scenarios.result.ScenarioRunResult` is evidence in the
form the run produced it. This module reads it into the shape the metrics
need - one :class:`EvaluationFrame` per processed frame, with the pipeline
outputs, the ground truth expressed in the **sensor frame**, and a reference
velocity - and performs the two adjustments the evaluation makes to ground
truth, both documented in ``docs/EVALUATION.md`` §2:

1. **Frame.** Perception reports positions in the sensor frame (ADR-013: no
   transform is applied). Ground truth is relative to the ego actor origin.
   The recorded mount offset is subtracted here, once.
2. **Velocity.** The recorded ground-truth velocity is the simulator's
   physics velocity of an actor that is *placed* every tick (ADR-047), which
   bears no relation to the scripted motion. The reference velocity is the
   finite difference of consecutive ground-truth positions.

Nothing here modifies the record.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from adaptx.carla.ground_truth import GroundTruthActor, GroundTruthFrame
from adaptx.core.exceptions import AdaptXError
from adaptx.models.common import Dimensions, ObjectClass, Vector3
from adaptx.models.spatial_map import MapBounds
from adaptx.scenarios.result import (
    PipelineFrameOutputs,
    ScenarioFrameRecord,
    ScenarioRunResult,
)


class EvaluationInputError(AdaptXError):
    """The record cannot be evaluated as it stands."""

    code = "evaluation_input_error"


@dataclass(frozen=True)
class ReferenceActor:
    """One ground-truth actor on one frame, in the sensor frame.

    ``position`` is what a detection or track should report for this actor.
    ``reference_velocity`` is ``None`` on the first frame the actor appears,
    because a finite difference needs two positions - and null is not zero.
    """

    actor_id: str
    simulator_actor_id: int
    object_class: ObjectClass
    position: Vector3
    reference_velocity: Vector3 | None
    dimensions: Dimensions
    distance_m: float
    eligible: bool
    ineligibility_reason: str | None = None


@dataclass(frozen=True)
class EvaluationFrame:
    """Everything the metrics need for one processed frame."""

    index: int
    time_s: float
    record: ScenarioFrameRecord
    outputs: PipelineFrameOutputs
    truth: GroundTruthFrame
    actors: tuple[ReferenceActor, ...]
    ego_position: Vector3 | None

    @property
    def eligible_actors(self) -> tuple[ReferenceActor, ...]:
        """Actors a correctly working pipeline could have perceived."""
        return tuple(actor for actor in self.actors if actor.eligible)


@dataclass(frozen=True)
class EvaluationDataset:
    """A run record read for evaluation."""

    result: ScenarioRunResult
    frames: tuple[EvaluationFrame, ...]
    actor_names: dict[int, str]
    sensor_mount: Vector3
    sensor_range_m: float | None
    map_bounds: MapBounds | None
    ego_stationary: bool | None
    warnings: list[str] = field(default_factory=list)

    @property
    def timestep_s(self) -> float:
        """The run's fixed simulation timestep."""
        return self.result.fixed_delta_seconds

    @property
    def scenario_actor_ids(self) -> list[str]:
        """Scenario-level names of the spawned actors, in spawn order."""
        return [record.actor_id for record in self.result.actors]

    def frame_at(self, index: int) -> EvaluationFrame | None:
        """The evaluated frame with ``index``, or ``None`` if it was not evaluated."""
        for frame in self.frames:
            if frame.index == index:
                return frame
        return None


def load_dataset(
    result: ScenarioRunResult, *, include_map_actors: bool = False
) -> EvaluationDataset:
    """Read a run record into an :class:`EvaluationDataset`.

    Args:
        result: The record. Must carry pipeline outputs on its processed
            frames; a counts-only record cannot support any metric here.
        include_map_actors: Treat every non-ego actor as ground truth rather
            than only the actors the scenario spawned.

    Raises:
        EvaluationInputError: no frame carries pipeline outputs.
    """
    processed = [record for record in result.frames if record.pipeline is not None]
    if not processed:
        raise EvaluationInputError(
            "the run record has no processed frames; nothing can be evaluated",
            details={"scenario_id": result.scenario_id, "frames": result.frame_count},
        )
    if not result.has_outputs:
        raise EvaluationInputError(
            "the run record carries stage counts but not the pipeline outputs; re-run the "
            "scenario with outputs recorded (the default since Phase 11)",
            details={"scenario_id": result.scenario_id},
        )

    warnings: list[str] = []
    if result.sensor is None:
        mount = Vector3()
        sensor_range: float | None = None
        warnings.append(
            "the record carries no sensor configuration: the mount offset is assumed zero "
            "and range eligibility is not applied"
        )
    else:
        mount = result.sensor.mount
        sensor_range = result.sensor.range_m

    actor_names = {record.simulator_actor_id: record.actor_id for record in result.actors}
    bounds = _map_bounds(result)

    previous_positions: dict[int, Vector3] = {}
    frames: list[EvaluationFrame] = []
    ego_positions: list[Vector3] = []
    for record, truth in zip(result.frames, result.ground_truth, strict=True):
        if record.outputs is None:
            # Unprocessed frames still advance the reference so a later
            # finite difference spans exactly one timestep.
            for actor in truth.others():
                previous_positions[actor.actor_id] = actor.position
            continue
        ego = next((actor for actor in truth.actors if actor.is_ego), None)
        if ego is not None:
            ego_positions.append(ego.world_position)

        references: list[ReferenceActor] = []
        for actor in truth.others():
            if not include_map_actors and actor.actor_id not in actor_names:
                previous_positions[actor.actor_id] = actor.position
                continue
            reference = _reference_actor(
                actor,
                name=actor_names.get(actor.actor_id, actor.type_id),
                previous=previous_positions.get(actor.actor_id),
                timestep_s=result.fixed_delta_seconds,
                mount=mount,
                sensor_range_m=sensor_range,
                bounds=bounds,
            )
            references.append(reference)
            previous_positions[actor.actor_id] = actor.position

        frames.append(
            EvaluationFrame(
                index=record.frame_index,
                time_s=record.scenario_time_s,
                record=record,
                outputs=record.outputs,
                truth=truth,
                actors=tuple(references),
                ego_position=None if ego is None else ego.world_position,
            )
        )

    ego_stationary: bool | None
    if not ego_positions:
        ego_stationary = None
        warnings.append("no ego actor in the ground truth; ego motion could not be checked")
    else:
        first = ego_positions[0]
        ego_stationary = all(_planar_distance(first, later) <= 1e-3 for later in ego_positions)
        if not ego_stationary:
            warnings.append(
                "the ego moved during the run; ego-relative ground truth on later frames "
                "is not directly comparable with a prediction made earlier"
            )

    return EvaluationDataset(
        result=result,
        frames=tuple(frames),
        actor_names=actor_names,
        sensor_mount=mount,
        sensor_range_m=sensor_range,
        map_bounds=bounds,
        ego_stationary=ego_stationary,
        warnings=warnings,
    )


def _map_bounds(result: ScenarioRunResult) -> MapBounds | None:
    for record in result.frames:
        if record.outputs is not None:
            return record.outputs.fixed_map.bounds
    return None


def _reference_actor(
    actor: GroundTruthActor,
    *,
    name: str,
    previous: Vector3 | None,
    timestep_s: float,
    mount: Vector3,
    sensor_range_m: float | None,
    bounds: MapBounds | None,
) -> ReferenceActor:
    position = ego_to_sensor_frame(actor.position, mount)
    velocity = (
        None
        if previous is None
        else Vector3(
            x=(actor.position.x - previous.x) / timestep_s,
            y=(actor.position.y - previous.y) / timestep_s,
            z=(actor.position.z - previous.z) / timestep_s,
        )
    )
    distance = (position.x**2 + position.y**2) ** 0.5
    eligible = True
    reason: str | None = None
    if sensor_range_m is not None and distance > sensor_range_m:
        eligible = False
        reason = f"beyond the sensor range of {sensor_range_m} m"
    elif bounds is not None and not (
        bounds.min_x <= position.x <= bounds.max_x and bounds.min_y <= position.y <= bounds.max_y
    ):
        eligible = False
        reason = "outside the map bounds"
    return ReferenceActor(
        actor_id=name,
        simulator_actor_id=actor.actor_id,
        object_class=actor.object_class,
        position=position,
        reference_velocity=velocity,
        dimensions=actor.dimensions,
        distance_m=distance,
        eligible=eligible,
        ineligibility_reason=reason,
    )


def ego_to_sensor_frame(position: Vector3, mount: Vector3) -> Vector3:
    """Express an ego-frame position in the sensor frame.

    The sensor sits at ``mount`` in the ego frame and shares its axes
    (ADR-009, ADR-013), so the conversion is a translation and nothing else.
    """
    return Vector3(x=position.x - mount.x, y=position.y - mount.y, z=position.z - mount.z)


def _planar_distance(a: Vector3, b: Vector3) -> float:
    return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5


__all__ = [
    "EvaluationDataset",
    "EvaluationFrame",
    "EvaluationInputError",
    "ReferenceActor",
    "ego_to_sensor_frame",
    "load_dataset",
]
