"""Scenario run results (Phase 10).

What a run *produced*, as raw evidence. Frame counts, simulator frame ids,
the scripted pose each actor was commanded to, the ground truth the simulator
reported, and the counts each pipeline stage returned.

**No conclusions.** There is no accuracy figure, no error distance, no
match rate anywhere in these contracts, because computing one is evaluation
and evaluation is Phase 11. A result that carried "detection accuracy: 94%"
would be presenting a Phase 11 claim without Phase 11 having been done.

What it does carry is everything Phase 11 will need to compute those things:
for every frame, the simulator frame id joins the LiDAR frame to its ground
truth, and the scripted expected pose sits beside both.
"""

from __future__ import annotations

from datetime import datetime
from itertools import pairwise

from pydantic import Field, model_validator

from adaptx.carla.ground_truth import GroundTruthFrame
from adaptx.models.common import AdaptXModel, DataSource, TimestampedModel, Vector3
from adaptx.scenarios.models import ResolvedScenario, ScenarioState


class ExpectedPose(AdaptXModel):
    """Where the scenario said an actor should be on one frame.

    Scripted intent, not a measurement. It is exact by construction and is
    the third leg of the comparison Phase 11 will make: what was commanded,
    what the simulator reported, what perception inferred.
    """

    actor_id: str = Field(min_length=1)
    position: Vector3 = Field(description="Ego-relative, ADAPT-X frame.")


class ScenarioActorRecord(AdaptXModel):
    """The link between a scenario actor and the simulator actor it became."""

    actor_id: str = Field(min_length=1, description="Scenario-level name.")
    simulator_actor_id: int = Field(ge=0, description="Id the simulator assigned on spawn.")
    blueprint: str = Field(min_length=1)


class StageCounts(AdaptXModel):
    """What each pipeline stage returned for one frame. Counts, never judgements."""

    processed_points: int = Field(ge=0)
    detections: int = Field(ge=0)
    tracks: int = Field(ge=0)
    trajectories: int = Field(ge=0)
    risk_level: str = Field(min_length=1)
    adaptive_cells: int = Field(ge=0)
    fixed_cells: int = Field(ge=0)
    regions_by_level: dict[str, int] = Field(default_factory=dict)
    stage_ms: dict[str, float] = Field(
        default_factory=dict, description="Measured duration per stage on this machine."
    )

    @property
    def total_ms(self) -> float:
        """Measured pipeline time for the frame, excluding simulator time."""
        return sum(self.stage_ms.values())


class ScenarioFrameRecord(AdaptXModel):
    """One frame of a run, with the identifiers that join it to everything else."""

    frame_index: int = Field(ge=0, description="0-based position within the scenario.")
    scenario_time_s: float = Field(ge=0.0, description="frame_index * fixed_delta_seconds.")
    simulator_frame_id: int = Field(
        ge=0, description="Frame number the simulator assigned; joins LiDAR to ground truth."
    )
    timestamp: datetime = Field(description="Simulation time of the frame, never wall clock.")
    point_count: int = Field(ge=0)
    expected_poses: list[ExpectedPose] = Field(default_factory=list)
    ground_truth_actor_count: int = Field(ge=0, description="Non-ego actors the simulator saw.")
    pipeline: StageCounts | None = Field(
        default=None, description="Null when the frame was recorded without processing."
    )


class ScenarioRunResult(TimestampedModel):
    """Everything one run produced, plus the definition that produced it.

    Carrying the full :class:`ResolvedScenario` means a result is
    self-describing: the seed, the drawn placements and the timestep are all
    here, so the run can be reproduced from the result without the catalogue.
    """

    scenario_id: str = Field(min_length=1)
    name: str = Field(min_length=1)
    seed: int = Field(ge=0)
    state: ScenarioState
    error: str | None = Field(default=None, description="Why the run failed, when it did.")

    resolved: ResolvedScenario
    actors: list[ScenarioActorRecord] = Field(default_factory=list)
    map_name: str | None = None
    simulator_version: str = Field(min_length=1)
    fixed_delta_seconds: float = Field(gt=0.0)

    frames: list[ScenarioFrameRecord] = Field(default_factory=list)
    ground_truth: list[GroundTruthFrame] = Field(default_factory=list)
    source: DataSource = Field(
        default=DataSource.SIMULATION,
        description="Always SIMULATION. A scenario run is never sensor data.",
    )

    @model_validator(mode="after")
    def _check_alignment(self) -> ScenarioRunResult:
        if len(self.ground_truth) != len(self.frames):
            raise ValueError(
                f"{len(self.frames)} frames but {len(self.ground_truth)} ground-truth frames; "
                "every frame must have exactly one"
            )
        for record, truth in zip(self.frames, self.ground_truth, strict=True):
            if record.simulator_frame_id != truth.frame_id:
                raise ValueError(
                    f"frame {record.frame_index}: LiDAR frame {record.simulator_frame_id} does "
                    f"not match ground-truth frame {truth.frame_id}"
                )
        indices = [record.frame_index for record in self.frames]
        if indices != list(range(len(indices))):
            raise ValueError("frame records must be contiguous from 0")
        return self

    @property
    def frame_count(self) -> int:
        """Frames actually stepped. May be fewer than planned when a run failed."""
        return len(self.frames)

    @property
    def planned_frame_count(self) -> int:
        """Frames the definition asked for."""
        return self.resolved.definition.frame_count

    @property
    def completed(self) -> bool:
        """Whether every planned frame was stepped."""
        return self.state is ScenarioState.COMPLETED

    @property
    def start_time(self) -> datetime | None:
        """Simulation time of the first frame, or ``None`` when nothing ran."""
        return self.frames[0].timestamp if self.frames else None

    @property
    def end_time(self) -> datetime | None:
        """Simulation time of the last frame, or ``None`` when nothing ran."""
        return self.frames[-1].timestamp if self.frames else None

    def timestamps_are_monotonic(self) -> bool:
        """Whether simulation time strictly increased across the run."""
        times = [record.timestamp for record in self.frames]
        return all(later > earlier for earlier, later in pairwise(times))

    def frame_intervals_s(self) -> list[float]:
        """Measured gap between successive frames, in seconds."""
        times = [record.timestamp for record in self.frames]
        return [(later - earlier).total_seconds() for earlier, later in pairwise(times)]
