"""Scenario execution (Phase 10).

The one place a scenario definition meets a simulator::

    ScenarioDefinition
        -> resolve(seed)            every randomised value drawn, once
        -> ScenarioRunner.run()
             open simulator, spawn actors
             per frame: place actors at their scripted pose
                        step -> RawPointCloudFrame -> (optional) pipeline
                        ground_truth() -> recorded, NEVER fed to the pipeline
             close simulator, always
        -> ScenarioRunResult

Determinism
-----------
Every randomised value comes from ``random.Random(definition.seed)``, an
explicit instance - never the module-level generator, so nothing else in the
process can perturb a draw and nothing here perturbs anyone else. The draw
happens once, before the simulator is opened, and the drawn values are
recorded on the result (ADR-046).

Actor motion is scripted by placing each actor at its closed-form expected
pose every frame rather than by applying a velocity and letting physics
integrate. The pose on frame ``n`` depends only on the definition, the seed
and ``n`` (ADR-047).

Ground truth
------------
Recorded beside every frame and passed to no stage of the pipeline. The
processing callback receives the sensor frame and nothing else, by signature
(ADR-045).

Cleanup
-------
``close()`` runs in a ``finally``: a failed spawn, a sensor timeout or an
exception inside processing all reach it, so a failed run leaves no actors
behind and restores the world settings. The result records ``FAILED`` and the
reason rather than raising through, so a batch of runs survives one bad one.
"""

from __future__ import annotations

import random
from collections.abc import Callable
from typing import Any

from adaptx.carla.client import carla_package_version
from adaptx.carla.session import CarlaSimulationSession
from adaptx.config.settings import Settings, get_settings
from adaptx.core.exceptions import AdaptXError, SimulatorUnavailableError
from adaptx.core.lifecycle import ApplicationContext, build_context
from adaptx.core.logging import get_logger
from adaptx.models.common import DataSource
from adaptx.models.point_cloud import RawPointCloudFrame
from adaptx.scenarios.interfaces import ScenarioSimulator
from adaptx.scenarios.models import (
    Placement,
    ResolvedActor,
    ResolvedScenario,
    ScenarioDefinition,
    ScenarioState,
)
from adaptx.scenarios.result import (
    ExpectedPose,
    ScenarioActorRecord,
    ScenarioFrameRecord,
    ScenarioRunResult,
    StageCounts,
)

logger = get_logger(__name__)

#: Processes one sensor frame and returns stage counts. Receives the frame and
#: nothing else - the signature is the separation guarantee.
FrameProcessor = Callable[[RawPointCloudFrame], StageCounts]


class ScenarioError(AdaptXError):
    """A scenario could not be run as defined."""

    code = "scenario_error"


# ---------------------------------------------------------------------------
# resolution
# ---------------------------------------------------------------------------
def resolve(definition: ScenarioDefinition) -> ResolvedScenario:
    """Draw every randomised value from the seed, once.

    Deterministic by construction: a fresh ``random.Random(seed)`` is consumed
    in actor order, so the same definition and seed always produce the same
    resolved placements, and a scenario with no jitter consumes nothing and
    resolves to its own definition.
    """
    rng = random.Random(definition.seed)
    actors: list[ResolvedActor] = []
    for actor in definition.actors:
        jitter = actor.placement_jitter_m
        if jitter > 0.0:
            d_forward = rng.uniform(-jitter, jitter)
            d_left = rng.uniform(-jitter, jitter)
        else:
            d_forward, d_left = 0.0, 0.0
        base = Placement(
            forward_m=actor.placement.forward_m + d_forward,
            left_m=actor.placement.left_m + d_left,
            up_m=actor.placement.up_m,
        )
        actors.append(
            ResolvedActor(
                actor_id=actor.actor_id,
                blueprint=actor.blueprint,
                object_class=actor.object_class,
                base=base,
                jitter_applied_m=(d_forward, d_left),
            )
        )
    return ResolvedScenario(definition=definition, actors=actors)


# ---------------------------------------------------------------------------
# pipeline processing
# ---------------------------------------------------------------------------
def pipeline_processor(context: ApplicationContext) -> FrameProcessor:
    """A frame processor that runs the existing Phase 2-8 chain, unchanged.

    Every call below is the same call the LiDAR endpoints make. Nothing
    branches on the frame having come from a scenario, and the closure holds
    no reference to any ground truth - it could not use it if it wanted to.
    """

    def process(frame: RawPointCloudFrame) -> StageCounts:
        if frame.source is not DataSource.SIMULATION:
            raise ScenarioError(
                f"a scenario frame must be labelled 'simulation', got '{frame.source.value}'",
                details={"source": frame.source.value},
            )
        processed = context.preprocessor.run(frame)
        detection = context.detector.detect(processed.frame)
        tracking = context.tracking.update(
            detection.objects,
            processed.frame.timestamp,
            frame_id=processed.frame.frame_id,
            sensor_id=processed.frame.sensor_id,
        )
        prediction = context.prediction.predict_from_tracking(tracking)
        spatial_map = context.mapping.build(processed.frame)
        risk = context.risk.assess_from_pipeline(
            tracking, prediction=prediction, spatial_map=spatial_map
        )
        adaptive = context.adaptive_mapping.run_from_pipeline(
            processed.frame, risk, tracking, trajectories=prediction.trajectories
        )
        return StageCounts(
            processed_points=processed.frame.point_count,
            detections=len(detection.objects),
            tracks=len(tracking.tracks),
            trajectories=len(prediction.trajectories),
            risk_level=risk.highest_risk_level.value,
            adaptive_cells=adaptive.accounting.total_cell_count,
            fixed_cells=spatial_map.accounting.total_cell_count,
            regions_by_level=adaptive.tiles_by_level(),
            stage_ms={
                "processing": processed.metrics.duration_ms,
                "detection": detection.duration_ms,
                "tracking": tracking.duration_ms,
                "prediction": prediction.duration_ms,
                "mapping": spatial_map.duration_ms,
                "risk": risk.duration_ms,
                "controller": adaptive.plan.duration_ms,
                "adaptive_mapping": adaptive.duration_ms,
            },
        )

    return process


# ---------------------------------------------------------------------------
# runner
# ---------------------------------------------------------------------------
class ScenarioRunner:
    """Runs one scenario definition against one simulator, once.

    A runner is single-use: it holds the lifecycle state of exactly one run,
    and a second scenario gets a second runner. That is what stops state from
    one run leaking into the next.

    Args:
        definition: What to run.
        simulator: The simulator to drive. Built from ``settings`` when
            omitted; injectable so the runner can be exercised against a
            stand-in.
        processor: What to do with each sensor frame. ``None`` records frames
            and ground truth without processing.
        settings: Application settings; the process defaults when omitted.
    """

    def __init__(
        self,
        definition: ScenarioDefinition,
        *,
        simulator: ScenarioSimulator | None = None,
        processor: FrameProcessor | None = None,
        settings: Settings | None = None,
    ) -> None:
        self._definition = definition
        self._settings = settings if settings is not None else get_settings()
        self._simulator = simulator
        self._processor = processor
        self._state = ScenarioState.CREATED
        self._resolved: ResolvedScenario | None = None
        self._handles: dict[str, Any] = {}
        self._records: list[ScenarioActorRecord] = []

    @property
    def state(self) -> ScenarioState:
        """Current lifecycle state."""
        return self._state

    @property
    def resolved(self) -> ResolvedScenario | None:
        """The resolved scenario, once validation has run."""
        return self._resolved

    # -- lifecycle ---------------------------------------------------------
    def run(self) -> ScenarioRunResult:
        """Validate, execute and clean up.

        A **run-time** failure - a spawn refused, a sensor timeout, an error
        inside processing - is reported on the result with ``state=FAILED``
        and the reason, after cleanup has run, so a batch of runs survives one
        bad one.

        A **malformed definition** raises :class:`ScenarioError` before any
        simulator is contacted. That is a misuse of the runner, not a run, and
        it must not be swallowed into a result.

        Raises:
            ScenarioError: the definition is invalid, or ``run`` was called twice.
        """
        if self._state is not ScenarioState.CREATED:
            raise ScenarioError(
                "a ScenarioRunner runs exactly once; construct a new one for another run",
                details={"state": self._state.value},
            )

        # A malformed definition is a misuse, not a run: it is rejected here,
        # explicitly and before any simulator is contacted, rather than
        # reported as a FAILED result. `model_copy(update=...)` can produce an
        # unvalidated definition, so the check is repeated rather than trusted.
        self._state = ScenarioState.VALIDATING
        try:
            self._definition = ScenarioDefinition.model_validate(self._definition.model_dump())
            self._resolved = resolve(self._definition)
        except Exception as exc:
            self._state = ScenarioState.FAILED
            raise ScenarioError(
                f"scenario '{self._definition.scenario_id}' is not valid: {exc}",
                details={"scenario_id": self._definition.scenario_id},
            ) from exc
        self._settings = self._apply_scenario_settings(self._settings)

        simulator = self._simulator if self._simulator is not None else self._build_simulator()
        frames: list[ScenarioFrameRecord] = []
        truths: list[Any] = []
        map_name: str | None = None
        simulator_version: str | None = None

        try:
            simulator.open()
            self._spawn_actors(simulator)
            self._state = ScenarioState.READY
            status = simulator.status()
            map_name = status.map_name
            simulator_version = status.server_version

            for index in range(self._definition.frame_count):
                time_s = self._definition.scenario_time(index)
                self._place_actors(simulator, time_s)
                frame = simulator.step()
                truth = simulator.ground_truth()
                self._state = ScenarioState.RUNNING

                counts = self._processor(frame) if self._processor is not None else None
                frames.append(
                    ScenarioFrameRecord(
                        frame_index=index,
                        scenario_time_s=time_s,
                        simulator_frame_id=frame.frame_id,
                        timestamp=frame.timestamp,
                        point_count=frame.point_count,
                        expected_poses=self._expected_poses(time_s),
                        ground_truth_actor_count=len(truth.others()),
                        pipeline=counts,
                    )
                )
                truths.append(truth)
        except Exception as exc:
            logger.warning(
                "scenario run failed",
                extra={"context": {"scenario": self._definition.scenario_id, "error": str(exc)}},
            )
            return self._failed(
                str(exc),
                frames=frames,
                truths=truths,
                map_name=map_name,
                simulator_version=simulator_version,
            )
        finally:
            simulator.close()
            self._handles.clear()

        self._state = ScenarioState.COMPLETED
        return self._result(
            state=ScenarioState.COMPLETED,
            error=None,
            frames=frames,
            truths=truths,
            map_name=map_name,
            simulator_version=simulator_version,
        )

    # -- internals ---------------------------------------------------------
    def _apply_scenario_settings(self, settings: Settings) -> Settings:
        """A settings copy whose simulator section agrees with the definition.

        The scenario is the authority on timestep, seed and map; the session
        reads them from settings, so the two are aligned here rather than
        trusting a user to keep ``.env`` in step with a definition.
        """
        adjusted = settings.model_copy(deep=True)
        adjusted.carla.enabled = True
        adjusted.carla.fixed_delta_seconds = self._definition.fixed_delta_seconds
        adjusted.carla.seed = self._definition.seed
        adjusted.carla.ego_blueprint = self._definition.ego.blueprint
        if self._definition.town is not None:
            adjusted.carla.town = self._definition.town
        return adjusted

    def _build_simulator(self) -> ScenarioSimulator:
        return CarlaSimulationSession(self._settings.carla)

    def _spawn_actors(self, simulator: ScenarioSimulator) -> None:
        assert self._resolved is not None
        for resolved in self._resolved.actors:
            handle = simulator.spawn_ahead_of_ego(
                resolved.blueprint,
                forward_m=resolved.base.forward_m,
                left_m=resolved.base.left_m,
                up_m=resolved.base.up_m,
            )
            self._handles[resolved.actor_id] = handle
            self._records.append(
                ScenarioActorRecord(
                    actor_id=resolved.actor_id,
                    simulator_actor_id=int(handle.id),
                    blueprint=resolved.blueprint,
                )
            )

    def _place_actors(self, simulator: ScenarioSimulator, time_s: float) -> None:
        assert self._resolved is not None
        for resolved in self._resolved.actors:
            pose = self._resolved.expected_pose(resolved.actor_id, time_s)
            simulator.place_ahead_of_ego(
                self._handles[resolved.actor_id],
                forward_m=pose.x,
                left_m=pose.y,
                up_m=pose.z,
            )

    def _expected_poses(self, time_s: float) -> list[ExpectedPose]:
        assert self._resolved is not None
        return [
            ExpectedPose(
                actor_id=resolved.actor_id,
                position=self._resolved.expected_pose(resolved.actor_id, time_s),
            )
            for resolved in self._resolved.actors
        ]

    def _failed(
        self,
        reason: str,
        *,
        frames: list[ScenarioFrameRecord],
        truths: list[Any],
        map_name: str | None = None,
        simulator_version: str | None = None,
    ) -> ScenarioRunResult:
        self._state = ScenarioState.FAILED
        return self._result(
            state=ScenarioState.FAILED,
            error=reason,
            frames=frames,
            truths=truths,
            map_name=map_name,
            simulator_version=simulator_version,
        )

    def _result(
        self,
        *,
        state: ScenarioState,
        error: str | None,
        frames: list[ScenarioFrameRecord],
        truths: list[Any],
        map_name: str | None,
        simulator_version: str | None,
    ) -> ScenarioRunResult:
        assert self._resolved is not None  # resolution precedes every result
        resolved = self._resolved
        return ScenarioRunResult(
            scenario_id=self._definition.scenario_id,
            name=self._definition.name,
            seed=self._definition.seed,
            state=state,
            error=error,
            resolved=resolved,
            actors=list(self._records),
            map_name=map_name,
            # The server's own report when the session obtained one; the
            # installed package otherwise (which, for CARLA 0.9.16, only says
            # "unknown" - it ships no __version__).
            simulator_version=simulator_version or carla_package_version(),
            fixed_delta_seconds=self._definition.fixed_delta_seconds,
            frames=frames,
            ground_truth=truths,
        )


# ---------------------------------------------------------------------------
# convenience
# ---------------------------------------------------------------------------
def run_scenario(
    definition: ScenarioDefinition,
    *,
    settings: Settings | None = None,
    context: ApplicationContext | None = None,
    simulator: ScenarioSimulator | None = None,
    process: bool = True,
) -> ScenarioRunResult:
    """Run a scenario through the pipeline with default wiring.

    Args:
        definition: What to run.
        settings: Application settings; process defaults when omitted.
        context: Pipeline context; built from ``settings`` when omitted and
            ``process`` is true.
        simulator: Simulator to drive; a CARLA session when omitted.
        process: Whether to push frames through the pipeline. False records
            sensor frames and ground truth only.
    """
    resolved_settings = settings if settings is not None else get_settings()
    processor: FrameProcessor | None = None
    if process:
        app = context if context is not None else build_context(resolved_settings)
        processor = pipeline_processor(app)
    runner = ScenarioRunner(
        definition, simulator=simulator, processor=processor, settings=resolved_settings
    )
    return runner.run()


__all__ = [
    "FrameProcessor",
    "ScenarioError",
    "ScenarioRunner",
    "SimulatorUnavailableError",
    "pipeline_processor",
    "resolve",
    "run_scenario",
]
