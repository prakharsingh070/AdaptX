"""Deterministic scenario framework (Phase 10).

Describes, seeds, runs and records controlled scenes for the ADAPT-X pipeline.

::

    ScenarioDefinition  (data: actors, placement, scripted motion, seed, timing)
        -> resolve(seed)
        -> ScenarioRunner -> simulator (CARLA, via the Phase 9 boundary)
             -> RawPointCloudFrame -> existing pipeline
             -> GroundTruthFrame   -> recorded, never fed to the pipeline
        -> ScenarioRunResult  (raw evidence, no conclusions)

The definition models import nothing from the CARLA boundary and nothing from
the simulator package (ADR-046). The runner is the only module that meets a
simulator, and it does so through :class:`ScenarioSimulator`, a protocol
extracted from the boundary rather than imposed on it (ADR-048).

What this package is not
------------------------
Not an evaluation framework. A run result records what was commanded, what
the simulator reported and what the pipeline counted - and never a figure that
compares them. That is Phase 11.
"""

from adaptx.scenarios.catalogue import CATALOGUE, load, scenario_ids
from adaptx.scenarios.interfaces import ScenarioSimulator
from adaptx.scenarios.models import (
    EgoDefinition,
    MotionSegment,
    Placement,
    ResolvedActor,
    ResolvedScenario,
    ScenarioActor,
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
from adaptx.scenarios.runner import (
    ScenarioError,
    ScenarioRunner,
    pipeline_processor,
    resolve,
    run_scenario,
)

__all__ = [
    "CATALOGUE",
    "EgoDefinition",
    "ExpectedPose",
    "MotionSegment",
    "Placement",
    "ResolvedActor",
    "ResolvedScenario",
    "ScenarioActor",
    "ScenarioActorRecord",
    "ScenarioDefinition",
    "ScenarioError",
    "ScenarioFrameRecord",
    "ScenarioRunResult",
    "ScenarioRunner",
    "ScenarioSimulator",
    "ScenarioState",
    "StageCounts",
    "load",
    "pipeline_processor",
    "resolve",
    "run_scenario",
    "scenario_ids",
]
