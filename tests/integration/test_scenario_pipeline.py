"""Catalogue scenarios through the whole pipeline (Phase 10).

Every catalogue scenario is run against the fake simulator and pushed through
the real Phase 2-8 chain. What that proves: the definitions are valid, the
runner drives the boundary correctly, and every stage consumes what a scenario
produces. What it does not prove: anything about CARLA, or anything about
whether the pipeline's output is *right* - that comparison is Phase 11.

Also holds the structural guarantees for the new package: the definition
models import nothing from the CARLA boundary, and nothing in ``adaptx``
outside that boundary imports the ``carla`` package.
"""

from __future__ import annotations

import json
import os
import pathlib
import socket
import subprocess
import sys

import pytest

from adaptx.carla.session import CarlaSimulationSession
from adaptx.config.settings import Settings
from adaptx.core.lifecycle import build_context
from adaptx.models.common import DataSource, ObjectClass
from adaptx.scenarios import (
    ScenarioDefinition,
    ScenarioState,
    load,
    resolve,
    run_scenario,
    scenario_ids,
)
from adaptx.scenarios.catalogue import CATALOGUE, CATALOGUE_SEED
from tests.fixtures.fake_carla import FakeCarlaModule, FakeWorld
from tests.integration.test_carla_pipeline import sim_settings


def short(definition: ScenarioDefinition, duration_s: float) -> ScenarioDefinition:
    """A validated copy of ``definition`` cut to ``duration_s``.

    Motion segments are clipped to the new end and any that would start after
    it are dropped, then the whole thing is re-validated - ``model_copy``
    alone would bypass the checks that make a definition trustworthy.
    """
    actors = []
    for actor in definition.actors:
        motion = [
            segment.model_copy(update={"stop_s": min(segment.stop_s, duration_s)})
            for segment in actor.motion
            if segment.start_s < duration_s
        ]
        actors.append(actor.model_copy(update={"motion": motion}))
    payload = {**definition.model_dump(), "duration_s": duration_s}
    payload["actors"] = [actor.model_dump() for actor in actors]
    return ScenarioDefinition.model_validate(payload)


def run(definition: ScenarioDefinition, settings: Settings, world: FakeWorld | None = None):  # type: ignore[no-untyped-def]
    simulator = CarlaSimulationSession(
        settings.carla.model_copy(update={"fixed_delta_seconds": definition.fixed_delta_seconds}),
        carla_module=FakeCarlaModule(world if world is not None else FakeWorld()),
    )
    return run_scenario(
        definition, settings=settings, context=build_context(settings), simulator=simulator
    )


@pytest.fixture
def settings() -> Settings:
    return sim_settings()


class TestCatalogue:
    def test_the_catalogue_has_the_four_documented_scenarios(self) -> None:
        assert scenario_ids() == [
            "stationary_vehicle",
            "vehicle_approach",
            "pedestrian_crossing",
            "cyclist_crossing",
        ]

    @pytest.mark.parametrize("scenario_id", list(CATALOGUE))
    def test_every_catalogue_entry_is_a_valid_definition(self, scenario_id: str) -> None:
        definition = load(scenario_id)
        assert definition.scenario_id == scenario_id
        assert definition.description
        assert definition.frame_count >= 2

    @pytest.mark.parametrize("scenario_id", list(CATALOGUE))
    def test_every_catalogue_entry_is_exact_and_carries_its_seed(self, scenario_id: str) -> None:
        """No catalogue scenario is randomised, and each still reports a seed."""
        definition = load(scenario_id)
        assert definition.has_randomised_elements is False
        assert definition.seed == CATALOGUE_SEED
        assert resolve(definition).actors[0].jitter_applied_m == (0.0, 0.0)

    def test_loading_builds_a_fresh_definition_each_time(self) -> None:
        """Data, not a shared object: a caller mutating one cannot affect the next."""
        assert load("vehicle_approach") is not load("vehicle_approach")
        assert load("vehicle_approach") == load("vehicle_approach")

    def test_an_unknown_scenario_lists_what_exists(self) -> None:
        with pytest.raises(KeyError, match="vehicle_approach"):
            load("no_such_scene")

    def test_the_approach_scenario_is_the_phase_9_smoke_scene(self) -> None:
        """Replaced, not lost: the constants moved into a definition."""
        definition = load("vehicle_approach")
        target = definition.actor("approaching")
        assert target.placement.forward_m == 45.0
        assert target.placement.left_m == 3.5
        assert target.motion[0].forward_mps == -8.0

    def test_the_crossing_scenario_has_a_timed_start_and_stop(self) -> None:
        segment = load("pedestrian_crossing").actor("pedestrian").motion[0]
        assert segment.start_s == 1.0
        assert segment.stop_s == 5.0
        assert segment.forward_mps == 0.0
        assert segment.left_mps > 0.0

    def test_the_cyclist_scenario_has_two_classes(self) -> None:
        classes = {actor.object_class for actor in load("cyclist_crossing").actors}
        assert classes == {ObjectClass.VEHICLE, ObjectClass.CYCLIST}


class TestEveryScenarioRunsThroughThePipeline:
    @pytest.mark.parametrize("scenario_id", list(CATALOGUE))
    def test_the_scenario_completes_and_produces_output(
        self, scenario_id: str, settings: Settings
    ) -> None:
        result = run(short(load(scenario_id), 0.5), settings)

        assert result.completed, result.error
        assert result.frame_count == result.planned_frame_count == 10
        assert all(record.point_count > 0 for record in result.frames)
        assert all(
            record.pipeline is not None and record.pipeline.adaptive_cells > 0
            for record in result.frames
        )
        assert any(
            record.pipeline is not None and record.pipeline.detections > 0
            for record in result.frames
        )

    @pytest.mark.parametrize("scenario_id", list(CATALOGUE))
    def test_ground_truth_sees_every_scripted_actor(
        self, scenario_id: str, settings: Settings
    ) -> None:
        definition = short(load(scenario_id), 0.5)
        result = run(definition, settings)
        assert all(
            truth.actor_count == len(definition.actors) + 1  # + ego
            for truth in result.ground_truth
        )

    def test_the_cyclist_scenario_produces_two_tracks(self, settings: Settings) -> None:
        result = run(short(load("cyclist_crossing"), 0.5), settings)
        last = result.frames[-1].pipeline
        assert last is not None
        assert last.detections == 2
        assert last.tracks == 2

    def test_the_approach_scenario_measures_a_velocity(self, settings: Settings) -> None:
        result = run(short(load("vehicle_approach"), 0.5), settings)
        # Trajectories need a measured velocity, which needs two observations.
        assert result.frames[0].pipeline is not None
        assert result.frames[0].pipeline.trajectories == 0
        assert any(
            record.pipeline is not None and record.pipeline.trajectories > 0
            for record in result.frames[2:]
        )

    def test_the_stationary_scenario_settles(self, settings: Settings) -> None:
        """Nothing moves, so the allocation must stop changing once it has settled."""
        result = run(short(load("stationary_vehicle"), 1.0), settings)
        later = [
            record.pipeline.regions_by_level
            for record in result.frames[10:]
            if record.pipeline is not None
        ]
        assert len({json.dumps(levels, sort_keys=True) for levels in later}) == 1


class TestGroundTruthStaysOutOfPerception:
    def test_the_pipeline_output_is_identical_with_and_without_ground_truth(
        self, settings: Settings
    ) -> None:
        """Run the same scenario with the runner and by hand, never reading truth.

        If any stage consulted ground truth, the runner's counts would differ
        from a manual pass that never calls ``ground_truth()``.
        """
        definition = short(load("vehicle_approach"), 0.5)
        result = run(definition, settings)

        manual = build_context(settings)
        simulator = CarlaSimulationSession(
            settings.carla, carla_module=FakeCarlaModule(FakeWorld())
        )
        simulator.open()
        actor = simulator.spawn_ahead_of_ego("vehicle.audi.tt", forward_m=45.0, left_m=3.5)
        counts = []
        for index in range(definition.frame_count):
            t = definition.scenario_time(index)
            simulator.place_ahead_of_ego(actor, forward_m=45.0 - 8.0 * t, left_m=3.5)
            frame = simulator.step()
            processed = manual.preprocessor.run(frame)
            detection = manual.detector.detect(processed.frame)
            tracking = manual.tracking.update(
                detection.objects,
                processed.frame.timestamp,
                frame_id=processed.frame.frame_id,
                sensor_id=processed.frame.sensor_id,
            )
            counts.append((len(detection.objects), len(tracking.tracks)))
        simulator.close()

        assert counts == [
            (r.pipeline.detections, r.pipeline.tracks)  # type: ignore[union-attr]
            for r in result.frames
        ]

    def test_no_pipeline_result_carries_ground_truth(self, settings: Settings) -> None:
        result = run(short(load("cyclist_crossing"), 0.5), settings)
        for record in result.frames:
            dumped = record.pipeline.model_dump_json() if record.pipeline else "{}"
            for key in ('"actor_id"', '"type_id"', '"world_position"', '"ground_truth"'):
                assert key not in dumped


def _leaked(modules: list[str], forbidden: str) -> list[str]:
    """Modules matching ``forbidden`` that importing ``modules`` pulls in.

    Runs in a fresh interpreter so the check starts from an empty module table.
    """
    lines = [
        "import json, sys",
        f"for name in {modules!r}:",
        "    __import__(name)",
        f"prefix = {forbidden!r}",
        "found = [n for n in sys.modules if n == prefix or n.startswith(prefix + '.')]",
        "print(json.dumps(sorted(found)))",
    ]
    completed = subprocess.run(
        [sys.executable, "-c", "\n".join(lines)], capture_output=True, text=True, check=False
    )
    assert completed.returncode == 0, completed.stderr
    return list(json.loads(completed.stdout.strip()))


class TestScenarioBoundary:
    def test_definition_models_import_nothing_from_the_carla_boundary(self) -> None:
        """A definition must be reusable by a simulator that is not CARLA (ADR-046).

        Checked against the source rather than by importing: importing a
        submodule runs the package ``__init__``, which legitimately pulls in
        the runner and through it the boundary. The invariant is about what
        the *definition layer* depends on, and only its own imports show that.
        """
        root = pathlib.Path(__file__).resolve().parents[2] / "src" / "adaptx" / "scenarios"
        offenders = [
            f"{name}: {line.strip()}"
            for name in ("models.py", "catalogue.py")
            for line in (root / name).read_text(encoding="utf-8").splitlines()
            if "adaptx.carla" in line and line.strip().startswith(("import ", "from "))
        ]
        assert offenders == []

    def test_the_scenario_package_never_imports_the_carla_package(self) -> None:
        leaked = _leaked(["adaptx.scenarios"], "carla")
        assert leaked == []

    def test_no_scenario_source_file_imports_carla(self) -> None:
        root = pathlib.Path(__file__).resolve().parents[2] / "src" / "adaptx" / "scenarios"
        offenders = [
            f"{path.name}: {line.strip()}"
            for path in root.glob("*.py")
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip().startswith(("import carla", "from carla import"))
        ]
        assert offenders == []

    def test_a_run_result_is_labelled_simulation_and_never_live(self, settings: Settings) -> None:
        result = run(short(load("stationary_vehicle"), 0.5), settings)
        assert result.source is DataSource.SIMULATION
        assert result.state is ScenarioState.COMPLETED


class TestCommandLine:
    def test_list_prints_the_catalogue(self) -> None:
        completed = subprocess.run(
            [sys.executable, "-m", "adaptx.scenarios", "list"],
            capture_output=True,
            text=True,
            check=False,
        )
        assert completed.returncode == 0, completed.stderr
        for scenario_id in scenario_ids():
            assert scenario_id in completed.stdout

    def test_run_without_a_simulator_fails_honestly(self) -> None:
        """No CARLA reachable: the command must say so and exit non-zero, not
        fake a run. Aimed at a closed port so the outcome is the same whether
        or not a server happens to be running on the default one - the first
        live validation found this test passing a real run and failing."""
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            closed_port = probe.getsockname()[1]  # released on exit; nothing listens
        completed = subprocess.run(
            [
                sys.executable,
                "-m",
                "adaptx.scenarios",
                "run",
                "vehicle_approach",
                "--host",
                "127.0.0.1",
                "--port",
                str(closed_port),
            ],
            capture_output=True,
            text=True,
            check=False,
            env={**os.environ, "ADAPTX_CARLA__TIMEOUT_S": "2"},
        )
        assert completed.returncode != 0
        assert "could not start" in completed.stdout
        assert "CARLA is optional" in completed.stdout

    def test_an_unknown_scenario_exits_with_usage_code(self) -> None:
        completed = subprocess.run(
            [sys.executable, "-m", "adaptx.scenarios", "run", "no_such_scene"],
            capture_output=True,
            text=True,
            check=False,
        )
        assert completed.returncode == 2
        assert "vehicle_approach" in completed.stdout
