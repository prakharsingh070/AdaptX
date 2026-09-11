"""Live CARLA smoke test (Phase 9).

The only test in the suite that touches a real simulator. It is deselected by
default (``-m "not carla"`` in ``pyproject.toml``) **and** skips itself when
the package or a server is missing, so it can never turn an absent optional
dependency into a red suite.

Run it deliberately, against a running server::

    ./CarlaUE4.sh -RenderOffScreen        # in the CARLA install
    pytest -m carla

What it proves, and what it does not
------------------------------------
It proves the adapter works against the real API: connecting, applying
deterministic settings, spawning, receiving genuine LiDAR, converting it, and
cleaning up. That is the one thing a stand-in cannot prove.

It proves **nothing about accuracy**. Detections here are unvalidated, and no
figure it prints may be reported as a perception-accuracy result. Comparing
perception against the ground truth it records is Phase 11.
"""

from __future__ import annotations

from itertools import pairwise

import pytest

from adaptx.carla.client import carla_package_available
from adaptx.carla.session import CarlaSimulationSession
from adaptx.config.settings import Settings
from adaptx.core.exceptions import SimulatorUnavailableError
from adaptx.core.lifecycle import build_context
from adaptx.models.common import DataSource
from adaptx.models.system import SimulationState

pytestmark = pytest.mark.carla


def live_settings() -> Settings:
    """Settings pointed at a real server, with a short run.

    The ``carla`` section is read from the environment (``ADAPTX_CARLA__*``)
    and then switched on: host, port and - on Town10HD_Opt - the spawn point
    are properties of the server being tested, not of the test. The catalogue's
    placements are ego-relative and on that map are on the road only from
    spawn point 1, so run with ``ADAPTX_CARLA__EGO_SPAWN_INDEX=1`` there
    (Experiment 011).
    """
    settings = Settings(
        app={"environment": "development", "debug": True},
        logging={"level": "WARNING"},
    )
    settings.carla.enabled = True
    settings.carla.sensor_timeout_s = 20.0
    return settings


def require_live_server() -> Settings:
    """Skip unless a CARLA server actually answers.

    Skipping is the honest outcome here: an absent optional dependency is not
    a failure, and reporting it as one would train everyone to ignore the
    suite.
    """
    if not carla_package_available():
        pytest.skip("the optional CARLA package is not installed")

    settings = live_settings()
    session = CarlaSimulationSession(settings.carla)
    try:
        session.open()
    except SimulatorUnavailableError as exc:
        pytest.skip(f"no CARLA server reachable: {exc.message}")
    finally:
        session.close()
    return settings


class TestLiveCarla:
    def test_a_live_session_opens_and_cleans_up(self) -> None:
        settings = require_live_server()
        session = CarlaSimulationSession(settings.carla)
        session.open()
        assert session.state is SimulationState.READY
        assert session.ego_actor_id is not None
        session.close()
        assert session.state is SimulationState.STOPPED

    def test_a_live_frame_is_labelled_simulation(self) -> None:
        settings = require_live_server()
        with CarlaSimulationSession(settings.carla) as session:
            frame = session.step()
        assert frame.source is DataSource.SIMULATION
        assert frame.point_count > 0

    def test_live_frames_advance_by_exactly_the_timestep(self) -> None:
        settings = require_live_server()
        with CarlaSimulationSession(settings.carla) as session:
            frames = [session.step() for _ in range(3)]

        deltas = [
            (later.timestamp - earlier.timestamp).total_seconds()
            for earlier, later in pairwise(frames)
        ]
        expected = settings.carla.fixed_delta_seconds
        assert deltas == pytest.approx([expected, expected], abs=1e-6)

    def test_live_ground_truth_is_recorded_beside_the_frame(self) -> None:
        settings = require_live_server()
        with CarlaSimulationSession(settings.carla) as session:
            frame = session.step()
            truth = session.ground_truth()

        assert truth.frame_id == frame.frame_id
        assert truth.source is DataSource.SIMULATION
        assert truth.ego_actor_id is not None

    def test_a_live_catalogue_scenario_completes(self) -> None:
        """The whole Phase 9 claim, against a real simulator - now as a scenario.

        Retargeted in Phase 10 when the smoke run became ``vehicle_approach``.
        """
        from adaptx.scenarios import load, run_scenario

        settings = require_live_server()
        result = run_scenario(
            load("vehicle_approach"), settings=settings, context=build_context(settings)
        )

        assert result.completed, result.error
        assert result.frame_count == result.planned_frame_count
        assert result.timestamps_are_monotonic()
        assert all(record.point_count > 0 for record in result.frames)
        assert all(
            record.pipeline is not None and record.pipeline.adaptive_cells > 0
            for record in result.frames
        )

    def test_every_catalogue_scenario_runs_live(self) -> None:
        """Phase 10 against a real simulator: each definition, start to finish.

        The blueprints a scenario asks for must exist on the server; a missing
        one is a real finding about the catalogue, not a test bug, and is
        reported as such through the FAILED result.
        """
        from adaptx.scenarios import load, run_scenario, scenario_ids

        settings = require_live_server()
        for scenario_id in scenario_ids():
            result = run_scenario(load(scenario_id), settings=settings, process=False)
            assert result.completed, f"{scenario_id}: {result.error}"
            assert result.frame_count == result.planned_frame_count
            assert len(result.ground_truth) == result.frame_count
            assert all(truth.others() for truth in result.ground_truth), scenario_id

    def test_the_live_run_leaves_no_actors_behind(self) -> None:
        """A leaked actor persists in the server for every later run."""
        settings = require_live_server()
        session = CarlaSimulationSession(settings.carla)
        session.open()
        world = session._require_world()
        before = len(world.get_actors())
        session.spawn_ahead_of_ego("vehicle.audi.tt", forward_m=25.0)
        session.step()
        session.close()

        remaining = CarlaSimulationSession(settings.carla)
        remaining.open()
        after = len(remaining._require_world().get_actors())
        remaining.close()
        assert after <= before
