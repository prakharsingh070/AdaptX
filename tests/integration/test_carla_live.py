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
from adaptx.carla.smoke import run_smoke
from adaptx.config.settings import CarlaSettings, Settings
from adaptx.core.exceptions import SimulatorUnavailableError
from adaptx.core.lifecycle import build_context
from adaptx.models.common import DataSource
from adaptx.models.system import SimulationState

pytestmark = pytest.mark.carla


def live_settings() -> Settings:
    """Settings pointed at a real server, with a short run."""
    carla = CarlaSettings(enabled=True, smoke_frames=6, sensor_timeout_s=20.0)
    return Settings(
        app={"environment": "development", "debug": True},
        logging={"level": "WARNING"},
        carla=carla.model_dump(),
    )


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

    def test_the_live_smoke_run_completes(self) -> None:
        """The whole Phase 9 claim, against a real simulator."""
        settings = require_live_server()
        result = run_smoke(settings, context=build_context(settings), frames=6)

        assert result.frame_count == 6
        assert result.timestamps_are_monotonic()
        assert all(record.point_count > 0 for record in result.frames)
        assert all(record.adaptive_cells > 0 for record in result.frames)

    def test_the_live_run_leaves_no_actors_behind(self) -> None:
        """A leaked actor persists in the server for every later run."""
        settings = require_live_server()
        session = CarlaSimulationSession(settings.carla)
        session.open()
        world = session._require_world()
        before = len(world.get_actors())
        session.spawn_ahead_of_ego(settings.carla.target_blueprint, forward_m=25.0)
        session.step()
        session.close()

        remaining = CarlaSimulationSession(settings.carla)
        remaining.open()
        after = len(remaining._require_world().get_actors())
        remaining.close()
        assert after <= before
