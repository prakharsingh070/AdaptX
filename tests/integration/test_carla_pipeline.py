"""CARLA into the existing pipeline, end to end (Phase 9).

The point of Phase 9, asserted directly: a simulated frame enters the
**existing** Phase 2-8 chain and comes out as an adaptive map, and no stage
below the boundary behaves differently because the points came from a
simulator.

The simulator here is :mod:`tests.fixtures.fake_carla`, a stand-in - CARLA
itself is not installed. That limits what these tests prove: they prove the
*integration* (conversion, contracts, temporal semantics, separation), not
anything about CARLA's own behaviour. The live equivalent is in
``test_carla_live.py`` and skips without a server.

The strongest test in this file is
:meth:`TestCoordinatesSurviveTheWholePipeline.test_a_target_on_the_left_is_detected_on_the_left`:
the fake emits points in CARLA's left-handed frame, so if the boundary flip
were dropped the object would be detected on the wrong side of the car while
everything still ran and reported success.
"""

from __future__ import annotations

from itertools import pairwise

import pytest

from adaptx.carla.session import CarlaSimulationSession
from adaptx.carla.smoke import APPROACH_LEFT_M, run_smoke
from adaptx.config.settings import CarlaSettings, LiDARSettings, MapSettings, Settings
from adaptx.core.lifecycle import ApplicationContext, build_context
from adaptx.models.common import DataSource
from tests.fixtures.fake_carla import FakeCarlaModule, FakeWorld

PIPELINE_SETTINGS = LiDARSettings(
    min_points=0,
    max_points=500_000,
    min_range_m=0.0,
    max_range_m=200.0,
    roi_x_min_m=-100.0,
    roi_x_max_m=100.0,
    roi_y_min_m=-100.0,
    roi_y_max_m=100.0,
    roi_z_min_m=-50.0,
    roi_z_max_m=50.0,
    voxel_enabled=True,
    voxel_size_m=0.1,
    ground_enabled=True,
    ground_cell_size_m=1.0,
    noise_enabled=False,
)

MAP_SETTINGS = MapSettings(
    min_x_m=-40.0, max_x_m=40.0, min_y_m=-40.0, max_y_m=40.0, resolution_m=0.5
)


def sim_settings(**overrides: object) -> Settings:
    """Application settings wired for a fake-simulator run."""
    carla = CarlaSettings(
        enabled=True,
        fixed_delta_seconds=0.05,
        lidar_rotation_frequency_hz=20.0,
        sensor_timeout_s=1.0,
        smoke_frames=4,
        **overrides,  # type: ignore[arg-type]
    )
    return Settings(
        app={"environment": "development", "debug": True},
        api={"cors_origins": []},
        logging={"level": "WARNING"},
        carla=carla.model_dump(),
        lidar=PIPELINE_SETTINGS.model_dump(),
        map=MAP_SETTINGS.model_dump(),
        websocket={"telemetry_interval_s": 0.05, "max_connections": 4},
    )


@pytest.fixture
def settings() -> Settings:
    return sim_settings()


@pytest.fixture
def context(settings: Settings) -> ApplicationContext:
    return build_context(settings)


@pytest.fixture
def session(settings: Settings) -> CarlaSimulationSession:
    return CarlaSimulationSession(settings.carla, carla_module=FakeCarlaModule(FakeWorld()))


def _leaked_modules(modules: list[str], forbidden: str) -> list[str]:
    """Modules matching ``forbidden`` that importing ``modules`` pulls in.

    Runs in a subprocess so the check starts from an empty module table without
    disturbing this one - clearing this process's would rebind every ADAPT-X
    class and quietly break unrelated tests further down the session.
    """
    import json
    import subprocess
    import sys

    lines = [
        "import json, sys",
        f"for name in {list(modules)!r}:",
        "    __import__(name)",
        f"prefix = {forbidden!r}",
        "found = [n for n in sys.modules if n == prefix or n.startswith(prefix + '.')]",
        "print(json.dumps(sorted(found)))",
    ]
    completed = subprocess.run(
        [sys.executable, "-c", "\n".join(lines)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    return list(json.loads(completed.stdout.strip()))


class TestSimulatedFrameEntersTheExistingPipeline:
    """Every stage is the same call the LiDAR endpoints already make."""

    def test_a_simulated_frame_reaches_an_adaptive_map(
        self, context: ApplicationContext, session: CarlaSimulationSession
    ) -> None:
        session.open()
        session.spawn_ahead_of_ego("vehicle.audi.tt", forward_m=22.0)
        frame = session.step()

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
        session.close()

        assert processed.frame.point_count > 0
        assert adaptive.tile_count > 0
        assert adaptive.accounting.mapped_point_count > 0

    def test_preprocessing_accepts_the_frame_unchanged(
        self, context: ApplicationContext, session: CarlaSimulationSession
    ) -> None:
        """No CARLA-specific branch anywhere: it is the ordinary ingest path."""
        session.open()
        frame = session.step()
        result = context.preprocessor.run(frame)
        session.close()

        assert result.frame.source is DataSource.SIMULATION
        assert result.metrics.duration_ms >= 0.0

    def test_the_source_stays_simulation_all_the_way_down(
        self, context: ApplicationContext, session: CarlaSimulationSession
    ) -> None:
        """A simulated frame must never become presentable as sensor output."""
        session.open()
        session.spawn_ahead_of_ego("vehicle.audi.tt", forward_m=22.0)
        frame = session.step()
        processed = context.preprocessor.run(frame)
        spatial_map = context.mapping.build(processed.frame)
        summary = processed.output_summary
        session.close()

        for value in (frame, processed.frame, spatial_map, summary):
            assert value.source is DataSource.SIMULATION
            assert value.source is not DataSource.LIVE_SENSOR

    def test_intensity_survives_the_whole_chain(
        self, context: ApplicationContext, session: CarlaSimulationSession
    ) -> None:
        """CARLA reports intensity and every stage slices the first three columns."""
        session.open()
        session.spawn_ahead_of_ego("vehicle.audi.tt", forward_m=22.0)
        frame = session.step()
        processed = context.preprocessor.run(frame)
        spatial_map = context.mapping.build(processed.frame)
        session.close()

        assert frame.has_intensity
        assert processed.frame.has_intensity
        assert spatial_map.accounting.mapped_point_count > 0

    def test_the_ingest_service_accepts_a_simulated_frame(
        self, context: ApplicationContext, session: CarlaSimulationSession
    ) -> None:
        session.open()
        session.spawn_ahead_of_ego("vehicle.audi.tt", forward_m=22.0)
        frame = session.step()
        processed = context.preprocessor.run(frame)
        summary = context.lidar.ingest(processed.frame, pre_validated=True)
        session.close()

        assert summary.source is DataSource.SIMULATION
        assert summary.point_count == processed.frame.point_count


class TestCoordinatesSurviveTheWholePipeline:
    def test_a_target_on_the_left_is_detected_on_the_left(
        self, context: ApplicationContext, session: CarlaSimulationSession
    ) -> None:
        """The end-to-end proof that the handedness flip happened.

        The target is placed to the ego's **left** in ADAPT-X terms. The fake
        emits its returns in CARLA's frame, where left is negative y. If the
        boundary conversion were missing, detection would report the object at
        negative y - on the right - and nothing would raise.
        """
        session.open()
        session.spawn_ahead_of_ego("vehicle.audi.tt", forward_m=18.0, left_m=6.0)
        frame = session.step()
        processed = context.preprocessor.run(frame)
        detection = context.detector.detect(processed.frame)
        session.close()

        assert detection.objects, "the target vehicle should be detected"
        nearest = min(detection.objects, key=lambda obj: obj.distance_m)
        assert nearest.position.y > 0.0, "a target on the left must detect on the left"
        assert nearest.position.x > 0.0, "a target ahead must detect ahead"

    def test_a_target_on_the_right_is_detected_on_the_right(
        self, context: ApplicationContext, session: CarlaSimulationSession
    ) -> None:
        session.open()
        session.spawn_ahead_of_ego("vehicle.audi.tt", forward_m=18.0, left_m=-6.0)
        frame = session.step()
        processed = context.preprocessor.run(frame)
        detection = context.detector.detect(processed.frame)
        session.close()

        assert detection.objects
        nearest = min(detection.objects, key=lambda obj: obj.distance_m)
        assert nearest.position.y < 0.0

    def test_the_detected_range_matches_where_the_target_was_placed(
        self, context: ApplicationContext, session: CarlaSimulationSession
    ) -> None:
        session.open()
        session.spawn_ahead_of_ego("vehicle.audi.tt", forward_m=20.0, left_m=0.0)
        frame = session.step()
        processed = context.preprocessor.run(frame)
        detection = context.detector.detect(processed.frame)
        session.close()

        nearest = min(detection.objects, key=lambda obj: obj.distance_m)
        assert nearest.position.x == pytest.approx(20.0, abs=3.0)


class TestTemporalSemantics:
    def test_tracking_sees_the_simulator_interval(
        self, context: ApplicationContext, session: CarlaSimulationSession
    ) -> None:
        """Velocity comes from these timestamps, so they must be simulator time."""
        session.open()
        target = session.spawn_ahead_of_ego("vehicle.audi.tt", forward_m=30.0)

        timestamps = []
        for index in range(4):
            session.place_ahead_of_ego(target, forward_m=30.0 - 0.4 * index)
            frame = session.step()
            processed = context.preprocessor.run(frame)
            detection = context.detector.detect(processed.frame)
            context.tracking.update(
                detection.objects,
                processed.frame.timestamp,
                frame_id=processed.frame.frame_id,
                sensor_id=processed.frame.sensor_id,
            )
            timestamps.append(processed.frame.timestamp)
        session.close()

        deltas = [(later - earlier).total_seconds() for earlier, later in pairwise(timestamps)]
        assert deltas == pytest.approx([0.05, 0.05, 0.05], abs=1e-9)

    def test_a_moving_target_produces_a_measured_velocity(
        self, context: ApplicationContext, session: CarlaSimulationSession
    ) -> None:
        """Null until two observations, then measured - never assumed."""
        session.open()
        target = session.spawn_ahead_of_ego("vehicle.audi.tt", forward_m=30.0)

        speeds: list[float | None] = []
        for index in range(5):
            session.place_ahead_of_ego(target, forward_m=30.0 - 0.4 * index)
            frame = session.step()
            processed = context.preprocessor.run(frame)
            detection = context.detector.detect(processed.frame)
            tracking = context.tracking.update(
                detection.objects,
                processed.frame.timestamp,
                frame_id=processed.frame.frame_id,
                sensor_id=processed.frame.sensor_id,
            )
            if tracking.tracks:
                speeds.append(tracking.tracks[0].speed_mps)
        session.close()

        assert speeds and speeds[0] is None, "one frame cannot show motion"
        assert any(speed is not None and speed > 0.0 for speed in speeds[1:])

    def test_prediction_receives_real_intervals(
        self, context: ApplicationContext, session: CarlaSimulationSession
    ) -> None:
        session.open()
        target = session.spawn_ahead_of_ego("vehicle.audi.tt", forward_m=30.0)

        prediction = None
        for index in range(4):
            session.place_ahead_of_ego(target, forward_m=30.0 - 0.4 * index)
            frame = session.step()
            processed = context.preprocessor.run(frame)
            detection = context.detector.detect(processed.frame)
            tracking = context.tracking.update(
                detection.objects,
                processed.frame.timestamp,
                frame_id=processed.frame.frame_id,
                sensor_id=processed.frame.sensor_id,
            )
            prediction = context.prediction.predict_from_tracking(tracking)
        session.close()

        assert prediction is not None
        assert prediction.considered_track_count > 0


class TestGroundTruthStaysSeparate:
    """Ground truth exists for evaluation and never enters processing."""

    def test_ground_truth_is_recorded_beside_the_frame(
        self, session: CarlaSimulationSession
    ) -> None:
        session.open()
        session.spawn_ahead_of_ego("vehicle.audi.tt", forward_m=25.0, left_m=3.0)
        frame = session.step()
        truth = session.ground_truth()
        session.close()

        assert truth.frame_id == frame.frame_id
        assert truth.timestamp == frame.timestamp
        assert truth.source is DataSource.SIMULATION
        assert truth.others(), "the target should appear in ground truth"

    def test_ground_truth_positions_use_the_adaptx_frame(
        self, session: CarlaSimulationSession
    ) -> None:
        session.open()
        session.spawn_ahead_of_ego("vehicle.audi.tt", forward_m=25.0, left_m=4.0)
        session.step()
        truth = session.ground_truth()
        session.close()

        nearest = truth.nearest()
        assert nearest is not None
        assert nearest.position.x == pytest.approx(25.0, abs=0.5)
        assert nearest.position.y == pytest.approx(4.0, abs=0.5)

    def test_the_pipeline_runs_identically_whether_ground_truth_is_read(
        self, settings: Settings
    ) -> None:
        """The decisive separation test.

        Running the chain with ground truth fetched every frame, and again
        without fetching it at all, must produce identical output. If any stage
        consulted it, the two runs would diverge.
        """

        def run(read_truth: bool) -> list[tuple[int, int, str]]:
            context = build_context(settings)
            session = CarlaSimulationSession(
                settings.carla, carla_module=FakeCarlaModule(FakeWorld())
            )
            session.open()
            target = session.spawn_ahead_of_ego("vehicle.audi.tt", forward_m=30.0)
            out = []
            for index in range(3):
                session.place_ahead_of_ego(target, forward_m=30.0 - 0.4 * index)
                frame = session.step()
                if read_truth:
                    session.ground_truth()
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
                out.append(
                    (
                        len(detection.objects),
                        adaptive.accounting.total_cell_count,
                        risk.highest_risk_level.value,
                    )
                )
            session.close()
            return out

        assert run(read_truth=True) == run(read_truth=False)

    def test_no_perception_module_imports_ground_truth(self) -> None:
        """Structural guarantee, not just a convention.

        If a perception module ever reaches for the ground-truth contracts, the
        separation has been broken and this fails immediately.

        Run in a subprocess: checking what an import pulls in means starting
        from a clean module table, and clearing this process's would rebind
        every ADAPT-X class and break unrelated tests downstream.
        """
        leaked = _leaked_modules(
            [
                "adaptx.perception.detector",
                "adaptx.tracking.tracker",
                "adaptx.prediction.constant_velocity",
                "adaptx.mapping.controller",
                "adaptx.mapping.adaptive_mapper",
                "adaptx.risk.heuristic",
            ],
            forbidden="adaptx.carla",
        )
        assert leaked == [], f"a perception module reached into the CARLA boundary: {leaked}"


class TestCarlaStaysBehindTheBoundary:
    def test_no_module_outside_the_carla_package_imports_carla(self) -> None:
        """INVARIANT 1: CARLA-specific code is isolated behind the adapter."""
        import pathlib

        root = pathlib.Path(__file__).resolve().parents[2] / "src" / "adaptx"
        offenders = []
        for path in root.rglob("*.py"):
            if "carla" in path.parts or path.parent.name == "carla":
                continue
            text = path.read_text(encoding="utf-8")
            for line in text.splitlines():
                stripped = line.strip()
                if stripped.startswith(("import carla", "from carla import")):
                    offenders.append(f"{path.relative_to(root)}: {stripped}")
        assert offenders == []

    def test_importing_adaptx_never_requires_carla(self) -> None:
        """INVARIANT 6: CARLA is optional for the test suite and the backend."""
        import sys

        assert "carla" not in sys.modules

    def test_the_whole_application_imports_without_the_carla_package(self) -> None:
        """The app may import the *boundary*; it must not require the simulator.

        ``adaptx.api.app`` legitimately pulls in ``adaptx.carla`` - that is the
        adapter, and the status endpoint needs it. What must never be required
        is the third-party ``carla`` distribution, which is optional and absent
        here. Conflating the two would make this test either vacuous or wrong.
        """
        leaked = _leaked_modules(["adaptx.api.app", "adaptx.core.lifecycle"], forbidden="carla")
        assert leaked == [], f"importing the app required the CARLA package: {leaked}"

    def test_the_pipeline_consumes_no_simulator_type(
        self, context: ApplicationContext, session: CarlaSimulationSession
    ) -> None:
        """INVARIANT 2: downstream sees ADAPT-X contracts, not CARLA objects."""
        from adaptx.models.point_cloud import RawPointCloudFrame

        session.open()
        frame = session.step()
        session.close()
        assert isinstance(frame, RawPointCloudFrame)
        assert type(frame).__module__.startswith("adaptx.models")


class TestSmokeRunner:
    """The scripted scenario, driven against the fake simulator."""

    def test_the_smoke_run_completes_every_frame(self, settings: Settings) -> None:
        result = run_smoke(
            settings,
            context=build_context(settings),
            session=CarlaSimulationSession(
                settings.carla, carla_module=FakeCarlaModule(FakeWorld())
            ),
            frames=4,
        )
        assert result.frame_count == 4
        assert result.timestamps_are_monotonic()

    def test_the_smoke_run_produces_perception_output(self, settings: Settings) -> None:
        result = run_smoke(
            settings,
            context=build_context(settings),
            session=CarlaSimulationSession(
                settings.carla, carla_module=FakeCarlaModule(FakeWorld())
            ),
            frames=4,
        )
        assert all(record.point_count > 0 for record in result.frames)
        assert any(record.detections > 0 for record in result.frames)
        assert all(record.adaptive_cells > 0 for record in result.frames)

    def test_the_smoke_run_records_ground_truth_for_every_frame(self, settings: Settings) -> None:
        result = run_smoke(
            settings,
            context=build_context(settings),
            session=CarlaSimulationSession(
                settings.carla, carla_module=FakeCarlaModule(FakeWorld())
            ),
            frames=3,
        )
        assert len(result.ground_truth) == 3
        assert all(truth.others() for truth in result.ground_truth)

    def test_the_target_approaches_over_the_run(self, settings: Settings) -> None:
        """Ground truth should show the scripted approach, monotonically."""
        result = run_smoke(
            settings,
            context=build_context(settings),
            session=CarlaSimulationSession(
                settings.carla, carla_module=FakeCarlaModule(FakeWorld())
            ),
            frames=5,
        )
        distances = [
            record.nearest_ground_truth_m
            for record in result.frames
            if record.nearest_ground_truth_m is not None
        ]
        assert len(distances) >= 2
        assert distances[-1] < distances[0]
        assert APPROACH_LEFT_M > 0.0

    def test_the_run_cleans_up_after_itself(self, settings: Settings) -> None:
        world = FakeWorld()
        run_smoke(
            settings,
            context=build_context(settings),
            session=CarlaSimulationSession(settings.carla, carla_module=FakeCarlaModule(world)),
            frames=3,
        )
        assert world.actors == {}, "the smoke run must leave no actors behind"

    def test_two_runs_produce_identical_records(self, settings: Settings) -> None:
        def run() -> list[tuple[int, int, int]]:
            result = run_smoke(
                settings,
                context=build_context(settings),
                session=CarlaSimulationSession(
                    settings.carla, carla_module=FakeCarlaModule(FakeWorld())
                ),
                frames=3,
            )
            return [
                (record.frame_id, record.point_count, record.adaptive_cells)
                for record in result.frames
            ]

        assert run() == run()
