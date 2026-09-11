"""Service-layer tests: metrics, LiDAR ingest and status aggregation."""

from __future__ import annotations

import time

import pytest

from adaptx.config.settings import LiDARSettings, Settings
from adaptx.core.exceptions import InvalidPointCloudError
from adaptx.core.lifecycle import ApplicationContext, build_context
from adaptx.models.common import DataSource
from adaptx.models.system import (
    ComponentReadiness,
    ImplementationStatus,
    LiDARStatus,
    SystemState,
)
from adaptx.perception.lidar import FrameValidationProcessor
from adaptx.services.lidar_service import LiDARIngestService
from adaptx.services.metrics_service import MetricsService
from tests.fixtures.point_clouds import make_frame


class TestMetricsService:
    def test_reports_nothing_measured_before_any_frame(self) -> None:
        metrics = MetricsService().snapshot()

        assert metrics.fps is None
        assert metrics.latency_ms is None
        assert metrics.point_count is None
        assert metrics.sample_count == 0
        assert any("no frames ingested" in note for note in metrics.unavailable)

    def test_gpu_is_always_reported_unavailable(self) -> None:
        """No GPU monitoring exists in Phase 1; the value must never be invented."""
        metrics = MetricsService().snapshot()
        assert metrics.gpu_percent is None
        assert any("gpu_percent" in note for note in metrics.unavailable)

    def test_single_frame_gives_latency_but_not_fps(self) -> None:
        service = MetricsService()
        service.record_frame(point_count=100, processing_time_s=0.002)
        metrics = service.snapshot()

        assert metrics.point_count == 100
        assert metrics.latency_ms == pytest.approx(2.0)
        assert metrics.processing_time_ms == pytest.approx(2.0)
        assert metrics.fps is None
        assert any("at least two" in note for note in metrics.unavailable)

    def test_fps_is_measured_once_two_frames_exist(self) -> None:
        """Two frames, however close together, must yield a measured fps.

        This failed on Python 3.12 on Windows, where ``time.monotonic()`` ticks
        every ~15.6 ms: back-to-back frames shared a timestamp, elapsed was
        zero, and fps was reported unmeasured. The clock is now
        ``perf_counter``. Frames are recorded with no delay on purpose - a sleep
        would hide the defect on any clock.
        """
        service = MetricsService()
        service.record_frame(point_count=1, processing_time_s=0.001)
        service.record_frame(point_count=2, processing_time_s=0.003)
        metrics = service.snapshot()

        assert metrics.fps is not None
        assert metrics.fps > 0
        assert metrics.sample_count == 2
        assert metrics.latency_ms == pytest.approx(2.0)

    def test_fps_survives_a_coarse_system_clock(self) -> None:
        """Regression guard for the 3.12 defect, independent of the host clock."""
        service = MetricsService()
        for index in range(5):
            service.record_frame(point_count=index, processing_time_s=0.001)
        assert service.snapshot().fps is not None

    def test_window_is_bounded(self) -> None:
        service = MetricsService(window=3)
        for index in range(10):
            service.record_frame(point_count=index, processing_time_s=0.001)
        assert service.snapshot().sample_count == 3

    def test_reset_clears_the_window(self) -> None:
        service = MetricsService()
        service.record_frame(point_count=5, processing_time_s=0.001)
        service.reset()
        assert service.snapshot().sample_count == 0


class TestLiDARIngestService:
    @staticmethod
    def _service(**overrides: float) -> LiDARIngestService:
        settings = LiDARSettings(min_points=1, max_points=1000, **overrides)  # type: ignore[arg-type]
        return LiDARIngestService(
            settings=settings,
            processor=FrameValidationProcessor(settings),
            metrics=MetricsService(),
        )

    def test_disconnected_before_any_frame(self) -> None:
        status = self._service().status()
        assert status.status is LiDARStatus.DISCONNECTED
        assert status.source is DataSource.UNAVAILABLE
        assert status.frames_received == 0

    def test_ingest_returns_a_summary_and_counts_the_frame(self) -> None:
        service = self._service()
        summary = service.ingest(make_frame(25, frame_id=4))

        assert summary.frame_id == 4
        assert summary.point_count == 25
        assert service.frames_received == 1
        assert service.last_summary is not None

    def test_synthetic_frames_report_simulated_not_connected(self) -> None:
        service = self._service()
        service.ingest(make_frame(10, source=DataSource.SYNTHETIC_TEST))
        status = service.status()

        assert status.status is LiDARStatus.SIMULATED
        assert "not sensor data" in status.detail

    def test_live_frames_report_connected(self) -> None:
        service = self._service()
        service.ingest(make_frame(10, source=DataSource.LIVE_SENSOR))
        assert service.status().status is LiDARStatus.CONNECTED

    def test_replayed_frames_report_simulated(self) -> None:
        service = self._service()
        service.ingest(make_frame(10, source=DataSource.REPLAY))
        assert service.status().status is LiDARStatus.SIMULATED

    def test_stale_frames_report_disconnected(self) -> None:
        service = self._service(frame_stale_after_s=0.001)
        service.ingest(make_frame(10))
        time.sleep(0.02)  # exceed the 1 ms staleness limit by a wide margin
        status = service.status()

        assert status.status is LiDARStatus.DISCONNECTED
        assert "staleness limit" in status.detail

    def test_pre_validated_frames_skip_the_input_limits(self) -> None:
        """A frame preprocessing reduced to zero is a valid observation.

        min_points is a limit on the raw input, not on the filtered output, so
        re-applying it after preprocessing would reject a legitimate result.
        """
        import numpy as np

        from adaptx.models.point_cloud import PointCloudFrame

        service = self._service()
        empty = PointCloudFrame(
            frame_id=0, sensor_id="s", points=np.empty((0, 3), dtype=np.float64)
        )

        with pytest.raises(InvalidPointCloudError):
            service.ingest(empty)

        summary = service.ingest(empty, pre_validated=True)
        assert summary.point_count == 0
        assert service.frames_received == 1

    def test_upstream_duration_is_added_to_the_measured_time(self) -> None:
        metrics = MetricsService()
        settings = LiDARSettings(min_points=0, max_points=1000)
        service = LiDARIngestService(
            settings=settings,
            processor=FrameValidationProcessor(settings),
            metrics=metrics,
        )

        service.ingest(make_frame(5), pre_validated=True, upstream_duration_s=0.25)
        assert metrics.snapshot().processing_time_ms >= 250.0

    def test_oversized_frames_are_rejected_and_not_counted(self) -> None:
        service = self._service()
        with pytest.raises(InvalidPointCloudError):
            service.ingest(make_frame(2000))
        assert service.frames_received == 0


class TestSystemService:
    @pytest.fixture
    def context(self) -> ApplicationContext:
        return build_context(Settings(carla={"enabled": False}, logging={"level": "WARNING"}))

    def test_running_when_required_components_are_ready(self, context: ApplicationContext) -> None:
        status = context.system.status()
        assert status.state is SystemState.RUNNING
        assert status.uptime_s >= 0.0

    def test_no_component_claims_to_be_finished_while_it_is_a_baseline(
        self, context: ApplicationContext
    ) -> None:
        components = {c.name: c for c in context.system.status().components}

        for name in ("mapping", "prediction", "tracking", "perception", "risk"):
            assert components[name].implementation is ImplementationStatus.PARTIAL
            assert components[name].implementation is not ImplementationStatus.IMPLEMENTED
            assert components[name].required is False

    def test_mapping_is_partial_because_it_is_a_fixed_resolution_baseline(
        self, context: ApplicationContext
    ) -> None:
        """Phase 6 implemented a fixed-resolution mapper; it is still fixed.

        Retargeted in Phase 6 (mapping was asserted PLANNED) and again in
        Phase 8, where the literal sentence "ADAPTIVE RESOLUTION IS NOT
        IMPLEMENTED" stopped being true: a controller now exists as its own
        component. The guarantee underneath is unchanged and is what is
        asserted here - *this* component is the uniform-resolution baseline,
        it never reports IMPLEMENTED, it never claims to choose its own
        resolution, and it stays available to be measured against.
        """
        components = {c.name: c for c in context.system.status().components}
        mapping = components["mapping"]

        assert mapping.implementation is ImplementationStatus.PARTIAL
        assert "fixed-resolution" in mapping.detail
        assert "never chooses one" in mapping.detail
        assert "retained unchanged for comparison" in mapping.detail
        assert "frame-local" in mapping.detail

    def test_adaptive_resolution_is_partial_and_never_claims_a_probability(
        self, context: ApplicationContext
    ) -> None:
        """Phase 8 added a controller, and it is a heuristic baseline too.

        The replacement guarantee for the sentence removed above: the adaptive
        component exists, is never IMPLEMENTED, and states plainly that its
        priority is not a collision probability.
        """
        components = {c.name: c for c in context.system.status().components}
        adaptive = components["adaptive_resolution"]

        assert adaptive.implementation is ImplementationStatus.PARTIAL
        assert adaptive.implementation is not ImplementationStatus.IMPLEMENTED
        assert adaptive.phase == 8
        assert "NOT A PROBABILITY OF COLLISION" in adaptive.detail
        assert "never validated against labelled data" in adaptive.detail

    def test_carla_is_partial_and_admits_no_live_run(self, context: ApplicationContext) -> None:
        """Phase 9 added the simulation boundary, and it is unexercised live.

        An adapter existing is not the same as an adapter having been run
        against a real server. The component must say which, must never
        report IMPLEMENTED, and must point at the live-state field rather than
        duplicate it.
        """
        components = {c.name: c for c in context.system.status().components}
        carla = components["carla"]

        assert carla.implementation is ImplementationStatus.PARTIAL
        assert carla.implementation is not ImplementationStatus.IMPLEMENTED
        assert carla.phase == 9
        assert carla.required is False
        assert "NO LIVE CARLA RUN HAS BEEN EXECUTED" in carla.detail
        assert "never reaches detection" in carla.detail
        assert "DATA SOURCE" in carla.detail

    def test_scenarios_are_partial_and_carry_no_evaluation_claim(
        self, context: ApplicationContext
    ) -> None:
        """Phase 10 added the scenario framework; it evaluates nothing.

        A framework that records ground truth is one step from claiming
        accuracy. The component must say plainly that the step has not been
        taken, and must never report IMPLEMENTED.
        """
        components = {c.name: c for c in context.system.status().components}
        scenarios = components["scenarios"]

        assert scenarios.implementation is ImplementationStatus.PARTIAL
        assert scenarios.implementation is not ImplementationStatus.IMPLEMENTED
        assert scenarios.phase == 10
        assert scenarios.required is False
        assert "NO accuracy" in scenarios.detail
        assert "NO pipeline stage" in scenarios.detail
        assert "No live CARLA run" in scenarios.detail

    def test_prediction_is_partial_because_it_is_a_baseline(
        self, context: ApplicationContext
    ) -> None:
        """Phase 5 implemented a constant-velocity predictor, not a finished one.

        Retargeted in Phase 5: prediction was previously asserted PLANNED here.
        It is now implemented as a baseline, so the guarantee that matters is
        that it never reports IMPLEMENTED and never hides what it cannot do.
        """
        components = {c.name: c for c in context.system.status().components}
        prediction = components["prediction"]

        assert prediction.implementation is ImplementationStatus.PARTIAL
        assert prediction.required is False
        assert "constant-velocity baseline" in prediction.detail
        assert "heuristic" in prediction.detail
        assert "unmeasured" in prediction.detail

    def test_tracking_is_partial_because_it_is_a_baseline(
        self, context: ApplicationContext
    ) -> None:
        """Phase 4 implemented a geometric tracker, not a finished one."""
        components = {c.name: c for c in context.system.status().components}
        tracking = components["tracking"]

        assert tracking.implementation is ImplementationStatus.PARTIAL
        assert tracking.readiness is ComponentReadiness.READY
        assert "baseline" in tracking.detail

    def test_detection_is_partial_because_it_is_a_baseline(
        self, context: ApplicationContext
    ) -> None:
        """Phase 3 implemented a geometric detector, not a finished one."""
        components = {c.name: c for c in context.system.status().components}
        perception = components["perception"]

        assert perception.implementation is ImplementationStatus.PARTIAL
        assert perception.readiness is ComponentReadiness.READY
        assert "baseline" in perception.detail

    def test_risk_is_partial_because_it_is_a_heuristic_baseline(
        self, context: ApplicationContext
    ) -> None:
        """Phase 7 implemented a heuristic risk engine, not a validated one.

        Retargeted in Phase 7: risk was previously NOT_READY because only a
        test baseline existed. An engine now runs, so the guarantee that
        matters is that a heuristic never reports IMPLEMENTED and never hides
        that its score is not a collision probability.
        """
        components = {c.name: c for c in context.system.status().components}
        risk = components["risk"]

        assert risk.implementation is ImplementationStatus.PARTIAL
        assert risk.implementation is not ImplementationStatus.IMPLEMENTED
        assert risk.readiness is ComponentReadiness.READY
        assert "baseline" in risk.detail
        assert "heuristic" in risk.detail
        assert "NOT A PROBABILITY OF COLLISION" in risk.detail
        assert "RISK DOES NOT DECIDE SPATIAL RESOLUTION" in risk.detail

    def test_lidar_ingest_is_partial_and_required(self, context: ApplicationContext) -> None:
        components = {c.name: c for c in context.system.status().components}
        ingest = components["lidar_ingest"]

        assert ingest.implementation is ImplementationStatus.PARTIAL
        assert ingest.readiness is ComponentReadiness.READY
        assert ingest.required is True

    def test_enabled_but_unconnected_carla_degrades_the_system(self) -> None:
        context = build_context(
            Settings(
                carla={"enabled": True, "host": "127.0.0.1", "port": 1},
                logging={"level": "WARNING"},
            )
        )
        context.carla.connect()  # fails, without raising
        assert context.system.status().state is SystemState.DEGRADED

    def test_stale_lidar_after_frames_degrades_the_system(self) -> None:
        context = build_context(
            Settings(
                carla={"enabled": False},
                lidar={"frame_stale_after_s": 0.001},
                logging={"level": "WARNING"},
            )
        )
        context.lidar.ingest(make_frame(10))
        time.sleep(0.02)  # exceed the 1 ms staleness limit by a wide margin
        assert context.system.status().state is SystemState.DEGRADED

    def test_components_are_copies(self, context: ApplicationContext) -> None:
        """Callers must not be able to mutate the declared component table."""
        first = context.system.components()
        first[0].detail = "mutated"
        assert context.system.components()[0].detail != "mutated"
