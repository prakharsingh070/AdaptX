"""Scene snapshot contract and service (Phase 12).

The snapshot bundles what the pipeline produced; the service keeps the
latest one. Neither computes anything, and neither can carry ground truth.
"""

from __future__ import annotations

import numpy as np
import pytest
from pydantic import ValidationError

from adaptx.config.settings import Settings
from adaptx.models.common import DataSource
from adaptx.models.risk import RiskLevel
from adaptx.models.scene import PointStage, SceneSnapshot, sample_points
from adaptx.scenarios.catalogue import load
from adaptx.scenarios.publish import ScenePublisher
from adaptx.scenarios.result import PipelineFrameOutputs
from adaptx.services.scene_service import SceneService, build_snapshot
from tests.fixtures.evaluation import ActorSpec, FrameSpec, RiskSpec, RunBuilder, TrackSpec
from tests.integration.test_scenario_pipeline import run, short, sim_settings


class TestPointSample:
    def test_a_small_cloud_is_kept_whole(self) -> None:
        points = np.arange(30, dtype=np.float64).reshape(10, 3)
        sample = sample_points(points, max_points=100)
        assert sample.total_count == 10 and sample.sample_count == 10
        assert not sample.is_downsampled and sample.stride == 1

    def test_a_large_cloud_is_strided_deterministically_and_rounded(self) -> None:
        points = np.random.default_rng(1).normal(size=(27_000, 4)) * 10
        first = sample_points(points, max_points=6000)
        second = sample_points(points, max_points=6000)
        assert first.is_downsampled and first.sample_count <= 6000
        assert first.stride == 5 and first.sample_count == 5400
        assert first == second
        assert all(round(v, 2) == v for row in first.xyz for v in row)
        assert first.xyz[1] == [round(float(v), 2) for v in points[5, :3]]

    def test_an_empty_cloud_is_an_empty_sample(self) -> None:
        sample = sample_points(np.zeros((0, 3)), max_points=10)
        assert sample.total_count == 0 and sample.xyz == []

    def test_the_sample_carries_its_stage(self) -> None:
        sample = sample_points(np.zeros((3, 3)), stage=PointStage.RAW)
        assert sample.stage is PointStage.RAW

    def test_max_points_must_be_positive(self) -> None:
        with pytest.raises(ValueError, match="at least 1"):
            sample_points(np.zeros((3, 3)), max_points=0)


def synthetic_outputs() -> PipelineFrameOutputs:
    record = (
        RunBuilder({"target": 5})
        .frame(
            FrameSpec(
                tracks=[TrackSpec(0, 10.0, 0.0), TrackSpec(1, 12.0, 2.0, velocity=None)],
                actors=[ActorSpec(5, 10.0, 0.0)],
                risk=[RiskSpec(0, RiskLevel.HIGH, 0.7), RiskSpec(1, RiskLevel.UNKNOWN, None)],
                trajectories={0: [(0.0, 10.0, 0.0), (0.25, 9.0, 0.0)]},
            )
        )
        .build()
    )
    outputs = record.frames[0].outputs
    assert outputs is not None
    return outputs


def snapshot_from(outputs: PipelineFrameOutputs, **overrides: object) -> SceneSnapshot:
    fields: dict[str, object] = {
        "frame_id": 7,
        "sensor_id": "lidar",
        "source": DataSource.SIMULATION,
        "frame_timestamp": outputs.tracking.timestamp,
        "origin": "api",
        "points": np.zeros((5, 3)),
        "point_stage": PointStage.PROCESSED,
        "detection": outputs.detection,
        "tracking": outputs.tracking,
        "prediction": outputs.prediction,
        "risk": outputs.risk,
        "plan": outputs.plan,
        "adaptive_map": outputs.adaptive_map,
        "fixed_map": outputs.fixed_map,
        "comparison": outputs.comparison,
        "processing_ms": 1.0,
    }
    fields.update(overrides)
    return build_snapshot(**fields)  # type: ignore[arg-type]


class TestSnapshot:
    def test_build_snapshot_keeps_every_output_as_produced(self) -> None:
        outputs = synthetic_outputs()
        snapshot = snapshot_from(outputs)
        assert snapshot.tracks == outputs.tracking.tracks
        assert snapshot.trajectories == outputs.prediction.trajectories
        assert snapshot.assessments == outputs.risk.assessments
        assert snapshot.tiles == outputs.plan.decisions
        assert snapshot.adaptive_map == outputs.adaptive_map
        assert snapshot.points is not None and snapshot.points.sample_count == 5
        assert snapshot.counts_by_risk_level() == {
            "low": 0,
            "medium": 0,
            "high": 1,
            "critical": 0,
            "unknown": 1,
        }

    def test_unknown_risk_survives_the_round_trip_as_unknown(self) -> None:
        snapshot = snapshot_from(synthetic_outputs())
        rebuilt = SceneSnapshot.model_validate_json(snapshot.model_dump_json())
        unknown = next(a for a in rebuilt.assessments if a.track_id == 1)
        assert unknown.risk_level is RiskLevel.UNKNOWN and unknown.risk_score is None
        null_velocity = next(t for t in rebuilt.tracks if t.track_id == 1)
        assert null_velocity.velocity is None

    def test_a_snapshot_has_no_place_for_ground_truth(self) -> None:
        payload = snapshot_from(synthetic_outputs()).model_dump(mode="json")
        assert "ground_truth" not in payload
        payload["ground_truth"] = {"actors": []}
        with pytest.raises(ValidationError):
            SceneSnapshot.model_validate(payload)

    def test_a_trajectory_or_assessment_without_its_track_is_refused(self) -> None:
        payload = snapshot_from(synthetic_outputs()).model_dump(mode="json")
        payload["tracks"] = payload["tracks"][:1]
        with pytest.raises(ValidationError, match="has no track"):
            SceneSnapshot.model_validate(payload)

    def test_a_scenario_origin_must_name_its_scenario(self) -> None:
        with pytest.raises(ValidationError, match="must name its scenario"):
            snapshot_from(synthetic_outputs(), origin="scenario:x")


class TestService:
    def test_publish_replaces_the_latest_and_bumps_the_sequence(self) -> None:
        service = SceneService()
        assert service.latest() == (0, None)
        first = snapshot_from(synthetic_outputs())
        assert service.publish(first) == 1
        second = snapshot_from(synthetic_outputs(), frame_id=8)
        assert service.publish(second) == 2
        sequence, latest = service.latest()
        assert sequence == 2 and latest is second
        assert service.summary()["frame_id"] == 8

    def test_reset_forgets_the_snapshot_but_keeps_counting(self) -> None:
        service = SceneService()
        service.publish(snapshot_from(synthetic_outputs()))
        service.reset()
        assert service.latest() == (2, None)


class TestPublisher:
    def test_the_publisher_builds_a_snapshot_from_a_real_pipeline_frame_and_no_ground_truth(
        self,
    ) -> None:
        """Round trip through the actual chain against the fake simulator."""
        settings = sim_settings()
        record = run(short(load("stationary_vehicle"), 0.2), settings)
        seen: list[SceneSnapshot] = []

        class Capture(ScenePublisher):
            def __call__(self, frame, index, time_s, produced):  # type: ignore[no-untyped-def]
                assert isinstance(produced, PipelineFrameOutputs)
                seen.append(
                    build_snapshot(
                        frame_id=frame.frame_id,
                        sensor_id=frame.sensor_id,
                        source=frame.source,
                        frame_timestamp=frame.timestamp,
                        origin="scenario:stationary_vehicle",
                        points=frame.points,
                        point_stage=PointStage.RAW,
                        detection=produced.detection,
                        tracking=produced.tracking,
                        prediction=produced.prediction,
                        risk=produced.risk,
                        plan=produced.plan,
                        adaptive_map=produced.adaptive_map,
                        fixed_map=produced.fixed_map,
                        comparison=produced.comparison,
                        processing_ms=produced.processing_ms,
                        scenario_id="stationary_vehicle",
                        frame_index=index,
                        scenario_time_s=time_s,
                    )
                )

        capture = Capture("http://127.0.0.1:1", scenario_id="stationary_vehicle")
        assert record.frame_count >= 2
        run(short(load("stationary_vehicle"), 0.2), settings, observer=capture)
        assert len(seen) == record.frame_count
        assert all(s.origin == "scenario:stationary_vehicle" for s in seen)
        assert all(s.points is not None and s.points.stage is PointStage.RAW for s in seen)
        assert all("ground_truth" not in s.model_dump() for s in seen)

    def test_a_dead_backend_does_not_fail_the_run(self) -> None:
        settings = sim_settings()
        publisher = ScenePublisher(
            "http://127.0.0.1:9", scenario_id="stationary_vehicle", timeout_s=0.2
        )
        result = run(short(load("stationary_vehicle"), 0.3), settings, observer=publisher)
        assert result.completed
        assert publisher.published == 0
        assert publisher.failed >= 1
        assert publisher.disabled  # gave up after repeated failures, run unaffected

    def test_the_publisher_ignores_frames_without_outputs(self) -> None:
        publisher = ScenePublisher("http://127.0.0.1:9", scenario_id="s")
        publisher(None, 0, 0.0, None)  # type: ignore[arg-type]
        assert publisher.published == 0 and publisher.failed == 0


class TestSettings:
    def test_dashboard_settings_have_safe_defaults(self) -> None:
        settings = Settings(carla={"enabled": False})
        assert settings.dashboard.scene_max_points == 6000
        assert str(settings.dashboard.report_dir) == "reports"
        assert str(settings.dashboard.run_dir) == "runs"
