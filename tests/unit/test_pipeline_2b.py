"""Full preprocessing chain tests with the Phase 2B stages enabled.

Covers the seven-stage pipeline end to end: stage ordering, the accounting
that must partition the input, metadata preservation and determinism.
"""

from __future__ import annotations

import itertools

import numpy as np
import pytest

from adaptx.config.settings import LiDARSettings
from adaptx.models.common import CoordinateFrame, DataSource
from adaptx.models.point_cloud import PointCloudFrame, RawPointCloudFrame
from adaptx.models.processing import ProcessingStage
from adaptx.perception.pipeline import LiDARProcessingPipeline

#: Permissive 2A bounds so only the 2B stages under test constrain anything.
BASE = LiDARSettings(
    min_points=0,
    max_points=100_000,
    min_range_m=0.0,
    max_range_m=10_000.0,
    roi_x_min_m=-1000.0,
    roi_x_max_m=1000.0,
    roi_y_min_m=-1000.0,
    roi_y_max_m=1000.0,
    roi_z_min_m=-1000.0,
    roi_z_max_m=1000.0,
)


def pipeline(**overrides: object) -> LiDARProcessingPipeline:
    return LiDARProcessingPipeline(BASE.model_copy(update=dict(overrides)))


def raw(points: list[list[float]], **kwargs: object) -> RawPointCloudFrame:
    defaults: dict[str, object] = {
        "frame_id": 11,
        "sensor_id": "test_lidar",
        "source": DataSource.SYNTHETIC_TEST,
        "coordinate_frame": CoordinateFrame.LIDAR,
    }
    defaults.update(kwargs)
    return RawPointCloudFrame.from_sequence(points, **defaults)


def scene() -> list[list[float]]:
    """A small deterministic scene: road, an object on it, and one stray point."""
    road = [[float(x) * 0.5, float(y) * 0.5, -1.8] for x in range(6) for y in range(4)]
    obstacle = [[1.0 + i * 0.05, 1.0, -0.5] for i in range(8)]
    stray = [[40.0, 40.0, 3.0]]
    return road + obstacle + stray


class TestStagesAreOptIn:
    def test_all_2b_stages_are_off_by_default(self) -> None:
        settings = LiDARSettings()
        assert settings.voxel_enabled is False
        assert settings.ground_enabled is False
        assert settings.noise_enabled is False

    def test_default_pipeline_reports_only_the_2a_stages(self) -> None:
        result = pipeline().run(raw(scene()))
        assert [s.stage for s in result.metrics.stages] == [
            ProcessingStage.VALIDATION,
            ProcessingStage.INVALID_REMOVAL,
            ProcessingStage.ROI_FILTER,
            ProcessingStage.RANGE_FILTER,
        ]

    def test_disabled_stages_report_zero_counts(self) -> None:
        metrics = pipeline().run(raw(scene())).metrics
        assert metrics.voxel_reduced_count == 0
        assert metrics.ground_point_count == 0
        assert metrics.noise_removed_count == 0

    def test_ground_frame_is_absent_unless_segmentation_ran(self) -> None:
        assert pipeline().run(raw(scene())).ground_frame is None
        assert pipeline().run(raw(scene())).ground_summary is None

    @pytest.mark.parametrize(
        ("flag", "stage"),
        [
            ("voxel_enabled", ProcessingStage.VOXEL_DOWNSAMPLE),
            ("ground_enabled", ProcessingStage.GROUND_SEGMENTATION),
            ("noise_enabled", ProcessingStage.NOISE_FILTER),
        ],
    )
    def test_each_stage_can_be_enabled_alone(self, flag: str, stage: ProcessingStage) -> None:
        result = pipeline(**{flag: True}).run(raw(scene()))
        assert stage in [s.stage for s in result.metrics.stages]


class TestFullChain:
    @staticmethod
    def _full() -> LiDARProcessingPipeline:
        return pipeline(
            voxel_enabled=True,
            voxel_size_m=0.2,
            ground_enabled=True,
            ground_cell_size_m=1.0,
            ground_height_tolerance_m=0.2,
            noise_enabled=True,
            noise_cell_size_m=1.0,
            noise_min_neighbors=1,
        )

    def test_all_seven_stages_run_in_order(self) -> None:
        result = self._full().run(raw(scene()))
        assert [s.stage for s in result.metrics.stages] == [
            ProcessingStage.VALIDATION,
            ProcessingStage.INVALID_REMOVAL,
            ProcessingStage.ROI_FILTER,
            ProcessingStage.RANGE_FILTER,
            ProcessingStage.VOXEL_DOWNSAMPLE,
            ProcessingStage.GROUND_SEGMENTATION,
            ProcessingStage.NOISE_FILTER,
        ]

    def test_stage_chain_is_continuous(self) -> None:
        stages = self._full().run(raw(scene())).metrics.stages
        for earlier, later in itertools.pairwise(stages):
            assert earlier.output_points == later.input_points

    def test_counts_partition_the_input(self) -> None:
        metrics = self._full().run(raw(scene())).metrics
        accounted = (
            metrics.invalid_point_count
            + metrics.roi_rejected_count
            + metrics.range_rejected_count
            + metrics.voxel_reduced_count
            + metrics.ground_point_count
            + metrics.noise_removed_count
            + metrics.output_point_count
        )
        assert accounted == metrics.input_point_count

    def test_ground_and_non_ground_are_both_available(self) -> None:
        result = self._full().run(raw(scene()))

        assert result.ground_frame is not None
        assert result.ground_frame.point_count == result.metrics.ground_point_count
        assert result.frame.point_count == result.metrics.non_ground_point_count
        assert result.ground_frame.point_count > 0
        assert result.frame.point_count > 0

    def test_ground_and_non_ground_do_not_overlap(self) -> None:
        result = self._full().run(raw(scene()))
        assert result.ground_frame is not None

        ground = {tuple(p) for p in result.ground_frame.points.tolist()}
        non_ground = {tuple(p) for p in result.frame.points.tolist()}
        assert ground.isdisjoint(non_ground)

    def test_the_road_ends_up_in_the_ground_frame(self) -> None:
        result = self._full().run(raw(scene()))
        assert result.ground_frame is not None

        road = result.ground_frame.points[result.ground_frame.points[:, 0] < 10.0]
        assert road.shape[0] > 0
        assert np.allclose(road[:, 2], -1.8)

    def test_an_isolated_stray_point_becomes_its_own_ground(self) -> None:
        """Known consequence of the stage order, asserted so it stays visible.

        The knowledge-base pipeline is voxelise -> ground -> noise, and ground
        points bypass the noise filter. A stray point alone in its xy cell is
        therefore the lowest point of that cell, is classified as ground, and
        the noise filter never sees it. Removing it would need the noise stage
        to run before segmentation, or to be applied to ground as well - both
        deviate from the documented order, so neither is done silently.
        """
        result = self._full().run(raw(scene()))
        assert result.ground_frame is not None

        stray = [40.0, 40.0, 3.0]
        assert stray in result.ground_frame.points.tolist()
        assert stray not in result.frame.points.tolist()

    def test_output_is_a_validated_finite_frame(self) -> None:
        result = self._full().run(raw([*scene(), [float("nan"), 0.0, 0.0]]))
        assert isinstance(result.frame, PointCloudFrame)
        assert np.isfinite(result.frame.points).all()
        assert result.metrics.invalid_point_count == 1


class TestStageInteraction:
    def test_voxelisation_runs_before_ground_segmentation(self) -> None:
        """Ground sees the downsampled cloud, so its input equals the voxel output."""
        stages = {
            s.stage: s
            for s in pipeline(voxel_enabled=True, voxel_size_m=0.5, ground_enabled=True)
            .run(raw(scene()))
            .metrics.stages
        }
        assert (
            stages[ProcessingStage.VOXEL_DOWNSAMPLE].output_points
            == stages[ProcessingStage.GROUND_SEGMENTATION].input_points
        )

    def test_noise_filtering_sees_only_non_ground_points(self) -> None:
        """Documented behaviour: ground bypasses the noise filter."""
        stages = {
            s.stage: s
            for s in pipeline(ground_enabled=True, noise_enabled=True, noise_min_neighbors=1)
            .run(raw(scene()))
            .metrics.stages
        }
        ground = stages[ProcessingStage.GROUND_SEGMENTATION]
        assert stages[ProcessingStage.NOISE_FILTER].input_points == ground.output_points

    def test_the_stray_point_is_removed_as_noise(self) -> None:
        result = pipeline(noise_enabled=True, noise_cell_size_m=1.0, noise_min_neighbors=2).run(
            raw(scene())
        )
        assert result.metrics.noise_removed_count >= 1
        assert [40.0, 40.0, 3.0] not in result.frame.points.tolist()

    def test_voxel_reduction_ratio_is_reported(self) -> None:
        metrics = pipeline(voxel_enabled=True, voxel_size_m=1.0).run(raw(scene())).metrics
        ratio = metrics.voxel_reduction_ratio

        assert ratio is not None
        assert 0.0 < ratio < 1.0

    def test_voxel_reduction_ratio_is_none_when_nothing_entered(self) -> None:
        empty = RawPointCloudFrame(
            frame_id=0, sensor_id="s", points=np.empty((0, 3), dtype=np.float64)
        )
        assert pipeline(voxel_enabled=True).run(empty).metrics.voxel_reduction_ratio is None


class TestEmptyAndDegenerate:
    @staticmethod
    def _all_enabled() -> LiDARProcessingPipeline:
        return pipeline(voxel_enabled=True, ground_enabled=True, noise_enabled=True)

    def test_empty_cloud_survives_every_stage(self) -> None:
        empty = RawPointCloudFrame(
            frame_id=0, sensor_id="s", points=np.empty((0, 3), dtype=np.float64)
        )
        result = self._all_enabled().run(empty)

        assert result.metrics.input_point_count == 0
        assert result.metrics.output_point_count == 0
        assert result.ground_frame is not None
        assert result.ground_frame.point_count == 0

    def test_single_point_cloud(self) -> None:
        result = self._all_enabled().run(raw([[1.0, 1.0, -1.8]]))
        assert result.metrics.input_point_count == 1
        assert result.metrics.ground_point_count == 1

    def test_a_cloud_reduced_to_nothing_is_not_an_error(self) -> None:
        result = pipeline(noise_enabled=True, noise_min_neighbors=99).run(raw(scene()))
        assert result.frame.point_count == 0
        assert result.metrics.noise_removed_count > 0


class TestMetadataPreservation:
    def test_metadata_survives_the_whole_chain(self) -> None:
        frame = raw(
            scene(),
            frame_id=77,
            sensor_id="roof_lidar",
            source=DataSource.REPLAY,
            coordinate_frame=CoordinateFrame.EGO,
        )
        result = pipeline(voxel_enabled=True, ground_enabled=True, noise_enabled=True).run(frame)

        for produced in (result.frame, result.ground_frame):
            assert produced is not None
            assert produced.frame_id == 77
            assert produced.sensor_id == "roof_lidar"
            assert produced.source is DataSource.REPLAY
            assert produced.coordinate_frame is CoordinateFrame.EGO
            assert produced.timestamp == frame.timestamp

    def test_intensity_column_survives_the_whole_chain(self) -> None:
        points = [[float(i) * 0.05, 0.0, -1.8, 0.1 * i] for i in range(10)]
        result = pipeline(voxel_enabled=True, voxel_size_m=0.2, ground_enabled=True).run(
            raw(points)
        )

        assert result.ground_frame is not None
        assert result.ground_frame.has_intensity is True
        assert result.ground_frame.fields == ("x", "y", "z", "intensity")

    def test_input_summary_still_describes_the_original(self) -> None:
        result = pipeline(voxel_enabled=True, voxel_size_m=5.0).run(raw(scene()))
        assert result.input_summary.point_count == len(scene())
        assert result.output_summary.point_count < result.input_summary.point_count


class TestMeasurement:
    def test_duration_is_measured(self) -> None:
        metrics = (
            pipeline(voxel_enabled=True, ground_enabled=True, noise_enabled=True)
            .run(raw(scene()))
            .metrics
        )
        assert metrics.duration_ms > 0.0

    def test_processor_is_identified(self) -> None:
        assert pipeline().run(raw(scene())).metrics.processor == "preprocessing_v1"


class TestDeterminism:
    def test_repeated_runs_are_identical(self) -> None:
        stage = pipeline(
            voxel_enabled=True, voxel_size_m=0.3, ground_enabled=True, noise_enabled=True
        )
        frame = raw(scene())
        first, second = stage.run(frame), stage.run(frame)

        assert np.array_equal(first.frame.points, second.frame.points)
        assert first.ground_frame is not None
        assert second.ground_frame is not None
        assert np.array_equal(first.ground_frame.points, second.ground_frame.points)
        assert first.metrics.output_point_count == second.metrics.output_point_count

    def test_two_instances_with_equal_config_agree(self) -> None:
        frame = raw(scene())
        config = {"voxel_enabled": True, "ground_enabled": True, "noise_enabled": True}
        a = pipeline(**config).run(frame)
        b = pipeline(**config).run(frame)
        assert np.array_equal(a.frame.points, b.frame.points)

    def test_input_frame_is_not_mutated(self) -> None:
        original = np.array(scene(), dtype=np.float64)
        frame = RawPointCloudFrame(frame_id=0, sensor_id="s", points=original.copy())
        pipeline(voxel_enabled=True, ground_enabled=True, noise_enabled=True).run(frame)
        assert np.array_equal(frame.points, original)
