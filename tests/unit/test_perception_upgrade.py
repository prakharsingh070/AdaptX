"""Perception rules added by the live perception upgrade, each measured live first.

Every threshold here came from CARLA 0.9.16 measurements recorded in
Experiment 015; the tests pin the rules, not the wisdom of the numbers.
"""

from __future__ import annotations

import numpy as np

from adaptx.config.settings import DetectionSettings, LiDARSettings, TrackingSettings
from adaptx.models.common import ObjectClass
from adaptx.models.detection import ClusterRejection
from adaptx.models.processing import floor_estimate_m
from adaptx.perception.classification import GeometricClassifier
from adaptx.perception.detector import GeometricObjectDetector
from adaptx.perception.pipeline import LiDARProcessingPipeline
from adaptx.tracking.tracker import GeometricObjectTracker
from tests.fixtures import scenes
from tests.fixtures.sequences import at, detection
from tests.integration.test_detection_pipeline import raw

GROUND = scenes.GROUND_Z_M


class TestElevatedClusters:
    """Overhead structure is not a road user: a cluster must touch the road."""

    def test_a_sign_above_the_road_is_rejected_when_a_floor_is_known(self) -> None:
        sign = scenes.box_shell((15.0, 3.0, GROUND + 3.0), (0.6, 0.2, 1.2))
        result = GeometricObjectDetector(DetectionSettings()).detect(
            scenes.frame(sign), floor_z_m=GROUND
        )
        assert result.objects == []
        assert [r.reason for r in result.rejected] == [ClusterRejection.ELEVATED]
        assert result.rejected[0].measured_value > 0.8
        assert result.rejected[0].threshold == 0.8

    def test_a_walker_on_the_road_passes_the_same_rule(self) -> None:
        result = GeometricObjectDetector(DetectionSettings()).detect(
            scenes.frame(scenes.pedestrian()), floor_z_m=GROUND
        )
        assert [o.object_class for o in result.objects] == [ObjectClass.PEDESTRIAN]

    def test_without_a_floor_estimate_nothing_is_rejected_as_elevated(self) -> None:
        """No ground stage, no floor: the rule stays off rather than guessing."""
        sign = scenes.box_shell((15.0, 3.0, GROUND + 3.0), (0.6, 0.2, 1.2))
        result = GeometricObjectDetector(DetectionSettings()).detect(scenes.frame(sign))
        assert all(r.reason is not ClusterRejection.ELEVATED for r in result.rejected)
        assert len(result.objects) == 1

    def test_the_rule_can_be_switched_off(self) -> None:
        sign = scenes.box_shell((15.0, 3.0, GROUND + 3.0), (0.6, 0.2, 1.2))
        result = GeometricObjectDetector(DetectionSettings(max_bottom_height_m=None)).detect(
            scenes.frame(sign), floor_z_m=GROUND
        )
        assert len(result.objects) == 1
        assert result.configuration.max_bottom_height_m is None

    def test_the_floor_estimate_is_the_ground_stage_median(self) -> None:
        pipeline = LiDARProcessingPipeline(LiDARSettings(ground_enabled=True))
        points = np.vstack([scenes.ground_plane(extent_m=20.0), scenes.vehicle()])
        processed = pipeline.run(raw(points))
        floor = floor_estimate_m(processed)
        assert floor is not None
        assert abs(floor - GROUND) < 0.2
        no_ground = LiDARProcessingPipeline(LiDARSettings(ground_enabled=False))
        assert floor_estimate_m(no_ground.run(raw(points))) is None


class TestPartialViewVehicles:
    """A car seen from behind never shows its length; its face is still a car."""

    def test_the_measured_audi_rear_view_is_a_vehicle(self) -> None:
        # 2.0 x 1.88 x 1.12 m: the Audi TT clustered from 5-10 m behind (Experiment 015).
        cls, fit = GeometricClassifier().classify(2.0, 1.88, 1.12)
        assert cls is ObjectClass.VEHICLE and fit > 0.0

    def test_the_far_rear_view_is_still_an_obstacle_not_a_vehicle(self) -> None:
        # 0.8 x 1.7 x 0.5 m at 20 m: too little visible height to call it a vehicle.
        cls, _ = GeometricClassifier().classify(0.8, 1.7, 0.5)
        assert cls is ObjectClass.OBSTACLE

    def test_a_bicycle_and_a_pedestrian_did_not_become_vehicles(self) -> None:
        assert GeometricClassifier().classify(1.7, 0.6, 1.7)[0] is ObjectClass.CYCLIST
        assert GeometricClassifier().classify(0.4, 0.4, 1.8)[0] is ObjectClass.PEDESTRIAN

    def test_a_wide_low_bench_is_not_a_vehicle(self) -> None:
        assert GeometricClassifier().classify(1.8, 0.5, 0.5)[0] is not ObjectClass.VEHICLE


class TestClassDecay:
    """A label the geometry stopped supporting does not live forever."""

    def test_three_unknown_observations_reset_the_class(self) -> None:
        tracker = GeometricObjectTracker(TrackingSettings(min_hits_to_confirm=1))
        tracker.update([detection((10.0, 0.0, 0.0), object_class=ObjectClass.PEDESTRIAN)], at(0.0))
        for step in range(1, 3):
            result = tracker.update(
                [detection((10.0, 0.0, 0.0), object_class=ObjectClass.UNKNOWN)], at(0.1 * step)
            )
            assert result.tracks[0].object_class is ObjectClass.PEDESTRIAN, "not yet"
        result = tracker.update(
            [detection((10.0, 0.0, 0.0), object_class=ObjectClass.UNKNOWN)], at(0.3)
        )
        assert result.tracks[0].object_class is ObjectClass.UNKNOWN
        assert result.tracks[0].track_id == 0, "identity is kept; only the label decayed"

    def test_an_agreeing_observation_resets_the_decay_count(self) -> None:
        tracker = GeometricObjectTracker(TrackingSettings(min_hits_to_confirm=1))
        tracker.update([detection((10.0, 0.0, 0.0), object_class=ObjectClass.VEHICLE)], at(0.0))
        tracker.update([detection((10.0, 0.0, 0.0), object_class=ObjectClass.UNKNOWN)], at(0.1))
        tracker.update([detection((10.0, 0.0, 0.0), object_class=ObjectClass.UNKNOWN)], at(0.2))
        tracker.update([detection((10.0, 0.0, 0.0), object_class=ObjectClass.VEHICLE)], at(0.3))
        tracker.update([detection((10.0, 0.0, 0.0), object_class=ObjectClass.UNKNOWN)], at(0.4))
        result = tracker.update(
            [detection((10.0, 0.0, 0.0), object_class=ObjectClass.UNKNOWN)], at(0.5)
        )
        assert result.tracks[0].object_class is ObjectClass.VEHICLE

    def test_a_far_flickering_detection_keeps_its_identity(self) -> None:
        """Measured live: a car at 20-26 m is detected on alternate frames."""
        tracker = GeometricObjectTracker(TrackingSettings())
        ids = set()
        for step in range(8):
            detections = [detection((25.0, 0.0, 0.0))] if step % 2 == 0 else []
            result = tracker.update(detections, at(0.05 * step))
            ids.update(t.track_id for t in result.tracks)
        assert len(ids) == 1, ids
