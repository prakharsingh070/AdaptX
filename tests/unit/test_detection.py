"""Geometric object detection tests (Phase 3).

Scenes are built from explicit geometry so every expected cluster count and
dimension is known by construction. Nothing here is random except the ground
plane, which is seeded.

Assertions target meaningful properties — how many objects, which class, where,
how big — rather than exact floating-point values or internal label ordering.
"""

from __future__ import annotations

import numpy as np
import pytest

from adaptx.config.settings import DetectionSettings
from adaptx.models.common import ObjectClass
from adaptx.models.detection import ClusterRejection
from adaptx.perception.classification import BANDS, EDGE_FIT, GeometricClassifier
from adaptx.perception.clustering import GridConnectedComponentClusterer
from adaptx.perception.detector import GeometricObjectDetector
from tests.fixtures import scenes


def detector(**overrides: object) -> GeometricObjectDetector:
    """A detector whose configuration starts at the defaults."""
    return GeometricObjectDetector(DetectionSettings(**overrides))  # type: ignore[arg-type]


def clusterer(tolerance_m: float = 0.5) -> GridConnectedComponentClusterer:
    return GridConnectedComponentClusterer(DetectionSettings(cluster_tolerance_m=tolerance_m))


class TestClustering:
    def test_empty_cloud(self) -> None:
        labels = clusterer().cluster(np.empty((0, 3), dtype=np.float64))
        assert labels.shape == (0,)

    def test_single_point_is_one_cluster(self) -> None:
        labels = clusterer().cluster(np.array([[1.0, 2.0, 3.0]]))
        assert labels.tolist() == [0]

    def test_one_object_is_one_cluster(self) -> None:
        labels = clusterer().cluster(scenes.vehicle())
        assert labels.max() == 0

    def test_two_separated_objects_are_two_clusters(self) -> None:
        points = np.vstack([scenes.vehicle((10.0, -5.0, 0.0)), scenes.vehicle((30.0, 8.0, 0.0))])
        assert clusterer().cluster(points).max() == 1

    def test_three_separated_objects_are_three_clusters(self) -> None:
        points = np.vstack(
            [
                scenes.vehicle((10.0, -6.0, 0.0)),
                scenes.pedestrian((20.0, 6.0, 0.0)),
                scenes.cyclist((30.0, 0.0, 0.0)),
            ]
        )
        assert clusterer().cluster(points).max() == 2

    def test_objects_closer_than_the_tolerance_merge(self) -> None:
        """Documented behaviour: touching cells join, so near objects merge."""
        points = np.vstack(
            [scenes.pedestrian((10.0, 0.0, 0.0)), scenes.pedestrian((10.0, 0.6, 0.0))]
        )
        assert clusterer(tolerance_m=1.0).cluster(points).max() == 0

    def test_a_smaller_tolerance_separates_them(self) -> None:
        points = np.vstack(
            [scenes.pedestrian((10.0, 0.0, 0.0)), scenes.pedestrian((10.0, 3.0, 0.0))]
        )
        assert clusterer(tolerance_m=0.3).cluster(points).max() == 1

    def test_labels_are_dense_and_zero_based(self) -> None:
        points = np.vstack([scenes.vehicle((10.0, -8.0, 0.0)), scenes.vehicle((40.0, 8.0, 0.0))])
        labels = clusterer().cluster(points)
        assert sorted(set(labels.tolist())) == [0, 1]

    def test_labels_follow_first_appearance(self) -> None:
        """Cluster 0 is whichever cluster's first point comes first."""
        far = scenes.vehicle((40.0, 8.0, 0.0))
        near = scenes.vehicle((10.0, -8.0, 0.0))
        labels = clusterer().cluster(np.vstack([far, near]))
        assert labels[0] == 0
        assert labels[len(far)] == 1

    def test_clustering_is_deterministic(self) -> None:
        points = np.vstack([scenes.vehicle(), scenes.pedestrian()])
        stage = clusterer()
        assert np.array_equal(stage.cluster(points), stage.cluster(points))

    def test_a_long_chain_stays_one_cluster(self) -> None:
        """Connectivity must propagate along a chain, not just to neighbours."""
        chain = np.array([[float(i) * 0.4, 0.0, 0.0] for i in range(200)])
        assert clusterer(tolerance_m=0.5).cluster(chain).max() == 0


class TestGeometry:
    def test_centroid_and_extents_match_the_box(self) -> None:
        result = detector().detect(scenes.frame(scenes.vehicle((12.0, -3.0, 0.5))))
        assert result.object_count == 1

        detected = result.objects[0]
        assert detected.position.x == pytest.approx(12.0, abs=0.05)
        assert detected.position.y == pytest.approx(-3.0, abs=0.05)
        assert detected.position.z == pytest.approx(0.5, abs=0.05)
        assert detected.extents[0] == pytest.approx(4.5, abs=0.05)
        assert detected.extents[1] == pytest.approx(1.9, abs=0.05)
        assert detected.extents[2] == pytest.approx(1.6, abs=0.05)

    def test_distance_is_measured_from_the_sensor_origin(self) -> None:
        result = detector().detect(scenes.frame(scenes.vehicle((30.0, 40.0, 0.0))))
        assert result.objects[0].distance_m == pytest.approx(50.0, abs=0.2)

    def test_bounding_box_is_axis_aligned(self) -> None:
        result = detector().detect(scenes.frame(scenes.vehicle()))
        assert result.objects[0].bounding_box.yaw_rad == 0.0

    def test_point_count_is_reported(self) -> None:
        points = scenes.vehicle()
        result = detector().detect(scenes.frame(points))
        assert result.objects[0].point_count == points.shape[0]

    def test_objects_at_different_distances_are_all_found(self) -> None:
        points = np.vstack(
            [
                scenes.vehicle((8.0, -4.0, 0.0)),
                scenes.vehicle((25.0, 0.0, 0.0)),
                scenes.vehicle((45.0, 6.0, 0.0)),
            ]
        )
        result = detector().detect(scenes.frame(points))
        distances = sorted(o.distance_m for o in result.objects)

        assert result.object_count == 3
        assert distances[0] < distances[1] < distances[2]

    def test_negative_coordinates_are_handled(self) -> None:
        result = detector().detect(scenes.frame(scenes.vehicle((-15.0, -10.0, -1.0))))
        assert result.object_count == 1
        assert result.objects[0].position.x == pytest.approx(-15.0, abs=0.05)


class TestClassification:
    @staticmethod
    def _classify(length: float, width: float, height: float) -> tuple[ObjectClass, float]:
        return GeometricClassifier().classify(length, width, height)

    def test_vehicle_sized_cluster(self) -> None:
        object_class, confidence = self._classify(4.5, 1.9, 1.6)
        assert object_class is ObjectClass.VEHICLE
        assert 0.0 < confidence <= 1.0

    def test_pedestrian_sized_cluster(self) -> None:
        object_class, _ = self._classify(0.6, 0.5, 1.75)
        assert object_class is ObjectClass.PEDESTRIAN

    def test_cyclist_sized_cluster(self) -> None:
        object_class, _ = self._classify(1.8, 0.6, 1.7)
        assert object_class is ObjectClass.CYCLIST

    def test_low_obstacle(self) -> None:
        object_class, _ = self._classify(0.8, 0.8, 0.6)
        assert object_class is ObjectClass.OBSTACLE

    def test_geometry_matching_nothing_is_unknown(self) -> None:
        object_class, confidence = self._classify(8.0, 3.0, 2.0)
        assert object_class is ObjectClass.UNKNOWN
        assert confidence == 0.0

    def test_geometry_matching_several_bands_is_unknown(self) -> None:
        """Ambiguity is reported, not resolved by picking a favourite."""
        overlapping = [
            (length, width, height)
            for length in (0.8, 1.5, 2.5)
            for width in (0.5, 0.9)
            for height in (0.9, 1.5, 1.9)
            if sum(band.contains(height, max(length, width), min(length, width)) for band in BANDS)
            > 1
        ]
        assert overlapping, "expected at least one genuinely ambiguous shape"
        for length, width, height in overlapping:
            assert self._classify(length, width, height)[0] is ObjectClass.UNKNOWN

    def test_classification_is_rotation_invariant_at_90_degrees(self) -> None:
        """Axis-aligned boxes swap x and y when an object turns; the class must not."""
        assert self._classify(4.5, 1.9, 1.6)[0] is self._classify(1.9, 4.5, 1.6)[0]

    def test_confidence_is_highest_at_the_centre_of_a_band(self) -> None:
        band = next(b for b in BANDS if b.object_class is ObjectClass.VEHICLE)
        centre_length = (band.min_length_m + band.max_length_m) / 2
        centre_width = (band.min_width_m + band.max_width_m) / 2
        centre_height = (band.min_height_m + band.max_height_m) / 2

        centred = self._classify(centre_length, centre_width, centre_height)[1]
        offset = self._classify(band.max_length_m, centre_width, centre_height)[1]

        assert centred == pytest.approx(1.0)
        assert offset < centred

    def test_confidence_at_a_band_edge_is_the_documented_floor(self) -> None:
        band = next(b for b in BANDS if b.object_class is ObjectClass.VEHICLE)
        confidence = self._classify(
            band.max_length_m,
            (band.min_width_m + band.max_width_m) / 2,
            (band.min_height_m + band.max_height_m) / 2,
        )[1]
        assert confidence == pytest.approx(EDGE_FIT, abs=1e-4)

    def test_confidence_never_leaves_the_unit_interval(self) -> None:
        for length in (0.1, 1.0, 3.0, 6.0, 20.0):
            for height in (0.1, 1.0, 2.0, 5.0):
                _, confidence = self._classify(length, length * 0.5, height)
                assert 0.0 <= confidence <= 1.0

    def test_classifier_declares_itself_a_baseline(self) -> None:
        assert GeometricClassifier().is_baseline is True


class TestClusterFiltering:
    def test_small_noise_cluster_is_rejected(self) -> None:
        result = detector().detect(scenes.frame(scenes.noise_speck()))

        assert result.object_count == 0
        assert result.rejected_count == 1
        assert result.rejected[0].reason is ClusterRejection.TOO_FEW_POINTS

    def test_rejection_records_the_measurement_and_threshold(self) -> None:
        result = detector(min_cluster_points=50).detect(scenes.frame(scenes.noise_speck()))
        rejection = result.rejected[0]

        assert rejection.measured_value == 3.0
        assert rejection.threshold == 50.0
        assert rejection.point_count == 3

    def test_a_flat_cluster_is_rejected_as_too_short(self) -> None:
        flat = scenes.box_shell((10.0, 0.0, 0.0), (2.0, 2.0, 0.01))
        result = detector().detect(scenes.frame(flat))

        assert result.object_count == 0
        assert result.rejected[0].reason is ClusterRejection.TOO_SHORT

    def test_an_enormous_cluster_is_rejected_by_footprint(self) -> None:
        wall = scenes.box_shell((20.0, 0.0, 0.0), (40.0, 0.5, 2.0))
        result = detector().detect(scenes.frame(wall))

        assert result.object_count == 0
        assert result.rejected[0].reason is ClusterRejection.FOOTPRINT_TOO_LARGE

    def test_too_tall_is_rejected(self) -> None:
        tall = scenes.box_shell((10.0, 0.0, 3.0), (1.0, 1.0, 8.0))
        result = detector().detect(scenes.frame(tall))
        assert result.rejected[0].reason is ClusterRejection.TOO_TALL

    def test_too_many_points_is_rejected(self) -> None:
        result = detector(max_cluster_points=10).detect(scenes.frame(scenes.vehicle()))
        assert result.rejected[0].reason is ClusterRejection.TOO_MANY_POINTS

    def test_accepted_plus_rejected_equals_cluster_count(self) -> None:
        points = np.vstack([scenes.vehicle(), scenes.noise_speck(), scenes.pedestrian()])
        result = detector().detect(scenes.frame(points))
        assert result.cluster_count == result.object_count + result.rejected_count


class TestDetectionResult:
    def test_result_metadata_matches_the_frame(self) -> None:
        frame = scenes.frame(scenes.vehicle(), frame_id=42, sensor_id="roof_lidar")
        result = detector().detect(frame)

        assert result.frame_id == 42
        assert result.sensor_id == "roof_lidar"
        assert result.timestamp == frame.timestamp
        assert result.detector == "geometric_detector_v1"
        assert result.is_baseline is True

    def test_detections_inherit_frame_provenance(self) -> None:
        frame = scenes.frame(scenes.vehicle())
        detected = detector().detect(frame).objects[0]

        assert detected.source is frame.source
        assert detected.coordinate_frame is frame.coordinate_frame
        assert detected.timestamp == frame.timestamp
        assert detected.frame_id == frame.frame_id

    def test_velocity_is_never_invented(self) -> None:
        """One frame cannot show motion, so velocity must stay null."""
        result = detector().detect(scenes.frame(scenes.vehicle()))
        assert all(o.velocity is None for o in result.objects)

    def test_durations_are_measured(self) -> None:
        result = detector().detect(scenes.frame(scenes.vehicle()))

        assert result.duration_ms > 0.0
        assert result.clustering_duration_ms > 0.0
        assert result.classification_duration_ms >= 0.0
        assert result.clustering_duration_ms <= result.duration_ms

    def test_configuration_travels_with_the_result(self) -> None:
        result = detector(cluster_tolerance_m=0.75).detect(scenes.frame(scenes.vehicle()))
        assert result.configuration.cluster_tolerance_m == 0.75

    def test_counts_by_class(self) -> None:
        points = np.vstack(
            [
                scenes.vehicle((12.0, -6.0, 0.0)),
                scenes.vehicle((30.0, 6.0, 0.0)),
                scenes.pedestrian((20.0, 0.0, 0.0)),
            ]
        )
        counts = detector().detect(scenes.frame(points)).counts_by_class()
        assert counts.get("vehicle") == 2
        assert counts.get("pedestrian") == 1

    def test_object_ids_are_unique_within_the_frame(self) -> None:
        points = np.vstack([scenes.vehicle((12.0, -6.0, 0.0)), scenes.pedestrian((25.0, 5.0, 0.0))])
        result = detector().detect(scenes.frame(points))
        ids = [o.object_id for o in result.objects]
        assert len(ids) == len(set(ids))

    def test_detector_reports_a_baseline_classifier(self) -> None:
        detected = detector().detect(scenes.frame(scenes.vehicle())).objects[0]
        assert detected.classifier == "geometric_bands_v1"
        assert detected.is_baseline_classification is True


class TestDegenerateInput:
    def test_empty_frame(self) -> None:
        result = detector().detect(scenes.empty_frame())

        assert result.cluster_count == 0
        assert result.object_count == 0
        assert result.rejected_count == 0
        assert result.input_point_count == 0
        assert result.duration_ms >= 0.0

    def test_all_ground_cloud_yields_no_objects(self) -> None:
        """A flat plane is one enormous cluster and must not become an object."""
        result = detector().detect(scenes.frame(scenes.ground_plane()))

        assert result.object_count == 0
        assert result.rejected_count >= 1

    def test_output_contains_no_invalid_numbers(self) -> None:
        points = np.vstack([scenes.vehicle(), scenes.pedestrian(), scenes.noise_speck()])
        result = detector().detect(scenes.frame(points))

        for detected in result.objects:
            values = [
                detected.position.x,
                detected.position.y,
                detected.position.z,
                detected.distance_m,
                detected.confidence,
                *detected.extents,
            ]
            assert all(np.isfinite(value) for value in values)


class TestConfiguration:
    def test_changing_the_tolerance_changes_the_result(self) -> None:
        points = np.vstack(
            [scenes.pedestrian((10.0, 0.0, 0.0)), scenes.pedestrian((10.0, 1.2, 0.0))]
        )
        merged = detector(cluster_tolerance_m=1.5).detect(scenes.frame(points))
        separate = detector(cluster_tolerance_m=0.2).detect(scenes.frame(points))

        assert merged.cluster_count == 1
        assert separate.cluster_count == 2

    def test_bounds_must_be_ordered(self) -> None:
        from pydantic import ValidationError

        with pytest.raises(ValidationError, match="min_height_m"):
            DetectionSettings(min_height_m=5.0, max_height_m=1.0)
        with pytest.raises(ValidationError, match="min_footprint_m"):
            DetectionSettings(min_footprint_m=9.0, max_footprint_m=1.0)
        with pytest.raises(ValidationError, match="min_cluster_points"):
            DetectionSettings(min_cluster_points=100, max_cluster_points=10)

    def test_tolerance_must_be_positive(self) -> None:
        from pydantic import ValidationError

        with pytest.raises(ValidationError):
            DetectionSettings(cluster_tolerance_m=0.0)


class TestDeterminism:
    def test_repeated_detection_is_identical(self) -> None:
        frame = scenes.frame(np.vstack([scenes.vehicle(), scenes.pedestrian()]))
        stage = detector()
        first, second = stage.detect(frame), stage.detect(frame)

        assert [o.object_class for o in first.objects] == [o.object_class for o in second.objects]
        assert [o.point_count for o in first.objects] == [o.point_count for o in second.objects]
        assert [o.position.x for o in first.objects] == [o.position.x for o in second.objects]

    def test_two_detectors_with_equal_configuration_agree(self) -> None:
        frame = scenes.frame(np.vstack([scenes.vehicle(), scenes.cyclist()]))
        a = detector().detect(frame)
        b = detector().detect(frame)
        assert [o.object_class for o in a.objects] == [o.object_class for o in b.objects]

    def test_the_input_frame_is_not_mutated(self) -> None:
        points = np.vstack([scenes.vehicle(), scenes.pedestrian()])
        frame = scenes.frame(points)
        original = frame.points.copy()
        detector().detect(frame)
        assert np.array_equal(frame.points, original)
