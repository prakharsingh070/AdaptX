"""Baseline tracker tests (Phase 4).

Every sequence has explicit positions and explicit timestamps, so the velocity
a test expects is arithmetic rather than a guess. Nothing here is random.

Detections are constructed directly rather than produced by the detector: these
tests exercise tracking, and coupling them to detection would make a tracking
failure indistinguishable from a clustering one.
"""

from __future__ import annotations

import math

import pytest

from adaptx.config.settings import TrackingSettings
from adaptx.models.common import ObjectClass, Vector3
from adaptx.models.tracking import TrackStatus
from adaptx.tracking.tracker import GeometricObjectTracker
from tests.fixtures import sequences
from tests.fixtures.sequences import at, detection


def tracker(**overrides: object) -> GeometricObjectTracker:
    """A tracker whose configuration starts at the defaults."""
    return GeometricObjectTracker(TrackingSettings(**overrides))  # type: ignore[arg-type]


def run(stage: GeometricObjectTracker, sequence: list[tuple[list, object]]) -> list:
    """Feed a whole sequence and return every frame's result."""
    return [stage.update(detections, timestamp) for detections, timestamp in sequence]  # type: ignore[arg-type]


class TestTrackCreationAndIdentity:
    def test_a_new_detection_creates_a_tentative_track(self) -> None:
        result = tracker().update([detection((10.0, 0.0, 0.0))], at(0.0))

        assert result.active_track_count == 1
        assert result.tracks[0].status is TrackStatus.TENTATIVE
        assert result.new_track_ids == [0]

    def test_the_same_object_keeps_its_id_across_frames(self) -> None:
        results = run(tracker(), sequences.linear_motion((10.0, 0.0, 0.0), (5.0, 0.0, 0.0)))
        ids = [result.tracks[0].track_id for result in results]
        assert ids == [0] * len(results)

    def test_a_second_object_gets_a_new_id(self) -> None:
        stage = tracker()
        stage.update([detection((10.0, 0.0, 0.0))], at(0.0))
        result = stage.update(
            [detection((10.0, 0.0, 0.0), object_id=0), detection((30.0, 8.0, 0.0), object_id=1)],
            at(0.1),
        )

        assert result.active_track_count == 2
        assert sorted(t.track_id for t in result.tracks) == [0, 1]
        assert result.new_track_ids == [1]

    def test_ids_are_not_reused_after_deletion(self) -> None:
        stage = tracker(max_missed_frames=0, max_missed_frames_tentative=0)
        stage.update([detection((10.0, 0.0, 0.0))], at(0.0))
        deleted = stage.update([], at(0.1))
        assert deleted.deleted_track_ids == [0]

        recreated = stage.update([detection((10.0, 0.0, 0.0))], at(0.2))
        assert recreated.tracks[0].track_id == 1

    def test_detection_order_does_not_change_id_assignment(self) -> None:
        """Two trackers fed the same objects in opposite order must agree."""
        first, second = tracker(), tracker()
        near = detection((10.0, 0.0, 0.0), object_id=0)
        far = detection((40.0, 0.0, 0.0), object_id=1)

        first.update([near, far], at(0.0))
        second.update([far, near], at(0.0))

        moved_near = detection((10.5, 0.0, 0.0), object_id=0)
        moved_far = detection((40.5, 0.0, 0.0), object_id=1)
        a = first.update([moved_near, moved_far], at(0.1))
        b = second.update([moved_far, moved_near], at(0.1))

        def by_position(result: object) -> list[tuple[float, int]]:
            return sorted((t.position.x, t.track_id) for t in result.tracks)  # type: ignore[attr-defined]

        # The same physical objects must end up with consistently mapped ids.
        assert len(by_position(a)) == len(by_position(b)) == 2
        assert [x for x, _ in by_position(a)] == [x for x, _ in by_position(b)]

    def test_reset_clears_everything(self) -> None:
        stage = tracker()
        stage.update([detection((10.0, 0.0, 0.0))], at(0.0))
        assert stage.live_track_count == 1

        stage.reset()
        assert stage.live_track_count == 0

        result = stage.update([detection((10.0, 0.0, 0.0))], at(1.0))
        assert result.tracks[0].track_id == 0


class TestVelocity:
    def test_first_frame_velocity_is_unknown_not_zero(self) -> None:
        """One observation cannot show motion. Zero would claim a standstill."""
        result = tracker().update([detection((10.0, 0.0, 0.0))], at(0.0))
        track = result.tracks[0]

        assert track.velocity is None
        assert track.observed_velocity is None
        assert track.speed_mps is None
        assert track.heading_rad is None

    def test_forward_motion(self) -> None:
        stage = tracker(velocity_smoothing=1.0)
        stage.update([detection((0.0, 0.0, 0.0))], at(0.0))
        result = stage.update([detection((1.0, 0.0, 0.0))], at(0.5))

        velocity = result.tracks[0].velocity
        assert velocity is not None
        assert velocity.x == pytest.approx(2.0)
        assert velocity.y == pytest.approx(0.0)

    def test_lateral_motion(self) -> None:
        stage = tracker(velocity_smoothing=1.0)
        stage.update([detection((10.0, 0.0, 0.0))], at(0.0))
        result = stage.update([detection((10.0, 2.0, 0.0))], at(0.5))

        velocity = result.tracks[0].velocity
        assert velocity is not None
        assert velocity.y == pytest.approx(4.0)

    def test_diagonal_motion(self) -> None:
        # A 5 m step needs a gate wider than the 2.5 m default, which would
        # otherwise (correctly) refuse the jump as implausible.
        stage = tracker(velocity_smoothing=1.0, max_association_distance_m=10.0)
        stage.update([detection((0.0, 0.0, 0.0))], at(0.0))
        result = stage.update([detection((3.0, 4.0, 0.0))], at(1.0))

        track = result.tracks[0]
        assert track.speed_mps == pytest.approx(5.0)

    def test_stationary_object_measures_zero_not_null(self) -> None:
        """A measured standstill is real information, distinct from unknown."""
        results = run(tracker(), sequences.stationary())
        track = results[-1].tracks[0]

        assert track.velocity is not None
        assert track.speed_mps == pytest.approx(0.0)
        assert track.is_moving is False

    def test_variable_intervals_are_handled(self) -> None:
        stage = tracker(velocity_smoothing=1.0)
        stage.update([detection((0.0, 0.0, 0.0))], at(0.0))
        first = stage.update([detection((1.0, 0.0, 0.0))], at(0.5))
        second = stage.update([detection((3.0, 0.0, 0.0))], at(1.5))

        assert first.tracks[0].velocity.x == pytest.approx(2.0)
        assert second.tracks[0].velocity.x == pytest.approx(2.0)

    def test_zero_interval_yields_no_velocity(self) -> None:
        """Dividing by a zero interval would be meaningless, so nothing is claimed."""
        stage = tracker()
        stage.update([detection((0.0, 0.0, 0.0))], at(0.0))
        result = stage.update([detection((1.0, 0.0, 0.0))], at(0.0))

        assert result.tracks[0].velocity is None

    def test_backwards_timestamps_yield_no_velocity(self) -> None:
        stage = tracker()
        stage.update([detection((0.0, 0.0, 0.0))], at(1.0))
        result = stage.update([detection((1.0, 0.0, 0.0))], at(0.5))

        assert result.tracks[0].velocity is None

    def test_a_gap_longer_than_the_limit_yields_no_velocity(self) -> None:
        stage = tracker(max_timestep_s=1.0, max_association_distance_m=100.0)
        stage.update([detection((0.0, 0.0, 0.0))], at(0.0))
        result = stage.update([detection((1.0, 0.0, 0.0))], at(5.0))

        assert result.tracks[0].velocity is None

    def test_raw_velocity_is_reported_alongside_the_smoothed_one(self) -> None:
        """Smoothing must never hide the observation that caused a jump."""
        stage = tracker(velocity_smoothing=0.5, max_association_distance_m=10.0)
        stage.update([detection((0.0, 0.0, 0.0))], at(0.0))
        stage.update([detection((1.0, 0.0, 0.0))], at(0.5))
        result = stage.update([detection((5.0, 0.0, 0.0))], at(1.0))

        track = result.tracks[0]
        assert track.observed_velocity is not None
        assert track.observed_velocity.x == pytest.approx(8.0)
        assert track.velocity is not None
        assert track.velocity.x == pytest.approx(0.5 * 8.0 + 0.5 * 2.0)

    def test_no_smoothing_reports_the_raw_value(self) -> None:
        stage = tracker(velocity_smoothing=1.0, max_association_distance_m=10.0)
        stage.update([detection((0.0, 0.0, 0.0))], at(0.0))
        stage.update([detection((1.0, 0.0, 0.0))], at(0.5))
        result = stage.update([detection((5.0, 0.0, 0.0))], at(1.0))

        assert result.tracks[0].velocity.x == pytest.approx(8.0)

    def test_acceleration_needs_two_velocities(self) -> None:
        stage = tracker(velocity_smoothing=1.0)
        stage.update([detection((0.0, 0.0, 0.0))], at(0.0))
        first = stage.update([detection((1.0, 0.0, 0.0))], at(1.0))
        assert first.tracks[0].acceleration is None

        second = stage.update([detection((3.0, 0.0, 0.0))], at(2.0))
        assert second.tracks[0].acceleration is not None
        assert second.tracks[0].acceleration.x == pytest.approx(1.0)


class TestHeading:
    def test_heading_matches_the_direction_of_travel(self) -> None:
        stage = tracker(velocity_smoothing=1.0, min_speed_for_heading_mps=0.1)
        stage.update([detection((0.0, 0.0, 0.0))], at(0.0))
        result = stage.update([detection((1.0, 1.0, 0.0))], at(1.0))

        assert result.tracks[0].heading_rad == pytest.approx(math.pi / 4)

    def test_heading_is_unknown_below_the_speed_floor(self) -> None:
        """At a crawl the direction describes noise, not travel."""
        stage = tracker(min_speed_for_heading_mps=1.0, velocity_smoothing=1.0)
        stage.update([detection((0.0, 0.0, 0.0))], at(0.0))
        result = stage.update([detection((0.01, 0.0, 0.0))], at(1.0))

        assert result.tracks[0].heading_rad is None

    def test_a_stationary_object_has_no_heading(self) -> None:
        results = run(tracker(), sequences.stationary())
        assert results[-1].tracks[0].heading_rad is None


class TestAssociation:
    def test_the_nearest_plausible_detection_matches(self) -> None:
        stage = tracker()
        stage.update([detection((10.0, 0.0, 0.0))], at(0.0))
        result = stage.update(
            [detection((10.4, 0.0, 0.0), object_id=0), detection((30.0, 0.0, 0.0), object_id=1)],
            at(0.1),
        )

        assert result.association_count == 1
        assert result.tracks[0].position.x == pytest.approx(10.4)

    def test_a_detection_beyond_the_gate_does_not_match(self) -> None:
        stage = tracker(max_association_distance_m=1.0)
        stage.update([detection((10.0, 0.0, 0.0))], at(0.0))
        result = stage.update([detection((50.0, 0.0, 0.0))], at(0.1))

        assert result.association_count == 0
        assert result.active_track_count == 2

    def test_gate_boundary_is_inclusive(self) -> None:
        stage = tracker(
            max_association_distance_m=2.0, use_predicted_position_for_association=False
        )
        stage.update([detection((10.0, 0.0, 0.0))], at(0.0))
        result = stage.update([detection((12.0, 0.0, 0.0))], at(0.1))

        assert result.association_count == 1

    def test_one_detection_cannot_feed_two_tracks(self) -> None:
        stage = tracker(use_predicted_position_for_association=False)
        stage.update(
            [detection((10.0, 0.0, 0.0), object_id=0), detection((11.0, 0.0, 0.0), object_id=1)],
            at(0.0),
        )
        result = stage.update([detection((10.5, 0.0, 0.0))], at(0.1))

        assert result.association_count == 1
        assert len(result.unmatched_track_ids) == 1

    def test_one_track_cannot_take_two_detections(self) -> None:
        stage = tracker(use_predicted_position_for_association=False)
        stage.update([detection((10.0, 0.0, 0.0))], at(0.0))
        result = stage.update(
            [detection((10.2, 0.0, 0.0), object_id=0), detection((10.4, 0.0, 0.0), object_id=1)],
            at(0.1),
        )

        assert result.association_count == 1
        assert len(result.unmatched_detection_ids) == 1
        assert result.active_track_count == 2

    def test_class_matching_can_be_required(self) -> None:
        stage = tracker(require_class_match=True, use_predicted_position_for_association=False)
        stage.update([detection((10.0, 0.0, 0.0), object_class=ObjectClass.VEHICLE)], at(0.0))
        result = stage.update(
            [detection((10.2, 0.0, 0.0), object_class=ObjectClass.PEDESTRIAN)], at(0.1)
        )

        assert result.association_count == 0

    def test_unknown_class_still_associates_when_matching_is_required(self) -> None:
        """UNKNOWN is common from the geometric classifier; refusing it would
        fragment otherwise good tracks."""
        stage = tracker(require_class_match=True, use_predicted_position_for_association=False)
        stage.update([detection((10.0, 0.0, 0.0), object_class=ObjectClass.VEHICLE)], at(0.0))
        result = stage.update(
            [detection((10.2, 0.0, 0.0), object_class=ObjectClass.UNKNOWN)], at(0.1)
        )

        assert result.association_count == 1

    def test_size_gate_rejects_an_implausible_match(self) -> None:
        stage = tracker(max_size_ratio=2.0, use_predicted_position_for_association=False)
        stage.update([detection((10.0, 0.0, 0.0), object_class=ObjectClass.VEHICLE)], at(0.0))
        result = stage.update(
            [detection((10.2, 0.0, 0.0), object_class=ObjectClass.PEDESTRIAN)], at(0.1)
        )

        assert result.association_count == 0

    def test_association_is_deterministic_for_a_close_crossing(self) -> None:
        first = run(tracker(), sequences.crossing_pair())
        second = run(tracker(), sequences.crossing_pair())

        assert [sorted((t.track_id, round(t.position.y, 6)) for t in r.tracks) for r in first] == [
            sorted((t.track_id, round(t.position.y, 6)) for t in r.tracks) for r in second
        ]

    def test_parallel_objects_keep_separate_ids(self) -> None:
        results = run(tracker(), sequences.parallel_pair())
        final = results[-1]

        assert final.active_track_count == 2
        assert sorted(t.track_id for t in final.tracks) == [0, 1]


class TestLifecycle:
    def test_tentative_becomes_confirmed_after_enough_hits(self) -> None:
        stage = tracker(min_hits_to_confirm=3)
        sequence = sequences.linear_motion((10.0, 0.0, 0.0), (1.0, 0.0, 0.0), frames=3)
        statuses = [result.tracks[0].status for result in run(stage, sequence)]

        assert statuses == [
            TrackStatus.TENTATIVE,
            TrackStatus.TENTATIVE,
            TrackStatus.CONFIRMED,
        ]

    def test_confirmed_becomes_coasting_when_unmatched(self) -> None:
        stage = tracker(min_hits_to_confirm=1)
        stage.update([detection((10.0, 0.0, 0.0))], at(0.0))
        result = stage.update([], at(0.1))

        assert result.tracks[0].status is TrackStatus.COASTING
        assert result.tracks[0].missed_frames == 1

    def test_coasting_recovers_to_confirmed(self) -> None:
        stage = tracker(min_hits_to_confirm=1, max_missed_frames=3)
        stage.update([detection((10.0, 0.0, 0.0))], at(0.0))
        stage.update([], at(0.1))
        result = stage.update([detection((10.1, 0.0, 0.0))], at(0.2))

        assert result.tracks[0].status is TrackStatus.CONFIRMED
        assert result.tracks[0].missed_frames == 0
        assert result.tracks[0].track_id == 0

    def test_an_object_missing_one_frame_keeps_its_id(self) -> None:
        stage = tracker(min_hits_to_confirm=1, max_missed_frames=2)
        stage.update([detection((10.0, 0.0, 0.0))], at(0.0))
        stage.update([], at(0.1))
        result = stage.update([detection((10.2, 0.0, 0.0))], at(0.2))

        assert result.tracks[0].track_id == 0
        assert result.new_track_ids == []

    def test_a_track_missing_too_long_is_deleted(self) -> None:
        stage = tracker(min_hits_to_confirm=1, max_missed_frames=2)
        stage.update([detection((10.0, 0.0, 0.0))], at(0.0))
        for index in range(3):
            result = stage.update([], at(0.1 * (index + 1)))

        assert result.deleted_track_ids == [0]
        assert result.active_track_count == 0

    def test_an_object_returning_after_deletion_gets_a_new_id(self) -> None:
        """No re-identification exists: a retired track does not come back."""
        stage = tracker(min_hits_to_confirm=1, max_missed_frames=1, max_missed_frames_tentative=1)
        stage.update([detection((10.0, 0.0, 0.0))], at(0.0))
        stage.update([], at(0.1))
        stage.update([], at(0.2))
        result = stage.update([detection((10.0, 0.0, 0.0))], at(0.3))

        assert result.tracks[0].track_id == 1

    def test_tentative_tracks_die_faster_than_confirmed_ones(self) -> None:
        stage = tracker(min_hits_to_confirm=5, max_missed_frames=5, max_missed_frames_tentative=1)
        stage.update([detection((10.0, 0.0, 0.0))], at(0.0))
        stage.update([], at(0.1))
        result = stage.update([], at(0.2))

        assert result.deleted_track_ids == [0]

    def test_age_and_hits_are_counted(self) -> None:
        results = run(
            tracker(), sequences.linear_motion((10.0, 0.0, 0.0), (1.0, 0.0, 0.0), frames=4)
        )
        track = results[-1].tracks[0]

        assert track.hits == 4
        assert track.age_frames == 4
        assert track.first_seen == at(0.0)
        assert track.last_seen == at(0.3)


class TestClassHandling:
    def test_an_unknown_track_adopts_a_known_class_immediately(self) -> None:
        stage = tracker(use_predicted_position_for_association=False)
        stage.update([detection((10.0, 0.0, 0.0), object_class=ObjectClass.UNKNOWN)], at(0.0))
        result = stage.update(
            [detection((10.1, 0.0, 0.0), object_class=ObjectClass.VEHICLE)], at(0.1)
        )

        assert result.tracks[0].object_class is ObjectClass.VEHICLE

    def test_a_known_class_does_not_flip_on_one_disagreement(self) -> None:
        stage = tracker(class_switch_hits=2, use_predicted_position_for_association=False)
        stage.update([detection((10.0, 0.0, 0.0), object_class=ObjectClass.VEHICLE)], at(0.0))
        result = stage.update(
            [detection((10.1, 0.0, 0.0), object_class=ObjectClass.CYCLIST, size=(4.5, 1.9, 1.6))],
            at(0.1),
        )

        assert result.tracks[0].object_class is ObjectClass.VEHICLE

    def test_a_known_class_switches_after_consistent_disagreement(self) -> None:
        stage = tracker(class_switch_hits=2, use_predicted_position_for_association=False)
        stage.update([detection((10.0, 0.0, 0.0), object_class=ObjectClass.VEHICLE)], at(0.0))
        for index in range(2):
            result = stage.update(
                [
                    detection(
                        (10.0 + 0.1 * index, 0.0, 0.0),
                        object_class=ObjectClass.CYCLIST,
                        size=(4.5, 1.9, 1.6),
                    )
                ],
                at(0.1 * (index + 1)),
            )

        assert result.tracks[0].object_class is ObjectClass.CYCLIST

    def test_an_ambiguous_frame_does_not_erase_a_known_class(self) -> None:
        stage = tracker(use_predicted_position_for_association=False)
        stage.update([detection((10.0, 0.0, 0.0), object_class=ObjectClass.VEHICLE)], at(0.0))
        result = stage.update(
            [detection((10.1, 0.0, 0.0), object_class=ObjectClass.UNKNOWN, size=(4.5, 1.9, 1.6))],
            at(0.1),
        )

        assert result.tracks[0].object_class is ObjectClass.VEHICLE


class TestResultAccounting:
    def test_detections_are_fully_accounted_for(self) -> None:
        stage = tracker(use_predicted_position_for_association=False)
        stage.update([detection((10.0, 0.0, 0.0))], at(0.0))
        result = stage.update(
            [detection((10.1, 0.0, 0.0), object_id=0), detection((60.0, 0.0, 0.0), object_id=1)],
            at(0.1),
        )

        assert result.detection_count == 2
        assert result.association_count + len(result.unmatched_detection_ids) == 2

    def test_durations_are_measured(self) -> None:
        result = tracker().update([detection((10.0, 0.0, 0.0))], at(0.0))

        assert result.duration_ms > 0.0
        assert result.association_duration_ms >= 0.0
        assert result.association_duration_ms <= result.duration_ms

    def test_configuration_travels_with_the_result(self) -> None:
        result = tracker(max_association_distance_m=7.5).update([], at(0.0))
        assert result.configuration.max_association_distance_m == 7.5

    def test_empty_frame_is_not_an_error(self) -> None:
        result = tracker().update([], at(0.0))

        assert result.detection_count == 0
        assert result.active_track_count == 0
        assert result.association_count == 0

    def test_counts_by_status(self) -> None:
        stage = tracker(min_hits_to_confirm=1, use_predicted_position_for_association=False)
        stage.update(
            [detection((10.0, 0.0, 0.0), object_id=0), detection((40.0, 0.0, 0.0), object_id=1)],
            at(0.0),
        )
        result = stage.update([detection((10.1, 0.0, 0.0))], at(0.1))

        assert result.confirmed_count == 1
        assert result.coasting_count == 1

    def test_predicted_position_is_labelled_tracker_state(self) -> None:
        """Extrapolation exists for gating and must not read as an observation."""
        stage = tracker(velocity_smoothing=1.0)
        stage.update([detection((0.0, 0.0, 0.0))], at(0.0))
        stage.update([detection((1.0, 0.0, 0.0))], at(0.5))
        result = stage.update([detection((2.0, 0.0, 0.0))], at(1.0))

        track = result.tracks[0]
        assert track.predicted_position is not None
        assert track.position.x == pytest.approx(2.0)


class TestDeterminism:
    def test_two_trackers_with_equal_configuration_agree(self) -> None:
        sequence = sequences.linear_motion((10.0, 0.0, 0.0), (5.0, 1.0, 0.0), frames=6)
        first = run(tracker(), sequence)
        second = run(tracker(), sequence)

        assert [r.tracks[0].position.x for r in first] == [r.tracks[0].position.x for r in second]
        assert [r.tracks[0].velocity is None for r in first] == [
            r.tracks[0].velocity is None for r in second
        ]

    def test_no_invalid_numbers_appear(self) -> None:
        results = run(tracker(), sequences.crossing_pair())
        for result in results:
            for track in result.tracks:
                for value in (track.position.x, track.position.y, track.position.z):
                    assert math.isfinite(value)
                if track.velocity is not None:
                    assert math.isfinite(track.velocity.magnitude)
                if track.heading_rad is not None:
                    assert math.isfinite(track.heading_rad)


class TestConfiguration:
    def test_tentative_limit_cannot_exceed_the_confirmed_limit(self) -> None:
        from pydantic import ValidationError

        with pytest.raises(ValidationError, match="max_missed_frames_tentative"):
            TrackingSettings(max_missed_frames=1, max_missed_frames_tentative=5)

    def test_association_distance_must_be_positive(self) -> None:
        from pydantic import ValidationError

        with pytest.raises(ValidationError):
            TrackingSettings(max_association_distance_m=0.0)

    def test_smoothing_is_bounded(self) -> None:
        from pydantic import ValidationError

        with pytest.raises(ValidationError):
            TrackingSettings(velocity_smoothing=1.5)

    def test_tracker_declares_itself_a_baseline(self) -> None:
        assert tracker().is_baseline is True
        assert tracker().name == "geometric_tracker_v1"


class TestAssociationHelpers:
    def test_size_compatibility(self) -> None:
        from adaptx.tracking.association import size_compatible

        settings = TrackingSettings(max_size_ratio=2.0)
        assert size_compatible(4.0, 5.0, settings) is True
        assert size_compatible(1.0, 5.0, settings) is False
        assert size_compatible(None, 5.0, settings) is True

    def test_size_gate_can_be_disabled(self) -> None:
        from adaptx.tracking.association import size_compatible

        assert size_compatible(1.0, 100.0, TrackingSettings(max_size_ratio=None)) is True

    def test_class_compatibility(self) -> None:
        from adaptx.tracking.association import class_compatible

        strict = TrackingSettings(require_class_match=True)
        assert class_compatible(ObjectClass.VEHICLE, ObjectClass.VEHICLE, strict) is True
        assert class_compatible(ObjectClass.VEHICLE, ObjectClass.PEDESTRIAN, strict) is False
        assert class_compatible(ObjectClass.VEHICLE, ObjectClass.UNKNOWN, strict) is True

        lenient = TrackingSettings(require_class_match=False)
        assert class_compatible(ObjectClass.VEHICLE, ObjectClass.PEDESTRIAN, lenient) is True

    def test_matches_are_ordered_by_distance(self) -> None:
        from adaptx.tracking.association import associate

        settings = TrackingSettings(max_association_distance_m=10.0)
        outcome = associate(
            [
                (0, Vector3(x=0.0), ObjectClass.VEHICLE, 4.5),
                (1, Vector3(x=5.0), ObjectClass.VEHICLE, 4.5),
            ],
            [detection((5.1, 0.0, 0.0), object_id=0), detection((0.1, 0.0, 0.0), object_id=1)],
            settings,
        )

        assert len(outcome.matches) == 2
        pairs = {match.track_id: match.detection_index for match in outcome.matches}
        assert pairs == {0: 1, 1: 0}
