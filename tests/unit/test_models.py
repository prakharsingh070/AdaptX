"""Data-contract tests for the vehicle, object, tracking, prediction, map and
risk models."""

from __future__ import annotations

import math
from datetime import UTC, datetime, timedelta

import pytest
from pydantic import ValidationError

from adaptx.models.common import (
    SCHEMA_VERSION,
    BoundingBox3D,
    Dimensions,
    ObjectClass,
    Vector3,
)
from adaptx.models.map import (
    AdaptiveMap,
    AdaptiveMapCell,
    OccupancyState,
    ResolutionContext,
    ResolutionLevel,
)
from adaptx.models.objects import DetectedObject
from adaptx.models.prediction import PredictedTrajectory, TrajectoryPoint
from adaptx.models.risk import ObjectRisk, RiskCell, RiskField, RiskLevel
from adaptx.models.tracking import TrackedObject, TrackStatus
from adaptx.models.vehicle import VehicleState


def _box() -> BoundingBox3D:
    return BoundingBox3D(
        center=Vector3(x=1.0, y=2.0, z=0.5),
        dimensions=Dimensions(length=4.0, width=2.0, height=1.5),
    )


class TestCommon:
    def test_vector_magnitude_and_distance(self) -> None:
        assert Vector3(x=3.0, y=4.0).magnitude == pytest.approx(5.0)
        assert Vector3().distance_to(Vector3(x=1.0, y=1.0)) == pytest.approx(math.sqrt(2))

    def test_non_finite_components_are_rejected(self) -> None:
        with pytest.raises(ValidationError, match="finite"):
            Vector3(x=float("nan"))

    def test_dimensions_must_be_positive(self) -> None:
        with pytest.raises(ValidationError):
            Dimensions(length=0.0, width=1.0, height=1.0)

    def test_bounding_box_volume(self) -> None:
        assert _box().volume_m3 == pytest.approx(12.0)

    def test_unknown_fields_are_rejected(self) -> None:
        with pytest.raises(ValidationError):
            Vector3(x=1.0, w=2.0)


class TestVehicleState:
    def test_defaults_and_speed(self) -> None:
        state = VehicleState(velocity=Vector3(x=3.0, y=4.0))
        assert state.speed_mps == pytest.approx(5.0)
        assert state.schema_version == SCHEMA_VERSION
        assert state.timestamp.tzinfo is not None

    def test_dimensions_are_configurable(self) -> None:
        state = VehicleState(dimensions=Dimensions(length=5.0, width=2.0, height=1.8))
        assert state.dimensions.length == 5.0

    def test_heading_must_be_finite(self) -> None:
        with pytest.raises(ValidationError, match="finite"):
            VehicleState(heading_rad=float("inf"))


class TestDetectedObject:
    def test_valid_detection(self) -> None:
        detection = DetectedObject(
            object_id=1,
            frame_id=10,
            object_class=ObjectClass.PEDESTRIAN,
            position=Vector3(x=5.0),
            bounding_box=_box(),
            confidence=0.87,
        )
        assert detection.object_class is ObjectClass.PEDESTRIAN
        assert detection.velocity is None

    @pytest.mark.parametrize("confidence", [-0.1, 1.1])
    def test_confidence_is_bounded(self, confidence: float) -> None:
        with pytest.raises(ValidationError):
            DetectedObject(
                object_id=1,
                frame_id=0,
                position=Vector3(),
                bounding_box=_box(),
                confidence=confidence,
            )


class TestTrackedObject:
    def test_defaults(self) -> None:
        track = TrackedObject(track_id=3, position=Vector3(x=2.0), confidence=0.5)
        assert track.status is TrackStatus.TENTATIVE
        assert track.age_frames == 1
        assert track.missed_frames == 0
        assert track.speed_mps == pytest.approx(0.0)
        assert track.last_seen is None

    def test_uncertainty_is_bounded(self) -> None:
        with pytest.raises(ValidationError):
            TrackedObject(track_id=0, position=Vector3(), confidence=0.5, uncertainty=1.5)

    def test_timestamp_must_be_timezone_aware(self) -> None:
        """Regression: a same-named subclass validator once disabled this check."""
        with pytest.raises(ValidationError, match="timezone-aware"):
            TrackedObject(
                track_id=0,
                position=Vector3(),
                confidence=0.5,
                timestamp=datetime(2026, 1, 1),  # deliberately naive
            )

    def test_last_seen_must_be_timezone_aware(self) -> None:
        with pytest.raises(ValidationError, match="timezone-aware"):
            TrackedObject(
                track_id=0,
                position=Vector3(),
                confidence=0.5,
                last_seen=datetime(2026, 1, 1),  # deliberately naive
            )

    def test_status_transitions_are_representable(self) -> None:
        track = TrackedObject(track_id=0, position=Vector3(), confidence=0.9)
        track.status = TrackStatus.CONFIRMED
        assert track.status is TrackStatus.CONFIRMED


class TestPredictedTrajectory:
    @staticmethod
    def _trajectory(offsets: list[float], horizon: float = 3.0) -> PredictedTrajectory:
        return PredictedTrajectory(
            track_id=1,
            horizon_s=horizon,
            timestep_s=1.0,
            confidence=0.7,
            predictor_name="test_predictor",
            points=[
                TrajectoryPoint(time_offset_s=offset, position=Vector3(x=offset), confidence=0.7)
                for offset in offsets
            ],
        )

    def test_valid_trajectory(self) -> None:
        trajectory = self._trajectory([1.0, 2.0, 3.0])
        assert len(trajectory.points) == 3
        assert trajectory.predictor_name == "test_predictor"

    def test_points_beyond_the_horizon_are_rejected(self) -> None:
        with pytest.raises(ValidationError, match="exceeds horizon"):
            self._trajectory([1.0, 5.0])

    def test_points_must_be_time_ordered(self) -> None:
        with pytest.raises(ValidationError, match="increasing time_offset_s"):
            self._trajectory([2.0, 1.0])

    def test_at_least_one_point_is_required(self) -> None:
        with pytest.raises(ValidationError):
            self._trajectory([])


class TestMapModels:
    def test_cell_defaults(self) -> None:
        cell = AdaptiveMapCell(x=1.0, y=2.0, resolution_m=0.5, occupancy=0.9)
        assert cell.resolution_level is ResolutionLevel.LOW
        assert cell.occupancy_state is OccupancyState.UNKNOWN
        assert cell.height_m is None

    def test_height_range_must_be_ordered(self) -> None:
        with pytest.raises(ValidationError, match="height_min_m"):
            AdaptiveMapCell(
                x=0.0,
                y=0.0,
                resolution_m=0.5,
                occupancy=0.1,
                height_min_m=2.0,
                height_max_m=1.0,
            )

    def test_occupancy_is_a_probability(self) -> None:
        with pytest.raises(ValidationError):
            AdaptiveMapCell(x=0.0, y=0.0, resolution_m=0.5, occupancy=1.2)

    def test_map_aggregates(self) -> None:
        cells = [
            AdaptiveMapCell(x=0.0, y=0.0, resolution_m=1.0, occupancy=0.0),
            AdaptiveMapCell(x=1.0, y=0.0, resolution_m=0.5, occupancy=0.0),
        ]
        adaptive = AdaptiveMap(frame_id=0, is_adaptive=True, range_m=60.0, cells=cells)
        assert adaptive.cell_count == 2
        assert adaptive.average_resolution_m == pytest.approx(0.75)

    def test_empty_map_has_no_average_resolution(self) -> None:
        empty = AdaptiveMap(frame_id=0, is_adaptive=False, range_m=60.0)
        assert empty.cell_count == 0
        assert empty.average_resolution_m is None

    def test_resolution_context_carries_the_decision_inputs(self) -> None:
        context = ResolutionContext(
            x=10.0,
            y=-4.0,
            distance_from_ego_m=11.0,
            risk_score=0.8,
            predicted_risk_score=0.9,
            uncertainty=0.3,
            object_density=0.02,
            max_object_speed_mps=6.0,
            in_ego_path=True,
            current_level=ResolutionLevel.MEDIUM,
        )
        assert context.in_ego_path is True
        assert context.current_level is ResolutionLevel.MEDIUM


class TestRiskModels:
    def test_cell_percent_view(self) -> None:
        cell = RiskCell(x=0.0, y=0.0, resolution_m=0.5, risk_score=0.82)
        assert cell.risk_percent == pytest.approx(82.0)
        assert cell.risk_level is RiskLevel.LOW  # level is set by the engine, not derived

    def test_risk_score_is_normalised(self) -> None:
        with pytest.raises(ValidationError):
            RiskCell(x=0.0, y=0.0, resolution_m=0.5, risk_score=82.0)

    def test_field_reports_the_maximum(self) -> None:
        field = RiskField(
            engine="test",
            is_baseline=True,
            cells=[RiskCell(x=0.0, y=0.0, resolution_m=1.0, risk_score=0.2)],
            object_risks=[ObjectRisk(track_id=1, risk_score=0.7)],
        )
        assert field.max_risk == pytest.approx(0.7)

    def test_empty_field_maximum_is_zero(self) -> None:
        assert RiskField(engine="test").max_risk == 0.0

    def test_uncertainty_is_independent_of_risk(self) -> None:
        """A low-risk object can still be highly uncertain."""
        risk = ObjectRisk(track_id=1, risk_score=0.05, uncertainty=0.95)
        assert risk.risk_score < risk.uncertainty


def test_timestamps_are_comparable_across_models() -> None:
    now = datetime.now(tz=UTC)
    earlier = now - timedelta(seconds=1)
    first = ObjectRisk(track_id=0, risk_score=0.1, timestamp=earlier)
    second = ObjectRisk(track_id=0, risk_score=0.1, timestamp=now)
    assert first.timestamp < second.timestamp
