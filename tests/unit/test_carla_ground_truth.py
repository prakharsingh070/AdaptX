"""Simulator ground-truth contract tests (Phase 9).

Ground truth is what the simulator *knows*. These contracts exist so it can
never be confused with what perception *inferred* - which is the confusion
that would quietly invalidate every accuracy figure Phase 11 goes on to
produce.
"""

from __future__ import annotations

from datetime import UTC, datetime

import pytest
from pydantic import ValidationError

from adaptx.carla.ground_truth import (
    GroundTruthActor,
    GroundTruthFrame,
    build_ground_truth_frame,
    classify_blueprint,
)
from adaptx.models.common import DataSource, Dimensions, ObjectClass, Vector3

EPOCH = datetime(2000, 1, 1, tzinfo=UTC)


def actor(
    actor_id: int = 1,
    *,
    type_id: str = "vehicle.audi.tt",
    x: float = 20.0,
    y: float = 0.0,
    is_ego: bool = False,
) -> GroundTruthActor:
    """A ground-truth actor with exact, stated values."""
    return GroundTruthActor(
        actor_id=actor_id,
        type_id=type_id,
        object_class=classify_blueprint(type_id),
        is_ego=is_ego,
        position=Vector3(x=x, y=y, z=0.0),
        velocity=Vector3(x=-8.0, y=0.0, z=0.0),
        heading_rad=0.0,
        dimensions=Dimensions(length=4.5, width=1.9, height=1.6),
        world_position=Vector3(x=x, y=y, z=0.0),
        distance_m=(x**2 + y**2) ** 0.5,
    )


class TestBlueprintClassification:
    @pytest.mark.parametrize(
        ("type_id", "expected"),
        [
            ("vehicle.tesla.model3", ObjectClass.VEHICLE),
            ("vehicle.audi.tt", ObjectClass.VEHICLE),
            ("walker.pedestrian.0001", ObjectClass.PEDESTRIAN),
            ("vehicle.diamondback.century", ObjectClass.CYCLIST),
            ("vehicle.harley-davidson.low_rider", ObjectClass.CYCLIST),
            ("static.prop.streetbarrier", ObjectClass.OBSTACLE),
            ("traffic.traffic_light", ObjectClass.OBSTACLE),
        ],
    )
    def test_known_blueprints_map_to_classes(self, type_id: str, expected: ObjectClass) -> None:
        assert classify_blueprint(type_id) is expected

    def test_two_wheelers_beat_the_general_vehicle_prefix(self) -> None:
        """CARLA files bicycles under `vehicle.`, so order matters."""
        assert classify_blueprint("vehicle.gazelle.omafiets") is ObjectClass.CYCLIST

    def test_an_unrecognised_blueprint_is_unknown_not_guessed(self) -> None:
        assert classify_blueprint("something.entirely.new") is ObjectClass.UNKNOWN

    def test_classification_is_case_insensitive(self) -> None:
        assert classify_blueprint("VEHICLE.Audi.TT") is ObjectClass.VEHICLE


class TestGroundTruthActorContract:
    def test_an_actor_carries_exact_values(self) -> None:
        record = actor()
        assert record.speed_mps == pytest.approx(8.0)
        assert record.object_class is ObjectClass.VEHICLE

    def test_there_is_no_confidence_field(self) -> None:
        """The simulator is not estimating, so nothing here is uncertain.

        A confidence on ground truth would be meaningless, and its presence
        would invite treating this like a detection.
        """
        assert "confidence" not in GroundTruthActor.model_fields

    def test_it_is_not_a_perception_contract(self) -> None:
        """Reusing DetectedObject would make a known quantity look inferred."""
        from adaptx.models.objects import DetectedObject

        assert GroundTruthActor is not DetectedObject
        assert not issubclass(GroundTruthActor, DetectedObject)

    def test_unknown_fields_are_rejected(self) -> None:
        with pytest.raises(ValidationError):
            GroundTruthActor(
                actor_id=1,
                type_id="vehicle.audi.tt",
                object_class=ObjectClass.VEHICLE,
                position=Vector3(),
                velocity=Vector3(),
                heading_rad=0.0,
                dimensions=Dimensions(length=1.0, width=1.0, height=1.0),
                world_position=Vector3(),
                distance_m=0.0,
                detection_confidence=0.9,  # type: ignore[call-arg]
            )


class TestGroundTruthFrame:
    def frame(self) -> GroundTruthFrame:
        return build_ground_truth_frame(
            frame_id=42,
            timestamp=EPOCH,
            map_name="Town03",
            actors=[
                actor(3, x=30.0),
                actor(1, is_ego=True, x=0.0, type_id="vehicle.tesla.model3"),
                actor(2, x=12.0, type_id="walker.pedestrian.0001"),
            ],
            ego_actor_id=1,
        )

    def test_a_frame_is_always_labelled_simulation(self) -> None:
        """Ground truth cannot come from a real sensor."""
        assert self.frame().source is DataSource.SIMULATION

    def test_actors_are_ordered_deterministically(self) -> None:
        assert [a.actor_id for a in self.frame().actors] == [1, 2, 3]

    def test_the_ego_is_excluded_from_others(self) -> None:
        others = self.frame().others()
        assert [a.actor_id for a in others] == [2, 3]
        assert all(not a.is_ego for a in others)

    def test_the_nearest_actor_excludes_the_ego(self) -> None:
        """The ego is at distance zero, and is never its own nearest object."""
        nearest = self.frame().nearest()
        assert nearest is not None
        assert nearest.actor_id == 2

    def test_nearest_is_none_when_the_ego_is_alone(self) -> None:
        alone = build_ground_truth_frame(
            frame_id=1,
            timestamp=EPOCH,
            map_name="Town03",
            actors=[actor(1, is_ego=True, x=0.0)],
            ego_actor_id=1,
        )
        assert alone.nearest() is None
        assert alone.others() == []

    def test_ties_break_on_actor_id_so_nearest_is_deterministic(self) -> None:
        tied = build_ground_truth_frame(
            frame_id=1,
            timestamp=EPOCH,
            map_name="Town03",
            actors=[actor(9, x=10.0), actor(4, x=10.0)],
            ego_actor_id=None,
        )
        nearest = tied.nearest()
        assert nearest is not None
        assert nearest.actor_id == 4

    def test_actors_can_be_filtered_by_class(self) -> None:
        pedestrians = self.frame().by_class(ObjectClass.PEDESTRIAN)
        assert [a.actor_id for a in pedestrians] == [2]

    def test_the_frame_counts_include_the_ego(self) -> None:
        assert self.frame().actor_count == 3

    def test_a_frame_records_which_simulator_frame_it_belongs_to(self) -> None:
        """This is what makes ground truth joinable to the LiDAR frame later."""
        frame = self.frame()
        assert frame.frame_id == 42
        assert frame.timestamp == EPOCH
