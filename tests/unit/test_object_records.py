"""Per-object records the backend builds for the dashboard, and the corridor rule."""

from __future__ import annotations

import pytest

from adaptx.control.corridor import PathRelation, governs, in_corridor, path_relation
from adaptx.models.common import ObjectClass, Vector3
from adaptx.models.risk import RiskLevel
from adaptx.models.scene import SceneSnapshot, TrackedObjectSnapshot
from adaptx.services.scene_service import object_records
from tests.fixtures.evaluation import FrameSpec, RunBuilder, TrackSpec
from tests.unit.test_scene_service import snapshot_from, synthetic_outputs


def v(x: float, y: float) -> Vector3:
    return Vector3(x=x, y=y, z=0.0)


class TestCorridor:
    def test_ahead_and_inside_is_in_path(self) -> None:
        assert in_corridor(v(10.0, 1.0), 1.8)
        assert path_relation(v(10.0, -1.7), [], 1.8) is PathRelation.IN_PATH

    def test_beside_the_corridor_is_outside_and_behind_is_behind(self) -> None:
        assert path_relation(v(10.0, 4.0), [], 1.8) is PathRelation.OUTSIDE
        assert path_relation(v(-3.0, 0.0), [], 1.8) is PathRelation.BEHIND

    def test_a_predicted_entry_is_crossing_and_governs(self) -> None:
        relation = path_relation(v(8.0, 6.0), [v(8.0, 4.0), v(8.0, 1.0)], 1.8)
        assert relation is PathRelation.CROSSING
        assert governs(relation) and governs(PathRelation.IN_PATH)
        assert not governs(PathRelation.OUTSIDE) and not governs(PathRelation.BEHIND)

    def test_a_prediction_that_stays_outside_does_not_cross(self) -> None:
        assert path_relation(v(8.0, 6.0), [v(9.0, 5.5), v(10.0, 5.0)], 1.8) is PathRelation.OUTSIDE


def outputs(objects, trajectories=None):  # type: ignore[no-untyped-def]
    record = (
        RunBuilder({"a": 1})
        .frame(
            FrameSpec(
                tracks=[
                    TrackSpec(i, x, y, velocity=vel, object_class=cls)
                    for i, x, y, cls, vel in objects
                ],
                risk=[],
                trajectories=trajectories or {},
            )
        )
        .build()
    )
    produced = record.frames[0].outputs
    assert produced is not None
    return produced


class TestObjectRecords:
    def test_every_field_is_a_pipeline_output_joined_by_track_id(self) -> None:
        produced = synthetic_outputs()  # tracks 0 (HIGH 0.7, path) and 1 (UNKNOWN, no velocity)
        records = {
            r.track_id: r
            for r in object_records(produced.tracking, produced.prediction, produced.risk, 1.8)
        }
        zero, one = records[0], records[1]
        assert zero.longitudinal_distance_m == 10.0 and zero.lateral_distance_m == 0.0
        assert zero.distance_m == pytest.approx(10.0)
        assert zero.risk_level is RiskLevel.HIGH and zero.risk_score == 0.7
        assert zero.path_relation is PathRelation.IN_PATH and zero.in_ego_path
        assert zero.predicted_points == 2 and zero.predicted_horizon_s is not None
        assert zero.speed_mps == 0.0, "a measured standstill is 0.0, not null"
        # Track 1: velocity unobserved, UNKNOWN risk, beside the path.
        assert one.speed_mps is None
        assert one.risk_level is RiskLevel.UNKNOWN and one.risk_score is None
        assert one.longitudinal_distance_m == 12.0 and one.lateral_distance_m == 2.0
        assert one.path_relation is PathRelation.OUTSIDE and not one.in_ego_path
        assert one.predicted_points == 0

    def test_confidence_is_null_for_an_unknown_class(self) -> None:
        produced = outputs(
            [
                (0, 10.0, 0.0, ObjectClass.UNKNOWN, (0.0, 0.0)),
                (1, 12.0, 0.0, ObjectClass.VEHICLE, (1.0, 0.0)),
            ]
        )
        records = {
            r.track_id: r
            for r in object_records(produced.tracking, produced.prediction, produced.risk, 1.8)
        }
        assert records[0].confidence is None
        assert records[1].confidence == pytest.approx(0.8)

    def test_a_track_without_an_assessment_has_null_distance_and_unknown_risk(self) -> None:
        produced = outputs([(3, 15.0, 1.0, ObjectClass.VEHICLE, (1.0, 0.0))])
        (record,) = object_records(produced.tracking, produced.prediction, produced.risk, 1.8)
        assert record.distance_m is None and record.relative_speed_mps is None
        assert record.risk_level is RiskLevel.UNKNOWN
        assert record.longitudinal_distance_m == 15.0, "position is still the track's"

    def test_the_snapshot_carries_the_records_and_refuses_orphans(self) -> None:
        snapshot = snapshot_from(synthetic_outputs())
        assert {r.track_id for r in snapshot.objects} == {0, 1}
        payload = snapshot.model_dump(mode="json")
        record = next(o for o in payload["objects"] if o["track_id"] == 1)
        assert record["speed_mps"] is None and record["risk_level"] == "unknown"
        assert record["path_relation"] == "OUTSIDE"
        payload["objects"].append({**record, "track_id": 99})
        with pytest.raises(ValueError, match="has no track"):
            SceneSnapshot.model_validate(payload)

    def test_records_round_trip_as_json(self) -> None:
        snapshot = snapshot_from(synthetic_outputs())
        rebuilt = SceneSnapshot.model_validate_json(snapshot.model_dump_json())
        assert rebuilt.objects == snapshot.objects
        assert all(isinstance(o, TrackedObjectSnapshot) for o in rebuilt.objects)
