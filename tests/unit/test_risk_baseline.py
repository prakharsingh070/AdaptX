"""Baseline risk calculator tests.

These verify the *baseline* only. It is not the ADAPT-X risk engine and these
tests must not be read as validating the eventual risk formulation.
"""

from __future__ import annotations

import pytest

from adaptx.config.settings import RiskSettings
from adaptx.models.common import DataSource, Vector3
from adaptx.models.risk import RiskLevel
from adaptx.models.tracking import TrackedObject
from adaptx.models.vehicle import VehicleState
from adaptx.risk.baseline import BaselineProximityRiskEngine


@pytest.fixture
def engine() -> BaselineProximityRiskEngine:
    return BaselineProximityRiskEngine(RiskSettings(max_range_m=100.0))


def _track(track_id: int, x: float, *, uncertainty: float = 0.0) -> TrackedObject:
    return TrackedObject(
        track_id=track_id,
        position=Vector3(x=x),
        confidence=0.9,
        uncertainty=uncertainty,
        source=DataSource.SYNTHETIC_TEST,
    )


def test_engine_declares_itself_a_baseline(engine: BaselineProximityRiskEngine) -> None:
    assert engine.is_baseline is True
    assert engine.name == "baseline_proximity"


def test_risk_decreases_with_distance(engine: BaselineProximityRiskEngine) -> None:
    field = engine.evaluate([_track(1, 10.0), _track(2, 50.0), _track(3, 90.0)])
    scores = [risk.risk_score for risk in field.object_risks]

    assert scores == sorted(scores, reverse=True)
    assert scores[0] == pytest.approx(0.9)
    assert scores[1] == pytest.approx(0.5)
    assert scores[2] == pytest.approx(0.1)


def test_risk_is_clamped_beyond_max_range(engine: BaselineProximityRiskEngine) -> None:
    field = engine.evaluate([_track(1, 500.0)])
    assert field.object_risks[0].risk_score == 0.0


def test_risk_is_measured_from_the_ego_position(engine: BaselineProximityRiskEngine) -> None:
    ego = VehicleState(position=Vector3(x=40.0), source=DataSource.SYNTHETIC_TEST)
    field = engine.evaluate([_track(1, 50.0)], ego_state=ego)
    assert field.object_risks[0].risk_score == pytest.approx(0.9)


def test_field_is_deterministic(engine: BaselineProximityRiskEngine) -> None:
    tracks = [_track(1, 12.5), _track(2, 33.0)]
    first = engine.evaluate(tracks)
    second = engine.evaluate(tracks)
    assert [r.risk_score for r in first.object_risks] == [r.risk_score for r in second.object_risks]


def test_field_is_flagged_as_baseline_and_has_no_cells(
    engine: BaselineProximityRiskEngine,
) -> None:
    field = engine.evaluate([_track(1, 10.0)])
    assert field.is_baseline is True
    assert field.engine == "baseline_proximity"
    assert field.cells == []  # this baseline produces no spatial field


@pytest.mark.parametrize(
    ("score", "expected"),
    [
        (0.0, RiskLevel.LOW),
        (0.34, RiskLevel.LOW),
        (0.35, RiskLevel.MEDIUM),
        (0.59, RiskLevel.MEDIUM),
        (0.60, RiskLevel.HIGH),
        (0.84, RiskLevel.HIGH),
        (0.85, RiskLevel.CRITICAL),
        (1.0, RiskLevel.CRITICAL),
    ],
)
def test_classification_thresholds(
    engine: BaselineProximityRiskEngine, score: float, expected: RiskLevel
) -> None:
    assert engine.classify(score) is expected


def test_classify_rejects_unnormalised_scores(engine: BaselineProximityRiskEngine) -> None:
    with pytest.raises(ValueError, match=r"\[0, 1\]"):
        engine.classify(82.0)


def test_uncertainty_passes_through_without_affecting_risk(
    engine: BaselineProximityRiskEngine,
) -> None:
    field = engine.evaluate([_track(1, 10.0, uncertainty=0.75)])
    risk = field.object_risks[0]
    assert risk.uncertainty == pytest.approx(0.75)
    assert risk.risk_score == pytest.approx(0.9)


def test_provenance_is_preserved(engine: BaselineProximityRiskEngine) -> None:
    field = engine.evaluate([_track(1, 10.0)])
    assert field.object_risks[0].source is DataSource.SYNTHETIC_TEST
