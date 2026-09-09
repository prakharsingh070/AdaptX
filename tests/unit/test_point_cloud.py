"""Point-cloud contract and Phase 1 processor tests."""

from __future__ import annotations

from datetime import UTC, datetime

import numpy as np
import pytest
from pydantic import ValidationError

from adaptx.config.settings import LiDARSettings
from adaptx.core.exceptions import InvalidPointCloudError
from adaptx.models.common import DataSource
from adaptx.models.point_cloud import PointCloudFrame
from adaptx.perception.lidar import FrameValidationProcessor
from tests.fixtures.point_clouds import make_frame, make_points


def test_frame_reports_point_count_and_metadata() -> None:
    frame = make_frame(250, frame_id=7)
    assert frame.point_count == 250
    assert frame.frame_id == 7
    assert frame.sensor_id == "test_lidar"
    assert frame.source is DataSource.SYNTHETIC_TEST
    assert frame.has_intensity is False


def test_frame_accepts_intensity_column() -> None:
    frame = make_frame(10, columns=4)
    assert frame.has_intensity is True
    assert frame.fields == ("x", "y", "z", "intensity")


def test_bounds_match_the_underlying_points() -> None:
    points = np.array([[0.0, -1.0, 2.0], [3.0, 4.0, -5.0]])
    frame = PointCloudFrame(frame_id=0, sensor_id="s", points=points)
    bounds = frame.bounds()

    assert bounds is not None
    assert (bounds.min_x, bounds.max_x) == (0.0, 3.0)
    assert (bounds.min_y, bounds.max_y) == (-1.0, 4.0)
    assert (bounds.min_z, bounds.max_z) == (-5.0, 2.0)
    assert bounds.extent == (3.0, 5.0, 7.0)


def test_empty_frame_has_no_bounds() -> None:
    frame = PointCloudFrame(frame_id=0, sensor_id="s", points=np.empty((0, 3)))
    assert frame.point_count == 0
    assert frame.bounds() is None


@pytest.mark.parametrize(
    "points",
    [
        np.zeros((3,)),  # not 2D
        np.zeros((2, 2)),  # too few columns
        np.zeros((2, 5)),  # too many columns
        np.zeros((2, 3), dtype=np.int32),  # not floating
    ],
)
def test_malformed_arrays_are_rejected(points: np.ndarray) -> None:
    with pytest.raises(ValidationError):
        PointCloudFrame(frame_id=0, sensor_id="s", points=points)


@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf])
def test_non_finite_points_are_rejected(bad: float) -> None:
    points = np.array([[0.0, 0.0, 0.0], [bad, 1.0, 1.0]])
    with pytest.raises(ValidationError, match="NaN or infinite"):
        PointCloudFrame(frame_id=0, sensor_id="s", points=points)


def test_naive_timestamp_is_rejected() -> None:
    with pytest.raises(ValidationError, match="timezone-aware"):
        PointCloudFrame(
            frame_id=0,
            sensor_id="s",
            points=make_points(3),
            timestamp=datetime(2026, 1, 1, 0, 0, 0),  # deliberately naive
        )


def test_timestamp_is_normalised_to_utc() -> None:
    frame = make_frame(3)
    assert frame.timestamp.tzinfo is not None
    assert frame.timestamp.utcoffset() == UTC.utcoffset(None)


def test_field_layout_must_match_column_count() -> None:
    with pytest.raises(ValidationError, match="do not match"):
        PointCloudFrame(
            frame_id=0,
            sensor_id="s",
            points=make_points(4, columns=3),
            fields=("x", "y", "z", "intensity"),
        )


def test_from_sequence_rejects_ragged_input() -> None:
    with pytest.raises(ValueError, match="numeric 2D array"):
        PointCloudFrame.from_sequence([[0.0, 0.0, 0.0], [1.0, 1.0]], frame_id=0, sensor_id="s")


def test_from_sequence_infers_the_field_layout() -> None:
    frame = PointCloudFrame.from_sequence(
        [[0.0, 1.0, 2.0, 0.5]], frame_id=0, sensor_id="s", source=DataSource.SYNTHETIC_TEST
    )
    assert frame.fields == ("x", "y", "z", "intensity")
    assert frame.point_count == 1


def test_summary_preserves_frame_metadata() -> None:
    frame = make_frame(12, frame_id=3)
    summary = frame.summary()

    assert summary.frame_id == frame.frame_id
    assert summary.sensor_id == frame.sensor_id
    assert summary.timestamp == frame.timestamp
    assert summary.point_count == 12
    assert summary.source is frame.source
    assert summary.bounds is not None


class TestFrameValidationProcessor:
    """Phase 1 processor: validates and passes through, never transforms."""

    @staticmethod
    def _processor(**overrides: int) -> FrameValidationProcessor:
        return FrameValidationProcessor(LiDARSettings(**overrides))

    def test_passes_a_valid_frame_through_unchanged(self) -> None:
        frame = make_frame(50)
        processed = self._processor(min_points=1, max_points=100).process(frame)

        assert processed is frame
        assert processed.point_count == 50

    def test_rejects_a_frame_below_the_minimum(self) -> None:
        frame = PointCloudFrame(frame_id=0, sensor_id="s", points=np.empty((0, 3)))
        with pytest.raises(InvalidPointCloudError) as excinfo:
            self._processor(min_points=1).process(frame)
        assert excinfo.value.details["min_points"] == 1

    def test_rejects_a_frame_above_the_maximum(self) -> None:
        frame = make_frame(50)
        with pytest.raises(InvalidPointCloudError) as excinfo:
            self._processor(max_points=10).process(frame)
        assert excinfo.value.details["point_count"] == 50

    def test_processed_frame_summarises_with_bounds(self) -> None:
        summary = self._processor(max_points=1000).process(make_frame(20)).summary()
        assert summary.point_count == 20
        assert summary.bounds is not None
