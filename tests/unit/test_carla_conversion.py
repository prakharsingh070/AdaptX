"""CARLA-to-ADAPT-X conversion tests (Phase 9).

The conversion is the part of the CARLA boundary that can be wrong *silently*.
A dropped sign flip mirrors the world: every object appears on the wrong side,
nothing raises, and the pipeline happily produces confident output about a
scene that never existed. So these are the most exhaustive tests in the phase,
and every expected value is derived by hand from ADR-009 rather than from
what the code happens to return.

CARLA: left-handed, +x forward, **+y right**, +z up.
ADAPT-X: right-handed, +x forward, **+y left**, +z up.

None of this needs CARLA installed - :mod:`adaptx.carla.conversion` imports no
simulator, which is exactly why the risky part is testable everywhere.
"""

from __future__ import annotations

import math
import struct
from datetime import UTC, datetime, timedelta
from itertools import pairwise

import numpy as np
import pytest

from adaptx.carla.conversion import (
    CARLA_LIDAR_STRIDE,
    SIMULATION_EPOCH,
    adaptx_offset_to_carla,
    build_raw_frame,
    carla_location_to_vector3,
    carla_points_to_adaptx,
    carla_vector_to_ego,
    carla_world_to_ego,
    carla_yaw_to_heading_rad,
    decode_lidar_buffer,
    ego_offset_to_carla_world,
    flip_y,
    simulation_timestamp,
)
from adaptx.core.exceptions import InvalidPointCloudError
from adaptx.models.common import CoordinateFrame, DataSource
from adaptx.models.point_cloud import XYZ_FIELDS, XYZI_FIELDS


def pack(points: list[list[float]]) -> bytes:
    """Pack points the way CARLA does: flat little-endian float32."""
    flat = [value for point in points for value in point]
    return struct.pack(f"<{len(flat)}f", *flat)


class TestAxisConversion:
    def test_y_is_mirrored_and_nothing_else_moves(self) -> None:
        """The whole conversion, stated once: negate y, leave x and z alone."""
        assert flip_y(1.0, 2.0, 3.0) == (1.0, -2.0, 3.0)

    def test_a_point_to_carlas_right_is_to_adaptx_right(self) -> None:
        """CARLA +y is right; ADAPT-X right is negative y."""
        assert flip_y(0.0, 5.0, 0.0) == (0.0, -5.0, 0.0)

    def test_a_point_to_carlas_left_is_to_adaptx_left(self) -> None:
        assert flip_y(0.0, -5.0, 0.0) == (0.0, 5.0, 0.0)

    def test_forward_is_unchanged(self) -> None:
        """Both frames put +x forward, so a point ahead stays ahead."""
        assert flip_y(10.0, 0.0, 0.0) == (10.0, 0.0, 0.0)

    def test_up_is_unchanged(self) -> None:
        assert flip_y(0.0, 0.0, 2.0) == (0.0, 0.0, 2.0)

    def test_the_conversion_is_its_own_inverse(self) -> None:
        """A mirror applied twice is the identity - which is why one direction
        of travel needs no separate function."""
        assert flip_y(*flip_y(3.0, -7.0, 1.5)) == (3.0, -7.0, 1.5)

    def test_handedness_actually_changes(self) -> None:
        """x cross y must point along +z in a right-handed frame.

        The property that makes this a handedness change rather than a
        relabelling: in CARLA's left-handed frame the same cross product
        points the other way.
        """
        x_axis = np.array(flip_y(1.0, 0.0, 0.0))
        y_axis = np.array(flip_y(0.0, 1.0, 0.0))  # CARLA +y (right) -> ADAPT-X -y
        assert np.allclose(np.cross(x_axis, y_axis), [0.0, 0.0, -1.0])

        # And the ADAPT-X basis itself is right-handed.
        assert np.allclose(np.cross([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]), [0.0, 0.0, 1.0])

    def test_negative_coordinates_are_ordinary(self) -> None:
        assert flip_y(-12.0, -4.0, -1.0) == (-12.0, 4.0, -1.0)

    def test_a_vector3_carries_the_converted_values(self) -> None:
        vector = carla_location_to_vector3(2.0, 3.0, 4.0)
        assert (vector.x, vector.y, vector.z) == (2.0, -3.0, 4.0)


class TestYawConversion:
    def test_zero_yaw_is_zero_heading(self) -> None:
        assert carla_yaw_to_heading_rad(0.0) == pytest.approx(0.0)

    def test_carla_yaw_of_ninety_points_right_which_is_negative_in_adaptx(self) -> None:
        """CARLA +90 deg faces CARLA +y (right); ADAPT-X calls that -pi/2."""
        assert carla_yaw_to_heading_rad(90.0) == pytest.approx(-math.pi / 2)

    def test_carla_yaw_of_minus_ninety_points_left(self) -> None:
        assert carla_yaw_to_heading_rad(-90.0) == pytest.approx(math.pi / 2)

    def test_a_half_turn_is_a_half_turn_either_way(self) -> None:
        assert abs(carla_yaw_to_heading_rad(180.0)) == pytest.approx(math.pi)

    def test_the_rotation_sense_matches_the_axis_flip(self) -> None:
        """A heading must point where the flipped axis points.

        Rotating the forward axis by the converted heading has to land on the
        same place the raw position conversion would - otherwise positions and
        orientations disagree and objects face the wrong way.
        """
        heading = carla_yaw_to_heading_rad(90.0)
        facing = (math.cos(heading), math.sin(heading))
        expected = flip_y(0.0, 1.0, 0.0)[:2]  # CARLA +y direction, converted
        assert facing == pytest.approx(expected, abs=1e-12)

    def test_a_non_finite_yaw_is_rejected(self) -> None:
        with pytest.raises(InvalidPointCloudError, match="finite"):
            carla_yaw_to_heading_rad(float("nan"))


class TestPointArrayConversion:
    def test_a_three_column_array_is_converted(self) -> None:
        points = np.array([[1.0, 2.0, 3.0], [-4.0, -5.0, -6.0]])
        assert np.allclose(carla_points_to_adaptx(points), [[1.0, -2.0, 3.0], [-4.0, 5.0, -6.0]])

    def test_intensity_is_passed_through_untouched(self) -> None:
        """A fourth column is a measurement, not a coordinate."""
        points = np.array([[1.0, 2.0, 3.0, 0.9]])
        assert np.allclose(carla_points_to_adaptx(points), [[1.0, -2.0, 3.0, 0.9]])

    def test_the_input_array_is_not_modified(self) -> None:
        """A caller may still want the CARLA-frame array afterwards."""
        points = np.array([[1.0, 2.0, 3.0]])
        carla_points_to_adaptx(points)
        assert np.allclose(points, [[1.0, 2.0, 3.0]])

    def test_an_empty_scan_converts_to_an_empty_array(self) -> None:
        converted = carla_points_to_adaptx(np.empty((0, 4)))
        assert converted.shape == (0, 4)

    def test_a_wrongly_shaped_array_is_rejected(self) -> None:
        with pytest.raises(InvalidPointCloudError, match="N, 3"):
            carla_points_to_adaptx(np.zeros((5, 2)))

    def test_conversion_is_vectorised_over_a_large_scan(self) -> None:
        """No per-point Python work: the whole scan is one column operation."""
        points = np.random.default_rng(7).uniform(-50, 50, size=(120_000, 4))
        converted = carla_points_to_adaptx(points)
        assert np.allclose(converted[:, 1], -points[:, 1])
        assert np.allclose(converted[:, 0], points[:, 0])


class TestLidarBufferDecoding:
    def test_a_buffer_decodes_to_one_row_per_point(self) -> None:
        buffer = pack([[1.0, 2.0, 3.0, 0.5], [4.0, 5.0, 6.0, 0.25]])
        decoded = decode_lidar_buffer(buffer)
        assert decoded.shape == (2, CARLA_LIDAR_STRIDE)
        assert decoded[1, 0] == pytest.approx(4.0)
        assert decoded[0, 3] == pytest.approx(0.5)

    def test_decoding_leaves_the_points_in_carla_coordinates(self) -> None:
        """Decoding and converting are separate steps on purpose."""
        decoded = decode_lidar_buffer(pack([[0.0, 9.0, 0.0, 1.0]]))
        assert decoded[0, 1] == pytest.approx(9.0)

    def test_an_empty_buffer_decodes_to_no_points(self) -> None:
        assert decode_lidar_buffer(b"").shape == (0, CARLA_LIDAR_STRIDE)

    def test_a_truncated_buffer_is_rejected_rather_than_reshaped(self) -> None:
        """Malformed sensor data must fail loudly, not silently lose a point."""
        with pytest.raises(InvalidPointCloudError, match="malformed"):
            decode_lidar_buffer(struct.pack("<5f", 1, 2, 3, 4, 5))


class TestSimulationTime:
    def test_elapsed_seconds_are_anchored_to_the_fixed_epoch(self) -> None:
        assert simulation_timestamp(0.0) == SIMULATION_EPOCH
        assert simulation_timestamp(1.5) == SIMULATION_EPOCH + timedelta(seconds=1.5)

    def test_timestamps_are_timezone_aware_utc(self) -> None:
        assert simulation_timestamp(2.0).tzinfo is UTC

    def test_successive_frames_strictly_increase(self) -> None:
        times = [simulation_timestamp(index * 0.05) for index in range(5)]
        assert all(b > a for a, b in pairwise(times))

    def test_the_interval_is_exactly_the_timestep(self) -> None:
        """Tracking measures velocity from this interval, so it must be exact."""
        first, second = simulation_timestamp(0.05), simulation_timestamp(0.10)
        assert (second - first).total_seconds() == pytest.approx(0.05, abs=1e-9)

    def test_the_same_simulation_time_always_yields_the_same_timestamp(self) -> None:
        """No wall-clock component anywhere: a replay reproduces exactly."""
        assert simulation_timestamp(3.25) == simulation_timestamp(3.25)

    def test_a_negative_or_non_finite_time_is_rejected(self) -> None:
        with pytest.raises(InvalidPointCloudError):
            simulation_timestamp(-1.0)
        with pytest.raises(InvalidPointCloudError):
            simulation_timestamp(float("inf"))


class TestWorldToEgo:
    """Placing world points in the ego frame, in ADAPT-X coordinates."""

    def test_an_ego_at_the_origin_facing_forward_only_flips_y(self) -> None:
        position = carla_world_to_ego((10.0, 4.0, 1.0), (0.0, 0.0, 0.0), 0.0)
        assert (position.x, position.y, position.z) == pytest.approx((10.0, -4.0, 1.0))

    def test_translation_is_relative_to_the_ego(self) -> None:
        position = carla_world_to_ego((30.0, 0.0, 0.0), (10.0, 0.0, 0.0), 0.0)
        assert position.x == pytest.approx(20.0)

    def test_a_rotated_ego_puts_the_point_in_its_own_frame(self) -> None:
        """Ego facing CARLA +y (right, yaw 90). A world point further along
        that axis is directly *ahead* of the vehicle."""
        position = carla_world_to_ego((0.0, 25.0, 0.0), (0.0, 5.0, 0.0), 90.0)
        assert position.x == pytest.approx(20.0)
        assert position.y == pytest.approx(0.0, abs=1e-9)

    def test_an_object_to_the_left_of_a_rotated_ego_reads_as_left(self) -> None:
        """Ego at CARLA yaw 90; work out which way "left" actually points.

        CARLA yaw 90 is ADAPT-X heading -pi/2, so the ego faces ADAPT-X -y.
        Rotating that facing a quarter turn counter-clockwise gives +x, so the
        vehicle's left is ADAPT-X +x - and x is the axis the conversion leaves
        alone, so it is CARLA +x too.

        Getting this backwards is the whole reason the test exists: a mirrored
        conversion still returns a plausible number for the point, and only the
        sign says the object was put on the wrong side of the car.
        """
        left = carla_world_to_ego((6.0, 0.0, 0.0), (0.0, 0.0, 0.0), 90.0)
        assert left.x == pytest.approx(0.0, abs=1e-9)
        assert left.y == pytest.approx(6.0), "CARLA +x must be to the left of a yaw-90 ego"

        right = carla_world_to_ego((-6.0, 0.0, 0.0), (0.0, 0.0, 0.0), 90.0)
        assert right.y == pytest.approx(-6.0)

    def test_height_is_a_plain_difference(self) -> None:
        position = carla_world_to_ego((0.0, 0.0, 3.0), (0.0, 0.0, 1.0), 45.0)
        assert position.z == pytest.approx(2.0)

    def test_a_velocity_is_rotated_but_not_translated(self) -> None:
        velocity = carla_vector_to_ego((0.0, 10.0, 0.0), 90.0)
        assert velocity.x == pytest.approx(10.0)
        assert velocity.y == pytest.approx(0.0, abs=1e-9)


class TestEgoToWorldRoundTrip:
    """The inverse a scenario needs, checked against the forward conversion."""

    @pytest.mark.parametrize("yaw", [0.0, 37.0, 90.0, -125.0, 180.0])
    def test_a_round_trip_returns_the_original_offset(self, yaw: float) -> None:
        ego = (12.0, -8.0, 0.5)
        offset = (30.0, 3.5, 0.0)
        world = ego_offset_to_carla_world(offset, ego, yaw)
        back = carla_world_to_ego(world, ego, yaw)
        assert (back.x, back.y, back.z) == pytest.approx(offset, abs=1e-9)

    def test_ahead_of_a_forward_facing_ego_is_ahead_in_the_world(self) -> None:
        world = ego_offset_to_carla_world((20.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)
        assert world == pytest.approx((20.0, 0.0, 0.0))

    def test_left_of_a_forward_facing_ego_is_carla_negative_y(self) -> None:
        """ADAPT-X left is CARLA's -y, because CARLA's +y is right."""
        world = ego_offset_to_carla_world((0.0, 4.0, 0.0), (0.0, 0.0, 0.0), 0.0)
        assert world == pytest.approx((0.0, -4.0, 0.0))

    def test_an_offset_travelling_outward_uses_the_same_flip(self) -> None:
        assert adaptx_offset_to_carla(1.0, 2.0, 3.0) == (1.0, -2.0, 3.0)


class TestRawFrameAssembly:
    def points(self) -> np.ndarray:
        return np.array([[10.0, 4.0, 0.5, 0.9], [20.0, -6.0, 1.0, 0.4]])

    def test_the_frame_is_labelled_simulation_never_live_sensor(self) -> None:
        """A simulated scan must never be presentable as hardware output."""
        frame = build_raw_frame(
            points_carla=self.points(), frame_id=7, sensor_id="lidar", elapsed_seconds=1.0
        )
        assert frame.source is DataSource.SIMULATION
        assert frame.source is not DataSource.LIVE_SENSOR

    def test_points_arrive_already_in_the_adaptx_frame(self) -> None:
        frame = build_raw_frame(
            points_carla=self.points(), frame_id=7, sensor_id="lidar", elapsed_seconds=1.0
        )
        assert frame.points[0, 1] == pytest.approx(-4.0)
        assert frame.points[1, 1] == pytest.approx(6.0)

    def test_the_simulator_frame_number_is_preserved(self) -> None:
        frame = build_raw_frame(
            points_carla=self.points(), frame_id=4321, sensor_id="lidar", elapsed_seconds=1.0
        )
        assert frame.frame_id == 4321

    def test_the_timestamp_is_simulation_time(self) -> None:
        frame = build_raw_frame(
            points_carla=self.points(), frame_id=1, sensor_id="lidar", elapsed_seconds=2.5
        )
        assert frame.timestamp == SIMULATION_EPOCH + timedelta(seconds=2.5)

    def test_the_timestamp_ignores_the_wall_clock(self) -> None:
        frame = build_raw_frame(
            points_carla=self.points(), frame_id=1, sensor_id="lidar", elapsed_seconds=0.0
        )
        assert abs((frame.timestamp - datetime.now(UTC)).total_seconds()) > 1_000_000

    def test_intensity_is_kept_by_default(self) -> None:
        frame = build_raw_frame(
            points_carla=self.points(), frame_id=1, sensor_id="lidar", elapsed_seconds=0.0
        )
        assert frame.points.shape[1] == 4
        assert frame.fields == XYZI_FIELDS
        assert frame.has_intensity

    def test_intensity_can_be_dropped(self) -> None:
        frame = build_raw_frame(
            points_carla=self.points(),
            frame_id=1,
            sensor_id="lidar",
            elapsed_seconds=0.0,
            include_intensity=False,
        )
        assert frame.points.shape[1] == 3
        assert frame.fields == XYZ_FIELDS

    def test_the_frame_is_in_the_lidar_coordinate_frame(self) -> None:
        frame = build_raw_frame(
            points_carla=self.points(), frame_id=1, sensor_id="roof", elapsed_seconds=0.0
        )
        assert frame.coordinate_frame is CoordinateFrame.LIDAR
        assert frame.sensor_id == "roof"

    def test_an_empty_scan_produces_a_valid_empty_frame(self) -> None:
        """A scan that returned nothing is an observation, not an error."""
        frame = build_raw_frame(
            points_carla=np.empty((0, 4)), frame_id=2, sensor_id="lidar", elapsed_seconds=0.1
        )
        assert frame.point_count == 0
        assert frame.bounds() is None
