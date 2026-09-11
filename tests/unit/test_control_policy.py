"""The baseline speed governor (post-Phase-12 extension).

Deterministic decisions from the risk engine's outputs and the ego's own
odometry. Every threshold here is a configured baseline; the tests pin the
rules, not the wisdom of the numbers.
"""

from __future__ import annotations

import pytest

from adaptx.config.settings import ControlSettings
from adaptx.control import ControllerState, EgoObservation, RiskGovernedSpeedPolicy
from adaptx.models.risk import RiskLevel
from adaptx.scenarios.result import PipelineFrameOutputs
from tests.fixtures.evaluation import FrameSpec, RiskSpec, RunBuilder, TrackSpec

Objects = list[tuple[int, float, float, RiskLevel, float | None]]


def settings(**overrides: object) -> ControlSettings:
    return ControlSettings(
        max_speed_mps=8.0,
        medium_speed_mps=5.0,
        high_speed_mps=2.0,
        min_safe_distance_m=8.0,
        emergency_distance_m=5.0,
        resume_dwell_frames=3,
        **overrides,  # type: ignore[arg-type]
    )


def ego(speed: float = 6.0, lane_error: float | None = 0.0) -> EgoObservation:
    return EgoObservation(speed_mps=speed, heading_rad=0.0, lane_heading_error_rad=lane_error)


def outputs(
    objects: Objects, trajectories: dict[int, list[tuple[float, float, float]]] | None = None
) -> PipelineFrameOutputs:
    """Real Phase 4/5/7 result contracts holding tracks at (x, y) with a level and score."""
    record = (
        RunBuilder({"a": 1})
        .frame(
            FrameSpec(
                tracks=[TrackSpec(track_id, x, y) for track_id, x, y, _, _ in objects],
                risk=[RiskSpec(track_id, level, score) for track_id, _, _, level, score in objects],
                trajectories=trajectories or {},
            )
        )
        .build()
    )
    produced = record.frames[0].outputs
    assert produced is not None
    return produced


def decide(
    policy: RiskGovernedSpeedPolicy,
    objects: Objects,
    speed: float = 6.0,
    trajectories: dict[int, list[tuple[float, float, float]]] | None = None,
    lane_error: float | None = 0.0,
):  # type: ignore[no-untyped-def]
    produced = outputs(objects, trajectories)
    return policy.decide(
        risk=produced.risk,
        tracking=produced.tracking,
        prediction=produced.prediction,
        ego=ego(speed, lane_error),
        dt_s=0.05,
    )


class TestTargets:
    def test_no_object_means_cruise_and_throttle(self) -> None:
        policy = RiskGovernedSpeedPolicy(settings())
        command = decide(policy, [], speed=2.0)
        assert command.target_speed_mps == 8.0
        assert command.throttle > 0.0 and command.brake == 0.0
        assert command.state is ControllerState.CRUISING
        assert command.governing_level is RiskLevel.LOW
        assert command.is_baseline

    @pytest.mark.parametrize(
        ("level", "score", "target"),
        [
            (RiskLevel.LOW, 0.1, 8.0),
            (RiskLevel.MEDIUM, 0.5, 5.0),
            (RiskLevel.HIGH, 0.7, 2.0),
            (RiskLevel.UNKNOWN, None, 5.0),
        ],
    )
    def test_the_level_table_sets_the_target(
        self, level: RiskLevel, score: float | None, target: float
    ) -> None:
        policy = RiskGovernedSpeedPolicy(settings())
        # Well ahead in the lane: in the path, but outside every distance rule.
        command = decide(policy, [(1, 30.0, 0.5, level, score)])
        assert command.target_speed_mps == target
        assert command.governing_level is level
        assert command.scene_level is level

    def test_unknown_is_never_treated_as_low(self) -> None:
        policy = RiskGovernedSpeedPolicy(settings())
        command = decide(policy, [(1, 30.0, 0.5, RiskLevel.UNKNOWN, None)])
        assert command.target_speed_mps < settings().max_speed_mps
        assert command.governing_level is RiskLevel.UNKNOWN

    def test_slowing_uses_the_brake_not_both_pedals(self) -> None:
        policy = RiskGovernedSpeedPolicy(settings())
        command = decide(policy, [(1, 30.0, 0.5, RiskLevel.HIGH, 0.7)], speed=7.0)
        assert command.brake > 0.0 and command.throttle == 0.0
        assert command.state is ControllerState.SLOWING

    def test_an_object_beside_the_road_is_reported_but_does_not_govern(self) -> None:
        """Measured live: lamp posts 5 m beside the road score HIGH on proximity."""
        policy = RiskGovernedSpeedPolicy(settings())
        command = decide(policy, [(1, 1.4, -5.3, RiskLevel.CRITICAL, 0.9)], speed=2.0)
        assert command.scene_level is RiskLevel.CRITICAL
        assert command.governing_level is RiskLevel.LOW
        assert command.target_speed_mps == 8.0
        assert "scene level critical" in command.reason


class TestDistances:
    def test_an_in_path_object_inside_the_safe_distance_holds(self) -> None:
        policy = RiskGovernedSpeedPolicy(settings())
        command = decide(policy, [(4, 7.0, 0.5, RiskLevel.MEDIUM, 0.5)], speed=3.0)
        assert command.target_speed_mps == 0.0
        assert command.nearest_in_path_m == pytest.approx((7.0**2 + 0.5**2) ** 0.5)
        assert command.governing_track_id == 4
        assert command.state is ControllerState.HOLDING
        assert command.brake > 0.0

    def test_inside_the_emergency_distance_is_full_brake(self) -> None:
        policy = RiskGovernedSpeedPolicy(settings())
        command = decide(policy, [(4, 4.0, 0.0, RiskLevel.LOW, 0.1)], speed=3.0)
        assert command.brake == 1.0 and command.throttle == 0.0
        assert command.state is ControllerState.EMERGENCY_BRAKING
        assert "emergency" in command.reason

    def test_an_object_beside_the_path_does_not_stop_the_ego(self) -> None:
        policy = RiskGovernedSpeedPolicy(settings())
        command = decide(policy, [(4, 4.0, 6.0, RiskLevel.LOW, 0.1)], speed=3.0)
        assert command.nearest_in_path_m is None
        assert command.target_speed_mps == 8.0

    def test_a_predicted_path_into_the_lane_counts_as_in_path(self) -> None:
        policy = RiskGovernedSpeedPolicy(settings())
        beside = [(9, 5.0, 5.0, RiskLevel.MEDIUM, 0.5)]  # 7.1 m away, off the path
        assert decide(policy, beside, speed=3.0).governing_track_id is None
        crossing = {9: [(0.0, 5.0, 5.0), (1.0, 5.0, 0.5)]}
        command = decide(policy, beside, speed=3.0, trajectories=crossing)
        assert command.governing_track_id == 9
        assert command.target_speed_mps == 0.0


class TestHysteresis:
    def test_critical_stops_and_resumes_only_after_the_dwell(self) -> None:
        policy = RiskGovernedSpeedPolicy(settings())
        critical = [(2, 30.0, 0.5, RiskLevel.CRITICAL, 0.9)]
        clear = [(2, 30.0, 0.5, RiskLevel.LOW, 0.1)]
        first = decide(policy, critical, speed=4.0)
        assert first.target_speed_mps == 0.0 and first.state is ControllerState.HOLDING
        stopped = decide(policy, critical, speed=0.0)
        assert stopped.state is ControllerState.STOPPED
        # Two clear frames: still holding (dwell is 3).
        assert decide(policy, clear, speed=0.0).target_speed_mps == 0.0
        assert decide(policy, clear, speed=0.0).target_speed_mps == 0.0
        # Third clear frame: resume.
        resumed = decide(policy, clear, speed=0.0)
        assert resumed.target_speed_mps == 8.0
        assert resumed.state is ControllerState.RESUMING
        assert resumed.throttle > 0.0

    def test_a_flicker_back_to_critical_restarts_the_dwell(self) -> None:
        policy = RiskGovernedSpeedPolicy(settings())
        critical = [(2, 30.0, 0.5, RiskLevel.CRITICAL, 0.9)]
        clear = [(2, 30.0, 0.5, RiskLevel.LOW, 0.1)]
        decide(policy, critical, speed=0.0)
        decide(policy, clear, speed=0.0)
        decide(policy, clear, speed=0.0)
        decide(policy, critical, speed=0.0)
        assert decide(policy, clear, speed=0.0).target_speed_mps == 0.0
        assert decide(policy, clear, speed=0.0).target_speed_mps == 0.0
        assert decide(policy, clear, speed=0.0).target_speed_mps == 8.0

    def test_reset_forgets_the_hold(self) -> None:
        policy = RiskGovernedSpeedPolicy(settings())
        decide(policy, [(2, 30.0, 0.5, RiskLevel.CRITICAL, 0.9)])
        policy.reset()
        assert decide(policy, []).target_speed_mps == 8.0


class TestSteering:
    def test_steer_follows_the_lane_error_within_the_limit(self) -> None:
        policy = RiskGovernedSpeedPolicy(settings(steering_gain=1.0, steering_limit=0.4))
        assert decide(policy, [], speed=3.0, lane_error=0.2).steer == pytest.approx(0.2)
        assert decide(policy, [], speed=3.0, lane_error=2.0).steer == 0.4
        assert decide(policy, [], speed=3.0, lane_error=-2.0).steer == -0.4
        assert decide(policy, [], speed=3.0, lane_error=None).steer == 0.0


class TestConfiguration:
    def test_settings_reject_inverted_distances_and_speeds(self) -> None:
        with pytest.raises(ValueError, match="emergency"):
            ControlSettings(min_safe_distance_m=4.0, emergency_distance_m=5.0)
        with pytest.raises(ValueError, match="high <= medium"):
            ControlSettings(medium_speed_mps=1.0, high_speed_mps=2.0)

    def test_configuration_is_reported_as_a_baseline(self) -> None:
        configuration = RiskGovernedSpeedPolicy(settings()).configuration()
        assert configuration.is_baseline
        assert configuration.policy == "risk_governed_speed_baseline"
        assert configuration.emergency_distance_m == 5.0
