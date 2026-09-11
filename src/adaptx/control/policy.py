"""Risk-governed speed policy: the baseline that drives the ego (live extension).

A pure function of what the perception stack produced this frame - the risk
assessments, the tracks and the predicted paths - plus the ego's own
odometry and lane geometry. It reads **no ground truth**: it cannot, because
nothing here has a parameter for it (ADR-045). The decision is a target
speed from a table keyed by the highest risk level present, tightened by the
distance of the closest object in the ego's path, and turned into a
throttle or brake command by a proportional law under acceleration limits.
Steering follows the map's lane centre, which is road geometry rather than
a perceived object, and is not planning.

What this is not: a validated, tuned or safety-rated controller. It is the
smallest loop that lets the dashboard show the perception stack changing
what the vehicle does. Every threshold is a configuration value with the
word "baseline" beside it.

Rules, in the order they are applied:

1. An in-path object inside ``emergency_distance_m`` - full brake, whatever
   its risk level says: proximity alone is enough to stop for.
2. A CRITICAL in-path level, or an in-path object inside
   ``min_safe_distance_m`` - target speed zero; the ego holds until the
   condition has been absent for ``resume_dwell_frames`` consecutive frames
   (hysteresis, so a flickering track does not make the vehicle lurch).
3. Otherwise the target is the table entry for the highest level among the
   objects in the ego's corridor - "in path" means ahead and within
   ``path_half_width_m`` of the ego axis now, or predicted to be. Objects
   beside the road do not set the speed (they still appear in
   ``scene_level``). UNKNOWN is **not** LOW: an unscored in-path object gets
   the ``unknown_speed_mps`` entry, which defaults to the MEDIUM speed.
4. Throttle or brake follows from the speed error, never both, with the
   commanded change per frame capped by the acceleration limits.
"""

from __future__ import annotations

from adaptx.config.settings import ControlSettings
from adaptx.control.corridor import governs, path_relation
from adaptx.control.models import (
    ControlCommand,
    ControlConfiguration,
    ControllerState,
    EgoObservation,
)
from adaptx.models.prediction_result import PredictionResult
from adaptx.models.risk import RiskLevel
from adaptx.models.risk_assessment import RiskAssessment, RiskAssessmentResult
from adaptx.models.tracking_result import TrackingResult

POLICY_NAME = "risk_governed_speed_baseline"

#: Ordering used to pick the governing level. UNKNOWN sits between LOW and
#: MEDIUM here only to select a *target speed*; it is not a point on the
#: risk scale (ADR-032) and the command records it as UNKNOWN.
_SEVERITY = {
    RiskLevel.LOW: 0,
    RiskLevel.UNKNOWN: 1,
    RiskLevel.MEDIUM: 2,
    RiskLevel.HIGH: 3,
    RiskLevel.CRITICAL: 4,
}


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class RiskGovernedSpeedPolicy:
    """Deterministic speed governor keyed on the risk engine's output."""

    def __init__(self, settings: ControlSettings) -> None:
        self._settings = settings
        self._hold_frames_clear = 0
        self._holding = False
        self._last_target = 0.0
        self._setpoint = 0.0
        self._state = ControllerState.CRUISING

    @property
    def state(self) -> ControllerState:
        """What the governor did on the last frame."""
        return self._state

    def reset(self) -> None:
        """Forget hysteresis state; used when a session restarts."""
        self._hold_frames_clear = 0
        self._holding = False
        self._last_target = 0.0
        self._setpoint = 0.0
        self._state = ControllerState.CRUISING

    def configuration(self) -> ControlConfiguration:
        """The parameters in force."""
        s = self._settings
        return ControlConfiguration(
            policy=POLICY_NAME,
            max_speed_mps=s.max_speed_mps,
            medium_speed_mps=s.medium_speed_mps,
            high_speed_mps=s.high_speed_mps,
            unknown_speed_mps=s.unknown_speed_mps,
            min_safe_distance_m=s.min_safe_distance_m,
            emergency_distance_m=s.emergency_distance_m,
            max_acceleration_mps2=s.max_acceleration_mps2,
            max_deceleration_mps2=s.max_deceleration_mps2,
            steering_limit=s.steering_limit,
            path_half_width_m=s.path_half_width_m,
            resume_dwell_frames=s.resume_dwell_frames,
        )

    # -- the decision --------------------------------------------------------
    def decide(
        self,
        *,
        risk: RiskAssessmentResult,
        tracking: TrackingResult,
        prediction: PredictionResult,
        ego: EgoObservation,
        dt_s: float,
    ) -> ControlCommand:
        """One frame's command from this frame's perception outputs.

        Args:
            risk: The risk engine's assessments (distance, level, score).
            tracking: The tracks, for where each object is relative to the ego.
            prediction: The predicted paths, used only to decide whether an
                object that is not in the path now is predicted to enter it.
            ego: The ego's own odometry and lane geometry.
            dt_s: The simulation timestep, for the acceleration limits.
        """
        s = self._settings
        positions = {t.track_id: t.position for t in tracking.tracks}
        predicted = {t.track_id: [p.position for p in t.points] for t in prediction.trajectories}

        # One corridor rule, shared with the scene snapshot (control.corridor),
        # so what the dashboard labels IN PATH is what governs the speed.
        in_path: list[RiskAssessment] = []
        for assessment in risk.assessments:
            position = positions.get(assessment.track_id)
            if position is None:
                continue
            relation = path_relation(
                position, predicted.get(assessment.track_id, ()), s.path_half_width_m
            )
            if governs(relation):
                in_path.append(assessment)
        nearest = min(in_path, key=lambda a: a.distance_m, default=None)

        # The speed table is keyed on the highest level among objects in (or
        # predicted to enter) the ego's corridor. Measured on the live server:
        # the Phase 7 engine scores a lamp post 5 m beside the road HIGH or
        # CRITICAL on proximity alone, and a governor keyed on the scene
        # maximum pinned the ego at the kerb forever. Objects beside the path
        # still appear in ``scene_level`` so the dashboard shows what the
        # risk engine said; they do not set the speed.
        scene_level = max(
            (a.risk_level for a in risk.assessments),
            key=lambda level: _SEVERITY[level],
            default=RiskLevel.LOW,
        )
        governing = max(
            (a.risk_level for a in in_path),
            key=lambda level: _SEVERITY[level],
            default=RiskLevel.LOW,
        )

        # 1. Emergency: proximity alone, regardless of level.
        if nearest is not None and nearest.distance_m <= s.emergency_distance_m:
            self._holding = True
            self._hold_frames_clear = 0
            self._state = ControllerState.EMERGENCY_BRAKING
            self._last_target = 0.0
            self._setpoint = 0.0
            return self._command(
                throttle=0.0,
                brake=1.0,
                target=0.0,
                ego=ego,
                governing=governing,
                scene_level=scene_level,
                nearest=nearest,
                reason=(
                    f"in-path object track {nearest.track_id} at {nearest.distance_m:.1f} m "
                    f"is inside the emergency distance {s.emergency_distance_m:.1f} m"
                ),
            )

        # 2. Hold: CRITICAL, or an in-path object inside the safe distance.
        must_hold = governing is RiskLevel.CRITICAL or (
            nearest is not None and nearest.distance_m <= s.min_safe_distance_m
        )
        if must_hold:
            self._holding = True
            self._hold_frames_clear = 0
        elif self._holding:
            self._hold_frames_clear += 1
            if self._hold_frames_clear >= s.resume_dwell_frames:
                self._holding = False

        if self._holding:
            reason = (
                "highest in-path risk level is CRITICAL"
                if governing is RiskLevel.CRITICAL
                else (
                    f"in-path object track {nearest.track_id} at {nearest.distance_m:.1f} m "
                    f"is inside the safe distance {s.min_safe_distance_m:.1f} m"
                    if nearest is not None and nearest.distance_m <= s.min_safe_distance_m
                    else f"holding {s.resume_dwell_frames - self._hold_frames_clear} more "
                    "frame(s) before resuming"
                )
            )
            self._state = (
                ControllerState.STOPPED if ego.speed_mps < 0.2 else ControllerState.HOLDING
            )
            target = 0.0
        else:
            target = {
                RiskLevel.LOW: s.max_speed_mps,
                RiskLevel.UNKNOWN: s.unknown_speed_mps,
                RiskLevel.MEDIUM: s.medium_speed_mps,
                RiskLevel.HIGH: s.high_speed_mps,
                RiskLevel.CRITICAL: 0.0,
            }[governing]
            if self._last_target < 0.5 and target > 0.0 and ego.speed_mps < 0.5:
                self._state = ControllerState.RESUMING
            elif target < ego.speed_mps - s.braking_threshold_mps:
                self._state = ControllerState.SLOWING
            else:
                self._state = ControllerState.CRUISING
            if in_path:
                reason = f"highest in-path risk level {governing.value}: target {target:.1f} m/s"
            elif risk.assessments:
                reason = (
                    f"no object in the path (scene level {scene_level.value}): "
                    f"cruising at {target:.1f} m/s"
                )
            else:
                reason = f"no assessed object: cruising at {target:.1f} m/s"

        # The commanded setpoint rises towards the target no faster than the
        # acceleration limit, so a track that flickers between levels
        # (measured live: identity switches on a parked car swung the level
        # HIGH->LOW->HIGH within a few frames) does not lurch the ego. A
        # falling target is applied at once: slowing is never delayed. The
        # command reports both: the target the rules chose and the setpoint
        # the pedals follow.
        self._last_target = target
        if target > self._setpoint:
            self._setpoint = min(
                target, max(self._setpoint, ego.speed_mps) + s.max_acceleration_mps2 * dt_s
            )
        else:
            self._setpoint = target
        # 4. Proportional throttle/brake under the acceleration limits.
        error = self._setpoint - ego.speed_mps
        max_up = s.max_acceleration_mps2 * dt_s
        max_down = s.max_deceleration_mps2 * dt_s
        if error > 0.0:
            throttle = _clamp(
                s.hold_throttle + s.throttle_gain * min(error, max_up * 10.0), 0.0, 1.0
            )
            brake = 0.0
        elif -error > s.braking_threshold_mps or target == 0.0:
            throttle = 0.0
            brake = _clamp(s.brake_gain * min(-error, max_down * 10.0), 0.0, 1.0)
            if target == 0.0 and ego.speed_mps > 0.0:
                brake = max(brake, 0.3)
        else:
            throttle, brake = 0.0, 0.0
        return self._command(
            throttle=throttle,
            brake=brake,
            target=target,
            ego=ego,
            governing=governing,
            scene_level=scene_level,
            nearest=nearest,
            reason=reason,
            setpoint=self._setpoint,
        )

    def _command(
        self,
        *,
        throttle: float,
        brake: float,
        target: float,
        ego: EgoObservation,
        governing: RiskLevel,
        scene_level: RiskLevel,
        nearest: RiskAssessment | None,
        reason: str,
        setpoint: float | None = None,
    ) -> ControlCommand:
        steer = 0.0
        if ego.lane_heading_error_rad is not None:
            steer = _clamp(
                self._settings.steering_gain * ego.lane_heading_error_rad,
                -self._settings.steering_limit,
                self._settings.steering_limit,
            )
        return ControlCommand(
            throttle=throttle,
            brake=brake,
            steer=steer,
            target_speed_mps=target,
            setpoint_mps=target if setpoint is None else setpoint,
            state=self._state,
            governing_level=governing,
            scene_level=scene_level,
            governing_track_id=None if nearest is None else nearest.track_id,
            nearest_in_path_m=None if nearest is None else nearest.distance_m,
            reason=reason,
        )


__all__ = ["POLICY_NAME", "RiskGovernedSpeedPolicy"]
