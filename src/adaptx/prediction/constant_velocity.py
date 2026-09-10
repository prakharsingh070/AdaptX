"""Deterministic constant-velocity trajectory prediction (Phase 5).

Consumes the :class:`~adaptx.models.tracking.TrackedObject` output of Phase 4
and extrapolates each track's measured velocity forward:

    tracks -> eligibility -> extrapolation -> heuristic uncertainty
           -> PredictedTrajectory[]

**This is a deterministic constant-velocity baseline.** It is not a learned
model, not a Kalman filter, and it makes no claim to be physically accurate:
real vehicles accelerate, brake, turn and follow lanes, and none of that is
modelled here. It is the explainable reference an ML predictor would later have
to beat, behind an unchanged interface (ADR-026).

The model
---------
For a track with a measured velocity, the position at ``t`` seconds after the
prediction time is::

    position(t) = position + velocity * (age_s + t)

``age_s`` is the **measured** interval between the track's last observation and
the prediction time. For a track matched in the current frame it is zero and
the formula collapses to the familiar ``p + v*t``. For a coasting track it is
positive, because the track's position is already stale by exactly that much;
ignoring it would silently pretend a missed frame never happened. The
trajectory is then marked ``EXTRAPOLATED`` rather than ``PREDICTED``.

Velocity semantics (ADR-023)
----------------------------
``velocity is None`` means *not measurable*, not *zero*. Such a track gets no
trajectory at all - it is recorded in ``skipped`` with
``INSUFFICIENT_VELOCITY``. Emitting a flat "stays where it is" path would
invent a measurement.

A measured ``Vector3(0, 0, 0)`` is different: it is a real observed standstill,
and it legitimately produces a stationary trajectory.

The smoothed ``velocity`` is used rather than ``observed_velocity``: it is the
tracker's best estimate of sustained motion, and single-frame noise is exactly
what should not be projected three seconds into the future.

Heading (``heading_rad``) is *not* used to steer the extrapolation. The
velocity vector is the authoritative motion vector; heading is derived from it
and is null below the tracker's speed floor. Forcing motion along a heading
that disagreed with the velocity would discard measured information.

Uncertainty
-----------
A **heuristic** radius that grows linearly with extrapolation time::

    uncertainty(t) = base_uncertainty_m + uncertainty_growth_mps * (age_s + t)

It is not a calibrated sigma, not a probability and not a confidence interval.
No labelled trajectories exist to calibrate one against, so none is claimed.

Not implemented here: collision or conflict reasoning, time-to-collision,
trajectory overlap and risk scoring. Those belong to the risk engine, which
consumes this output.
"""

from __future__ import annotations

import math
import time
from datetime import datetime, timedelta

from adaptx.config.settings import PredictionSettings
from adaptx.core.logging import get_logger
from adaptx.models.common import Vector3
from adaptx.models.prediction import (
    PredictedTrajectory,
    PredictionStatus,
    TrajectoryPoint,
)
from adaptx.models.prediction_result import (
    PredictionConfiguration,
    PredictionResult,
    SkippedTrack,
)
from adaptx.models.tracking import TrackedObject, TrackStatus
from adaptx.prediction.interfaces import TrajectoryPredictor

logger = get_logger(__name__)

#: Motion model applied. Recorded on every result for traceability.
MODEL_NAME = "constant_velocity"

#: Identifier of the uncertainty model. Heuristic, uncalibrated.
UNCERTAINTY_MODEL = "heuristic_linear_growth"


class ConstantVelocityPredictor(TrajectoryPredictor):
    """Extrapolates each track's measured velocity over a fixed horizon."""

    name = "constant_velocity_v1"
    #: True: this is a deterministic geometric baseline, not a learned model.
    is_baseline = True

    def __init__(self, settings: PredictionSettings) -> None:
        self._settings = settings

    # -- state -------------------------------------------------------------
    @property
    def configuration(self) -> PredictionConfiguration:
        """Snapshot of the settings that shape this predictor's behaviour."""
        settings = self._settings
        return PredictionConfiguration(
            horizon_s=settings.horizon_s,
            interval_s=settings.interval_s,
            max_tracks=settings.max_tracks,
            max_speed_mps=settings.max_speed_mps,
            base_uncertainty_m=settings.base_uncertainty_m,
            uncertainty_growth_mps=settings.uncertainty_growth_mps,
            confidence_hits_full=settings.confidence_hits_full,
        )

    # -- prediction --------------------------------------------------------
    def predict(
        self,
        tracks: list[TrackedObject],
        timestamp: datetime,
        *,
        horizon_s: float | None = None,
        timestep_s: float | None = None,
        frame_id: int = 0,
        sensor_id: str = "unknown",
    ) -> PredictionResult:
        """Predict a trajectory for every eligible track.

        Stateless: the same tracks and timestamp always produce the same
        result, and nothing is carried between calls.

        Args:
            tracks: Live tracks from a tracking update, in any order.
            timestamp: Source time predictions are made from.
            horizon_s: Overrides the configured horizon for this call.
            timestep_s: Overrides the configured interval for this call.
            frame_id: Frame identifier recorded on the result.
            sensor_id: Sensor identifier recorded on the result.

        Raises:
            ValueError: If an override is non-positive, non-finite, or the
                interval exceeds the horizon.
        """
        started = time.perf_counter()

        horizon = self._settings.horizon_s if horizon_s is None else horizon_s
        interval = self._settings.interval_s if timestep_s is None else timestep_s
        _validate_span(horizon, interval)

        offsets = _offsets(horizon, interval)
        # Absolute point times are identical for every track, so they are built
        # once per call rather than once per point per track.
        point_times = tuple(timestamp + timedelta(seconds=offset) for offset in offsets)
        trajectories: list[PredictedTrajectory] = []
        skipped: list[SkippedTrack] = []

        for index, track in enumerate(tracks):
            if index >= self._settings.max_tracks:
                skipped.append(
                    SkippedTrack(
                        track_id=track.track_id,
                        status=PredictionStatus.LIMIT_EXCEEDED,
                        reason=(
                            f"per-call limit of {self._settings.max_tracks} tracks "
                            f"reached before this track"
                        ),
                    )
                )
                continue

            outcome = self._predict_track(track, timestamp, horizon, interval, offsets, point_times)
            if isinstance(outcome, SkippedTrack):
                skipped.append(outcome)
            else:
                trajectories.append(outcome)

        duration_ms = (time.perf_counter() - started) * 1000.0

        logger.debug(
            "prediction complete",
            extra={
                "context": {
                    "frame_id": frame_id,
                    "considered": len(tracks),
                    "predicted": len(trajectories),
                    "skipped": len(skipped),
                    "horizon_s": horizon,
                    "duration_ms": round(duration_ms, 3),
                }
            },
        )

        return PredictionResult(
            timestamp=timestamp,
            frame_id=frame_id,
            sensor_id=sensor_id,
            predictor=self.name,
            model_name=MODEL_NAME,
            is_baseline=self.is_baseline,
            uncertainty_model=UNCERTAINTY_MODEL,
            trajectories=trajectories,
            skipped=skipped,
            considered_track_count=len(tracks),
            duration_ms=duration_ms,
            configuration=self.configuration,
        )

    # -- internals ---------------------------------------------------------
    def _predict_track(
        self,
        track: TrackedObject,
        timestamp: datetime,
        horizon: float,
        interval: float,
        offsets: tuple[float, ...],
        point_times: tuple[datetime, ...],
    ) -> PredictedTrajectory | SkippedTrack:
        """Predict one track, or explain why it was not predicted."""
        if track.status is TrackStatus.LOST:
            return SkippedTrack(
                track_id=track.track_id,
                status=PredictionStatus.TRACK_LOST,
                reason="track is terminated; no active prediction is published for it",
            )

        velocity = track.velocity
        if velocity is None:
            return SkippedTrack(
                track_id=track.track_id,
                status=PredictionStatus.INSUFFICIENT_VELOCITY,
                reason=(
                    "no measured velocity; a single observation cannot show motion "
                    "and assuming zero would claim a standstill that was never observed"
                ),
            )

        speed = velocity.magnitude
        if speed > self._settings.max_speed_mps:
            return SkippedTrack(
                track_id=track.track_id,
                status=PredictionStatus.INVALID_VELOCITY,
                reason=(
                    f"measured speed {speed:.2f} m/s exceeds the sanity bound of "
                    f"{self._settings.max_speed_mps} m/s; the value is rejected rather "
                    f"than clipped"
                ),
            )

        age_s = _observation_age_s(track, timestamp)
        if age_s > horizon:
            return SkippedTrack(
                track_id=track.track_id,
                status=PredictionStatus.STALE_OBSERVATION,
                reason=(
                    f"last observation is {age_s:.2f} s old, beyond the {horizon} s "
                    f"horizon; the output would be more gap-filling than prediction"
                ),
            )

        is_extrapolated = age_s > 0.0 or track.status is TrackStatus.COASTING
        confidence = self._track_confidence(track)
        base = self._settings.base_uncertainty_m
        growth = self._settings.uncertainty_growth_mps
        uncertainty_at_zero = base + growth * age_s

        points: list[TrajectoryPoint] = []
        for offset, point_time in zip(offsets, point_times, strict=True):
            elapsed = age_s + offset
            uncertainty = base + growth * elapsed
            points.append(
                TrajectoryPoint(
                    timestamp=point_time,
                    time_offset_s=offset,
                    position=Vector3(
                        x=track.position.x + velocity.x * elapsed,
                        y=track.position.y + velocity.y * elapsed,
                        z=track.position.z + velocity.z * elapsed,
                    ),
                    velocity=velocity,
                    confidence=_clamp(confidence * (uncertainty_at_zero / uncertainty)),
                    position_uncertainty_m=uncertainty,
                )
            )

        return PredictedTrajectory(
            timestamp=timestamp,
            track_id=track.track_id,
            horizon_s=horizon,
            timestep_s=interval,
            points=points,
            confidence=confidence,
            predictor_name=self.name,
            status=(
                PredictionStatus.EXTRAPOLATED if is_extrapolated else PredictionStatus.PREDICTED
            ),
            observation_age_s=age_s,
            coordinate_frame=track.coordinate_frame,
            source=track.source,
        )

    def _track_confidence(self, track: TrackedObject) -> float:
        """Evidence behind a track, on ``[0, 1]``.

        Deterministic and explicitly **not** a probability that the prediction
        is correct - no labelled trajectories exist to measure that against.
        It combines the only two evidence signals the tracker provides:

        * how often the track has been seen, saturating at
          ``confidence_hits_full``, so a tentative track scores below a
          well-established one;
        * how many consecutive frames it has been missed, so a coasting track
          scores below one observed in the current frame.
        """
        hits_factor = min(track.hits, self._settings.confidence_hits_full) / (
            self._settings.confidence_hits_full
        )
        coasting_factor = 1.0 / (1.0 + track.missed_frames)
        return _clamp(hits_factor * coasting_factor)


def _observation_age_s(track: TrackedObject, timestamp: datetime) -> float:
    """Measured seconds between a track's last observation and ``timestamp``.

    Zero when the track was seen in the current frame, and zero when the track
    has never been seen or reports a time in the future - a negative age would
    mean predicting backwards from an observation that has not happened.
    """
    if track.last_seen is None:
        return 0.0
    age = (timestamp - track.last_seen).total_seconds()
    if not math.isfinite(age) or age <= 0.0:
        return 0.0
    return age


def _validate_span(horizon: float, interval: float) -> None:
    """Reject a horizon or interval that cannot describe a trajectory."""
    if not math.isfinite(horizon) or horizon <= 0.0:
        raise ValueError(f"horizon_s must be finite and positive, got {horizon}")
    if not math.isfinite(interval) or interval <= 0.0:
        raise ValueError(f"timestep_s must be finite and positive, got {interval}")
    if interval > horizon:
        raise ValueError(f"timestep_s ({interval}) must be <= horizon_s ({horizon})")


def _offsets(horizon: float, interval: float) -> tuple[float, ...]:
    """Time offsets from ``t+0`` to the horizon inclusive.

    Each offset is ``index * interval`` rather than a running sum, so floating
    error cannot accumulate along the trajectory. The final offset is clamped
    to the horizon exactly, which keeps the sequence inside the contract's
    horizon validator when the horizon is not an exact multiple of the
    interval in binary floating point.
    """
    steps = math.floor(horizon / interval + 1e-9)
    return tuple(min(index * interval, horizon) for index in range(steps + 1))


def _clamp(value: float) -> float:
    """Confine a score to ``[0, 1]``."""
    return max(0.0, min(1.0, value))


def build_predictor(settings: PredictionSettings) -> ConstantVelocityPredictor:
    """Construct the configured baseline predictor."""
    return ConstantVelocityPredictor(settings)
