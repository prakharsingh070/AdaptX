"""Trajectory-prediction result contracts (Phase 5).

The predictor reports what it predicted **and what it declined to predict, with
a reason**. A frame where nothing was predicted is therefore distinguishable
from one where tracks existed but none had a usable measured velocity - the
same accounting rule Phase 3 applied to rejected clusters (ADR-022) and Phase 4
to unmatched detections.

Durations are measured with :func:`time.perf_counter`. Nothing is estimated.
"""

from __future__ import annotations

from pydantic import Field, model_validator

from adaptx.models.common import AdaptXModel, TimestampedModel
from adaptx.models.prediction import PredictedTrajectory, PredictionStatus


class PredictionConfiguration(AdaptXModel):
    """Effective prediction configuration that produced a result.

    Carried on every result so a record is self-describing. A plain model
    rather than a reference to
    :class:`~adaptx.config.settings.PredictionSettings`, keeping
    :mod:`adaptx.models` free of any dependency on the configuration layer -
    the same split used for processing, detection and tracking.
    """

    horizon_s: float
    interval_s: float
    max_tracks: int
    max_speed_mps: float
    base_uncertainty_m: float
    uncertainty_growth_mps: float
    confidence_hits_full: int

    @property
    def points_per_trajectory(self) -> int:
        """Points a full-horizon trajectory contains, t+0 to the horizon inclusive."""
        return int(self.horizon_s / self.interval_s) + 1


class SkippedTrack(AdaptXModel):
    """A track that was considered and deliberately not predicted.

    Exists so "no trajectory" is never silent. Every track handed to the
    predictor appears either in ``trajectories`` or here.
    """

    track_id: int = Field(ge=0)
    status: PredictionStatus = Field(description="Why no trajectory was produced.")
    reason: str = Field(min_length=1, description="Human-readable explanation of the status.")


class PredictionResult(TimestampedModel):
    """Everything one prediction pass produced.

    ``timestamp`` is the source time predictions were made from - the tracking
    frame's time - and every trajectory point is offset from it.
    """

    frame_id: int = Field(ge=0)
    sensor_id: str = Field(min_length=1)
    predictor: str = Field(min_length=1, description="Identifier of the predictor that ran.")
    model_name: str = Field(
        min_length=1,
        description="Motion model applied, e.g. 'constant_velocity'. Not a learned model.",
    )
    is_baseline: bool = Field(
        default=True,
        description=(
            "True while prediction is a deterministic geometric baseline rather "
            "than a learned motion model."
        ),
    )
    uncertainty_model: str = Field(
        default="heuristic_linear_growth",
        description=(
            "Identifier of the uncertainty model. Heuristic and uncalibrated: "
            "no labelled trajectories exist to calibrate it against."
        ),
    )

    trajectories: list[PredictedTrajectory] = Field(
        default_factory=list, description="One entry per track that could be predicted."
    )
    skipped: list[SkippedTrack] = Field(
        default_factory=list, description="Tracks considered but not predicted, with reasons."
    )

    considered_track_count: int = Field(
        ge=0, description="Tracks handed to the predictor, before any filtering."
    )

    duration_ms: float = Field(ge=0.0, description="Whole prediction pass, measured.")

    configuration: PredictionConfiguration

    @model_validator(mode="after")
    def _check_accounting(self) -> PredictionResult:
        accounted = len(self.trajectories) + len(self.skipped)
        if self.considered_track_count != accounted:
            raise ValueError(
                f"considered_track_count ({self.considered_track_count}) must equal "
                f"predicted ({len(self.trajectories)}) + skipped ({len(self.skipped)})"
            )
        return self

    @property
    def predicted_track_count(self) -> int:
        """Tracks that received a trajectory."""
        return len(self.trajectories)

    @property
    def skipped_track_count(self) -> int:
        """Tracks considered but deliberately not predicted."""
        return len(self.skipped)

    @property
    def predicted_point_count(self) -> int:
        """Total trajectory points across every trajectory."""
        return sum(len(trajectory.points) for trajectory in self.trajectories)

    @property
    def predicted_track_ids(self) -> list[int]:
        """Ids of the tracks that received a trajectory, in result order."""
        return [trajectory.track_id for trajectory in self.trajectories]

    def trajectory_for(self, track_id: int) -> PredictedTrajectory | None:
        """The trajectory for ``track_id``, or ``None`` if it was not predicted.

        Provided for the risk engine, which reasons per track and would
        otherwise scan the list itself.
        """
        for trajectory in self.trajectories:
            if trajectory.track_id == track_id:
                return trajectory
        return None

    def counts_by_status(self) -> dict[str, int]:
        """How many tracks landed on each status, predicted and skipped alike."""
        counts: dict[str, int] = {}
        for trajectory in self.trajectories:
            key = trajectory.status.value
            counts[key] = counts.get(key, 0) + 1
        for entry in self.skipped:
            key = entry.status.value
            counts[key] = counts.get(key, 0) + 1
        return counts
