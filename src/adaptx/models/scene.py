"""Scene snapshot contract (Phase 12).

One frame of what the pipeline produced, bundled for display: the tracks,
the predicted paths, the risk assessments, the resolution decisions, the map
summaries and a **sample** of the processed point cloud. It is the payload of
the dashboard's live channel and it carries nothing the pipeline did not
already compute - the dashboard renders it and adds nothing (ADR-055).

What is deliberately absent
---------------------------
Ground truth. A snapshot is built from the sensor frame and the pipeline
outputs and from nothing else, so a live view can never show the simulator's
answer beside the pipeline's guess (ADR-045). The full map grid is absent
too: a 0.5 m map over the default bounds is 57,600 cells and the adaptive
map can be far larger; the summaries and the per-tile decisions travel, the
cells do not.
"""

from __future__ import annotations

from datetime import datetime
from enum import StrEnum

import numpy as np
from pydantic import Field, model_validator

from adaptx.control.corridor import PathRelation
from adaptx.control.models import ControlCommand
from adaptx.live.models import LiveFrameInfo
from adaptx.models.adaptive_map import AdaptiveSpatialMapSummary, MappingComparison
from adaptx.models.adaptive_resolution import ResolutionBudget, TileResolutionDecision
from adaptx.models.common import (
    AdaptXModel,
    CoordinateFrame,
    DataSource,
    ObjectClass,
    TimestampedModel,
)
from adaptx.models.prediction import PredictedTrajectory
from adaptx.models.risk import RiskLevel
from adaptx.models.risk_assessment import RiskAssessment
from adaptx.models.spatial_map import SpatialMapSummary
from adaptx.models.tracking import TrackedObject, TrackStatus
from adaptx.models.vehicle import VehicleState

#: Default ceiling on points in a scene sample. Six thousand points draw
#: comfortably at ten frames a second on a canvas; a full sweep is ~27,000.
DEFAULT_MAX_SAMPLE_POINTS = 6000


class PointStage(StrEnum):
    """Which cloud the sample was taken from."""

    RAW = "raw"
    PROCESSED = "processed"


class PointSample(AdaptXModel):
    """A deterministic, size-capped sample of a frame's points, for drawing only.

    ``is_downsampled`` is true whenever ``sample_count < total_count``; the
    stride is fixed so two samples of the same frame are identical. Points are
    rounded to centimetres. This is a picture of the cloud, not the cloud.
    """

    total_count: int = Field(ge=0, description="Points in the processed frame.")
    sample_count: int = Field(ge=0)
    is_downsampled: bool
    stride: int = Field(ge=1, description="Every ``stride``-th point was kept.")
    stage: PointStage = Field(
        description="raw: the sensor frame as received; processed: after Phase 2 filtering."
    )
    coordinate_frame: CoordinateFrame = CoordinateFrame.EGO
    xyz: list[list[float]] = Field(
        default_factory=list, description="``sample_count`` rows of ``[x, y, z]`` in metres."
    )

    @model_validator(mode="after")
    def _check_shape(self) -> PointSample:
        if len(self.xyz) != self.sample_count:
            raise ValueError(f"sample_count {self.sample_count} but {len(self.xyz)} rows")
        if self.sample_count > self.total_count:
            raise ValueError("a sample cannot hold more points than the frame")
        if any(len(row) != 3 for row in self.xyz):
            raise ValueError("every sampled point must be [x, y, z]")
        return self


def sample_points(
    points: np.ndarray,
    *,
    max_points: int = DEFAULT_MAX_SAMPLE_POINTS,
    stage: PointStage = PointStage.PROCESSED,
) -> PointSample:
    """Take every ``k``-th point so that at most ``max_points`` remain."""
    if max_points < 1:
        raise ValueError("max_points must be at least 1")
    total = int(points.shape[0])
    if total == 0:
        return PointSample(
            total_count=0, sample_count=0, is_downsampled=False, stride=1, stage=stage
        )
    stride = max(1, -(-total // max_points))  # ceiling division
    kept = np.asarray(points[::stride, :3], dtype=np.float64)
    rounded = np.round(kept, 2)
    # Built from a (N, 3) array this function shaped itself, so the row check
    # the validator would run over every point (4-5 ms per 6000 rows, measured
    # in the live loop) is skipped here; a sample arriving as JSON is still
    # validated in full.
    return PointSample.model_construct(
        total_count=total,
        sample_count=int(rounded.shape[0]),
        is_downsampled=stride > 1,
        stride=stride,
        stage=stage,
        coordinate_frame=CoordinateFrame.EGO,
        xyz=rounded.tolist(),
    )


class TrackedObjectSnapshot(AdaptXModel):
    """One tracked object as the dashboard lists it: every field from the pipeline.

    Built by the backend from the track (Phase 4), its risk assessment (Phase 7)
    and its predicted path (Phase 5); the dashboard formats and draws it. The
    frame is the sensor frame, +X forward and +Y left, origin at the LiDAR
    (ADR-009), so ``longitudinal_distance_m`` is the track's ``x`` and
    ``lateral_distance_m`` its ``y`` (positive = left); ``distance_m`` is the
    risk engine's planar (XY) distance to the ego reference. Velocities are
    **ego-relative** - there is no ego-motion compensation (Experiment 014).
    ``confidence`` is the detector's geometric fit score, not a probability,
    and is null when the class is UNKNOWN. What is not measured is null.
    """

    track_id: int = Field(ge=0)
    object_class: ObjectClass
    tracking_state: TrackStatus
    distance_m: float | None = Field(
        default=None,
        ge=0.0,
        description="Planar distance from the risk assessment; null if unassessed.",
    )
    longitudinal_distance_m: float = Field(description="Track x: ahead (+) / behind (-).")
    lateral_distance_m: float = Field(description="Track y: left (+) / right (-).")
    speed_mps: float | None = Field(
        default=None, ge=0.0, description="Tracked speed, ego-relative; null until measured."
    )
    relative_speed_mps: float | None = Field(
        default=None,
        description="Closing speed from the risk assessment: positive = approaching the ego.",
    )
    risk_level: RiskLevel
    risk_score: float | None = None
    in_ego_path: bool
    path_relation: PathRelation
    confidence: float | None = Field(default=None, ge=0.0, le=1.0)
    hits: int = Field(ge=0)
    age_frames: int = Field(ge=0)
    predicted_horizon_s: float | None = Field(
        default=None, gt=0.0, description="Horizon of the predicted path, if one exists."
    )
    predicted_points: int = Field(default=0, ge=0)


class SceneSnapshot(TimestampedModel):
    """Everything the dashboard draws for one frame. Produced, never derived."""

    frame_id: int = Field(ge=0)
    sensor_id: str = Field(min_length=1)
    source: DataSource = Field(description="Provenance of the frame the pipeline processed.")
    origin: str = Field(
        min_length=1,
        description=(
            "Who handed the frame to the pipeline: 'api' for a request to the LiDAR "
            "endpoints, 'scenario:<id>' for a scenario run publishing its frames, "
            "'live:<id>' for the live simulation session."
        ),
    )
    scenario_id: str | None = None
    frame_index: int | None = Field(default=None, ge=0)
    scenario_time_s: float | None = Field(default=None, ge=0.0)
    frame_timestamp: datetime = Field(description="The frame's own (simulation) time.")

    points: PointSample | None = Field(
        default=None, description="Null when the producer did not supply points."
    )
    detection_count: int = Field(ge=0)
    tracks: list[TrackedObject] = Field(default_factory=list)
    trajectories: list[PredictedTrajectory] = Field(default_factory=list)
    assessments: list[RiskAssessment] = Field(default_factory=list)
    highest_risk_level: RiskLevel
    objects: list[TrackedObjectSnapshot] = Field(
        default_factory=list,
        description="One entry per track: the joined, backend-built object record.",
    )
    tiles: list[TileResolutionDecision] = Field(
        default_factory=list, description="Every tile's decision for this frame."
    )
    budget: ResolutionBudget | None = None
    fixed_map: SpatialMapSummary | None = None
    adaptive_map: AdaptiveSpatialMapSummary | None = None
    comparison: MappingComparison | None = None
    stage_ms: dict[str, float] = Field(
        default_factory=dict, description="Measured stage durations for this frame."
    )
    # Live simulation only (post-Phase-12 extension). The ego's own odometry
    # from the simulator, the command the controller issued for the NEXT
    # step, and the session's measured timing. None of it describes another
    # actor; ground truth about other actors has no field here and never will.
    ego: VehicleState | None = Field(
        default=None, description="Ego odometry (SIMULATION-labelled), live sessions only."
    )
    control: ControlCommand | None = Field(
        default=None, description="What the baseline controller commanded after this frame."
    )
    live: LiveFrameInfo | None = Field(
        default=None, description="Session state and measured loop timing, live sessions only."
    )

    @model_validator(mode="after")
    def _check_consistency(self) -> SceneSnapshot:
        track_ids = {track.track_id for track in self.tracks}
        for trajectory in self.trajectories:
            if trajectory.track_id not in track_ids:
                raise ValueError(
                    f"trajectory for track {trajectory.track_id} has no track in the snapshot"
                )
        for assessment in self.assessments:
            if assessment.track_id not in track_ids:
                raise ValueError(
                    f"assessment for track {assessment.track_id} has no track in the snapshot"
                )
        for item in self.objects:
            if item.track_id not in track_ids:
                raise ValueError(f"object record for track {item.track_id} has no track")
        if self.scenario_id is None and self.origin.startswith(("scenario:", "live:")):
            raise ValueError("a scenario-origin snapshot must name its scenario")
        if self.origin.startswith("live:") and self.live is None:
            raise ValueError("a live-origin snapshot must carry its live frame info")
        if self.live is not None and not self.origin.startswith("live:"):
            raise ValueError("only a live-origin snapshot carries live frame info")
        return self

    @property
    def track_count(self) -> int:
        """Live tracks in the frame."""
        return len(self.tracks)

    def counts_by_risk_level(self) -> dict[str, int]:
        """Assessments per level, UNKNOWN included and never folded into LOW."""
        counts = {level.value: 0 for level in RiskLevel}
        for assessment in self.assessments:
            counts[assessment.risk_level.value] += 1
        return counts


__all__ = [
    "DEFAULT_MAX_SAMPLE_POINTS",
    "PointSample",
    "PointStage",
    "SceneSnapshot",
    "TrackedObjectSnapshot",
    "sample_points",
]
