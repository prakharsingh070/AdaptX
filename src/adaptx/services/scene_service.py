"""The latest scene, for the dashboard (Phase 12).

Holds exactly one :class:`~adaptx.models.scene.SceneSnapshot` - the most
recent frame the pipeline produced or was handed - and a sequence number
that changes whenever it is replaced. The live channel (``/ws/scene``)
watches the sequence; the REST endpoint returns the snapshot on demand.

The service computes nothing. :func:`build_snapshot` bundles results the
pipeline already returned; ``publish`` stores them. There is no history: a
dashboard that reconnects sees the current frame, not the past (ADR-055).
"""

from __future__ import annotations

from datetime import datetime
from threading import Lock

import numpy as np

from adaptx.control.models import ControlCommand
from adaptx.core.logging import get_logger
from adaptx.live.models import LiveFrameInfo
from adaptx.models.adaptive_map import AdaptiveSpatialMapSummary, MappingComparison
from adaptx.models.adaptive_resolution import ResolutionPlan
from adaptx.models.common import DataSource
from adaptx.models.detection import DetectionResult
from adaptx.models.prediction_result import PredictionResult
from adaptx.models.risk_assessment import RiskAssessmentResult
from adaptx.models.scene import (
    DEFAULT_MAX_SAMPLE_POINTS,
    PointStage,
    SceneSnapshot,
    sample_points,
)
from adaptx.models.spatial_map import SpatialMapSummary
from adaptx.models.tracking_result import TrackingResult
from adaptx.models.vehicle import VehicleState

logger = get_logger(__name__)


def build_snapshot(
    *,
    frame_id: int,
    sensor_id: str,
    source: DataSource,
    frame_timestamp: datetime,
    origin: str,
    points: np.ndarray | None,
    point_stage: PointStage,
    detection: DetectionResult,
    tracking: TrackingResult,
    prediction: PredictionResult,
    risk: RiskAssessmentResult,
    plan: ResolutionPlan,
    adaptive_map: AdaptiveSpatialMapSummary,
    fixed_map: SpatialMapSummary | None,
    comparison: MappingComparison | None,
    processing_ms: float,
    scenario_id: str | None = None,
    frame_index: int | None = None,
    scenario_time_s: float | None = None,
    max_points: int = DEFAULT_MAX_SAMPLE_POINTS,
    ego: VehicleState | None = None,
    control: ControlCommand | None = None,
    live: LiveFrameInfo | None = None,
) -> SceneSnapshot:
    """Bundle one frame's pipeline outputs for display. Nothing is derived."""
    return SceneSnapshot(
        ego=ego,
        control=control,
        live=live,
        timestamp=frame_timestamp,
        frame_id=frame_id,
        sensor_id=sensor_id,
        source=source,
        origin=origin,
        scenario_id=scenario_id,
        frame_index=frame_index,
        scenario_time_s=scenario_time_s,
        frame_timestamp=frame_timestamp,
        points=(
            None
            if points is None
            else sample_points(points, max_points=max_points, stage=point_stage)
        ),
        detection_count=len(detection.objects),
        tracks=list(tracking.tracks),
        trajectories=list(prediction.trajectories),
        assessments=list(risk.assessments),
        highest_risk_level=risk.highest_risk_level,
        tiles=list(plan.decisions),
        budget=plan.budget,
        fixed_map=fixed_map,
        adaptive_map=adaptive_map,
        comparison=comparison,
        stage_ms={
            "processing": processing_ms,
            "detection": detection.duration_ms,
            "tracking": tracking.duration_ms,
            "prediction": prediction.duration_ms,
            "mapping": 0.0 if fixed_map is None else fixed_map.duration_ms,
            "risk": risk.duration_ms,
            "controller": plan.duration_ms,
            "adaptive_mapping": adaptive_map.mapping_duration_ms,
        },
    )


class SceneService:
    """Keeps the most recent scene snapshot and a sequence number."""

    def __init__(self) -> None:
        self._lock = Lock()
        self._latest: SceneSnapshot | None = None
        self._sequence = 0

    @property
    def sequence(self) -> int:
        """Increments on every publish; 0 until the first."""
        with self._lock:
            return self._sequence

    def latest(self) -> tuple[int, SceneSnapshot | None]:
        """The current sequence and snapshot, read together."""
        with self._lock:
            return self._sequence, self._latest

    def publish(self, snapshot: SceneSnapshot) -> int:
        """Replace the current snapshot. Returns the new sequence number."""
        with self._lock:
            self._sequence += 1
            self._latest = snapshot
            sequence = self._sequence
        logger.debug(
            "scene published",
            extra={
                "context": {
                    "sequence": sequence,
                    "frame_id": snapshot.frame_id,
                    "origin": snapshot.origin,
                    "tracks": snapshot.track_count,
                }
            },
        )
        return sequence

    def reset(self) -> None:
        """Forget the current snapshot; the sequence keeps counting."""
        with self._lock:
            self._sequence += 1
            self._latest = None

    def summary(self) -> dict[str, object]:
        """Compact state for status and telemetry: never the snapshot itself."""
        sequence, latest = self.latest()
        return {
            "sequence": sequence,
            "has_scene": latest is not None,
            "frame_id": None if latest is None else latest.frame_id,
            "origin": None if latest is None else latest.origin,
            "track_count": None if latest is None else latest.track_count,
        }


__all__ = ["SceneService", "build_snapshot"]
