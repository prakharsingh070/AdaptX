"""Publishing a scenario run's frames to the dashboard (Phase 12).

``python -m adaptx.scenarios run <id> --publish http://127.0.0.1:8000`` hands
each processed frame to the backend's ``POST /api/v1/scene/frame`` so the
dashboard can draw it as it happens. The scenario's own pipeline does the
processing exactly as before and the record it writes is unchanged; the
publisher is a :data:`~adaptx.scenarios.runner.FrameObserver`, so it sees the
sensor frame and the outputs and **never ground truth** (ADR-045, ADR-055).

A dashboard that is not running must not fail a scenario run: connection
errors are logged, counted and, after a few in a row, the publisher stops
trying. Standard library only - no HTTP client dependency for a local POST.
"""

from __future__ import annotations

import urllib.error
import urllib.request

from adaptx.core.logging import get_logger
from adaptx.models.point_cloud import RawPointCloudFrame
from adaptx.models.scene import DEFAULT_MAX_SAMPLE_POINTS, PointStage
from adaptx.scenarios.result import PipelineFrameOutputs, StageCounts
from adaptx.services.scene_service import build_snapshot

logger = get_logger(__name__)

#: Consecutive failures after which the publisher gives up for the run.
MAX_CONSECUTIVE_FAILURES = 3


class ScenePublisher:
    """POSTs one :class:`~adaptx.models.scene.SceneSnapshot` per processed frame."""

    def __init__(
        self,
        base_url: str,
        *,
        scenario_id: str,
        max_points: int = DEFAULT_MAX_SAMPLE_POINTS,
        timeout_s: float = 2.0,
    ) -> None:
        self._url = base_url.rstrip("/") + "/api/v1/scene/frame"
        self._scenario_id = scenario_id
        self._max_points = max_points
        self._timeout_s = timeout_s
        self.published = 0
        self.failed = 0
        self._consecutive_failures = 0
        self.disabled = False

    @property
    def url(self) -> str:
        """Where snapshots are sent."""
        return self._url

    def __call__(
        self,
        frame: RawPointCloudFrame,
        frame_index: int,
        scenario_time_s: float,
        produced: StageCounts | PipelineFrameOutputs | None,
    ) -> None:
        if self.disabled or not isinstance(produced, PipelineFrameOutputs):
            return
        snapshot = build_snapshot(
            frame_id=frame.frame_id,
            sensor_id=frame.sensor_id,
            source=frame.source,
            frame_timestamp=frame.timestamp,
            origin=f"scenario:{self._scenario_id}",
            points=frame.points,
            point_stage=PointStage.RAW,
            detection=produced.detection,
            tracking=produced.tracking,
            prediction=produced.prediction,
            risk=produced.risk,
            plan=produced.plan,
            adaptive_map=produced.adaptive_map,
            fixed_map=produced.fixed_map,
            comparison=produced.comparison,
            processing_ms=produced.processing_ms,
            scenario_id=self._scenario_id,
            frame_index=frame_index,
            scenario_time_s=scenario_time_s,
            max_points=self._max_points,
        )
        body = snapshot.model_dump_json().encode("utf-8")
        request = urllib.request.Request(
            self._url, data=body, method="POST", headers={"Content-Type": "application/json"}
        )
        try:
            with urllib.request.urlopen(request, timeout=self._timeout_s) as response:
                response.read()
        except (urllib.error.URLError, OSError, ValueError) as exc:
            self.failed += 1
            self._consecutive_failures += 1
            logger.warning(
                "scene publish failed",
                extra={"context": {"url": self._url, "frame": frame.frame_id, "error": str(exc)}},
            )
            if self._consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
                self.disabled = True
                logger.warning(
                    "scene publishing disabled for the rest of the run",
                    extra={"context": {"url": self._url, "failures": self.failed}},
                )
            return
        self.published += 1
        self._consecutive_failures = 0


__all__ = ["MAX_CONSECUTIVE_FAILURES", "ScenePublisher"]
