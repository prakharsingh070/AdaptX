"""LiDAR frame ingestion.

Phase 1 accepts frames, validates them against the point-cloud contract and the
configured limits, records measured ingest metrics and keeps the metadata of
the most recent frame. Nothing is detected, mapped or tracked: those modules do
not exist yet.

The reported LiDAR status is derived from frames that were actually received
and from their declared provenance, so a synthetic or replayed feed can never
present itself as a live sensor.
"""

from __future__ import annotations

import time
from threading import Lock

from adaptx.config.settings import LiDARSettings
from adaptx.core.logging import get_logger
from adaptx.models.common import DataSource
from adaptx.models.point_cloud import PointCloudFrame, PointCloudSummary
from adaptx.models.system import LiDARSourceStatus, LiDARStatus
from adaptx.perception.lidar import FrameValidationProcessor
from adaptx.services.metrics_service import MetricsService

logger = get_logger(__name__)

#: Provenances that mean "not a live sensor".
_NON_LIVE_SOURCES = frozenset({DataSource.SIMULATION, DataSource.REPLAY, DataSource.SYNTHETIC_TEST})


class LiDARIngestService:
    """Validates and accounts for incoming point-cloud frames."""

    def __init__(
        self,
        settings: LiDARSettings,
        processor: FrameValidationProcessor,
        metrics: MetricsService,
    ) -> None:
        self._settings = settings
        self._processor = processor
        self._metrics = metrics
        self._lock = Lock()
        self._last_summary: PointCloudSummary | None = None
        self._last_received_monotonic: float | None = None
        self._frames_received = 0

    def ingest(self, frame: PointCloudFrame) -> PointCloudSummary:
        """Validate ``frame``, record metrics and return its summary.

        Raises:
            adaptx.core.exceptions.InvalidPointCloudError: the frame violates
                the contract or the configured size limits.
        """
        started = time.perf_counter()
        processed = self._processor.process(frame)
        summary = processed.summary()
        elapsed = time.perf_counter() - started

        with self._lock:
            self._last_summary = summary
            self._last_received_monotonic = time.monotonic()
            self._frames_received += 1

        self._metrics.record_frame(point_count=summary.point_count, processing_time_s=elapsed)
        # DEBUG: this runs once per frame and must not flood the log.
        logger.debug(
            "lidar frame ingested",
            extra={
                "context": {
                    "frame_id": summary.frame_id,
                    "sensor_id": summary.sensor_id,
                    "point_count": summary.point_count,
                    "source": summary.source.value,
                }
            },
        )
        return summary

    @property
    def last_summary(self) -> PointCloudSummary | None:
        """Metadata of the most recently ingested frame, if any."""
        with self._lock:
            return self._last_summary

    @property
    def frames_received(self) -> int:
        """Total frames accepted since startup."""
        with self._lock:
            return self._frames_received

    def status(self) -> LiDARSourceStatus:
        """Derive the LiDAR channel status from received frames."""
        with self._lock:
            summary = self._last_summary
            received_at = self._last_received_monotonic
            count = self._frames_received

        if summary is None or received_at is None:
            return LiDARSourceStatus(
                status=LiDARStatus.DISCONNECTED,
                source=DataSource.UNAVAILABLE,
                frames_received=count,
                detail="no point-cloud frame has been received",
            )

        age_s = time.monotonic() - received_at
        if age_s > self._settings.frame_stale_after_s:
            return LiDARSourceStatus(
                status=LiDARStatus.DISCONNECTED,
                source=summary.source,
                last_frame_id=summary.frame_id,
                last_frame_age_s=age_s,
                frames_received=count,
                detail=(
                    f"last frame is {age_s:.1f}s old, older than the "
                    f"{self._settings.frame_stale_after_s:.1f}s staleness limit"
                ),
            )

        is_live = summary.source not in _NON_LIVE_SOURCES
        return LiDARSourceStatus(
            status=LiDARStatus.CONNECTED if is_live else LiDARStatus.SIMULATED,
            source=summary.source,
            last_frame_id=summary.frame_id,
            last_frame_age_s=age_s,
            frames_received=count,
            detail=(
                "receiving frames"
                if is_live
                else f"receiving frames labelled '{summary.source.value}', not sensor data"
            ),
        )
