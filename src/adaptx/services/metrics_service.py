"""Measured runtime metrics.

Every value reported here is measured by this process over a rolling window of
actually-ingested frames. Nothing is estimated, extrapolated or defaulted to a
plausible-looking number: an unmeasured metric is ``None`` and is named in
:attr:`~adaptx.models.system.SystemMetrics.unavailable`
(``docs/knowledge-base/20-constraints.md``).

Definitions
-----------
``fps``
    Frames ingested per second, computed from the arrival times of the frames
    in the window: ``(n - 1) / (t_last - t_first)``. Requires at least two
    frames.
``latency_ms``
    Mean server-side processing time per frame over the window: validation and
    bookkeeping, plus the preprocessing pipeline when it ran for that frame.
    It is not sensor-to-output latency, which cannot be measured until the rest
    of the perception pipeline exists.
``processing_time_ms``
    Ingest processing time of the most recent frame.
``cpu_percent`` / ``memory_mb``
    This process, via ``psutil``. CPU is measured between successive calls.
``gpu_percent``
    Not measured in Phase 1; no GPU monitoring dependency is installed.
"""

from __future__ import annotations

import time
from collections import deque
from threading import Lock

from adaptx.core.logging import get_logger
from adaptx.models.system import SystemMetrics

logger = get_logger(__name__)

try:  # psutil is a declared dependency, but degrade cleanly if it is absent.
    import psutil

    _PROCESS: psutil.Process | None = psutil.Process()
except Exception:  # pragma: no cover - only hit on an unusual install.
    psutil = None
    _PROCESS = None

_GPU_NOTE = "gpu_percent: not measured (no GPU monitoring dependency in Phase 1)"
_PSUTIL_NOTE = "cpu_percent, memory_mb: psutil unavailable in this environment"
_NO_FRAMES_NOTE = "fps, latency_ms, processing_time_ms, point_count: no frames ingested yet"
_ONE_FRAME_NOTE = "fps: needs at least two ingested frames"


class MetricsService:
    """Collects measured ingest and process metrics."""

    def __init__(self, window: int = 30) -> None:
        self._lock = Lock()
        self._arrivals: deque[float] = deque(maxlen=window)
        self._processing_times_s: deque[float] = deque(maxlen=window)
        self._last_point_count: int | None = None
        if _PROCESS is not None:
            # Prime the CPU counter so the first real reading is a measurement
            # rather than psutil's meaningless initial 0.0.
            _PROCESS.cpu_percent(interval=None)

    def record_frame(self, *, point_count: int, processing_time_s: float) -> None:
        """Record one ingested frame. Called on the hot path; must stay cheap."""
        with self._lock:
            # perf_counter, not monotonic: on Windows before Python 3.13,
            # monotonic() ticks every ~15.6 ms, so two frames recorded in the
            # same tick share a timestamp, elapsed is zero and fps is never
            # measured. perf_counter is sub-microsecond on every supported
            # interpreter and platform.
            self._arrivals.append(time.perf_counter())
            self._processing_times_s.append(processing_time_s)
            self._last_point_count = point_count

    def reset(self) -> None:
        """Clear the measurement window."""
        with self._lock:
            self._arrivals.clear()
            self._processing_times_s.clear()
            self._last_point_count = None

    def snapshot(self) -> SystemMetrics:
        """Return the current measured metrics."""
        with self._lock:
            arrivals = list(self._arrivals)
            processing = list(self._processing_times_s)
            point_count = self._last_point_count

        unavailable: list[str] = [_GPU_NOTE]
        fps: float | None = None
        latency_ms: float | None = None
        processing_time_ms: float | None = None

        if not arrivals:
            unavailable.append(_NO_FRAMES_NOTE)
        else:
            latency_ms = (sum(processing) / len(processing)) * 1000.0
            processing_time_ms = processing[-1] * 1000.0
            elapsed = arrivals[-1] - arrivals[0]
            if len(arrivals) >= 2 and elapsed > 0:
                fps = (len(arrivals) - 1) / elapsed
            else:
                unavailable.append(_ONE_FRAME_NOTE)

        cpu_percent, memory_mb = self._process_metrics()
        if cpu_percent is None:
            unavailable.append(_PSUTIL_NOTE)

        return SystemMetrics(
            fps=fps,
            latency_ms=latency_ms,
            processing_time_ms=processing_time_ms,
            cpu_percent=cpu_percent,
            gpu_percent=None,
            memory_mb=memory_mb,
            point_count=point_count,
            sample_count=len(arrivals),
            unavailable=unavailable,
        )

    @staticmethod
    def _process_metrics() -> tuple[float | None, float | None]:
        if _PROCESS is None:
            return None, None
        try:
            cpu = float(_PROCESS.cpu_percent(interval=None))
            memory = float(_PROCESS.memory_info().rss) / (1024.0 * 1024.0)
        except Exception:  # pragma: no cover - process vanished or permission denied.
            logger.warning("process metrics unavailable", exc_info=True)
            return None, None
        return cpu, memory
