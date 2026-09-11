"""Stored evidence for the dashboard: evaluation reports and recorded runs (Phase 12).

Read-only. The service lists the JSON files in two operator-configured
directories, validates one on request against its Phase 10 or Phase 11
contract, and serves it - whole for a report, frame by frame for a run, so a
browser never parses an 80 MB record at once. Nothing is written, nothing is
recomputed; a comparison of two reports goes through the Phase 11
:func:`~adaptx.evaluation.compare.compare_reports` contract untouched.

Names are plain file names inside the configured directory. A name with a
path separator, a parent reference or the wrong extension is refused before
the file system is touched, so nothing outside the directory is reachable
(docs/DASHBOARD.md §8).
"""

from __future__ import annotations

import json
from collections import OrderedDict
from pathlib import Path
from threading import Lock
from typing import TYPE_CHECKING, TypeVar

from pydantic import Field, ValidationError

from adaptx.config.settings import DashboardSettings
from adaptx.core.exceptions import StoredEvidenceError
from adaptx.core.logging import get_logger
from adaptx.models.common import AdaptXModel

# The evaluation and scenario packages import the application context, and
# the context owns this service: importing them at module level would be a
# cycle. They are imported where they are used; the annotations are strings.
if TYPE_CHECKING:
    from adaptx.evaluation.compare import ComparisonReport
    from adaptx.evaluation.models import EvaluationReport
    from adaptx.scenarios.result import ScenarioRunResult

logger = get_logger(__name__)

ModelT = TypeVar("ModelT", bound=AdaptXModel)


class StoredFile(AdaptXModel):
    """One stored file, as listed."""

    name: str
    size_bytes: int
    modified: float


class RunSummary(AdaptXModel):
    """A recorded run without its frames: enough to choose and to label it."""

    name: str
    scenario_id: str
    scenario_name: str
    seed: int
    state: str
    map_name: str | None
    simulator_version: str
    fixed_delta_seconds: float
    frame_count: int
    planned_frame_count: int
    has_outputs: bool
    actors: list[str]
    sensor: dict[str, object] | None
    frame_ids: list[int]
    scenario_times_s: list[float]


class RunFrame(AdaptXModel):
    """One frame of a recorded run, with the pipeline outputs it carries.

    Ground truth is served **only** here, in evaluation playback, labelled as
    what it is; the live channel never carries it.
    """

    name: str
    frame: dict[str, object] = Field(description="The ScenarioFrameRecord, serialised.")
    ground_truth: dict[str, object] = Field(description="The GroundTruthFrame, serialised.")


def _safe_name(name: str, suffix: str = ".json") -> str:
    if not name or name != Path(name).name or name in {".", ".."}:
        raise StoredEvidenceError(f"'{name}' is not a plain file name", details={"name": name})
    if not name.endswith(suffix):
        raise StoredEvidenceError(f"'{name}' is not a {suffix} file", details={"name": name})
    return name


def _list(directory: Path) -> list[StoredFile]:
    if not directory.is_dir():
        return []
    files = []
    for path in sorted(directory.glob("*.json")):
        stat = path.stat()
        files.append(StoredFile(name=path.name, size_bytes=stat.st_size, modified=stat.st_mtime))
    return files


class EvidenceService:
    """Lists, validates and serves stored reports and runs, read-only."""

    def __init__(self, settings: DashboardSettings) -> None:
        self._settings = settings
        self._lock = Lock()
        self._runs: OrderedDict[str, ScenarioRunResult] = OrderedDict()

    @property
    def report_dir(self) -> Path:
        """Where evaluation reports are read from."""
        return self._settings.report_dir

    @property
    def run_dir(self) -> Path:
        """Where recorded runs are read from."""
        return self._settings.run_dir

    # -- reports -----------------------------------------------------------
    def list_reports(self) -> list[StoredFile]:
        """Every ``*.json`` in the report directory, unvalidated."""
        return _list(self.report_dir)

    def load_report(self, name: str) -> EvaluationReport:
        """Read and validate one report.

        Raises:
            StoredEvidenceError: the name is unsafe, the file is missing, or
                it is not an ``EvaluationReport``.
        """
        from adaptx.evaluation.models import EvaluationReport

        path = self.report_dir / _safe_name(name)
        return self._read(path, EvaluationReport)

    def compare(self, first: str, second: str) -> ComparisonReport:
        """Compare two stored reports with the Phase 11 contract."""
        from adaptx.evaluation.compare import compare_reports

        return compare_reports(self.load_report(first), self.load_report(second))

    # -- runs --------------------------------------------------------------
    def list_runs(self) -> list[StoredFile]:
        """Every ``*.json`` in the run directory, unvalidated."""
        return _list(self.run_dir)

    def load_run(self, name: str) -> ScenarioRunResult:
        """Read and validate one run, keeping a few parsed in memory for playback."""
        from adaptx.scenarios.result import ScenarioRunResult

        safe = _safe_name(name)
        with self._lock:
            cached = self._runs.get(safe)
            if cached is not None:
                self._runs.move_to_end(safe)
                return cached
        result = self._read(self.run_dir / safe, ScenarioRunResult)
        with self._lock:
            self._runs[safe] = result
            while len(self._runs) > self._settings.run_cache_size:
                self._runs.popitem(last=False)
        return result

    def run_summary(self, name: str) -> RunSummary:
        """The run without its frames."""
        result = self.load_run(name)
        return RunSummary(
            name=_safe_name(name),
            scenario_id=result.scenario_id,
            scenario_name=result.name,
            seed=result.seed,
            state=result.state.value,
            map_name=result.map_name,
            simulator_version=result.simulator_version,
            fixed_delta_seconds=result.fixed_delta_seconds,
            frame_count=result.frame_count,
            planned_frame_count=result.planned_frame_count,
            has_outputs=result.has_outputs,
            actors=[record.actor_id for record in result.actors],
            sensor=None if result.sensor is None else result.sensor.model_dump(mode="json"),
            frame_ids=[record.simulator_frame_id for record in result.frames],
            scenario_times_s=[record.scenario_time_s for record in result.frames],
        )

    def run_frame(self, name: str, index: int) -> RunFrame:
        """One frame of the run, with its ground truth beside it."""
        result = self.load_run(name)
        if index < 0 or index >= result.frame_count:
            raise StoredEvidenceError(
                f"frame {index} is outside the run's {result.frame_count} frames",
                details={"name": name, "index": index, "frames": result.frame_count},
            )
        return RunFrame(
            name=_safe_name(name),
            frame=result.frames[index].model_dump(mode="json"),
            ground_truth=result.ground_truth[index].model_dump(mode="json"),
        )

    # -- internals ---------------------------------------------------------
    @staticmethod
    def _read(path: Path, model: type[ModelT]) -> ModelT:
        if not path.is_file():
            raise StoredEvidenceError(f"no such file: {path.name}", details={"path": str(path)})
        try:
            text = path.read_text(encoding="utf-8")
            return model.model_validate_json(text)
        except (OSError, json.JSONDecodeError, ValidationError, ValueError) as exc:
            logger.warning(
                "stored evidence rejected",
                extra={"context": {"path": str(path), "model": model.__name__}},
            )
            raise StoredEvidenceError(
                f"{path.name} is not a valid {model.__name__}: {exc}",
                details={"path": str(path), "model": model.__name__},
            ) from exc

    def summary(self) -> dict[str, object]:
        """Compact state for status."""
        return {
            "report_dir": str(self.report_dir),
            "run_dir": str(self.run_dir),
            "reports": len(self.list_reports()),
            "runs": len(self.list_runs()),
        }


__all__ = ["EvidenceService", "RunFrame", "RunSummary", "StoredFile"]
