"""Evaluate one run record into one report.

The only entry point the CLI and the tests need::

    report = evaluate(result)                      # defaults
    report = evaluate(result, configuration=...)   # explicit thresholds

Deterministic: the same record and configuration produce the same report
content. The only fields that differ between two evaluations of one record
are ``evaluation_id``, ``timestamp`` and ``git_commit``.
"""

from __future__ import annotations

import subprocess
import uuid
from pathlib import Path

from adaptx import __version__
from adaptx.core.logging import get_logger
from adaptx.evaluation.adaptive import evaluate_adaptive
from adaptx.evaluation.dataset import EvaluationDataset, load_dataset
from adaptx.evaluation.mapping import evaluate_mapping
from adaptx.evaluation.models import EvaluationConfiguration, EvaluationReport
from adaptx.evaluation.prediction import evaluate_prediction
from adaptx.evaluation.resource import evaluate_resource
from adaptx.evaluation.risk import evaluate_risk
from adaptx.evaluation.tracking import evaluate_detection, evaluate_tracking, match_tracks
from adaptx.scenarios.result import ScenarioRunResult

logger = get_logger(__name__)


def evaluate(
    result: ScenarioRunResult,
    *,
    configuration: EvaluationConfiguration | None = None,
    git_commit: str | None = None,
) -> EvaluationReport:
    """Evaluate ``result`` and return the report.

    Args:
        result: A run record carrying pipeline outputs.
        configuration: Thresholds; the baseline defaults when omitted.
        git_commit: Commit to record; read from the working tree when omitted
            and left ``None`` when that fails.

    Raises:
        EvaluationInputError: the record cannot support an evaluation.
    """
    config = configuration if configuration is not None else EvaluationConfiguration()
    dataset = load_dataset(result, include_map_actors=config.include_map_actors)
    primary = match_tracks(dataset, gate_m=config.primary_gate_m)

    report = EvaluationReport(
        evaluation_id=uuid.uuid4().hex,
        scenario_id=result.scenario_id,
        scenario_name=result.name,
        seed=result.seed,
        run_state=result.state,
        run_timestamp=result.timestamp,
        map_name=result.map_name,
        simulator_version=result.simulator_version,
        fixed_delta_seconds=result.fixed_delta_seconds,
        frame_count=result.frame_count,
        planned_frame_count=result.planned_frame_count,
        evaluated_frame_count=len(dataset.frames),
        sensor=result.sensor,
        sensor_mount=dataset.sensor_mount,
        adaptx_version=__version__,
        git_commit=git_commit if git_commit is not None else read_git_commit(),
        configuration=config,
        pipeline_configuration=_pipeline_configuration(dataset),
        scenario_actors=dataset.scenario_actor_ids,
        detection=evaluate_detection(dataset, config),
        tracking=evaluate_tracking(dataset, config, primary),
        prediction=evaluate_prediction(dataset, config, primary),
        risk=evaluate_risk(dataset, config, primary),
        mapping=evaluate_mapping(dataset),
        adaptive_resolution=evaluate_adaptive(dataset, config, primary),
        resource=evaluate_resource(dataset),
        warnings=list(dataset.warnings),
    )
    if not result.completed:
        report.warnings.append(
            f"the run did not complete: state {result.state.value}, "
            f"{result.frame_count} of {result.planned_frame_count} frames"
        )
    logger.info(
        "scenario evaluated",
        extra={
            "context": {
                "scenario": result.scenario_id,
                "frames": len(dataset.frames),
                "evaluation_id": report.evaluation_id,
            }
        },
    )
    return report


def _pipeline_configuration(dataset: EvaluationDataset) -> dict[str, dict[str, object]]:
    """Effective stage configurations, from the first evaluated frame's outputs."""
    if not dataset.frames:
        return {}
    outputs = dataset.frames[0].outputs
    return {
        "tracking": outputs.tracking.configuration.model_dump(mode="json"),
        "prediction": outputs.prediction.configuration.model_dump(mode="json"),
        "risk": outputs.risk.configuration.model_dump(mode="json"),
        "mapping": outputs.fixed_map.configuration.model_dump(mode="json"),
        "adaptive_resolution": outputs.plan.configuration.model_dump(mode="json"),
    }


def read_git_commit(path: Path | None = None) -> str | None:
    """The commit of the checkout at ``path``, or ``None`` when it cannot be read.

    Best effort by design: an evaluation outside a repository is still an
    evaluation, and it must say "unknown" rather than fail or invent one.
    """
    try:
        completed = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            check=False,
            cwd=path,
            timeout=10.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if completed.returncode != 0:
        return None
    commit = completed.stdout.strip()
    return commit or None


__all__ = ["evaluate", "read_git_commit"]
