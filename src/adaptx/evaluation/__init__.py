"""Evaluation against simulator ground truth (Phase 11).

The one package that reads ground truth. It consumes a recorded
:class:`~adaptx.scenarios.result.ScenarioRunResult` - offline, with no
simulator - and produces an :class:`~adaptx.evaluation.models.EvaluationReport`
that says what was measured, what could not be, and why.

Nothing under :mod:`adaptx.perception`, :mod:`adaptx.tracking`,
:mod:`adaptx.prediction`, :mod:`adaptx.mapping`, :mod:`adaptx.risk` or
:mod:`adaptx.services` imports this package, and a test asserts it.

Every figure is simulation evidence and every report says so
(``docs/EVALUATION.md``).
"""

from adaptx.evaluation.compare import ComparisonReport, compare_reports
from adaptx.evaluation.dataset import EvaluationDataset, EvaluationInputError, load_dataset
from adaptx.evaluation.evaluator import evaluate
from adaptx.evaluation.models import (
    Distribution,
    EvaluationConfiguration,
    EvaluationReport,
    MetricStatus,
)
from adaptx.evaluation.report import render

__all__ = [
    "ComparisonReport",
    "Distribution",
    "EvaluationConfiguration",
    "EvaluationDataset",
    "EvaluationInputError",
    "EvaluationReport",
    "MetricStatus",
    "compare_reports",
    "evaluate",
    "load_dataset",
    "render",
]
