"""Repeatability: do two evaluations of the same scenario agree?

The same scenario, seed and configuration must produce identical
**deterministic content** - matches, errors, cells, levels, churn - and will
produce different **timings**, because a duration is a measurement of the
machine on the day. This module draws exactly that line: every field whose
name marks it as a duration or a ratio of durations is excluded, along with
the identity fields (``evaluation_id``, ``timestamp``, ``git_commit``,
``run_timestamp``) and the whole ``resource`` section. Everything else must
match to the digit.
"""

from __future__ import annotations

from typing import Any

from pydantic import Field

from adaptx.evaluation.models import EvaluationReport
from adaptx.models.common import AdaptXModel

#: Top-level fields that identify an evaluation rather than describe a run.
#: ``adaptx_version`` is deliberately not here: reports from different
#: versions of the code are not repeats of one another.
IDENTITY_FIELDS: frozenset[str] = frozenset(
    {"evaluation_id", "timestamp", "git_commit", "run_timestamp"}
)

#: Field names, at any depth, that carry a measured duration or a ratio of
#: durations and are therefore expected to differ between repeats.
TIMING_FIELDS: frozenset[str] = frozenset(
    {"duration_ms", "mapping_duration_ratio", "total_duration_ratio", "stage_ms", "pipeline_ms"}
)

#: Sections that consist entirely of timing.
TIMING_SECTIONS: frozenset[str] = frozenset({"resource"})

#: Field names, at any depth, that the simulator assigns per session. Two
#: runs of one scenario give the same actor different ids; the scenario-level
#: ``actor_id`` name is the stable identity and is compared.
SESSION_IDENTITY_FIELDS: frozenset[str] = frozenset({"simulator_actor_id"})


class ComparisonReport(AdaptXModel):
    """The outcome of comparing two evaluation reports for repeatability."""

    scenario_id: str
    same_scenario: bool
    same_configuration: bool
    deterministic_content_identical: bool
    differences: list[str] = Field(
        default_factory=list, description="Paths whose deterministic content differs."
    )
    timing_fields_ignored: list[str] = Field(
        default_factory=list, description="Paths excluded because they carry timings."
    )

    @property
    def repeatable(self) -> bool:
        """True when both reports describe the same scenario identically."""
        return (
            self.same_scenario and self.same_configuration and self.deterministic_content_identical
        )


def compare_reports(first: EvaluationReport, second: EvaluationReport) -> ComparisonReport:
    """Compare deterministic content, ignoring identity and timing."""
    left = first.model_dump(mode="json")
    right = second.model_dump(mode="json")
    differences: list[str] = []
    ignored: list[str] = []
    _diff(left, right, "", differences, ignored)
    return ComparisonReport(
        scenario_id=first.scenario_id,
        same_scenario=(
            first.scenario_id == second.scenario_id
            and first.seed == second.seed
            and first.fixed_delta_seconds == second.fixed_delta_seconds
        ),
        same_configuration=first.configuration == second.configuration,
        deterministic_content_identical=not differences,
        differences=differences,
        timing_fields_ignored=ignored,
    )


def _diff(left: Any, right: Any, path: str, differences: list[str], ignored: list[str]) -> None:
    if isinstance(left, dict) and isinstance(right, dict):
        for key in sorted(set(left) | set(right)):
            child = f"{path}.{key}" if path else key
            if (not path and key in IDENTITY_FIELDS) or (not path and key in TIMING_SECTIONS):
                ignored.append(child)
                continue
            if key in TIMING_FIELDS or key in SESSION_IDENTITY_FIELDS:
                ignored.append(child)
                continue
            if key not in left or key not in right:
                differences.append(f"{child}: present on one side only")
                continue
            _diff(left[key], right[key], child, differences, ignored)
        return
    if isinstance(left, list) and isinstance(right, list):
        if len(left) != len(right):
            differences.append(f"{path}: {len(left)} items vs {len(right)}")
            return
        for index, (a, b) in enumerate(zip(left, right, strict=True)):
            _diff(a, b, f"{path}[{index}]", differences, ignored)
        return
    if left != right:
        differences.append(f"{path}: {left!r} vs {right!r}")


__all__ = [
    "IDENTITY_FIELDS",
    "SESSION_IDENTITY_FIELDS",
    "TIMING_FIELDS",
    "ComparisonReport",
    "compare_reports",
]
