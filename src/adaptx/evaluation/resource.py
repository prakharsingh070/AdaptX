"""Measured cost of the run, from the durations the record carries.

Every duration was measured by the pipeline itself with ``perf_counter`` on
the machine that ran the scenario; this module only summarises. It is not a
real-time claim: no frame budget is being evaluated. Peak process memory was
not sampled during the run and is reported as unavailable rather than
approximated from anything else.
"""

from __future__ import annotations

from adaptx.evaluation.dataset import EvaluationDataset
from adaptx.evaluation.models import Distribution, MetricStatus, ResourceEvaluation

PEAK_MEMORY_REASON = (
    "peak process memory was not sampled during the scenario run; the record holds the "
    "measured size of each map grid, which is reported instead"
)


def evaluate_resource(dataset: EvaluationDataset) -> ResourceEvaluation:
    """Per-stage and whole-pipeline time per frame, and grid memory."""
    if not dataset.frames:
        empty = Distribution.of([])
        return ResourceEvaluation(
            status=MetricStatus.UNAVAILABLE,
            reason="no evaluated frame",
            pipeline_ms=empty,
            fixed_grid_bytes=empty,
            adaptive_grid_bytes=empty,
            peak_memory_reason=PEAK_MEMORY_REASON,
        )
    stages: dict[str, list[float]] = {}
    totals: list[float] = []
    for frame in dataset.frames:
        counts = frame.record.pipeline
        assert counts is not None  # an evaluated frame is a processed frame
        for stage, duration in counts.stage_ms.items():
            stages.setdefault(stage, []).append(duration)
        totals.append(counts.total_ms)
    return ResourceEvaluation(
        status=MetricStatus.MEASURED,
        stage_ms={stage: Distribution.of(values) for stage, values in stages.items()},
        pipeline_ms=Distribution.of(totals),
        fixed_grid_bytes=Distribution.of(
            [float(f.outputs.comparison.fixed.grid_bytes) for f in dataset.frames]
        ),
        adaptive_grid_bytes=Distribution.of(
            [float(f.outputs.comparison.adaptive.grid_bytes) for f in dataset.frames]
        ),
        peak_memory_status=MetricStatus.UNAVAILABLE,
        peak_memory_reason=PEAK_MEMORY_REASON,
    )


__all__ = ["PEAK_MEMORY_REASON", "evaluate_resource"]
