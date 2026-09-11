"""Map workload (Question C) - and why map accuracy is not here.

The record holds each map's summary, not its grid, and CARLA actor ground
truth is not an occupancy reference: the buildings, poles and parked meshes
that fill most occupied cells are not actors and are not in it. Building a
reference from simulator ray casts would be a Phase 9 boundary change made
after the fact. So **mapping accuracy is not evaluated in Phase 11**, that is
written on the section, and what is reported is the measured workload of the
fixed and the adaptive map for the same frames (``docs/EVALUATION.md`` §4.5).
"""

from __future__ import annotations

from adaptx.evaluation.dataset import EvaluationDataset
from adaptx.evaluation.models import Distribution, MappingEvaluation, MapWorkload, MetricStatus

MAP_ACCURACY_REASON = (
    "mapping accuracy not evaluated in Phase 11: the run record carries map summaries, not "
    "grids, and CARLA actor ground truth is not an occupancy reference because static scene "
    "geometry is not an actor"
)


def evaluate_mapping(dataset: EvaluationDataset) -> MappingEvaluation:
    """Per-frame workload of both map variants, summarised over the run."""
    fixed = [frame.outputs.comparison.fixed for frame in dataset.frames]
    adaptive = [frame.outputs.comparison.adaptive for frame in dataset.frames]
    if not fixed:
        return MappingEvaluation(
            status=MetricStatus.UNAVAILABLE,
            reason="no evaluated frame",
            accuracy_reason=MAP_ACCURACY_REASON,
        )
    first = dataset.frames[0].outputs.fixed_map
    bounds = first.bounds
    return MappingEvaluation(
        status=MetricStatus.MEASURED,
        accuracy_status=MetricStatus.UNAVAILABLE,
        accuracy_reason=MAP_ACCURACY_REASON,
        bounds_m={
            "min_x": bounds.min_x,
            "max_x": bounds.max_x,
            "min_y": bounds.min_y,
            "max_y": bounds.max_y,
        },
        fixed_resolution_m=first.resolution.resolution_m,
        fixed=MapWorkload(
            variant="fixed",
            total_cells=Distribution.of([float(v.total_cell_count) for v in fixed]),
            occupied_cells=Distribution.of([float(v.occupied_cell_count) for v in fixed]),
            occupancy_ratio=Distribution.of([v.occupancy_ratio for v in fixed]),
            mapped_points=Distribution.of([float(v.mapped_point_count) for v in fixed]),
            out_of_bounds_points=Distribution.of(
                [float(v.out_of_bounds_point_count) for v in fixed]
            ),
            grid_bytes=Distribution.of([float(v.grid_bytes) for v in fixed]),
            duration_ms=Distribution.of([v.duration_ms for v in fixed]),
        ),
        adaptive=MapWorkload(
            variant="adaptive",
            total_cells=Distribution.of([float(v.total_cell_count) for v in adaptive]),
            occupied_cells=Distribution.of([float(v.occupied_cell_count) for v in adaptive]),
            occupancy_ratio=Distribution.of([v.occupancy_ratio for v in adaptive]),
            mapped_points=Distribution.of([float(v.mapped_point_count) for v in adaptive]),
            out_of_bounds_points=Distribution.of(
                [float(v.out_of_bounds_point_count) for v in adaptive]
            ),
            grid_bytes=Distribution.of([float(v.grid_bytes) for v in adaptive]),
            duration_ms=Distribution.of([v.duration_ms for v in adaptive]),
        ),
    )


__all__ = ["MAP_ACCURACY_REASON", "evaluate_mapping"]
