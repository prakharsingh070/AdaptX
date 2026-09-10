"""Fixed-versus-adaptive mapping comparison (Phase 8).

ADAPT-X asserts that allocating spatial detail by risk and uncertainty beats
allocating it uniformly. This module is how that assertion is **checked**
rather than repeated: both variants are built from the same processed frame,
and what each spent is counted.

What is compared is workload, not correctness
---------------------------------------------
Cells, bytes and durations. Nothing here measures whether either map is
*right*: no labelled reference map exists, so map correctness stays unmeasured
and unmeasurable (ADR-028). A comparison in which the adaptive map costs more
is a real result and is reported exactly like one that costs less.

The number the claim rests on
-----------------------------
``cells_on_high_priority_tiles`` against ``cells_on_low_priority_tiles``. A
uniform fine map spends most of its cells recording that nothing was observed
(Experiment 005). The adaptive map is only interesting if the cells it does
spend land where the priority actually is.
"""

from __future__ import annotations

from adaptx.models.adaptive_map import (
    BYTES_PER_CELL,
    AdaptiveSpatialMap,
    MappingComparison,
    MappingVariantMetrics,
)
from adaptx.models.adaptive_resolution import ResolutionPlan
from adaptx.models.spatial_map import SpatialMap


def describe_fixed(spatial_map: SpatialMap) -> MappingVariantMetrics:
    """Workload of one fixed-resolution map."""
    accounting = spatial_map.accounting
    return MappingVariantMetrics(
        variant="fixed",
        mapper=spatial_map.mapper,
        is_adaptive=spatial_map.is_adaptive,
        uniform_resolution_m=spatial_map.resolution_m,
        finest_resolution_m=spatial_map.resolution_m,
        coarsest_resolution_m=spatial_map.resolution_m,
        total_cell_count=accounting.total_cell_count,
        occupied_cell_count=accounting.occupied_cell_count,
        mapped_point_count=accounting.mapped_point_count,
        out_of_bounds_point_count=accounting.out_of_bounds_point_count,
        grid_bytes=accounting.total_cell_count * BYTES_PER_CELL,
        duration_ms=spatial_map.duration_ms,
    )


def describe_adaptive(adaptive_map: AdaptiveSpatialMap) -> MappingVariantMetrics:
    """Workload of one region-adaptive map.

    ``uniform_resolution_m`` is null by construction: the whole point is that
    no single cell size describes this map.
    """
    accounting = adaptive_map.accounting
    return MappingVariantMetrics(
        variant="adaptive",
        mapper=adaptive_map.mapper,
        is_adaptive=adaptive_map.is_adaptive,
        uniform_resolution_m=None,
        finest_resolution_m=adaptive_map.finest_resolution_m,
        coarsest_resolution_m=adaptive_map.coarsest_resolution_m,
        total_cell_count=accounting.total_cell_count,
        occupied_cell_count=accounting.occupied_cell_count,
        mapped_point_count=accounting.mapped_point_count,
        out_of_bounds_point_count=accounting.out_of_bounds_point_count,
        grid_bytes=adaptive_map.grid_bytes,
        duration_ms=adaptive_map.duration_ms,
    )


def partition_by_priority(plan: ResolutionPlan) -> tuple[list[int], list[int]]:
    """Split tile indices into high-priority and low-priority groups.

    High means the priority reached the configured HIGH threshold. Low means it
    fell below the MEDIUM threshold **or** no object influenced the region at
    all - a region with no evidence is not a busy one, and grouping it with the
    quiet regions is the honest reading.

    Regions between the two thresholds belong to neither group: they are the
    middle of the scene, and counting them as either would flatter or penalise
    the comparison.
    """
    configuration = plan.configuration
    high: list[int] = []
    low: list[int] = []
    for decision in plan.decisions:
        priority = decision.detail_priority
        if priority is None or priority < configuration.threshold_medium:
            low.append(decision.tile_index)
        elif priority >= configuration.threshold_high:
            high.append(decision.tile_index)
    return high, low


def compare(spatial_map: SpatialMap, adaptive_map: AdaptiveSpatialMap) -> MappingComparison:
    """Compare a fixed and an adaptive map built over the same frame.

    Args:
        spatial_map: The Phase 6 fixed-resolution baseline result.
        adaptive_map: The Phase 8 region-adaptive result.

    Returns:
        Both workloads plus how the adaptive cells were distributed across
        high-priority and low-priority regions.

    Raises:
        ValueError: the two maps did not see the same input, which would make
            every ratio meaningless.
    """
    fixed_input = spatial_map.accounting.input_point_count
    adaptive_input = adaptive_map.accounting.input_point_count
    if fixed_input != adaptive_input:
        raise ValueError(
            f"a comparison requires identical input: the fixed map saw {fixed_input} points "
            f"and the adaptive map saw {adaptive_input}"
        )

    plan = adaptive_map.plan
    high, low = partition_by_priority(plan)
    high_set, low_set = set(high), set(low)

    cells_high = sum(tile.cell_count for tile in adaptive_map.tiles if tile.tile_index in high_set)
    cells_low = sum(tile.cell_count for tile in adaptive_map.tiles if tile.tile_index in low_set)

    return MappingComparison(
        fixed=describe_fixed(spatial_map),
        adaptive=describe_adaptive(adaptive_map),
        high_priority_tile_count=len(high),
        low_priority_tile_count=len(low),
        cells_on_high_priority_tiles=cells_high,
        cells_on_low_priority_tiles=cells_low,
    )
