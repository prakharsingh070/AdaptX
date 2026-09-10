"""The conventional fixed-resolution processing baseline.

Purpose
-------
ADAPT-X's claim is that allocating spatial resolution by risk beats spending it
uniformly. Testing that claim needs something to beat. This module defines that
something: the whole pipeline run at **one fixed voxel size everywhere**, which
is what conventional LiDAR preprocessing does.

It is a named, pinned *configuration*, not a new algorithm. Every stage it uses
already exists; the baseline's contribution is fixing the parameters and
writing down the assumptions, so a future comparison is against a stated
reference rather than whatever settings happened to be in the environment.

What this baseline is **not**
-----------------------------
It is not the fixed-resolution *map* baseline of ADR-003. That one concerns the
2.5D occupancy map in Phase 5 and does not exist yet. Conflating the two would
make it look as though ADAPT-X already had a mapping baseline to compare
against. They are separate:

* **This (Phase 2C):** fixed voxel size during point-cloud processing.
* **ADR-003 (Phase 5):** fixed cell size in the 2.5D occupancy map.

Assumptions
-----------
* One voxel size for the entire scene, regardless of range, object density or
  risk. A distant pedestrian and empty tarmac get identical treatment - which
  is exactly the behaviour ADAPT-X intends to improve on.
* The sensor is mounted roughly level, so +z is up (ADR-009, ADR-013).
* Ground is locally flat within a cell (ADR-016).
* Isolated returns are noise (ADR-014).

Limitations
-----------
* Choosing the fixed size is a trade-off with no good answer: fine enough for a
  distant object means wasteful on near ground, and vice versa. That tension is
  the thing worth measuring later.
* The default below is a *starting point*, not a tuned or validated value. No
  measurement has established it as optimal for anything.
* Comparisons are only meaningful against results from the same machine,
  dataset and seed.
"""

from __future__ import annotations

from adaptx.config.settings import LiDARSettings
from adaptx.perception.pipeline import LiDARProcessingPipeline

#: Fixed voxel edge length for the baseline, in metres. A common default in
#: automotive preprocessing, chosen as a plausible starting point - not a tuned
#: or measured optimum.
BASELINE_VOXEL_SIZE_M = 0.20

#: Name recorded on every result produced with this profile.
BASELINE_PROFILE = "fixed_resolution_baseline"

#: Name recorded when only the Phase 2A stages run.
FILTER_ONLY_PROFILE = "filter_only"


def fixed_resolution_settings(
    base: LiDARSettings | None = None, *, voxel_size_m: float = BASELINE_VOXEL_SIZE_M
) -> LiDARSettings:
    """Return settings for the fixed-resolution baseline.

    Enables every processing stage and pins the voxel size. Range and ROI bounds
    come from ``base`` so the baseline can be evaluated against whatever sensing
    volume is under study.

    Args:
        base: Settings to build on. Defaults to the stock configuration.
        voxel_size_m: The single voxel size used everywhere.
    """
    source = base if base is not None else LiDARSettings()
    return source.model_copy(
        update={
            "voxel_enabled": True,
            "voxel_size_m": voxel_size_m,
            "ground_enabled": True,
            "noise_enabled": True,
            # The harness deliberately measures degenerate inputs, including a
            # zero-point scan, so the raw-input floor is lifted. That floor is a
            # policy about acceptable sensor input; it is not part of what the
            # baseline itself is testing.
            "min_points": 0,
        }
    )


def filter_only_settings(base: LiDARSettings | None = None) -> LiDARSettings:
    """Return settings with the Phase 2B stages off.

    Useful as a second reference point: it isolates what validation, ROI and
    range filtering cost on their own, so the added cost of downsampling,
    segmentation and noise filtering can be attributed rather than guessed at.
    """
    source = base if base is not None else LiDARSettings()
    return source.model_copy(
        update={
            "voxel_enabled": False,
            "ground_enabled": False,
            "noise_enabled": False,
            "min_points": 0,
        }
    )


def build_baseline_pipeline(
    base: LiDARSettings | None = None, *, voxel_size_m: float = BASELINE_VOXEL_SIZE_M
) -> LiDARProcessingPipeline:
    """Construct a pipeline configured as the fixed-resolution baseline."""
    return LiDARProcessingPipeline(fixed_resolution_settings(base, voxel_size_m=voxel_size_m))


#: Name recorded when the pipeline runs and detection follows.
DETECTION_PROFILE = "fixed_resolution_with_detection"
