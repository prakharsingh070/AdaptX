"""Point-cloud preprocessing result contracts (Phase 2A).

The pipeline reports what it did to every frame: how many points entered, how
many each stage rejected and why, how many survived, and how long it took.
Durations are measured with a monotonic clock by the code that ran the stage;
nothing here is estimated.
"""

from __future__ import annotations

from enum import StrEnum

from pydantic import ConfigDict, Field, model_validator

from adaptx.models.common import AdaptXModel, TimestampedModel
from adaptx.models.point_cloud import PointCloudFrame, PointCloudSummary


class ProcessingStage(StrEnum):
    """A stage of the preprocessing pipeline, in execution order.

    The first four always run (Phase 2A). The last three are opt-in
    (Phase 2B, ADR-012) and appear only when they were enabled.
    """

    VALIDATION = "validation"
    INVALID_REMOVAL = "invalid_removal"
    ROI_FILTER = "roi_filter"
    RANGE_FILTER = "range_filter"
    VOXEL_DOWNSAMPLE = "voxel_downsample"
    GROUND_SEGMENTATION = "ground_segmentation"
    NOISE_FILTER = "noise_filter"


class StageMetrics(AdaptXModel):
    """What one stage did to the frame.

    ``rejected_points`` counts points that did not continue past this stage,
    out of the points that reached it, so the stage counts partition the input
    without double counting a point already rejected upstream.

    For ground segmentation "rejected" means *separated out as ground* rather
    than discarded: those points are kept and returned separately.
    """

    stage: ProcessingStage
    input_points: int = Field(ge=0)
    output_points: int = Field(ge=0)
    rejected_points: int = Field(ge=0)
    duration_ms: float = Field(
        default=0.0,
        ge=0.0,
        description=(
            "Time spent in this stage's own computation, measured with "
            "time.perf_counter. Stage durations sum to less than the frame total; "
            "the difference is reported as overhead_ms."
        ),
    )

    @model_validator(mode="after")
    def _check_conservation(self) -> StageMetrics:
        if self.input_points != self.output_points + self.rejected_points:
            raise ValueError(
                f"stage {self.stage}: input_points ({self.input_points}) must equal "
                f"output_points ({self.output_points}) + rejected_points "
                f"({self.rejected_points})"
            )
        return self


class ProcessingMetrics(TimestampedModel):
    """JSON-safe record of one frame's trip through the pipeline.

    ``duration_ms`` is wall-clock time measured with :func:`time.perf_counter`
    around the whole pipeline. It is a measurement of this run on this machine,
    not a performance claim.
    """

    processor: str = Field(min_length=1)
    input_point_count: int = Field(ge=0)
    invalid_point_count: int = Field(
        ge=0, description="Points dropped for containing NaN or an infinity."
    )
    roi_rejected_count: int = Field(ge=0, description="Finite points outside the ROI box.")
    range_rejected_count: int = Field(
        ge=0, description="Points inside the ROI but outside the range band."
    )
    voxel_reduced_count: int = Field(
        default=0, ge=0, description="Points merged away by voxel downsampling."
    )
    ground_point_count: int = Field(
        default=0,
        ge=0,
        description="Points classified as ground and returned separately, not discarded.",
    )
    noise_removed_count: int = Field(
        default=0, ge=0, description="Points dropped as outliers by the noise filter."
    )
    output_point_count: int = Field(
        ge=0, description="Points in the final frame. Excludes ground when segmentation ran."
    )
    duration_ms: float = Field(
        ge=0.0, description="Wall-clock time for the whole pipeline on this frame."
    )
    stages: list[StageMetrics] = Field(default_factory=list)

    @model_validator(mode="after")
    def _check_conservation(self) -> ProcessingMetrics:
        accounted = (
            self.invalid_point_count
            + self.roi_rejected_count
            + self.range_rejected_count
            + self.voxel_reduced_count
            + self.ground_point_count
            + self.noise_removed_count
        )
        if self.input_point_count != accounted + self.output_point_count:
            raise ValueError(
                f"input_point_count ({self.input_point_count}) must equal accounted-for "
                f"({accounted}) + output_point_count ({self.output_point_count})"
            )
        return self

    @property
    def stage_duration_ms(self) -> float:
        """Sum of the individually measured stage durations."""
        return sum(stage.duration_ms for stage in self.stages)

    @property
    def overhead_ms(self) -> float:
        """Measured time outside the stages themselves.

        Frame construction, the single array compaction shared by the filtering
        stages, summaries and metric assembly. Reported rather than hidden or
        silently folded into a stage, so no stage duration is inflated.
        """
        return max(0.0, self.duration_ms - self.stage_duration_ms)

    @property
    def non_ground_point_count(self) -> int:
        """Points that survived as non-ground, i.e. the final output count."""
        return self.output_point_count

    @property
    def voxel_reduction_ratio(self) -> float | None:
        """Fraction of points voxelisation merged away, or ``None`` if it did not run."""
        entered = self.input_point_count - (
            self.invalid_point_count + self.roi_rejected_count + self.range_rejected_count
        )
        if entered == 0:
            return None
        return self.voxel_reduced_count / entered

    @property
    def retention_ratio(self) -> float | None:
        """Fraction of input points that survived, or ``None`` for an empty frame."""
        if self.input_point_count == 0:
            return None
        return self.output_point_count / self.input_point_count


class PipelineConfiguration(AdaptXModel):
    """The effective processing configuration that produced a result.

    Carried on every result so a record is self-describing: a benchmark or a
    replayed frame can be reproduced from what it reports, without consulting
    the environment it happened to run in.

    Held as a plain model rather than a reference to
    :class:`~adaptx.config.settings.LiDARSettings` so that :mod:`adaptx.models`
    keeps no dependency on the configuration layer; the pipeline builds it.
    """

    min_range_m: float
    max_range_m: float
    roi_x_min_m: float
    roi_x_max_m: float
    roi_y_min_m: float
    roi_y_max_m: float
    roi_z_min_m: float
    roi_z_max_m: float

    voxel_enabled: bool
    voxel_size_m: float

    ground_enabled: bool
    ground_cell_size_m: float
    ground_height_tolerance_m: float
    ground_max_height_m: float | None

    noise_enabled: bool
    noise_cell_size_m: float
    noise_min_neighbors: int

    @property
    def enabled_optional_stages(self) -> list[ProcessingStage]:
        """The opt-in stages this configuration turns on, in pipeline order."""
        stages = []
        if self.voxel_enabled:
            stages.append(ProcessingStage.VOXEL_DOWNSAMPLE)
        if self.ground_enabled:
            stages.append(ProcessingStage.GROUND_SEGMENTATION)
        if self.noise_enabled:
            stages.append(ProcessingStage.NOISE_FILTER)
        return stages


class PointCloudProcessingResult(AdaptXModel):
    """The processed frame plus the record of how it was produced.

    Holds a NumPy array through ``frame`` and so is not itself JSON-safe; the
    API serialises :attr:`metrics` and :attr:`output_summary` instead.
    """

    model_config = ConfigDict(
        extra="forbid", validate_assignment=True, arbitrary_types_allowed=True
    )

    frame: PointCloudFrame = Field(
        description=(
            "Cleaned, filtered, all-finite output frame. When ground segmentation ran "
            "this holds the non-ground points only."
        )
    )
    input_summary: PointCloudSummary = Field(description="Metadata of the frame as received.")
    metrics: ProcessingMetrics
    configuration: PipelineConfiguration = Field(
        description="Effective configuration that produced this result."
    )
    ground_frame: PointCloudFrame | None = Field(
        default=None,
        description=(
            "Points classified as ground, or null when ground segmentation did not run. "
            "Ground points bypass the noise filter, which is applied to non-ground only."
        ),
    )

    @property
    def output_summary(self) -> PointCloudSummary:
        """Metadata of the processed frame, safe to serialise."""
        return self.frame.summary()

    @property
    def ground_summary(self) -> PointCloudSummary | None:
        """Metadata of the ground frame, or ``None`` when segmentation did not run."""
        return None if self.ground_frame is None else self.ground_frame.summary()
