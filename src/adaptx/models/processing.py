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
    """A stage of the Phase 2A preprocessing pipeline, in execution order."""

    VALIDATION = "validation"
    INVALID_REMOVAL = "invalid_removal"
    ROI_FILTER = "roi_filter"
    RANGE_FILTER = "range_filter"


class StageMetrics(AdaptXModel):
    """What one stage did to the frame.

    ``rejected_points`` counts points this stage removed *from the points that
    reached it*, so the stage counts partition the input without double
    counting a point rejected by an earlier stage.
    """

    stage: ProcessingStage
    input_points: int = Field(ge=0)
    output_points: int = Field(ge=0)
    rejected_points: int = Field(ge=0)

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
    output_point_count: int = Field(ge=0)
    duration_ms: float = Field(ge=0.0)
    stages: list[StageMetrics] = Field(default_factory=list)

    @model_validator(mode="after")
    def _check_conservation(self) -> ProcessingMetrics:
        removed = self.invalid_point_count + self.roi_rejected_count + self.range_rejected_count
        if self.input_point_count != removed + self.output_point_count:
            raise ValueError(
                f"input_point_count ({self.input_point_count}) must equal removed "
                f"({removed}) + output_point_count ({self.output_point_count})"
            )
        return self

    @property
    def retention_ratio(self) -> float | None:
        """Fraction of input points that survived, or ``None`` for an empty frame."""
        if self.input_point_count == 0:
            return None
        return self.output_point_count / self.input_point_count


class PointCloudProcessingResult(AdaptXModel):
    """The processed frame plus the record of how it was produced.

    Holds a NumPy array through ``frame`` and so is not itself JSON-safe; the
    API serialises :attr:`metrics` and :attr:`output_summary` instead.
    """

    model_config = ConfigDict(
        extra="forbid", validate_assignment=True, arbitrary_types_allowed=True
    )

    frame: PointCloudFrame = Field(description="Cleaned, filtered, all-finite output frame.")
    input_summary: PointCloudSummary = Field(description="Metadata of the frame as received.")
    metrics: ProcessingMetrics

    @property
    def output_summary(self) -> PointCloudSummary:
        """Metadata of the processed frame, safe to serialise."""
        return self.frame.summary()
