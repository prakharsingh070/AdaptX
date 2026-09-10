"""Object-detection result contracts (Phase 3).

The detector reports what it found *and what it discarded*. A rejected cluster
carries the reason and the measurement that triggered it, so a scene where the
detector silently sees nothing is distinguishable from one where it saw
candidates and turned them down.

Durations are measured with :func:`time.perf_counter`. Nothing here is
estimated.
"""

from __future__ import annotations

from enum import StrEnum

from pydantic import Field, model_validator

from adaptx.models.common import AdaptXModel, TimestampedModel, Vector3
from adaptx.models.objects import DetectedObject
from adaptx.models.point_cloud import PointCloudSummary


class ClusterRejection(StrEnum):
    """Why a candidate cluster did not become a detected object."""

    TOO_FEW_POINTS = "too_few_points"
    TOO_MANY_POINTS = "too_many_points"
    TOO_SHORT = "too_short"
    TOO_TALL = "too_tall"
    FOOTPRINT_TOO_SMALL = "footprint_too_small"
    FOOTPRINT_TOO_LARGE = "footprint_too_large"


class RejectedCluster(AdaptXModel):
    """A candidate cluster the filter turned down, and why."""

    cluster_id: int = Field(ge=0)
    reason: ClusterRejection
    point_count: int = Field(ge=0)
    centroid: Vector3
    extents: Vector3 = Field(description="Axis-aligned x, y, z extents in metres.")
    measured_value: float = Field(
        description="The measurement that triggered the rejection, in its own unit."
    )
    threshold: float = Field(description="The configured limit it fell outside.")


class DetectionConfiguration(AdaptXModel):
    """Effective detection configuration that produced a result.

    Carried on every result so a record is self-describing. Held as a plain
    model rather than a reference to
    :class:`~adaptx.config.settings.DetectionSettings`, keeping
    :mod:`adaptx.models` free of any dependency on the configuration layer.
    """

    cluster_tolerance_m: float
    min_cluster_points: int
    max_cluster_points: int
    min_height_m: float
    max_height_m: float
    min_footprint_m: float
    max_footprint_m: float


class DetectionResult(TimestampedModel):
    """Everything one detection pass produced.

    ``objects`` are the accepted detections; ``rejected`` explains the rest.
    ``cluster_count`` is the number of candidates found before filtering, so
    ``cluster_count == len(objects) + len(rejected)`` always holds.
    """

    frame_id: int = Field(ge=0)
    sensor_id: str = Field(min_length=1)
    detector: str = Field(min_length=1, description="Identifier of the detector that ran.")
    is_baseline: bool = Field(
        default=True,
        description="True while detection is a geometric baseline rather than a trained model.",
    )

    objects: list[DetectedObject] = Field(default_factory=list)
    rejected: list[RejectedCluster] = Field(default_factory=list)

    input_point_count: int = Field(ge=0, description="Points in the frame handed to the detector.")
    non_ground_point_count: int = Field(
        ge=0,
        description=(
            "Points actually clustered. Equals input_point_count: the detector "
            "consumes the non-ground output of the processing pipeline and does "
            "no segmentation of its own."
        ),
    )
    cluster_count: int = Field(ge=0, description="Candidate clusters before filtering.")

    duration_ms: float = Field(ge=0.0, description="Whole detection pass, measured.")
    clustering_duration_ms: float = Field(default=0.0, ge=0.0)
    classification_duration_ms: float = Field(default=0.0, ge=0.0)

    configuration: DetectionConfiguration
    input_summary: PointCloudSummary | None = Field(
        default=None, description="Metadata of the frame the detector consumed."
    )

    @model_validator(mode="after")
    def _check_cluster_accounting(self) -> DetectionResult:
        accounted = len(self.objects) + len(self.rejected)
        if self.cluster_count != accounted:
            raise ValueError(
                f"cluster_count ({self.cluster_count}) must equal accepted "
                f"({len(self.objects)}) + rejected ({len(self.rejected)})"
            )
        return self

    @property
    def object_count(self) -> int:
        """Number of accepted detections."""
        return len(self.objects)

    @property
    def rejected_count(self) -> int:
        """Number of candidate clusters filtered out."""
        return len(self.rejected)

    def counts_by_class(self) -> dict[str, int]:
        """Detections per object class, for status panels and summaries."""
        counts: dict[str, int] = {}
        for detected in self.objects:
            counts[detected.object_class.value] = counts.get(detected.object_class.value, 0) + 1
        return counts
