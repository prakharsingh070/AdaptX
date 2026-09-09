"""Risk and uncertainty contracts.

Risk is normalised to ``[0, 1]`` throughout the backend (ADR-006). The
``risk_percent`` view exists for dashboards that render a 0-100 scale, matching
the illustrative JSON in ``docs/knowledge-base/17_data-schema.md``.

Risk and uncertainty are separate quantities: an object can be low-risk but
uncertain, or high-risk and well observed
(``docs/knowledge-base/07_uncertainty.md``).
"""

from __future__ import annotations

from enum import StrEnum

from pydantic import Field

from adaptx.models.common import (
    AdaptXModel,
    CoordinateFrame,
    DataSource,
    TimestampedModel,
)
from adaptx.models.map import ResolutionLevel


class RiskLevel(StrEnum):
    """Discretised risk band derived from a normalised risk score."""

    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


class RiskFactors(AdaptXModel):
    """Normalised contributions behind a risk score.

    Populated by a risk-engine implementation so the dashboard and event replay
    can explain *why* a region became risky. All fields are optional because a
    given engine may not model every factor.
    """

    proximity: float | None = Field(default=None, ge=0.0, le=1.0)
    relative_velocity: float | None = Field(default=None, ge=0.0, le=1.0)
    time_to_collision: float | None = Field(default=None, ge=0.0, le=1.0)
    trajectory_overlap: float | None = Field(default=None, ge=0.0, le=1.0)
    object_importance: float | None = Field(default=None, ge=0.0, le=1.0)
    uncertainty: float | None = Field(default=None, ge=0.0, le=1.0)


class RiskCell(TimestampedModel):
    """Risk associated with one spatial cell.

    ``x``/``y`` are the cell-centre coordinates in metres within
    ``coordinate_frame``; ``resolution_m`` is the cell edge length.
    """

    x: float
    y: float
    resolution_m: float = Field(gt=0.0)
    risk_score: float = Field(ge=0.0, le=1.0, description="Normalised risk in [0, 1].")
    risk_level: RiskLevel = RiskLevel.LOW
    uncertainty: float = Field(default=0.0, ge=0.0, le=1.0)
    resolution_level: ResolutionLevel = ResolutionLevel.LOW
    factors: RiskFactors | None = None
    coordinate_frame: CoordinateFrame = CoordinateFrame.EGO
    source: DataSource = DataSource.LIVE_SENSOR

    @property
    def risk_percent(self) -> float:
        """Risk on a 0-100 scale, for dashboard display only."""
        return self.risk_score * 100.0


class ObjectRisk(TimestampedModel):
    """Risk attributed to a single tracked object."""

    track_id: int = Field(ge=0)
    risk_score: float = Field(ge=0.0, le=1.0)
    risk_level: RiskLevel = RiskLevel.LOW
    uncertainty: float = Field(default=0.0, ge=0.0, le=1.0)
    factors: RiskFactors | None = None
    source: DataSource = DataSource.LIVE_SENSOR


class RiskField(TimestampedModel):
    """A spatial risk field plus its per-object attribution.

    ``engine`` and ``is_baseline`` record which formulation produced the field
    so baseline and ADAPT-X results are never conflated.
    """

    engine: str = Field(min_length=1)
    is_baseline: bool = False
    cells: list[RiskCell] = Field(default_factory=list)
    object_risks: list[ObjectRisk] = Field(default_factory=list)
    coordinate_frame: CoordinateFrame = CoordinateFrame.EGO
    source: DataSource = DataSource.LIVE_SENSOR

    @property
    def max_risk(self) -> float:
        """Highest cell or object risk in the field; 0.0 when empty."""
        scores = [cell.risk_score for cell in self.cells]
        scores += [risk.risk_score for risk in self.object_risks]
        return max(scores, default=0.0)
