"""The heuristic risk engine against proximity events (Question D).

No catalogue scenario contains a collision, so there is no collision label
and nothing here is a collision-prediction figure. What ground truth supports
objectively is **proximity**: an actor is inside the band when its planar
ground-truth distance is at most ``proximity_event_m``. The matched track is
**alerting** when its risk level is at or above ``alert_level``.

``UNKNOWN`` stays unknown. A matched frame whose assessment has no score is
counted in ``unknown_risk_frames`` and takes part in nothing else - it is
never read as LOW, never read as 0.0 (ADR-035).

Ordering concordance is the one metric that reads the score as a number: over
every pair of frames for one actor where both frames carry a score, the share
in which the higher score coincided with the smaller distance. It asks only
whether the score *orders* proximity, which is the least the heuristic
claims, and not whether it means anything absolute.
"""

from __future__ import annotations

from collections import Counter
from itertools import combinations

from adaptx.evaluation.dataset import EvaluationDataset
from adaptx.evaluation.matching import FrameMatching
from adaptx.evaluation.models import (
    ActorRiskMetrics,
    EvaluationConfiguration,
    MetricStatus,
    RiskEvaluation,
)
from adaptx.models.risk import RiskLevel
from adaptx.models.risk_assessment import RiskAssessment

#: Ordinal rank of each level, for "at or above".
_LEVEL_RANK: dict[RiskLevel, int] = {
    RiskLevel.UNKNOWN: -1,
    RiskLevel.LOW: 0,
    RiskLevel.MEDIUM: 1,
    RiskLevel.HIGH: 2,
    RiskLevel.CRITICAL: 3,
}


def is_alerting(assessment: RiskAssessment, alert_level: RiskLevel) -> bool:
    """Whether an assessment is at or above ``alert_level``. UNKNOWN never alerts."""
    if assessment.risk_level is RiskLevel.UNKNOWN:
        return False
    return _LEVEL_RANK[assessment.risk_level] >= _LEVEL_RANK[alert_level]


def ordering_concordance(samples: list[tuple[float, float]]) -> tuple[int, float | None]:
    """Share of (distance, score) pairs ordered inversely, and the pair count.

    Pairs with a tie in either quantity are dropped: they say nothing about
    order. Returns ``(0, None)`` below two usable pairs.
    """
    concordant = 0
    counted = 0
    for (d1, s1), (d2, s2) in combinations(samples, 2):
        if d1 == d2 or s1 == s2:
            continue
        counted += 1
        if (d1 < d2) == (s1 > s2):
            concordant += 1
    if counted == 0:
        return 0, None
    return counted, concordant / counted


def evaluate_risk(
    dataset: EvaluationDataset,
    configuration: EvaluationConfiguration,
    matching: dict[int, FrameMatching],
) -> RiskEvaluation:
    """Alert recall, lead time and ordering per actor; unlabelled alerts counted only."""
    band = configuration.proximity_event_m
    alert_level = configuration.alert_level

    names: dict[int, str] = {}
    event_frames: Counter[int] = Counter()
    matched_event_frames: Counter[int] = Counter()
    alert_in_event: Counter[int] = Counter()
    early_alerts: Counter[int] = Counter()
    unknown: Counter[int] = Counter()
    scored: Counter[int] = Counter()
    first_event: dict[int, float] = {}
    first_alert: dict[int, float] = {}
    samples: dict[int, list[tuple[float, float]]] = {}
    level_counts: dict[int, Counter[str]] = {}
    unlabelled_alerts = 0
    any_eligible = False

    for frame in dataset.frames:
        assessments = {a.track_id: a for a in frame.outputs.risk.assessments}
        frame_matching = matching[frame.index]
        matched_tracks = {m.candidate_id for m in frame_matching.matches}
        for track_id, assessment in assessments.items():
            if track_id not in matched_tracks and is_alerting(assessment, alert_level):
                unlabelled_alerts += 1

        for actor in frame.eligible_actors:
            any_eligible = True
            actor_id = actor.simulator_actor_id
            names[actor_id] = actor.actor_id
            in_band = actor.distance_m <= band
            if in_band:
                event_frames[actor_id] += 1
                first_event.setdefault(actor_id, frame.time_s)
            match = frame_matching.match_for_actor(actor_id)
            if match is None:
                continue
            matched_assessment = assessments.get(match.candidate_id)
            if matched_assessment is None:
                continue
            assessment = matched_assessment
            level_counts.setdefault(actor_id, Counter())[assessment.risk_level.value] += 1
            if assessment.risk_score is None:
                unknown[actor_id] += 1
            else:
                scored[actor_id] += 1
                samples.setdefault(actor_id, []).append((actor.distance_m, assessment.risk_score))
            alerting = is_alerting(assessment, alert_level)
            if alerting:
                first_alert.setdefault(actor_id, frame.time_s)
            if in_band:
                matched_event_frames[actor_id] += 1
                alert_in_event[actor_id] += int(alerting)
            elif alerting:
                early_alerts[actor_id] += 1

    if not any_eligible:
        return RiskEvaluation(
            status=MetricStatus.UNAVAILABLE,
            reason="no eligible ground-truth actor on any evaluated frame",
            proximity_event_m=band,
            alert_level=alert_level.value,
            unlabelled_alert_track_frames=unlabelled_alerts,
        )

    actors: list[ActorRiskMetrics] = []
    for actor_id in sorted(names):
        pairs, concordance = ordering_concordance(samples.get(actor_id, []))
        event_time = first_event.get(actor_id)
        alert_time = first_alert.get(actor_id)
        actors.append(
            ActorRiskMetrics(
                actor_id=names[actor_id],
                simulator_actor_id=actor_id,
                event_frames=event_frames[actor_id],
                matched_event_frames=matched_event_frames[actor_id],
                alert_frames_in_event=alert_in_event[actor_id],
                alert_recall=(
                    None
                    if matched_event_frames[actor_id] == 0
                    else alert_in_event[actor_id] / matched_event_frames[actor_id]
                ),
                first_event_time_s=event_time,
                first_alert_time_s=alert_time,
                lead_time_s=(
                    None if event_time is None or alert_time is None else event_time - alert_time
                ),
                early_alert_frames=early_alerts[actor_id],
                unknown_risk_frames=unknown[actor_id],
                scored_frames=scored[actor_id],
                ordering_pairs=pairs,
                ordering_concordance=concordance,
                risk_level_counts=dict(level_counts.get(actor_id, Counter())),
            )
        )

    status = MetricStatus.MEASURED
    reason: str | None = None
    if not any(event_frames.values()):
        status = MetricStatus.PARTIAL
        reason = f"no actor came within {band} m, so alert recall and lead time have no event"
    return RiskEvaluation(
        status=status,
        reason=reason,
        proximity_event_m=band,
        alert_level=alert_level.value,
        collision_labels_present=False,
        actors=actors,
        unlabelled_alert_track_frames=unlabelled_alerts,
    )


__all__ = ["evaluate_risk", "is_alerting", "ordering_concordance"]
