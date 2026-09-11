"""Human-readable rendering of an evaluation report.

Text, not conclusions: every line prints a measured value or the reason one
is absent. The closing sections are fixed. A reader who stops at the
conclusion still sees the limitations, because they are part of it.
"""

from __future__ import annotations

from adaptx.evaluation.models import (
    Distribution,
    EvaluationReport,
    MetricStatus,
    Section,
)

_WIDTH = 60


def _fmt(value: float | None, unit: str = "", digits: int = 3) -> str:
    if value is None:
        return "n/a"
    return f"{value:.{digits}f}{unit}"


def _dist(d: Distribution, unit: str = "", digits: int = 3) -> str:
    if d.count == 0:
        return "n/a (no samples)"
    p95 = _fmt(d.p95, unit, digits) if d.p95 is not None else "n/a"
    return (
        f"n={d.count} mean {_fmt(d.mean, unit, digits)} median {_fmt(d.median, unit, digits)} "
        f"p95 {p95} max {_fmt(d.max, unit, digits)}"
    )


def _header(title: str) -> list[str]:
    return ["", title, "-" * len(title)]


def _status_line(section: Section) -> list[str]:
    lines = [f"Status: {section.status.value}"]
    if section.reason:
        lines.append(f"Reason: {section.reason}")
    lines.extend(f"Warning: {warning}" for warning in section.warnings)
    return lines


def render(report: EvaluationReport) -> str:
    """The text report."""
    lines: list[str] = [
        "=" * _WIDTH,
        "ADAPT-X PHASE 11 EVALUATION",
        "=" * _WIDTH,
        f"Scenario:        {report.scenario_name} ({report.scenario_id})",
        f"Seed:            {report.seed}",
        f"CARLA version:   {report.simulator_version}",
        f"Map:             {report.map_name or 'unknown'}",
        f"ADAPT-X:         {report.adaptx_version} @ {report.git_commit or 'commit unknown'}",
        f"Run state:       {report.run_state.value}",
        f"Timestep:        {report.fixed_delta_seconds} s",
        f"Frames:          {report.frame_count} of {report.planned_frame_count} planned",
        f"Evaluated frames:{report.evaluated_frame_count:>5}",
        f"Scenario actors: {', '.join(report.scenario_actors) or 'none'}",
        f"Sensor mount:    {_vec(report)}",
        f"Gates:           {report.configuration.match_gates_m} m "
        f"(primary {report.configuration.primary_gate_m} m)",
        "",
        "SIMULATION EVIDENCE. Every figure below is computed from a CARLA run.",
    ]
    for warning in report.warnings:
        lines.append(f"Warning: {warning}")

    # -- detection ---------------------------------------------------------
    lines += _header("DETECTION")
    lines += _status_line(report.detection)
    for gate in report.detection.gates:
        lines.append(
            f"gate {gate.gate_m:.1f} m: recall {_fmt(gate.match_rate)} "
            f"({gate.matched_pairs}/{gate.eligible_pairs}) | class agreement "
            f"{_fmt(gate.class_agreement_rate)} | planar error "
            f"{_dist(gate.position_error_planar_m, ' m')}"
        )

    # -- tracking ----------------------------------------------------------
    lines += _header("TRACKING")
    lines += _status_line(report.tracking)
    for tgate in report.tracking.gates:
        lines.append(
            f"gate {tgate.gate_m:.1f} m: match rate {_fmt(tgate.match_rate)} "
            f"({tgate.matched_pairs}/{tgate.eligible_pairs}) | class agreement "
            f"{_fmt(tgate.class_agreement_rate)}"
        )
        lines.append(f"    position error planar: {_dist(tgate.position_error_planar_m, ' m')}")
        lines.append(f"    position error 3-D:    {_dist(tgate.position_error_3d_m, ' m')}")
        lines.append(
            f"    velocity error:        {_dist(tgate.velocity_error_mps, ' m/s')} | "
            f"velocity null {tgate.velocity_null_pairs} | no reference "
            f"{tgate.velocity_reference_missing_pairs}"
        )
    for actor in report.tracking.continuity:
        lines.append(
            f"continuity {actor.actor_id}: coverage {_fmt(actor.coverage)} "
            f"({actor.frames_matched}/{actor.frames_eligible}) | track ids {actor.track_ids} | "
            f"id switches {actor.id_switches} | fragments {actor.fragments} "
            f"(longest {actor.longest_fragment_frames}) | status {actor.matched_status_counts}"
        )
    lines.append(
        f"unlabelled track-frames: {report.tracking.unlabelled_track_frames} "
        f"({report.tracking.unlabelled_track_ids} ids never matched) - not false positives"
    )

    # -- prediction --------------------------------------------------------
    p = report.prediction
    lines += _header("TRAJECTORY")
    lines += _status_line(p)
    lines.append(
        f"trajectories in record {p.trajectories_in_record} | evaluated "
        f"{p.trajectories_evaluated} | skipped {p.skipped or '{}'} | predictor skipped "
        f"{p.predictor_skips or '{}'}"
    )
    lines.append(f"ADE: {_dist(p.ade_m, ' m')}")
    lines.append(f"FDE: {_dist(p.fde_m, ' m')}")
    lines.append(f"horizon coverage: {_dist(p.horizon_coverage)}")
    for offset, dist in p.error_by_offset_m.items():
        lines.append(f"    t+{offset} s: {_dist(dist, ' m')}")
    lines.append(f"ego stationary: {p.ego_stationary}")

    # -- risk --------------------------------------------------------------
    r = report.risk
    lines += _header("RISK")
    lines += _status_line(r)
    lines.append(
        f"proximity event: actor within {r.proximity_event_m} m | alert: level >= "
        f"{r.alert_level} | collision labels: {'present' if r.collision_labels_present else 'none'}"
    )
    for ra in r.actors:
        lines.append(
            f"{ra.actor_id}: event frames {ra.event_frames} (matched {ra.matched_event_frames}) | "
            f"alert recall {_fmt(ra.alert_recall)} | lead time {_fmt(ra.lead_time_s, ' s', 2)} "
            f"(event {_fmt(ra.first_event_time_s, ' s', 2)}, alert "
            f"{_fmt(ra.first_alert_time_s, ' s', 2)}) | early alerts {ra.early_alert_frames} | "
            f"UNKNOWN {ra.unknown_risk_frames} | ordering {_fmt(ra.ordering_concordance)} "
            f"over {ra.ordering_pairs} pairs | levels {ra.risk_level_counts}"
        )
    lines.append(
        f"alerting unlabelled track-frames: {r.unlabelled_alert_track_frames} (count only)"
    )

    # -- mapping -----------------------------------------------------------
    m = report.mapping
    lines += _header("MAPPING")
    lines += _status_line(m)
    lines.append(f"reference available: no | map accuracy: {m.accuracy_status.value}")
    lines.append(f"    {m.accuracy_reason}")
    for workload in (m.fixed, m.adaptive):
        if workload is None:
            continue
        lines.append(
            f"{workload.variant}: cells {_dist(workload.total_cells, '', 0)} | occupancy "
            f"{_dist(workload.occupancy_ratio)} | bytes {_dist(workload.grid_bytes, '', 0)} | "
            f"build {_dist(workload.duration_ms, ' ms', 2)}"
        )

    # -- adaptive ----------------------------------------------------------
    a = report.adaptive_resolution
    lines += _header("ADAPTIVE RESOLUTION")
    lines += _status_line(a)
    if a.status is not MetricStatus.UNAVAILABLE:
        lines.append(f"tiles {a.tile_count} of {a.tile_size_m} m | levels {a.level_cell_sizes_m}")
        lines.append(f"fixed cells:    {_dist(a.fixed_cells, '', 0)}")
        lines.append(f"adaptive cells: {_dist(a.adaptive_cells, '', 0)}")
        lines.append(f"cell ratio adaptive/fixed: {_dist(a.cell_ratio)}")
        lines.append(f"byte ratio adaptive/fixed: {_dist(a.byte_ratio)}")
        lines.append(f"build time ratio (mapping only): {_dist(a.mapping_duration_ratio)}")
        lines.append(f"build time ratio (controller + mapping): {_dist(a.total_duration_ratio)}")
        lines.append(f"tile resolution: {_dist(a.tile_resolution_m, ' m')}")
        lines.append(
            f"unique resolutions per frame: {_dist(a.unique_resolutions_per_frame, '', 1)}"
        )
        lines.append(f"area-weighted resolution: {_dist(a.area_weighted_resolution_m, ' m')}")
        lines.append(
            f"actor tile levels: {a.actor_tile_levels} | other tiles {a.other_tile_levels}"
        )
        lines.append(f"actor tile resolution: {_dist(a.actor_tile_resolution_m, ' m')}")
        lines.append(f"other tile resolution: {_dist(a.other_tile_resolution_m, ' m')}")
        for group, dist in a.actor_tile_resolution_by_risk_m.items():
            lines.append(f"    actor tile at risk {group}: {_dist(dist, ' m')}")
        lines.append(
            f"refinement lead (frames, + = before arrival): "
            f"{_dist(a.refinement_lead_frames, '', 1)} | never refined {a.entries_never_refined} "
            f"of {len(a.refinements)} entries"
        )
        if a.churn is not None:
            c = a.churn
            lines.append(
                f"churn: changed tiles/frame {_dist(c.changed_tiles_per_frame, '', 2)} | frames "
                f"with change {c.frames_with_any_change}/{c.frames} | transitions "
                f"{c.total_transitions} over {c.tiles_ever_changed} tiles | reversals "
                f"{c.reversals} (window {c.min_dwell_frames}) | holds hysteresis "
                f"{c.hysteresis_holds} dwell {c.dwell_holds}"
            )
        lines.append(
            f"frames over budget {a.frames_over_budget} | demoted tiles {a.demoted_tiles_total}"
        )

    # -- resource ----------------------------------------------------------
    res = report.resource
    lines += _header("RESOURCE")
    lines += _status_line(res)
    lines.append(f"peak memory: {res.peak_memory_status.value} - {res.peak_memory_reason}")
    lines.append(f"pipeline per frame: {_dist(res.pipeline_ms, ' ms', 2)}")
    for stage, dist in res.stage_ms.items():
        lines.append(f"    {stage:<17}{_dist(dist, ' ms', 2)}")
    lines.append(f"fixed grid bytes:    {_dist(res.fixed_grid_bytes, '', 0)}")
    lines.append(f"adaptive grid bytes: {_dist(res.adaptive_grid_bytes, '', 0)}")

    # -- conclusion --------------------------------------------------------
    lines += _header("CONCLUSION")
    lines.append("Measured findings only. See the sections above; nothing is summarised")
    lines.append("into a verdict, because a verdict would need thresholds this phase")
    lines.append("does not have the evidence to set.")

    lines += _header("LIMITATIONS")
    lines.extend(f"- {limitation}" for limitation in report.limitations)
    lines.append("=" * _WIDTH)
    return "\n".join(lines)


def _vec(report: EvaluationReport) -> str:
    mount = report.sensor_mount
    if mount is None:
        return "unknown"
    return f"({mount.x}, {mount.y}, {mount.z}) m in the ego frame"


__all__ = ["render"]
