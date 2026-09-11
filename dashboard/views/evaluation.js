// Evaluation viewer: a stored Phase 11 EvaluationReport, section by section,
// with the status and reason of each, the gate every figure was computed at,
// and the report's own limitations. Negative results are on the same cards
// as everything else. Nothing here is recomputed.

import { el, card, kv, notice, clear, table } from "../ui.js";
import { state, subscribe } from "../app.js";
import { fmt, fmtInt, fmtMs, fmtPercent, fmtDistribution, fmtSeconds, fmtMetres, NOT_AVAILABLE, isMissing } from "../data/format.js";
import { lineChart, barChart } from "../render/charts.js";

function statusTag(section) {
  return { text: section.status, cls: section.status };
}

function reasonLine(section) {
  const parts = [];
  if (section.reason) parts.push(el("div", { class: "note", text: `Reason: ${section.reason}` }));
  for (const w of section.warnings ?? []) parts.push(el("div", { class: "note", text: `Warning: ${w}` }));
  return parts;
}

function dist(d, unit = "", digits = 2) {
  return fmtDistribution(d, unit, digits);
}

export function reportHeader(r) {
  return card(`Evaluation report — ${r.scenario_name} (${r.scenario_id})`, kv([
    ["seed", r.seed], ["run state", r.run_state], ["frames evaluated / recorded / planned", `${r.evaluated_frame_count} / ${r.frame_count} / ${r.planned_frame_count}`],
    ["CARLA version", r.simulator_version], ["map", r.map_name], ["timestep (s)", r.fixed_delta_seconds],
    ["ADAPT-X version / commit", `${r.adaptx_version} @ ${r.git_commit ?? "unknown"}`],
    ["evaluated at", r.timestamp], ["run recorded at", r.run_timestamp],
    ["match gates (m)", r.configuration.match_gates_m.join(", ")], ["PRIMARY GATE (m)", r.configuration.primary_gate_m],
    ["proximity event band (m)", r.configuration.proximity_event_m], ["alert level", r.configuration.alert_level], ["refined level", r.configuration.refined_level],
    ["thresholds are", r.configuration.is_baseline ? "baseline (untuned)" : "custom"],
    ["scenario actors", r.scenario_actors.join(", ")],
    ["sensor mount (x, y, z) m", r.sensor_mount ? `${r.sensor_mount.x}, ${r.sensor_mount.y}, ${r.sensor_mount.z}` : null],
    ["source", r.source],
  ]), { tag: { text: "STORED EVALUATION", cls: "stored" }, cls: "full" });
}

function gateTable(section, primary, withVelocity) {
  const headers = ["gate (m)", "eligible", "matched", withVelocity ? "match rate" : "recall", "class agreement", "planar error"];
  if (withVelocity) headers.push("velocity error", "velocity null");
  const rows = (section.gates ?? []).map((g) => {
    const row = [
      { num: `${g.gate_m}${g.gate_m === primary ? " ★" : ""}` }, { num: g.eligible_pairs }, { num: g.matched_pairs },
      { num: fmtPercent(g.match_rate) }, { num: fmtPercent(g.class_agreement_rate) }, dist(g.position_error_planar_m, " m"),
    ];
    if (withVelocity) row.push(dist(g.velocity_error_mps, " m/s"), { num: g.velocity_null_pairs });
    return row;
  });
  return rows.length ? table(headers, rows) : notice("No gate metrics (see status).");
}

function detectionCard(r) {
  const s = r.detection;
  return card("Detection", [...reasonLine(s), gateTable(s, r.configuration.primary_gate_m, false), el("div", { class: "note", text: "★ primary gate. Recall = eligible actor-frames with a detection inside the gate. No precision: unmatched detections are unlabelled static geometry, not false positives." })], { tag: statusTag(s), cls: "full" });
}

function trackingCard(r) {
  const s = r.tracking;
  const cont = (s.continuity ?? []).map((c) => [c.actor_id, { num: fmtPercent(c.coverage) }, `${c.frames_matched}/${c.frames_eligible}`, c.track_ids.join(" "), { num: c.id_switches }, { num: c.fragments }, { num: c.longest_fragment_frames }, JSON.stringify(c.matched_status_counts)]);
  return card("Tracking", [
    ...reasonLine(s),
    gateTable(s, r.configuration.primary_gate_m, true),
    el("div", { class: "chart-title", text: `Continuity at the primary gate (${s.continuity_gate_m ?? r.configuration.primary_gate_m} m)` }),
    cont.length ? table(["actor", "coverage", "matched/eligible", "track ids", "id switches", "fragments", "longest", "status counts"], cont) : notice("No continuity rows."),
    kv([["unlabelled track-frames (not false positives)", s.unlabelled_track_frames], ["track ids never matched", s.unlabelled_track_ids]]),
  ], { tag: statusTag(s), cls: "full" });
}

function predictionCard(r) {
  const p = r.prediction;
  const chart = el("canvas", { class: "chart" });
  const c = card("Trajectory prediction (ADE / FDE)", [
    ...reasonLine(p),
    kv([
      ["gate used (m)", p.gate_m], ["trajectories in record", p.trajectories_in_record], ["evaluated", p.trajectories_evaluated],
      ["skipped by the evaluator", Object.entries(p.skipped).map(([k, v]) => `${k}: ${v}`).join("; ") || "none"],
      ["skipped by the predictor", Object.entries(p.predictor_skips).map(([k, v]) => `${k}: ${v}`).join("; ") || "none"],
      ["ADE (m)", dist(p.ade_m, " m")], ["FDE (m)", dist(p.fde_m, " m")], ["horizon coverage", dist(p.horizon_coverage, "", 2)],
      ["ego stationary", p.ego_stationary === null ? null : String(p.ego_stationary)],
    ]),
    el("div", { class: "chart-title", text: "Planar error by horizon offset (metres; n per point in the table)" }),
    chart,
    table(["t+ (s)", "n", "mean (m)", "median (m)", "max (m)"], Object.entries(p.error_by_offset_m).map(([k, d]) => [{ num: k }, { num: d.count }, { num: fmt(d.mean) }, { num: fmt(d.median) }, { num: fmt(d.max) }])),
  ], { tag: statusTag(p), cls: "wide" });
  requestAnimationFrame(() => lineChart(chart, {
    title: "Mean planar error by offset", unit: "m", xLabel: "t+ s",
    series: [{ label: "mean error", colour: "#38bdf8", points: Object.entries(p.error_by_offset_m).map(([k, d]) => ({ x: Number(k), y: d.mean })) }],
    source: "report.prediction.error_by_offset_m",
  }));
  return c;
}

function riskCard(r) {
  const s = r.risk;
  const rows = (s.actors ?? []).map((a) => [a.actor_id, { num: a.event_frames }, { num: a.matched_event_frames }, { num: fmtPercent(a.alert_recall) }, { num: fmtSeconds(a.lead_time_s) }, { num: a.early_alert_frames }, { num: a.unknown_risk_frames }, { num: `${fmt(a.ordering_concordance, { digits: 3 })} / ${a.ordering_pairs}` }, JSON.stringify(a.risk_level_counts)]);
  return card("Risk against proximity events", [
    ...reasonLine(s),
    kv([["event: actor within (m)", s.proximity_event_m], ["alert: level ≥", s.alert_level], ["collision labels", s.collision_labels_present ? "present" : "none — no collision metric exists"], ["alerting unlabelled track-frames (count only)", s.unlabelled_alert_track_frames]]),
    rows.length ? table(["actor", "event frames", "matched", "alert recall", "lead time", "early alerts", "UNKNOWN frames", "ordering / pairs", "levels"], rows) : notice("No actor rows."),
    el("div", { class: "note", text: "Risk score is a heuristic engineering score, not a probability of collision. UNKNOWN frames are counted and never scored." }),
  ], { tag: statusTag(s), cls: "wide" });
}

function mappingCard(r) {
  const m = r.mapping;
  const workload = (w) => w ? kv([["cells", dist(w.total_cells, "", 0)], ["occupied cells", dist(w.occupied_cells, "", 0)], ["occupancy ratio", dist(w.occupancy_ratio, "", 3)], ["grid bytes", dist(w.grid_bytes, "", 0)], ["build (ms)", dist(w.duration_ms, " ms")]]) : notice(NOT_AVAILABLE);
  return card("Mapping (workload only)", [
    ...reasonLine(m),
    el("div", { class: "notice warn", text: `Occupancy accuracy: ${m.accuracy_status.toUpperCase()} — ${m.accuracy_reason}` }),
    kv([["bounds (m)", m.bounds_m ? `x ${m.bounds_m.min_x}…${m.bounds_m.max_x}, y ${m.bounds_m.min_y}…${m.bounds_m.max_y}` : null], ["fixed resolution (m)", m.fixed_resolution_m]]),
    el("div", { class: "chart-title", text: "Fixed map" }), workload(m.fixed),
    el("div", { class: "chart-title", text: "Adaptive map" }), workload(m.adaptive),
  ], { tag: statusTag(m), cls: "wide" });
}

export function adaptiveCard(r, { full = false } = {}) {
  const a = r.adaptive_resolution;
  const c = a.churn;
  const refinements = (a.refinements ?? []).map((x) => [x.actor_id, { num: x.tile_index }, { num: x.entered_frame }, { num: x.refined_frame ?? NOT_AVAILABLE }, { num: x.lead_frames === null || x.lead_frames === undefined ? "never" : x.lead_frames }]);
  return card("Adaptive resolution (paired with the fixed map, same frames)", [
    ...reasonLine(a),
    kv([
      ["tiles", a.tile_count === null ? null : `${a.tile_count} × ${a.tile_size_m} m`],
      ["level cell sizes (m)", Object.entries(a.level_cell_sizes_m).map(([k, v]) => `${k} ${v}`).join(" · ")],
      ["fixed cells", dist(a.fixed_cells, "", 0)], ["adaptive cells", dist(a.adaptive_cells, "", 0)],
      ["cell ratio adaptive/fixed", dist(a.cell_ratio, "", 3)], ["byte ratio", dist(a.byte_ratio, "", 3)],
      ["build-time ratio (mapping only)", dist(a.mapping_duration_ratio, "×", 2)], ["build-time ratio (controller + mapping)", dist(a.total_duration_ratio, "×", 2)],
      ["tile resolution (m, pooled)", dist(a.tile_resolution_m, " m", 3)], ["unique resolutions per frame", dist(a.unique_resolutions_per_frame, "", 1)],
      ["area-weighted resolution (m)", dist(a.area_weighted_resolution_m, " m", 3)],
      ["actor-tile levels", JSON.stringify(a.actor_tile_levels)], ["other-tile levels", JSON.stringify(a.other_tile_levels)],
      ["actor-tile resolution (m)", dist(a.actor_tile_resolution_m, " m", 3)], ["other-tile resolution (m)", dist(a.other_tile_resolution_m, " m", 3)],
      ...Object.entries(a.actor_tile_resolution_by_risk_m).map(([k, d]) => [`actor tile at risk ${k}`, dist(d, " m", 3)]),
      ["refined level", a.refined_level], ["refinement lead (frames; + = before arrival)", dist(a.refinement_lead_frames, "", 1)], ["entries never refined", a.entries_never_refined],
      ["frames over budget", a.frames_over_budget], ["demoted tiles", a.demoted_tiles_total],
    ]),
    el("div", { class: "chart-title", text: "Churn" }),
    c ? kv([["changed tiles per frame", dist(c.changed_tiles_per_frame, "", 2)], ["frames with any change", `${c.frames_with_any_change} / ${c.frames}`], ["transitions", c.total_transitions], ["tiles ever changed", c.tiles_ever_changed], ["reversals (within dwell window)", `${c.reversals} (window ${c.min_dwell_frames})`], ["hysteresis holds / dwell holds", `${c.hysteresis_holds} / ${c.dwell_holds}`]]) : notice(NOT_AVAILABLE),
    full ? el("div", { class: "chart-title", text: "Refinement entries" }) : null,
    full ? (refinements.length ? table(["actor", "tile", "entered frame", "refined frame", "lead"], refinements) : notice("No entries.")) : null,
  ], { tag: statusTag(a), cls: full ? "full" : "wide" });
}

function resourceCard(r) {
  const s = r.resource;
  const chart = el("canvas", { class: "chart" });
  const c = card("Resource (measured on the machine that ran the scenario; not a real-time claim)", [
    ...reasonLine(s),
    el("div", { class: "notice", text: `Peak memory: ${s.peak_memory_status} — ${s.peak_memory_reason}` }),
    kv([["pipeline per frame (ms)", dist(s.pipeline_ms, " ms", 1)], ["fixed grid bytes", dist(s.fixed_grid_bytes, "", 0)], ["adaptive grid bytes", dist(s.adaptive_grid_bytes, "", 0)]]),
    chart,
    table(["stage", "n", "median (ms)", "p95 (ms)", "max (ms)"], Object.entries(s.stage_ms).map(([k, d]) => [k, { num: d.count }, { num: fmt(d.median, { digits: 1 }) }, { num: isMissing(d.p95) ? NOT_AVAILABLE : fmt(d.p95, { digits: 1 }) }, { num: fmt(d.max, { digits: 1 }) }])),
  ], { tag: statusTag(s), cls: "wide" });
  requestAnimationFrame(() => barChart(chart, {
    title: "Median stage duration", unit: "ms", source: "report.resource.stage_ms",
    items: Object.entries(s.stage_ms).map(([k, d]) => ({ label: k, value: d.median, format: (v) => fmtMs(v) })),
  }));
  return c;
}

function limitationsCard(r) {
  return card("Limitations (from the report)", [
    el("ul", { class: "limits" }, r.limitations.map((l) => el("li", { text: l }))),
    ...(r.warnings ?? []).map((w) => el("div", { class: "note", text: `Warning: ${w}` })),
  ], { cls: "full" });
}

export function renderEvaluation(root) {
  const grid = el("div", { class: "grid-eval" });
  root.append(grid);
  function draw() {
    clear(grid);
    const r = state.reports.report;
    if (!r) {
      grid.append(el("div", { class: "full" }, [notice(state.reports.error ? `Evaluation report could not be loaded: ${state.reports.error.message}` : "No evaluation report loaded. Choose one in the rail (reports come from the backend's configured report directory).", state.reports.error ? "error" : "")]));
      return;
    }
    grid.append(reportHeader(r), detectionCard(r), trackingCard(r), predictionCard(r), riskCard(r), mappingCard(r), adaptiveCard(r), resourceCard(r), limitationsCard(r));
  }
  draw();
  return subscribe((topic) => { if (topic === "report") draw(); });
}

export { fmtInt, fmtMetres };
