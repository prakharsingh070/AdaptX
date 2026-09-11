// Object Inspector: every field of the selected track, its assessment and its
// predicted path, as received. Nulls read "Not available"; UNKNOWN stays
// UNKNOWN and is drawn distinctly; risk score and uncertainty are separate.

import { el, card, kv, notice, clear, levelTag, table } from "../ui.js";
import { state, subscribe, currentScene, selectTrack } from "../app.js";
import { fmt, fmtMetres, fmtSeconds, fmtVector, fmtPercent, NOT_AVAILABLE, isMissing, classGlyph, classLabel, pathLabel } from "../data/format.js";
import { findObject } from "../data/normalise.js";

export function inspectorPanel(object, scene) {
  if (!scene) return notice("No scene. The inspector shows the selected object of the live scene or of the playback frame.");
  if (!object) {
    const rows = scene.objects.map((o) => [
      { num: o.trackId }, o.track.object_class, o.track.status,
      { num: o.assessment ? fmtMetres(o.assessment.distance_m) : NOT_AVAILABLE },
      o.assessment ? levelTag(o.assessment.risk_level) : el("span", { class: "muted", text: "no assessment" }),
    ]);
    return el("div", {}, [
      notice(`${scene.objects.length} tracked object(s) in frame ${scene.frameId}. Click one in the scene or in this list.`),
      table(["track", "class", "status", "assessed dist.", "risk"], rows, { onRow: (i) => selectTrack(scene.objects[i].trackId) }),
    ]);
  }
  const t = object.track, a = object.assessment, tr = object.trajectory, r = object.record;
  const recordCard = r
    ? card(`${classGlyph(r.object_class)} ${classLabel(r.object_class)} #${r.track_id}`, [
      kv([
        ["class", classLabel(r.object_class)],
        ["distance (planar)", isMissing(r.distance_m) ? null : fmtMetres(r.distance_m, 1), { missing: "Not available (no risk assessment)" }],
        ["longitudinal (ahead +)", fmtMetres(r.longitudinal_distance_m, 1)],
        ["lateral (left +)", fmt(r.lateral_distance_m, { digits: 1, sign: true, unit: " m" })],
        ["speed (ego-relative)", isMissing(r.speed_mps) ? null : `${fmt(r.speed_mps, { digits: 1 })} m/s`, { missing: "Not available (velocity not yet measured)" }],
        ["relative speed (closing +)", isMissing(r.relative_speed_mps) ? null : `${fmt(r.relative_speed_mps, { digits: 1, sign: true })} m/s`],
        ["risk", levelTag(r.risk_level)],
        ["risk score", isMissing(r.risk_score) ? null : fmt(r.risk_score, { digits: 3 }), { missing: "Not available (UNKNOWN)" }],
        ["ego path", el("span", { class: r.in_ego_path ? "path in" : "path", text: pathLabel(r.path_relation) })],
        ["track state", `${r.tracking_state} · ${r.hits} hits · ${r.age_frames} frames`],
        ["prediction", r.predicted_points ? `${fmtSeconds(r.predicted_horizon_s, 1)} horizon · ${r.predicted_points} points` : null, { missing: "Not available (no predicted path)" }],
        ["confidence (geometric fit)", isMissing(r.confidence) ? null : fmtPercent(r.confidence, 0), { missing: NOT_AVAILABLE }],
      ]),
      el("div", { class: "note", text: "Backend-built record: every value is a Phase 4/5/7 output joined by track id. Sensor frame, +X forward, +Y left; velocities are ego-relative (no ego-motion compensation). Confidence is a geometric fit score, not a probability." }),
    ], { tag: { text: "LIVE", cls: "live" } })
    : null;
  const trackCard = card(`Track #${t.track_id}`, kv([
    ["class", t.object_class],
    ["status", t.status],
    ["position (x, y, z) m", fmtVector(t.position)],
    ["velocity (m/s)", t.velocity ? fmtVector(t.velocity) : null, { missing: "Not available (velocity not yet measured — null is not zero)" }],
    ["observed velocity", t.observed_velocity ? fmtVector(t.observed_velocity) : null],
    ["heading (rad)", t.heading_rad],
    ["confidence (geometric fit, not a probability)", fmt(t.confidence, { digits: 3 })],
    ["state uncertainty", fmt(t.uncertainty, { digits: 3 })],
    ["hits / age / missed", `${t.hits} / ${t.age_frames} / ${t.missed_frames}`],
    ["points", t.point_count],
    ["box (l × w × h) m", t.bounding_box ? `${fmt(t.bounding_box.dimensions.length)} × ${fmt(t.bounding_box.dimensions.width)} × ${fmt(t.bounding_box.dimensions.height)}` : null],
    ["first seen", t.first_seen], ["last seen", t.last_seen],
    ["source", t.source],
  ]), { tag: { text: scene.mode, cls: scene.mode === "live" ? "live" : "stored" } });

  let riskCard;
  if (!a) {
    riskCard = card("Risk", notice("No assessment for this track in this frame."));
  } else {
    const factors = a.factor_scores ?? {};
    riskCard = card("Risk (heuristic score — not a probability of collision)", [
      kv([
        ["level", levelTag(a.risk_level)],
        ["risk score", isMissing(a.risk_score) ? null : fmt(a.risk_score, { digits: 3 }), { missing: "Not available (UNKNOWN: nothing could be scored)" }],
        ["assessment status", a.status],
        ["distance (planar) m", fmtMetres(a.distance_m)],
        ["closing speed (m/s)", a.closing_speed_mps, { missing: NOT_AVAILABLE }],
        ["speed (m/s)", a.speed_mps],
        ["factors used", (a.factors ?? []).join(", ") || "none"],
        ["reason", a.reason],
      ]),
      el("div", { class: "chart-title", text: "Factor scores (null = factor could not be computed; it is dropped, never scored zero)" }),
      kv(Object.entries(factors).map(([k, v]) => [k, isMissing(v) ? null : fmt(v, { digits: 3 })])),
      el("div", { class: "chart-title", text: "Uncertainty — a separate quantity from risk" }),
      kv([
        ["uncertainty score", fmt(a.uncertainty.score, { digits: 3 })],
        ["reasons", (a.uncertainty.reasons ?? []).join(", ") || "none"],
        ["observation age (s)", fmtSeconds(a.uncertainty.observation_age_s)],
        ["stale", String(a.uncertainty.is_stale)],
        ["velocity known", String(a.uncertainty.velocity_known)],
        ["prediction available", String(a.uncertainty.prediction_available)],
        ["track confidence", fmt(a.uncertainty.track_confidence, { digits: 3 })],
      ]),
      el("div", { class: "chart-title", text: "Map context" }),
      kv([
        ["observation", a.map_context.observation],
        ["points in cell", a.map_context.point_count],
        ["max height (m)", a.map_context.max_height_m],
      ]),
      el("div", { class: "chart-title", text: "Predicted-path relevance" }),
      a.trajectory ? kv([
        ["closest predicted approach (m)", fmtMetres(a.trajectory.min_distance_m)],
        ["time to closest approach (s)", fmtSeconds(a.trajectory.time_to_min_distance_s)],
        ["uncertainty at closest (m)", fmtMetres(a.trajectory.uncertainty_at_min_m)],
        ["horizon (s)", fmtSeconds(a.trajectory.horizon_s)],
        ["approaching", String(a.trajectory.is_approaching)],
      ]) : el("div", { class: "muted small", text: "Not available: no predicted path for this track." }),
    ]);
  }

  let pathCard;
  if (!tr) {
    pathCard = card("Predicted path", notice("Not available: the predictor produced no trajectory for this track in this frame (a track without a measured velocity is skipped, not assumed stationary)."));
  } else {
    pathCard = card("Predicted path (constant-velocity baseline)", [
      kv([
        ["status", tr.status], ["predictor", tr.predictor_name], ["horizon (s)", fmtSeconds(tr.horizon_s)],
        ["interval (s)", fmtSeconds(tr.timestep_s)], ["points", tr.points.length],
        ["trajectory confidence (heuristic)", fmt(tr.confidence, { digits: 3 })],
        ["observation age (s)", fmtSeconds(tr.observation_age_s)],
      ]),
      table(["t+ (s)", "x", "y", "uncertainty (m, heuristic)"], tr.points.map((p) => [
        { num: fmtSeconds(p.time_offset_s) }, { num: fmt(p.position.x) }, { num: fmt(p.position.y) }, { num: fmtMetres(p.position_uncertainty_m) },
      ])),
    ]);
  }
  return el("div", {}, [recordCard, trackCard, riskCard, pathCard]);
}

export function renderInspector(root) {
  const box = el("div", { class: "grid-eval" });
  root.append(box);
  function draw() {
    clear(box);
    const scene = currentScene();
    const object = findObject(scene, state.selectedTrackId);
    const panel = el("div", { class: "full" }, [inspectorPanel(object, scene)]);
    box.append(panel);
    if (object) box.append(el("div", { class: "full" }, [el("button", { class: "playback", text: "clear selection", onclick: () => selectTrack(null) })]));
  }
  draw();
  return subscribe((topic) => { if (["scene", "selection", "run"].includes(topic)) draw(); });
}

export { fmtPercent };
