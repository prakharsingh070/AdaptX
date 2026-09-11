// Run / Scenario: the selected recorded run's metadata, sensor and pipeline
// configuration, ground truth of the current playback frame (labelled), and
// the report's reproducibility fields.

import { el, card, kv, notice, clear, table, levelTag } from "../ui.js";
import { state, subscribe, currentScene, seek } from "../app.js";
import { fmt, fmtMetres, fmtVector, fmtSeconds } from "../data/format.js";

export function renderRun(root) {
  const grid = el("div", { class: "grid-eval" });
  root.append(grid);
  function draw() {
    clear(grid);
    const s = state.runs.summary;
    if (!s) {
      grid.append(el("div", { class: "full" }, [notice(state.runs.error ? `Run could not be loaded: ${state.runs.error.message}` : "No recorded run selected. Choose one in the rail (runs come from the backend's configured run directory).", state.runs.error ? "error" : "")]));
    } else {
      grid.append(card(`Run ${s.name}`, kv([
        ["scenario", `${s.scenario_name} (${s.scenario_id})`], ["seed", s.seed], ["state", s.state], ["map", s.map_name], ["simulator", s.simulator_version],
        ["timestep (s)", s.fixed_delta_seconds], ["frames recorded / planned", `${s.frame_count} / ${s.planned_frame_count}`],
        ["pipeline outputs recorded", String(s.has_outputs)], ["scenario actors", s.actors.join(", ")],
      ]), { tag: { text: "STORED", cls: "stored" }, cls: "wide" }));
      const sensor = s.sensor;
      grid.append(card("Sensor configuration (as configured for the run)", sensor ? kv([
        ["channels", sensor.channels], ["range (m)", sensor.range_m], ["points / s", sensor.points_per_second], ["rotation (Hz)", sensor.rotation_frequency_hz],
        ["FOV upper / lower (°)", `${sensor.upper_fov_deg} / ${sensor.lower_fov_deg}`], ["drop-off rate", sensor.dropoff_general_rate],
        ["mount (x, y, z) m", fmtVector(sensor.mount)], ["intensity recorded", String(sensor.include_intensity)],
      ]) : notice("Not available: this record predates the sensor field."), { cls: "wide" }));
      const scene = currentScene();
      if (scene && scene.mode === "playback") {
        const gt = scene.groundTruth;
        const others = gt ? gt.actors.filter((a) => !a.is_ego) : [];
        grid.append(card(`Ground truth — frame ${scene.frameId} (what the simulator knew; evaluation-only)`, [
          notice("GROUND TRUTH. Never shown in the live scene and never fed to perception (ADR-045). Positions are ego-relative in the ADAPT-X frame; the recorded velocity of a placed actor is not meaningful (ADR-052)."),
          others.length ? table(["actor id", "type", "class", "position (x, y, z) m", "distance (m)"], others.map((a) => [{ num: a.actor_id }, a.type_id, a.object_class, fmtVector(a.position), { num: fmtMetres(a.distance_m) }])) : notice("No non-ego actors in this frame."),
          scene.expectedPoses && scene.expectedPoses.length ? table(["scenario actor", "commanded position (x, y, z) m"], scene.expectedPoses.map((p) => [p.actor_id, fmtVector(p.position)])) : null,
        ], { tag: { text: "GROUND TRUTH · STORED", cls: "stored" }, cls: "full" }));
        if (scene.outputsMissing) grid.append(el("div", { class: "full" }, [notice("This frame carries stage counts only — no pipeline outputs were recorded, so there is nothing to draw for it.", "warn")]));
      }
      const frames = s.frame_ids.map((id, i) => [{ num: i }, { num: id }, { num: fmtSeconds(s.scenario_times_s[i]) }]);
      grid.append(card("Frames", el("div", { style: "max-height:260px;overflow:auto" }, [table(["index", "simulator frame", "t (s)"], frames, { onRow: (i) => seek(i), selectedIndex: state.playback.index })]), { cls: "full" }));
    }
    const r = state.reports.report;
    if (r) {
      grid.append(card("Loaded evaluation report — reproducibility", kv([
        ["scenario / seed", `${r.scenario_id} / ${r.seed}`], ["CARLA", `${r.simulator_version} on ${r.map_name}`], ["timestep (s)", r.fixed_delta_seconds],
        ["ADAPT-X", `${r.adaptx_version} @ ${r.git_commit ?? "unknown"}`], ["evaluated frames", r.evaluated_frame_count], ["evaluation id", r.evaluation_id],
        ["gates / primary (m)", `${r.configuration.match_gates_m.join(", ")} / ${r.configuration.primary_gate_m}`],
        ["run state", r.run_state],
      ]), { tag: { text: "STORED EVALUATION", cls: "stored" }, cls: "wide" }));
      grid.append(card("Pipeline configuration recorded on the run (first evaluated frame)", el("div", { style: "max-height:320px;overflow:auto" }, Object.entries(r.pipeline_configuration).map(([stage, cfg]) => el("div", {}, [el("div", { class: "chart-title", text: stage }), kv(Object.entries(cfg).map(([k, v]) => [k, typeof v === "object" ? JSON.stringify(v) : v]))]))), { cls: "wide" }));
    }
    if (!s && !r) grid.append(el("div", { class: "full" }, [notice("Select a run for playback, a report for evaluation, or both. They are labelled separately and are only the same run when their scenario, seed and commit agree.")]));
  }
  draw();
  return subscribe((topic) => { if (["run", "report", "scene"].includes(topic)) draw(); });
}

export { fmt, levelTag };
