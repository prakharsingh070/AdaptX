// Dashboard home, laid out after docs/design/dashboard-target.png: pipeline
// strip, large viewport, status columns, mid-row map cards, bottom-row stage
// cards. Every panel is fed by backend data or says plainly that it is not.

import { el, card, kv, notice, clear, levelTag, notImplemented, table } from "../ui.js";
import { state, subscribe, currentScene, currentMode, selectTrack } from "../app.js";
import { fmt, fmtInt, fmtMs, fmtPercent, fmtTime, fmtSeconds, fmtMetres, NOT_MEASURED, NOT_AVAILABLE, NOT_IMPLEMENTED, isMissing } from "../data/format.js";
import { countClasses } from "../data/normalise.js";
import { tileFactorMap } from "../render/charts.js";
import { mountViewport } from "./viewport.js";

const STAGES = [
  ["lidar_preprocessing", "Processing"], ["perception", "Detection"], ["tracking", "Tracking"],
  ["prediction", "Prediction"], ["mapping", "Mapping"], ["risk", "Risk"], ["adaptive_resolution", "Adaptive resolution"],
  ["vehicle_control", "Control"], ["carla", "CARLA"], ["live_simulation", "Live session"], ["scenarios", "Scenarios"], ["evaluation", "Evaluation"],
];

const liveTag = () => ({ text: "LIVE", cls: "live" });
const sceneTag = (scene) => ({ text: scene ? scene.mode : currentMode(), cls: (scene ? scene.mode : currentMode()) === "live" ? "live" : "stored" });

function componentMap() {
  const map = new Map();
  for (const c of state.systemStatus?.components ?? []) map.set(c.name, c);
  return map;
}

function stageStrip() {
  const comps = componentMap();
  const strip = el("div", { class: "stage-strip" });
  STAGES.forEach(([name, label], i) => {
    const c = comps.get(name);
    const impl = c ? c.implementation : "PLANNED";
    strip.append(el("span", { class: `stage ${impl.toLowerCase()}`, title: c ? c.detail : "no status from the backend", text: `${label} · ${impl}` }));
    if (i < STAGES.length - 1) strip.append(el("span", { class: "arrow", text: "→" }));
  });
  return strip;
}

function systemStatusCard() {
  const s = state.systemStatus, m = state.telemetry.last?.data?.metrics ?? null;
  const scene = currentScene();
  const control = scene?.control ?? null;
  const timing = scene?.live?.timing ?? null;
  const pairs = [
    ["state", s ? s.state : null],
    ["risk level (scene, this frame)", scene ? levelTag(scene.highestRiskLevel) : null],
    ["risk level (in ego path)", control ? levelTag(control.governing_level) : null, { missing: "Not available (no live session)" }],
    ["ingest FPS (measured)", m ? fmt(m.fps, { digits: 1 }) : null, { missing: "Not available (needs two ingested frames)" }],
    ["pipeline latency (ms)", timing ? fmtMs(timing.pipeline_ms, 0) : (m ? fmtMs(m.latency_ms) : null)],
    ["loop period / step (ms)", timing ? `${fmtMs(timing.loop_ms, 0)} / ${fmtMs(timing.fixed_delta_s * 1000, 0)}` : null],
    ["sim / wall-clock speed", timing ? el("span", { class: timing.lagging ? "lvl lvl-high" : "", text: `${fmt(timing.realtime_factor, { digits: 2 })}×${timing.lagging ? " LAGGING" : ""}` }) : null],
    ["CPU (%)", m ? m.cpu_percent : null],
    ["GPU (%)", m ? m.gpu_percent : null, { missing: NOT_MEASURED }],
    ["memory (MB)", m ? fmt(m.memory_mb, { digits: 1 }) : null],
    ["uptime (s)", s ? fmt(s.uptime_s, { digits: 0 }) : null],
    ["telemetry", state.telemetry.state.connected ? (state.telemetry.state.stale ? `stale · last ${fmtTime(state.telemetry.state.lastMessageAt)}` : "live") : "disconnected"],
  ];
  const body = [kv(pairs)];
  if (m && m.unavailable?.length) body.push(el("div", { class: "note", text: m.unavailable.join(" · ") }));
  return card("System Status", body, { tag: liveTag() });
}

function detectedObjectsCard() {
  const scene = currentScene();
  if (!scene) return card("Detected Objects", notice("No frame yet."), { tag: { text: currentMode(), cls: currentMode() === "live" ? "live" : "stored" } });
  const classes = countClasses(scene.tracks);
  const pairs = ["vehicle", "pedestrian", "cyclist", "obstacle", "unknown"].map((k) => [k, classes[k] ?? 0]);
  pairs.push(["detections this frame", scene.detectionCount]);
  pairs.push(["live tracks", scene.tracks.length]);
  return card("Tracked Objects by class", [kv(pairs), el("div", { class: "note", text: "Classes are the geometric classifier's labels, not ground truth. Static scene geometry appears as obstacle/unknown tracks." })], { tag: { text: scene.mode, cls: scene.mode === "live" ? "live" : "stored" } });
}

function riskDistributionCard() {
  const scene = currentScene();
  if (!scene) return card("Risk distribution", notice("No frame yet."));
  const c = scene.riskCounts;
  const rows = [["low", c.low], ["medium", c.medium], ["high", c.high], ["critical", c.critical], ["UNKNOWN (unscored, not low)", c.unknown]];
  return card("Risk distribution (heuristic levels)", [
    kv(rows.map(([k, v]) => [k, v])),
    el("div", { class: "note", text: "Risk score is a heuristic engineering score, not a probability of collision. UNKNOWN means nothing could be scored." }),
  ]);
}

function recentEventsCard() {
  if (currentMode() === "playback") {
    return card("Recent Events", notice("Events are produced by the live session only. Playback shows a recorded run's outputs frame by frame."), { tag: { text: "playback", cls: "stored" } });
  }
  const events = state.liveEvents;
  if (!events.length) {
    return card("Recent Events", [
      notice(state.liveStatus ? `No events yet (session ${state.liveStatus.state}).` : "Live status unavailable."),
      el("div", { class: "note", text: "Events are what the live loop noticed in the pipeline's outputs: confirmed tracks, lost tracks, scene risk level changes, controller state, tile resolution changes, scenario actors, collisions." }),
    ], { tag: liveTag() });
  }
  const list = el("ul", { class: "event-list" });
  for (const e of events.slice(0, 14)) {
    list.append(el("li", {}, [
      el("span", { class: `dot ${e.kind}` }),
      el("span", { text: e.detail }),
      el("span", { class: "when", text: e.simulation_time_s !== null && e.simulation_time_s !== undefined ? `sim ${fmtSeconds(e.simulation_time_s, 2)}` : fmtTime(e.wall_time) }),
    ]));
  }
  return card("Recent Events", [list, el("div", { class: "note", text: `${events.length} kept on this page · from GET /api/v1/live/events · sim time is the simulator's clock` })], { tag: liveTag() });
}

function simulationInfoCard() {
  const c = state.carlaStatus, sim = c?.simulation ?? null, ls = state.liveStatus;
  const scene = currentScene();
  const live = scene?.live ?? null;
  return card("Simulation / Map Info", kv([
    ["CARLA", c ? `${c.status}${c.is_mock ? " (mock)" : ""}` : null],
    ["server version", sim ? sim.server_version : null],
    ["map", sim && sim.map_name ? String(sim.map_name).split("/").pop() : null],
    ["session", ls ? `${ls.state}${ls.scenario_name ? " · " + ls.scenario_name : ""}` : null],
    ["scenario / seed", ls && ls.scenario_id ? `${ls.scenario_id} / ${ls.seed}` : null],
    ["session time (s)", ls && ls.session_time_s !== null ? fmtSeconds(ls.session_time_s, 2) : null],
    ["sync / dt (s)", sim && sim.fixed_delta_seconds ? `${sim.synchronous_mode} / ${sim.fixed_delta_seconds}` : null],
    ["simulation frame / time", sim && sim.simulation_frame !== null ? `${sim.simulation_frame} / ${fmtSeconds(sim.simulation_time_s, 2)}` : null],
    ["ego speed (m/s)", ls && ls.ego_speed_mps !== null ? fmt(ls.ego_speed_mps, { digits: 2 }) : null],
    ["ego spawn index", sim ? sim.ego_spawn_index : null],
    ["scenario actors / traffic", live ? `${live.active_actor_count} / ${live.traffic_count}` : (ls ? `${ls.active_actors.length} / ${ls.traffic_count}` : null)],
    ["collisions (safety sensor)", ls ? ls.collision_count : null],
    ["weather / time of day", null, { missing: NOT_IMPLEMENTED }],
    ["detail", c ? c.detail : null],
  ]), { tag: liveTag() });
}

function adaptiveSummaryCard() {
  const scene = currentScene();
  const a = scene?.adaptiveMap ?? null;
  if (!a) return card("Adaptive 2.5D Map", notice("No adaptive map in the current frame."));
  const cmp = scene.comparison;
  return card("Adaptive 2.5D Map", [
    kv([
      ["tiles", `${a.tile_count} × ${a.tile_size_m} m`],
      ["tiles by level", Object.entries(a.tiles_by_level).map(([k, v]) => `${k} ${v}`).join(" · ")],
      ["cells (adaptive)", fmtInt(a.accounting.total_cell_count)],
      ["cells (fixed, same frame)", cmp ? fmtInt(cmp.fixed.total_cell_count) : null],
      ["finest / coarsest (m)", `${a.finest_resolution_m} / ${a.coarsest_resolution_m}`],
      ["area-weighted resolution (m)", fmt(a.area_weighted_resolution_m, { digits: 3 })],
      ["changed tiles", a.changed_tile_count],
      ["controller + mapping (ms)", `${fmtMs(a.controller_duration_ms)} + ${fmtMs(a.mapping_duration_ms)}`],
      ["grid bytes", fmtInt(a.grid_bytes)],
    ]),
    el("div", { class: "note", text: "Tile fills in the scene show each region's actual level; per-cell occupancy is not streamed live." }),
  ], { tag: { text: scene.mode, cls: scene.mode === "live" ? "live" : "stored" } });
}

function riskMapCard() {
  const scene = currentScene();
  const canvas = el("canvas", { class: "chart", style: "height:170px" });
  const body = [
    canvas,
    el("div", { class: "note", text: "Per-tile RISK FACTOR as the Phase 8 resolution controller recorded it (the risk engine's object-level scores, spread over the tiles they influence). Hatched = no factor for that tile. This is the controller's input, drawn as received — a spatial risk FIELD is not implemented." }),
  ];
  const c = card("Risk Map (per-tile risk factor)", body, { tag: scene ? sceneTag(scene) : { text: "N/A", cls: "na" } });
  requestAnimationFrame(() => tileFactorMap(canvas, scene ? scene.tiles : [], "risk", { title: "+X up · +Y left · ego at centre" }));
  return c;
}

function trackingPredictionCard() {
  const scene = currentScene();
  if (!scene) return card("Object Tracking & Prediction", notice("No frame yet."));
  // The nearest objects by the risk engine's own assessed distance (a sort of
  // a received field, not a computation); the inspector lists every track.
  const LIMIT = 12;
  const ordered = [...scene.objects].sort((a, b) => (a.assessment?.distance_m ?? Infinity) - (b.assessment?.distance_m ?? Infinity));
  const shown = ordered.slice(0, LIMIT);
  const rows = shown.map((o) => [
    { num: `#${o.trackId}` }, o.track.object_class, { num: o.assessment ? fmtMetres(o.assessment.distance_m, 1) : NOT_AVAILABLE },
    { num: o.track.velocity ? `${fmt(o.track.velocity.x, { digits: 1 })}, ${fmt(o.track.velocity.y, { digits: 1 })} m/s` : NOT_AVAILABLE },
    o.trajectory ? `${o.trajectory.points.length} pts / ${fmtSeconds(o.trajectory.horizon_s, 1)}` : "no path",
    o.assessment ? levelTag(o.assessment.risk_level) : el("span", { class: "muted", text: "—" }),
  ]);
  const selected = shown.findIndex((o) => o.trackId === state.selectedTrackId);
  return card("Object Tracking & Prediction", [
    rows.length ? table(["track", "class", "dist (assessed)", "velocity x, y", "predicted path", "risk"], rows, { onRow: (i) => selectTrack(shown[i].trackId), selectedIndex: selected }) : notice("No live tracks in this frame."),
    el("div", { class: "note", text: `${scene.objects.length > LIMIT ? `Nearest ${LIMIT} of ${scene.objects.length} tracks by assessed distance; the Object Inspector lists all. ` : ""}Confidence values are geometric fit scores; nothing here is a probability.` }),
  ], { tag: { text: scene.mode, cls: scene.mode === "live" ? "live" : "stored" } });
}

function stageCards() {
  const scene = currentScene();
  const ms = scene?.stageMs ?? {};
  const comps = componentMap();
  const impl = (name) => comps.get(name)?.implementation ?? "PLANNED";
  const lidar = card("LiDAR Processing", kv([
    ["raw points", scene?.points ? fmtInt(scene.points.total_count) : null],
    ["processing (ms)", fmtMs(ms.processing)],
    ["status", impl("lidar_preprocessing")],
  ]));
  const perception = card("Perception", kv([
    ["objects detected", scene ? scene.detectionCount : null],
    ["detection (ms)", fmtMs(ms.detection)],
    ["detection accuracy", null, { missing: "Not measured live — see Evaluation" }],
    ["status", impl("perception")],
  ]));
  const prediction = card("Prediction", kv([
    ["paths predicted", scene ? scene.trajectories.length : null],
    ["horizon (s)", scene?.trajectories[0] ? fmtSeconds(scene.trajectories[0].horizon_s, 1) : (state.telemetry.last?.data?.prediction?.horizon_s ?? null)],
    ["prediction (ms)", fmtMs(ms.prediction)],
    ["confidence", null, { missing: "heuristic, per path — see inspector" }],
  ]));
  const routing = card("Routing", [notImplemented("Routing", "no owning phase in the roadmap"), el("div", { class: "note", text: "Steering follows the map's lane centre (road geometry from CARLA waypoints); no route is planned." })], { tag: { text: "N/A", cls: "na" } });
  const decision = controlCard(scene);
  const planning = card("Planning", notImplemented("Planning", "no owning phase in the roadmap"), { tag: { text: "N/A", cls: "na" } });
  const performance = performanceCard(scene);
  return [lidar, perception, prediction, routing, decision, planning, performance];
}

function bar(label, value, cls = "", centred = false) {
  const v = value === null || value === undefined ? null : value;
  const track = el("div", { class: `bar ${cls}` });
  if (v !== null) {
    if (centred) {
      const left = v < 0 ? 50 + v * 50 : 50, width = Math.abs(v) * 50;
      track.append(el("span", { style: `left:${left}%;width:${Math.max(1, width)}%` }));
    } else track.append(el("span", { style: `width:${Math.max(0, Math.min(1, v)) * 100}%` }));
  }
  return [el("span", { text: label }), track, el("span", { class: "mono", text: v === null ? NOT_AVAILABLE : fmt(v, { digits: 2 }) })];
}

function controlCard(scene) {
  const control = scene?.control ?? null;
  const ls = state.liveStatus;
  if (!control) {
    return card("Decision / Control", [
      notice(currentMode() === "playback" ? "No controller in playback: recorded runs have a stationary ego." : "No live session: the ego controller runs only inside a live session."),
      el("div", { class: "note", text: "A risk-governed speed BASELINE (not an autonomous-driving controller): a target speed per in-path risk level, an emergency-brake distance, hold-then-resume. Reads the risk, tracking and prediction outputs and the ego's own odometry; never ground truth." }),
    ], { tag: { text: "N/A", cls: "na" } });
  }
  const bars = el("div", { class: "control-bars" });
  for (const cells of [bar("throttle", control.throttle), bar("brake", control.brake, "brake"), bar("steer (+left)", control.steer, "steer", true)]) bars.append(...cells);
  return card("Decision / Control (baseline)", [
    kv([
      ["controller state", el("span", { class: `lvl ${["EMERGENCY_BRAKING", "STOPPED", "HOLDING"].includes(control.state) ? "lvl-critical" : control.state === "SLOWING" ? "lvl-high" : "lvl-low"}`, text: control.state })],
      ["target / setpoint (m/s)", `${fmt(control.target_speed_mps, { digits: 1 })} / ${fmt(control.setpoint_mps, { digits: 1 })}`],
      ["ego speed (m/s)", ls && ls.ego_speed_mps !== null ? fmt(ls.ego_speed_mps, { digits: 2 }) : null],
      ["governing level (in path)", levelTag(control.governing_level)],
      ["nearest in-path object", control.nearest_in_path_m !== null ? `track #${control.governing_track_id} at ${fmtMetres(control.nearest_in_path_m, 1)}` : null, { missing: "none in the path" }],
      ["reason", control.reason],
    ]),
    bars,
    el("div", { class: "note", text: `Baseline: ${ls ? `max ${ls.control_configuration.max_speed_mps} m/s · safe ${ls.control_configuration.min_safe_distance_m} m · emergency ${ls.control_configuration.emergency_distance_m} m` : ""}. Untuned, unvalidated; collisions are counted by the safety sensor, not prevented by claim.` }),
  ], { tag: liveTag() });
}

function performanceCard(scene) {
  const timing = scene?.live?.timing ?? null;
  const ls = state.liveStatus;
  if (!timing) return card("Performance", [notice("Loop timing is measured by the live session only."), el("div", { class: "note", text: "Browser draw cost: Experiment 013. GPU: not measured." })], { tag: { text: "N/A", cls: "na" } });
  return card("Performance (measured)", [
    kv([
      ["loop period (ms)", `${fmtMs(timing.loop_ms, 0)} · median ${timing.loop_ms_median !== null ? fmtMs(timing.loop_ms_median, 0) : "—"} · max ${timing.loop_ms_max !== null ? fmtMs(timing.loop_ms_max, 0) : "—"}`],
      ["tick + LiDAR wait (ms)", fmtMs(timing.step_ms, 0)],
      ["pipeline (ms)", fmtMs(timing.pipeline_ms, 0)],
      ["control (ms)", fmtMs(timing.control_ms, 1)],
      ["snapshot (ms, prev. frame)", fmtMs(timing.snapshot_ms, 1)],
      ["sim / wall-clock speed", el("span", { class: timing.lagging ? "lvl lvl-high" : "lvl lvl-low", text: `${fmt(timing.realtime_factor, { digits: 2 })}×${timing.lagging ? " LAGGING" : ""}` })],
      ["frames processed / published", ls ? `${ls.frames_processed} / ${ls.snapshots_published}` : timing.frames_processed],
      ["frames skipped for display", fmtInt(state.skippedFrames)],
      ["GPU", null, { missing: NOT_MEASURED }],
    ]),
    el("div", { class: "note", text: "Wall-clock perf_counter differences inside the loop. Below 1.0× the simulation runs slower than real time; nothing here is a real-time claim." }),
  ], { tag: liveTag() });
}

function evaluationSummaryCard() {
  const r = state.reports.report;
  if (!r) return card("Latest evaluation", [notice(state.reports.error ? `Report could not be loaded: ${state.reports.error.message}` : "No evaluation report loaded. Choose one in the rail.")], { tag: { text: "STORED", cls: "stored" } });
  const g = r.tracking.gates?.find((x) => x.gate_m === r.configuration.primary_gate_m) ?? null;
  const d = r.detection.gates?.find((x) => x.gate_m === r.configuration.primary_gate_m) ?? null;
  return card(`Latest evaluation — ${r.scenario_id}`, [
    kv([
      ["status per section", Object.entries({ det: r.detection.status, trk: r.tracking.status, pred: r.prediction.status, risk: r.risk.status, map: r.mapping.status, adapt: r.adaptive_resolution.status }).map(([k, v]) => `${k}:${v}`).join(" ")],
      [`detection recall @ ${r.configuration.primary_gate_m} m`, d ? fmtPercent(d.match_rate) : null],
      [`tracking match rate @ ${r.configuration.primary_gate_m} m`, g ? fmtPercent(g.match_rate) : null],
      ["ADE (m)", r.prediction.ade_m.count ? fmt(r.prediction.ade_m.mean) : null, { missing: `Not available (${r.prediction.status})` }],
      ["adaptive / fixed cells", r.adaptive_resolution.cell_ratio.count ? fmt(r.adaptive_resolution.cell_ratio.mean, { digits: 3 }) : null],
      ["build-time ratio (mapping)", r.adaptive_resolution.mapping_duration_ratio.count ? fmt(r.adaptive_resolution.mapping_duration_ratio.median, { digits: 2 }) + "×" : null],
      ["mapping accuracy", null, { missing: "Not evaluated" }],
    ]),
    el("div", { class: "note", text: `Simulation evidence · seed ${r.seed} · ${r.simulator_version} · commit ${r.git_commit ? r.git_commit.slice(0, 10) : "unknown"} · ${r.frame_count} frames` }),
  ], { tag: { text: "STORED EVALUATION", cls: "stored" } });
}

export function renderOverview(root) {
  const strip = card("ADAPT-X pipeline", el("div"));
  const grid = el("div", { class: "grid-overview" });
  root.append(strip, grid);

  const viewportHost = el("div", { class: "span-2", style: "display:flex;flex-direction:column;min-height:560px" });
  const col1 = el("div", { style: "display:flex;flex-direction:column;gap:12px" });
  const col2 = el("div", { style: "display:flex;flex-direction:column;gap:12px" });
  const mid1 = el("div"), mid2 = el("div"), mid3 = el("div", { class: "span-2" });
  const bottom = el("div", { class: "span-4", style: "display:grid;grid-template-columns:repeat(auto-fit,minmax(230px,1fr));gap:12px" });
  const evalRow = el("div", { class: "span-4" });
  const disclaimer = el("div", { class: "span-4 notice", text: "Simulation/research visualisation. Risk is heuristic and not a collision probability. No safety guarantee is implied. Live panels show the last frame the backend processed; stored evaluation panels show a recorded report — never the same run unless labelled so." });
  grid.append(viewportHost, col1, col2, mid1, mid2, mid3, bottom, evalRow, disclaimer);
  const unmountViewport = mountViewport(viewportHost, { compact: true });

  function draw() {
    clear(strip.querySelector("div")).append(stageStrip());
    clear(col1).append(systemStatusCard(), detectedObjectsCard());
    clear(col2).append(recentEventsCard(), simulationInfoCard());
    clear(mid1).append(adaptiveSummaryCard());
    clear(mid2).append(riskMapCard(), riskDistributionCard());
    clear(mid3).append(trackingPredictionCard());
    clear(bottom).append(...stageCards());
    clear(evalRow).append(evaluationSummaryCard());
  }
  draw();
  // Redraw on data, not on every status poll while a live frame is streaming:
  // the scene topic arrives up to 20 times a second and carries everything.
  const unsubscribe = subscribe((topic) => { if (topic !== "telemetry" || !state.liveScene) draw(); });
  return () => { unsubscribe(); unmountViewport(); };
}

export { isMissing };
