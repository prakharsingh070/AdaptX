// Application shell: state, channels, navigation, header pills. Views live
// in views/ and render from the shared state; they never fetch on their own
// except the evidence views, which go through data/evidence.js.

import { api } from "./data/api.js";
import { Channel } from "./data/channel.js";
import { ReportStore, RunStore } from "./data/evidence.js";
import { fmt, fmtMs, fmtTime, fmtSeconds } from "./data/format.js";
import { sceneFromSnapshot, sceneFromRunFrame } from "./data/normalise.js";
import { clear } from "./ui.js";
import { renderOverview } from "./views/overview.js";
import { renderLive } from "./views/live.js";
import { renderInspector } from "./views/inspector.js";
import { renderMap } from "./views/map.js";
import { renderAdaptive } from "./views/adaptive.js";
import { renderEvaluation } from "./views/evaluation.js";
import { renderComparison } from "./views/comparison.js";
import { renderRun } from "./views/run.js";

const VIEWS = {
  overview: renderOverview,
  live: renderLive,
  inspector: renderInspector,
  map: renderMap,
  adaptive: renderAdaptive,
  evaluation: renderEvaluation,
  comparison: renderComparison,
  run: renderRun,
};

export const state = {
  view: "overview",
  // Which data the views draw. "live": the scene channel and the live session.
  // "stored": a recorded run's playback and stored reports. Never mixed: the
  // stored mode is chosen explicitly and every panel is labelled.
  uiMode: "live",
  backend: { ok: false, error: null, version: null, name: null },
  systemStatus: null,
  carlaStatus: null,
  mapStatus: null,
  riskStatus: null,
  telemetry: { last: null, channel: null, state: { connected: false, stale: true, lastMessageAt: null } },
  sceneChannel: null,
  sceneState: { connected: false, stale: true, lastMessageAt: null, lastSequence: null },
  liveSnapshot: null,
  liveScene: null,
  receipts: [],
  skippedFrames: 0,
  liveStatus: null,
  liveError: null,
  liveScenarios: [],
  liveEvents: [],
  liveEventSequence: 0,
  liveBusy: false,
  selectedTrackId: null,
  toggles: { points: true, tiles: true, trajectories: true, velocity: true, labels: true, grid: true },
  reports: new ReportStore(),
  runs: new RunStore(),
  playback: { index: 0, playing: false, timer: null, frame: null, scene: null, error: null },
  listeners: new Set(),
};

export function subscribe(listener) {
  state.listeners.add(listener);
  return () => state.listeners.delete(listener);
}

// ---- per-viewer memory ----------------------------------------------------
// What a refresh keeps: the chosen report and run, and the overlay toggles.
// The view itself lives in the URL hash. Nothing here is data - only choices -
// and storage may be unavailable, so every access is guarded.
const MEMORY_KEY = "adaptx.console.choices";

function remember() {
  try {
    const choices = { report: state.reports.name, run: state.runs.name, toggles: state.toggles, uiMode: state.uiMode };
    window.sessionStorage.setItem(MEMORY_KEY, JSON.stringify(choices));
  } catch { /* storage unavailable: choices are simply not kept */ }
}

function recall() {
  try {
    const raw = window.sessionStorage.getItem(MEMORY_KEY);
    const choices = raw ? JSON.parse(raw) : {};
    choices.scenario = window.sessionStorage.getItem(MEMORY_KEY + ".scenario");
    return choices;
  } catch { return null; }
}

export function emit(topic) {
  for (const listener of state.listeners) listener(topic);
}

/** The scene the views draw: the playback frame in stored mode, otherwise the live scene. */
export function currentScene() {
  return currentMode() === "playback" ? state.playback.scene : state.liveScene;
}

/** "playback" only when STORED EVALUATION is selected AND a run is loaded; otherwise "live". */
export function currentMode() {
  return state.uiMode === "stored" && state.runs.summary ? "playback" : "live";
}

export function setUiMode(mode) {
  state.uiMode = mode === "stored" ? "stored" : "live";
  if (state.uiMode === "live") stopPlayback();
  document.getElementById("mode-live").checked = state.uiMode === "live";
  document.getElementById("mode-stored").checked = state.uiMode === "stored";
  document.getElementById("live-controls").classList.toggle("hidden", state.uiMode !== "live");
  remember();
  emit("mode");
  emit("scene");
  updatePills();
}

export function selectTrack(trackId) {
  state.selectedTrackId = trackId;
  emit("selection");
}

// ---- backend polling ----------------------------------------------------
async function pollStatus() {
  try {
    const [health, system, carla] = await Promise.all([api.health(), api.systemStatus(), api.carlaStatus()]);
    state.backend = { ok: true, error: null, version: health.version ?? system.version ?? null, name: health.name ?? null };
    state.systemStatus = system;
    state.carlaStatus = carla;
  } catch (error) {
    state.backend = { ok: false, error: error.message, version: state.backend.version, name: state.backend.name };
  }
  emit("status");
}

async function loadStatic() {
  try { state.mapStatus = await api.mapStatus(); } catch { state.mapStatus = null; }
  try { state.riskStatus = await api.riskStatus(); } catch { state.riskStatus = null; }
  try {
    state.liveScenarios = await api.liveScenarios();
    const select = document.getElementById("select-scenario");
    clear(select);
    for (const sc of state.liveScenarios) select.append(new Option(`${sc.name} (${sc.scenario_id})`, sc.scenario_id));
    const remembered = recall()?.scenario;
    select.value = state.liveScenarios.some((sc) => sc.scenario_id === remembered) ? remembered : (state.liveScenarios.find((sc) => sc.scenario_id === "mixed_obstacles") ?? state.liveScenarios[0])?.scenario_id ?? "";
    onScenarioChosen();
  } catch { state.liveScenarios = []; }
  emit("status");
}

// ---- live session ----------------------------------------------------------
async function pollLive() {
  try {
    const status = await api.liveStatus();
    state.liveStatus = status;
    state.liveError = null;
    if (status.event_sequence > state.liveEventSequence) {
      const batch = await api.liveEvents(state.liveEventSequence);
      const known = new Set(state.liveEvents.map((e) => e.sequence));
      for (const event of batch.events) {
        if (known.has(event.sequence)) continue;
        state.liveEvents.unshift(event);
        state.liveEventSequence = Math.max(state.liveEventSequence, event.sequence);
      }
      state.liveEvents.sort((a, b) => b.sequence - a.sequence);
      state.liveEvents.length = Math.min(state.liveEvents.length, 200);
    }
  } catch (error) {
    state.liveStatus = null;
    state.liveError = error.message;
  }
  updateLiveControls();
  emit("live");
}

function onScenarioChosen() {
  const id = document.getElementById("select-scenario").value;
  const chosen = state.liveScenarios.find((sc) => sc.scenario_id === id);
  document.getElementById("live-scenario-note").textContent = chosen ? chosen.description : "";
  if (chosen) document.getElementById("input-seed").value = String(chosen.default_seed);
  try { window.sessionStorage.setItem(MEMORY_KEY + ".scenario", id); } catch { /* not kept */ }
}

async function liveAction(action) {
  if (state.liveBusy) return;
  state.liveBusy = true;
  updateLiveControls();
  const note = document.getElementById("live-status-note");
  try {
    if (action === "start" || action === "reset") { state.liveEvents = []; state.liveEventSequence = 0; state.skippedFrames = 0; }
    if (action === "start") {
      const scenario = document.getElementById("select-scenario").value || null;
      const seedText = document.getElementById("input-seed").value;
      const seed = seedText === "" ? null : Number(seedText);
      note.textContent = "starting: opening the CARLA session…";
      state.liveStatus = await api.liveStart(scenario, seed);
    } else if (action === "pause") state.liveStatus = await api.livePause();
    else if (action === "resume") state.liveStatus = await api.liveResume();
    else if (action === "stop") state.liveStatus = await api.liveStop();
    else if (action === "reset") state.liveStatus = await api.liveReset();
    state.liveError = null;
  } catch (error) {
    state.liveError = error.message;
  } finally {
    state.liveBusy = false;
  }
  updateLiveControls();
  emit("live");
  pollStatus();
}

function updateLiveControls() {
  const status = state.liveStatus;
  const st = status ? status.state : null;
  const controls = status ? status.controls_enabled : false;
  const running = st === "RUNNING", paused = st === "PAUSED";
  const active = running || paused || st === "STARTING";
  const set = (id, enabled) => { document.getElementById(id).disabled = !enabled || state.liveBusy; };
  set("btn-start", controls && !active);
  set("btn-pause", controls && running);
  set("btn-resume", controls && paused);
  set("btn-stop", controls && active);
  set("btn-reset", controls && (active || st === "STOPPED" || st === "COLLIDED" || st === "ERROR") && status.scenario_id);
  const note = document.getElementById("live-status-note");
  if (state.liveError) note.textContent = `error: ${state.liveError}`;
  else if (!status) note.textContent = "live status unavailable";
  else if (!controls) note.textContent = `${st}: controls disabled on the backend (ADAPTX_LIVE__CONTROLS_ENABLED=false)`;
  else note.textContent = `${st}${status.scenario_id ? ` · ${status.scenario_id} · seed ${status.seed}` : ""} — ${status.detail}`;
}

async function loadEvidenceLists() {
  const reportSelect = document.getElementById("select-report");
  const runSelect = document.getElementById("select-run");
  const note = document.getElementById("rail-evidence-note");
  try {
    const [reports, runs] = await Promise.all([api.reports(), api.runs()]);
    fillSelect(reportSelect, reports.files.map((f) => f.name), "— none loaded —", state.reports.name);
    fillSelect(runSelect, runs.files.map((f) => f.name), "— live —", state.runs.name);
    note.textContent = `${reports.files.length} report(s) in ${reports.directory}; ${runs.files.length} run(s) in ${runs.directory}`;
    restoreChoices(reports.files.map((f) => f.name), runs.files.map((f) => f.name));
  } catch (error) {
    note.textContent = `evidence directories unavailable: ${error.message}`;
  }
}

let restored = false;
async function restoreChoices(reportNames, runNames) {
  if (restored) return;
  restored = true;
  const choices = recall();
  if (!choices) return;
  if (choices.report && reportNames.includes(choices.report) && !state.reports.name) {
    document.getElementById("select-report").value = choices.report;
    await selectReport(choices.report);
  }
  if (choices.run && runNames.includes(choices.run) && !state.runs.name) {
    document.getElementById("select-run").value = choices.run;
    await selectRun(choices.run);
  }
}

function fillSelect(select, names, emptyLabel, current) {
  clear(select);
  select.append(new Option(emptyLabel, ""));
  for (const name of names) select.append(new Option(name, name));
  select.value = current ?? "";
}

// ---- channels -----------------------------------------------------------
function startChannels() {
  state.telemetry.channel = new Channel("/ws/telemetry", {
    staleAfterMs: 4000,
    onMessage: (envelope) => {
      if (envelope.type === "telemetry") state.telemetry.last = envelope;
      emit("telemetry");
    },
    onState: (s) => { state.telemetry.state = s; updatePills(); },
  });
  state.telemetry.channel.open();

  state.sceneChannel = new Channel("/ws/scene", {
    staleAfterMs: 3000,
    onMessage: (envelope) => {
      if (envelope.type === "scene") {
        state.liveSnapshot = envelope.data;
        state.liveScene = sceneFromSnapshot(envelope.data);
        state.receipts.unshift({
          at: Date.now(),
          frameId: envelope.data.frame_id,
          origin: envelope.data.origin,
          tracks: envelope.data.tracks.length,
          level: envelope.data.highest_risk_level,
          sequence: envelope.sequence,
        });
        state.receipts.length = Math.min(state.receipts.length, 12);
        // Latest-only delivery: the backend tells us how many frames were
        // superseded between two pushes. Counted, never hidden.
        state.skippedFrames += envelope.skipped ?? 0;
        emit("scene");
      } else if (envelope.type === "cleared") {
        state.liveSnapshot = null;
        state.liveScene = null;
        emit("scene");
      }
    },
    onState: (s) => { state.sceneState = s; updatePills(); emit("channel"); },
  });
  state.sceneChannel.open();
}

// ---- playback -----------------------------------------------------------
export async function selectRun(name) {
  stopPlayback();
  state.playback = { index: 0, playing: false, timer: null, frame: null, scene: null, error: null };
  if (!name) {
    state.runs.clear();
    remember();
    emit("run");
    updatePills();
    return;
  }
  await state.runs.load(name);
  remember();
  if (state.runs.error) {
    state.playback.error = state.runs.error.message;
  } else {
    await seek(0);
  }
  emit("run");
  updatePills();
}

export async function seek(index) {
  if (!state.runs.summary) return;
  const clamped = Math.max(0, Math.min(state.runs.frameCount - 1, index));
  state.playback.index = clamped;
  try {
    const frame = await state.runs.frame(clamped);
    state.playback.frame = frame;
    state.playback.scene = sceneFromRunFrame(state.runs.name, state.runs.summary, frame);
    state.playback.error = null;
  } catch (error) {
    state.playback.error = error.message;
    state.playback.scene = null;
  }
  emit("scene");
}

export function play() {
  if (!state.runs.summary || state.playback.playing) return;
  state.playback.playing = true;
  const dt = Math.max(30, (state.runs.summary.fixed_delta_seconds ?? 0.05) * 1000);
  state.playback.timer = setInterval(async () => {
    if (state.playback.index >= state.runs.frameCount - 1) { stopPlayback(); emit("scene"); return; }
    await seek(state.playback.index + 1);
  }, dt);
  emit("scene");
}

export function stopPlayback() {
  if (state.playback.timer) clearInterval(state.playback.timer);
  state.playback.timer = null;
  state.playback.playing = false;
}

export async function selectReport(name) {
  if (!name) { state.reports.clear(); remember(); emit("report"); return; }
  await state.reports.load(name);
  remember();
  emit("report");
}

// ---- header -------------------------------------------------------------
function setPill(id, text, cls) {
  const pill = document.getElementById(id);
  pill.textContent = text;
  pill.className = `pill ${cls}`;
}

function updatePills() {
  const b = state.backend;
  setPill("pill-backend", b.ok ? `Backend: online${b.version ? " v" + b.version : ""}` : `Backend: unreachable`, b.ok ? "pill-ok" : "pill-off");
  const c = state.carlaStatus;
  if (!c) setPill("pill-carla", "CARLA: unknown", "pill-off");
  else {
    const stateText = String(c.status ?? "unknown");
    const sim = c.simulation ?? null;
    const version = sim && sim.server_version ? ` ${sim.server_version}` : "";
    const map = sim && sim.map_name ? ` · ${String(sim.map_name).split("/").pop()}` : "";
    setPill("pill-carla", `CARLA ${stateText}${version}${map}${c.is_mock ? " (mock)" : ""}`, stateText === "CONNECTED" ? "pill-ok" : stateText === "ERROR" ? "pill-err" : "pill-off");
  }
  const ls = state.liveStatus;
  if (!ls) setPill("pill-session", "Session: unavailable", "pill-off");
  else if (ls.state === "RUNNING" || ls.state === "PAUSED") setPill("pill-session", `Session ${ls.state}: ${ls.scenario_id} · seed ${ls.seed} · t ${fmtSeconds(ls.session_time_s ?? 0, 1)}`, ls.state === "RUNNING" ? "pill-ok" : "pill-warn");
  else setPill("pill-session", `Session: ${ls.state}${ls.state === "COLLIDED" ? " (safety stop)" : ""}`, ls.state === "ERROR" || ls.state === "COLLIDED" ? "pill-err" : "pill-off");
  const lag = document.getElementById("pill-lag");
  const timing = state.liveScene?.live?.timing ?? null;
  const lagging = currentMode() === "live" && timing !== null && timing.lagging && state.sceneState.connected && !state.sceneState.stale && ls && ls.state === "RUNNING";
  lag.classList.toggle("hidden", !lagging);
  if (lagging) lag.textContent = `PIPELINE LAGGING · ${fmt(timing.realtime_factor, { digits: 2 })}× wall-clock speed (loop ${fmtMs(timing.loop_ms, 0)} for a ${fmtMs(timing.fixed_delta_s * 1000, 0)} step)`;
  const t = state.telemetry.state;
  setPill("pill-telemetry", t.connected ? (t.stale ? `Telemetry: stale (last ${fmtTime(t.lastMessageAt)})` : "Telemetry: live") : `Telemetry: disconnected${t.lastMessageAt ? " (last " + fmtTime(t.lastMessageAt) + ")" : ""}`, t.connected && !t.stale ? "pill-ok" : t.connected ? "pill-warn" : "pill-off");
  const s = state.sceneState;
  const hasScene = state.liveSnapshot !== null;
  setPill("pill-scene", s.connected ? (hasScene ? (s.stale ? `Scene: idle (last frame ${fmtTime(s.lastMessageAt)})` : "Scene: streaming") : "Scene: connected, no frame yet") : "Scene: disconnected", s.connected ? (hasScene && !s.stale ? "pill-ok" : "pill-warn") : "pill-off");
  const mode = document.getElementById("pill-mode");
  const playback = currentMode() === "playback";
  mode.textContent = state.uiMode === "stored"
    ? (playback ? `STORED EVALUATION · playback of ${state.runs.name}` : "STORED EVALUATION · no run selected")
    : "LIVE SIMULATION";
  mode.className = `pill pill-mode ${state.uiMode === "stored" ? "stored" : ""}`;
  const scene = currentScene();
  const readout = document.getElementById("frame-readout");
  readout.textContent = scene
    ? `frame ${scene.frameId} · t ${scene.scenarioTimeS !== null && scene.scenarioTimeS !== undefined ? fmtSeconds(scene.scenarioTimeS) : fmtTime(scene.frameTimestamp)}${scene.scenarioId ? " · " + scene.scenarioId : ""}`
    : "frame — · t —";
  document.getElementById("footer-version").textContent = `${b.name ?? "ADAPT-X"}${b.version ? " v" + b.version : ""}`;
  document.getElementById("footer-source").textContent = playback ? `stored run ${state.runs.name}` : (scene ? `live scene from ${scene.origin} (source: ${scene.source})` : "no live scene yet");
  document.getElementById("rail-source").textContent = state.uiMode === "stored"
    ? (playback ? "Recorded run (playback) — nothing is recomputed" : "Stored evaluation: choose a run below to play it back")
    : (c && c.status === "CONNECTED" ? "Live scene from the running CARLA session" : "Live mode: CARLA is not connected — no live data is shown or invented");
}

// ---- navigation ---------------------------------------------------------
let activeUnmount = null;

function navigate() {
  const hash = window.location.hash.replace("#", "") || "overview";
  state.view = VIEWS[hash] ? hash : "overview";
  for (const link of document.querySelectorAll(".rail-link")) link.classList.toggle("active", link.dataset.view === state.view);
  if (activeUnmount) { activeUnmount(); activeUnmount = null; }
  const root = clear(document.getElementById("content"));
  activeUnmount = VIEWS[state.view](root) || null;
}

function bindRail() {
  document.getElementById("select-report").addEventListener("change", (e) => selectReport(e.target.value));
  document.getElementById("select-run").addEventListener("change", (e) => { selectRun(e.target.value); if (e.target.value) setUiMode("stored"); });
  document.getElementById("mode-live").addEventListener("change", () => setUiMode("live"));
  document.getElementById("mode-stored").addEventListener("change", () => setUiMode("stored"));
  document.getElementById("select-scenario").addEventListener("change", onScenarioChosen);
  for (const action of ["start", "pause", "resume", "stop", "reset"]) {
    document.getElementById(`btn-${action}`).addEventListener("click", () => liveAction(action));
  }
  const rememberedMode = recall()?.uiMode;
  state.uiMode = rememberedMode === "stored" ? "stored" : "live";
  document.getElementById("mode-live").checked = state.uiMode === "live";
  document.getElementById("mode-stored").checked = state.uiMode === "stored";
  document.getElementById("live-controls").classList.toggle("hidden", state.uiMode !== "live");
  const saved = recall()?.toggles;
  if (saved && typeof saved === "object") {
    for (const key of Object.keys(state.toggles)) if (typeof saved[key] === "boolean") state.toggles[key] = saved[key];
  }
  const bind = (id, key) => {
    const box = document.getElementById(id);
    box.checked = state.toggles[key];
    box.addEventListener("change", (e) => { state.toggles[key] = e.target.checked; remember(); emit("toggles"); });
  };
  bind("tg-points", "points"); bind("tg-tiles", "tiles"); bind("tg-traj", "trajectories");
  bind("tg-vel", "velocity"); bind("tg-labels", "labels"); bind("tg-grid", "grid");
}

// ---- boot ---------------------------------------------------------------
window.addEventListener("hashchange", navigate);
subscribe((topic) => { if (["status", "scene", "run", "channel", "live", "mode"].includes(topic)) updatePills(); });

bindRail();
navigate();
pollStatus();
loadStatic();
loadEvidenceLists();
startChannels();
pollLive();
setInterval(pollStatus, 3000);
setInterval(pollLive, 1000);
setInterval(loadEvidenceLists, 15000);
