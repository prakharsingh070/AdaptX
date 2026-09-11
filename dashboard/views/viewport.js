// The scene viewport: a canvas with 3-D / top-down tabs, pan, zoom, fit,
// click-to-select, and a playback bar when a recorded run is selected.
// Shared by the Overview and the Live Scene views.

import { el, clear } from "../ui.js";
import { TopDownRenderer, PerspectiveRenderer } from "../render/scene.js";
import { state, subscribe, currentScene, currentMode, selectTrack, seek, play, stopPlayback } from "../app.js";
import { fmtSeconds, fmtInt, fmt } from "../data/format.js";
import { api } from "../data/api.js";

export function mountViewport(root, { compact = false } = {}) {
  let mode = "3d";
  const tabs = el("div", { class: "viewport-tabs" });
  const tab3d = el("button", { text: "3D LiDAR View", class: "active", onclick: () => setMode("3d") });
  const tabTop = el("button", { text: "Top View", onclick: () => setMode("top") });
  // The Front View is the ego's RGB camera, a real CARLA sensor attached to
  // the ego and refreshed per live frame. Display only: no perception stage
  // reads it, and it is never a recording.
  const tabFront = el("button", { text: "Front View (camera)", onclick: () => setMode("front") });
  const fit = el("button", { text: "Fit", onclick: () => { top.fitScene(currentScene()); redraw(); } });
  const modeTag = el("span", { class: "tag live", text: "LIVE" });
  tabs.append(tab3d, tabTop, tabFront, fit, el("span", { class: "spacer" }), modeTag);
  const wrap = el("div", { class: "canvas-wrap" });
  const canvas3d = el("canvas");
  const canvasTop = el("canvas", { class: "hidden" });
  const camera = el("img", { class: "camera-frame hidden", alt: "ego camera" });
  const legend = el("div", { class: "legend" });
  const overlay = el("div", { class: "overlay-msg hidden" });
  wrap.append(canvas3d, canvasTop, camera, legend, overlay);
  const viewport = el("div", { class: "viewport card" }, [tabs, wrap]);
  root.append(viewport);

  const persp = new PerspectiveRenderer(canvas3d);
  const top = new TopDownRenderer(canvasTop);
  let fitted = false;
  // Debug hook for manual validation from the browser console; read-only use.
  window.adaptxDebug = { persp, top, currentScene };

  function setMode(next) {
    mode = next;
    tab3d.classList.toggle("active", mode === "3d");
    tabTop.classList.toggle("active", mode === "top");
    tabFront.classList.toggle("active", mode === "front");
    canvas3d.classList.toggle("hidden", mode !== "3d");
    canvasTop.classList.toggle("hidden", mode !== "top");
    camera.classList.toggle("hidden", mode !== "front");
    resize();
  }

  let lastCameraFrame = null;
  function refreshCamera(scene) {
    const available = currentMode() === "live" && state.liveStatus && state.liveStatus.camera_available;
    if (!available || !scene) { lastCameraFrame = null; camera.removeAttribute("src"); return false; }
    if (scene.frameId !== lastCameraFrame) {
      lastCameraFrame = scene.frameId;
      camera.src = api.liveCameraUrl(scene.frameId);
    }
    return true;
  }

  function resize() {
    persp.resize(); top.resize();
    redraw();
  }

  function redraw() {
    const scene = currentScene();
    Object.assign(persp.toggles, state.toggles); Object.assign(top.toggles, state.toggles);
    persp.selectedTrackId = state.selectedTrackId; top.selectedTrackId = state.selectedTrackId;
    // Fit the plan view once, and only once it has a real size: a hidden
    // canvas measures 0 x 0 and a fit against that collapses the map.
    if (!fitted && top.view.width > 1 && top.view.height > 1) { top.fitScene(scene); fitted = Boolean(scene); }
    if (mode === "3d") persp.draw(scene); else if (mode === "top") top.draw(scene);
    const cameraShown = mode === "front" ? refreshCamera(scene) : false;
    const playback = currentMode() === "playback";
    modeTag.textContent = playback ? "STORED EVALUATION · PLAYBACK" : "LIVE";
    modeTag.className = `tag ${playback ? "stored" : "live"}`;
    if (mode === "front" && !cameraShown) {
      overlay.classList.remove("hidden");
      overlay.textContent = playback
        ? "No camera in playback: recorded runs carry no images."
        : "Camera not available: no live session with the RGB camera attached (ADAPTX_LIVE__CAMERA_ENABLED).";
    } else if (!scene) {
      overlay.classList.remove("hidden");
      const carla = state.carlaStatus;
      overlay.textContent = playback
        ? (state.playback.error ? `Run could not be loaded: ${state.playback.error}` : "Loading frame…")
        : state.uiMode === "stored"
          ? "STORED EVALUATION: choose a recorded run in the rail to play it back."
          : !state.sceneState.connected
            ? "Scene channel disconnected — no live data."
            : carla && carla.status !== "CONNECTED"
              ? `CARLA ${carla ? carla.status : "DISCONNECTED"} — no live data. Start a live session in the rail (${carla ? carla.detail : ""}).`
              : "No frame has been processed yet. Start a live session in the rail.";
    } else {
      overlay.classList.add("hidden");
    }
    const stale = !playback && state.sceneState.stale && scene;
    const timing = scene && scene.live ? scene.live.timing : null;
    legend.innerHTML = `<div><span style="background:#34d399"></span>LOW <span style="background:#fbbf24"></span>MEDIUM <span style="background:#fb923c"></span>HIGH <span style="background:#ef4444"></span>CRITICAL <span style="background:#a78bfa;border:1px dashed #a78bfa"></span>UNKNOWN (dashed)</div>`
      + `<div>tiles: <span style="background:rgba(52,211,153,0.5)"></span>medium <span style="background:rgba(251,191,36,0.6)"></span>high <span style="background:rgba(239,68,68,0.6)"></span>critical · low is unfilled</div>`
      + (scene && scene.points ? `<div>points: ${fmtInt(scene.points.sample_count)} of ${fmtInt(scene.points.total_count)} (${scene.points.stage}, ${scene.points.is_downsampled ? "downsampled for display" : "complete"}), height-coloured</div>` : `<div>no point cloud in ${playback ? "playback (records carry none)" : "this frame"}</div>`)
      + (stale ? `<div style="color:#fbbf24">STALE — last frame received ${new Date(state.sceneState.lastMessageAt).toLocaleTimeString()}</div>` : "")
      + (timing ? `<div style="color:${timing.lagging ? "#fbbf24" : "#8b98b0"}">loop ${fmt(timing.loop_ms, { digits: 0 })} ms for a ${fmt(timing.fixed_delta_s * 1000, { digits: 0 })} ms step · ${fmt(timing.realtime_factor, { digits: 2 })}× wall-clock speed${timing.lagging ? " · PIPELINE LAGGING" : ""} · ${fmtInt(state.skippedFrames)} frame(s) skipped for display</div>` : "")
      + (mode === "front" && cameraShown ? `<div>ego RGB camera, live from CARLA (display only; perception uses LiDAR)</div>` : "");
  }

  // interaction
  let drag = null;
  const pointerDown = (e) => { drag = { x: e.clientX, y: e.clientY, moved: false }; };
  const pointerMove = (e) => {
    if (!drag) return;
    const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
    if (Math.abs(dx) + Math.abs(dy) > 2) drag.moved = true;
    if (mode === "3d") persp.view.orbit(-dx * 0.005); else top.view.panPixels(dx, dy);
    drag.x = e.clientX; drag.y = e.clientY;
    redraw();
  };
  const pointerUp = (e) => {
    if (drag && !drag.moved) {
      const rect = (mode === "3d" ? canvas3d : canvasTop).getBoundingClientRect();
      const hit = (mode === "3d" ? persp : top).hitTest(e.clientX - rect.left, e.clientY - rect.top);
      selectTrack(hit);
    }
    drag = null;
  };
  const wheel = (e) => {
    e.preventDefault();
    const factor = e.deltaY > 0 ? 1.15 : 1 / 1.15;
    if (mode === "3d") persp.view.zoom(1 / factor);
    else { const rect = canvasTop.getBoundingClientRect(); top.view.zoomAt(e.clientX - rect.left, e.clientY - rect.top, factor); }
    redraw();
  };
  for (const c of [canvas3d, canvasTop]) {
    c.addEventListener("pointerdown", pointerDown);
    c.addEventListener("pointermove", pointerMove);
    c.addEventListener("pointerup", pointerUp);
    c.addEventListener("pointerleave", () => { drag = null; });
    c.addEventListener("wheel", wheel, { passive: false });
  }

  // playback bar
  let bar = null;
  if (!compact) {
    bar = el("div", { class: "playback card" });
    root.append(bar);
  }
  function renderBar() {
    if (!bar) return;
    clear(bar);
    if (currentMode() !== "playback") {
      bar.append(el("span", { class: "muted small", text: "Live scene. Select a recorded run in the rail to enable frame playback." }));
      return;
    }
    const total = state.runs.frameCount;
    const idx = state.playback.index;
    const slider = el("input", { type: "range", min: 0, max: Math.max(0, total - 1), value: idx, oninput: (e) => seek(Number(e.target.value)) });
    const scene = currentScene();
    bar.append(
      el("button", { text: "⏮", onclick: () => seek(0) }),
      el("button", { text: "◀", onclick: () => seek(idx - 1) }),
      el("button", { text: state.playback.playing ? "❚❚" : "▶", onclick: () => (state.playback.playing ? (stopPlayback(), redrawAll()) : play()) }),
      el("button", { text: "▶|", onclick: () => seek(idx + 1) }),
      el("button", { text: "⏭", onclick: () => seek(total - 1) }),
      slider,
      el("span", { class: "mono small", text: `frame ${idx + 1}/${total}${scene ? ` · id ${scene.frameId} · t ${fmtSeconds(scene.scenarioTimeS)}` : ""}` }),
      el("span", { class: "tag stored", text: "playback of recorded outputs — nothing is recomputed" }),
    );
  }

  function redrawAll() { redraw(); renderBar(); }
  const unsubscribe = subscribe((topic) => {
    if (["scene", "selection", "toggles", "run", "channel"].includes(topic)) {
      if (topic === "run") fitted = false;
      redrawAll();
    }
  });
  const observer = new ResizeObserver(() => resize());
  observer.observe(wrap);
  resize();
  renderBar();
  return () => { unsubscribe(); observer.disconnect(); };
}
