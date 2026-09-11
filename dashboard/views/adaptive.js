// Adaptive Resolution: the signature view. Left, the current frame's tile
// grid beside a uniform grid at the fixed map's resolution, both from real
// data. Right, the stored report's paired fixed-vs-adaptive figures.

import { el, card, kv, notice, clear } from "../ui.js";
import { state, subscribe, currentScene } from "../app.js";
import { fmt, fmtInt, fmtMs, resolutionColour } from "../data/format.js";
import { TopDownView } from "../render/projection.js";
import { adaptiveCard } from "./evaluation.js";
import { pairedChart } from "../render/charts.js";

function fitView(canvas, tiles) {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(rect.width * dpr); canvas.height = Math.round(rect.height * dpr);
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = "#070b13"; ctx.fillRect(0, 0, rect.width, rect.height);
  const view = new TopDownView({ width: rect.width, height: rect.height });
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (const t of tiles) { minX = Math.min(minX, t.bounds.min_x); maxX = Math.max(maxX, t.bounds.max_x); minY = Math.min(minY, t.bounds.min_y); maxY = Math.max(maxY, t.bounds.max_y); }
  view.fit(minX, maxX, minY, maxY, 16);
  return { ctx, view, rect, bounds: { minX, maxX, minY, maxY } };
}

function drawAdaptive(canvas, scene) {
  const tiles = scene?.tiles ?? [];
  if (!tiles.length) { blank(canvas, "No tile decisions in this frame."); return; }
  const { ctx, view, rect } = fitView(canvas, tiles);
  for (const t of tiles) {
    const [x0, y0] = view.toScreen(t.bounds.max_x, t.bounds.max_y);
    const [x1, y1] = view.toScreen(t.bounds.min_x, t.bounds.min_y);
    const px = Math.min(x0, x1), py = Math.min(y0, y1), w = Math.abs(x1 - x0), h = Math.abs(y1 - y0);
    ctx.fillStyle = resolutionColour(t.level); ctx.fillRect(px, py, w, h);
    const cellPx = t.resolution_m / view.metresPerPixel;
    ctx.strokeStyle = "rgba(255,255,255,0.10)";
    if (cellPx >= 2.5) {
      for (let x = px; x <= px + w + 0.01; x += cellPx) { ctx.beginPath(); ctx.moveTo(x, py); ctx.lineTo(x, py + h); ctx.stroke(); }
      for (let y = py; y <= py + h + 0.01; y += cellPx) { ctx.beginPath(); ctx.moveTo(px, y); ctx.lineTo(px + w, y); ctx.stroke(); }
    }
    ctx.strokeStyle = "rgba(56,189,248,0.35)"; ctx.strokeRect(px, py, w, h);
  }
  markObjects(ctx, view, scene);
  ctx.fillStyle = "#38bdf8"; ctx.font = "10px monospace"; ctx.fillText(`ADAPTIVE — ${fmtInt(scene.adaptiveMap?.accounting.total_cell_count)} cells, ${scene.tiles.length} tiles`, 8, rect.height - 8);
}

function drawFixed(canvas, scene) {
  const tiles = scene?.tiles ?? [];
  const fixed = scene?.fixedMap ?? null;
  if (!tiles.length || !fixed) { blank(canvas, fixed ? "No tiles to frame the fixed grid." : "No fixed map in this frame."); return; }
  const { ctx, view, rect, bounds } = fitView(canvas, tiles);
  const [x0, y0] = view.toScreen(bounds.maxX, bounds.maxY);
  const [x1, y1] = view.toScreen(bounds.minX, bounds.minY);
  const px = Math.min(x0, x1), py = Math.min(y0, y1), w = Math.abs(x1 - x0), h = Math.abs(y1 - y0);
  ctx.fillStyle = "rgba(148,163,184,0.10)"; ctx.fillRect(px, py, w, h);
  const cellPx = fixed.resolution.resolution_m / view.metresPerPixel;
  ctx.strokeStyle = "rgba(255,255,255,0.10)";
  if (cellPx >= 2.5) {
    for (let x = px; x <= px + w + 0.01; x += cellPx) { ctx.beginPath(); ctx.moveTo(x, py); ctx.lineTo(x, py + h); ctx.stroke(); }
    for (let y = py; y <= py + h + 0.01; y += cellPx) { ctx.beginPath(); ctx.moveTo(px, y); ctx.lineTo(px + w, y); ctx.stroke(); }
  } else {
    ctx.fillStyle = "#7f8ca3"; ctx.font = "10px monospace"; ctx.fillText(`lattice too fine to draw at this zoom (${fixed.resolution.resolution_m} m cells)`, px + 6, py + 14);
  }
  ctx.strokeStyle = "rgba(148,163,184,0.5)"; ctx.strokeRect(px, py, w, h);
  markObjects(ctx, view, scene);
  ctx.fillStyle = "#94a3b8"; ctx.font = "10px monospace"; ctx.fillText(`FIXED — ${fmtInt(fixed.accounting.total_cell_count)} cells, uniform ${fixed.resolution.resolution_m} m`, 8, rect.height - 8);
}

function markObjects(ctx, view, scene) {
  for (const o of scene.objects) {
    const [px, py] = view.toScreen(o.track.position.x, o.track.position.y);
    ctx.fillStyle = "#e2e8f0"; ctx.beginPath(); ctx.arc(px, py, 3, 0, Math.PI * 2); ctx.fill();
  }
  const [ex, ey] = view.toScreen(0, 0);
  ctx.strokeStyle = "#38bdf8"; ctx.strokeRect(ex - 4, ey - 8, 8, 16);
}

function blank(canvas, text) {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(rect.width * dpr); canvas.height = Math.round(rect.height * dpr);
  const ctx = canvas.getContext("2d"); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = "#070b13"; ctx.fillRect(0, 0, rect.width, rect.height);
  ctx.fillStyle = "#7f8ca3"; ctx.font = "12px sans-serif"; ctx.fillText(text, 12, 24);
}

export function renderAdaptive(root) {
  const grid = el("div", { class: "grid-eval" });
  const fixedCanvas = el("canvas", { style: "width:100%;height:100%" });
  const adaptiveCanvas = el("canvas", { style: "width:100%;height:100%" });
  const fixedCard = card("FIXED baseline — uniform cells", el("div", { class: "canvas-wrap", style: "height:380px" }, [fixedCanvas]), { cls: "wide" });
  const adaptiveCard2 = card("ADAPTIVE — one map, several resolutions", el("div", { class: "canvas-wrap", style: "height:380px" }, [adaptiveCanvas]), { cls: "wide" });
  const frameNote = el("div", { class: "full" });
  const reportHost = el("div", { class: "full" });
  const chartCard = card("This frame: fixed vs adaptive workload", el("canvas", { class: "chart", style: "height:180px" }), { cls: "full" });
  grid.append(fixedCard, adaptiveCard2, frameNote, chartCard, reportHost);
  root.append(grid);

  function draw() {
    const scene = currentScene();
    drawFixed(fixedCanvas, scene); drawAdaptive(adaptiveCanvas, scene);
    clear(frameNote);
    if (scene) {
      const a = scene.adaptiveMap, cmp = scene.comparison;
      frameNote.append(card(`Frame ${scene.frameId} (${scene.mode})`, kv([
        ["tiles by level", a ? Object.entries(a.tiles_by_level).map(([k, v]) => `${k} ${v}`).join(" · ") : null],
        ["cells fixed / adaptive", cmp ? `${fmtInt(cmp.fixed.total_cell_count)} / ${fmtInt(cmp.adaptive.total_cell_count)}` : null],
        ["grid bytes fixed / adaptive", cmp ? `${fmtInt(cmp.fixed.grid_bytes)} / ${fmtInt(cmp.adaptive.grid_bytes)}` : null],
        ["build ms fixed / adaptive (+ controller)", cmp ? `${fmtMs(cmp.fixed.duration_ms)} / ${fmtMs(cmp.adaptive.duration_ms)} (+ ${fmtMs(a ? a.controller_duration_ms : null)})` : null],
        ["cells on high-priority tiles", cmp ? fmtInt(cmp.cells_on_high_priority_tiles) : null],
        ["cells on low-priority tiles", cmp ? fmtInt(cmp.cells_on_low_priority_tiles) : null],
        ["notes (from the comparison)", cmp ? cmp.notes : null],
        ["budget", scene.budget ? `${scene.budget.within_budget ? "within" : "OVER"} · demoted ${scene.budget.demoted_tile_count}` : null],
      ]), { tag: { text: scene.mode, cls: scene.mode === "live" ? "live" : "stored" } }));
      const canvas = chartCard.querySelector("canvas");
      if (cmp) {
        pairedChart(canvas, {
          title: "Per-frame workload", unit: "as labelled", source: "MappingComparison of this frame",
          categories: [
            { label: "cells", fixed: cmp.fixed.total_cell_count, adaptive: cmp.adaptive.total_cell_count, format: (v) => fmtInt(v) },
            { label: "occupied cells", fixed: cmp.fixed.occupied_cell_count, adaptive: cmp.adaptive.occupied_cell_count, format: (v) => fmtInt(v) },
            { label: "grid bytes", fixed: cmp.fixed.grid_bytes, adaptive: cmp.adaptive.grid_bytes, format: (v) => fmtInt(v) },
            { label: "build ms", fixed: cmp.fixed.duration_ms, adaptive: cmp.adaptive.duration_ms, format: (v) => fmt(v, { digits: 1 }) },
          ],
        });
      } else {
        blank(canvas, "No fixed-vs-adaptive comparison in this frame.");
      }
    } else {
      frameNote.append(notice("No frame. The frame panels draw the live scene or the playback frame; the report panel below draws the stored evaluation."));
    }
    clear(reportHost);
    const r = state.reports.report;
    reportHost.append(r ? adaptiveCard(r, { full: true }) : notice("No evaluation report loaded — the paired fixed-vs-adaptive figures over a whole run come from a stored report. Choose one in the rail."));
  }
  draw();
  const observer = new ResizeObserver(() => draw());
  observer.observe(fixedCanvas); observer.observe(adaptiveCanvas);
  const unsubscribe = subscribe((topic) => { if (["scene", "run", "report", "toggles"].includes(topic)) draw(); });
  return () => { unsubscribe(); observer.disconnect(); };
}
