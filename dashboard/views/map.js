// Spatial Map: the fixed map's summary and the adaptive tile grid of the
// current frame, drawn tile by tile with each tile's actual cell size. The
// per-cell grid is not streamed, so per-cell occupancy is not shown live and
// the legend says which states exist and that unobserved is not free.

import { el, card, kv, notice, clear } from "../ui.js";
import { state, subscribe, currentScene } from "../app.js";
import { fmt, fmtInt, fmtMs, fmtMetres, resolutionColour } from "../data/format.js";
import { TopDownView } from "../render/projection.js";

function drawTiles(canvas, scene) {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(rect.width * dpr); canvas.height = Math.round(rect.height * dpr);
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = "#070b13"; ctx.fillRect(0, 0, rect.width, rect.height);
  const view = new TopDownView({ width: rect.width, height: rect.height });
  const tiles = scene?.tiles ?? [];
  if (!tiles.length) {
    ctx.fillStyle = "#7f8ca3"; ctx.font = "12px sans-serif"; ctx.fillText("No tile decisions in this frame.", 12, 24);
    return;
  }
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (const t of tiles) { minX = Math.min(minX, t.bounds.min_x); maxX = Math.max(maxX, t.bounds.max_x); minY = Math.min(minY, t.bounds.min_y); maxY = Math.max(maxY, t.bounds.max_y); }
  view.fit(minX, maxX, minY, maxY, 24);
  for (const t of tiles) {
    const [x0, y0] = view.toScreen(t.bounds.max_x, t.bounds.max_y);
    const [x1, y1] = view.toScreen(t.bounds.min_x, t.bounds.min_y);
    const px = Math.min(x0, x1), py = Math.min(y0, y1), w = Math.abs(x1 - x0), h = Math.abs(y1 - y0);
    ctx.fillStyle = resolutionColour(t.level); ctx.fillRect(px, py, w, h);
    ctx.strokeStyle = "rgba(56,189,248,0.25)"; ctx.strokeRect(px, py, w, h);
    // Draw the tile's own cell lattice so the resolution is visible, not just its colour.
    const cellPx = t.resolution_m / view.metresPerPixel;
    if (cellPx >= 3) {
      ctx.strokeStyle = "rgba(255,255,255,0.08)";
      for (let x = px; x < px + w; x += cellPx) { ctx.beginPath(); ctx.moveTo(x, py); ctx.lineTo(x, py + h); ctx.stroke(); }
      for (let y = py; y < py + h; y += cellPx) { ctx.beginPath(); ctx.moveTo(px, y); ctx.lineTo(px + w, y); ctx.stroke(); }
    }
    if (w > 28) { ctx.fillStyle = "#dbe4f3"; ctx.font = "9px monospace"; ctx.fillText(`${t.resolution_m}m`, px + 2, py + 10); }
  }
  for (const o of scene.objects) {
    const [px, py] = view.toScreen(o.track.position.x, o.track.position.y);
    ctx.fillStyle = "#e2e8f0"; ctx.beginPath(); ctx.arc(px, py, 3, 0, Math.PI * 2); ctx.fill();
  }
  const [ex, ey] = view.toScreen(0, 0);
  ctx.strokeStyle = "#38bdf8"; ctx.lineWidth = 1.5; ctx.strokeRect(ex - 4, ey - 8, 8, 16);
  ctx.fillStyle = "#38bdf8"; ctx.font = "10px monospace"; ctx.fillText("+X ahead ↑   +Y left ←", 10, rect.height - 8);
}

export function renderMap(root) {
  const grid = el("div", { class: "grid-eval" });
  const canvasCard = card("Adaptive tile grid — each tile at its actual cell size", el("div", { class: "canvas-wrap", style: "height:520px" }), { cls: "wide" });
  const canvas = el("canvas", { style: "width:100%;height:100%;border-radius:8px" });
  canvasCard.querySelector(".canvas-wrap").append(canvas);
  const legend = el("div", { class: "note" }, [
    "Legend: fill = tile level (low unfilled, medium green, high amber, critical red); lattice = cell size in metres. ",
    "Cell states in the map contract: OBSERVED_OCCUPIED, OBSERVED_EMPTY, OUT_OF_BOUNDS, UNOBSERVED — an unobserved cell reports null height and is NOT free space. ",
    "Per-cell occupancy is not streamed to the dashboard; request POST /api/v1/lidar/map for a full grid.",
  ]);
  canvasCard.append(legend);
  const fixedCard = el("div"), adaptiveCard = el("div"), tilesCard = el("div", { class: "full" });
  grid.append(canvasCard, el("div", { style: "display:flex;flex-direction:column;gap:12px", class: "wide" }, [fixedCard, adaptiveCard]), tilesCard);
  root.append(grid);

  function draw() {
    const scene = currentScene();
    drawTiles(canvas, scene);
    const tag = scene ? { text: scene.mode, cls: scene.mode === "live" ? "live" : "stored" } : null;
    clear(fixedCard);
    const f = scene?.fixedMap ?? null;
    fixedCard.append(f ? card("Fixed-resolution map (Phase 6 baseline)", [kv([
      ["resolution (m/cell)", f.resolution.resolution_m],
      ["bounds x (m)", `${f.bounds.min_x} … ${f.bounds.max_x}`], ["bounds y (m)", `${f.bounds.min_y} … ${f.bounds.max_y}`],
      ["grid", `${f.width} × ${f.height} = ${fmtInt(f.accounting.total_cell_count)} cells`],
      ["occupied / total cells", `${fmtInt(f.accounting.occupied_cell_count)} / ${fmtInt(f.accounting.total_cell_count)}`],
      ["points mapped / out of bounds", `${fmtInt(f.accounting.mapped_point_count)} / ${fmtInt(f.accounting.out_of_bounds_point_count)}`],
      ["build (ms)", fmtMs(f.duration_ms)],
      ["occupancy accuracy", null, { missing: "Not evaluated (no reference)" }],
    ])], { tag }) : card("Fixed-resolution map", notice("No fixed map in this frame (built when the request asks for map context or a comparison).")));
    clear(adaptiveCard);
    const a = scene?.adaptiveMap ?? null;
    adaptiveCard.append(a ? card("Adaptive map (Phase 8)", [kv([
      ["tiles", `${a.tile_count} × ${a.tile_size_m} m`],
      ["cells", fmtInt(a.accounting.total_cell_count)],
      ["cells by level", Object.entries(a.cells_by_level).map(([k, v]) => `${k} ${fmtInt(v)}`).join(" · ")],
      ["occupied cells", fmtInt(a.accounting.occupied_cell_count)],
      ["finest / coarsest (m)", `${a.finest_resolution_m} / ${a.coarsest_resolution_m}`],
      ["area-weighted (m)", fmt(a.area_weighted_resolution_m, { digits: 3 })],
      ["grid bytes", fmtInt(a.grid_bytes)],
      ["controller / mapping (ms)", `${fmtMs(a.controller_duration_ms)} / ${fmtMs(a.mapping_duration_ms)}`],
      ["budget", scene.budget ? `${fmtInt(scene.budget.total_cell_count)} / ${fmtInt(scene.budget.max_total_cells)} cells, ${scene.budget.within_budget ? "within" : "OVER"} budget, ${scene.budget.demoted_tile_count} demoted` : null],
    ])], { tag }) : card("Adaptive map", notice("No adaptive map in this frame.")));
    clear(tilesCard);
    if (scene?.tiles?.length) {
      const rows = scene.tiles.filter((t) => String(t.level).toLowerCase() !== "low").map((t) => [
        { num: t.tile_index }, t.level, { num: fmtMetres(t.resolution_m) }, { num: t.cell_width * t.cell_height },
        t.detail_priority === null || t.detail_priority === undefined ? "Not available (no influence)" : { num: fmt(t.detail_priority, { digits: 3 }) },
        (t.reasons ?? []).join(", "), t.changed ? "yes" : "no", (t.influencing_track_ids ?? []).join(" "),
      ]);
      tilesCard.append(card(`Tiles above the base level (${rows.length} of ${scene.tiles.length})`, rows.length ? el("div", { style: "max-height:260px;overflow:auto" }, [tableFor(rows)]) : notice("Every tile is at the base level in this frame.")));
    }
  }
  function tableFor(rows) {
    const t = document.createElement("table"); t.className = "data";
    t.innerHTML = "<thead><tr><th>tile</th><th>level</th><th>cell size</th><th>cells</th><th>detail priority (heuristic)</th><th>reasons</th><th>changed</th><th>influencing tracks</th></tr></thead>";
    const body = document.createElement("tbody");
    for (const r of rows) {
      const tr = document.createElement("tr");
      for (const c of r) { const td = document.createElement("td"); if (c && typeof c === "object" && "num" in c) { td.className = "num"; td.textContent = c.num; } else td.textContent = c; tr.append(td); }
      body.append(tr);
    }
    t.append(body);
    return t;
  }
  draw();
  const observer = new ResizeObserver(() => draw());
  observer.observe(canvas);
  const unsubscribe = subscribe((topic) => { if (["scene", "run", "toggles"].includes(topic)) draw(); });
  return () => { unsubscribe(); observer.disconnect(); };
}
