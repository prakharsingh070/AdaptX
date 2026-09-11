// Small canvas charts for values the report already holds. Each function
// draws exactly the numbers it is given: no smoothing, no aggregation, no
// derived series. Every chart carries a title, a unit and a legend.
import { NOT_AVAILABLE } from "../data/format.js";

function prepare(canvas) {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.round(rect.width * dpr));
  canvas.height = Math.max(1, Math.round(rect.height * dpr));
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, rect.width, rect.height);
  ctx.fillStyle = "#0b1120";
  ctx.fillRect(0, 0, rect.width, rect.height);
  return { ctx, width: rect.width, height: rect.height };
}

function textLine(ctx, text, x, y, colour = "#7f8ca3") {
  ctx.fillStyle = colour;
  ctx.font = "10px monospace";
  ctx.fillText(text, x, y);
}

/** Horizontal bars. items: [{label, value, colour}], unit string, source string. */
export function barChart(canvas, { title, unit = "", items, source = "" }) {
  const { ctx, width, height } = prepare(canvas);
  textLine(ctx, `${title}${unit ? ` (${unit})` : ""}`, 8, 12, "#dbe4f3");
  const present = items.filter((i) => i.value !== null && i.value !== undefined && Number.isFinite(i.value));
  const max = Math.max(1e-9, ...present.map((i) => i.value));
  const top = 20, rowH = Math.min(22, (height - top - 16) / Math.max(1, items.length));
  const labelW = 130;
  items.forEach((item, index) => {
    const y = top + index * rowH;
    textLine(ctx, item.label, 8, y + rowH * 0.7);
    if (item.value === null || item.value === undefined) {
      textLine(ctx, NOT_AVAILABLE, labelW, y + rowH * 0.7, "#8b98b0");
      return;
    }
    const w = ((width - labelW - 60) * item.value) / max;
    ctx.fillStyle = item.colour || "#38bdf8";
    ctx.fillRect(labelW, y + rowH * 0.2, Math.max(1, w), rowH * 0.55);
    textLine(ctx, item.format ? item.format(item.value) : String(item.value), labelW + Math.max(1, w) + 6, y + rowH * 0.7, "#dbe4f3");
  });
  if (source) textLine(ctx, `source: ${source}`, 8, height - 5);
}

/** Line/points over an ordered x axis. series: [{label, colour, points: [{x, y}]}]. */
export function lineChart(canvas, { title, unit = "", xLabel = "", series, source = "" }) {
  const { ctx, width, height } = prepare(canvas);
  textLine(ctx, `${title}${unit ? ` (${unit})` : ""}`, 8, 12, "#dbe4f3");
  const all = series.flatMap((s) => s.points).filter((p) => p.y !== null && p.y !== undefined);
  if (!all.length) { textLine(ctx, `${NOT_AVAILABLE} (no samples)`, 8, 32, "#8b98b0"); return; }
  const xs = all.map((p) => p.x), ys = all.map((p) => p.y);
  const minX = Math.min(...xs), maxX = Math.max(...xs), minY = 0, maxY = Math.max(1e-9, ...ys);
  const left = 44, right = width - 12, top = 22, bottom = height - 26;
  const sx = (x) => left + ((x - minX) / Math.max(1e-9, maxX - minX)) * (right - left);
  const sy = (y) => bottom - ((y - minY) / (maxY - minY)) * (bottom - top);
  ctx.strokeStyle = "#1f2a3d"; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(left, top); ctx.lineTo(left, bottom); ctx.lineTo(right, bottom); ctx.stroke();
  textLine(ctx, maxY.toFixed(2), 4, top + 4); textLine(ctx, "0", 4, bottom);
  textLine(ctx, xLabel, right - xLabel.length * 6, height - 6);
  series.forEach((s, si) => {
    ctx.strokeStyle = s.colour; ctx.fillStyle = s.colour; ctx.lineWidth = 1.5;
    ctx.beginPath();
    let started = false;
    for (const p of s.points) {
      if (p.y === null || p.y === undefined) { started = false; continue; }
      const px = sx(p.x), py = sy(p.y);
      if (!started) { ctx.moveTo(px, py); started = true; } else ctx.lineTo(px, py);
    }
    ctx.stroke();
    for (const p of s.points) {
      if (p.y === null || p.y === undefined) continue;
      ctx.beginPath(); ctx.arc(sx(p.x), sy(p.y), 2.2, 0, Math.PI * 2); ctx.fill();
    }
    textLine(ctx, `■ ${s.label}`, left + 4 + si * 120, top - 2, s.colour);
  });
  if (source) textLine(ctx, `source: ${source}`, 8, height - 6);
}

/** Two bars side by side per category: fixed vs adaptive. Each category is
 * drawn on its own scale (its taller bar fills the height), because the
 * categories carry different units; the comparison is within a pair only. */
export function pairedChart(canvas, { title, unit = "", categories, source = "" }) {
  const { ctx, width, height } = prepare(canvas);
  textLine(ctx, `${title}${unit ? ` (${unit})` : ""} — each pair on its own scale`, 8, 12, "#dbe4f3");
  textLine(ctx, "■ fixed", width - 150, 12, "#94a3b8");
  textLine(ctx, "■ adaptive", width - 90, 12, "#38bdf8");
  const left = 8, top = 22, bottom = height - 24;
  const slot = (width - left - 8) / Math.max(1, categories.length);
  categories.forEach((c, i) => {
    const x = left + i * slot;
    const barW = Math.max(6, slot * 0.28);
    const max = Math.max(1e-9, c.fixed ?? 0, c.adaptive ?? 0);
    const draw = (value, colour, offset) => {
      if (value === null || value === undefined) { textLine(ctx, "n/a", x + offset, bottom - 4, "#8b98b0"); return; }
      const h = ((bottom - top - 14) * value) / max;
      ctx.fillStyle = colour; ctx.fillRect(x + offset, bottom - h, barW, h);
      textLine(ctx, c.format ? c.format(value) : String(value), x + offset, bottom - h - 3, "#dbe4f3");
    };
    draw(c.fixed, "#94a3b8", slot * 0.15);
    draw(c.adaptive, "#38bdf8", slot * 0.15 + barW + 4);
    textLine(ctx, c.label, x + 2, height - 8);
  });
  if (source) textLine(ctx, `source: ${source}`, width - 8 - source.length * 6 - 48, height - 8);
}

/**
 * A top-down map of one per-tile factor (e.g. `factor_scores.risk`) from the
 * adaptive resolution plan, +X up and +Y left like the scene. Each tile is
 * filled by the value the controller recorded for it; a null factor (not
 * computed for that tile) is hatched, never painted as zero. Nothing is
 * interpolated between tiles.
 */
export function tileFactorMap(canvas, tiles, key, { title = "", palette = "risk" } = {}) {
  const { ctx, width, height } = prepare(canvas);
  if (!tiles || !tiles.length) { textLine(ctx, `${NOT_AVAILABLE} (no tiles in this frame)`, 8, 20, "#8b98b0"); return; }
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (const t of tiles) {
    minX = Math.min(minX, t.bounds.min_x); maxX = Math.max(maxX, t.bounds.max_x);
    minY = Math.min(minY, t.bounds.min_y); maxY = Math.max(maxY, t.bounds.max_y);
  }
  const top = title ? 18 : 4;
  const scale = Math.min((width - 8) / (maxY - minY), (height - top - 4) / (maxX - minX));
  const ox = width / 2 + ((maxY + minY) / 2) * scale; // +Y is left on screen
  const oy = top + (maxX - minX) * scale / 2 + ((maxX + minX) / 2) * scale; // +X is up
  const sx = (y) => ox - y * scale, sy = (x) => oy - x * scale;
  for (const t of tiles) {
    const b = t.bounds;
    const x0 = sx(b.max_y), x1 = sx(b.min_y), y0 = sy(b.max_x), y1 = sy(b.min_x);
    const value = t.factor_scores ? t.factor_scores[key] : null;
    if (value === null || value === undefined) {
      ctx.fillStyle = "#0f1524"; ctx.fillRect(x0, y0, x1 - x0, y1 - y0);
      ctx.strokeStyle = "rgba(139,152,176,0.35)"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(x0, y1); ctx.lineTo(x1, y0); ctx.stroke();
    } else {
      ctx.fillStyle = factorColour(value, palette);
      ctx.fillRect(x0, y0, x1 - x0, y1 - y0);
    }
    ctx.strokeStyle = "rgba(56,189,248,0.18)"; ctx.lineWidth = 1; ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
  }
  // the ego, at the origin
  ctx.fillStyle = "#e2e8f0"; ctx.fillRect(sx(0.9), sy(2.3), sx(-0.9) - sx(0.9), sy(-2.3) - sy(2.3));
  if (title) textLine(ctx, title, 8, 12, "#8b98b0");
}

function factorColour(value, palette) {
  const v = Math.max(0, Math.min(1, value));
  if (palette === "risk") {
    if (v < 0.35) return `rgba(52,211,153,${0.25 + v})`;
    if (v < 0.6) return `rgba(251,191,36,${0.35 + v * 0.5})`;
    if (v < 0.85) return `rgba(251,146,60,${0.45 + v * 0.4})`;
    return `rgba(239,68,68,${0.55 + v * 0.4})`;
  }
  return `rgba(56,189,248,${0.15 + v * 0.8})`;
}
