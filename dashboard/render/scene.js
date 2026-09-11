// Canvas renderers for a scene view model (data/normalise.js): a top-down
// plan and a perspective picture. Both draw exactly what the model holds -
// the point sample, tiles, tracks, predicted paths, velocity vectors - and
// colour risk by the assessment's own level. Nothing is computed here beyond
// pixel placement.

import { TopDownView, PerspectiveView, heightColour } from "./projection.js";
import { levelColour, resolutionColour } from "../data/format.js";

const GROUND_Z = -1.8; // sensor sits 1.8 m above the ego origin by default; drawn as a reference only

function boxCorners(track) {
  const box = track.bounding_box;
  const cx = track.position.x, cy = track.position.y, cz = track.position.z;
  if (!box) {
    const h = 1.0;
    return [[cx - h, cy - h, cz - 0.5], [cx + h, cy - h, cz - 0.5], [cx + h, cy + h, cz - 0.5], [cx - h, cy + h, cz - 0.5],
      [cx - h, cy - h, cz + 0.5], [cx + h, cy - h, cz + 0.5], [cx + h, cy + h, cz + 0.5], [cx - h, cy + h, cz + 0.5]];
  }
  const c = box.center ?? track.position;
  const l = box.dimensions.length / 2, w = box.dimensions.width / 2, h = box.dimensions.height / 2;
  const yaw = box.yaw_rad ?? 0;
  const cosY = Math.cos(yaw), sinY = Math.sin(yaw);
  const corners = [];
  for (const dz of [-h, h]) {
    for (const [dx, dy] of [[-l, -w], [l, -w], [l, w], [-l, w]]) {
      corners.push([c.x + dx * cosY - dy * sinY, c.y + dx * sinY + dy * cosY, c.z + dz]);
    }
  }
  return corners;
}

function riskOf(object) {
  return object.assessment ? object.assessment.risk_level : null;
}

export class TopDownRenderer {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.view = new TopDownView();
    this.toggles = { points: true, tiles: true, trajectories: true, velocity: true, labels: true, grid: true };
    this.selectedTrackId = null;
    this.hitboxes = [];
  }

  resize() {
    const rect = this.canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = Math.max(1, Math.round(rect.width * dpr));
    this.canvas.height = Math.max(1, Math.round(rect.height * dpr));
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.view.width = rect.width;
    this.view.height = rect.height;
  }

  fitScene(scene) {
    if (scene && scene.tiles && scene.tiles.length) {
      let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
      for (const t of scene.tiles) {
        minX = Math.min(minX, t.bounds.min_x); maxX = Math.max(maxX, t.bounds.max_x);
        minY = Math.min(minY, t.bounds.min_y); maxY = Math.max(maxY, t.bounds.max_y);
      }
      this.view.fit(minX, maxX, minY, maxY);
    } else {
      this.view.fit(-20, 60, -30, 30);
    }
  }

  /** Track id under a screen point, or null. */
  hitTest(px, py) {
    for (const hit of this.hitboxes) {
      if (px >= hit.x0 && px <= hit.x1 && py >= hit.y0 && py <= hit.y1) return hit.trackId;
    }
    return null;
  }

  draw(scene) {
    const ctx = this.ctx;
    const { width, height } = this.view;
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = "#070b13";
    ctx.fillRect(0, 0, width, height);
    this.hitboxes = [];

    if (this.toggles.grid) this._grid();
    if (!scene) { this._axes(); return; }
    if (this.toggles.tiles) this._tiles(scene.tiles);
    if (this.toggles.points && scene.points) this._points(scene.points);
    this._ego();
    for (const object of scene.objects) this._object(object);
    this._axes();
  }

  _grid() {
    const ctx = this.ctx, v = this.view;
    const step = v.metresPerPixel > 0.25 ? 20 : v.metresPerPixel > 0.08 ? 10 : 5;
    const [x0, y0] = v.toWorld(0, v.height);
    const [x1, y1] = v.toWorld(v.width, 0);
    ctx.strokeStyle = "rgba(255,255,255,0.05)";
    ctx.lineWidth = 1;
    for (let x = Math.floor(x0 / step) * step; x <= x1; x += step) {
      const [, py] = v.toScreen(x, 0);
      ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(v.width, py); ctx.stroke();
    }
    for (let y = Math.floor(Math.min(y0, y1) / step) * step; y <= Math.max(y0, y1); y += step) {
      const [px] = v.toScreen(0, y);
      ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, v.height); ctx.stroke();
    }
  }

  _axes() {
    const ctx = this.ctx;
    const x = 34, y = this.view.height - 34;
    ctx.strokeStyle = "#38bdf8"; ctx.fillStyle = "#38bdf8"; ctx.lineWidth = 1.5; ctx.font = "10px monospace";
    ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x, y - 26); ctx.stroke(); ctx.fillText("+X ahead", x + 4, y - 20);
    ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x - 26, y); ctx.stroke(); ctx.fillText("+Y left", x - 26, y + 12);
  }

  _tiles(tiles) {
    const ctx = this.ctx, v = this.view;
    for (const tile of tiles ?? []) {
      const [x0, y0] = v.toScreen(tile.bounds.max_x, tile.bounds.max_y);
      const [x1, y1] = v.toScreen(tile.bounds.min_x, tile.bounds.min_y);
      ctx.fillStyle = resolutionColour(tile.level);
      ctx.fillRect(Math.min(x0, x1), Math.min(y0, y1), Math.abs(x1 - x0), Math.abs(y1 - y0));
      ctx.strokeStyle = "rgba(56,189,248,0.15)";
      ctx.strokeRect(Math.min(x0, x1), Math.min(y0, y1), Math.abs(x1 - x0), Math.abs(y1 - y0));
    }
  }

  _points(sample) {
    const ctx = this.ctx, v = this.view;
    const xyz = sample.xyz;
    let minZ = Infinity, maxZ = -Infinity;
    for (const p of xyz) { if (p[2] < minZ) minZ = p[2]; if (p[2] > maxZ) maxZ = p[2]; }
    const span = Math.max(0.5, maxZ - minZ);
    for (const p of xyz) {
      const [px, py] = v.toScreen(p[0], p[1]);
      if (px < -2 || py < -2 || px > v.width + 2 || py > v.height + 2) continue;
      ctx.fillStyle = heightColour((p[2] - minZ) / span);
      ctx.fillRect(px, py, 1.5, 1.5);
    }
  }

  _ego() {
    const ctx = this.ctx, v = this.view;
    const [px, py] = v.toScreen(0, 0);
    ctx.strokeStyle = "#e2e8f0"; ctx.fillStyle = "rgba(226,232,240,0.15)"; ctx.lineWidth = 1.5;
    const l = 4.7 / v.metresPerPixel / 2, w = 1.9 / v.metresPerPixel / 2;
    ctx.beginPath(); ctx.rect(px - w, py - l, 2 * w, 2 * l); ctx.fill(); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(px - w, py - l); ctx.lineTo(px, py - l - 8); ctx.lineTo(px + w, py - l); ctx.stroke();
    ctx.fillStyle = "#e2e8f0"; ctx.font = "10px monospace"; ctx.fillText("EGO", px + w + 4, py + 3);
  }

  _object(object) {
    const ctx = this.ctx, v = this.view;
    const track = object.track;
    const colour = levelColour(riskOf(object));
    const selected = object.trackId === this.selectedTrackId;
    const corners = boxCorners(track).slice(0, 4).map(([x, y]) => v.toScreen(x, y));
    ctx.strokeStyle = colour; ctx.lineWidth = selected ? 2.5 : 1.5;
    ctx.setLineDash(riskOf(object) === "unknown" ? [4, 3] : []);
    ctx.beginPath();
    corners.forEach(([px, py], i) => (i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py)));
    ctx.closePath(); ctx.stroke(); ctx.setLineDash([]);
    if (selected) { ctx.fillStyle = "rgba(56,189,248,0.18)"; ctx.fill(); }
    const xs = corners.map((c) => c[0]), ys = corners.map((c) => c[1]);
    const x0 = Math.min(...xs) - 4, x1 = Math.max(...xs) + 4, y0 = Math.min(...ys) - 4, y1 = Math.max(...ys) + 4;
    this.hitboxes.push({ trackId: object.trackId, x0, x1, y0, y1 });

    const [cx, cy] = v.toScreen(track.position.x, track.position.y);
    if (this.toggles.velocity && track.velocity) {
      const [vx, vy] = v.toScreen(track.position.x + track.velocity.x, track.position.y + track.velocity.y);
      ctx.strokeStyle = "#e2e8f0"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(vx, vy); ctx.stroke();
    }
    if (this.toggles.trajectories && object.trajectory) {
      ctx.strokeStyle = "rgba(34,211,238,0.9)"; ctx.lineWidth = 1.2; ctx.setLineDash([3, 3]);
      ctx.beginPath();
      object.trajectory.points.forEach((p, i) => {
        const [px, py] = v.toScreen(p.position.x, p.position.y);
        if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
      });
      ctx.stroke(); ctx.setLineDash([]);
      const last = object.trajectory.points[object.trajectory.points.length - 1];
      if (last && last.position_uncertainty_m > 0) {
        const [px, py] = v.toScreen(last.position.x, last.position.y);
        ctx.strokeStyle = "rgba(34,211,238,0.35)";
        ctx.beginPath(); ctx.arc(px, py, last.position_uncertainty_m / v.metresPerPixel, 0, Math.PI * 2); ctx.stroke();
      }
    }
    if (this.toggles.labels) {
      ctx.fillStyle = colour; ctx.font = "10px monospace";
      const level = riskOf(object);
      ctx.fillText(`#${track.track_id} ${track.object_class}${level ? " " + String(level).toUpperCase() : ""}`, x1 + 2, y0 + 10);
    }
  }
}

export class PerspectiveRenderer {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.view = new PerspectiveView();
    this.toggles = { points: true, tiles: true, trajectories: true, velocity: true, labels: true, grid: true };
    this.selectedTrackId = null;
    this.hitboxes = [];
  }

  resize() {
    const rect = this.canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = Math.max(1, Math.round(rect.width * dpr));
    this.canvas.height = Math.max(1, Math.round(rect.height * dpr));
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.view.width = rect.width;
    this.view.height = rect.height;
  }

  hitTest(px, py) {
    for (const hit of this.hitboxes) {
      if (px >= hit.x0 && px <= hit.x1 && py >= hit.y0 && py <= hit.y1) return hit.trackId;
    }
    return null;
  }

  draw(scene) {
    const ctx = this.ctx, v = this.view;
    ctx.clearRect(0, 0, v.width, v.height);
    const sky = ctx.createLinearGradient(0, 0, 0, v.height);
    sky.addColorStop(0, "#0b1222"); sky.addColorStop(1, "#070b13");
    ctx.fillStyle = sky; ctx.fillRect(0, 0, v.width, v.height);
    this.hitboxes = [];
    if (this.toggles.grid) this._ground();
    if (!scene) return;
    if (this.toggles.tiles) this._tiles(scene.tiles);
    if (this.toggles.points && scene.points) this._points(scene.points);
    this._ego();
    const sorted = [...scene.objects].sort((a, b) => b.track.position.x - a.track.position.x);
    for (const object of sorted) this._object(object);
    ctx.fillStyle = "#38bdf8"; ctx.font = "10px monospace";
    ctx.fillText("perspective picture: camera behind and above the ego, +X ahead (drag to orbit, wheel to zoom)", 10, v.height - 10);
  }

  _line3(a, b, style, width = 1, dash = []) {
    const pa = this.view.toScreen(...a), pb = this.view.toScreen(...b);
    if (!pa || !pb) return;
    const ctx = this.ctx;
    ctx.strokeStyle = style; ctx.lineWidth = width; ctx.setLineDash(dash);
    ctx.beginPath(); ctx.moveTo(pa[0], pa[1]); ctx.lineTo(pb[0], pb[1]); ctx.stroke(); ctx.setLineDash([]);
  }

  _ground() {
    for (let x = -20; x <= 80; x += 10) this._line3([x, -40, GROUND_Z], [x, 40, GROUND_Z], "rgba(255,255,255,0.05)");
    for (let y = -40; y <= 40; y += 10) this._line3([-20, y, GROUND_Z], [80, y, GROUND_Z], "rgba(255,255,255,0.05)");
  }

  _tiles(tiles) {
    const ctx = this.ctx;
    for (const tile of tiles ?? []) {
      if (String(tile.level).toLowerCase() === "low") continue; // base level: no fill, keeps the picture legible
      const b = tile.bounds;
      const pts = [[b.min_x, b.min_y], [b.max_x, b.min_y], [b.max_x, b.max_y], [b.min_x, b.max_y]]
        .map(([x, y]) => this.view.toScreen(x, y, GROUND_Z));
      if (pts.some((p) => !p)) continue;
      ctx.globalAlpha = 0.45; // lighter on the ground plane so the cloud stays legible
      ctx.fillStyle = resolutionColour(tile.level);
      ctx.beginPath(); pts.forEach((p, i) => (i === 0 ? ctx.moveTo(p[0], p[1]) : ctx.lineTo(p[0], p[1]))); ctx.closePath(); ctx.fill();
      ctx.globalAlpha = 1;
      ctx.strokeStyle = "rgba(56,189,248,0.25)"; ctx.lineWidth = 1; ctx.stroke();
    }
  }

  _points(sample) {
    const ctx = this.ctx;
    const xyz = sample.xyz;
    let minZ = Infinity, maxZ = -Infinity;
    for (const p of xyz) { if (p[2] < minZ) minZ = p[2]; if (p[2] > maxZ) maxZ = p[2]; }
    const span = Math.max(0.5, maxZ - minZ);
    for (const p of xyz) {
      const s = this.view.toScreen(p[0], p[1], p[2]);
      if (!s) continue;
      const [px, py, depth] = s;
      if (px < 0 || py < 0 || px > this.view.width || py > this.view.height) continue;
      ctx.fillStyle = heightColour((p[2] - minZ) / span);
      const size = Math.max(1, Math.min(3, 40 / depth));
      ctx.fillRect(px, py, size, size);
    }
  }

  _ego() {
    const l = 2.35, w = 0.95, z0 = GROUND_Z, z1 = GROUND_Z + 1.5;
    const c = [[-l, -w, z0], [l, -w, z0], [l, w, z0], [-l, w, z0], [-l, -w, z1], [l, -w, z1], [l, w, z1], [-l, w, z1]];
    this._box(c, "#e2e8f0", 1.5);
  }

  _box(c, style, width, dash = []) {
    const edges = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]];
    for (const [a, b] of edges) this._line3(c[a], c[b], style, width, dash);
  }

  _object(object) {
    const ctx = this.ctx, v = this.view;
    const track = object.track;
    const colour = levelColour(riskOf(object));
    const selected = object.trackId === this.selectedTrackId;
    const corners = boxCorners(track);
    this._box(corners, colour, selected ? 2.5 : 1.5, riskOf(object) === "unknown" ? [4, 3] : []);
    const projected = corners.map((c) => v.toScreen(...c)).filter(Boolean);
    if (projected.length) {
      const xs = projected.map((p) => p[0]), ys = projected.map((p) => p[1]);
      const x0 = Math.min(...xs) - 4, x1 = Math.max(...xs) + 4, y0 = Math.min(...ys) - 4, y1 = Math.max(...ys) + 4;
      this.hitboxes.push({ trackId: object.trackId, x0, x1, y0, y1 });
      if (this.toggles.labels) {
        ctx.fillStyle = colour; ctx.font = "10px monospace";
        const level = riskOf(object);
        ctx.fillText(`#${track.track_id} ${track.object_class}${level ? " " + String(level).toUpperCase() : ""}`, x0, y0 - 3);
      }
    }
    if (this.toggles.velocity && track.velocity) {
      const p = track.position;
      this._line3([p.x, p.y, p.z], [p.x + track.velocity.x, p.y + track.velocity.y, p.z + track.velocity.z], "#e2e8f0", 1);
    }
    if (this.toggles.trajectories && object.trajectory) {
      const pts = object.trajectory.points;
      for (let i = 1; i < pts.length; i += 1) {
        const a = pts[i - 1].position, b = pts[i].position;
        this._line3([a.x, a.y, a.z], [b.x, b.y, b.z], "rgba(34,211,238,0.9)", 1.2, [3, 3]);
      }
    }
  }
}
