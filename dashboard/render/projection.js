// Screen-space transformations for the ADAPT-X frame (ADR-009): +X forward,
// +Y left, +Z up, metres, ego at the origin.
//
// Top-down: +X maps to screen UP and +Y to screen LEFT, so "ahead" is up and
// "left" is left, exactly as a driver would draw it. This is a presentation
// choice, documented here and in docs/DASHBOARD.md §6; no other transform is
// applied to positions.
//
// Perspective: a fixed pinhole camera behind and above the ego looking along
// +X, with a slight downward pitch. It is a picture, not a sensor model.

export class TopDownView {
  constructor({ width = 800, height = 600, metresPerPixel = 0.1, centreX = 20, centreY = 0 } = {}) {
    this.width = width;
    this.height = height;
    this.metresPerPixel = metresPerPixel; // zoom
    this.centreX = centreX; // world X at the screen centre
    this.centreY = centreY; // world Y at the screen centre
  }

  /** World (x forward, y left) -> screen (px, py). */
  toScreen(x, y) {
    const px = this.width / 2 - (y - this.centreY) / this.metresPerPixel;
    const py = this.height / 2 - (x - this.centreX) / this.metresPerPixel;
    return [px, py];
  }

  /** Screen -> world. */
  toWorld(px, py) {
    const y = this.centreY - (px - this.width / 2) * this.metresPerPixel;
    const x = this.centreX - (py - this.height / 2) * this.metresPerPixel;
    return [x, y];
  }

  zoomAt(px, py, factor) {
    const [wx, wy] = this.toWorld(px, py);
    this.metresPerPixel = Math.min(2, Math.max(0.01, this.metresPerPixel * factor));
    const [nx, ny] = this.toWorld(px, py);
    this.centreX += wx - nx;
    this.centreY += wy - ny;
  }

  panPixels(dx, dy) {
    this.centreY += dx * this.metresPerPixel;
    this.centreX += dy * this.metresPerPixel;
  }

  /** Fit a world box (minX..maxX, minY..maxY) into the viewport with a margin. */
  fit(minX, maxX, minY, maxY, marginPx = 20) {
    const spanX = Math.max(1e-3, maxX - minX);
    const spanY = Math.max(1e-3, maxY - minY);
    const mppY = spanY / Math.max(1, this.width - 2 * marginPx);
    const mppX = spanX / Math.max(1, this.height - 2 * marginPx);
    this.metresPerPixel = Math.max(mppX, mppY);
    this.centreX = (minX + maxX) / 2;
    this.centreY = (minY + maxY) / 2;
  }
}

export class PerspectiveView {
  constructor({ width = 800, height = 600 } = {}) {
    this.width = width;
    this.height = height;
    // Camera in the ADAPT-X frame: behind (-x), above (+z), looking along +x.
    this.camera = { x: -14, y: 0, z: 9 };
    this.pitchRad = 0.45; // positive tilts the view down towards the road
    this.yawRad = 0; // orbit around the ego, radians
    this.focalPx = 520;
  }

  /** World -> screen; returns null when the point is behind the camera. */
  toScreen(x, y, z) {
    // Rotate about the ego by yaw so the camera can orbit.
    const cy = Math.cos(this.yawRad), sy = Math.sin(this.yawRad);
    const rx = x * cy - y * sy;
    const ry = x * sy + y * cy;
    // Camera-relative vector: forward = +x, left = +y, up = +z.
    let fx = rx - this.camera.x;
    const fy = ry - this.camera.y;
    let fz = z - this.camera.z;
    // Pitch about the camera's left axis.
    const cp = Math.cos(this.pitchRad), sp = Math.sin(this.pitchRad);
    const dx = fx * cp - fz * sp;
    const dz = fx * sp + fz * cp;
    fx = dx; fz = dz;
    if (fx <= 0.2) return null;
    const px = this.width / 2 - (fy / fx) * this.focalPx;
    const py = this.height / 2 - (fz / fx) * this.focalPx;
    return [px, py, fx];
  }

  orbit(dYaw) {
    this.yawRad += dYaw;
  }

  zoom(factor) {
    this.focalPx = Math.min(2000, Math.max(150, this.focalPx * factor));
  }
}

/** Turbo-like colour ramp for heights; t in [0,1]. */
export function heightColour(t) {
  const clamped = Math.min(1, Math.max(0, t));
  const stops = [
    [48, 18, 59], [70, 107, 227], [39, 200, 205], [95, 240, 80], [250, 200, 20], [240, 80, 20],
  ];
  const scaled = clamped * (stops.length - 1);
  const i = Math.min(stops.length - 2, Math.floor(scaled));
  const f = scaled - i;
  const a = stops[i], b = stops[i + 1];
  const r = Math.round(a[0] + (b[0] - a[0]) * f);
  const g = Math.round(a[1] + (b[1] - a[1]) * f);
  const bl = Math.round(a[2] + (b[2] - a[2]) * f);
  return `rgb(${r},${g},${bl})`;
}
