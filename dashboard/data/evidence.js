// Stored evidence: evaluation reports and recorded runs, from the backend's
// read-only endpoints. A loaded report is frozen so no view can mutate it;
// a run is fetched one frame at a time and cached by index.

import { api } from "./api.js";

export function deepFreeze(value) {
  if (value && typeof value === "object" && !Object.isFrozen(value)) {
    Object.freeze(value);
    for (const key of Object.keys(value)) deepFreeze(value[key]);
  }
  return value;
}

export class ReportStore {
  constructor() {
    this.name = null;
    this.report = null;
    this.error = null;
  }

  async list() {
    return api.reports();
  }

  async load(name) {
    this.error = null;
    try {
      const report = await api.report(name);
      this.name = name;
      this.report = deepFreeze(report);
    } catch (error) {
      this.name = name;
      this.report = null;
      this.error = error;
    }
    return this.report;
  }

  clear() {
    this.name = null;
    this.report = null;
    this.error = null;
  }
}

export class RunStore {
  constructor({ prefetch = 8 } = {}) {
    this.name = null;
    this.summary = null;
    this.frames = new Map();
    this.error = null;
    this.prefetch = prefetch;
  }

  async list() {
    return api.runs();
  }

  async load(name) {
    this.error = null;
    this.frames = new Map();
    try {
      const summary = await api.run(name);
      this.name = name;
      this.summary = deepFreeze(summary);
    } catch (error) {
      this.name = name;
      this.summary = null;
      this.error = error;
    }
    return this.summary;
  }

  clear() {
    this.name = null;
    this.summary = null;
    this.frames = new Map();
    this.error = null;
  }

  get frameCount() {
    return this.summary ? this.summary.frame_count : 0;
  }

  async frame(index) {
    if (!this.summary) return null;
    if (index < 0 || index >= this.frameCount) return null;
    const cached = this.frames.get(index);
    if (cached) return cached;
    const frame = deepFreeze(await api.runFrame(this.name, index));
    this.frames.set(index, frame);
    // Fetch a few frames ahead so playback does not stall on every step.
    for (let ahead = 1; ahead <= this.prefetch; ahead += 1) {
      const next = index + ahead;
      if (next < this.frameCount && !this.frames.has(next)) {
        api.runFrame(this.name, next).then((f) => this.frames.set(next, deepFreeze(f))).catch(() => {});
      }
    }
    return frame;
  }
}
