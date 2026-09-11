// REST access to the existing backend. Every function returns the parsed JSON
// exactly as the backend produced it, or throws an ApiError with the backend's
// own error envelope when it sent one. No caching, no reshaping.

export class ApiError extends Error {
  constructor(message, { status = 0, code = "network", details = null } = {}) {
    super(message);
    this.name = "ApiError";
    this.status = status;
    this.code = code;
    this.details = details;
  }
}

export function apiBase() {
  // Served from the same origin at /dashboard/; the API lives at the root.
  return `${window.location.origin}`;
}

export function wsBase() {
  const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
  return `${protocol}//${window.location.host}`;
}

// One fetch site. GET for everything the dashboard reads; POST only for the
// five high-level live-session controls (start / pause / resume / stop /
// reset), which the backend allowlists and audits. Nothing here can spawn,
// move, tick or drive anything.
async function request(path, { method = "GET", body = null } = {}) {
  let response;
  try {
    const init = { method, headers: { Accept: "application/json" } };
    if (body !== null) {
      init.headers["Content-Type"] = "application/json";
      init.body = JSON.stringify(body);
    }
    response = await fetch(`${apiBase()}${path}`, init);
  } catch (error) {
    throw new ApiError(`backend unreachable: ${error.message}`);
  }
  let payload = null;
  try {
    payload = await response.json();
  } catch {
    payload = null;
  }
  if (!response.ok) {
    const envelope = payload && payload.error ? payload.error : null;
    throw new ApiError(
      envelope ? envelope.message : `${response.status} ${response.statusText}`,
      { status: response.status, code: envelope ? envelope.code : "http", details: envelope ? envelope.details : payload },
    );
  }
  return payload;
}

const getJson = (path) => request(path);
const postJson = (path, body = {}) => request(path, { method: "POST", body });

export const api = {
  health: () => getJson("/health"),
  systemStatus: () => getJson("/api/v1/system/status"),
  systemMetrics: () => getJson("/api/v1/system/metrics"),
  carlaStatus: () => getJson("/api/v1/carla/status"),
  mapStatus: () => getJson("/api/v1/map/status"),
  riskStatus: () => getJson("/api/v1/risk/status"),
  sceneLatest: () => getJson("/api/v1/scene/latest"),
  reports: () => getJson("/api/v1/reports"),
  report: (name) => getJson(`/api/v1/reports/${encodeURIComponent(name)}`),
  compareReports: (first, second) =>
    getJson(`/api/v1/reports/compare?first=${encodeURIComponent(first)}&second=${encodeURIComponent(second)}`),
  runs: () => getJson("/api/v1/runs"),
  run: (name) => getJson(`/api/v1/runs/${encodeURIComponent(name)}`),
  runFrame: (name, index) => getJson(`/api/v1/runs/${encodeURIComponent(name)}/frames/${index}`),
  liveStatus: () => getJson("/api/v1/live/status"),
  liveScenarios: () => getJson("/api/v1/live/scenarios"),
  liveEvents: (since) => getJson(`/api/v1/live/events?since=${since}&limit=200`),
  liveStart: (scenarioId, seed) => postJson("/api/v1/live/start", { scenario_id: scenarioId, seed }),
  livePause: () => postJson("/api/v1/live/pause"),
  liveResume: () => postJson("/api/v1/live/resume"),
  liveStop: () => postJson("/api/v1/live/stop"),
  liveReset: () => postJson("/api/v1/live/reset"),
  /** URL of the newest ego camera frame; the query defeats caching per frame. */
  liveCameraUrl: (frameId) => `${apiBase()}/api/v1/live/camera?f=${frameId}`,
};
