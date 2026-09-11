// View models. Reshape backend contracts for rendering and nothing else:
// join a track with its assessment and trajectory by track_id, keep every
// field as received, and never derive a value the backend did not send.
//
// A scene view model has the same shape whether it came from the live
// channel (SceneSnapshot) or from a recorded run frame (ScenarioFrameRecord
// with PipelineFrameOutputs), so the renderers do not know which mode is on.

/** Build a scene view model from a live SceneSnapshot. */
export function sceneFromSnapshot(snapshot) {
  if (!snapshot) return null;
  return buildScene({
    mode: "live",
    frameId: snapshot.frame_id,
    frameTimestamp: snapshot.frame_timestamp,
    scenarioTimeS: snapshot.scenario_time_s ?? null,
    frameIndex: snapshot.frame_index ?? null,
    source: snapshot.source,
    origin: snapshot.origin,
    scenarioId: snapshot.scenario_id ?? null,
    points: snapshot.points ?? null,
    detectionCount: snapshot.detection_count,
    tracks: snapshot.tracks ?? [],
    trajectories: snapshot.trajectories ?? [],
    assessments: snapshot.assessments ?? [],
    highestRiskLevel: snapshot.highest_risk_level,
    tiles: snapshot.tiles ?? [],
    budget: snapshot.budget ?? null,
    fixedMap: snapshot.fixed_map ?? null,
    adaptiveMap: snapshot.adaptive_map ?? null,
    comparison: snapshot.comparison ?? null,
    stageMs: snapshot.stage_ms ?? {},
    // Live-session fields (post-Phase-12): the ego's own odometry, the
    // baseline controller's command and the loop's measured timing. Absent
    // on snapshots from the endpoints or a scenario publisher.
    ego: snapshot.ego ?? null,
    control: snapshot.control ?? null,
    live: snapshot.live ?? null,
    groundTruth: null,
  });
}

/** Build a scene view model from one frame of a recorded run (playback). */
export function sceneFromRunFrame(runName, runSummary, runFrame) {
  if (!runFrame || !runFrame.frame) return null;
  const record = runFrame.frame;
  const outputs = record.outputs;
  if (!outputs) {
    return buildScene({
      mode: "playback",
      frameId: record.simulator_frame_id,
      frameTimestamp: record.timestamp,
      scenarioTimeS: record.scenario_time_s,
      frameIndex: record.frame_index,
      source: "simulation",
      origin: `run:${runName}`,
      scenarioId: runSummary ? runSummary.scenario_id : null,
      points: null,
      detectionCount: record.pipeline ? record.pipeline.detections : null,
      tracks: [],
      trajectories: [],
      assessments: [],
      highestRiskLevel: record.pipeline ? record.pipeline.risk_level : null,
      tiles: [],
      budget: null,
      fixedMap: null,
      adaptiveMap: null,
      comparison: null,
      stageMs: record.pipeline ? record.pipeline.stage_ms : {},
      ego: null,
      control: null,
      live: null,
      groundTruth: runFrame.ground_truth ?? null,
      outputsMissing: true,
    });
  }
  return buildScene({
    mode: "playback",
    frameId: record.simulator_frame_id,
    frameTimestamp: record.timestamp,
    scenarioTimeS: record.scenario_time_s,
    frameIndex: record.frame_index,
    source: "simulation",
    origin: `run:${runName}`,
    scenarioId: runSummary ? runSummary.scenario_id : null,
    points: null,
    detectionCount: outputs.detection.objects.length,
    tracks: outputs.tracking.tracks,
    trajectories: outputs.prediction.trajectories,
    assessments: outputs.risk.assessments,
    highestRiskLevel: record.pipeline ? record.pipeline.risk_level : null,
    tiles: outputs.plan.decisions,
    budget: outputs.plan.budget,
    fixedMap: outputs.fixed_map,
    adaptiveMap: outputs.adaptive_map,
    comparison: outputs.comparison,
    stageMs: record.pipeline ? record.pipeline.stage_ms : {},
    ego: null,
    control: null,
    live: null,
    groundTruth: runFrame.ground_truth ?? null,
    expectedPoses: record.expected_poses ?? [],
  });
}

function buildScene(fields) {
  const trajectoriesById = new Map(fields.trajectories.map((t) => [t.track_id, t]));
  const assessmentsById = new Map(fields.assessments.map((a) => [a.track_id, a]));
  const objects = fields.tracks.map((track) => ({
    trackId: track.track_id,
    track,
    trajectory: trajectoriesById.get(track.track_id) ?? null,
    assessment: assessmentsById.get(track.track_id) ?? null,
  }));
  return { ...fields, objects, riskCounts: countLevels(fields.assessments) };
}

/** Assessments per level. UNKNOWN is counted as UNKNOWN. */
export function countLevels(assessments) {
  const counts = { low: 0, medium: 0, high: 0, critical: 0, unknown: 0 };
  for (const a of assessments ?? []) {
    const key = String(a.risk_level ?? "unknown").toLowerCase();
    counts[key in counts ? key : "unknown"] += 1;
  }
  return counts;
}

/** Tracks per class, straight from the class field. */
export function countClasses(tracks) {
  const counts = {};
  for (const t of tracks ?? []) {
    const key = String(t.object_class ?? "unknown");
    counts[key] = (counts[key] ?? 0) + 1;
  }
  return counts;
}

/** The selected object, by stable track id, or null. */
export function findObject(scene, trackId) {
  if (!scene || trackId === null || trackId === undefined) return null;
  return scene.objects.find((o) => o.trackId === trackId) ?? null;
}

/** Bounds of a set of tile decisions, or null. */
export function tileBounds(tiles) {
  if (!tiles || tiles.length === 0) return null;
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (const t of tiles) {
    minX = Math.min(minX, t.bounds.min_x); maxX = Math.max(maxX, t.bounds.max_x);
    minY = Math.min(minY, t.bounds.min_y); maxY = Math.max(maxY, t.bounds.max_y);
  }
  return { minX, maxX, minY, maxY };
}

/** Read-only wrapper: a report as loaded, with its sections indexed by name. */
export function reportModel(report) {
  if (!report) return null;
  const sections = {
    detection: report.detection,
    tracking: report.tracking,
    prediction: report.prediction,
    risk: report.risk,
    mapping: report.mapping,
    adaptive_resolution: report.adaptive_resolution,
    resource: report.resource,
  };
  return { report, sections, primaryGate: report.configuration.primary_gate_m, gates: report.configuration.match_gates_m };
}

/** Gate metrics for one gate value from a detection/tracking section. */
export function gateAt(section, gate) {
  if (!section || !section.gates) return null;
  return section.gates.find((g) => g.gate_m === gate) ?? null;
}
