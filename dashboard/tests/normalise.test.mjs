// View models reshape backend contracts and derive nothing.
import test from "node:test";
import assert from "node:assert/strict";
import { sceneFromSnapshot, sceneFromRunFrame, countLevels, findObject, gateAt } from "../data/normalise.js";
import { deepFreeze } from "../data/evidence.js";

const track = (id, extra = {}) => ({ track_id: id, object_class: "vehicle", status: "confirmed", position: { x: 10, y: 0, z: 0 }, velocity: null, ...extra });
const assessment = (id, level, score) => ({ track_id: id, risk_level: level, risk_score: score });
const snapshot = {
  frame_id: 7, frame_timestamp: "2000-01-01T00:00:00Z", scenario_time_s: 0.35, frame_index: 7, source: "simulation",
  origin: "scenario:x", scenario_id: "x", points: { total_count: 100, sample_count: 10, is_downsampled: true, stride: 10, stage: "raw", xyz: [] },
  detection_count: 3, tracks: [track(1), track(2, { velocity: { x: 1, y: 0, z: 0 } })],
  trajectories: [{ track_id: 2, points: [] }],
  assessments: [assessment(1, "unknown", null), assessment(2, "high", 0.7)],
  highest_risk_level: "high", tiles: [], budget: null, fixed_map: null, adaptive_map: null, comparison: null, stage_ms: { detection: 1 },
};

test("a live snapshot joins tracks with their assessment and path by track id", () => {
  const scene = sceneFromSnapshot(snapshot);
  assert.equal(scene.mode, "live");
  assert.equal(scene.objects.length, 2);
  const one = findObject(scene, 1), two = findObject(scene, 2);
  assert.equal(one.assessment.risk_level, "unknown");
  assert.equal(one.assessment.risk_score, null); // stays null
  assert.equal(one.trajectory, null); // no path: stays absent
  assert.equal(two.trajectory.track_id, 2);
  assert.equal(one.track.velocity, null); // null velocity stays null
  assert.equal(scene.groundTruth, null); // never in a live scene
});

test("risk counts keep UNKNOWN as its own bucket", () => {
  assert.deepEqual(countLevels(snapshot.assessments), { low: 0, medium: 0, high: 1, critical: 0, unknown: 1 });
  assert.deepEqual(countLevels([{ risk_level: "bogus" }]), { low: 0, medium: 0, high: 0, critical: 0, unknown: 1 });
});

test("selection is by stable track id, not array index", () => {
  const scene = sceneFromSnapshot({ ...snapshot, tracks: [track(42), track(1)] });
  assert.equal(findObject(scene, 1).trackId, 1);
  assert.equal(findObject(scene, 0), null);
  assert.equal(findObject(scene, null), null);
});

test("a recorded run frame becomes a playback scene with ground truth attached and labelled", () => {
  const frame = {
    frame: {
      frame_index: 3, scenario_time_s: 0.15, simulator_frame_id: 1003, timestamp: "2000-01-01T00:00:00.15Z", point_count: 50,
      expected_poses: [{ actor_id: "a", position: { x: 20, y: 3.5, z: 0 } }], ground_truth_actor_count: 1,
      pipeline: { detections: 1, risk_level: "medium", stage_ms: { detection: 2 } },
      outputs: {
        detection: { objects: [{}] }, tracking: { tracks: [track(5)] }, prediction: { trajectories: [] },
        risk: { assessments: [assessment(5, "medium", 0.5)] }, fixed_map: { a: 1 }, adaptive_map: { b: 2 },
        plan: { decisions: [], budget: { within_budget: true } }, comparison: null,
      },
    },
    ground_truth: { actors: [{ actor_id: 9, is_ego: false }] },
  };
  const scene = sceneFromRunFrame("run.json", { scenario_id: "s" }, frame);
  assert.equal(scene.mode, "playback");
  assert.equal(scene.points, null); // records carry no cloud
  assert.equal(scene.objects[0].assessment.risk_score, 0.5);
  assert.deepEqual(scene.groundTruth, frame.ground_truth);
  assert.equal(scene.expectedPoses.length, 1);
});

test("a frame without outputs is a scene with nothing to draw, not a crash", () => {
  const scene = sceneFromRunFrame("r", null, { frame: { frame_index: 0, scenario_time_s: 0, simulator_frame_id: 1, timestamp: "t", pipeline: null, outputs: null }, ground_truth: null });
  assert.equal(scene.outputsMissing, true);
  assert.equal(scene.objects.length, 0);
  assert.equal(scene.detectionCount, null);
});

test("view models never mutate a frozen report or run", () => {
  const frozen = deepFreeze(JSON.parse(JSON.stringify(snapshot)));
  const scene = sceneFromSnapshot(frozen);
  assert.throws(() => { frozen.tracks[0].position.x = 99; });
  assert.equal(scene.tracks[0].position.x, 10);
  assert.equal(gateAt({ gates: [{ gate_m: 2, x: 1 }] }, 2).x, 1);
  assert.equal(gateAt({ gates: [] }, 2), null);
});

test("a live-session snapshot carries ego, control and timing through unchanged; playback carries none", () => {
  const live = {
    ...snapshot, origin: "live:static_obstacle", scenario_id: "static_obstacle",
    ego: { position: { x: 1, y: 2, z: 0 }, velocity: { x: 3, y: 0, z: 0 }, heading_rad: 0, source: "simulation" },
    control: { throttle: 0.2, brake: 0, steer: 0, target_speed_mps: 8, setpoint_mps: 3.1, state: "CRUISING", governing_level: "low", scene_level: "high", reason: "r", is_baseline: true },
    live: { scenario_id: "static_obstacle", seed: 42, simulation_frame: 1, simulation_time_s: 0.05, session_time_s: 0.05, controller_state: "CRUISING", collision_count: 0, active_actor_count: 0, traffic_count: 0, timing: { step_ms: 30, pipeline_ms: 120, control_ms: 1, snapshot_ms: 6, loop_ms: 160, fixed_delta_s: 0.05, realtime_factor: 0.31, lagging: true, frames_processed: 1 } },
  };
  const scene = sceneFromSnapshot(live);
  assert.equal(scene.mode, "live");
  assert.equal(scene.ego.source, "simulation");
  assert.equal(scene.control.setpoint_mps, 3.1); // reported, not recomputed
  assert.equal(scene.live.timing.lagging, true);  // the backend's measurement, as received
  assert.equal(scene.groundTruth, null);
  const playback = sceneFromRunFrame("r", null, { frame: { frame_index: 0, scenario_time_s: 0, simulator_frame_id: 1, timestamp: "t", pipeline: null, outputs: null }, ground_truth: null });
  assert.equal(playback.ego, null);
  assert.equal(playback.control, null);
  assert.equal(playback.live, null);
});
