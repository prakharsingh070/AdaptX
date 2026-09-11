// The screen-space transformation is documented as +X up, +Y left; prove it.
import test from "node:test";
import assert from "node:assert/strict";
import { TopDownView, PerspectiveView, heightColour } from "../render/projection.js";

test("top-down: ahead is up and left is left, ego at the centre", () => {
  const v = new TopDownView({ width: 400, height: 300, metresPerPixel: 0.1, centreX: 0, centreY: 0 });
  const [ex, ey] = v.toScreen(0, 0);
  assert.deepEqual([ex, ey], [200, 150]);
  const [, aheadY] = v.toScreen(10, 0);
  assert.ok(aheadY < ey, "+X must move up the screen");
  const [leftX] = v.toScreen(0, 5);
  assert.ok(leftX < ex, "+Y must move left on the screen");
  const [rightX] = v.toScreen(0, -5);
  assert.ok(rightX > ex);
});

test("top-down: toWorld inverts toScreen", () => {
  const v = new TopDownView({ width: 640, height: 480, metresPerPixel: 0.07, centreX: 12, centreY: -3 });
  const [px, py] = v.toScreen(23.5, -8.25);
  const [x, y] = v.toWorld(px, py);
  assert.ok(Math.abs(x - 23.5) < 1e-9 && Math.abs(y + 8.25) < 1e-9);
});

test("top-down: fit puts the box inside the viewport", () => {
  const v = new TopDownView({ width: 500, height: 500 });
  v.fit(-60, 60, -60, 60, 20);
  const [x0, y0] = v.toScreen(60, 60), [x1, y1] = v.toScreen(-60, -60);
  assert.ok(x0 >= 0 && y0 >= 0 && x1 <= 500 && y1 <= 500);
});

test("perspective: the ego is below the vanishing point and far points rise towards it", () => {
  const v = new PerspectiveView({ width: 800, height: 600 });
  const ego = v.toScreen(0, 0, -1.8), far = v.toScreen(60, 0, -1.8);
  assert.ok(ego && far, "both visible");
  assert.ok(ego[1] > far[1], "further ahead is higher on screen");
  assert.ok(ego[1] > 300 && ego[1] < 600, "ego in the lower half, on screen");
  assert.equal(v.toScreen(-40, 0, 0), null, "behind the camera is culled");
});

test("height colour is a continuous ramp in [0,1] and clamps outside it", () => {
  assert.equal(heightColour(-1), heightColour(0));
  assert.equal(heightColour(2), heightColour(1));
  assert.notEqual(heightColour(0), heightColour(1));
});
