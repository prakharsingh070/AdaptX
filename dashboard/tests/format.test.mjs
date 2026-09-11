// Formatting is the only place a missing value becomes text. It must never
// become a number, and UNKNOWN must never become LOW.
import test from "node:test";
import assert from "node:assert/strict";
import {
  fmt, fmtInt, fmtPercent, fmtBytesMB, fmtMs, fmtDistribution, levelLabel, levelClass, levelColour,
  isMissing, NOT_AVAILABLE,
} from "../data/format.js";

test("null and undefined render as Not available, never 0", () => {
  assert.equal(fmt(null), NOT_AVAILABLE);
  assert.equal(fmt(undefined), NOT_AVAILABLE);
  assert.equal(fmtInt(null), NOT_AVAILABLE);
  assert.equal(fmtPercent(null), NOT_AVAILABLE);
  assert.equal(fmtBytesMB(null), NOT_AVAILABLE);
  assert.equal(fmtMs(null), NOT_AVAILABLE);
  assert.notEqual(fmt(null), "0");
  assert.notEqual(fmt(null), "0.00");
});

test("NaN is missing too", () => {
  assert.equal(isMissing(Number.NaN), true);
  assert.equal(fmt(Number.NaN), NOT_AVAILABLE);
});

test("a real zero stays a zero", () => {
  assert.equal(fmt(0), "0.00");
  assert.equal(fmtInt(0), "0");
  assert.equal(fmtPercent(0), "0.0%");
});

test("units are appended and digits respected", () => {
  assert.equal(fmt(1.23456, { unit: " m", digits: 3 }), "1.235 m");
  assert.equal(fmtMs(12.34), "12.3 ms");
  assert.equal(fmtBytesMB(1048576), "1.00 MB");
});

test("UNKNOWN risk stays UNKNOWN and is styled apart from LOW", () => {
  assert.equal(levelLabel("unknown"), "UNKNOWN");
  assert.equal(levelClass("unknown"), "lvl lvl-unknown");
  assert.notEqual(levelColour("unknown"), levelColour("low"));
  assert.equal(levelLabel(null), NOT_AVAILABLE);
  assert.equal(levelClass(null), "lvl lvl-unknown");
});

test("an empty distribution says so instead of printing zeros", () => {
  assert.match(fmtDistribution({ count: 0 }), /Not available/);
  assert.match(fmtDistribution(null), /Not available/);
  const text = fmtDistribution({ count: 3, mean: 1, median: 1, p95: null, max: 2 }, " m");
  assert.match(text, /n=3/);
  assert.match(text, /p95 n\/a/);
});
