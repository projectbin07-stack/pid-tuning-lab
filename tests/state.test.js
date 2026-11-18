import test from "node:test";
import assert from "node:assert/strict";
import {
  decodeState,
  encodeState,
  formatValue,
  fromSlider,
  gainRange,
  toSlider,
} from "../src/ui/state.js";
import { defaultSettings } from "../src/core/lab.js";

test("defaults encode to just the plant", () => {
  const qs = encodeState({ a: defaultSettings("motor"), b: null, run: 0, paused: false });
  assert.equal(qs, "plant=motor");
  const back = decodeState(qs);
  assert.deepEqual(back.a, defaultSettings("motor"));
  assert.equal(back.b, null);
  assert.equal(back.run, 0);
  assert.equal(back.paused, false);
});

test("settings, plant parameters and the comparison round-trip through the URL", () => {
  const a = {
    ...defaultSettings("thermal"),
    form: /** @type {const} */ ("ideal"),
    kc: 0.1,
    ti: 30,
    td: 2,
    antiWindup: /** @type {const} */ ("clamp"),
    noiseStd: 0.3,
    params: { Pmax: 60 },
  };
  const b = { ...defaultSettings("thermal"), kp: 0.2, params: { Pmax: 60 } };
  const qs = encodeState({ a, b, run: 500, paused: true });
  assert.match(qs, /form=ideal/);
  assert.match(qs, /B\.kp=0\.2/);
  assert.match(qs, /p\.Pmax=60/);
  const s = decodeState("?" + qs);
  assert.deepEqual(s.a, a);
  assert.deepEqual(s.b, b);
  assert.equal(s.run, 500);
  assert.equal(s.paused, true);
});

test("invalid URL values fall back to defaults", () => {
  const s = decodeState(
    "plant=rocket&kp=abc&ki=&aw=magic&form=series&run=-5&p.K=1e9&p.nope=3&ts=0.000001",
  );
  assert.deepEqual(s.a, defaultSettings("fopdt"));
  assert.equal(s.run, 0);
  const limits = decodeState("plant=fopdt&umin=5&umax=1");
  assert.equal(limits.a.uMin, defaultSettings("fopdt").uMin);
  assert.equal(limits.a.uMax, defaultSettings("fopdt").uMax);
  assert.equal(decodeState("run=1.5").run, 0);
  assert.equal(decodeState("run=99999999").run, 200000);
});

test("logarithmic slider mapping", () => {
  assert.equal(toSlider(0, 0.01, 100), 0);
  assert.equal(toSlider(1, 0.01, 100), 500);
  assert.equal(toSlider(1000, 0.01, 100), 1000);
  assert.equal(fromSlider(0, 0.01, 100), 0);
  assert.equal(fromSlider(500, 0.01, 100), 1);
  assert.equal(fromSlider(1000, 0.01, 100), 100);
  assert.deepEqual(gainRange(2, 5), [0.02, 200]);
  assert.deepEqual(gainRange(0, 5), [0.05, 500]);
});

test("metric formatting", () => {
  assert.equal(formatValue(NaN), "n/a");
  assert.equal(formatValue(Infinity), "inf");
  assert.equal(formatValue(0.12345, "s"), "0.123 s");
  assert.equal(formatValue(0), "0");
  assert.equal(formatValue(123456), "1.23e+5");
  assert.equal(formatValue(0.0001234), "1.23e-4");
});
