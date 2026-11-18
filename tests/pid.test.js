import test from "node:test";
import assert from "node:assert/strict";
import {
  Pid,
  derivativeFilterTime,
  idealGains,
  parallelGains,
  pidSettings,
  trackingTime,
} from "../src/core/pid.js";
import { defaultSettings, evaluate, makeConfig } from "../src/core/lab.js";
import { Simulation } from "../src/core/sim.js";
import { createRng } from "../src/core/rng.js";
import { near } from "./helpers.js";

test("proportional-only output is Kp (b r - y)", () => {
  const pid = new Pid({ kp: 2, b: 0.5, sampleTime: 0.1 });
  pid.reset(0, 0, 0);
  near(pid.step(4, 1), 2 * (0.5 * 4 - 1), 1e-12);
});

test("integral accumulates Ki T e, applied from the next sample", () => {
  const pid = new Pid({ kp: 0, ki: 3, sampleTime: 0.1, antiWindup: "none" });
  pid.reset(0, 0, 0);
  const u = [1, 2, 3].map(() => pid.step(1, 0));
  near(u[0], 0, 1e-12);
  near(u[1], 0.3, 1e-12);
  near(u[2], 0.6, 1e-12);
});

test("filtered derivative responds to a measurement step as Kd/(Tf+T) a^k", () => {
  const T = 0.01;
  const pid = new Pid({ kp: 1, kd: 0.5, n: 10, sampleTime: T, c: 0 });
  const tf = 0.05; // Td / N = (0.5 / 1) / 10
  near(derivativeFilterTime(pid.s), tf, 1e-15);
  pid.reset(0, 0, 0);
  const a = tf / (tf + T);
  const u0 = pid.step(0, 1); // y jumps from 0 to 1
  near(u0 - -1, (-0.5 / (tf + T)) * 1, 1e-12);
  const u1 = pid.step(0, 1);
  near(u1 - -1, (-0.5 / (tf + T)) * a, 1e-12);
});

test("without a filter the derivative is a plain backward difference", () => {
  const pid = new Pid({ kp: 0, kd: 0.2, n: 0, sampleTime: 0.1 });
  pid.reset(0, 0, 0);
  near(pid.step(0, -1), 0.2 * (1 / 0.1), 1e-12);
  assert.equal(derivativeFilterTime(pid.s), 0);
  // Kp = 0 in parallel form: Tf = Kd / N.
  near(derivativeFilterTime(pidSettings({ kp: 0, kd: 0.2, n: 4 })), 0.05, 1e-15);
});

test("setpoint weight c = 0 removes the derivative kick, c = 1 keeps it", () => {
  for (const c of [0, 1]) {
    const pid = new Pid({ kp: 1, kd: 1, n: 10, c, sampleTime: 0.01 });
    pid.reset(0, 0, 0);
    const u = pid.step(1, 0);
    if (c === 0) near(u, 1, 1e-12);
    else assert.ok(u > 10, `derivative kick ${u}`);
  }
});

test("ideal and parallel forms give identical outputs", () => {
  const ideal = new Pid({ form: "ideal", kc: 2, ti: 4, td: 0.5, n: 8, sampleTime: 0.05 });
  const g = parallelGains(ideal.s);
  assert.deepEqual(g, { kp: 2, ki: 0.5, kd: 1 });
  const par = new Pid({ form: "parallel", ...g, n: 8, sampleTime: 0.05 });
  ideal.reset(0, 0, 0);
  par.reset(0, 0, 0);
  const rng = createRng(3);
  for (let k = 0; k < 50; k++) {
    const r = k > 5 ? 1 : 0;
    const y = rng.next();
    near(ideal.step(r, y), par.step(r, y), 1e-12);
  }
  assert.deepEqual(idealGains(g), { kc: 2, ti: 4, td: 0.5 });
  assert.deepEqual(idealGains({ kp: 0, ki: 0, kd: 0 }), { kc: 0, ti: 0, td: 0 });
  assert.ok(Number.isNaN(idealGains({ kp: 0, ki: 1, kd: 1 }).td));
  assert.equal(parallelGains(pidSettings({ form: "ideal", kc: 3, ti: 0, td: 0 })).ki, 0);
});

test("output and rate limits are respected", () => {
  const pid = new Pid({ kp: 100, uMin: -1, uMax: 2, rateLimit: 5, sampleTime: 0.1 });
  pid.reset(0, 0, 0);
  const u = [];
  for (let k = 0; k < 10; k++) u.push(pid.step(1, 0));
  assert.deepEqual(
    u.map((v) => Number(v.toFixed(12))),
    [0.5, 1, 1.5, 2, 2, 2, 2, 2, 2, 2],
  );
  assert.ok(pid.terms.saturated);
  const neg = new Pid({ kp: 100, uMin: -1, uMax: 2, sampleTime: 0.1 });
  neg.reset(0, 0, 0);
  assert.equal(neg.step(-1, 0), -1);
});

test("tracking time defaults to sqrt(Ti Td), or Ti/2 for PI", () => {
  near(trackingTime(pidSettings({ kp: 2, ki: 0.5, kd: 1 })), Math.sqrt(4 * 0.5), 1e-12);
  near(trackingTime(pidSettings({ kp: 2, ki: 0.5 })), 2, 1e-12);
  near(trackingTime(pidSettings({ kp: 0, ki: 0.5 })), 2, 1e-12);
  assert.equal(trackingTime(pidSettings({ kp: 2 })), Infinity);
  assert.equal(trackingTime(pidSettings({ kp: 2, ki: 1, tt: 0.3 })), 0.3);
});

test("settings are validated", () => {
  assert.throws(() => pidSettings({ sampleTime: 0 }), /sampleTime/);
  assert.throws(() => pidSettings({ uMin: 1, uMax: 1 }), /uMax/);
  // @ts-expect-error invalid on purpose
  assert.throws(() => pidSettings({ form: "series" }), /form/);
  // @ts-expect-error invalid on purpose
  assert.throws(() => pidSettings({ antiWindup: "magic" }), /anti-windup/);
});

/**
 * FOPDT with a setpoint that needs 0.5 of the 0.6 available output, and an
 * aggressive integral, so the output saturates during the step.
 * @param {"none" | "clamp" | "backcalc"} aw
 */
function saturatedStep(aw) {
  return evaluate({
    ...defaultSettings("fopdt"),
    kp: 1,
    ki: 0.6,
    kd: 0,
    uMin: 0,
    uMax: 0.6,
    antiWindup: aw,
    disturbance: 0,
  }).step;
}

test("anti-windup reduces overshoot under saturation", () => {
  const none = saturatedStep("none");
  const clamp = saturatedStep("clamp");
  const back = saturatedStep("backcalc");
  assert.ok(none.saturationTime > 2, `saturated for ${none.saturationTime} s`);
  assert.ok(none.overshoot > 15, `overshoot without anti-windup ${none.overshoot}`);
  assert.ok(clamp.overshoot < none.overshoot / 2, `clamp ${clamp.overshoot} vs ${none.overshoot}`);
  assert.ok(back.overshoot < none.overshoot / 2, `backcalc ${back.overshoot} vs ${none.overshoot}`);
  assert.ok(clamp.iae < none.iae && back.iae < none.iae);
});

test("derivative filter attenuates measurement noise in the control signal", () => {
  /** @param {number} n */
  const tv = (n) =>
    evaluate({
      ...defaultSettings("msd"),
      n,
      noiseStd: 0.002,
      disturbance: 0,
    }).step.totalVariation;
  const unfiltered = tv(0);
  const n20 = tv(20);
  const n5 = tv(5);
  assert.ok(n20 < unfiltered / 2, `N=20 ${n20} vs unfiltered ${unfiltered}`);
  assert.ok(n5 < n20, `N=5 ${n5} vs N=20 ${n20}`);
});

test("manual/auto transfer is bumpless in both directions", () => {
  const { config } = makeConfig({ ...defaultSettings("fopdt"), noiseStd: 0 });
  const sim = new Simulation(config);
  sim.r = 1;
  const u = [];
  for (let k = 0; k < 2000; k++) {
    if (k === 600) sim.pid.setManual(true);
    if (k === 700) sim.pid.manualOutput = 0.2; // operator moves the output
    if (k === 1400) sim.pid.setManual(false);
    u.push(sim.step().u);
  }
  // Entering manual: the output stays where it was.
  near(u[600], u[599], 1e-9, "auto -> manual");
  // Leaving manual: the integral has been tracking, so the first automatic
  // output continues from the manual value (only one sample of P change).
  const jump = Math.abs(u[1400] - u[1399]);
  assert.ok(jump < 0.01, `jump at manual -> auto: ${jump}`);
  // For comparison: without tracking the output would jump to the stale value.
  assert.ok(Math.abs(u[1399] - 0.2) < 1e-12);
});

test("changing gains on the fly does not bump the output", () => {
  const pid = new Pid({ kp: 1, ki: 1, sampleTime: 0.1 });
  pid.reset(1, 0.5, 0.3);
  const before = pid.step(1, 0.5);
  pid.update({ kp: 3, b: 0.5 }, 1, 0.5);
  const after = pid.step(1, 0.5);
  // Only the integral increment Ki T e = 0.05 separates the two samples.
  near(after - before, 0.05, 1e-12);
});

test("reset starts in steady state with the requested output", () => {
  const pid = new Pid({ kp: 2, ki: 1, kd: 1, sampleTime: 0.1, uMin: 0, uMax: 1 });
  pid.reset(3, 3, 0.4, 0.1);
  near(pid.step(3, 3, 0.1), 0.4, 1e-12);
  // Out-of-range initial outputs are clamped.
  pid.reset(0, 0, 5);
  assert.equal(pid.uPrev, 1);
});
