import test from "node:test";
import assert from "node:assert/strict";
import { Simulation, adcStep, encoderSpeedStep, quantize, runScenario } from "../src/core/sim.js";
import { disturbanceMetrics, stepMetrics } from "../src/core/metrics.js";
import { defaultSettings, evaluate, makeConfig } from "../src/core/lab.js";
import { near } from "./helpers.js";

/** @typedef {import("../src/core/sim.js").Trace} Trace */

test("quantiser and sensor resolution helpers", () => {
  assert.equal(quantize(0.26, 0.1), 0.30000000000000004);
  assert.equal(quantize(-0.24, 0.1), -0.2);
  assert.equal(quantize(0.123, 0), 0.123);
  near(encoderSpeedStep(4096, 0.001), (2 * Math.PI) / 4.096, 1e-12);
  near(adcStep(10, 300), 0.29296875, 1e-15);
});

test("the same seed reproduces a noisy run exactly; another seed does not", () => {
  const s = { ...defaultSettings("motor"), noiseStd: 0.5, quantStep: 0.1, seed: 7 };
  const a = evaluate(s);
  const b = evaluate(s);
  assert.deepEqual(a.trace, b.trace);
  assert.deepEqual(a.step, b.step);
  const c = evaluate({ ...s, seed: 8 });
  assert.notDeepEqual(a.trace.ym, c.trace.ym);
});

test("the control signal is held between controller samples", () => {
  const { config } = makeConfig(defaultSettings("fopdt")); // dt 0.01, Ts 0.05
  const trace = runScenario(config, {
    duration: 3,
    r1: 1,
    stepTime: 0,
    disturbance: 0,
    disturbanceTime: 0,
  });
  for (let k = 0; k < trace.u.length; k++) {
    if (k % 5 !== 0) assert.equal(trace.u[k], trace.u[k - 1], `sample ${k}`);
  }
  assert.notEqual(trace.u[5], trace.u[0]);
});

test("measurement quantisation and noise reach the controller, not the plant output", () => {
  const { config } = makeConfig({ ...defaultSettings("fopdt"), quantStep: 0.25 });
  const trace = runScenario(config, {
    duration: 20,
    r1: 1,
    stepTime: 0,
    disturbance: 0,
    disturbanceTime: 0,
  });
  for (const v of trace.ym) near(v / 0.25, Math.round(v / 0.25), 1e-9);
  assert.ok(trace.y.some((v) => Math.abs(v / 0.25 - Math.round(v / 0.25)) > 0.01));
});

test("sensor lag delays the measurement like a first-order filter", () => {
  const { config } = makeConfig({ ...defaultSettings("fopdt"), lagTau: 2, sampleTime: 0.01 });
  const sim = new Simulation(config);
  sim.pid.setManual(true);
  sim.pid.manualOutput = 0.5;
  /** @type {number[]} */
  const ym = [];
  /** @type {number[]} */
  const y = [];
  for (let k = 0; k < 3000; k++) {
    const s = sim.step();
    ym.push(s.ym);
    y.push(s.y);
  }
  // Measurement lags the output during the rise and catches up at the end.
  assert.ok(ym[800] < y[800] - 0.05);
  near(ym[2999], y[2999], 0.01);
});

test("feed-forward of the static inverse speeds up the arm and the thermal step", () => {
  for (const id of ["arm", "thermal"]) {
    const base = defaultSettings(id);
    const without = evaluate(base).step;
    const withFf = evaluate({ ...base, feedForward: 1 }).step;
    assert.ok(withFf.iae < without.iae, `${id}: IAE ${withFf.iae} vs ${without.iae}`);
  }
});

test("load disturbance is rejected by integral action", () => {
  const e = evaluate(defaultSettings("fopdt"));
  assert.ok(e.disturbance);
  assert.ok(e.disturbance.peakDeviation > 0.05);
  near(e.trace.y[e.trace.y.length - 1], 1, 0.01);
});

/**
 * Synthetic trace sampled every h seconds.
 * @param {(t: number) => number} f
 * @param {number} tEnd
 * @param {number} h
 * @param {(t: number) => number} [u]
 * @returns {Trace}
 */
function synthetic(f, tEnd, h, u = () => 0) {
  /** @type {Trace} */
  const tr = { t: [], r: [], y: [], ym: [], u: [], d: [] };
  for (let k = 0; k * h <= tEnd + 1e-9; k++) {
    const t = k * h;
    tr.t.push(t);
    tr.r.push(t >= 1 ? 1 : 0);
    tr.y.push(f(t));
    tr.ym.push(f(t));
    tr.u.push(u(t));
    tr.d.push(0);
  }
  return tr;
}

test("first-order response: rise time tau ln 9, 2 % settling tau ln 50, IAE tau", () => {
  const tau = 2;
  const tr = synthetic(
    (t) => (t < 1 ? 0 : 1 - Math.exp(-(t - 1) / tau)),
    61,
    0.001,
    () => 1,
  );
  const m = stepMetrics(tr, { stepTime: 1, endTime: 61, r0: 0, r1: 1, uMin: 0, uMax: 1 });
  near(m.riseTime, tau * Math.log(9), 1e-3);
  near(m.settlingTime, tau * Math.log(50), 2e-3);
  assert.equal(m.overshoot, 0);
  near(m.iae, tau, 2e-3);
  near(m.ise, tau / 2, 2e-3);
  near(m.itae, tau * tau, 5e-3);
  near(m.steadyStateError, 0, 1e-6);
  near(m.effort, 60, 1e-6);
  near(m.saturationTime, 60, 1e-6);
  assert.equal(m.totalVariation, 0);
  const m5 = stepMetrics(tr, { stepTime: 1, endTime: 61, r0: 0, r1: 1, band: 0.05 });
  near(m5.settlingTime, tau * Math.log(20), 2e-3);
});

test("second-order response: overshoot exp(-zeta pi / sqrt(1 - zeta^2))", () => {
  const zeta = 0.3;
  const wn = 2;
  const wd = wn * Math.sqrt(1 - zeta * zeta);
  const tr = synthetic(
    (t) => {
      if (t < 1) return 0;
      const s = t - 1;
      return (
        1 -
        Math.exp(-zeta * wn * s) *
          (Math.cos(wd * s) + (zeta / Math.sqrt(1 - zeta * zeta)) * Math.sin(wd * s))
      );
    },
    30,
    0.0005,
  );
  const m = stepMetrics(tr, { stepTime: 1, endTime: 30, r0: 0, r1: 1 });
  near(m.overshoot, 100 * Math.exp((-zeta * Math.PI) / Math.sqrt(1 - zeta * zeta)), 1e-3);
});

test("a response that never settles or never rises reports NaN", () => {
  const tr = synthetic((t) => (t < 1 ? 0 : 0.5), 10, 0.01);
  const m = stepMetrics(tr, { stepTime: 1, endTime: 10, r0: 0, r1: 1 });
  assert.ok(Number.isNaN(m.riseTime));
  assert.ok(Number.isNaN(m.settlingTime));
  near(m.steadyStateError, 0.5, 1e-12);
  const instant = synthetic(() => 1, 10, 0.01);
  instant.r = instant.r.map(() => 1);
  const mi = stepMetrics(instant, { stepTime: 1, endTime: 10, r0: 0, r1: 1 });
  assert.equal(mi.settlingTime, 0);
  assert.throws(() => stepMetrics(tr, { stepTime: 1, endTime: 1, r0: 0, r1: 1 }), /two samples/);
  assert.throws(() => stepMetrics(tr, { stepTime: 1, endTime: 5, r0: 1, r1: 1 }), /non-zero/);
});

test("downward steps are measured the same way as upward ones", () => {
  const tau = 1;
  const tr = synthetic((t) => (t < 1 ? 1 : Math.exp(-(t - 1) / tau)), 30, 0.001);
  tr.r = tr.t.map((t) => (t >= 1 ? 0 : 1));
  const m = stepMetrics(tr, { stepTime: 1, endTime: 30, r0: 1, r1: 0 });
  near(m.riseTime, tau * Math.log(9), 1e-3);
});

test("disturbance metrics: peak, IAE and recovery", () => {
  // e(t) = 0.2 exp(-(t - 1)) after t = 1.
  const tr = synthetic((t) => (t < 1 ? 1 : 1 - 0.2 * Math.exp(-(t - 1))), 30, 0.001);
  tr.r = tr.r.map(() => 1);
  const m = disturbanceMetrics(tr, { time: 1, band: 0.02 });
  near(m.peakDeviation, 0.2, 1e-9);
  near(m.iae, 0.2, 1e-3);
  near(m.recoveryTime, Math.log(10), 2e-3);
  const never = disturbanceMetrics(tr, { time: 1, band: 1e-15 });
  assert.ok(Number.isNaN(never.recoveryTime));
  const none = disturbanceMetrics(tr, { time: 1, band: 1 });
  assert.equal(none.recoveryTime, 0);
  assert.throws(() => disturbanceMetrics(tr, { time: 100, band: 1 }), /no samples/);
});
