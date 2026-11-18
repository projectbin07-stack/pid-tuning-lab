import test from "node:test";
import assert from "node:assert/strict";
import {
  cabs,
  cdiv,
  cmul,
  defaultRange,
  frequencyResponse,
  logspace,
  pidLoop,
  pidResponse,
  polyAtJw,
  stabilityMargins,
  tfRational,
} from "../src/core/frequency.js";
import { pidSettings } from "../src/core/pid.js";
import { defaultSettings, evaluate, loopMargins, makeConfig } from "../src/core/lab.js";
import { near } from "./helpers.js";

/** @typedef {import("../src/core/frequency.js").LoopFunction} LoopFunction */

/**
 * Root of f on [a, b] by bisection (f(a) and f(b) differ in sign).
 * @param {(x: number) => number} f
 * @param {number} a
 * @param {number} b
 */
function root(f, a, b) {
  for (let i = 0; i < 200; i++) {
    const m = (a + b) / 2;
    if (Math.sign(f(m)) === Math.sign(f(a))) a = m;
    else b = m;
  }
  return (a + b) / 2;
}

test("complex helpers and polynomial evaluation", () => {
  assert.deepEqual(cmul({ re: 1, im: 2 }, { re: 3, im: -1 }), { re: 5, im: 5 });
  const q = cdiv({ re: 5, im: 5 }, { re: 3, im: -1 });
  near(q.re, 1, 1e-15);
  near(q.im, 2, 1e-15);
  // 1 + 2s + 3s^2 at s = 2j: 1 - 12 + 4j
  assert.deepEqual(polyAtJw([1, 2, 3], 2), { re: -11, im: 4 });
  near(cabs(tfRational({ num: [1], den: [1, 1], delay: 0 }, 1)), Math.SQRT1_2, 1e-15);
  const w = logspace(0.01, 100, 5);
  near(w[2], 1, 1e-12);
});

test("integrator with delay: PM = 90 deg - K theta, GM = pi / (2 theta K)", () => {
  // L(jw) = K e^{-j w theta} / (j w), K = 0.5, theta = 1. Hand-derived:
  // |L| = 1 at w = K; phase there -90 deg - K theta rad.
  // Phase -180 deg where w theta = pi/2, so GM = (pi / (2 theta)) / K.
  const K = 0.5;
  const theta = 1;
  /** @type {LoopFunction} */
  const loop = (w) => ({ rational: { re: 0, im: -K / w }, delay: theta });
  const m = stabilityMargins(loop, 1e-3, 100);
  near(m.gainCrossover, K, 1e-6);
  near(m.phaseMargin, 90 - (K * theta * 180) / Math.PI, 1e-6);
  near(m.phaseCrossover, Math.PI / 2, 1e-6);
  near(m.gainMargin, Math.PI / (2 * theta * K), 1e-6);
  near(m.delayMargin, (m.phaseMargin * Math.PI) / 180 / K, 1e-6);
});

test("third-order lag K/(s+1)^3: GM = 8/K at w = sqrt(3)", () => {
  const K = 4;
  /** @type {LoopFunction} */
  const loop = (w) => ({
    rational: tfRational({ num: [K], den: [1, 3, 3, 1], delay: 0 }, w),
    delay: 0,
  });
  const m = stabilityMargins(loop, 1e-3, 1e3);
  near(m.phaseCrossover, Math.sqrt(3), 1e-6);
  near(m.gainMargin, 8 / K, 1e-6);
  near(m.gainMarginDb, 20 * Math.log10(2), 1e-6);
  // |L| = 1 where (1 + w^2)^(3/2) = K.
  const wc = Math.sqrt(K ** (2 / 3) - 1);
  near(m.gainCrossover, wc, 1e-6);
  near(m.phaseMargin, 180 - (3 * Math.atan(wc) * 180) / Math.PI, 1e-6);
});

test("FOPDT under P control through the sampled-loop model, with a fast sample time", () => {
  // Plant 2 e^{-0.5 s} / (s + 1), Kp = 1, T = 1 ms (ZOH effect negligible).
  // |L| = 1 at w = sqrt(3); PM = 180 - atan(sqrt 3) - 0.5 sqrt 3 rad = 70.38 deg.
  // Phase crossover where atan(w) + 0.5 w = pi.
  const pid = pidSettings({ kp: 1, sampleTime: 0.001 });
  const tf = { num: [2], den: [1, 1], delay: 0.5 };
  const m = stabilityMargins(pidLoop(tf, pid), 1e-3, Math.PI / 0.001);
  const pmHand = 180 - ((Math.atan(Math.sqrt(3)) + 0.5 * Math.sqrt(3)) * 180) / Math.PI;
  near(m.gainCrossover, Math.sqrt(3), 1e-3);
  near(m.phaseMargin, pmHand, 0.06); // ZOH adds w T / 2 = 0.05 deg of lag
  // Including the ZOH half-sample delay T/2 makes the hand value exact.
  const w180 = root((w) => Math.atan(w) + (0.5 + 0.0005) * w - Math.PI, 0.1, 10);
  near(m.phaseCrossover, w180, 1e-5);
  near(m.gainMargin, Math.sqrt(1 + w180 * w180) / 2, 1e-4);
});

test("discrete PID response approaches the continuous PID at low frequency", () => {
  const pid = pidSettings({ kp: 2, ki: 3, kd: 0.5, n: 10, sampleTime: 0.001 });
  const tf = 0.5 / 2 / 10;
  for (const w of [0.1, 1, 10]) {
    const c = pidResponse(pid, w);
    // Kp + Ki/(jw) + Kd jw / (1 + jw Tf)
    const der = cdiv({ re: 0, im: 0.5 * w }, { re: 1, im: w * tf });
    const ref = { re: 2 + der.re, im: -3 / w + der.im };
    assert.ok(Math.hypot(c.re - ref.re, c.im - ref.im) / cabs(ref) < 0.01, `w=${w}`);
  }
});

test("phase of a type-1 loop starts at -90 deg and is unwrapped", () => {
  const pid = pidSettings({ kp: 1, ki: 1, sampleTime: 0.01 });
  const fr = frequencyResponse(
    pidLoop({ num: [1], den: [1, 1], delay: 2 }, pid),
    logspace(1e-4, 300, 2000),
  );
  near(fr.phaseDeg[0], -90, 0.1);
  for (let i = 1; i < fr.phaseDeg.length; i++)
    assert.ok(fr.phaseDeg[i] <= fr.phaseDeg[i - 1] + 1e-9);
  assert.ok(fr.phaseDeg[fr.phaseDeg.length - 1] < -10000); // delay keeps adding lag
  near(Math.hypot(fr.re[0], fr.im[0]), 10 ** (fr.magDb[0] / 20), 1e-6);
});

test("no crossover gives infinite margins", () => {
  /** @type {LoopFunction} */
  const loop = (w) => ({
    rational: tfRational({ num: [0.5], den: [1, 1], delay: 0 }, w),
    delay: 0,
  });
  const m = stabilityMargins(loop, 1e-3, 1e3);
  assert.equal(m.gainMargin, Infinity);
  assert.equal(m.phaseMargin, Infinity);
  assert.ok(Number.isNaN(m.gainCrossover));
});

test("an unstable loop has a negative phase margin and a gain margin below 1", () => {
  /** @type {LoopFunction} */
  const loop = (w) => ({
    rational: tfRational({ num: [20], den: [1, 3, 3, 1], delay: 0 }, w),
    delay: 0,
  });
  const m = stabilityMargins(loop, 1e-3, 1e3);
  assert.ok(m.phaseMargin < 0);
  near(m.gainMargin, 8 / 20, 1e-6);
  assert.equal(m.delayMargin, 0);
});

test("a type-2 loop sitting at -180 deg at low frequency is not reported as unstable", () => {
  // Arm at q = pi/2 has no gravity stiffness: plant 1/(J s^2 + b s), PI adds 1/s.
  const e = evaluate(defaultSettings("arm"));
  assert.ok(e.margins);
  assert.ok(e.margins.phaseMargin > 30, `PM ${e.margins.phaseMargin}`);
  assert.ok(e.margins.gainMarginDb > 6, `GM ${e.margins.gainMarginDb}`);
  assert.match(e.marginNote, /linearised/);
});

test("sensor lag reduces the phase margin", () => {
  const s = defaultSettings("fopdt");
  const a = loopMargins(makeConfig(s).config, 1).margins;
  const b = loopMargins(makeConfig({ ...s, lagTau: 0.5 }).config, 1).margins;
  assert.ok(a && b && b.phaseMargin < a.phaseMargin);
});

test("default range spans six decades up to Nyquist", () => {
  const [lo, hi] = defaultRange(0.01);
  near(hi, (0.999 * Math.PI) / 0.01, 1e-9);
  near(hi / lo, 1e6, 1e-3);
});

test("margins of every default configuration are finite and positive", () => {
  for (const id of ["fopdt", "msd", "motor", "integrating", "thermal", "arm"]) {
    const m = evaluate(defaultSettings(id)).margins;
    assert.ok(m);
    assert.ok(m.phaseMargin > 30 && m.phaseMargin < 120, `${id} PM ${m.phaseMargin}`);
    assert.ok(m.gainMarginDb > 6, `${id} GM ${m.gainMarginDb}`);
  }
});
