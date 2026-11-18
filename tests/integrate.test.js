import test from "node:test";
import assert from "node:assert/strict";
import { DelayBuffer, integrate, rk4Step } from "../src/core/integrate.js";
import { near } from "./helpers.js";

/** dx/dt = -x + sin(t): smooth, non-autonomous, with a known solution. */
const f = (/** @type {number} */ t, /** @type {Float64Array} */ x) =>
  Float64Array.of(-x[0] + Math.sin(t));
// x(0) = 0  =>  x(t) = (sin t - cos t + e^-t) / 2
const exact = (/** @type {number} */ t) => (Math.sin(t) - Math.cos(t) + Math.exp(-t)) / 2;

/** @param {number} h */
function errorAt2(h) {
  /** @type {Float64Array} */
  let x = Float64Array.of(0);
  const n = Math.round(2 / h);
  for (let k = 0; k < n; k++) x = rk4Step(f, k * h, x, h);
  return Math.abs(x[0] - exact(2));
}

test("RK4 global error falls with the fourth power of the step", () => {
  const e1 = errorAt2(0.1);
  const e2 = errorAt2(0.05);
  const e3 = errorAt2(0.025);
  const order1 = Math.log2(e1 / e2);
  const order2 = Math.log2(e2 / e3);
  near(order1, 4, 0.15, "observed order (h=0.1 -> 0.05)");
  near(order2, 4, 0.15, "observed order (h=0.05 -> 0.025)");
  assert.ok(e3 < 1e-7);
});

test("integrate with sub-steps equals repeated RK4 steps", () => {
  const a = integrate(f, 0.3, Float64Array.of(0.5), 0.2, 4);
  /** @type {Float64Array} */
  let b = Float64Array.of(0.5);
  for (let i = 0; i < 4; i++) b = rk4Step(f, 0.3 + i * 0.05, b, 0.05);
  near(a[0], b[0], 1e-15);
  // substeps below 1 are treated as 1
  const c = integrate(f, 0, Float64Array.of(1), 0.1, 0);
  near(c[0], rk4Step(f, 0, Float64Array.of(1), 0.1)[0], 0);
});

test("RK4 integrates a harmonic oscillator for many periods without drifting", () => {
  const osc = (/** @type {number} */ _t, /** @type {Float64Array} */ x) =>
    Float64Array.of(x[1], -x[0]);
  /** @type {Float64Array} */
  let x = Float64Array.of(1, 0);
  const h = 0.01;
  const n = Math.round((20 * Math.PI) / h);
  for (let k = 0; k < n; k++) x = rk4Step(osc, k * h, x, h);
  near(x[0], Math.cos(n * h), 1e-6);
  near(x[1], -Math.sin(n * h), 1e-6);
});

test("delay buffer delays by a whole number of samples", () => {
  const d = new DelayBuffer(0.3, 0.1, -1);
  assert.equal(d.samples, 3);
  const out = [1, 2, 3, 4, 5].map((v) => d.push(v));
  assert.deepEqual(out, [-1, -1, -1, 1, 2]);
});

test("zero delay passes values straight through", () => {
  const d = new DelayBuffer(0, 0.01);
  assert.equal(d.push(7), 7);
  assert.equal(d.push(8), 8);
});

test("delay buffer rejects invalid arguments", () => {
  assert.throws(() => new DelayBuffer(1, 0), RangeError);
  assert.throws(() => new DelayBuffer(-1, 0.1), RangeError);
});
