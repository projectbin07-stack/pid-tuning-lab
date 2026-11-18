import test from "node:test";
import assert from "node:assert/strict";
import { PLANTS, getPlant, plantIds, plantParams, polyAdd, polyMul } from "../src/core/plants.js";
import { defaultSettings, makeConfig } from "../src/core/lab.js";
import { stepExperiment } from "../src/core/tuning.js";
import { cdiv, cabs, tfRational } from "../src/core/frequency.js";
import { near } from "./helpers.js";

/**
 * Open-loop step of size du from rest at the plant's default r0.
 * @param {string} id
 * @param {Record<string, number>} params
 * @param {number} du
 * @param {number} duration
 */
function openLoopStep(id, params, du, duration) {
  const { config } = makeConfig({ ...defaultSettings(id), params });
  return stepExperiment(config, { du, duration });
}

test("FOPDT step response matches K du (1 - exp(-(t - theta)/tau))", () => {
  const p = { K: 2, tau: 5, theta: 1 };
  const exp = openLoopStep("fopdt", p, 0.5, 30);
  let worst = 0;
  for (let k = 0; k < exp.t.length; k++) {
    const t = exp.t[k];
    const expected = t <= p.theta ? 0 : 2 * 0.5 * (1 - Math.exp(-(t - p.theta) / p.tau));
    worst = Math.max(worst, Math.abs(exp.yTrue[k] - expected));
  }
  assert.ok(worst < 1e-9, `max deviation ${worst}`);
});

test("underdamped mass-spring-damper matches the analytic step response", () => {
  const m = 1;
  const c = 0.8;
  const k = 4;
  const F = 2;
  const wn = Math.sqrt(k / m);
  const zeta = c / (2 * Math.sqrt(k * m));
  const wd = wn * Math.sqrt(1 - zeta * zeta);
  const exp = openLoopStep("msd", { m, c, k }, F, 12);
  let worst = 0;
  for (let i = 0; i < exp.t.length; i++) {
    const t = exp.t[i];
    const env = Math.exp(-zeta * wn * t);
    const expected =
      (F / k) *
      (1 - env * (Math.cos(wd * t) + (zeta / Math.sqrt(1 - zeta * zeta)) * Math.sin(wd * t)));
    worst = Math.max(worst, Math.abs(exp.yTrue[i] - expected));
  }
  assert.ok(worst < 1e-9, `max deviation ${worst}`);
  // Peak overshoot of the open-loop response: exp(-zeta pi / sqrt(1 - zeta^2)).
  const peak = Math.max(...exp.yTrue) / (F / k) - 1;
  near(peak, Math.exp((-zeta * Math.PI) / Math.sqrt(1 - zeta * zeta)), 1e-4, "overshoot");
});

test("critically damped and overdamped settings do not overshoot", () => {
  for (const c of [4, 10]) {
    const exp = openLoopStep("msd", { m: 1, c, k: 4 }, 4, 20);
    assert.ok(Math.max(...exp.yTrue) <= 1 + 1e-9);
    near(exp.yTrue[exp.yTrue.length - 1], 1, 1e-3);
  }
});

test("every plant is at rest when started at y0 with its steady input", () => {
  for (const plant of PLANTS) {
    const p = plantParams(plant);
    for (const y0 of [plant.defaults.r0, plant.defaults.r1 * 0.5]) {
      const x = plant.initialState(y0, p);
      near(plant.output(x, p), y0, 1e-12, `${plant.id} output`);
      const dx = plant.derivative(x, plant.steadyInput(y0, p), 0, p);
      for (const v of dx) near(v, 0, 1e-9, `${plant.id} derivative at y0=${y0}`);
    }
  }
});

/**
 * Frequency response of the ODEs linearised numerically about the
 * equilibrium at y0: G(jw) = C (jw I - A)^-1 B, by complex Gaussian elimination.
 * @param {import("../src/core/plants.js").Plant} plant
 * @param {Record<string, number>} p
 * @param {number} y0
 * @param {number} w
 */
function numericResponse(plant, p, y0, w) {
  const x0 = plant.initialState(y0, p);
  const u0 = plant.steadyInput(y0, p);
  const n = x0.length;
  const h = 1e-6;
  /** @type {number[][]} */
  const A = [];
  for (let i = 0; i < n; i++) A.push(new Array(n).fill(0));
  for (let j = 0; j < n; j++) {
    const xp = Float64Array.from(x0);
    const xm = Float64Array.from(x0);
    const step = h * Math.max(1, Math.abs(x0[j]));
    xp[j] += step;
    xm[j] -= step;
    const fp = plant.derivative(xp, u0, 0, p);
    const fm = plant.derivative(xm, u0, 0, p);
    for (let i = 0; i < n; i++) A[i][j] = (fp[i] - fm[i]) / (2 * step);
  }
  const fp = plant.derivative(x0, u0 + h, 0, p);
  const fm = plant.derivative(x0, u0 - h, 0, p);
  const B = Array.from(fp, (v, i) => (v - fm[i]) / (2 * h));
  /** @type {number[]} */
  const C = [];
  for (let j = 0; j < n; j++) {
    const xp = Float64Array.from(x0);
    xp[j] += 1;
    C.push(plant.output(xp, p) - plant.output(x0, p));
  }
  // Augmented complex matrix [jw I - A | B].
  const M = A.map((row, i) => [
    ...row.map((a, j) => ({ re: -a, im: i === j ? w : 0 })),
    { re: B[i], im: 0 },
  ]);
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) if (cabs(M[r][col]) > cabs(M[piv][col])) piv = r;
    [M[col], M[piv]] = [M[piv], M[col]];
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = cdiv(M[r][col], M[col][col]);
      for (let c2 = col; c2 <= n; c2++) {
        const t = M[col][c2];
        M[r][c2] = {
          re: M[r][c2].re - (f.re * t.re - f.im * t.im),
          im: M[r][c2].im - (f.re * t.im + f.im * t.re),
        };
      }
    }
  }
  let re = 0;
  let im = 0;
  for (let i = 0; i < n; i++) {
    const z = cdiv(M[i][n], M[i][i]);
    re += C[i] * z.re;
    im += C[i] * z.im;
  }
  return { re, im };
}

test("documented transfer functions agree with the linearised differential equations", () => {
  for (const plant of PLANTS) {
    // Coulomb friction is not part of the motor's linear model.
    const p = plantParams(plant, plant.id === "motor" ? { Tc: 0 } : {});
    const y0 = plant.id === "arm" ? 0.7 : plant.defaults.r0;
    const tf = plant.transferFunction(p, y0);
    for (const w of [0.01, 0.3, 3, 30, 300]) {
      const a = tfRational(tf, w);
      const b = numericResponse(plant, p, y0, w);
      const scale = Math.max(cabs(a), 1e-12);
      assert.ok(
        Math.hypot(a.re - b.re, a.im - b.im) / scale < 1e-4,
        `${plant.id} at w=${w}: tf ${a.re},${a.im} vs ode ${b.re},${b.im}`,
      );
    }
  }
});

test("thermal steady state equals the transfer function's DC gain Pmax Ra", () => {
  const thermal = getPlant("thermal");
  const p = plantParams(thermal);
  const tf = thermal.transferFunction(p, p.Ta);
  near(tf.num[0] / tf.den[0], p.Pmax * p.Ra, 1e-9);
  const exp = openLoopStep("thermal", {}, 0.5, 600);
  near(exp.yTrue[exp.y.length - 1] - exp.y0, 0.5 * p.Pmax * p.Ra, 0.05, "rise after 600 s");
});

test("DC motor reaches the no-friction steady speed Kt V / (R b + Kt Ke)", () => {
  const exp = openLoopStep("motor", { Tc: 0 }, 6, 1);
  const q = plantParams(getPlant("motor"), { Tc: 0 });
  near(exp.yTrue[exp.y.length - 1], (q.Kt * 6) / (q.R * q.b + q.Kt * q.Ke), 0.01);
});

test("Coulomb friction lowers the motor's steady speed by R Tc / (R b + Kt Ke)", () => {
  const exp = openLoopStep("motor", {}, 6, 1);
  const q = plantParams(getPlant("motor"));
  const expected = (q.Kt * 6 - q.R * q.Tc) / (q.R * q.b + q.Kt * q.Ke);
  near(exp.yTrue[exp.y.length - 1], expected, 0.05);
});

test("integrating plant follows x(t) = K du (t - tau (1 - exp(-t/tau)))", () => {
  const exp = openLoopStep("integrating", { K: 1, tau: 0.2 }, 0.3, 5);
  const last = exp.y.length - 1;
  near(exp.yTrue[last], 0.3 * (exp.t[last] - 0.2 * (1 - Math.exp(-exp.t[last] / 0.2))), 1e-9);
});

test("the arm hangs at q = 0 and needs m g l sin(q) to hold an angle", () => {
  const arm = getPlant("arm");
  const p = plantParams(arm);
  near(arm.steadyInput(Math.PI / 2, p), p.m * 9.81 * p.l, 1e-12);
  near(arm.steadyInput(0, p), 0, 1e-12);
  // Small swings about the bottom without damping: period 2 pi sqrt(J / (m g l)).
  const exp = openLoopStep("arm", { b: 0 }, 0.01, 3);
  const J = p.m * p.l * p.l + p.Jm;
  const period = 2 * Math.PI * Math.sqrt(J / (p.m * 9.81 * p.l));
  // First peak of the swing comes after half a period.
  let peak = 1;
  while (exp.yTrue[peak + 1] > exp.yTrue[peak]) peak++;
  near(exp.t[peak], period / 2, 0.005);
});

test("parameter overrides are validated", () => {
  const fopdt = getPlant("fopdt");
  assert.equal(plantParams(fopdt, { K: 3 }).K, 3);
  assert.throws(() => plantParams(fopdt, { nope: 1 }), /no parameter/);
  assert.throws(() => plantParams(fopdt, { K: 1000 }), /must be in/);
  assert.throws(() => plantParams(fopdt, { K: NaN }), /must be in/);
  assert.throws(() => getPlant("cart-pole"), /unknown plant/);
  assert.deepEqual(plantIds(), ["fopdt", "msd", "motor", "integrating", "thermal", "arm"]);
});

test("polynomial helpers", () => {
  assert.deepEqual(polyMul([1, 2], [3, 4]), [3, 10, 8]);
  assert.deepEqual(polyAdd([1, 2, 3], [1, 1], -1), [0, 1, 3]);
  assert.deepEqual(polyAdd([1], [1, 1]), [2, 1]);
});

test("every plant documents its equations, parameters and defaults", () => {
  for (const plant of PLANTS) {
    assert.ok(plant.equations.length > 0);
    assert.ok(plant.params.length > 0);
    for (const s of plant.params) {
      assert.ok(s.min <= s.value && s.value <= s.max, `${plant.id}.${s.key}`);
    }
    const d = plant.defaults;
    assert.ok(d.sampleTime >= d.dt && d.uMax > d.uMin && d.duration > d.disturbanceTime);
  }
});
