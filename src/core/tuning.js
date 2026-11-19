// Plant experiments, FOPDT identification and PID tuning rules.
//
// Experiments run on the simulated plant through the same Simulation class
// as the closed loop, so sensor noise, quantisation, dead time and actuator
// limits affect them just as they would on a test bench.

import { Simulation } from "./sim.js";

/** @typedef {import("./sim.js").SimConfig} SimConfig */
/** @typedef {import("./sim.js").Trace} Trace */

/**
 * @typedef {object} Fopdt
 * @property {number} K static gain (output units per input unit)
 * @property {number} tau time constant (s)
 * @property {number} theta dead time (s)
 *
 * @typedef {object} Gains PID gains in both forms
 * @property {number} kp
 * @property {number} ki
 * @property {number} kd
 * @property {number} kc
 * @property {number} ti 0 = no integral action
 * @property {number} td
 *
 * @typedef {"p" | "pi" | "pid"} ControllerType
 *
 * @typedef {object} StepExperiment
 * @property {number[]} t time since the input step (s)
 * @property {number[]} y measured output (what an identification method sees)
 * @property {number[]} yTrue true plant output
 * @property {number} u0 input before the step
 * @property {number} du input step size
 * @property {number} y0 output before the step
 *
 * @typedef {object} RelayResult
 * @property {number} ku ultimate gain estimate 4 d / (pi sqrt(a^2 - eps^2))
 * @property {number} pu ultimate period estimate (s)
 * @property {number} amplitude output oscillation amplitude a
 * @property {number} relayAmplitude d
 * @property {number} hysteresis eps
 * @property {number} periods number of full periods used
 * @property {boolean} converged false if the period was still changing at maxTime
 * @property {number} kuHarmonic 1 / |G| at the oscillation frequency, from the first
 *   Fourier components of the measured output and of the relay output
 * @property {number} phaseDeg phase of G at the oscillation frequency from the same
 *   components (-180 deg for an ideal relay and a sinusoidal output)
 * @property {number} periodSpread (max - min) / mean of the periods used
 * @property {number} amplitudeSpread (max - min) / mean of the per-cycle amplitudes
 * @property {number} asymmetry largest |2 t_high / P - 1| over the cycles used
 *   (0 = equal time at +d and -d)
 * @property {Trace} trace
 */

/**
 * @param {number} kc
 * @param {number} ti
 * @param {number} td
 * @returns {Gains}
 */
export function fromIdeal(kc, ti, td) {
  return { kp: kc, ki: ti > 0 ? kc / ti : 0, kd: kc * td, kc, ti, td };
}

/**
 * Apply an open-loop input step from equilibrium. The plant starts at rest
 * with the input that holds it there; the output is recorded for a baseline
 * period (default 10 % of the duration) to estimate y0 through the noise,
 * then the input steps by du. Times are relative to the step. The
 * controller is in manual mode the whole time.
 * @param {SimConfig} config
 * @param {{du: number, duration: number, baseline?: number}} opts
 * @returns {StepExperiment}
 */
export function stepExperiment(config, opts) {
  const sim = new Simulation({ ...config, feedForward: 0 });
  const u0 = sim.u;
  sim.pid.setManual(true);
  sim.pid.manualOutput = u0;
  // Baseline: a whole number of controller periods.
  const periods = Math.round((opts.baseline ?? 0.1 * opts.duration) / (sim.period * config.dt));
  let sum = 0;
  for (let k = 0; k < periods * sim.period; k++) sum += sim.step().ym;
  const y0 = periods > 0 ? sum / (periods * sim.period) : sim.measure();
  const t0 = sim.t;
  sim.pid.manualOutput = u0 + opts.du;
  const n = Math.round(opts.duration / config.dt);
  /** @type {number[]} */
  const t = [];
  /** @type {number[]} */
  const y = [];
  /** @type {number[]} */
  const yTrue = [];
  for (let k = 0; k <= n; k++) {
    const s = sim.step();
    t.push(s.t - t0);
    y.push(s.ym);
    yTrue.push(s.y);
  }
  return { t, y, yTrue, u0, du: opts.du, y0 };
}

/**
 * Mean of the last `fraction` of an array.
 * @param {number[]} v
 * @param {number} fraction
 */
function tailMean(v, fraction) {
  const m = Math.max(1, Math.round(v.length * fraction));
  let s = 0;
  for (let i = v.length - m; i < v.length; i++) s += v[i];
  return s / m;
}

/**
 * Two-point FOPDT fit (Smith's method): the times at which the response
 * reaches 28.3 % and 63.2 % of its final change give
 * tau = 1.5 (t63 - t28) and theta = t63 - tau. K = final change / du.
 * Uses the mean of the last 5 % of samples as the final value, so the
 * experiment must be long enough to settle.
 * @param {StepExperiment} exp
 * @returns {Fopdt}
 */
export function identifyTwoPoint(exp) {
  const dy = tailMean(exp.y, 0.05) - exp.y0;
  if (dy === 0 || exp.du === 0) throw new RangeError("the step produced no output change");
  const z = exp.y.map((v) => (v - exp.y0) / dy);
  /** @param {number} level */
  const first = (level) => {
    for (let k = 1; k < z.length; k++) {
      if (z[k] >= level) {
        const f = (level - z[k - 1]) / (z[k] - z[k - 1]);
        return exp.t[k - 1] + f * (exp.t[k] - exp.t[k - 1]);
      }
    }
    return NaN;
  };
  const t28 = first(0.283);
  const t63 = first(0.632);
  const tau = 1.5 * (t63 - t28);
  return { K: dy / exp.du, tau, theta: Math.max(0, t63 - tau) };
}

/**
 * Response of an FOPDT model to a unit input step, at time t after the step.
 * @param {Fopdt} m
 * @param {number} t
 * @returns {number}
 */
export function fopdtStep(m, t) {
  return t <= m.theta ? 0 : m.K * (1 - Math.exp(-(t - m.theta) / m.tau));
}

/**
 * Least-squares FOPDT fit. For fixed (tau, theta) the best K has a closed
 * form; (log tau, theta) are found by a grid search followed by Nelder-Mead.
 * @param {StepExperiment} exp
 * @returns {Fopdt & {rms: number}}
 */
export function identifyLeastSquares(exp) {
  if (exp.du === 0) throw new RangeError("the step produced no output change");
  const dy = exp.y.map((v) => (v - exp.y0) / exp.du);
  const tEnd = exp.t[exp.t.length - 1];

  /** @param {number} tau @param {number} theta */
  const fit = (tau, theta) => {
    let num = 0;
    let den = 0;
    for (let k = 0; k < dy.length; k++) {
      const g = exp.t[k] <= theta ? 0 : 1 - Math.exp(-(exp.t[k] - theta) / tau);
      num += g * dy[k];
      den += g * g;
    }
    const K = den > 0 ? num / den : 0;
    let sse = 0;
    for (let k = 0; k < dy.length; k++) {
      const g = exp.t[k] <= theta ? 0 : 1 - Math.exp(-(exp.t[k] - theta) / tau);
      sse += (dy[k] - K * g) ** 2;
    }
    return { K, sse };
  };
  /** @param {number[]} q */
  const cost = (q) => {
    const theta = q[1];
    if (theta < 0 || theta > tEnd / 2 || q[0] > Math.log(10 * tEnd)) return Infinity;
    return fit(Math.exp(q[0]), theta).sse;
  };

  // Coarse grid.
  let best = [Math.log(tEnd / 10), 0];
  let bestCost = cost(best);
  for (let i = 0; i <= 24; i++) {
    const lt = Math.log(tEnd / 1000) + (i / 24) * Math.log(1000 / 1.5);
    for (let j = 0; j <= 20; j++) {
      const q = [lt, (j / 20) * (tEnd / 2)];
      const cq = cost(q);
      if (cq < bestCost) {
        best = q;
        bestCost = cq;
      }
    }
  }
  const q = nelderMead(cost, best, [0.2, tEnd / 80], 400);
  const tau = Math.exp(q[0]);
  const { K, sse } = fit(tau, q[1]);
  return { K, tau, theta: q[1], rms: Math.sqrt(sse / dy.length) * Math.abs(exp.du) };
}

/**
 * Minimal Nelder-Mead simplex minimiser (deterministic).
 * @param {(q: number[]) => number} f
 * @param {number[]} x0
 * @param {number[]} scale initial simplex size per coordinate
 * @param {number} iterations
 * @returns {number[]}
 */
export function nelderMead(f, x0, scale, iterations) {
  const n = x0.length;
  /** @type {{x: number[], f: number}[]} */
  let pts = [{ x: x0.slice(), f: f(x0) }];
  for (let i = 0; i < n; i++) {
    const x = x0.slice();
    x[i] += scale[i];
    pts.push({ x, f: f(x) });
  }
  /** @param {number[]} a @param {number[]} b @param {number} t */
  const lerp = (a, b, t) => a.map((v, i) => v + t * (b[i] - v));
  for (let it = 0; it < iterations; it++) {
    pts.sort((a, b) => a.f - b.f);
    const centroid = new Array(n).fill(0);
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) centroid[j] += pts[i].x[j] / n;
    const worst = pts[n];
    const xr = lerp(worst.x, centroid, 2);
    const fr = f(xr);
    if (fr < pts[0].f) {
      const xe = lerp(worst.x, centroid, 3);
      const fe = f(xe);
      pts[n] = fe < fr ? { x: xe, f: fe } : { x: xr, f: fr };
    } else if (fr < pts[n - 1].f) {
      pts[n] = { x: xr, f: fr };
    } else {
      const xc = lerp(worst.x, centroid, 0.5);
      const fc = f(xc);
      if (fc < worst.f) {
        pts[n] = { x: xc, f: fc };
      } else {
        const b = pts[0].x;
        pts = pts.map((p, i) => {
          if (i === 0) return p;
          const x = lerp(b, p.x, 0.5);
          return { x, f: f(x) };
        });
      }
    }
  }
  pts.sort((a, b) => a.f - b.f);
  return pts[0].x;
}

/**
 * Astrom-Hagglund relay experiment. The loop is closed through an ideal
 * relay u = u0 +/- d (with hysteresis eps) around the operating point; the
 * limit cycle's period approximates the ultimate period and, by the
 * describing function of the relay, its amplitude a gives the ultimate gain
 * Ku = 4 d / (pi sqrt(a^2 - eps^2)).
 *
 * Besides that describing-function estimate, the result carries the ratio of
 * the first Fourier components of the measured output and of the relay output
 * over the periods used. For a linear plant this ratio is the plant's
 * frequency response at the oscillation frequency, without assuming that the
 * output is sinusoidal, so `kuHarmonic` = 1 / |G| there. It also reports how
 * regular the cycle was (period and amplitude spread, high/low asymmetry),
 * which `relayValidity` uses.
 *
 * `extraDelay` makes the relay act on the measurement taken that many
 * controller samples earlier: a known delay on the controller side, used by
 * the validity test. The plant starts at rest at config.r0, which is used as
 * the setpoint.
 * @param {SimConfig} config
 * @param {{amplitude: number, hysteresis?: number, maxTime: number, periods?: number, extraDelay?: number}} opts
 * @returns {RelayResult}
 */
export function relayExperiment(config, opts) {
  const eps = opts.hysteresis ?? 0;
  const d = opts.amplitude;
  const wanted = opts.periods ?? 4;
  const extra = Math.max(0, Math.round(opts.extraDelay ?? 0));
  if (!(d > 0)) throw new RangeError("relay amplitude must be positive");
  const probe = new Simulation({ ...config, feedForward: 0 });
  const u0 = probe.u;
  let high = true;
  /** @type {number[]} */
  const rises = [];
  /** @type {number[]} */
  const falls = [];
  /** @type {number[]} measurements of the last extra + 1 samples */
  const past = [];
  const sim = new Simulation({
    ...config,
    feedForward: 0,
    controlLaw: (r, ym, t) => {
      past.push(ym);
      if (past.length > extra + 1) past.shift();
      const e = r - past[0];
      if (!high && e > eps) {
        high = true;
        rises.push(t);
      } else if (high && e < -eps) {
        high = false;
        falls.push(t);
      }
      return high ? u0 + d : u0 - d;
    },
  });
  /** @type {Trace} */
  const trace = { t: [], r: [], y: [], ym: [], u: [], d: [] };
  const n = Math.round(opts.maxTime / config.dt);
  // The limit cycle takes several periods to become symmetric. Run until
  // the last `wanted` periods agree within 0.5 % (or time runs out), then
  // average over them.
  const needed = wanted + 2;
  let converged = false;
  let seen = 0;
  for (let k = 0; k <= n && !converged; k++) {
    const s = sim.step();
    if (rises.length !== seen) {
      seen = rises.length;
      if (seen >= needed) {
        const recent = rises.slice(seen - wanted - 1);
        const periods = recent.slice(1).map((v, i) => v - recent[i]);
        const mean = periods.reduce((a, b) => a + b, 0) / wanted;
        converged = Math.max(...periods) - Math.min(...periods) <= 0.005 * mean;
      }
    }
    trace.t.push(s.t);
    trace.r.push(s.r);
    trace.y.push(s.y);
    trace.ym.push(s.ym);
    trace.u.push(s.u);
    trace.d.push(s.d);
  }
  if (rises.length < needed) {
    throw new Error(
      `relay experiment produced ${rises.length} switches in ${opts.maxTime} s; no sustained oscillation`,
    );
  }
  const used = rises.slice(rises.length - wanted - 1);
  const t0 = used[0];
  const t1 = used[used.length - 1];
  const pu = (t1 - t0) / wanted;
  const w = (2 * Math.PI) / pu;
  let lo = Infinity;
  let hi = -Infinity;
  // Per-cycle extremes, and the first Fourier components of ym and u over
  // exactly `wanted` periods (rectangle rule on the plant interval; u is
  // piecewise constant on it, and the constant offsets drop out).
  const cycleLo = new Array(wanted).fill(Infinity);
  const cycleHi = new Array(wanted).fill(-Infinity);
  let yRe = 0;
  let yIm = 0;
  let uRe = 0;
  let uIm = 0;
  for (let k = 0; k < trace.t.length; k++) {
    const tk = trace.t[k];
    if (tk < t0 - 1e-12) continue;
    lo = Math.min(lo, trace.ym[k]);
    hi = Math.max(hi, trace.ym[k]);
    if (tk >= t1 - 1e-12) continue;
    const i = Math.min(wanted - 1, Math.floor((tk - t0) / pu));
    cycleLo[i] = Math.min(cycleLo[i], trace.ym[k]);
    cycleHi[i] = Math.max(cycleHi[i], trace.ym[k]);
    const cs = Math.cos(w * (tk - t0));
    const sn = Math.sin(w * (tk - t0));
    yRe += trace.ym[k] * cs;
    yIm -= trace.ym[k] * sn;
    uRe += trace.u[k] * cs;
    uIm -= trace.u[k] * sn;
  }
  const a = (hi - lo) / 2;
  const ku = (4 * d) / (Math.PI * Math.sqrt(Math.max(a * a - eps * eps, 1e-300)));
  const den = uRe * uRe + uIm * uIm;
  const gRe = (yRe * uRe + yIm * uIm) / den;
  const gIm = (yIm * uRe - yRe * uIm) / den;
  const amps = cycleHi.map((v, i) => (v - cycleLo[i]) / 2);
  const ampMean = amps.reduce((x, y) => x + y, 0) / wanted;
  const cyclePeriods = used.slice(1).map((v, i) => v - used[i]);
  // Time at +d in each cycle: from its rise to the next fall.
  let asymmetry = 0;
  for (let i = 0; i < wanted; i++) {
    const fall = falls.find((f) => f > used[i]);
    const tHigh = fall === undefined || fall > used[i + 1] ? cyclePeriods[i] : fall - used[i];
    asymmetry = Math.max(asymmetry, Math.abs((2 * tHigh) / cyclePeriods[i] - 1));
  }
  return {
    ku,
    pu,
    amplitude: a,
    relayAmplitude: d,
    hysteresis: eps,
    periods: wanted,
    converged,
    kuHarmonic: 1 / Math.hypot(gRe, gIm),
    phaseDeg: (Math.atan2(gIm, gRe) * 180) / Math.PI,
    periodSpread: (Math.max(...cyclePeriods) - Math.min(...cyclePeriods)) / pu,
    amplitudeSpread: (Math.max(...amps) - Math.min(...amps)) / ampMean,
    asymmetry,
    trace,
  };
}

/**
 * Limits of the relay validity test (see docs/models.md, "Is the relay
 * result meaningful?").
 */
export const RELAY_LIMITS = Object.freeze({
  /** fewest controller samples per oscillation period */
  minSamplesPerPeriod: 10,
  /** largest relative lengthening of the period caused by one extra sample of delay */
  maxDelaySensitivity: 0.25,
  /** largest (max - min) / mean of the per-cycle amplitudes */
  maxAmplitudeSpread: 0.05,
  /** largest |2 t_high / P - 1| */
  maxAsymmetry: 0.1,
});

/**
 * @typedef {object} RelayValidity
 * @property {boolean} valid every criterion passed
 * @property {number} sampleTime controller sample time actually used (s)
 * @property {number} samplesPerPeriod Pu / sample time
 * @property {number} puDelayed period with one extra controller sample of delay (s)
 * @property {number} delaySensitivity (puDelayed - Pu) / Pu
 * @property {boolean} timingDominated the samples-per-period or delay-sensitivity criterion failed
 * @property {boolean} irregular the cycle had not settled, or its amplitude or symmetry varied
 * @property {string[]} failures one sentence fragment per failed criterion
 */

/**
 * Decide whether a relay experiment identified a property of the plant
 * rather than of the controller's timing. The test repeats the experiment
 * with the relay acting on a measurement one controller sample older, and
 * checks:
 *
 * 1. Delay sensitivity. If the limit cycle is set by the plant's own phase
 *    lag, one extra sample of delay changes the period only by roughly
 *    T / D, where D = -d(arg L)/dw is the loop's phase slope at the
 *    oscillation; if the loop delay (sample, hold and that extra sample)
 *    is what brings the phase to -180 deg, the period scales with it. The
 *    relative change must stay at or below `maxDelaySensitivity`.
 * 2. Samples per period. Below `minSamplesPerPeriod` the period is resolved
 *    to worse than 10 % and the ZN derivative time Pu/8 is about one sample.
 * 3. A steady, symmetric cycle: the period had converged (the last periods
 *    agree within 0.5 %), the per-cycle amplitude varies by at most
 *    `maxAmplitudeSpread` and the relay spends nearly equal time at +d and -d.
 *
 * @param {SimConfig} config the configuration the experiment ran on
 * @param {{amplitude: number, hysteresis?: number, maxTime: number, periods?: number}} opts
 *   the experiment's options
 * @param {RelayResult} res the experiment's result
 * @returns {RelayValidity}
 */
export function relayValidity(config, opts, res) {
  const T = Math.max(1, Math.round(config.pid.sampleTime / config.dt)) * config.dt;
  let puDelayed = Infinity;
  try {
    puDelayed = relayExperiment(config, { ...opts, extraDelay: 1 }).pu;
  } catch {
    // No sustained oscillation with one more sample of delay: the cycle
    // depended entirely on the timing.
  }
  const samplesPerPeriod = res.pu / T;
  const delaySensitivity = (puDelayed - res.pu) / res.pu;
  const L = RELAY_LIMITS;
  /** @type {string[]} */
  const failures = [];
  /** @param {number} x */
  const pct = (x) => `${(100 * x).toFixed(1)} %`;
  if (samplesPerPeriod < L.minSamplesPerPeriod) {
    failures.push(
      `the period is only ${samplesPerPeriod.toFixed(1)} controller samples (at least ${L.minSamplesPerPeriod} needed)`,
    );
  }
  if (!(Math.abs(delaySensitivity) <= L.maxDelaySensitivity)) {
    failures.push(
      Number.isFinite(delaySensitivity)
        ? `one extra sample of loop delay changed the period by ${pct(Math.abs(delaySensitivity))} (at most ${pct(L.maxDelaySensitivity)} allowed)`
        : "with one extra sample of loop delay there was no sustained oscillation",
    );
  }
  const timingDominated = failures.length > 0;
  const irregular =
    !res.converged || res.amplitudeSpread > L.maxAmplitudeSpread || res.asymmetry > L.maxAsymmetry;
  if (!res.converged) {
    failures.push(
      `the period had not settled (spread ${pct(res.periodSpread)} over the last ${res.periods} cycles)`,
    );
  }
  if (res.amplitudeSpread > L.maxAmplitudeSpread) {
    failures.push(
      `the amplitude varied by ${pct(res.amplitudeSpread)} between cycles (at most ${pct(L.maxAmplitudeSpread)} allowed)`,
    );
  }
  if (res.asymmetry > L.maxAsymmetry) {
    failures.push(
      `the relay spent unequal times high and low (asymmetry ${pct(res.asymmetry)}, at most ${pct(L.maxAsymmetry)} allowed)`,
    );
  }
  return {
    valid: failures.length === 0,
    sampleTime: T,
    samplesPerPeriod,
    puDelayed,
    delaySensitivity,
    timingDominated,
    irregular,
    failures,
  };
}

/**
 * Ziegler-Nichols closed-loop (ultimate sensitivity) rule.
 * @param {number} ku
 * @param {number} pu
 * @param {ControllerType} type
 * @returns {Gains}
 */
export function zieglerNicholsUltimate(ku, pu, type = "pid") {
  if (type === "p") return fromIdeal(0.5 * ku, 0, 0);
  if (type === "pi") return fromIdeal(0.45 * ku, pu / 1.2, 0);
  return fromIdeal(0.6 * ku, pu / 2, pu / 8);
}

/**
 * Tyreus-Luyben rule: a detuned alternative to Ziegler-Nichols for the
 * same Ku, Pu, with less overshoot and slower disturbance rejection.
 * @param {number} ku
 * @param {number} pu
 * @param {ControllerType} type
 * @returns {Gains}
 */
export function tyreusLuyben(ku, pu, type = "pid") {
  if (type === "p") return fromIdeal(0.5 * ku, 0, 0);
  if (type === "pi") return fromIdeal(ku / 3.2, 2.2 * pu, 0);
  return fromIdeal(ku / 2.2, 2.2 * pu, pu / 6.3);
}

/**
 * Ziegler-Nichols open-loop (reaction curve) rule from an FOPDT model.
 * @param {Fopdt} m
 * @param {ControllerType} type
 * @returns {Gains}
 */
export function zieglerNicholsStep(m, type = "pid") {
  const a = (m.K * m.theta) / m.tau;
  if (type === "p") return fromIdeal(1 / a, 0, 0);
  if (type === "pi") return fromIdeal(0.9 / a, 3 * m.theta, 0);
  return fromIdeal(1.2 / a, 2 * m.theta, 0.5 * m.theta);
}

/**
 * Cohen-Coon rule from an FOPDT model.
 * @param {Fopdt} m
 * @param {ControllerType} type
 * @returns {Gains}
 */
export function cohenCoon(m, type = "pid") {
  const r = m.theta / m.tau;
  const k = 1 / (m.K * r);
  if (type === "p") return fromIdeal(k * (1 + r / 3), 0, 0);
  if (type === "pi") {
    return fromIdeal(k * (0.9 + r / 12), (m.theta * (30 + 3 * r)) / (9 + 20 * r), 0);
  }
  return fromIdeal(
    k * (4 / 3 + r / 4),
    (m.theta * (32 + 6 * r)) / (13 + 8 * r),
    (4 * m.theta) / (11 + 2 * r),
  );
}

/**
 * Skogestad's SIMC PI rule for an FOPDT model:
 * Kc = tau / (K (tauc + theta)), Ti = min(tau, 4 (tauc + theta)).
 * The closed-loop time constant tauc defaults to theta ("tight control"),
 * floored at tau / 10 so that a model with no dead time still gets a
 * finite gain.
 * @param {Fopdt} m
 * @param {number} [tauc]
 * @returns {Gains}
 */
export function simc(m, tauc) {
  const tc = tauc ?? Math.max(m.theta, m.tau / 10);
  const kc = m.tau / (m.K * (tc + m.theta));
  return fromIdeal(kc, Math.min(m.tau, 4 * (tc + m.theta)), 0);
}

export const TUNING_METHODS = ["simc", "cohen-coon", "zn-step", "zn-relay", "tyreus-luyben"];
