// Time-domain performance measures computed from a simulated trace. All of
// them use the true plant output, not the noisy measurement.

/** @typedef {import("./sim.js").Trace} Trace */

/**
 * @typedef {object} StepMetrics
 * @property {number} riseTime 10 % to 90 % of the step, seconds (NaN if never reached)
 * @property {number} settlingTime from the step until y stays within the band, seconds (NaN if not settled)
 * @property {number} overshoot peak beyond the new setpoint, percent of the step
 * @property {number} steadyStateError mean r - y over the last 10 % of the window
 * @property {number} iae integral of |e| dt
 * @property {number} ise integral of e^2 dt
 * @property {number} itae integral of (t - t_step) |e| dt
 * @property {number} effort integral of |u| dt
 * @property {number} totalVariation sum of |u[k] - u[k-1]|
 * @property {number} saturationTime seconds with u at a limit
 *
 * @typedef {object} DisturbanceMetrics
 * @property {number} peakDeviation largest |r - y| after the disturbance
 * @property {number} iae integral of |e| dt after the disturbance
 * @property {number} recoveryTime until |r - y| stays within the band, seconds (NaN if not recovered)
 */

/**
 * Time at which `values` first reaches `level` going in direction `sign`,
 * linearly interpolated between samples.
 * @param {number[]} t
 * @param {number[]} values
 * @param {number} level
 * @param {number} from
 * @param {number} to exclusive
 * @returns {number}
 */
function crossing(t, values, level, from, to) {
  for (let k = from; k < to; k++) {
    if (values[k] >= level) {
      if (k === from) return t[k];
      const f = (level - values[k - 1]) / (values[k] - values[k - 1]);
      return t[k - 1] + f * (t[k] - t[k - 1]);
    }
  }
  return NaN;
}

/**
 * @param {number[]} t
 * @param {number} time
 * @returns {number} first index with t >= time
 */
function indexAt(t, time) {
  let k = 0;
  while (k < t.length && t[k] < time - 1e-12) k++;
  return k;
}

/**
 * Metrics of a setpoint step from r0 to r1 at `stepTime`, over the samples
 * before `endTime` (for example the time a disturbance hits).
 * @param {Trace} trace
 * @param {{stepTime: number, endTime: number, r0: number, r1: number, band?: number, uMin?: number, uMax?: number}} opts
 *   band is the settling band as a fraction of the step (default 0.02)
 * @returns {StepMetrics}
 */
export function stepMetrics(trace, opts) {
  const { t, y, r, u } = trace;
  const band = opts.band ?? 0.02;
  const from = indexAt(t, opts.stepTime);
  const to = Math.min(t.length, indexAt(t, opts.endTime));
  if (to - from < 2) throw new RangeError("window must contain at least two samples");
  const delta = opts.r1 - opts.r0;
  if (delta === 0) throw new RangeError("step size must be non-zero");

  // Normalised response: 0 at the old setpoint, 1 at the new one.
  const z = y.map((v) => (v - opts.r0) / delta);
  const t10 = crossing(t, z, 0.1, from, to);
  const t90 = crossing(t, z, 0.9, from, to);

  let peak = -Infinity;
  for (let k = from; k < to; k++) peak = Math.max(peak, z[k]);

  let lastOutside = -1;
  for (let k = from; k < to; k++) if (Math.abs(z[k] - 1) > band) lastOutside = k;
  let settlingTime;
  if (lastOutside === to - 1) settlingTime = NaN;
  else if (lastOutside < 0) settlingTime = 0;
  else settlingTime = t[lastOutside + 1] - opts.stepTime;

  const tailFrom = Math.max(from, to - Math.max(1, Math.round((to - from) / 10)));
  let ssSum = 0;
  for (let k = tailFrom; k < to; k++) ssSum += r[k] - y[k];

  let iae = 0;
  let ise = 0;
  let itae = 0;
  let effort = 0;
  let tv = 0;
  let sat = 0;
  const uMin = opts.uMin ?? -Infinity;
  const uMax = opts.uMax ?? Infinity;
  const tol = 1e-9 * Math.max(1, Math.abs(uMin), Math.abs(uMax));
  for (let k = from; k < to; k++) {
    const h = k + 1 < t.length ? t[k + 1] - t[k] : 0;
    const e = r[k] - y[k];
    iae += Math.abs(e) * h;
    ise += e * e * h;
    itae += (t[k] - opts.stepTime) * Math.abs(e) * h;
    effort += Math.abs(u[k]) * h;
    if (k > from) tv += Math.abs(u[k] - u[k - 1]);
    if (u[k] >= uMax - tol || u[k] <= uMin + tol) sat += h;
  }

  return {
    riseTime: t90 - t10,
    settlingTime,
    overshoot: Math.max(0, peak - 1) * 100,
    steadyStateError: ssSum / (to - tailFrom),
    iae,
    ise,
    itae,
    effort,
    totalVariation: tv,
    saturationTime: sat,
  };
}

/**
 * Metrics of the response to a load disturbance at `time`, until the end of
 * the trace. `band` is absolute, in output units.
 * @param {Trace} trace
 * @param {{time: number, band: number}} opts
 * @returns {DisturbanceMetrics}
 */
export function disturbanceMetrics(trace, opts) {
  const { t, y, r } = trace;
  const from = indexAt(t, opts.time);
  if (t.length - from < 2) throw new RangeError("no samples after the disturbance");
  let peak = 0;
  let iae = 0;
  let lastOutside = -1;
  for (let k = from; k < t.length; k++) {
    const e = Math.abs(r[k] - y[k]);
    peak = Math.max(peak, e);
    if (k + 1 < t.length) iae += e * (t[k + 1] - t[k]);
    if (e > opts.band) lastOutside = k;
  }
  let recoveryTime;
  if (lastOutside === t.length - 1) recoveryTime = NaN;
  else if (lastOutside < 0) recoveryTime = 0;
  else recoveryTime = t[lastOutside + 1] - opts.time;
  return { peakDeviation: peak, iae, recoveryTime };
}
