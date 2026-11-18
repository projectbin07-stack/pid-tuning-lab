// Loop frequency response and stability margins.
//
// The loop is L(jw) = C(e^{jwT}) * ZOH(jw) * G(jw) * S(jw) where C is the
// discrete PID exactly as implemented in pid.js (feedback path, so setpoint
// weights and feed-forward do not appear), ZOH is the zero-order hold
// (1 - e^{-jwT}) / (jwT), G is the plant's linear transfer function including
// its dead time, and S is the first-order sensor lag. This is the usual
// continuous-time approximation of a sampled loop: it is accurate below the
// Nyquist frequency pi/T and ignores aliasing.

import { derivativeFilterTime, parallelGains } from "./pid.js";

/** @typedef {import("./plants.js").TransferFunction} TransferFunction */
/** @typedef {import("./pid.js").PidSettings} PidSettings */

/**
 * @typedef {object} Complex
 * @property {number} re
 * @property {number} im
 *
 * @typedef {object} LoopPoint
 * Value of the loop at one frequency, split into a rational part and a pure
 * delay so that the phase of the delay can be added exactly (no unwrapping).
 * @property {Complex} rational
 * @property {number} delay seconds; total phase = arg(rational) - w * delay
 *
 * @typedef {(w: number) => LoopPoint} LoopFunction
 *
 * @typedef {object} Margins
 * @property {number} gainMargin linear factor (Infinity if the phase never reaches -180 deg)
 * @property {number} gainMarginDb
 * @property {number} phaseCrossover rad/s (NaN if none)
 * @property {number} phaseMargin degrees (Infinity if |L| never crosses 1)
 * @property {number} gainCrossover rad/s (NaN if none)
 * @property {number} delayMargin seconds of extra delay the loop tolerates
 *
 * @typedef {object} FrequencyResponse
 * @property {number[]} w rad/s
 * @property {number[]} magDb
 * @property {number[]} phaseDeg unwrapped
 * @property {number[]} re
 * @property {number[]} im
 */

/** @param {number} re @param {number} im @returns {Complex} */
const c = (re, im) => ({ re, im });

/** @param {Complex} a @param {Complex} b @returns {Complex} */
export const cmul = (a, b) => c(a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re);

/** @param {Complex} a @param {Complex} b @returns {Complex} */
export function cdiv(a, b) {
  const den = b.re * b.re + b.im * b.im;
  return c((a.re * b.re + a.im * b.im) / den, (a.im * b.re - a.re * b.im) / den);
}

/** @param {Complex} a @returns {number} */
export const cabs = (a) => Math.hypot(a.re, a.im);

/**
 * Evaluate a polynomial with ascending coefficients at s = jw.
 * @param {number[]} coeffs
 * @param {number} w
 * @returns {Complex}
 */
export function polyAtJw(coeffs, w) {
  let acc = c(0, 0);
  for (let k = coeffs.length - 1; k >= 0; k--) {
    acc = cmul(acc, c(0, w));
    acc = c(acc.re + coeffs[k], acc.im);
  }
  return acc;
}

/**
 * Plant frequency response without its delay.
 * @param {TransferFunction} tf
 * @param {number} w
 * @returns {Complex}
 */
export function tfRational(tf, w) {
  return cdiv(polyAtJw(tf.num, w), polyAtJw(tf.den, w));
}

/**
 * Discrete PID feedback path C(z) at z = e^{jwT}:
 * Kp + Ki T z^-1 / (1 - z^-1) + Kd / (Tf + T) (1 - z^-1) / (1 - a z^-1), a = Tf / (Tf + T).
 * @param {PidSettings} pid
 * @param {number} w
 * @returns {Complex}
 */
export function pidResponse(pid, w) {
  const T = pid.sampleTime;
  const { kp, ki, kd } = parallelGains(pid);
  const tf = derivativeFilterTime(pid);
  const zinv = c(Math.cos(w * T), -Math.sin(w * T));
  const oneMinus = c(1 - zinv.re, -zinv.im);
  let out = c(kp, 0);
  if (ki !== 0) {
    const integ = cdiv(c(ki * T * zinv.re, ki * T * zinv.im), oneMinus);
    out = c(out.re + integ.re, out.im + integ.im);
  }
  if (kd !== 0) {
    const a = tf / (tf + T);
    const g = kd / (tf + T);
    const der = cdiv(c(g * oneMinus.re, g * oneMinus.im), c(1 - a * zinv.re, -a * zinv.im));
    out = c(out.re + der.re, out.im + der.im);
  }
  return out;
}

/**
 * Build the loop function for a PID on a plant transfer function.
 * @param {TransferFunction} plant
 * @param {PidSettings} pid
 * @param {number} [sensorLag] seconds
 * @returns {LoopFunction}
 */
export function pidLoop(plant, pid, sensorLag = 0) {
  const T = pid.sampleTime;
  return (w) => {
    let L = cmul(pidResponse(pid, w), tfRational(plant, w));
    // ZOH = exp(-jwT/2) * sinc(wT/2); the exponential goes into the delay.
    const x = (w * T) / 2;
    const sinc = x === 0 ? 1 : Math.sin(x) / x;
    L = c(L.re * sinc, L.im * sinc);
    if (sensorLag > 0) L = cdiv(L, c(1, w * sensorLag));
    return { rational: L, delay: plant.delay + T / 2 };
  };
}

/**
 * Logarithmically spaced frequencies.
 * @param {number} wMin
 * @param {number} wMax
 * @param {number} n
 * @returns {number[]}
 */
export function logspace(wMin, wMax, n) {
  const a = Math.log10(wMin);
  const b = Math.log10(wMax);
  return Array.from({ length: n }, (_, i) => 10 ** (a + ((b - a) * i) / (n - 1)));
}

/**
 * Evaluate the loop on a grid. The rational phase is unwrapped along the grid
 * and shifted by a multiple of 360 deg so that it starts in (-360, 0]; the
 * delay phase is added analytically.
 * @param {LoopFunction} loop
 * @param {number[]} w
 * @returns {FrequencyResponse}
 */
export function frequencyResponse(loop, w) {
  /** @type {FrequencyResponse} */
  const out = { w, magDb: [], phaseDeg: [], re: [], im: [] };
  let prev = NaN;
  let offset = 0;
  for (let i = 0; i < w.length; i++) {
    const { rational, delay } = loop(w[i]);
    let ph = Math.atan2(rational.im, rational.re);
    if (i === 0) {
      while (ph > 1e-9) ph -= 2 * Math.PI;
      offset = ph - Math.atan2(rational.im, rational.re);
    } else {
      let raw = Math.atan2(rational.im, rational.re) + offset;
      while (raw - prev > Math.PI) {
        raw -= 2 * Math.PI;
        offset -= 2 * Math.PI;
      }
      while (raw - prev < -Math.PI) {
        raw += 2 * Math.PI;
        offset += 2 * Math.PI;
      }
      ph = raw;
    }
    prev = ph;
    const mag = cabs(rational);
    const total = ph - w[i] * delay;
    out.magDb.push(20 * Math.log10(mag));
    out.phaseDeg.push((total * 180) / Math.PI);
    out.re.push(mag * Math.cos(total));
    out.im.push(mag * Math.sin(total));
  }
  return out;
}

/**
 * Refine a crossing of f between w0 and w1 (f changes sign) by bisection in
 * log frequency.
 * @param {(w: number) => number} f
 * @param {number} w0
 * @param {number} w1
 * @returns {number}
 */
function bisect(f, w0, w1) {
  let a = Math.log(w0);
  let b = Math.log(w1);
  const fa = f(w0);
  for (let i = 0; i < 60; i++) {
    const m = 0.5 * (a + b);
    if (Math.sign(f(Math.exp(m))) === Math.sign(fa)) a = m;
    else b = m;
  }
  return Math.exp(0.5 * (a + b));
}

/**
 * Gain and phase margins from the loop frequency response, found on a
 * logarithmic grid and refined by bisection. With several crossings the
 * smallest margin is reported. Phase crossovers below the lowest gain
 * crossover with |L| > 1 are skipped (see below); this is a heuristic, not a
 * full Nyquist stability test.
 * @param {LoopFunction} loop
 * @param {number} wMin
 * @param {number} wMax
 * @param {number} [n]
 * @returns {Margins}
 */
export function stabilityMargins(loop, wMin, wMax, n = 4000) {
  const w = logspace(wMin, wMax, n);
  const fr = frequencyResponse(loop, w);
  const magAt = (/** @type {number} */ x) => 20 * Math.log10(cabs(loop(x).rational));
  /**
   * Phase in degrees continued from grid index i (the rational part is smooth
   * between neighbouring grid points, so take the branch nearest to it).
   * @param {number} x
   * @param {number} i
   */
  const phaseAt = (x, i) => {
    const { rational, delay } = loop(x);
    // Unwrapped rational phase at the grid point.
    const target = (fr.phaseDeg[i] * Math.PI) / 180 + w[i] * delay;
    let ph = Math.atan2(rational.im, rational.re);
    while (ph - target > Math.PI) ph -= 2 * Math.PI;
    while (ph - target < -Math.PI) ph += 2 * Math.PI;
    return ((ph - x * delay) * 180) / Math.PI;
  };

  let gm = Infinity;
  let wpc = NaN;
  let pm = Infinity;
  let wgc = NaN;
  let dm = Infinity;
  let wFirst = Infinity;
  let skipped = Infinity;
  let skippedW = NaN;
  for (let i = 0; i + 1 < n; i++) {
    // Gain crossover: |L| passes through 1 (0 dB).
    if (Math.sign(fr.magDb[i]) !== Math.sign(fr.magDb[i + 1]) && fr.magDb[i] !== 0) {
      const wc = bisect(magAt, w[i], w[i + 1]);
      const ph = phaseAt(wc, i);
      // Distance of the phase from the nearest odd multiple of -180 deg.
      const margin = ((((ph + 180) % 360) + 540) % 360) - 180;
      if (margin < pm) {
        pm = margin;
        wgc = wc;
      }
      const delayMargin = margin > 0 ? (margin * Math.PI) / 180 / wc : 0;
      dm = Math.min(dm, delayMargin);
      wFirst = Math.min(wFirst, wc);
    }
  }
  for (let i = 0; i + 1 < n; i++) {
    // Phase crossover: phase passes through -180 - 360 k deg.
    const k0 = Math.floor((fr.phaseDeg[i] + 180) / 360);
    const k1 = Math.floor((fr.phaseDeg[i + 1] + 180) / 360);
    if (k0 !== k1) {
      const level = 360 * Math.max(k0, k1) - 180;
      const wc = bisect((x) => phaseAt(x, i) - level, w[i], w[i + 1]);
      const g = 1 / cabs(loop(wc).rational);
      // A crossing below the gain crossover where |L| > 1 is a gain
      // *reduction* margin of a conditionally stable loop (or numerical
      // noise of a type-2 loop sitting at -180 deg); it is not reported.
      if (wc < wFirst && g < 1) {
        if (g < skipped) {
          skipped = g;
          skippedW = wc;
        }
        continue;
      }
      if (g < gm) {
        gm = g;
        wpc = wc;
      }
    }
  }
  // A loop with a negative phase margin is unstable; then the skipped
  // crossing is the one that matters (reducing the gain would help).
  if (pm <= 0 && skipped < gm) {
    gm = skipped;
    wpc = skippedW;
  }
  return {
    gainMargin: gm,
    gainMarginDb: 20 * Math.log10(gm),
    phaseCrossover: wpc,
    phaseMargin: pm,
    gainCrossover: wgc,
    delayMargin: dm,
  };
}

/**
 * Default frequency range for a controller sample time: six decades up to
 * the Nyquist frequency.
 * @param {number} sampleTime
 * @returns {[number, number]}
 */
export function defaultRange(sampleTime) {
  const wMax = (0.999 * Math.PI) / sampleTime;
  return [wMax * 1e-6, wMax];
}
