// Discrete PID controller in the form described by Astrom & Hagglund,
// "Advanced PID Control" (2006), chapter 3 and 13: setpoint weighting,
// filtered derivative, anti-windup, output and rate limits, and bumpless
// manual/automatic transfer.

/**
 * @typedef {"parallel" | "ideal"} PidForm
 * @typedef {"none" | "clamp" | "backcalc"} AntiWindup
 *
 * @typedef {object} PidSettings
 * @property {PidForm} form which set of gains is used
 * @property {number} kp parallel form: proportional gain
 * @property {number} ki parallel form: integral gain (1/s)
 * @property {number} kd parallel form: derivative gain (s)
 * @property {number} kc ideal form: controller gain
 * @property {number} ti ideal form: integral time (s); <= 0 switches the integral off
 * @property {number} td ideal form: derivative time (s)
 * @property {number} n derivative filter: Tf = Td / N; <= 0 means no filter
 * @property {number} b setpoint weight on the proportional term
 * @property {number} c setpoint weight on the derivative term (0 = derivative on measurement)
 * @property {number} uMin
 * @property {number} uMax
 * @property {AntiWindup} antiWindup
 * @property {number} tt back-calculation tracking time (s); <= 0 picks sqrt(Ti Td) or Ti / 2
 * @property {number} rateLimit maximum |du/dt| in output units per second; <= 0 means none
 * @property {number} sampleTime controller sample time (s)
 *
 * @typedef {object} PidTerms values from the latest update
 * @property {number} p
 * @property {number} i integral state before this update's integration
 * @property {number} d
 * @property {number} ff
 * @property {number} v unconstrained output p + i + d + ff
 * @property {number} u output actually applied
 * @property {boolean} saturated u differs from v
 */

/** @type {PidSettings} */
export const DEFAULT_PID = {
  form: "parallel",
  kp: 1,
  ki: 0,
  kd: 0,
  kc: 1,
  ti: 0,
  td: 0,
  n: 10,
  b: 1,
  c: 0,
  uMin: -Infinity,
  uMax: Infinity,
  antiWindup: "backcalc",
  tt: 0,
  rateLimit: 0,
  sampleTime: 0.01,
};

/**
 * Parallel-form gains, whichever form the settings use.
 * @param {PidSettings} s
 * @returns {{kp: number, ki: number, kd: number}}
 */
export function parallelGains(s) {
  if (s.form === "ideal") {
    return { kp: s.kc, ki: s.ti > 0 ? s.kc / s.ti : 0, kd: s.kc * s.td };
  }
  return { kp: s.kp, ki: s.ki, kd: s.kd };
}

/**
 * Ideal-form equivalents of parallel gains. Ti is 0 when there is no integral
 * action; Ti and Td are undefined (NaN) when Kp is 0.
 * @param {{kp: number, ki: number, kd: number}} g
 * @returns {{kc: number, ti: number, td: number}}
 */
export function idealGains(g) {
  return {
    kc: g.kp,
    ti: g.ki > 0 ? g.kp / g.ki : 0,
    td: g.kp !== 0 ? g.kd / g.kp : g.kd === 0 ? 0 : NaN,
  };
}

/**
 * Derivative filter time constant Tf = Td / N. In parallel form Td = Kd / Kp;
 * with Kp = 0 the filter uses Tf = Kd / N instead.
 * @param {PidSettings} s
 * @returns {number}
 */
export function derivativeFilterTime(s) {
  if (!(s.n > 0)) return 0;
  const { kp, kd } = parallelGains(s);
  const td = kp !== 0 ? kd / kp : kd;
  return Math.abs(td) / s.n;
}

/**
 * Tracking time constant for back-calculation anti-windup.
 * @param {PidSettings} s
 * @returns {number}
 */
export function trackingTime(s) {
  if (s.tt > 0) return s.tt;
  const { kp, ki, kd } = parallelGains(s);
  if (!(ki > 0)) return Infinity;
  const ti = kp / ki;
  const td = kp !== 0 ? kd / kp : 0;
  // Astrom & Hagglund suggest Tt = sqrt(Ti Td); fall back to Ti / 2 for PI.
  if (ti > 0 && td > 0) return Math.sqrt(ti * td);
  return ti > 0 ? ti / 2 : ki > 0 ? 1 / ki : Infinity;
}

/**
 * @param {Partial<PidSettings>} [settings]
 * @returns {PidSettings}
 */
export function pidSettings(settings = {}) {
  const s = { ...DEFAULT_PID, ...settings };
  if (!(s.sampleTime > 0)) throw new RangeError("sampleTime must be positive");
  if (!(s.uMax > s.uMin)) throw new RangeError("uMax must be greater than uMin");
  if (!["parallel", "ideal"].includes(s.form)) throw new RangeError(`unknown form "${s.form}"`);
  if (!["none", "clamp", "backcalc"].includes(s.antiWindup)) {
    throw new RangeError(`unknown anti-windup "${s.antiWindup}"`);
  }
  return s;
}

export class Pid {
  /**
   * @param {Partial<PidSettings>} settings
   */
  constructor(settings) {
    /** @type {PidSettings} */
    this.s = pidSettings(settings);
    this.integral = 0;
    this.dState = 0;
    this.edPrev = 0;
    this.uPrev = 0;
    this.manual = false;
    this.manualOutput = 0;
    /** @type {PidTerms} */
    this.terms = { p: 0, i: 0, d: 0, ff: 0, v: 0, u: 0, saturated: false };
  }

  /**
   * Start in steady state: the output is u0, the derivative has no history
   * and the integral holds whatever the proportional term does not supply.
   * @param {number} r
   * @param {number} y
   * @param {number} u0
   * @param {number} [ff]
   */
  reset(r, y, u0, ff = 0) {
    const { kp } = parallelGains(this.s);
    const u = Math.min(this.s.uMax, Math.max(this.s.uMin, u0));
    this.edPrev = this.s.c * r - y;
    this.dState = 0;
    this.integral = u - kp * (this.s.b * r - y) - ff;
    this.uPrev = u;
    this.manualOutput = u;
  }

  /**
   * Change gains or limits while running. The integral is adjusted so that
   * the output does not jump when Kp, b or the form changes (bumpless
   * parameter change, Astrom & Hagglund 2006, section 13.5).
   * @param {Partial<PidSettings>} settings
   * @param {number} r
   * @param {number} y
   */
  update(settings, r, y) {
    const before = parallelGains(this.s).kp * (this.s.b * r - y);
    this.s = pidSettings({ ...this.s, ...settings });
    const after = parallelGains(this.s).kp * (this.s.b * r - y);
    this.integral += before - after;
  }

  /**
   * Switch between manual and automatic. Entering manual keeps the last
   * output; leaving it starts from the integral that has been tracking.
   * @param {boolean} manual
   */
  setManual(manual) {
    if (manual && !this.manual) this.manualOutput = this.uPrev;
    this.manual = manual;
  }

  /**
   * One controller sample.
   * @param {number} r setpoint
   * @param {number} y measurement
   * @param {number} [ff] feed-forward added to the output
   * @returns {number} control signal to hold until the next sample
   */
  step(r, y, ff = 0) {
    const s = this.s;
    const T = s.sampleTime;
    const { kp, ki, kd } = parallelGains(s);
    const tf = derivativeFilterTime(s);

    const e = r - y;
    const p = kp * (s.b * r - y);
    const ed = s.c * r - y;
    // Backward-difference filtered derivative: D = Kd s / (1 + Tf s).
    const d = (tf / (tf + T)) * this.dState + (kd / (tf + T)) * (ed - this.edPrev);
    const i = this.integral;
    const v = p + i + d + ff;

    let u;
    if (this.manual) {
      u = Math.min(s.uMax, Math.max(s.uMin, this.manualOutput));
    } else {
      u = Math.min(s.uMax, Math.max(s.uMin, v));
      if (s.rateLimit > 0) {
        const du = s.rateLimit * T;
        u = Math.min(this.uPrev + du, Math.max(this.uPrev - du, u));
      }
    }

    if (this.manual) {
      // Integral tracks so that switching back to auto gives the same output.
      this.integral = u - p - d - ff;
    } else if (s.antiWindup === "backcalc") {
      const tt = trackingTime(s);
      this.integral += ki * T * e + (Number.isFinite(tt) ? (T / tt) * (u - v) : 0);
    } else if (s.antiWindup === "clamp") {
      // Conditional integration: stop integrating while the output is
      // limited and the error would push it further into the limit.
      const pushingOut = (v > u && e > 0) || (v < u && e < 0);
      if (!pushingOut) this.integral += ki * T * e;
    } else {
      this.integral += ki * T * e;
    }

    this.dState = d;
    this.edPrev = ed;
    this.uPrev = u;
    this.terms = { p, i, d, ff, v, u, saturated: Math.abs(u - v) > 1e-12 };
    return u;
  }
}
