// Canvas drawing: strip charts, Bode and Nyquist plots, plant schematics.
// Colours come from CSS custom properties so light and dark themes work.

/**
 * @typedef {object} Series
 * @property {ArrayLike<number>} x
 * @property {ArrayLike<number>} y
 * @property {string} color CSS colour or var name (--c-a)
 * @property {number[]} [dash]
 * @property {number} [width]
 * @property {string} [label]
 *
 * @typedef {object} Theme
 * @property {string} fg
 * @property {string} muted
 * @property {string} grid
 * @property {string} bg
 * @property {(name: string) => string} color
 */

/** @returns {Theme} */
export function theme() {
  const cs = getComputedStyle(document.documentElement);
  /** @param {string} name */
  const v = (name) => cs.getPropertyValue(name).trim() || "#888";
  return {
    fg: v("--fg"),
    muted: v("--muted"),
    grid: v("--grid"),
    bg: v("--chart-bg"),
    color: (name) => (name.startsWith("--") ? v(name) : name),
  };
}

/**
 * Size the backing store to the element's CSS size times devicePixelRatio.
 * @param {HTMLCanvasElement} canvas
 * @returns {{ctx: CanvasRenderingContext2D, w: number, h: number}}
 */
export function setupCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(50, canvas.clientWidth);
  const h = Math.max(50, canvas.clientHeight);
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
  }
  const ctx = /** @type {CanvasRenderingContext2D} */ (canvas.getContext("2d"));
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w, h };
}

/**
 * About n "nice" tick values covering [min, max].
 * @param {number} min
 * @param {number} max
 * @param {number} n
 * @returns {number[]}
 */
export function niceTicks(min, max, n) {
  const span = max - min;
  if (!(span > 0)) return [min];
  const raw = span / n;
  const mag = 10 ** Math.floor(Math.log10(raw));
  const step = [1, 2, 2.5, 5, 10].map((f) => f * mag).find((s) => s >= raw) ?? 10 * mag;
  const out = [];
  for (let v = Math.ceil(min / step) * step; v <= max + step * 1e-9; v += step) {
    out.push(Math.abs(v) < step * 1e-9 ? 0 : v);
  }
  return out;
}

/** @param {number} v */
function tickLabel(v) {
  const a = Math.abs(v);
  if (a !== 0 && (a >= 1e4 || a < 1e-3)) return v.toExponential(0);
  return String(Number(v.toPrecision(4)));
}

/**
 * @typedef {object} ChartOptions
 * @property {Series[]} series
 * @property {number} xMin
 * @property {number} xMax
 * @property {number} [yMin] automatic from the data when omitted
 * @property {number} [yMax]
 * @property {string} yLabel
 * @property {string} [xLabel]
 * @property {Array<{x: number, label: string}>} [markers] vertical event lines
 * @property {Array<{y: number, color: string}>} [hlines]
 */

/**
 * Line chart with axes, used for the live strips and the step test.
 * @param {HTMLCanvasElement} canvas
 * @param {ChartOptions} o
 */
export function drawChart(canvas, o) {
  const { ctx, w, h } = setupCanvas(canvas);
  const th = theme();
  ctx.fillStyle = th.bg;
  ctx.fillRect(0, 0, w, h);
  const pad = { l: 52, r: 10, t: 18, b: o.xLabel ? 30 : 20 };
  let yMin = o.yMin ?? Infinity;
  let yMax = o.yMax ?? -Infinity;
  if (o.yMin === undefined || o.yMax === undefined) {
    for (const s of o.series) {
      for (let i = 0; i < s.y.length; i++) {
        const x = s.x[i];
        if (x < o.xMin || x > o.xMax || !Number.isFinite(s.y[i])) continue;
        if (o.yMin === undefined) yMin = Math.min(yMin, s.y[i]);
        if (o.yMax === undefined) yMax = Math.max(yMax, s.y[i]);
      }
    }
  }
  if (!Number.isFinite(yMin) || !Number.isFinite(yMax)) {
    yMin = -1;
    yMax = 1;
  }
  if (yMax - yMin < 1e-9) {
    const m = Math.max(Math.abs(yMax), 1e-3) * 0.1;
    yMin -= m;
    yMax += m;
  } else if (o.yMin === undefined || o.yMax === undefined) {
    const m = (yMax - yMin) * 0.08;
    if (o.yMin === undefined) yMin -= m;
    if (o.yMax === undefined) yMax += m;
  }
  const pw = w - pad.l - pad.r;
  const ph = h - pad.t - pad.b;
  /** @param {number} x */
  const X = (x) => pad.l + ((x - o.xMin) / (o.xMax - o.xMin)) * pw;
  /** @param {number} y */
  const Y = (y) => pad.t + (1 - (y - yMin) / (yMax - yMin)) * ph;

  ctx.font = "11px system-ui, sans-serif";
  ctx.lineWidth = 1;
  ctx.strokeStyle = th.grid;
  ctx.fillStyle = th.muted;
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  for (const v of niceTicks(yMin, yMax, Math.max(2, Math.floor(ph / 32)))) {
    const y = Math.round(Y(v)) + 0.5;
    ctx.beginPath();
    ctx.moveTo(pad.l, y);
    ctx.lineTo(pad.l + pw, y);
    ctx.stroke();
    ctx.fillText(tickLabel(v), pad.l - 5, y);
  }
  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  for (const v of niceTicks(o.xMin, o.xMax, Math.max(2, Math.floor(pw / 80)))) {
    const x = Math.round(X(v)) + 0.5;
    ctx.beginPath();
    ctx.moveTo(x, pad.t);
    ctx.lineTo(x, pad.t + ph);
    ctx.stroke();
    ctx.fillText(tickLabel(v), x, pad.t + ph + 4);
  }
  if (o.xLabel) ctx.fillText(o.xLabel, pad.l + pw / 2, h - 13);
  ctx.textAlign = "left";
  ctx.fillStyle = th.fg;
  ctx.fillText(o.yLabel, pad.l, 2);

  for (const m of o.markers ?? []) {
    if (m.x < o.xMin || m.x > o.xMax) continue;
    const x = Math.round(X(m.x)) + 0.5;
    ctx.strokeStyle = th.muted;
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.moveTo(x, pad.t);
    ctx.lineTo(x, pad.t + ph);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = th.muted;
    ctx.fillText(m.label, x + 3, pad.t + 2);
  }
  for (const l of o.hlines ?? []) {
    if (l.y < yMin || l.y > yMax) continue;
    ctx.strokeStyle = th.color(l.color);
    ctx.setLineDash([2, 4]);
    ctx.beginPath();
    ctx.moveTo(pad.l, Y(l.y));
    ctx.lineTo(pad.l + pw, Y(l.y));
    ctx.stroke();
    ctx.setLineDash([]);
  }

  ctx.save();
  ctx.beginPath();
  ctx.rect(pad.l, pad.t, pw, ph);
  ctx.clip();
  for (const s of o.series) {
    ctx.strokeStyle = th.color(s.color);
    ctx.lineWidth = s.width ?? 1.5;
    ctx.setLineDash(s.dash ?? []);
    ctx.beginPath();
    let pen = false;
    // Decimate to about two points per pixel column (min and max).
    const n = s.x.length;
    const stride = Math.max(1, Math.floor(n / (pw * 2)));
    for (let i = 0; i < n; i += stride) {
      let lo = s.y[i];
      let hi = s.y[i];
      for (let j = i + 1; j < Math.min(n, i + stride); j++) {
        lo = Math.min(lo, s.y[j]);
        hi = Math.max(hi, s.y[j]);
      }
      if (!Number.isFinite(lo)) {
        pen = false;
        continue;
      }
      const x = X(s.x[i]);
      if (!pen) ctx.moveTo(x, Y(lo));
      else ctx.lineTo(x, Y(lo));
      if (hi !== lo) ctx.lineTo(x, Y(hi));
      pen = true;
    }
    ctx.stroke();
  }
  ctx.restore();
  ctx.setLineDash([]);
  ctx.strokeStyle = th.muted;
  ctx.lineWidth = 1;
  ctx.strokeRect(pad.l + 0.5, pad.t + 0.5, pw, ph);
}

/**
 * @typedef {object} FreqCurve
 * @property {number[]} w
 * @property {number[]} magDb
 * @property {number[]} phaseDeg
 * @property {number[]} re
 * @property {number[]} im
 * @property {string} color
 * @property {number[]} [dash]
 * @property {import("../core/frequency.js").Margins | null} margins
 */

/**
 * Bode magnitude and phase in one canvas, log frequency axis, with gain and
 * phase margin markers.
 * @param {HTMLCanvasElement} canvas
 * @param {FreqCurve[]} curves
 */
export function drawBode(canvas, curves) {
  const { ctx, w, h } = setupCanvas(canvas);
  const th = theme();
  ctx.fillStyle = th.bg;
  ctx.fillRect(0, 0, w, h);
  if (curves.length === 0) return;
  const pad = { l: 52, r: 10, t: 16, b: 30, gap: 22 };
  const pw = w - pad.l - pad.r;
  const ph = (h - pad.t - pad.b - pad.gap) / 2;
  const wMin = Math.min(...curves.map((c) => c.w[0]));
  const wMax = Math.max(...curves.map((c) => c.w[c.w.length - 1]));
  const lx0 = Math.log10(wMin);
  const lx1 = Math.log10(wMax);
  /** @param {number} x */
  const X = (x) => pad.l + ((Math.log10(x) - lx0) / (lx1 - lx0)) * pw;

  // Magnitude range: clamp to something readable.
  let mMin = Infinity;
  let mMax = -Infinity;
  let pMin = Infinity;
  for (const c of curves) {
    for (const v of c.magDb) {
      mMin = Math.min(mMin, v);
      mMax = Math.max(mMax, v);
    }
    for (const v of c.phaseDeg) pMin = Math.min(pMin, v);
  }
  mMax = Math.min(Math.max(mMax, 20), 120);
  mMin = Math.max(Math.min(mMin, -20), mMax - 160);
  pMin = Math.max(pMin, -720);
  const pMax = 0;
  const panels = [
    { top: pad.t, min: mMin, max: mMax, label: "|L| (dB)", key: /** @type {const} */ ("magDb") },
    {
      top: pad.t + ph + pad.gap,
      min: pMin,
      max: pMax,
      label: "phase (deg)",
      key: /** @type {const} */ ("phaseDeg"),
    },
  ];
  ctx.font = "11px system-ui, sans-serif";
  for (const p of panels) {
    /** @param {number} v */
    const Y = (v) => p.top + (1 - (v - p.min) / (p.max - p.min)) * ph;
    ctx.strokeStyle = th.grid;
    ctx.fillStyle = th.muted;
    ctx.lineWidth = 1;
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    const ticks =
      p.key === "phaseDeg"
        ? niceTicks(p.min, p.max, 4).map((v) => v)
        : niceTicks(p.min, p.max, Math.max(2, Math.floor(ph / 30)));
    for (const v of ticks) {
      const y = Math.round(Y(v)) + 0.5;
      ctx.beginPath();
      ctx.moveTo(pad.l, y);
      ctx.lineTo(pad.l + pw, y);
      ctx.stroke();
      ctx.fillText(tickLabel(v), pad.l - 5, y);
    }
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    for (let e = Math.ceil(lx0); e <= Math.floor(lx1); e++) {
      const x = Math.round(X(10 ** e)) + 0.5;
      ctx.beginPath();
      ctx.moveTo(x, p.top);
      ctx.lineTo(x, p.top + ph);
      ctx.stroke();
      if (p.key === "phaseDeg") ctx.fillText(`1e${e}`, x, p.top + ph + 4);
    }
    // Reference line: 0 dB or -180 deg.
    const ref = p.key === "magDb" ? 0 : -180;
    if (ref >= p.min && ref <= p.max) {
      ctx.strokeStyle = th.muted;
      ctx.setLineDash([4, 3]);
      ctx.beginPath();
      ctx.moveTo(pad.l, Y(ref));
      ctx.lineTo(pad.l + pw, Y(ref));
      ctx.stroke();
      ctx.setLineDash([]);
    }
    ctx.fillStyle = th.fg;
    ctx.textAlign = "left";
    ctx.fillText(p.label, pad.l, p.top - 14);

    ctx.save();
    ctx.beginPath();
    ctx.rect(pad.l, p.top, pw, ph);
    ctx.clip();
    for (const [ci, c] of curves.entries()) {
      ctx.strokeStyle = th.color(c.color);
      ctx.lineWidth = 1.5;
      ctx.setLineDash(c.dash ?? []);
      ctx.beginPath();
      const vals = c[p.key];
      const offset = ci * 14;
      for (let i = 0; i < c.w.length; i++) {
        const x = X(c.w[i]);
        const y = Y(vals[i]);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
      ctx.setLineDash([]);
      // Margin markers.
      const m = c.margins;
      if (m) {
        ctx.strokeStyle = th.color(c.color);
        ctx.lineWidth = 2;
        if (
          p.key === "magDb" &&
          Number.isFinite(m.phaseCrossover) &&
          Number.isFinite(m.gainMarginDb)
        ) {
          const x = X(m.phaseCrossover);
          ctx.beginPath();
          ctx.moveTo(x, Y(0));
          ctx.lineTo(x, Y(-m.gainMarginDb));
          ctx.stroke();
          ctx.fillStyle = th.color(c.color);
          ctx.fillText(
            `GM ${m.gainMarginDb.toFixed(1)} dB`,
            Math.min(x + 4, pad.l + pw - 80),
            Y(0) + 3 + offset,
          );
        }
        if (
          p.key === "phaseDeg" &&
          Number.isFinite(m.gainCrossover) &&
          Number.isFinite(m.phaseMargin)
        ) {
          const x = X(m.gainCrossover);
          ctx.beginPath();
          ctx.moveTo(x, Y(-180));
          ctx.lineTo(x, Y(-180 + m.phaseMargin));
          ctx.stroke();
          ctx.fillStyle = th.color(c.color);
          ctx.fillText(
            `PM ${m.phaseMargin.toFixed(1)} deg`,
            Math.min(x + 4, pad.l + pw - 90),
            Y(-180) - 16 - offset,
          );
        }
      }
    }
    ctx.restore();
    ctx.strokeStyle = th.muted;
    ctx.lineWidth = 1;
    ctx.strokeRect(pad.l + 0.5, p.top + 0.5, pw, ph);
  }
  ctx.fillStyle = th.muted;
  ctx.textAlign = "center";
  ctx.fillText("frequency (rad/s)", pad.l + pw / 2, h - 13);
}

/**
 * Nyquist plot of L(jw) for w > 0 near the critical point, with the unit
 * circle and the -1 point.
 * @param {HTMLCanvasElement} canvas
 * @param {FreqCurve[]} curves
 */
export function drawNyquist(canvas, curves) {
  const { ctx, w, h } = setupCanvas(canvas);
  const th = theme();
  ctx.fillStyle = th.bg;
  ctx.fillRect(0, 0, w, h);
  const R = 2; // visible radius around the origin
  const size = Math.min(w, h) - 20;
  const cx = w / 2 + size * 0.1;
  const cy = h / 2;
  const scale = size / (2 * R);
  /** @param {number} re */
  const X = (re) => cx + re * scale;
  /** @param {number} im */
  const Y = (im) => cy - im * scale;
  ctx.strokeStyle = th.grid;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, cy);
  ctx.lineTo(w, cy);
  ctx.moveTo(cx, 0);
  ctx.lineTo(cx, h);
  ctx.stroke();
  ctx.setLineDash([4, 3]);
  ctx.strokeStyle = th.muted;
  ctx.beginPath();
  ctx.arc(cx, cy, scale, 0, 2 * Math.PI);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = th.fg;
  ctx.font = "11px system-ui, sans-serif";
  ctx.beginPath();
  ctx.arc(X(-1), Y(0), 3.5, 0, 2 * Math.PI);
  ctx.fill();
  ctx.fillText("-1", X(-1) - 14, Y(0) + 4);
  ctx.fillStyle = th.muted;
  ctx.fillText("Re", w - 20, cy - 6);
  ctx.fillText("Im", cx + 4, 12);
  ctx.save();
  ctx.beginPath();
  ctx.rect(0, 0, w, h);
  ctx.clip();
  for (const c of curves) {
    ctx.strokeStyle = th.color(c.color);
    ctx.lineWidth = 1.5;
    ctx.setLineDash(c.dash ?? []);
    ctx.beginPath();
    let pen = false;
    for (let i = 0; i < c.re.length; i++) {
      const re = c.re[i];
      const im = c.im[i];
      // Far outside the view the curve is irrelevant; lift the pen.
      if (Math.hypot(re, im) > 50) {
        pen = false;
        continue;
      }
      if (!pen) ctx.moveTo(X(re), Y(im));
      else ctx.lineTo(X(re), Y(im));
      pen = true;
    }
    ctx.stroke();
    ctx.setLineDash([]);
    const m = c.margins;
    if (m && Number.isFinite(m.phaseMargin) && Number.isFinite(m.gainCrossover)) {
      // Phase-margin point on the unit circle.
      const a = Math.PI + (m.phaseMargin * Math.PI) / 180;
      ctx.fillStyle = th.color(c.color);
      ctx.beginPath();
      ctx.arc(
        X(Math.cos(a)),
        Y(-Math.abs(Math.sin(a)) * Math.sign(m.phaseMargin)),
        4,
        0,
        2 * Math.PI,
      );
      ctx.fill();
    }
    if (m && Number.isFinite(m.gainMargin) && m.gainMargin > 0) {
      ctx.strokeStyle = th.color(c.color);
      const x = X(-1 / m.gainMargin);
      ctx.beginPath();
      ctx.moveTo(x - 4, Y(0) - 4);
      ctx.lineTo(x + 4, Y(0) + 4);
      ctx.moveTo(x - 4, Y(0) + 4);
      ctx.lineTo(x + 4, Y(0) - 4);
      ctx.stroke();
    }
  }
  ctx.restore();
}

/**
 * Small schematic of each plant.
 * @param {HTMLCanvasElement} canvas
 * @param {string} plantId
 */
export function drawSchematic(canvas, plantId) {
  const { ctx, w, h } = setupCanvas(canvas);
  const th = theme();
  ctx.fillStyle = th.bg;
  ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle = th.fg;
  ctx.fillStyle = th.fg;
  ctx.lineWidth = 1.5;
  ctx.font = "12px system-ui, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  const cy = h / 2;
  /** @param {number} x @param {number} y @param {number} bw @param {number} bh @param {string} label */
  const box = (x, y, bw, bh, label) => {
    ctx.strokeRect(x - bw / 2, y - bh / 2, bw, bh);
    ctx.fillText(label, x, y);
  };
  /** @param {number} x0 @param {number} y0 @param {number} x1 @param {number} y1 @param {string} [label] */
  const arrow = (x0, y0, x1, y1, label) => {
    ctx.beginPath();
    ctx.moveTo(x0, y0);
    ctx.lineTo(x1, y1);
    ctx.stroke();
    const a = Math.atan2(y1 - y0, x1 - x0);
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x1 - 7 * Math.cos(a - 0.4), y1 - 7 * Math.sin(a - 0.4));
    ctx.lineTo(x1 - 7 * Math.cos(a + 0.4), y1 - 7 * Math.sin(a + 0.4));
    ctx.closePath();
    ctx.fill();
    if (label) ctx.fillText(label, (x0 + x1) / 2, (y0 + y1) / 2 - 10);
  };
  const s = Math.min(w / 320, h / 120);
  ctx.save();
  ctx.translate(w / 2 - 160 * s, cy - 60 * s);
  ctx.scale(s, s);
  const y = 60;
  if (plantId === "fopdt") {
    arrow(10, y, 60, y, "u");
    box(95, y, 70, 40, "e^-θs");
    arrow(130, y, 180, y);
    box(225, y, 90, 40, "K/(τs+1)");
    arrow(270, y, 310, y, "y");
  } else if (plantId === "msd") {
    ctx.beginPath();
    ctx.moveTo(20, 20);
    ctx.lineTo(20, 100);
    ctx.stroke();
    // spring
    ctx.beginPath();
    ctx.moveTo(20, 45);
    for (let i = 0; i <= 8; i++) ctx.lineTo(40 + i * 12, 45 + (i % 2 ? -8 : 8));
    ctx.lineTo(150, 45);
    ctx.stroke();
    ctx.fillText("k", 85, 28);
    // damper
    ctx.beginPath();
    ctx.moveTo(20, 80);
    ctx.lineTo(80, 80);
    ctx.moveTo(80, 70);
    ctx.lineTo(80, 90);
    ctx.moveTo(70, 68);
    ctx.lineTo(100, 68);
    ctx.moveTo(70, 92);
    ctx.lineTo(100, 92);
    ctx.moveTo(90, 80);
    ctx.lineTo(150, 80);
    ctx.stroke();
    ctx.fillText("c", 85, 105);
    box(185, 62, 70, 64, "m");
    arrow(295, 62, 225, 62, "F = u + d");
    arrow(185, 110, 230, 110);
    ctx.fillText("x", 240, 110);
  } else if (plantId === "motor") {
    arrow(10, y, 50, y, "V");
    box(80, y, 50, 36, "R, L");
    ctx.beginPath();
    ctx.arc(150, y, 24, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.fillText("M", 150, y);
    ctx.beginPath();
    ctx.moveTo(105, y);
    ctx.lineTo(126, y);
    ctx.moveTo(174, y);
    ctx.lineTo(220, y);
    ctx.stroke();
    ctx.beginPath();
    ctx.ellipse(235, y, 10, 28, 0, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.fillText("J", 235, y + 42);
    ctx.fillText("ω", 275, y - 30);
    ctx.beginPath();
    ctx.arc(265, y, 20, -1.2, 1.2);
    ctx.stroke();
    ctx.fillText("TL, b, Tc", 290, y + 42);
  } else if (plantId === "integrating") {
    arrow(10, y, 55, y, "v*");
    box(100, y, 90, 40, "K/(τs+1)");
    arrow(145, y, 195, y, "v");
    box(220, y, 50, 40, "1/s");
    arrow(245, y, 300, y, "x");
  } else if (plantId === "thermal") {
    // heater -> block -> sensor, block -> ambient
    box(50, y, 70, 40, "heater C0");
    box(160, y, 80, 50, "block C1");
    box(270, y, 70, 40, "sensor C2");
    ctx.fillText("R01", 105, y - 34);
    ctx.fillText("R12", 218, y - 34);
    ctx.beginPath();
    ctx.moveTo(85, y);
    ctx.lineTo(120, y);
    ctx.moveTo(200, y);
    ctx.lineTo(235, y);
    ctx.stroke();
    arrow(160, y + 25, 160, y + 55);
    ctx.fillText("Ra → ambient", 215, y + 50);
    arrow(50, 5, 50, y - 20);
    ctx.fillText("Pmax·u", 90, 12);
  } else if (plantId === "arm") {
    const px = 160;
    const py = 30;
    ctx.beginPath();
    ctx.arc(px, py, 6, 0, 2 * Math.PI);
    ctx.stroke();
    const q = 0.9;
    const ex = px + 70 * Math.sin(q);
    const ey = py + 70 * Math.cos(q);
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.lineTo(ex, ey);
    ctx.stroke();
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(ex, ey, 9, 0, 2 * Math.PI);
    ctx.fill();
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.lineTo(px, py + 85);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.beginPath();
    ctx.arc(px, py, 30, Math.PI / 2 - q, Math.PI / 2);
    ctx.stroke();
    ctx.fillText("q", px + 14, py + 42);
    ctx.fillText("m", ex + 18, ey + 4);
    ctx.fillText("l", (px + ex) / 2 + 12, (py + ey) / 2 - 6);
    arrow(ex, ey + 12, ex, ey + 40);
    ctx.fillText("g", ex + 10, ey + 34);
    ctx.fillText("τ", px - 22, py - 8);
  }
  ctx.restore();
}
