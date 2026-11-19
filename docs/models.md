# Models, controller and methods

This document describes exactly what the code computes. Every plant is a
simulated model with illustrative parameters. None of them was fitted to, or
checked against, a real motor, heater or mechanism.

Notation: `u` is the controller output (plant input), `y` the plant output,
`d` the load disturbance, `r` the setpoint. All coefficients of transfer
functions in the code are stored in ascending powers of `s`.

## Simulation

Source: `src/core/sim.js`, `src/core/integrate.js`.

- **Integration.** Each plant is a set of ODEs `dx/dt = f(x, u, d)`. The
  simulator advances them with the classical fourth-order Runge-Kutta method
  at a fixed plant interval `dt` (optionally split into sub-steps). `u` and
  `d` are held constant over each interval. The test suite checks the
  observed order of convergence (4.0 +/- 0.15) on a problem with a known
  solution.
- **Dead time.** A ring buffer delays the plant input by `round(theta / dt)`
  samples, so dead time is exact when it is a multiple of `dt` and rounded
  otherwise.
- **Sampling.** The controller runs every `Ts` (rounded to a whole number of
  plant intervals) and its output is held between samples (zero-order hold).
- **Sensor chain**, applied in this order: first-order lag with time
  constant `tau_s` (exact discretisation per plant interval), white Gaussian
  noise drawn from a seeded generator (mulberry32 + Box-Muller) at each
  controller sample, then mid-tread quantisation to a step `q`. The
  controller sees only this measurement; all metrics use the true output.
- **Determinism.** Same settings and seed give bit-identical traces in the
  browser and the CLI.

## Plants

Source: `src/core/plants.js`. Default parameter values are in the UI and
`node bin/pid-tuning-lab.js plants --json`.

### First order plus dead time (`fopdt`)

```
tau dy/dt = -y + K (u(t - theta) + d)
G(s) = K e^(-theta s) / (tau s + 1)
```

Defaults K = 2, tau = 5 s, theta = 1 s. The model most tuning rules assume.
The disturbance enters at the plant input (input units) and is not delayed.

### Mass-spring-damper (`msd`)

```
m x'' + c x' + k x = u + d
G(s) = 1 / (m s^2 + c s + k),  omega_n = sqrt(k/m),  zeta = c / (2 sqrt(k m))
```

Defaults m = 1 kg, c = 0.8 N s/m, k = 4 N/m: omega_n = 2 rad/s, zeta = 0.2
(underdamped). Larger `c` gives critically damped (`c = 2 sqrt(k m)`) or
overdamped behaviour.

### DC motor, speed output (`motor`)

```
L di/dt = V - R i - Ke w
J dw/dt = Kt i - b w - Tc tanh(w / ws) - TL
G(s) = w/V = Kt / ((L s + R)(J s + b) + Kt Ke)       (Tc = 0)
```

Defaults R = 2 ohm, L = 1 mH, Kt = Ke = 0.02, J = 2e-5 kg m^2,
b = 1e-5 N m s, Tc = 4 mN m, ws = 0.5 rad/s, supply +/- 12 V (the controller
output limits). Electrical time constant L/R = 0.5 ms, mechanical about
100 ms. Coulomb friction is smoothed with `tanh(w / ws)` so the right-hand
side stays continuous for RK4; the smoothing speed `ws` is a modelling
choice, not a physical constant, and the model has no true stiction
(stick-slip). The transfer function used for margins leaves Coulomb friction
out. The disturbance is a load torque that opposes positive speed.

### Integrating plant (`integrating`)

```
tau dv/dt = -v + K (u + d)
dx/dt = v
G(s) = K / (s (tau s + 1))
```

An axis whose velocity loop is already closed: the controller commands a
velocity and measures position. Defaults K = 1, tau = 0.2 s, velocity
command limited to +/- 0.5 m/s. There is no steady state for a constant
input, so the step-test tuning methods refuse this plant.

### Thermal, hotend style (`thermal`)

Three thermal capacities in a chain: heater cartridge (`Th`, C0), aluminium
block (`T1`, C1, losing heat to ambient through Ra) and thermistor (`T2`,
C2, measured):

```
C0 dTh/dt = Pmax u - (Th - T1)/R01
C1 dT1/dt = (Th - T1)/R01 + d - (T1 - Ta)/Ra - (T1 - T2)/R12
C2 dT2/dt = (T1 - T2)/R12
```

With `A = C2 R12 s + 1`, `B = C0 R01 s + 1` and
`g = 1/Ra + 1/R12 + 1/R01`, the transfer function from heater power to
sensor temperature (deviation from ambient) is

```
T2 / (Pmax u) = 1 / (A B (C1 s + g) - B/R12 - A/R01)
```

Defaults Pmax = 40 W, C0 = 3 J/K, R01 = 0.8 K/W, C1 = 8 J/K, Ra = 6 K/W,
C2 = 0.5 J/K, R12 = 4 K/W, Ta = 25 degC. Steady state `T2 = Ta + Pmax Ra u`,
so the maximum reachable temperature is 265 degC. `u` is a duty cycle limited
to [0, 1]: the heater cannot cool, which makes integrator windup easy to
provoke on a large setpoint step. The disturbance is a heat flow into the
block (negative = a part-cooling fan). Real hotends have a nonlinear
thermistor, radiation losses and PWM heating; none of that is modelled.

### Robot arm joint (`arm`, nonlinear)

```
J q'' = tau + d - b q' - m g l sin(q),   J = m l^2 + Jm
linearised about q0:  G(s) = 1 / (J s^2 + b s + m g l cos(q0))
```

A single link with a point mass `m` at length `l`, driven by a
torque-limited joint (default +/- 2.5 N m, while holding the link horizontal
takes m g l = 1.47 N m). `q = 0` hangs straight down. Gravity makes the
effective stiffness depend on the angle, so a controller tuned at one angle
behaves differently at another. Bode, Nyquist and margins for this plant use
the model linearised about the setpoint and are labelled as such.

## Controller

Source: `src/core/pid.js`. The structure follows Astrom and Hagglund,
_Advanced PID Control_ (ISA, 2006), chapters 3 and 13.

Continuous form:

```
u = Kp (b r - y) + Ki integral(r - y) dt + Kd d/dt (c r - y) filtered by 1/(1 + Tf s) + ff
```

- **Forms.** Parallel gains (Kp, Ki, Kd) or ideal/ISA gains (Kc, Ti, Td)
  with `Kp = Kc`, `Ki = Kc/Ti`, `Kd = Kc Td`. Both produce identical outputs
  (tested). `Ti <= 0` switches integral action off.
- **Derivative filter.** `Tf = Td / N` (with `Td = Kd/Kp`; if Kp = 0,
  `Tf = Kd/N`). `N <= 0` disables the filter. Discretised with the backward
  difference, which is stable for every Tf:
  `D[k] = Tf/(Tf+T) D[k-1] + Kd/(Tf+T) (ed[k] - ed[k-1])`.
- **Setpoint weights.** `b` on the proportional term, `c` on the derivative
  term. The default `c = 0` is derivative on measurement (no derivative kick
  on setpoint steps). The weights do not change the feedback loop, so they do
  not change the margins.
- **Integral.** Forward Euler: `I[k+1] = I[k] + Ki T e[k]`.
- **Output limits and rate limit.** `u` is clamped to [umin, umax], then its
  change per sample is limited to `rate * T`.
- **Anti-windup.**
  - _Clamping_ (conditional integration): the integral is frozen while the
    output is limited and the error would drive it further into the limit.
  - _Back-calculation_: `I[k+1] = I[k] + Ki T e[k] + (T/Tt)(u[k] - v[k])`,
    where `v` is the unconstrained output and `u` the applied one. The default
    `Tt = sqrt(Ti Td)` (or `Ti/2` without derivative action), as suggested by
    Astrom and Hagglund.
  - _None_ is available to show what windup looks like.
- **Bumpless transfer.** In manual mode the operator sets the output and the
  integral tracks `I = u_manual - P - D - ff`, so the first automatic sample
  continues from the manual value. Entering manual keeps the last output.
  Changing gains while running adjusts the integral so that a change of Kp
  or b does not step the output.
- **Feed-forward.** Optional static feed-forward `ff = w * u_ss(r)`, where
  `u_ss` is the plant's steady-state input for output `r` (for example the
  gravity torque `m g l sin r` for the arm, or `(r - Ta)/(Ra Pmax)` for the
  hotend) and `w` in [0, 1] is the weight. It uses the model's own
  parameters, so it is a perfect model inverse; a real one would not be.

## Metrics

Source: `src/core/metrics.js`. Computed on the true plant output over the
window from the setpoint step to the disturbance (or the end).

| Metric             | Definition                                                                              |
| ------------------ | --------------------------------------------------------------------------------------- |
| Rise time          | 10 % to 90 % of the step, linearly interpolated                                         |
| Settling time      | from the step until `y` stays within +/- 2 % of the step (band configurable in the API) |
| Overshoot          | peak beyond the new setpoint, percent of the step                                       |
| Steady-state error | mean `r - y` over the last 10 % of the window                                           |
| IAE, ISE, ITAE     | integrals of `                                                                          | e             | `, `e^2`, `(t - t_step)                                          | e   | `   |
| Control effort     | integral of `                                                                           | u             | dt`                                                              |
| Total variation    | sum of `                                                                                | u[k] - u[k-1] | `; shows how much noise reaches the actuator                     |
| Saturation time    | time with `u` at a limit                                                                |
| Disturbance        | peak `                                                                                  | r - y         | `, IAE and recovery time into a band of 2 % of the setpoint step |

`n/a` means the quantity was never reached in the window (for example a
response that never settles).

## Stability margins

Source: `src/core/frequency.js`. The loop transfer function is evaluated as

```
L(jw) = C(e^{jwT}) * ZOH(jw) * G(jw) * 1/(1 + jw tau_s)
ZOH(jw) = (1 - e^{-jwT}) / (jwT) = e^{-jwT/2} sinc(wT/2)
```

where `C(z)` is the discrete PID feedback path exactly as implemented
(proportional, forward-Euler integral, backward-difference filtered
derivative). This is the standard continuous approximation of a sampled
loop: valid below the Nyquist frequency `pi/T`, without aliasing. The plant's
dead time and the hold's half-sample delay are added to the phase
analytically; the rational part's phase is unwrapped on a 4000-point
logarithmic grid spanning six decades below Nyquist.

- **Gain margin**: `1/|L|` where the phase crosses -180 deg (+ multiples of
  360 deg), refined by bisection; the smallest is reported.
- **Phase margin**: `180 + arg L` at `|L| = 1`; the smallest is reported.
- **Delay margin**: phase margin (rad) / gain-crossover frequency.

Phase crossings below the gain crossover where `|L| > 1` (gain-reduction
margins of conditionally stable loops, or a type-2 loop whose phase sits on
-180 deg at low frequency, such as the arm held horizontally) are not
reported as the gain margin unless the phase margin is negative. This is a
heuristic, not a Nyquist encirclement count; for an open-loop unstable plant
(none of the built-in ones is) the margins would not establish stability.

Tests compare the result with hand-derived values: an integrator with delay
(`PM = 90 deg - K theta`, `GM = pi/(2 theta K)`), `K/(s+1)^3` (`GM = 8/K` at
`sqrt 3` rad/s), and P control of an FOPDT plant.

## Identification and tuning

Source: `src/core/tuning.js`, `src/core/lab.js`. The experiments run on the
simulated plant with the configured sensor noise, quantisation and limits.

### Step test and FOPDT fit

The plant rests at the initial setpoint; the output is averaged over a
baseline (10 % of the test) to estimate `y0`; then the input steps by the
amount that would move the output to the target setpoint in steady state.

- **Two-point method** (Smith): from the times `t28` and `t63` at which the
  response reaches 28.3 % and 63.2 % of its final change,
  `tau = 1.5 (t63 - t28)`, `theta = t63 - tau`, `K = dy/du`. Fast and
  transparent, but it uses two points of a noisy curve and needs the
  response to have settled (the final value is the mean of the last 5 %).
- **Least squares**: minimises the sum of squared errors between the
  measurement and `K du (1 - exp(-(t - theta)/tau))`. For fixed
  `(tau, theta)` the optimal K has a closed form; `(log tau, theta)` are
  found by a coarse grid and refined with Nelder-Mead. Much less sensitive to
  noise than the two-point method (tested with noise), but still a
  first-order fit: higher-order or oscillatory plants are approximated, and
  the dead time absorbs the extra lags.

Warnings are attached when the fit is poor (RMS error over 3 % of the
response), when the output had not settled, and when `theta/tau < 0.05` for
the rules that then give very high gains.

### Tuning rules

| Method                                                                       | Needs      | Rule (Kc, Ti, Td)                                                                                                                                        | Known weaknesses                                                                                               |
| ---------------------------------------------------------------------------- | ---------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| `simc` (Skogestad 2003)                                                      | FOPDT fit  | PI: `Kc = tau / (K (tauc + theta))`, `Ti = min(tau, 4 (tauc + theta))`, `tauc = theta` (floored at `tau/10` here)                                        | Model-based; quality depends on the fit. PI only in this implementation (SIMC PID needs a second-order model). |
| `cohen-coon` (1953)                                                          | FOPDT fit  | PID: `Kc = (tau/(K theta)) (4/3 + theta/(4 tau))`, `Ti = theta (32 + 6 r)/(13 + 8 r)`, `Td = 4 theta/(11 + 2 r)`, `r = theta/tau`; PI and P variants too | Designed for quarter-amplitude decay: aggressive, small margins, very high gains when `theta/tau` is small.    |
| `zn-step` (Ziegler-Nichols 1942, reaction curve)                             | FOPDT fit  | PID: `Kc = 1.2 tau/(K theta)`, `Ti = 2 theta`, `Td = theta/2`                                                                                            | Same quarter-decay target as Cohen-Coon; oscillatory, sensitive to the dead-time estimate.                     |
| `zn-relay` (Ziegler-Nichols ultimate sensitivity, via Astrom-Hagglund relay) | Relay test | PID: `Kc = 0.6 Ku`, `Ti = Pu/2`, `Td = Pu/8`; PI `0.45 Ku, Pu/1.2`; P `0.5 Ku`                                                                           | Aggressive (quarter decay). Ku from the describing function is approximate (see below).                        |
| `tyreus-luyben` (1992)                                                       | Relay test | PID: `Kc = Ku/2.2`, `Ti = 2.2 Pu`, `Td = Pu/6.3`; PI `Ku/3.2, 2.2 Pu`                                                                                    | Deliberately detuned: more robust, slower disturbance rejection.                                               |

### Relay autotune

Astrom and Hagglund (1984): the loop is closed through a relay
`u = u0 +/- d` around the operating point (the target setpoint, with the
plant started at rest there). If the plant has enough phase lag, a limit
cycle appears near the frequency where the loop phase is -180 deg. The
experiment runs until the last four periods agree within 0.5 %, then

```
Pu = mean period,   Ku = 4 d / (pi sqrt(a^2 - eps^2))
```

where `a` is the measured output amplitude and `eps` the relay hysteresis
(set to `3 * noise std + quantisation step` in the app so that noise does
not cause chattering). This `Ku` is the describing-function estimate and is
what the rules use by default.

The experiment also computes a **first-harmonic estimate**. Over the same
whole periods it takes the first Fourier component of the measured output,
`Y1`, and of the relay output, `U1`. In a periodic steady state a linear
plant maps each harmonic of its input to the same harmonic of its output
through its frequency response, so `Y1 / U1 = G(j w_osc)` exactly, with
`w_osc = 2 pi / Pu`, whatever the waveform. Then
`Ku_harmonic = 1 / |Y1 / U1|`. This removes the describing function's
assumption that the output is a sinusoid. It is still the gain at the
oscillation frequency, which can differ from the true ultimate frequency
(see below). CLI `--ku harmonic` and the app's "Ku: first harmonic" option
make the rules use it. When the two estimates differ by more than 5 %, the
result carries a note that gives both values.

#### Is the relay result meaningful? (validity test)

A relay oscillates wherever the _loop_ phase reaches -180 deg, and the loop
includes the controller: sampling and the zero-order hold add roughly one
sample time `T` of delay (`T/2` from the hold, Astrom and Wittenmark 1997,
plus up to one sample before the relay sees a sign change). If the plant
alone never reaches -180 deg, the oscillation exists only because of that
delay. `Pu` and `Ku` then describe the controller's timing, not the plant.
Ziegler-Nichols or Tyreus-Luyben gains computed from them change with the
sample time and are usually too aggressive. Low-order plants without dead
time are like this (at most two lags, or an integrator and one lag): the
motor, the mass-spring-damper, the integrating axis and the arm. Their
phase approaches -180 deg only as the frequency goes to infinity.

`relayValidity` (`src/core/tuning.js`) checks any relay experiment with
measured data only, so the same procedure would work on a real bench:

1. **Delay sensitivity.** The experiment is repeated with the relay acting
   on the measurement one controller sample older (a known extra delay
   `T`). Near the oscillation frequency, extra delay `T` shifts the
   frequency by about `dw / w = -T / D`, where `D = -d(arg L)/dw` is the
   loop's phase slope there (its group delay). If the plant sets the cycle,
   `D` is dominated by the plant (for example about `theta + tau/(1 + w^2 tau^2)`
   for an FOPDT) and the period barely moves. If the loop delay sets it,
   `D` is of the order of `T` and the period grows by a large fraction. The
   relative change `(Pu' - Pu) / Pu` is therefore an estimate of the share of
   the loop's phase slope that one sample accounts for. The test requires it
   to be at most **25 %**. This threshold is a judgement call. On the
   built-in plants the values are bimodal (table below): at most 7 % where
   the plant has its own -180 deg crossing, and at least 80 % where it does
   not.
2. **Samples per period.** `Pu >= 10 T`. With fewer samples the period is
   resolved to worse than 10 % (so criterion 1 cannot resolve 25 % reliably),
   and the ZN derivative time `Pu/8` is about one sample.
3. **A steady, symmetric cycle.** The last four periods agree within 0.5 %,
   the per-cycle amplitude varies by at most 5 %, and the relay spends nearly
   equal time high and low (`|2 t_high / P - 1| <= 10 %`). If not, `Pu` and
   `a` are not well defined. Typical causes are a nonlinear plant (the arm)
   or a load or bias that the relay is not centred on.

If any criterion fails, the CLI prints `validity FAILED` and a warning, and
the app shows the warning under the tuning result. The warning names the
failed criteria, says the gains are not trustworthy and suggests SIMC or
Cohen-Coon from a step test. For a plant that is not self-regulating it
says instead that the step-test methods do not apply. The gains are still
computed and applied, so they can be inspected.

Values at the default settings (CLI `tune --method zn-relay`):

| Plant         | `T`     | `Pu`     | samples / period | `dPu/Pu` with +1 sample | Result                                                     |
| ------------- | ------- | -------- | ---------------- | ----------------------- | ---------------------------------------------------------- |
| `fopdt`       | 0.05 s  | 3.8 s    | 76               | 5.3 %                   | passes                                                     |
| `thermal`     | 0.1 s   | 11.6 s   | 116              | 6.9 %                   | passes                                                     |
| `motor`       | 1 ms    | 4 ms     | 4                | 100 %                   | fails (delay sensitivity, samples per period)              |
| `msd`         | 0.01 s  | 0.48 s   | 48               | 87.5 %                  | fails (delay sensitivity)                                  |
| `integrating` | 0.01 s  | 0.2 s    | 20               | 80 %                    | fails (delay sensitivity)                                  |
| `arm`         | 0.005 s | 0.3275 s | 65.5             | 85.5 %                  | fails (delay sensitivity, period and amplitude not steady) |

The plant models confirm the split. The FOPDT model reaches -180 deg
through its dead time and the hotend through its three lags, at a finite
frequency. The other four never do on their own. The result moves with the
sample time as expected (tested). On the FOPDT plant the delay sensitivity
rises from 3.2 % at `T = 0.03 s` through 5.3 % (0.05 s) to 20 % (0.2 s), and
the test fails at 0.3 s, where one sample is 30 % of the dead time. On the
motor, `Pu` is 4 samples at every sample time from 0.5 ms to 4 ms and 8
samples at 0.2 ms. It is set by `T`, and none of these sample times makes
the test pass. The mass-spring-damper also still fails with a controller
five times faster (65 %).

#### Accuracy of Ku when the test passes

- **Describing-function error.** The describing function treats the relay
  as if its input were a sinusoid of amplitude `a`. When the plant does not
  filter the square wave well, the output is not a sinusoid and `Ku` is
  biased. For an FOPDT plant and an ideal continuous relay the limit cycle
  can be solved exactly. The output dips for `theta` after each switch and
  then relaxes exponentially, which gives
  `a = K d (1 - e^(-theta/tau))` and `Pu = 2 theta + 2 tau ln(2 - e^(-theta/tau))`
  (derivation: integrate the first-order response over the dead time and
  the following half period; see Yu 2006 for this exact relay analysis).
  With the defaults (K = 2, tau = 5 s, theta = 1 s) the describing function
  gives `Ku = 4 / (pi K (1 - e^-0.2)) = 3.51` and `Pu = 3.67 s`. The model's
  true ultimate point is `Ku = 4.25`, `Pu = 3.72 s`, so the estimate is 17 %
  low from the approximation alone. Because `theta/tau` is small, the output
  is close to a triangle wave, whose fundamental is `8/pi^2 = 0.81` times its
  peak. That is where most of the error comes from. The simulation
  reproduces both formulas within 1 % with a 10 ms controller (tested).
- **Sampling.** At the default `T = 0.05 s` the hold adds about `T/2` of
  delay. With that delay the loop's own ultimate point is `Ku = 4.16`,
  `Pu = 3.81 s`. The relay measures `Pu = 3.8 s` and describing-function
  `Ku = 3.39`, 18.5 % low. The first-harmonic estimate is 4.16, which
  matches to within 0.2 %.
- **Hysteresis.** With hysteresis the oscillation settles where the loop
  phase is `-180 deg + asin(eps/a)`, not at the ultimate point (tested).
  `4 d / (pi sqrt(a^2 - eps^2))` is then `1/|G|` at that slightly lower
  frequency, and so is the first-harmonic estimate.
- **Other plants.** On `1/(s+1)^3` (2 ms sample time) the describing
  function gives 7.78 and the first harmonic 7.76, both about 3 % below the
  true 8. The oscillation period is 3.68 s rather than 3.63 s, so both
  values are `1/|G|` at a slightly lower frequency (`1/|G|` = 7.75 there).
  On the default hotend the two estimates are 0.315 and 0.313, against 0.322
  at the model's ultimate frequency (Pu 11.4 s versus the measured 11.6 s).

Nothing in the lab corrects for the period error. The describing-function
value stays the default, so v0.1.0 results do not change. The note in the
output gives the expected size of the error for the experiment at hand.

## References

- K. J. Astrom, T. Hagglund, _Advanced PID Control_, ISA, 2006.
- K. J. Astrom, T. Hagglund, "Automatic tuning of simple regulators with
  specifications on phase and amplitude margins", _Automatica_ 20(5), 1984.
- S. Skogestad, "Simple analytic rules for model reduction and PID controller
  tuning", _Journal of Process Control_ 13, 2003.
- G. H. Cohen, G. A. Coon, "Theoretical consideration of retarded control",
  _Trans. ASME_ 75, 1953.
- J. G. Ziegler, N. B. Nichols, "Optimum settings for automatic controllers",
  _Trans. ASME_ 64, 1942.
- B. D. Tyreus, W. L. Luyben, "Tuning PI controllers for integrator/dead time
  processes", _Ind. Eng. Chem. Res._ 31, 1992.
- C. L. Smith, _Digital Computer Process Control_, Intext, 1972 (two-point
  FOPDT method).
- K. J. Astrom, B. Wittenmark, _Computer-Controlled Systems: Theory and
  Design_, 3rd ed., Prentice Hall, 1997 (zero-order hold as a half-sample
  delay; choice of sample time).
- C.-C. Yu, _Autotuning of PID Controllers: A Relay Feedback Approach_,
  2nd ed., Springer, 2006 (exact relay oscillation of FOPDT processes and
  the accuracy of the describing-function estimate).
