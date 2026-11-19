# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/).

## [0.1.1] - 2026-11-19

### Fixed

- Relay autotune (`zn-relay`, `tyreus-luyben`) presented relay-derived gains
  without any warning on the mass-spring-damper, integrating axis and arm
  plants. These plants never reach -180 deg of phase on their own, so the
  relay oscillation is set by the controller's sampling delay and the
  gains are not valid. Version 0.1.0 warned only on the motor, through a
  "period under 10 samples" check that the other three pass. The arm also
  got a "not settled" warning, but it did not say the gains were invalid.

### Added

- A general relay validity check (`relayValidity` in `src/core/tuning.js`)
  for any plant. It repeats the relay test with one extra controller sample
  of delay and fails when the period changes by more than 25 %. It also
  requires at least 10 samples per period, a converged period, at most 5 %
  amplitude spread and at most 10 % high/low asymmetry. It passes on
  `fopdt` and `thermal` and fails on `motor`, `msd`, `integrating` and
  `arm` at the default settings. A failed check shows a warning (CLI and
  app) that names the failed criteria, says why the gains are not
  trustworthy and suggests SIMC or Cohen-Coon where they apply.
- First-harmonic Ku estimate from the Fourier components of the relay
  experiment. The CLI has `--ku df|harmonic` and the app a "Ku" selector.
  When the describing-function and first-harmonic estimates differ by more
  than 5 %, a note states the expected error. On the default FOPDT plant
  the describing function is 19 % below the first harmonic.
- CLI text output: a `validity` line and both Ku estimates for relay
  methods. JSON output: `relay.validity`, `relay.kuDescribingFunction`,
  `relay.kuHarmonic`, `relay.kuEstimate` and a top-level `notes` array.
- Documentation of the check, the describing-function error on FOPDT plants
  (exact relay-oscillation formulas) and the effect of sampling, in
  `docs/models.md`.

### Changed

- The motor-only "under 10 controller samples" warning and the separate
  "had not settled" warning are now criteria of the general check.
- With the default `--ku df`, relay gains, Ku and Pu are unchanged from
  0.1.0. Regression tests pin them.

## [0.1.0] - 2026-11-19

First release.

### Added

- Six simulated plants with documented equations: first order plus dead time,
  mass-spring-damper, DC motor with Coulomb friction, integrating axis,
  three-node hotend thermal model, nonlinear arm joint.
- Fixed-step RK4 simulation with sub-steps, dead-time buffer, controller
  sample time separate from the plant step, seeded sensor noise,
  quantisation and sensor lag.
- Discrete PID in parallel and ideal form with filtered derivative,
  setpoint weights, output and rate limits, clamping and back-calculation
  anti-windup, bumpless manual/auto transfer and feed-forward.
- Step and disturbance metrics, and gain, phase and delay margins from the
  sampled-loop frequency response.
- Step-test FOPDT identification (two-point and least squares), relay
  autotune, and SIMC, Cohen-Coon, Ziegler-Nichols and Tyreus-Luyben rules
  with warnings when a test does not fit a rule's assumptions.
- Browser app with live strip charts, compare mode, Bode and Nyquist plots,
  shareable URLs, keyboard shortcuts, phone layout and light/dark themes.
- Headless CLI (`run`, `tune`, `plants`) with text, JSON and CSV output.
- Tests with `node:test`, JSDoc type checking, CI on Ubuntu and Windows with
  Node 22 and 24, a headless Chrome smoke test, and GitHub Pages deployment.

[0.1.1]: https://github.com/projectbin07-stack/pid-tuning-lab/releases/tag/v0.1.1
[0.1.0]: https://github.com/projectbin07-stack/pid-tuning-lab/releases/tag/v0.1.0
