# Contributing

Bug reports, corrections to the models or the tuning rules, and pull requests
are welcome.

## Setup

```bash
git clone https://github.com/projectbin07-stack/pid-tuning-lab.git
cd pid-tuning-lab
npm ci            # dev tools only: TypeScript (type checking), Prettier, Node types
npm run serve     # http://127.0.0.1:8000/
```

The site has no runtime dependencies and no build step: `index.html` loads
the ES modules in `src/` directly. Any static file server works.

## Checks

All of these must pass; CI runs them on Ubuntu and Windows with Node 22 and 24,
plus a CLI smoke test and a headless Chrome smoke test of the page.

```bash
npm run format:check
npm run typecheck
npm run coverage      # node:test with line, branch and function thresholds
```

`npm run check` runs all three.

## Guidelines

- Keep the shipped page free of runtime dependencies. Dev tools go in
  `devDependencies` and must not be imported from `src/`.
- Keep computation out of `src/ui/app.js` and `src/ui/draw.js`. Everything
  that computes a number lives in `src/core/` (or `src/ui/state.js`) and has
  tests.
- Test against values you can derive independently: an analytic response, a
  hand-derived margin, a known ultimate gain. A test that only checks that
  code runs is not enough.
- If you change a model, a default parameter or a tuning rule, update
  `docs/models.md` and the README numbers that depend on it, and put the
  before and after CLI output in the pull request.
- Plant parameters are illustrative. Do not describe a model as matching a
  specific product unless you have measured data and include it.
- Types are JSDoc comments checked by `tsc --checkJs` in strict mode.
- Small, focused commits with imperative messages.
