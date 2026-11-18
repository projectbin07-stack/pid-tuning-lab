# Security policy

## Supported versions

Only the latest release and the version deployed on GitHub Pages receive fixes.

## Reporting a vulnerability

Please do not open a public issue for a security problem. Use the private
vulnerability reporting of this repository on GitHub (Security tab, "Report a
vulnerability"):
https://github.com/projectbin07-stack/pid-tuning-lab/security/advisories/new

Include what you found, how to reproduce it and the version or commit. This is
a spare-time project maintained by one person, so there is no guaranteed
response time.

## Scope

The web page is static. It makes no network requests after loading its own
files, sends no data anywhere, uses no cookies and stores nothing. URL
parameters are parsed as numbers or matched against fixed lists and are never
evaluated as code or inserted as HTML. The "Copy link" button writes the
current URL to the clipboard and nothing else.

The CLI writes a file only when given `--csv <file>`, to that path. It opens
no network connections. `scripts/serve.js` is a development server bound to
127.0.0.1; do not expose it to a network.

## Not a safety component

This is a teaching and experimentation tool. The plants are simulated models
with illustrative parameters, nothing has been validated on hardware, and
gains from it must be verified on the real system with appropriate safety
limits (current limits, thermal runaway protection, end stops) before use.
