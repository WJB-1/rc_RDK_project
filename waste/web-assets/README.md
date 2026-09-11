# Web Assets Archive

This directory holds browser resources removed from the live `web` asset tree
on 2026-09-11. They remain recoverable source material and are not served by a
Flask static route.

## Unreferenced JavaScript

The files in `unreferenced-js/` had no references from live Python, HTML, or
test sources at migration time:

- `main.js`
- `map-view.js`
- `simulator.js`
- `sim_control.js`
- `telemetry.js`

## Preserved Version Bundles

The following older assets are deliberately **not** in this archive:

- `web/versions/legacy-dashboard-1.0/` preserves the original dashboard pages.
- `web/versions/legacy-simulator-1.0/` preserves the simulator template and
  Canvas render/interaction scripts as a starting point for a future
  Navigation 2.0 visualization debugger.
