# Navigation 2.0 Web UI

This is the active Navigation 2.0 dashboard bundle. `web.app.create_app()`
serves `templates/dashboard.html` at `/` and exposes `static/` through Flask's
normal `/static` route.

- `templates/` defines the modular dashboard page.
- `static/css/` contains its dashboard styling.
- `static/js/` contains snapshot rendering, timeline, controls, map, and
  transport modules.
- `static/js/navigation2-legacy-adapter.js` is retained as a compatibility
  adapter for the older Canvas dashboard data model.
