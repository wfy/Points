# 01: Implement Viewer Protocol and Concrete Strategies

**What to build:**
Implement the unified 3D point cloud viewer interface and strategies in `modules/viewer.py`. Define `ViewerHandler` Protocol specifying `open(file_path: str) -> bool`, `close() -> bool`, and `is_available() -> bool`. Implement concrete classes: `QTModelerFinder` (consolidating search paths and `QTMODELER_PATH` env var), `QTModelerViewer` (with subprocess management and safe Windows process killing), `SystemDefaultViewer` (platform default opener via `os.startfile` or system command), and `NullViewer` (silent no-op implementation for headless/testing mode). Implement the `get_viewer()` factory function.

**Blocked by:** None (can start immediately)

**Status:** ready-for-agent

- [x] Define `ViewerHandler` Protocol in `modules/viewer.py` with `open`, `close`, and `is_available`.
- [x] Implement `QTModelerFinder` consolidating canonical install paths and `QTMODELER_PATH` environment variable.
- [x] Implement `QTModelerViewer` supporting subprocess launch, safe process termination, and fallback to system default.
- [x] Implement `SystemDefaultViewer` and `NullViewer` (safe no-op for tests and headless CI).
- [x] Implement factory `get_viewer(config: Optional[ExportConfig] = None, name: Optional[str] = None) -> ViewerHandler`.
- [x] Add unit tests in `tests/test_viewer.py` validating `NullViewer`, factory config dispatch (when `open_qtmodeler=False`), and environment variable overrides.
