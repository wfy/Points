# Specification: Protocol-Driven Viewer and GUI Decoupling

## Problem Statement

In the current codebase, external visualization viewers (QTModeler) and graphical user interface (GUI) file selection are tangled directly into core utility and I/O routines:
1. **Process Management Coupled with Data Export (I/O & Process Coupling)**:
   In `modules/utils.py`, `export_colored_las` (the core LAS classification and RGB serializer) takes `force_kill_viewer: bool = False`. When True, it directly invokes `close_qtmodeler()`, which executes a platform-dependent Windows command `taskkill /F /IM QTModeler.exe /T` via `subprocess.run`. A pure file serializer should never manage operating system process trees.
2. **Lack of Headless & Test Isolation (No Headless/Null Viewer)**:
   `open_in_qtmodeler` in `modules/utils.py` hardcodes 8 Windows installation paths and launches subprocesses or falls back to `os.startfile`. In test suites (`test_pipeline_executor.py`, `test_pipeline_m3_smoke.py`) and automated batch scripts, callers must manually set `cfg.export.open_qtmodeler = False` to prevent unexpected external process launches. There is no concept of a `NullViewer` (Null Object Pattern).
3. **GUI Dependency in Utilities (GUI Pollution in Core Modules)**:
   `modules/utils.py` statically imports `tkinter` and `filedialog` at the module root. In headless servers, Linux CI environments, or Docker containers without a display server, importing `modules/utils.py` (or `modules/__init__.py`) can cause immediate crashes (`ImportError`, `TclError`) or hangs.
4. **Duplicated Search Path Resolution**:
   `modules/qtmodeler_protocol_handler.py` duplicate-scans the exact same 8 Windows paths as `utils.py`, violating the DRY principle.

## Solution

Decouple viewers and GUI file selection from core algorithm/IO modules using the Strategy/Protocol pattern and Lazy Dynamic Loading:
1. **Viewer Protocol & Concrete Implementations (`modules/viewer.py`)**:
   - `ViewerHandler(Protocol)`: Defines `open(file_path: str) -> bool`, `close() -> bool`, and `is_available() -> bool`.
   - `QTModelerViewer`: Encapsulates QTModeler executable discovery via `QTModelerFinder`, subprocess launching, and safe termination.
   - `SystemDefaultViewer`: Opens files using the operating system's default viewer (`os.startfile` on Windows, `xdg-open` on Linux, `open` on macOS).
   - `NullViewer`: Safe, silent no-op implementation designed for automated testing, CI/CD, and headless batch execution.
   - Factory function: `get_viewer(config: Optional[ExportConfig] = None, name: Optional[str] = None) -> ViewerHandler`. When `open_qtmodeler` is False, automatically returns `NullViewer`.
2. **Pure I/O Exporter (`modules/utils.py`)**:
   - Deprecate `force_kill_viewer: bool = False` in `export_colored_las` (retained for backward compatibility, but does not kill processes).
   - Pre-export process termination is elevated to `PipelineExecutor` or CLI execution lifecycle via `ViewerHandler.close()`.
   - Remove root imports of `tkinter`, `filedialog`, and `subprocess` from `modules/utils.py`.
3. **Dedicated GUI File Selector with Headless Fallback (`modules/gui.py`)**:
   - Extract `select_file_gui()` into `modules/gui.py`.
   - Dynamically load `tkinter` only when called.
   - Catch `(ImportError, Exception)`: If GUI is unavailable or fails, gracefully prompt the user via CLI input (`input(...)`) or return `None`.
4. **URL Protocol Handler Consolidation (`modules/qtmodeler_protocol_handler.py`)**:
   - Refactor `qtmodeler_protocol_handler.py` to reuse `find_qtmodeler` and `QTModelerViewer` from `modules/viewer.py`, eliminating path duplication.
5. **Backwards Compatibility**:
   - `modules/utils.py` re-exports deprecated shim functions `open_in_qtmodeler`, `close_qtmodeler`, and `select_file_gui` forwarding to the new modules with `DeprecationWarning`.
   - `modules/__init__.py` exposes `get_viewer`, `ViewerHandler`, `NullViewer`, `QTModelerViewer`, alongside legacy shims.

## User Stories

1. **As a CI/Test Engineer**, I want automated tests and batch pipelines to run with `NullViewer` by default when `open_qtmodeler=False`, so that tests never trigger UI popups or process kill commands.
2. **As a Server/Docker Developer**, I want to import `modules.utils` and `modules` without requiring `tkinter` or an active X11/Wayland display server, so that headless microservices run reliably without GUI dependency crashes.
3. **As a Data Exporter Developer**, I want `export_colored_las` to be a pure file serializer without process killing side effects, making file writing deterministic and thread-safe.
4. **As a CLI User**, I want `fast_powerline_classifier.py` to seamlessly find and launch QTModeler if installed, fall back to the system default viewer if missing, or prompt in terminal if running over SSH without a GUI.
5. **As a QA Engineer**, I want all existing 25 unit tests and smoke tests to continue passing with Exit Code: 0.

## Implementation Decisions

### 1. `modules/viewer.py` [NEW]
- Define `ViewerHandler(Protocol)`:
  ```python
  class ViewerHandler(Protocol):
      def open(self, file_path: str) -> bool: ...
      def close(self) -> bool: ...
      def is_available(self) -> bool: ...
  ```
- Define `QTModelerFinder`:
  - Consolidates canonical paths:
    ```python
    POSSIBLE_PATHS = [
        r"C:\Program Files\QTModeler_820_UX_TRIAL\QTModeler.exe",
        r"C:\Program Files\Applied Imagery\QT Modeler\QTModeler.exe",
        r"C:\QTModeler_840_UX\QTModeler.exe",
        r"D:\Program Files\QTModeler_820_UX_TRIAL\QTModeler.exe",
        r"D:\Program Files\Applied Imagery\QT Modeler\QTModeler.exe",
        r"D:\QTModeler_840_UX\QTModeler.exe",
        r"E:\Program Files\Applied Imagery\QT Modeler\QTModeler.exe",
        r"E:\QTModeler_840_UX\QTModeler.exe",
    ]
    ```
  - Also inspects environment variable `QTMODELER_PATH`.
- Define `QTModelerViewer`:
  - `open(file_path: str) -> bool`: launches QTModeler subprocess with `file_path`. If not installed, falls back to `SystemDefaultViewer().open(file_path)`.
  - `close() -> bool`: executes `taskkill /F /IM QTModeler.exe /T` (only on Windows).
  - `is_available() -> bool`: returns `bool(find_qtmodeler())`.
- Define `SystemDefaultViewer`:
  - `open(file_path: str) -> bool`: `os.startfile` on Windows, `xdg-open` / `open` on POSIX.
  - `close() -> bool`: No-op (returns True).
  - `is_available() -> bool`: True on desktop OS.
- Define `NullViewer`:
  - `open(file_path: str) -> bool`: returns True (no-op).
  - `close() -> bool`: returns True (no-op).
  - `is_available() -> bool`: returns False.
- Factory `get_viewer(config: Optional[ExportConfig] = None, name: Optional[str] = None) -> ViewerHandler`:
  - If `config is not None and not config.open_qtmodeler`: return `NullViewer()`.
  - If `name == "null"`: return `NullViewer()`.
  - If `name == "qtmodeler"`: return `QTModelerViewer()`.
  - If `name == "system"`: return `SystemDefaultViewer()`.
  - Default: if QTModeler is available, return `QTModelerViewer()`, else `SystemDefaultViewer()`.

### 2. `modules/gui.py` [NEW]
- `select_file_gui(title: str = "选择要分类处理的点云文件 (LAS/LAZ)", filetypes: Optional[list] = None) -> Optional[str]`:
  - Dynamic `import tkinter as tk; from tkinter import filedialog`.
  - Encapsulate in `try...except (ImportError, Exception)`:
    - If display / tkinter is missing: print warning and fall back to `input("请输入点云文件路径: ").strip()`.
  - Returns sanitized file path or None if cancelled.

### 3. `modules/utils.py` [MODIFY]
- Remove `import tkinter`, `from tkinter import filedialog`, `import subprocess`.
- In `export_colored_las`:
  - Retain `force_kill_viewer: bool = False` signature.
  - Remove `close_qtmodeler()` invocation. (Optionally log debug warning if True).
- Retain deprecated shims:
  - `close_qtmodeler()`: delegates to `get_viewer(name="qtmodeler").close()`.
  - `open_in_qtmodeler(las_path)`: delegates to `get_viewer(name="qtmodeler").open(las_path)`.
  - `select_file_gui()`: delegates to `modules.gui.select_file_gui()`.

### 4. `modules/qtmodeler_protocol_handler.py` [MODIFY]
- Import `find_qtmodeler` and `QTModelerViewer` from `modules.viewer`.
- Delegate path resolution and opening to `QTModelerViewer().open(las_path)`.

### 5. `fast_powerline_classifier.py` [MODIFY]
- Update imports: import `select_file_gui` from `modules.gui` (or `modules`), `get_viewer` from `modules.viewer`.
- In CLI pipeline:
  - At start: if `cfg.export.open_qtmodeler`, call `get_viewer(cfg.export).close()` to clear locks before starting pipeline.
  - At end: call `get_viewer(cfg.export).open(out_path)`.

### 6. `modules/__init__.py` [MODIFY]
- Export `ViewerHandler`, `QTModelerViewer`, `SystemDefaultViewer`, `NullViewer`, `get_viewer`, `find_qtmodeler`.
- Export `select_file_gui` from `modules.gui`.

## Testing Decisions

### 1. New Unit Tests (`tests/test_viewer.py`)
- `test_null_viewer()`: Asserts `NullViewer.open()` and `close()` return True without side effects.
- `test_factory_with_config()`:
  - When `cfg.export.open_qtmodeler = False`, `get_viewer(cfg.export)` returns instance of `NullViewer`.
  - When `name="null"`, returns `NullViewer`.
- `test_system_default_viewer_contract()`: Asserts `is_available()` returns boolean and `close()` returns True.
- `test_finder_environment_variable()`: Asserts `QTModelerFinder` respects custom `QTMODELER_PATH` environment variable.

### 2. New GUI Unit Tests (`tests/test_gui.py`)
- `test_select_file_gui_headless_fallback()`: Mocks `tkinter` import failure, asserts fallback prompt or graceful None return without crashing.
- `test_utils_clean_imports()`: Asserts importing `modules.utils` does NOT populate `sys.modules` with `tkinter`.

### 3. Full Regression Verification
- Run `.\venv_312\Scripts\python.exe -m unittest discover -s tests -p "test_*.py"` (asserting all 25+ tests pass with Exit Code: 0).
- Run `.\venv_312\Scripts\python.exe tests/test_pipeline_m3_smoke.py` (Exit Code: 0).

## Out of Scope

- Refactoring Corridor Cutter geometry dependencies (Candidate 04).
- Integrating alternative 3D viewer backends like Potree or CloudCompare (deferred to future feature releases).
