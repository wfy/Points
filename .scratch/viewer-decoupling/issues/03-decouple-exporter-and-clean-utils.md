# 03: Decouple Exporter from Process Management and Clean Utils

**What to build:**
Cleanse `modules/utils.py` into a pure numerical and LAS I/O utility module:
1. Remove module-level static imports of `tkinter`, `filedialog`, and `subprocess`.
2. In `export_colored_las`, deprecate `force_kill_viewer: bool = False` (retain signature for backward compatibility, but remove the `taskkill` subprocess call so file serialization has zero process-killing side effects).
3. Provide backward-compatible shims for `close_qtmodeler`, `open_in_qtmodeler`, and `select_file_gui` forwarding to `modules.viewer` and `modules.gui`.
4. Update `fast_powerline_classifier.py` to import `select_file_gui` from `modules.gui` and `get_viewer` from `modules.viewer`. Move process termination to the pipeline start lifecycle so files are safely unlocked before processing starts.

**Blocked by:** Ticket 01, Ticket 02

**Status:** ready-for-agent

- [x] Remove `tkinter`, `filedialog`, and `subprocess` imports from `modules/utils.py` root.
- [x] Refactor `export_colored_las` to remove process-killing logic while keeping `force_kill_viewer` as a deprecated parameter.
- [x] Provide backward-compatible shims in `modules/utils.py` forwarding to `modules.viewer` and `modules.gui`.
- [x] Refactor `fast_powerline_classifier.py` to use `modules.gui.select_file_gui` and `modules.viewer.get_viewer(cfg.export)`.
