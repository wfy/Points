# 02: Isolate GUI File Dialog with Headless Fallback

**What to build:**
Create a dedicated GUI module `modules/gui.py` to decouple graphical windowing from numerical utilities. Migrate `select_file_gui()` into `modules/gui.py` using dynamic lazy importing of `tkinter` and `filedialog`. Catch `(ImportError, Exception)`: if running in a headless environment, Docker container, or if display server is missing, provide a graceful fallback to terminal console input (`input(...)`) or return `None` without crashing.

**Blocked by:** None (can run in parallel with Ticket 01)

**Status:** ready-for-agent

- [x] Create `modules/gui.py` and implement `select_file_gui(title, filetypes) -> Optional[str]`.
- [x] Use runtime lazy import of `tkinter` inside `select_file_gui` so importing `modules.gui` never eagerly triggers GUI initialization.
- [x] Gracefully catch GUI failure / missing display and fall back to terminal input or return `None`.
- [x] Add unit tests in `tests/test_gui.py` asserting clean import without loading `tkinter` and verifying headless fallback behavior when mocked.
