# 04: Consolidate Protocol Handler and Verify Regression

**What to build:**
Consolidate URL protocol handling in `modules/qtmodeler_protocol_handler.py` to reuse `modules.viewer` path discovery and opening logic, eliminating hardcoded path duplication. Update `modules/__init__.py` to export the new viewer and GUI symbols. Execute the entire unit test suite and the real-world LiDAR smoke test to ensure 100% green regression verification with Exit Code: 0.

**Blocked by:** Ticket 01, Ticket 02, Ticket 03

**Status:** ready-for-agent

- [x] Refactor `modules/qtmodeler_protocol_handler.py` to import and reuse `QTModelerViewer` and `find_qtmodeler` from `modules.viewer`.
- [x] Update `modules/__init__.py` to export `ViewerHandler`, `QTModelerViewer`, `SystemDefaultViewer`, `NullViewer`, `get_viewer`, `find_qtmodeler`, and `select_file_gui`.
- [x] Run full unit test suite (`.\venv_312\Scripts\python.exe -m unittest discover -s tests`) to guarantee all existing 25 tests + new tests pass (Exit Code: 0).
- [x] Run `tests/test_pipeline_m3_smoke.py` verifying both synthetic and real LiDAR (`17-18(17_18).las`) end-to-end pipelines pass (Exit Code: 0).
