# 04: PipelineExecutor Memory Integration and Full Regression Verification

**What to build:**
Integrate corridor span slicing directly into the lifecycle of `PipelineExecutor`. In `PipelineExecutor.run()`, if `config.corridor.split_spans` is True and `len(tower_infos) >= 2`, compute `CorridorCutter.cut_spans` directly using existing in-memory `points` and `tower_infos`, attaching the resulting `SpanSegment` list to `result.spans`. In `PipelineExecutor.run_file()`, if `result.spans` is present and non-empty, invoke `CorridorCutter.export_spans` to persist split LAS files. Remove the redundant disk re-reading of `las_input_path` in `fast_powerline_classifier.py`. Export `CorridorCutter` and `SpanSegment` in `modules/__init__.py`. Execute all unit tests and the smoke test to guarantee 100% green regression verification with Exit Code: 0.

**Blocked by:** Ticket 01, Ticket 02, Ticket 03

**Status:** ready-for-agent

- [x] In `PipelineExecutor.run()`, execute in-memory span slicing when `config.corridor.split_spans=True` and attach to `result.spans`.
- [x] In `PipelineExecutor.run_file()`, export split spans when `result.spans` is populated.
- [x] In `fast_powerline_classifier.py`, remove redundant `laspy.read(las_input_path)` in `fast_classify_and_color_powerline`.
- [x] Export `CorridorCutter` and `SpanSegment` in `modules/__init__.py`.
- [x] Run full unit test suite (`.\venv_312\Scripts\python.exe -m unittest discover -s tests`) to guarantee all existing 46 tests + new tests pass (Exit Code: 0).
- [x] Run `tests/test_pipeline_m3_smoke.py` verifying synthetic and real LiDAR data pass (Exit Code: 0).
