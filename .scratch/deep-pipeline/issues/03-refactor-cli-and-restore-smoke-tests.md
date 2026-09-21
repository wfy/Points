# 03: Refactor CLI Entry and Restore Smoke Test Gate

**What to build:** Refactor `fast_powerline_classifier.py` so that `fast_classify_and_color_powerline` delegates to `PipelineExecutor.run_file()`. Add the `--stop-after` CLI argument to permit partial stage runs from the command line. Remove temporary `#` comments from the pipeline and ensure `tests/test_pipeline_m3_smoke.py` passes 100%.

**Blocked by:** 02: Implement Deep PipelineExecutor with Stage-level Short-Circuiting

**Status:** ready-for-agent

- [x] `fast_classify_and_color_powerline` delegates execution to `PipelineExecutor`.
- [x] Command line flag `--stop-after {ground,tower,wire}` correctly halts processing at the specified stage.
- [x] Commented-out lines in `fast_powerline_classifier.py` are completely cleaned up.
- [x] `tests/test_pipeline_m3_smoke.py` and existing test suite pass with Exit Code: 0.
