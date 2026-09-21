# 04: Deprecate Legacy Shim and Verify Smoke Tests

**What to build:**
Mark legacy entry points as deprecated and run full automated verification across all test suites. Deprecate `extract_wires_topdown` in `modules/topdown_wire_extractor.py`, forwarding its invocations to `WireExtractor.extract()`. Run all unit tests and the end-to-end smoke test `tests/test_pipeline_m3_smoke.py` across synthetic spans and real LiDAR data (`17-18(17_18).las`), validating zero regressions, 100% test pass rate (Exit Code: 0), and execution time improvements.

**Blocked by:** 03: Integrate PipelineExecutor and Decouple Exporter

**Status:** ready-for-agent

- [x] `extract_wires_topdown` in `modules/topdown_wire_extractor.py` issues a `DeprecationWarning` and delegates to `WireExtractor.extract()`.
- [x] `modules/__init__.py` exports `WireExtractor` and `WireExtractionResult`.
- [x] Full unit test suite passes: `python -m unittest discover -s tests` with Exit Code: 0 (25 tests passed).
- [x] Pipeline smoke test passes: `python tests/test_pipeline_m3_smoke.py` passes on synthetic and real span data with Exit Code: 0.
- [x] Stage 3 progress logging continues to report accurate cable point counts and wire cluster group numbers.
