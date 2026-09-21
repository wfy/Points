# 04: Deprecate Shallow powerline_extractor Facade

**What to build:** Clean up `modules/powerline_extractor.py`, mark it as deprecated with a thin forwarding shim, remove its unused imports, and ensure all internal modules communicate directly with the core wire extractor.

**Blocked by:** 03: Refactor CLI Entry and Restore Smoke Test Gate

**Status:** ready-for-agent

- [x] Unused imports (`cluster_wire_candidates`, `filter_canopy_by_probes`, etc.) removed from `modules/powerline_extractor.py`.
- [x] Module marked with deprecation notice pointing callers to `modules.topdown_wire_extractor`.
- [x] No remaining active references inside `fast_powerline_classifier.py` or `PipelineExecutor`.
- [x] Full regression test suite passes cleanly.
