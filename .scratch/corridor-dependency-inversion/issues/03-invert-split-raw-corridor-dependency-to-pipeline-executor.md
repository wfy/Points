# 03: Invert split_raw_corridor Dependency to PipelineExecutor

**What to build:**
Refactor `split_raw_corridor` (both inside `CorridorCutter` and the top-level procedural function) to delegate execution to `PipelineExecutor(config).run_file(las_input_path, stop_after=PipelineStage.TOWER)`. Remove the manual duplicate invocations of `separate_ground` and `detect_towers`. Provide backward-compatible shims for `order_towers_along_line`, `cut_corridors_by_spans`, `export_split_spans`, and `split_raw_corridor`.

**Blocked by:** Ticket 01, Ticket 02

**Status:** ready-for-agent

- [x] Refactor `CorridorCutter.split_raw_corridor` to delegate to `PipelineExecutor` with `stop_after=PipelineStage.TOWER`.
- [x] Remove manual duplicate calls to `separate_ground` and `detect_towers` from `modules/corridor_cutter.py`.
- [x] Maintain deprecated shims `order_towers_along_line`, `cut_corridors_by_spans`, `export_split_spans`, and `split_raw_corridor` delegating to `CorridorCutter`.
- [x] Add unit tests in `tests/test_corridor_cutter.py` verifying `split_raw_corridor` delegation and shim forwarding.
