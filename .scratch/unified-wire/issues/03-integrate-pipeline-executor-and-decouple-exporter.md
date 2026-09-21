# 03: Integrate PipelineExecutor and Decouple Exporter

**What to build:**
Connect the new `WireExtractor` to the pipeline lifecycle and decouple the LAS coloring subsystem from internal graph representations. Update `PipelineExecutor.run()` to invoke `WireExtractor.extract()` during Stage 3 and store the resulting `wires` list in `PipelineResult`. Refactor `export_colored_las` in `modules/utils.py` so that line coloring directly indexes `wire.global_indices` for each wire in `wires`, without reading an external length-$N$ `point_line_id` array or invoking `find_line_func`. Ensure legacy parameter forms continue to work without breaking.

**Blocked by:** 02: Implement Deep WireExtractor with Residual Processing

**Status:** ready-for-agent

- [x] `PipelineExecutor.run()` calls `WireExtractor(config=cfg).extract(...)` and propagates `wires` to `PipelineResult`.
- [x] `export_colored_las` in `modules/utils.py` directly colors points via `wire.global_indices` when present in `all_confirmed`.
- [x] No `find_line_func` closure or separate $N$-length `point_line_id` array is required to color wires properly.
- [x] Existing LAS export tests pass with exact color verification.
