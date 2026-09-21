# 02: Implement Deep PipelineExecutor with Stage-level Short-Circuiting

**What to build:** Implement the deep `PipelineExecutor` class in `modules/pipeline_executor.py` offering the high-level in-memory seam `run(points, config) -> PipelineResult` and file convenience `run_file(in_las, out_las) -> PipelineResult`. All 4 algorithm stages are orchestrated internally; stage execution halts cleanly when `stop_after` is reached.

**Blocked by:** 01: Define PipelineResult Entity and PipelineStage Enum

**Status:** ready-for-agent

- [x] `PipelineExecutor.run(points, config)` executes Stage 1 (Ground), Stage 2 (Tower), Stage 3 (Wire), and Stage 4 (Topology).
- [x] If `config.pipeline.stop_after == PipelineStage.TOWER`, Stage 3 and 4 are cleanly bypassed and the returned `PipelineResult` contains valid ground and tower classifications.
- [x] Direct routing to `extract_wires_topdown` for Stage 3 without passing through `powerline_extractor.py`.
- [x] Automated unit test verifies `PipelineExecutor.run()` on synthetic point clouds with full execution and short-circuited execution.
