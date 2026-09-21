# 01: Define PipelineResult Entity and PipelineStage Enum

**What to build:** Establish the `PipelineStage` execution enum, extend `PipelineConfig` with stage controls (`stop_after`), and define the `PipelineResult` domain model to hold points classifications, extracted entities, and timing metrics without leaking intermediate arrays.

**Blocked by:** None (can start immediately)

**Status:** ready-for-agent

- [x] `PipelineStage` enum defines `GROUND`, `TOWER`, `WIRE`, `TOPOLOGY`, `EXPORT`.
- [x] `PipelineConfig` accepts `stop_after: Optional[PipelineStage]` and `stages: Optional[Set[PipelineStage]]`.
- [x] `PipelineResult` dataclass encapsulates `classification`, `ground_indices`, `tower_indices`, `wire_indices`, `towers`, `wires`, and `stage_timings`.
- [x] Unit tests verify `PipelineResult` construction, indexing helpers, and dictionary serialization.
