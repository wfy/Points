# Specification: Deep PipelineExecutor with Stage-level Execution Control

## Problem Statement

Currently, the PowerLine-FastClassifier pipeline coordinates 4 separate algorithm stages directly inside the CLI entry script (`fast_powerline_classifier.py`). This procedural script passes more than 10 loose NumPy coordinate and boolean mask arrays across stages. When engineers need to isolate and debug a specific stage (such as M2 tower detection during benchmark iterations), they resort to manually commenting out subsequent stages (M3 wire extraction and M4 topology validation) using `#` in the production code. This breaks automated test contracts (e.g., `test_pipeline_m3_smoke.py`), risks accidental commits of broken pipeline code, and prevents fast in-memory unit testing due to tight coupling with disk LAS files.

## Solution

Encapsulate the entire 4-stage LiDAR classification process inside a deep module named `PipelineExecutor`. The executor presents a single high-level in-memory seam: `run(points) -> PipelineResult`, completely hiding intermediate arrays, voxel trees, and local coordinate shifts. It supports stage-level execution control via a `PipelineStage` enum and `stop_after` configuration, enabling partial pipeline runs (e.g. up to tower detection) programmatically without modifying production code. The executor also provides a `run_file(input_las, output_las)` convenience method for seamless CLI and batch execution.

## User Stories

1. As an algorithm engineer, I want to invoke the point cloud classification pipeline via a single in-memory call `executor.run(points)`, so that I can write lightning-fast automated unit tests without reading or writing files to disk.
2. As a benchmark tester, I want to configure the pipeline with `stop_after=PipelineStage.TOWER`, so that I can test tower detection metrics without executing expensive wire catenary tracking.
3. As a CLI user, I want a command-line flag `--stop-after {ground,tower,wire}`, so that I can inspect intermediate classification outputs without modifying source code.
4. As a downstream consumer, I want the pipeline to return a structured `PipelineResult` object, so that I can access point classification codes, tower entities, wire clusters, and stage timings through standard typed attributes.
5. As an engineer running batch evaluations, I want `PipelineResult` to provide an `export_las(output_path)` method, so that I can persist results to disk only when needed.
6. As a developer maintaining regression tests, I want the pipeline smoke test `test_pipeline_m3_smoke.py` to pass deterministically on both synthetic and real LiDAR data, so that continuous integration acts as a reliable quality gate.
7. As a system architect, I want shallow pass-through modules like `powerline_extractor.py` bypassed and deprecated, so that the call graph remains direct and free of unused dependencies.

## Implementation Decisions

1. **High-level In-Memory Seam**: The primary interface of `PipelineExecutor` is `run(points: np.ndarray, config: Optional[PipelineConfig]) -> PipelineResult`. An auxiliary method `run_file(input_path: str, output_path: str) -> PipelineResult` handles file I/O and delegates directly to `run()`.
2. **Stage Lifecycle Enum**: A `PipelineStage` enum defines the discrete pipeline phases: `GROUND`, `TOWER`, `WIRE`, `TOPOLOGY`, and `EXPORT`.
3. **Execution Control & Interception**: `PipelineConfig` includes a `stop_after: Optional[PipelineStage]` field. If `stop_after == PipelineStage.TOWER`, the executor stops after Stage 2, marks points classified so far, and returns a valid `PipelineResult` without executing Stages 3 and 4.
4. **Domain Result Structure (`PipelineResult`)**: A dataclass capturing:
   - `classification`: (N,) uint8 array containing ASPRS classification codes (Class 2 Ground, 14 Wire, 15 Tower, 1 Unclassified).
   - `ground_indices`, `tower_indices`, `wire_indices`: 1D int arrays of global point indices.
   - `towers`: List of `TowerEntity` objects.
   - `wires`: List of `WireCluster` objects.
   - `stage_timings`: Dict mapping stage names to elapsed execution seconds.
   - `export_las(output_path)`: Helper method invoking the LAS export subsystem.
5. **Facade Deprecation**: Calls from `fast_powerline_classifier.py` and `PipelineExecutor` route directly to `modules.topdown_wire_extractor.extract_wires_topdown`, bypassing `modules/powerline_extractor.py`.
6. **Backward Compatibility**: `fast_classify_and_color_powerline` is retained as a thin wrapper around `PipelineExecutor.run_file()`, preserving 100% compatibility for external callers (`run_tower_tests.py`, `run_batch_eval.py`).

## Testing Decisions

- **Good Test Criteria**: Tests must verify external behavior through the highest seam (`run(points)` or `run_file()`). Tests must assert on classification array validity, entity counts, point exclusivity (no overlapping tower/wire points), and stage timing presence, without asserting on private intermediate variables.
- **Modules Tested**: `PipelineExecutor`, `fast_powerline_classifier`, and `PipelineResult`.
- **Prior Art & Existing Tests**:
  - `tests/test_pipeline_m3_smoke.py` provides the canonical synthetic and real span end-to-end verification.
  - `tests/test_tower_center_localization.py` and `tests/test_lower_tower_capture.py` provide tower-level validation.

## Out of Scope

- Merging internal dual channels in `topdown_wire_extractor` (deferred to Candidate 02).
- Rewriting QTModeler protocol handler or GUI dialogs (deferred to Candidate 03).
- Modifying corridor cutting geometry algorithms (deferred to Candidate 04).

## Further Notes

- Restoring un-commented execution of M3 in `fast_powerline_classifier.py` immediately restores `tests/test_pipeline_m3_smoke.py` to passing state.
