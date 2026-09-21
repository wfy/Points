# Specification: Corridor Slicing Dependency Inversion and Pipeline Integration

## Problem Statement

In the powerline point cloud classification architecture, the corridor slicing subsystem (`modules/corridor_cutter.py`) splits large-scale transmission lines into independent span-level point clouds ("两塔一档"). The current design has four architectural flaws:

1. **Domain Model Fragmentation**:
   `SpanSegment` is declared in `modules/corridor_cutter.py`, whereas all other canonical entities (`TowerEntity`, `WireCluster`, `GroundResult`, `PipelineResult`) live in `modules/models.py`.
2. **Duplicated Pipeline Orchestration in Pre-Splitting (`split_raw_corridor`)**:
   `split_raw_corridor` manually runs `separate_ground` (Stage 1) and `detect_towers` (Stage 2) from scratch. This duplicates the execution logic, stage timings, and telemetry logging of `PipelineExecutor`. With the introduction of `PipelineStage.TOWER` short-circuiting in Candidate 01, this manual orchestration is redundant technical debt.
3. **Redundant Disk I/O in Post-Splitting (`--split-spans`)**:
   In `fast_powerline_classifier.py`, when `--split-spans` is enabled, the CLI calls `laspy.read(las_input_path)` after full pipeline classification finishes, reloading the entire multi-million point LAS file from disk just to extract 3D coordinates that `PipelineExecutor` already had in memory.
4. **Shallow Procedural Functions**:
   `corridor_cutter.py` consists of loose procedural functions (`order_towers_along_line`, `cut_corridors_by_spans`, `export_split_spans`, `split_raw_corridor`). The greedy nearest-neighbor ordering in `order_towers_along_line` lacks heading smoothness penalties, risking non-monotonic index ordering on transmission lines with large turning angles.

## Solution

Invert dependencies and encapsulate the corridor slicing subsystem into a unified domain service:

1. **Domain Model Consolidation (`modules/models.py`)**:
   Move `SpanSegment` to `modules/models.py` as a first-class citizen alongside `TowerEntity` and `PipelineResult`. Retain a re-export shim in `modules/corridor_cutter.py`.
2. **Deep Domain Module (`CorridorCutter` in `modules/corridor_cutter.py`)**:
   Encapsulate all geometric calculations, topological ordering, and span export into `CorridorCutter`:
   - `order_towers(tower_infos: List[Union[TowerEntity, dict]]) -> List[int]`: Topological sorting with heading smoothness penalties.
   - `cut_spans(points: np.ndarray, tower_infos: List[Union[TowerEntity, dict]], corridor_half_width: float = 30.0, buffer_length: float = 12.0) -> List[SpanSegment]`: Vectorized OBB bounding box point slicing.
   - `export_spans(las_classified_path: str, spans: List[SpanSegment], output_dir: Optional[str] = None) -> List[str]`: Zero-copy, lossless span LAS file serialization.
   - `split_raw_corridor(las_input_path: str, output_dir: Optional[str] = None, config: Optional[PipelineConfig] = None) -> List[str]`: High-level facade delegating to `PipelineExecutor(stop_after=PipelineStage.TOWER)`.
3. **Pipeline Lifecycle Integration (`modules/pipeline_executor.py`)**:
   - Add `spans: Optional[List[SpanSegment]] = None` to `PipelineResult`.
   - In `PipelineExecutor.run()`: if `config.corridor.split_spans` is True and `len(towers) >= 2`, immediately compute `spans = CorridorCutter.cut_spans(...)` using in-memory `points` and `tower_infos`.
   - In `PipelineExecutor.run_file()`: if `result.spans` is populated, automatically trigger `CorridorCutter.export_spans(...)` to persist split LAS files.
4. **Eliminate Redundant Disk I/O in CLI (`fast_powerline_classifier.py`)**:
   Remove manual re-reading of `las_input_path` in `fast_classify_and_color_powerline`, letting `PipelineExecutor` handle span cutting and serialization natively.
5. **Full Backward Compatibility**:
   Retain `order_towers_along_line`, `cut_corridors_by_spans`, `export_split_spans`, and `split_raw_corridor` as top-level function wrappers in `modules/corridor_cutter.py` delegating to `CorridorCutter`.

## User Stories

1. **As a pipeline developer**, I want `PipelineResult` to contain `spans: List[SpanSegment]` when span splitting is requested, so that span segments are available in memory without reading files from disk.
2. **As a performance engineer**, I want `--split-spans` to slice spans in memory during pipeline execution without re-reading the LAS file from disk, saving several seconds on large datasets.
3. **As an algorithm engineer**, I want `split_raw_corridor` to delegate to `PipelineExecutor(stop_after=TOWER)`, ensuring that tower detection algorithms and parameters are identical across whole-line and single-span workflows.
4. **As a field engineer**, I want tower topological ordering to penalize sudden angular heading changes, ensuring correct sequence order on turning towers (转角塔).
5. **As a QA engineer**, I want all existing 46 unit tests and smoke tests to pass without regressions (Exit Code: 0).

## Implementation Decisions

### 1. Model Migration (`modules/models.py`)
```python
@dataclass
class SpanSegment:
    """两塔一档走廊切片数据模型"""
    span_index: int                       # 档段顺序编号 (0-based)
    tower_from_idx: int                   # 起始杆塔全局索引
    tower_to_idx: int                     # 终止杆塔全局索引
    tower_from_pos: np.ndarray            # 起始杆塔 2D 坐标 [cx, cy]
    tower_to_pos: np.ndarray              # 终止杆塔 2D 坐标 [cx, cy]
    span_length: float                    # 档距水平跨度 (m)
    point_indices: np.ndarray             # 属于本档走廊的点云全局索引
    output_las_path: str = ""             # 输出 LAS 文件完整路径
```
Add `spans: Optional[List[SpanSegment]] = None` to `PipelineResult`.

### 2. `CorridorCutter` Deep Module (`modules/corridor_cutter.py`)
- Define `class CorridorCutter`:
  - `order_towers`: Computes pairwise distance matrix, identifies maximum distance terminal nodes, and runs greedy step with heading direction alignment penalty:
    $\text{cost}(curr, next) = \text{dist}(curr, next) \times (1.0 + \lambda \cdot (1.0 - \cos \theta))$
  - `cut_spans`: Projects points onto span axis $u$ and normal axis $n$, building vectorized boolean masks.
  - `export_spans`: Reads classified LAS once and slices with `span_las = las[span.point_indices]`.
  - `split_raw_corridor`: Uses `PipelineExecutor.run_file(las_input_path, stop_after=PipelineStage.TOWER)`.
- Re-export `SpanSegment` and provide deprecated function shims for backward compatibility.

### 3. Pipeline Lifecycle Integration (`modules/pipeline_executor.py`)
- In `PipelineExecutor.run()`:
  ```python
  spans = None
  if getattr(cfg.corridor, 'split_spans', False) and len(tower_infos) >= 2:
      spans = CorridorCutter.cut_spans(
          points=points,
          tower_infos=tower_infos,
          corridor_half_width=cfg.corridor.corridor_half_width,
          buffer_length=cfg.corridor.buffer_length
      )
  ```
- In `PipelineExecutor.run_file()`:
  ```python
  if result.spans is not None and len(result.spans) > 0:
      CorridorCutter.export_spans(actual_path, result.spans)
  ```

### 4. CLI Refactoring (`fast_powerline_classifier.py`)
- Remove lines 51-64 (`t_split = time.time() ... laspy.read(las_input_path)`), as `PipelineExecutor` now executes span cutting directly in memory.

## Testing Decisions

### 1. New Unit Tests (`tests/test_corridor_cutter.py`)
- `test_order_towers_linear()`: Validates monotonic ordering for straight-line towers.
- `test_order_towers_corner_turn()`: Validates robust ordering with turning angle.
- `test_cut_spans_bounding_box()`: Validates OBB point capture on synthetic tower span.
- `test_split_raw_corridor_delegation()`: Verifies delegation to `PipelineExecutor` with `stop_after=TOWER`.
- `test_pipeline_executor_in_memory_spans()`: Verifies `PipelineResult.spans` is populated when `cfg.corridor.split_spans=True`.
- `test_backward_compatible_shims()`: Verifies `order_towers_along_line`, `cut_corridors_by_spans`, `export_split_spans` function shims.

### 2. Full Regression Verification
- Run all 46+ unit tests (`.\venv_312\Scripts\python.exe -m unittest discover -s tests -p "test_*.py"`).
- Run `tests/test_pipeline_m3_smoke.py` verifying synthetic and real LiDAR data pipelines pass (Exit Code: 0).

## Out of Scope

- Multi-branch 3D tree graph slicing (e.g. T-junctions in distribution grids).
