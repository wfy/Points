# Specification: Unified WireExtractor with Encapsulated Topology and Residual Processing

## Problem Statement

In the point cloud powerline extraction pipeline, Stage 3 (M3 Wire Extraction) is fragmented across four separate modules:
1. `modules/topdown_wire_extractor.py` (corridor directional slicing and catenary fitting)
2. `modules/wire_seed_extractor.py` (PCA linearity seed extraction)
3. `modules/wire_tracker.py` (voxel clustering and 3D catenary tracking with disjoint-set Union-Find)
4. `modules/tension_topology.py` (tension tower jumper wire extraction)

This fragmentation creates three critical architectural defects:
1. **Topology & Union-Find Leakage**: The internal graph algorithm (Union-Find) leaks out of the extraction module into `PipelineExecutor` and `export_colored_las` via `find_line_func: Callable[[int], int]`, `suspect_line_ids: Set[int]`, and a full point-cloud-sized integer array `point_line_id` of length $N$. Downstream coloring logic is forced to query an internal function closure for every point.
2. **Fragile Array Index Offsetting**: Merging top-down corridor slices with bottom-up tracker clusters relies on manual scalar loops (`t_id + n_top_clusters`), closure re-wrapping (`merged_find_line`), and secondary additions for tension jumpers (`len(merged_confirmed) + 1`), risking ID collisions and out-of-bound errors.
3. **Eager Dual-Channel Redundancy**: Regardless of whether corridor slices successfully reconstruct all span wires with high confidence, the bottom-up seed tracker and voxel clustering always execute eagerly across all high-altitude points, performing duplicate 3D catenary curve fittings and wasting 30% to 40% of runtime on multi-span corridors.

## Solution

Consolidate M3 wire extraction into a single deep module `WireExtractor` located at `modules/wire_extractor.py`.
The module provides a single, high-leverage in-memory interface:
```python
WireExtractor.extract(
    points: np.ndarray,
    off_ground_pts: np.ndarray,
    off_ground_idx: np.ndarray,
    rel_z: np.ndarray,
    is_tower: np.ndarray,
    tower_infos: List[TowerEntity],
    config: Optional[PipelineConfig] = None
) -> WireExtractionResult
```

Key architectural mechanics:
1. **Encapsulated Topology & Flattened Entities**: The Union-Find disjoint sets are resolved and flattened internally. The result delivers finalized `WireCluster` domain entities, each self-containing its `global_indices: np.ndarray`, unique `line_id: int`, and fitted `catenary: Optional[CatenaryModel]`.
2. **Elimination of Leaking Closures**: `find_line_func` and `suspect_line_ids` are eliminated from the public interface. Downstream consumers (e.g. `export_colored_las`) color points by iterating over `result.wires`, eliminating the need for an $N$-length dense index array.
3. **Complementary Residual Dual-Channel**: Channel A (corridor slicing) processes spans first. Points captured by Channel A are subtracted from high-altitude candidates. Channel B (seed tracker) operates strictly on residual points (e.g., wires outside terminal towers or spans without towers). If residual points are below threshold (< 50 points), Channel B short-circuits.
4. **Seamless Jumper Absorption**: Tension jumper extraction is unified within the same sequence, assigning sequential line IDs cleanly.
5. **Full Backward Compatibility**: `WireExtractionResult` provides tuple unpacking and a cached `point_line_id` property for legacy code, and `extract_wires_topdown` is retained as a deprecated forwarding shim.

## User Stories

1. As a pipeline developer, I want to invoke wire extraction through a single call to `WireExtractor.extract()`, so that channel orchestration, point index offsetting, and jumper extraction are completely encapsulated.
2. As an exporter developer, I want `WireCluster` entities to directly hold `line_id` and `global_indices`, so that `export_colored_las` can assign RGB colors by iterating over wires without invoking an internal `find_line_func` closure.
3. As a performance engineer, I want Channel B to run only on residual high-altitude points not captured by Channel A, so that multi-span datasets eliminate redundant 3D catenary fittings and save 30%–40% execution time.
4. As an algorithm engineer, I want `WireExtractionResult` to cleanly separate conductor wires, jumpers, and reserved insulator slots, so that downstream topology validation and clearance analysis work with typed domain objects.
5. As a QA engineer, I want all existing test suites (`tests/test_pipeline_executor.py`, `tests/test_pipeline_m3_smoke.py`) to pass without modification (Exit Code: 0), ensuring zero regressions.

## Implementation Decisions

1. **`WireCluster` Entity Enhancement (`modules/models.py`)**:
   Add typed attributes to `WireCluster`:
   - `line_id: int = 0`: Sequential 1-based unique identifier for the wire.
   - `global_indices: np.ndarray = field(default_factory=lambda: np.array([], dtype=int))`: 1D array of global point indices in the original point cloud.
   - `is_suspect: bool = False`: Flag indicating if the wire is flagged as canopy/vegetation suspect.
   - `is_jumper: bool = False`: Flag indicating if the cluster is a tension jumper.

2. **Domain Result Structure (`WireExtractionResult` in `modules/models.py`)**:
   Dataclass encapsulating:
   - `wires: List[WireCluster]`: List of all confirmed wire clusters with flattened indices and line IDs.
   - `cable_indices: np.ndarray`: Global point indices of all wire points (union of all `wire.global_indices`).
   - `jumper_indices: np.ndarray`: Global point indices of jumper points.
   - `insulator_indices: np.ndarray`: Reserved slot for future insulator classification (Class 16).
   - Backward-compatible properties:
     - `point_line_id`: Dynamically assembled or cached array of size $N$, mapping each point to `line_id`.
     - `find_line_func`: Default identity `lambda x: x`.
     - `suspect_line_ids`: Set of `line_id`s where `is_suspect == True`.
     - `__iter__`: Supports 5-tuple and 6-tuple unpacking identical to `ExtractionResult`.

3. **Deep `WireExtractor` Module (`modules/wire_extractor.py`)**:
   - Implements `WireExtractor` with `extract(...) -> WireExtractionResult`.
   - **Step 1 (Channel A: Corridor Slices)**: Invokes `extract_wires_by_corridor_slices`. Assigns sequential `line_id` (1..k) and sets `global_indices` on each cluster.
   - **Step 2 (Residual Filtering)**: Subtracts Channel A points from `high_indices`. If `len(residual_indices) < 50`, skips Channel B; otherwise executes seed extraction and tracker on residual points.
   - **Step 3 (Channel B Flattening)**: Resolves tracker disjoint sets into `WireCluster`s, assigning sequential IDs starting from $k + 1$.
   - **Step 4 (Tension Jumpers)**: Extracts jumpers via `extract_tension_jumpers`, wrapping newly found jumpers into `WireCluster(is_jumper=True)` with sequential IDs.
   - **Step 5 (Tower Mutual Exclusivity)**: Ensures no wire point overlaps with confirmed tower points.

4. **Exporter Refactoring (`modules/utils.py`)**:
   In `export_colored_las`, check if `all_confirmed` clusters have non-empty `global_indices`. If present, color directly via `red[wire.global_indices] = cr`, bypassing `point_line_id` and `find_line_func`. Fall back gracefully if legacy parameters are provided.

5. **Pipeline Integration (`modules/pipeline_executor.py`)**:
   Replace direct call to `extract_wires_topdown` in `PipelineExecutor.run()` with `WireExtractor(config=cfg).extract(...)`.

6. **Deprecation**:
   `extract_wires_topdown` in `modules/topdown_wire_extractor.py` is marked with `@deprecated` warning and delegates directly to `WireExtractor().extract(...)`.

## Testing Decisions

- **New Unit Tests (`tests/test_wire_extractor.py`)**:
  - Test `WireExtractor.extract()` on synthetic span points, asserting:
    - All returned `WireCluster`s have non-empty `global_indices` and valid `line_id > 0`.
    - No duplicate `line_id`s across clusters.
    - Zero overlap between wire point indices and tower point indices.
    - Tuple unpacking compatibility with `ExtractionResult`.
  - Test residual short-circuit: Verify that when Channel A captures all points, Channel B does not execute.
- **End-to-End Regression**:
  - Run `.\venv_312\Scripts\python.exe -m unittest discover -s tests` (must pass 19/19 existing tests + new unit tests).
  - Run `.\venv_312\Scripts\python.exe tests/test_pipeline_m3_smoke.py` on both synthetic span and real LiDAR file `17-18(17_18).las` (Exit Code: 0).

## Out of Scope

- Rewriting QTModeler protocol handler or GUI dialogs (Candidate 03).
- Modifying corridor cutting geometry algorithms (Candidate 04).
- Changing ASPRS classification standard code mappings.
