# 02: Implement Deep WireExtractor with Residual Processing

**What to build:**
Create the deep `WireExtractor` module in `modules/wire_extractor.py`. Provide a single in-memory entry point `WireExtractor.extract(...) -> WireExtractionResult` that encapsulates the execution of corridor slices, residual point extraction, and tension jumper extraction. Implement the complementary execution policy: Channel A (corridor slicing) runs first across spans; points claimed by Channel A are subtracted from the high-altitude pool; Channel B (seed tracker) processes only the remaining residual points, short-circuiting if fewer than 50 points remain. Flatten all disjoint sets internally and assign sequential non-colliding `line_id`s to all clusters.

**Blocked by:** 01: WireCluster Enhancement and WireExtractionResult Contract

**Status:** ready-for-agent

- [x] `WireExtractor` class created in `modules/wire_extractor.py` with `extract(points, off_ground_pts, off_ground_idx, rel_z, is_tower, tower_infos, config) -> WireExtractionResult`.
- [x] Channel A corridor slicing runs and assigns sequential `line_id`s (1..k) and sets `global_indices` on `WireCluster`s.
- [x] Residual filtering correctly subtracts Channel A indices from `high_indices`; if residual count < 50, Channel B is skipped.
- [x] Channel B resolves Union-Find sets internally and assigns sequential `line_id`s starting from $k + 1$.
- [x] Tension jumpers are extracted and appended with sequential `line_id`s.
- [x] Point exclusivity is enforced: no wire indices overlap with confirmed tower indices.
- [x] Unit tests in `tests/test_wire_extractor.py` assert on cluster counts, non-colliding IDs, and residual short-circuiting.
