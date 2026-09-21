# 01: WireCluster Enhancement and WireExtractionResult Contract

**What to build:**
Enhance the core domain model for powerline wires. Update `WireCluster` to explicitly own its point membership (`global_indices: np.ndarray`), unique sequential identifier (`line_id: int`), suspect flag (`is_suspect: bool`), and jumper flag (`is_jumper: bool`). Introduce `WireExtractionResult` in `modules/models.py` as a typed domain container that seals internal Union-Find graph structures and provides flattened wire entities, while maintaining full backward-compatibility with tuple unpacking (`__iter__`) and attribute access.

**Blocked by:** None (can start immediately)

**Status:** ready-for-agent

- [x] `WireCluster` in `modules/models.py` has `line_id`, `global_indices`, `is_suspect`, and `is_jumper` attributes with sensible defaults.
- [x] `WireExtractionResult` is defined with `wires: List[WireCluster]`, `cable_indices: np.ndarray`, `jumper_indices: np.ndarray`, and `insulator_indices: np.ndarray`.
- [x] `WireExtractionResult` provides backward-compatible properties (`point_line_id`, `suspect_line_ids`, `find_line_func`) and supports 5-tuple and 6-tuple unpacking.
- [x] Unit tests in `tests/test_wire_models.py` verify creation, index consistency, and tuple unpacking compatibility.
