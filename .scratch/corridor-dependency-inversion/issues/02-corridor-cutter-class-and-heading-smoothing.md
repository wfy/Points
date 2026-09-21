# 02: Implement CorridorCutter Deep Module with Heading Smoothing

**What to build:**
Encapsulate corridor spatial geometry into a unified deep service `CorridorCutter` in `modules/corridor_cutter.py`. Implement `order_towers` with a heading angular smoothness penalty to avoid jump errors across sharp corner turns. Implement `cut_spans` for vectorized OBB bounding box point assignment. Implement `export_spans` for zero-copy, lossless span LAS file serialization.

**Blocked by:** Ticket 01

**Status:** ready-for-agent

- [x] Implement `CorridorCutter` class in `modules/corridor_cutter.py`.
- [x] Implement `CorridorCutter.order_towers` incorporating directional angle alignment penalty ($1.0 + \lambda(1.0 - \cos\theta)$) with defensive handling for $<2$ towers.
- [x] Implement `CorridorCutter.cut_spans` performing vectorized OBB slicing and returning `List[SpanSegment]`.
- [x] Implement `CorridorCutter.export_spans` writing individual span LAS files with point subset attributes preserved.
- [x] Add unit tests in `tests/test_corridor_cutter.py` testing linear ordering, corner-turn ordering, and OBB bounding box point capture.
