# 02: Slope adaptive top absolute Z gate

**What to build:**
在 `modules/tower_detector.py` 的初始候选提取与 `valid_mask` 计算处，消除坡底相对高程膨胀畸变对塔顶造成的硬截断。引入基于绝对高程安全保护或山地落差补偿（`cluster_abs_max_z` / `delta_h_relief`），使得建在陡坡长短腿上的铁塔顶端金属角钢顺利通过 `valid_mask` 并全量参与铁塔候选点集分析。

**Blocked by:** 01: Slope tower top regression harness

**Status:** closed

- [x] 在 `detect_towers` 的局部候选点切片提取处，放宽或基于绝对高程对塔顶进行保护（避免 `local_rel_z <= max_z + 4.0` 在陡坡下坡面处误杀塔顶）
- [x] 确保平原铁塔原有行为完全不受影响，仅在陡坡山地补偿高差
- [x] `tests/test_slope_tower_top.py` 通过第一阶段验证，塔顶候选点捕获率提升至 > 95%
