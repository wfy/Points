# 01: Hillside tree cluster test harness & diagnostic verification

**What to build:**
构建针对高压输电线路跨档山坡独立与成片高大树林（$H \approx 28\text{m}$，树冠展宽 $15\text{m} \sim 20\text{m}$，无金属刚性横担横梁）的自动化回归测试夹具。复现在 `cloud2.las` 中由于圆形树冠触发 `is_tension_jumper` 导致档中 6.8万点树林被误判为杆塔的缺陷。

**Blocked by:** None (can start immediately)

**Status:** closed

- [x] 构建包含 28m~35m 密集半球形/圆柱形树冠与真实杆塔跨档布局的合成点云测试夹具
- [x] 验证现有检测逻辑在独立树林簇上的误识别现象（产生伪杆塔实体）
- [x] 编写 `tests/test_slope_tree_cluster_rejection.py`，提供自动化断言（树丛误识别杆塔实体数为 0）
