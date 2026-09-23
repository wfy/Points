# 01: Slope vegetation rejection test harness

**What to build:**
构建针对高压输电线路中塔身侧边低空密集坡面植被（10m~14m）的回归测试夹具。复现低空山坡密实树冠被横担切片检测误判为横担/耐张跳线，导致最下横担高程 `z_lowest_arm` 错误下陷至 30% 塔高（10m），进而导致大范围边坡植被被错误纳入杆塔（上蓝下黄）的缺陷。

**Blocked by:** None (can start immediately)

**Status:** closed

- [x] 构建包含 34m 真实铁塔与 10m~14m 高度密集边坡树冠（横向延展 13m+）的合成点云测试夹具
- [x] 验证现有检测逻辑在低空树冠上的误触发（`z_lowest_arm` 被错误拉低至 10m~12m）
- [x] 编写 `tests/test_slope_vegetation_stripping.py`，提供自动化断言
