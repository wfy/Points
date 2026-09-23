# 02: Transmission crossarm elevation floor and aspect gate

**What to build:**
在 `modules/tower_detector.py` 的横担切片扫描与跳线检测中，实施输电线路物理几何先验约束：
1. 输电网模式（`voltage_mode='high_voltage'`）下，横担最低搜索高度不得低于 `max_z * 0.55`（34m 输电塔导线绝不可能悬挂在 10m 高度触地）；
2. 约束横担切片的长宽比 `w_span / w_thick >= 2.0`，杜绝水平近椭圆状的稠密树冠伪装成横担。
确保 `cloud0` #71 塔最下横担高程正确锁定在真实高空横担处（Z >= 26m）。

**Blocked by:** 01: Slope vegetation rejection test harness

**Status:** closed

- [x] 在 `detect_towers` 切片扫描循环中，引入 `min_search_z` 输电网高程保护门限
- [x] 增加长宽比与跳线厚度几何先验约束，阻断宽树冠的误触发
- [x] 单元测试 `test_slope_vegetation_stripping.py` 验证 `z_lowest_arm >= 25.0m`
