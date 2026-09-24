# 01: Transmission crossarm floor and span threshold

**What to build:**
在 `modules/tower_detector.py` 中修正横担半宽最小搜索阈值 `min_arm_span`，释放对 110kV/220kV 中小型角钢输电铁塔的几何约束：
1. `min_arm_span` 下限由 5.0m 调整为 3.5m（例如 `max(max_z * 0.08, 3.5)`）；
2. 保证 `cloud0.las` 真实铁塔的下横担（19.5m 处，w1=4.19m）与中横担（24.5m 处，w1=4.61m）被完整检出；
3. 最低横担高程 `z_lowest_arm` 从错误的 31.2m 精准归位至真实下横担处（~19.5m），杜绝下层横担及绝缘子被四棱台削断漏识的问题；
4. 采用轻量针对性单测验证（仅针对横担归位与防止漏识，不跑冗余全量大图）。

**Blocked by:** None (can start immediately)

**Status:** closed

- [x] 调整 `min_arm_span` 与切片长宽比判定，使其兼容 110kV 输电铁塔（半宽 3.5m~4.6m）
- [x] 单元测试验证 110kV 铁塔最低横担锁定在下横担区间（<= 21.0m 且 >= 18.0m），不再被虚抬至 31m
- [x] 下层横担点完整保留在杆塔点云内，零漏识
