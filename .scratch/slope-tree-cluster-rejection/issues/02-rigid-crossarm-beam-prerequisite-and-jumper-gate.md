# 02: Rigid crossarm beam prerequisite & tension jumper gating

**What to build:**
重构 `modules/tower_detector.py` 的横担判定与跳线准入逻辑：
1. 真实高压输电塔严禁“仅有跳线而无横担横梁”，候选簇必须至少包含有效刚性横担切片（`is_beam == True`），禁止纯圆形树冠伪装的孤立跳线成立；
2. 约束耐张跳线存在的物理前提：跳线必须依附于横担下方区间（即该候选簇已检测到有效横担梁），严禁孤立圆形树丛仅凭各向同性厚度直接注册为横担候选；
3. 收紧无横担兜底分支（`is_robust_lattice`），必须同时满足严格的塔腰截面几何上限；
4. 单元测试 `test_slope_tree_cluster_rejection.py` 验证树丛候选被彻底否决。

**Blocked by:** 01: Hillside tree cluster test harness & diagnostic verification

**Status:** closed

- [x] 在 `detect_towers` 切片聚类逻辑中，增加强硬前置条件：必须存在至少一个 `is_beam == True` 的高空刚性横担切片
- [x] 限制 `is_tension_jumper` 的生效条件：跳线切片仅作为已有横担主梁的辅助拓展，不得在没有梁的情况下独立生成杆塔实体
- [x] 收紧 `is_robust_lattice` 兜底条件，增加对无横担高塔的腰部几何与立柱特征校验
- [x] 单元测试通过，树丛误判为 0
