# 02: FrustumFirstAnchor 四棱台自底向上定姿与全塔单一刚体朝向锁定 (REVERTED)

**What to build:** 在地面上方纯净四棱台区间自底向上提取 4 根刚性主腿角钢进行定姿。
**Post-Evaluation finding:** 经实测，该方案的世界坐标系象限划分与强行正交立面绑定破坏了大部分正常铁塔的朝向，导致问题放大。
**Action:** 根据用户明确指示，已于 2026-09-22 彻底撤销此项修改，恢复原生鲁棒的 RANSAC/PCA 朝向机制，保留 Q1 和 Q3 的有效修复。

**Blocked by:** 01: PhysicalProportionGuard 物理几何比例熔断与横担阶跃检测

**Status:** reverted

- [x] 撤销 `extract_frustum_first_anchor` 函数
- [x] 恢复原生基于高空横担角钢 RANSAC 拟合的主轴朝向
- [x] 恢复纯净塔身多层切片几何轴心自校准 (Pure Trunk Multi-Slice Centroid Recalibration)
- [x] 验证 Q1 (紧致 OBB) 与 Q3 (30% 比例熔断) 保持完好且 61 项测试通过
