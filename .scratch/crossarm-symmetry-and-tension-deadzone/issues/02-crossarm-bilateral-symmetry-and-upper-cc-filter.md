# 02: Crossarm bilateral symmetry and upper box connected component filter

**What to build:**
在 `modules/tower_detector.py` 中重构上半部定向包围盒的横担半宽计算与点云提取逻辑，消除单侧边坡树木拉偏横担长度与孤立悬空树斑误染蓝色的缺陷：
1. **横担双侧物理对称性仲裁**：分别度量横担左翼 $W_{\text{left}}$（$p_1 < 0$）与右翼 $W_{\text{right}}$（$p_1 > 0$）。依据铁塔刚性双侧对称设计先验，横担半宽由双侧有效支撑面约束（例如双侧较小者加合理安装裕量，或限制双侧比例），防止单侧边坡树木单方面将 `half_arm_w` 膨胀拉大，消除 `cloud0_sign.las` 左右横担长度悬殊的问题；
2. **上半部定向盒 3D 体素连通性过滤**：对落在 `mask_high` 包围盒内的点云，引入自塔心立柱种子向外扩展的 3D 体素连通性校验，彻底剔除在空间中与铁塔主材断开数米之外的悬空独立树冠点（即图 1 青色框内的孤立蓝斑）；
3. **保证真实角钢与跳线 100% 完整**：在滤除悬空树斑的同时，确保真实角钢、绝缘子串和跳线与铁塔连通部分零损伤；
4. 编写针对性轻量单测验证单侧树木拉偏抑制与悬空孤立树斑剔除。

**Blocked by:** 01: Tension tower crossarm along line thickness release and dead zone elimination

**Status:** closed

- [x] 实现横担左右双侧独立展宽测量与刚性对称性仲裁，杜绝单侧山坡树冠拉大横担包围盒
- [x] 在 `mask_high` 中增加 3D 体素连通性分析，过滤与铁塔主体非连通的孤立悬空边坡树斑
- [x] 针对性测试验证 `cloud0.las` 类似场景下左右横担长度基本对称，且右侧边坡悬空蓝斑被彻底剔除
