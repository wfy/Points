# 03: Mid-section waist narrowing & cavity-to-canopy profile filter

**What to build:**
针对铁塔与树冠的 3D 形态学本质差异，引入塔身垂直收腰与形态过滤机制：
1. 铁塔具有严格的下大、中收腰、上展宽的“沙漏/四棱台”外轮廓，塔腰截面半径严格受限于 $W_{\text{waist}} \le \text{cap}(H) \approx 3.2\text{m} \sim 3.8\text{m}$；若中部截面呈巨大树冠外凸（半径 $> 6.0\text{m}$）或各向实心分布，则作为非杆塔植被坚决予以剔除；
2. 在真实点云 `cloud2.las` 与 `cloud7.las` 上端到端验证，彻底消除档中 6.8万点与 29.6万点的误识别树林，仅保留真正高压杆塔；
3. 全量测试套件保持 Exit Code: 0。

**Blocked by:** 02: Rigid crossarm beam prerequisite & tension jumper gating

**Status:** closed

- [x] 在 `detect_towers` 中引入塔腰收缩比或立柱轮廓检测，剔除中部膨胀的球形/圆柱形树林簇
- [x] 跨档山坡独立与顺线高大树林通过横担梁准入门禁与塔腰轮廓过滤实现 100% 阻断排除
- [x] 真实高压杆塔横担翼展与立柱形态完整保留
- [x] 全量测试套件 `python -m unittest discover tests` 保持 Exit Code: 0
