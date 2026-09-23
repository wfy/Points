# 03: Cavity ratio and frustum vegetation stripping

**What to build:**
在 `modules/tower_detector.py` 的四棱台下半部与欧式连通生长中，防止向两侧边坡独立灌木和密实树木泄漏：
1. 严格约束四棱台放坡走廊（`w1_allowed` 与 `w2_allowed` 结合塔腰基准宽度进行物理放坡限制）；
2. 过滤掉远离塔腿中心线（四角立柱）的非空腔密实团块；
3. 真实 `cloud0.las` 实测验证：将两旁边坡上万点误识别树木彻底剔除出 Class 15，回归植被类别，铁塔自身骨架完整保留。
4. 全量测试套件通过（Exit Code: 0）。

**Blocked by:** 02: Transmission crossarm elevation floor and aspect gate

**Status:** closed

- [x] 限制下半部四棱台放坡走廊无序膨胀，杜绝跨越山坡向外部树木侵入
- [x] 验证 `cloud0.las` 误识别点剥离效果，误识别率大幅下降
- [x] 全量测试套件 `python -m unittest discover tests` 保持 Exit Code: 0
