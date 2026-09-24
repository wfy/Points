# 01: Tension tower crossarm along line thickness release and dead zone elimination

**What to build:**
在 `modules/tower_detector.py` 中重构横担横梁切片判定逻辑，彻底解决大型耐张塔多层横担因顺线耐张绝缘子串厚度被一票否决的问题：
1. 破除 `w2 <= max_arm_thick` (2.07m) 的硬卡门槛。对于具备大翼展特征（$w_1 \ge 10.0\text{m}$）且具备清晰横线比（$w_1 \ge 2.2 \times w_2$）的耐张横担切片，顺线厚度上限放宽至 $5.5\text{m}$，完整容纳水平耐张绝缘子串与跳线；
2. 消除 `is_beam` 与 `is_tension_jumper` 之间的 $[2.07\text{m}, 5.5\text{m}]$ 断层死区，避免耐张横担被两不管逻辑误杀；
3. 保证 `cloud7.las` 3# 塔的 3 层下部横担（相对高程 26m、32m、38m 处，翼展 15m~19m）被稳定检出，最低横担 `z_lowest_arm` 从错误的 43.4m 精准归位至真实下横担处（~26m）；
4. 蓝黄分割线从虚高的 88% 塔身处（3154.9m）回归至合理腰身位置（~3140m），3 层宽大横担 100% 恢复为纯正蓝色，消除横担漏识与削断；
5. 针对性编写轻量单测验证耐张塔厚横担检出，避免全量大图过量测试。

**Blocked by:** None (can start immediately)

**Status:** closed

- [x] 调整 `modules/tower_detector.py` 中横梁判定逻辑，支持大翼展耐张横担放宽顺线厚度至 5.5m，消除判定死区
- [x] 单元测试验证耐张塔 3 层大翼展厚横担（$w_2 \approx 4.0\text{m} \sim 5.0\text{m}$）被完整登记为候选横担
- [x] 最低横担从 43.4m 塔顶地线羊角正确归位至 26m 真实下横担处，蓝黄分割线位置合理且 3 层横担均为蓝色
