# 02: Absolute horizontal elevation boundary cut

**What to build:**
在 `modules/tower_detector.py` 与 `modules/pipeline_executor.py` 中，彻底废除使用离地相对高程 `rel_z` 切分黄蓝分界面的逻辑：
1. 以塔基核心四脚中轴地面标高为基准，计算绝对物理切分高程：$Z_{\text{cut\_abs}} = Z_{\text{base\_ground}} + z_{\text{waist}}$；
2. 空间判别面严格统一为绝对水平面：$P_z \ge Z_{\text{cut\_abs}}$ 为上半部定向盒（蓝），$P_z < Z_{\text{cut\_abs}}$ 为下半部四棱台（黄）；
3. `pipeline_executor.py` 染色完全依据绝对物理标高，消除山坡地形投影导致的倾斜，黄蓝分割线在三维真彩色中绝对水平；
4. 对 `cloud0.las` 做最小单塔切片验证，确认分割线水平且高度合理。

**Blocked by:** 01: Transmission crossarm floor and span threshold

**Status:** closed

- [x] 彻底移除使用 `rel_z` 进行黄蓝分界面划分的倾斜逻辑
- [x] 引入绝对物理切分标高 $Z_{\text{cut\_abs}}$，实现重力方向严格水平截切
- [x] `pipeline_executor.py` 中黄蓝染色与几何分界逻辑统一依据绝对高程切分
- [x] 针对性最小测试验证斜坡地面上铁塔黄蓝切分线在三维空间中绝对水平
