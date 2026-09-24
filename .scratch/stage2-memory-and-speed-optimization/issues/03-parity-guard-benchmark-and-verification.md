# 03: Parity guard benchmark and end-to-end performance verification

**What to build:**
建立硬性双轨对齐验证门禁（`Parity Verification Guard`），确保阶段二内存与速度优化前后结果完全一致、零精度损失：
1. 编写对齐验证测试套件 `tests/test_stage2_optimization_parity.py`；
2. 覆盖多类型典型场景：直线塔、耐张塔、边坡隆起树木、悬空绝缘子跳线等；
3. 严格对比优化前后的输出指标：
   - 提取铁塔数量一致；
   - 塔心坐标 $(c_x, c_y)$ 与最高点高程 `abs_max_z` 浮点误差为 0；
   - 最低横担标高 `z_lowest_arm` 与蓝黄分割线高程 `abs_z_boundary` 完全一致；
   - `is_tower`、`is_tower_arm` 最终分类掩膜点索引集合 100% 相同（差异点数 = 0）；
4. 验证阶段二处理耗时由 >150s 降至 <5s，内存峰值稳定在 3.0GB 以内。

**Blocked by:** 02: Zero-copy 2D grid spatial index replacing global cKDTree

**Status:** closed

- [x] 编写并运行 `tests/test_stage2_optimization_parity.py` 双轨对齐测试
- [x] 验证铁塔实体输出几何属性与优化前零偏差
- [x] 验证分类掩膜 `is_tower` 逐点一致率达到 100.00%
- [x] 验证阶段二耗时降幅 > 95%，内存峰值压降 > 70%
