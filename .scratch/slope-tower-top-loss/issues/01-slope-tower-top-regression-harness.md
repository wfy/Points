# 01: Slope tower top regression harness

**What to build:**
构建针对陡坡地形（基底落差近 20m）与外展猫头塔顶羊角角钢的端到端测试用例与基准校验夹具。模拟包含 20m 陡坡剪切的地面高程与猫头塔顶部羊角几何结构，在单测中复现坡底相对高程膨胀导致塔尖被剪切的现象，提供可重复运行的回归测试套件。

**Blocked by:** None (can start immediately)

**Status:** closed

- [x] 构建包含大坡度基底（高程落差 15m~20m）与外展地线羊角的合成点云测试夹具
- [x] 验证现有检测逻辑在陡坡猫头塔顶部的漏识现象（复现 `local_rel_z > max_z + 4.0` 截断）
- [x] 单元测试通过 `python -m unittest tests/test_slope_tower_top.py` 执行且在修复前准确断言捕获缺口
