# 01: 塔腰纯净段正方形截面正交定姿 (Waist MinArea-OBB)

**What to build:** 在蓝黄分割线下方 $[Z_{\text{lowest\_arm}} - 3.5\text{m}, Z_{\text{lowest\_arm}} - 0.5\text{m}]$ 高密度钢构截面内，通过 2D 最小外接面积旋转扫描（MinArea-OBB）在 $[0, 90^\circ)$ 范围内高效求解使包络面积最小的旋转角，解析出与塔腰四面完全平行的正交基底对 $(\vec{u}_a, \vec{u}_b)$，彻底摆脱世界坐标系世界轴设定的束缚。

**Blocked by:** None (can start immediately)

**Status:** ready-for-agent

- [ ] 实现 `extract_waist_orthogonal_axes` 函数，在塔腰纯净段执行 2D 最小外接矩形扫描
- [ ] 确保在 $[0, 90^\circ)$ 范围内以 $0.5^\circ$ 步长精确锁定正方形截面的全局极小面积方向
- [ ] 输出相互垂直的单位方向向量对 $(\vec{u}_a, \vec{u}_b)$
- [ ] 单元测试验证：针对任意倾角正方形点云与 5-6# 真实塔腰点云均能稳定解析出物理外立面法向
