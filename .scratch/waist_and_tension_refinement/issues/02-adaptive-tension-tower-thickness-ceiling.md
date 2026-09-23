# 02: 220kV+ 耐张塔与直线塔顺线厚度动态自适应分路 (Adaptive Tension Tower Thickness Ceiling)

**What to build:** 
在高空 UpperTowerBox 计算中，通过高空区顺线方向（$d_{v2}$）点云展宽与大弧垂跳线特征动态识别耐张塔类型；对耐张塔将顺线厚度上限 `max_ceiling` 放开至 `min(max(0.12 * h_true, 5.5), 6.0m)`，并将 UpperTowerBox 底部基准下延 2.5m（`z_upper_floor = z_lowest_arm - 2.5`），以完整容纳耐张绝缘子串与下垂外展的大弧垂跳线环，彻底解决 220kV+ 耐张塔跳线被四棱台切除的问题；对直线塔继续保持严格紧致的 4.2m 上限阻断出线跨中导线；实现耐张塔跳线完整保全与跨中导线精准阻断的双向兼得。

**Blocked by:** 01: 顺线先验防倒挂与塔腰正交基准轴向中轴再校准 (Anti-Inversion & Orthogonal Centroid Recalibration)

**Status:** closed

- [x] 实现耐张塔大弧垂跳线特征动态识别判定（基于高空顺线投影 P95 >= 3.8m 与跳线点密度）
- [x] 耐张塔顺线厚度上限自适应放开至 5.5m~6.0m，且 UpperTowerBox 底部下沉 2.5m 覆盖下垂跳线环
- [x] 直线塔保持 4.2m 紧致上限阻断出线导线（>= 7m 跨中导线捕获率 < 5%）
- [x] 编写单测验证耐张塔跳线捕获率 >= 85% 且跨中导线不染色
