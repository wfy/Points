# 02: Zero-copy 2D grid spatial index replacing global cKDTree

**What to build:**
彻底消除全图非地面点全局 KD-Tree（`cKDTree(off_ground_pts[:, :2])`）引发的 5GB~10GB 内存暴涨，重构局部候选塔点云提取机制：
1. 废除对全量数千万至上亿非地面点构建 `cKDTree` 的实现；
2. 构建基于 2D 网格空间分桶索引（`GridSpatialIndex`），复用初筛网格，将点按二维网格桶索引；
3. 对每座候选塔 $(c_x, c_y)$ 与搜索半径 $r$，先圈定覆盖圆域的矩形网格桶集 $[c_x - r, c_x + r] \times [c_y - r, c_y + r]$，仅提取候选桶内的点做欧氏距离精确过滤 $(x - c_x)^2 + (y - c_y)^2 \le r^2$；
4. 保证提取出的局部点云索引集合与原 `query_ball_point` 完全一致（逐点 100% 对齐）；
5. 编写针对性单元测试，对比网格空间索引与 `cKDTree.query_ball_point` 在千万点下的点集完全等价性，并验证内存峰值立减 5GB 以上。

**Blocked by:** 01: Linear histogram aggregation via sorted slice in cluster_tower_voxels

**Status:** closed

- [x] 在 `modules/tower_detector.py` 中实现 `GridSpatialIndex`，废除全局 `cKDTree`
- [x] 确保每座候选塔提取出的局部点云索引与原 `query_ball_point` 100% 相同
- [x] 验证千万点场景下内存占用立减 5GB~10GB，且局部提取耗时控制在毫秒级
