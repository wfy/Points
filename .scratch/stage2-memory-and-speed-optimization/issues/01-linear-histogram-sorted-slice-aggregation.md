# 01: Linear histogram aggregation via sorted slice in cluster_tower_voxels

**What to build:**
重构 `modules/tower_detector.py` 中的 `cluster_tower_voxels`，将候选网格高度连续性与高位骨架重心统计从 $\mathcal{O}(K \times N)$ 降至单趟 $\mathcal{O}(N)$ 复杂度：
1. 废除在千万级数组上反复遍历并广播比对 `m = (sub_inv == g_i)` 的循环机制；
2. 废除 `np.isin(inv_2d, valid_grid_idx)` 的大数组二次排序消耗；
3. 采用预排序单趟切片算法（`SortedSliceAggregation`）：对落入有效网格的点云进行排序分段，利用指针切片批量提取每个有效网格的最大高度、唯一高度层数以及顶部 6m 骨架点加权分；
4. 保证输出的初筛候选塔列表（`cx, cy, max_z, score`）与优化前 100% 相同；
5. 编写针对性轻量单元测试，验证排序切片聚合与原逻辑输出严格等价，且数千万点数据耗时降至 1 秒以内。

**Blocked by:** None (can start immediately)

**Status:** ready-for-agent

- [ ] 在 `modules/tower_detector.py` 中实现 `SortedSliceAggregation`，重构 `cluster_tower_voxels`
- [ ] 消除千万级布尔掩膜分配与循环比对，将初筛耗时从 >150s 压降至 <1s
- [ ] 验证初筛候选塔中心坐标与评分与原实现 100% 浮点对齐
