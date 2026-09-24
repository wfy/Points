# 0007. Zero-Copy 2D Grid Cell Spatial Indexing, Linear Histogram Aggregation, and Double-Precision Preservation

## Status
Accepted

## Context
在海量激光雷达点云（整档/全线数千万至上亿点，如 3000万~1亿点大场景）执行全自动杆塔识别（`PipelineStage.TOWER`）时，阶段二暴露出严重的计算耗时与内存峰值瓶颈：
1. **CPU 耗时瓶颈（耗时占比 > 85%，可超 150~200 秒）**：
   在 `cluster_tower_voxels` 中，对初筛出的数千个有效 2D 网格 $g_i$，代码通过 Python 循环在全量点云数组上执行 `m = (sub_inv == g_i)`。其时间复杂度退化为 $\mathcal{O}(K \times N)$（相当于数十亿次元素比对与数组广播），严重拖慢全流程效率；
2. **内存峰值爆炸（峰值突破 10GB~16GB）**：
   - 为截取 2~7 座候选塔周围 $r \le 30\text{m}$ 的局部点云，代码构建了全量非地面点的全局 2D KD 树 `off_ground_tree = cKDTree(off_ground_pts[:, :2])`。SciPy 的 C++ 内部树节点结构对上亿点平均消耗 64~96 字节/点，仅该单棵全局树就独占 **5GB ~ 10GB** 内存；
   - `cluster_tower_voxels` 中的 `np.unique(..., return_inverse=True)` 与 `np.isin(...)` 多次在全量数组上申请多倍排序与掩膜缓冲区，叠加数千次临时布尔数组分配，导致 Windows 堆内存严重碎片化，物理内存无法释放；
   - 加上原始点云对象堆叠，导致阶段二内存峰值直接飙升突破 10GB 以上，在低配置设备上极易触发 OOM 崩溃。

同时，用户提出了硬性约束：**优化必须在 100% 不影响现阶段识别精度与几何效果的前提下完成，绝不允许任何坐标偏移与漏识误识。**

## Decision
我们正式确立**“2D 网格空间分桶索引 (GridSpatialIndex) + 线性单趟聚合 (SortedSliceAggregation) + 双精度绝对坐标坚守 (Double-Precision Preservation)”**的阶段二性能与内存重构技术架构规范：

1. **二维网格空间分桶索引 (`GridSpatialIndex`) 替代全局 `cKDTree`**：
   - 彻底废除对全量非地面点构建 `cKDTree` 的做法，一次性剔除 5GB~10GB 的最大内存占用；
   - 建立轻量级 2D 网格哈希/分桶索引（复用已有的 2m 投影网格）；
   - 在根据候选塔中心 $(c_x, c_y)$ 截取 $r \le 30\text{m}$ 局部点云时，先通过网格包围盒 $[c_x - r, c_x + r] \times [c_y - r, c_y + r]$ 快速检出对应桶内的候选点，再施加完全相同的精确欧氏距离判定 $(x - c_x)^2 + (y - c_y)^2 \le r^2$；
   - **数学等价性保障**：提取出的局部点云索引集合与原 `query_ball_point` 完全一致，点集合完全无差异。

2. **单趟预排序切片聚合 (`SortedSliceAggregation`)**：
   - 废除 `cluster_tower_voxels` 中遍历网格并在全量大数组上 `== g_i` 的 $\mathcal{O}(K \times N)$ 广播机制；
   - 废除 `np.isin` 的全量大数组二次排序消耗；
   - 对有效点按网格编号执行单趟排序（$\mathcal{O}(N \log N)$ 仅需 ~0.1s），通过指针分段切片（Slice）直接提取每个网格内部的高度、唯一层数与重心加权分；
   - **计算等价性保障**：每个网格计算得出的最大高度、网格点数与权重重心数值与原逻辑浮点完全对齐，初筛候选塔列表 100% 相同。

3. **坚决坚守全局三维坐标 `float64` 双精度 (`Double-Precision Preservation`)**：
   - 坚决否决将全局点云三维坐标转为 `float32` 的危险做法；
   - 在中国电力机巡与 GIS 高斯/UTM 绝对投影坐标系（米级坐标数值高达数百万至千万）下，`float32` 的 24 位有效尾数会直接引入 $0.2\text{m} \sim 0.5\text{m}$ 的致命坐标抖动，破坏横担 RANSAC 与毫米级角钢拟合；
   - 全局点云坐标及派生实体属性严格保持 `float64`，从数据源头上确保几何精度零漂移。

4. **硬性双轨对齐门禁 (`Parity Verification Guard`)**：
   - 任何涉及阶段二性能优化的代码合并，必须经过 Parity 测试：
     1. 铁塔实体位置 $(c_x, c_y)$、最低横担标高 $z_{\text{lowest\_arm}}$ 与分割高程 $z_{\text{boundary}}$ 浮点偏差严格为 0；
     2. `is_tower` 与 `is_tower_arm` 分类掩膜点索引集合必须与优化前 100% 相同，差异点数为 0；
     3. 阶段二内存峰值实测必须稳定压降在 3.0GB 以内（万万点场景），耗时由 >150s 压降至 5s 以内。

## Consequences
- 彻底解决阶段二在超大点云下的内存爆炸问题，将内存峰值从 >10GB~16GB 锐减至 2GB~3GB 左右（降幅 > 75%）。
- 消除数十亿次低效比对，阶段二处理时间由原本 150~200 秒压降至 1~5 秒以内，全流程端到端速度提升数倍。
- 绝不引入任何第三方复杂编译依赖（如 Numba/Cython），保持轻量级纯 Python + NumPy/SciPy 生态。
- 保证铁塔检测几何模型（`UpperTowerBox` / `LowerTowerFrustum` / 刚性双侧对称）的输出行为 100% 零漂移、零损失。
