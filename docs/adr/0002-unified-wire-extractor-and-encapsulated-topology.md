# 0002. Unified WireExtractor with Encapsulated Topology and Residual Processing

## Status
Accepted

## Context
在点云电力线三维重建中，阶段三导线提取（M3）原先在 `topdown_wire_extractor.py` 中编排了走廊定向切片（Corridor Slices）、种子体素追踪（Seed Tracker）和耐张跳线提取（Tension Jumpers）三个分支。这种结构存在以下严重架构缺陷：
1. **拓扑与并查集泄露**：模块内部使用并查集（Union-Find）维护连通分量，但在结果输出中却向下游泄露了长达 $N$ 的 `point_line_id` 数组、`suspect_line_ids` 集合以及 `find_line_func: Callable[[int], int]` 闭包。下游导出与染色模块（如 `export_colored_las`）被迫感知并查集的内部查找逻辑。
2. **手动索引偏移与跨模块缝隙脆弱**：在合并切片导线与追踪器导线时，通过 `for g_idx in missing_global_pts` 手动进行标量偏移 (`t_id + n_top_clusters`) 并包装闭包函数 `merged_find_line`，在跳线提取后又再次进行标量累加，极易引发 ID 冲突与越界。
3. **双通道全量无差别重算**：无论走廊切片是否已完美覆盖主档距，种子追踪器始终对全量高空点做无差别重复搜索与 3D 悬链线拟合，耗费了 30%~40% 的冗余计算时间。

## Decision
我们决定构建深度模块 `WireExtractor`，彻底收拢并封装导线提取生命周期：
1. **单一深度接口 (Deep Module Seam)**：
   在 `modules/wire_extractor.py` 中实现 `WireExtractor.extract(...) -> WireExtractionResult`，统一调度走廊切片、差量种子追踪与耐张跳线抽取。外部调用方（如 `PipelineExecutor`）只面对单一契约。
2. **拓扑就地压平与实体直出 (Flattened Union-Find & Entity Coloring)**：
   并查集作为内部临时图算法，在模块内部计算完成后立即就地压平（Flatten）。`WireExtractionResult` 产出最终确认的 `wires: List[WireCluster]`，每个 `WireCluster` 自包含全局点云索引 `global_indices: np.ndarray`、唯一标识 `line_id: int` 与拟合好的 `catenary` 模型。彻底消除对外暴露 `find_line_func` 与全局大数组。
3. **主辅差量协同模式 (Residual Complementary Execution)**：
   走廊切片（通道一）优先提取两塔跨间导线；提取完毕后从高空候选点集排除已锁定点。种子追踪（通道二）仅对差量残留点（如终端塔外侧引出线、孤立未成档导线）进行增量扫描；若残留点不足阈值（< 50点）则直接短路，彻底消除全量重复计算。
4. **渐进式兼容与废弃层**：
   在 `WireExtractionResult` 上保留 `point_line_id` 与解包迭代器支持以平滑兼容既有测试；将旧入口 `extract_wires_topdown` 标记为 `[DEPRECATED]` 并转调 `WireExtractor`。

## Consequences
- 彻底封死拓扑并查集与全局大数组向外泄露的渠道，`export_colored_las` 与 `PipelineExecutor` 大幅精简。
- 大幅降低大场景多档距点云的整体计算耗时（消除冗余的二次种子聚类与重拟合）。
- 既有单元测试与冒烟测试保持 100% 绿色兼容。
