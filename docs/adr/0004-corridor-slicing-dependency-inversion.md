# 0004. Corridor Slicing Dependency Inversion and Pipeline Integration

## Status
Accepted

## Context
在电力线点云分类系统中，走廊分档切片（Corridor Slicing）用于在主干输电线路各基铁塔之间切分独立的“两塔一档”空间。原先在 `modules/corridor_cutter.py` 中的实现存在以下结构缺陷：
1. **领域模型分散 (Model Fragmentation)**：
   `SpanSegment` 数据类定义在 `modules/corridor_cutter.py` 中，而系统中所有其他核心领域模型（`TowerEntity`、`WireCluster`、`PipelineResult` 等）均收拢在 `modules/models.py`。
2. **流水线调度重复实现 (Duplicated Pipeline Stages)**：
   独立切片预处理函数 `split_raw_corridor` 在内部手动编排了地面剥离 `separate_ground` 与铁塔检测 `detect_towers`。这与 `PipelineExecutor` 的阶段 1、阶段 2 逻辑与控制台进度日志完全重复。在引入 `stop_after=PipelineStage.TOWER` 机制后，该重复编排已成为纯粹的技术冗余。
3. **二次磁盘 I/O 浪费 (Redundant Disk I/O)**：
   在 CLI 运行中，当配置 `--split-spans` 后置切档时，代码在主流程完成后使用 `laspy.read(las_input_path)` 从磁盘全量重新读取点云以获取坐标数组，浪费了大规模激光雷达点云的内存缓存。
4. **过程式松散结构 (Loose Procedural Architecture)**：
   切档逻辑由 4 个松散顶层函数构成，缺少统一的领域服务类。拓扑排序算法在遇急转角时缺少走向平滑性防护。

## Decision
我们决定实施依赖倒置与领域模型收拢，将走廊切档深度内聚并无缝融入流水线：
1. **领域模型统揽与深度服务抽象 (`modules/models.py`, `modules/corridor_cutter.py`)**：
   - 将 `SpanSegment` 数据模型迁入 `modules/models.py`，保持实体库单一可信源。
   - 构建高内聚深度模块 `CorridorCutter`，提供统一接口：
     - `order_towers(towers) -> List[int]`：带航向夹角平滑惩罚的杆塔主干拓扑排序。
     - `cut_spans(points, towers, half_width, buffer) -> List[SpanSegment]`：基于 OBB 向量化定向走廊空间切分。
     - `export_spans(las_classified_path, spans, output_dir) -> List[str]`：切片无损高保真 LAS 落盘。
     - `split_raw_corridor(las_input_path, output_dir, config) -> List[str]`：纯走廊切片预处理门面。
   - 原顶层函数保留为向后兼容 Shim。
2. **依赖倒置与阶段控制复用 (`split_raw_corridor`)**：
   重构 `split_raw_corridor`，直接委托 `PipelineExecutor.run_file(..., stop_after=PipelineStage.TOWER)`。彻底消除手动重复调用阶段 1 与阶段 2，保证整线初筛与后续精细分类在算法和配置上 100% 行为一致。
3. **内存级流水线集成与二次 I/O 消除 (`PipelineResult.spans`)**：
   - 在 `PipelineResult` 中增加可选字段 `spans: Optional[List[SpanSegment]] = None`。
   - 在 `PipelineExecutor.run()` 中，若 `config.corridor.split_spans=True` 且锁定的有效铁塔数 $\ge 2$，直接复用内存中已加载的 `points` 数组计算 `spans`，避免重复读取磁盘 LAS 文件。
   - 在 `PipelineExecutor.run_file()` 落盘着色 LAS 后，自动调度 `export_spans` 完成物理分档导出。

## Consequences
- 彻底消除 `corridor_cutter.py` 对阶段 1、阶段 2 算法的手工重复编排，统一收敛至 `PipelineExecutor`。
- 消除百万级点云在后置切档时的磁盘二次重复读取，端到端执行效率显著提升。
- `SpanSegment` 正式纳入领域模型体系，`PipelineResult` 具备完整的走廊分档表达能力。
- 历史函数与 CLI 接口 100% 向后兼容。
