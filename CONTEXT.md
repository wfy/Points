# PowerLine-FastClassifier

基于无人机激光雷达点云的输电线路走廊导线与杆塔全自动智能分类系统。

## Language

**TowerEntity**:
锁定的单座输电线路铁塔几何骨架实体，包含塔心三维坐标、横担主轴方向、塔厚方向及横担标高。
_Avoid_: Pole, Mast, Pylon, Structure

**WireCluster**:
沿走廊走向提取出的单相/单根连续导线或地线点云聚合簇，可包含三维物理悬链线拟合参数。
_Avoid_: CableSegment, LineString, WireObject

**Span**:
输电线路中两座相邻铁塔之间的跨越档距区间。
_Avoid_: Section, Gap, Interval

**Corridor**:
以两塔连线为中心轴线、向两侧水平外扩指定半宽的定向包围空间。
_Avoid_: Buffer, BoundingBox, ROI

**PipelineExecutor**:
点云分类流水线调度与状态封装的深度核心模块，统筹地面剥离、铁塔定位、导线提取与依附拓扑校验的全流程生命周期。
_Avoid_: PipelineManager, ClassifierWorkflow, PipelineService

**PipelineStage**:
流水线执行生命周期的离散阶段标识（包含 GROUND, TOWER, WIRE, TOPOLOGY, EXPORT），用于精细化控制执行与断点调试。
_Avoid_: Phase, Step, PipelineStatus

**PipelineResult**:
点云分类流水线执行后输出的整体领域成果，封装各类别点云掩膜、提取出的铁塔与导线实体以及耗时指标。
_Avoid_: OutputData, ExecutionContext, RawResult

**WireExtractor**:
负责根据杆塔先验与空间几何姿态，统筹走廊定向切片、种子体素追踪及跳线拟合，输出标准化导线实体的深度计算模块。
_Avoid_: CableDetector, LineFinder, WireTracer

**WireExtractionResult**:
导线提取阶段的领域成果容器，封装最终导线点索引集合、独立的线路簇列表 (`WireCluster`) 及分相着色映射，绝不泄露并查集或中间拓扑指针。
_Avoid_: RawWireOutput, ExtractedLines

**TensionJumper**:
耐张塔横担下方跨接不同档段导线的特异性引流跳线弧段实体。
_Avoid_: LoopCable, BridgeWire, BypassLine
