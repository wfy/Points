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

**ViewerHandler**:
负责管理三维点云成果预览生命周期（探测、启动与关闭外部三维可视化软件）的统一协议抽象。
_Avoid_: ViewerManager, DisplayController, ModelerService

**QTModelerViewer**:
适配 Applied Imagery QTModeler 专业点云可视化分析软件的具体查看器实现。
_Avoid_: QTApp, ModelerRunner

**NullViewer**:
在自动化测试、批量离线运行或无头服务器环境下执行静默无操作的安全空对象查看器。
_Avoid_: MockViewer, DummyViewer, EmptyViewer

**SpanSegment**:
输电线路两座相邻铁塔之间的单档走廊定向包围空间及切分出的点云子集数据实体。
_Avoid_: SpanSlice, SubCloud, CorridorBlock

**CorridorCutter**:
负责铁塔拓扑链路排序、走廊定向包围盒（OBB）几何投影切割与多档点云无损导出的深度计算模块。
_Avoid_: SpanSplitter, CorridorManager, LasSlicer

**UpperTowerBox**:
覆盖输电线路铁塔塔头、多层横担及绝缘子串金具的定向三维紧致矩形体包围盒（OBB），以横担展宽主轴和顺线厚度为主轴，自适应适配跨电压等级绝缘子串长度。
_Avoid_: UpperBoundingBox, TowerHeadCube, TopBox

**LowerTowerFrustum**:
自横担下沿基准线以物理放坡斜率向下延伸至塔脚基础的空间四棱截锥体包围盒，配合塔腰向下的三维骨架体素连通生长，实现塔腿角钢与地表林木的精准剥离。
_Avoid_: LowerCone, TowerPyramid, BottomBox

**WaistBoundaryElevation**:
划分 UpperTowerBox 与 LowerTowerFrustum 的水平空间分界基准高程，严格定义在最下层横担及耐张跳线弧垂底端下方安全裕量处，阻断上部大包络对下部林木的误吞。
_Avoid_: SplitHeight, CutPlane, TowerWaistZ

