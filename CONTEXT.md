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

**PhysicalProportionGuard**:
输电线路铁塔上下空间几何比例熔断机制，刚性约束最下层横担基准高程（WaistBoundaryElevation）不得低于全塔净高的 30%，且横担半宽必须相对于塔身立柱存在阶跃突变，阻断下塔身放坡角钢误判为横担。
_Avoid_: RatioRule, HeightCheck, SanityClamp

**GridSpatialIndex**:
二维粗网格空间分桶索引，复用 2D 网格将点云映射至局部网格桶，以纯切片索引完全替代全图级数千万点的 cKDTree，在毫秒级内完成候选塔圆形包围域的精确零拷贝过滤。
_Avoid_: FullKDTree, GlobalPointCloudTree, SpatialHashDict

**SortedSliceAggregation**:
一维单趟预排序切片聚合算法，对非地面点按网格编号执行单趟排序并借助连续指针切片统计每个网格的垂直连续性、唯一高度层与重心加权，彻底废除在全量点云大数组上的多重布尔掩膜广播。
_Avoid_: MaskLoop, BroadcastLookup, GridUniqueLoop

**AsymmetricCrossarmBox**:
覆盖耐张转角塔等非对称金具的横担双侧独立定向矩形包围盒，对横担轴向正负两翼分别度量实际外展并施加独立半宽，彻底废除宽侧必须被窄侧硬剪的刚性对称假设，允许单侧大弧垂跳线与金具完整保留。
_Avoid_: SymmetricArmBox, RigidArmOBB, ForcedSymmetricBox

**TallMultiCircuitLowerBoundary**:
高耸多回路输电铁塔横担扫描下延先验，在总高大（>=45m）且沿塔身纵向排布多层横担的重型铁塔上，将横担切片初筛起始门槛放宽至 35% 塔高（结合 12m 绝对防树底线），确保最底层横担被完整检出，防止蓝黄分割线腰斩倒挂。
_Avoid_: FixedHalfHeightFilter, HighTowerRatioCut

**AdaptiveFrustumSlope**:
下半部四棱台自适应物理放坡增长率，将塔身向下的放坡斜率上限放宽至 0.15 m/m，配合塔脚 1.2m 空间体素连通生长与陡坡基准高程解耦，完整捕获大开度耐张塔腿与山地长短腿塔靴。
_Avoid_: FixedSlopeCap, StiffConeRate


