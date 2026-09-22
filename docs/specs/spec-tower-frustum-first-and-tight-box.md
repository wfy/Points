# Spec: Frustum-First Tower Anchor, Single-Orientation Alignment, and Bounding Box Convergence

**Triage Label**: `ready-for-agent`

## Problem Statement

在对无人机激光雷达点云进行输电线路杆塔提取与点云分类的实测中，高压输电铁塔的几何形态收敛遭遇了三类严重的几何与分类缺陷：
1. **上半部包围盒过度外扩吞噬导线（0-1# 图 1）**：在直线悬垂或耐张铁塔中，上半部矩形包围盒的顺线厚度下限被硬编码固定在 5.5m，且横担半宽计算存在过度的百分比外推。当跨中导线脱离耐张绝缘子串继续向档中延伸时，这些非连接导线点被错误卷入铁塔长方体包围盒中并染为蓝色（杆塔分类代码 15），破坏了铁塔与导线的独立性；
2. **转角耐张塔朝向扭偏与 45° 剪刀差（5-6# 图 2）**：转角耐张塔横担通常沿转角平分线布置，导致塔头横担与跨档两塔直线连线存在夹角。原有算法通过两塔连线粗先验进行正交投影仲裁时，错误锁定了塔脚 4 根主腿的 45° 对角线方向（34.3°），而非塔身真实外立面（-15.4°/74.6°）。下部四棱台以 45° 斜向菱形切正方形塔身，导致俯视图中上部蓝色横担与下部黄色塔基产生严重的剪刀差错位；
3. **高塔下部放坡冒充横担导致蓝黄分割线下坠（68-69# 图 3）**：当铁塔高度达到 56m 以上时，下半身四棱台自然外展放坡，在 16m 处塔腿半宽已达到 8.1m，满足了粗糙的“展宽大于5m且长宽差大于2m”的切片横担判定条件。同时由于 68-69# 塔横担左右极其不对称（左侧 16.6m，右侧 3.3m），原自顶向下算法在高空计算塔心被向单侧拉偏达 3.8m。中心拉偏加上放坡误判，导致最下横担基准高程一路“雪崩式坠落”至离地 14m 处（整座塔 97.2% 被当成上部蓝色盒子），造成下半身黄色塔身几乎消失，塔脚角钢大量漏识。

## Solution

建立“以纯净四棱台为几何锚点的自底向上定姿（FrustumFirstAnchor）+ 全塔单一刚体朝向对齐 + 物理几何比例熔断（PhysicalProportionGuard）+ 紧致矩形体收敛”的系统性解决方案：
1. **四棱台自底向上基准定姿**：利用地表上方 5m~20m 纯净四棱台区间的 4 根刚性角钢主腿，拟合出铁塔毫米级精度的物理对称中心与四立面正交外墙朝向，彻底摆脱高空非对称横担与导线对塔心和朝向的污染；
2. **全塔单一刚体朝向锁定**：上半部 UpperTowerBox 与下半部 LowerTowerFrustum 共享完全一致的正交朝向，矩形体包围盒朝向即为铁塔物理朝向，消除菱形错位；
3. **物理几何比例熔断**：在领域模型中植入国网输电铁塔土木结构工程规律，刚性约束最下层横担高程不得低于全塔净高的 30%，并引入相对塔腰立柱的展宽阶跃突变（$\Delta w \ge 2.5\text{m}$）检验，杜绝塔身自然放坡误判为横担；
4. **UpperTowerBox 紧致收敛**：废除顺线厚度 5.5m 硬编码下限，使顺线厚度紧密贴合绝缘子挂点端部（3.0m~3.8m），缩减横担半宽外推裕量，使上部严格收敛为紧致长方体 OBB，自然阻断非连接导线入盒。

## User Stories

1. As a power grid GIS engineer, I want the tower center to be computed from the 4 foundation legs near the ground, so that asymmetric crossarms do not pull the tower center sideways.
2. As a LiDAR inspection developer, I want the tower orientation vectors to align with the physical rectangular faces of the tower body, so that the lower frustum is not rotated into a 45-degree diamond relative to the legs.
3. As an autonomous drone surveyor, I want the upper tower box to have its orientation strictly parallel to the tower body orientation, so that the entire tower structure is unified without angular shear.
4. As a transmission line operator, I want the waist boundary elevation between upper box and lower frustum to have a hard floor at 30% of the total tower height, so that the boundary never collapses to the ground on tall towers.
5. As an inspection data processor, I want crossarm slice detection to require a sharp outward jump in width relative to the trunk, so that natural four-legged slope expansion on tall towers is never mistaken for a crossarm.
6. As a power line classifier user, I want the upper tower bounding box along the wire direction to be tightly bounded by insulator attachments rather than an arbitrary 5.5m minimum, so that free span conductors outside the insulator strings are excluded from the tower.
7. As a downstream 3D modeling engineer, I want the upper tower point cloud to form a clean, tight rectangular cuboid (OBB), so that CAD/GIM models can be fitted directly without wire spikes.
8. As an inspection QA specialist, I want lower tower leg points on angle towers (like 5-6#) to be completely captured by the frustum box without clipping, so that corner steel flange connections are preserved for clearance auditing.
9. As a data pipeline operator, I want tall tension towers (like 68-69#) to maintain a realistic upper-to-lower height ratio between 35%:65% and 50%:50%, so that lower tower lattice members are not misclassified as vegetation or ground.
10. As a field inspection technician, I want suspension towers (like 0-1#) to cleanly separate span conductors from tower steelwork, so that conductor clearance measurements are not contaminated by false tower points.

## Implementation Decisions

### 1. Frustum-First Anchor Engine in Tower Detector
- 铁塔几何定姿流程改为自底向上：
  1. 初筛体素候选后，首先在地面以上纯净四棱台高度区间（相对高程 $z \in [0.15 \times H, 0.40 \times H]$ 且 $z \ge 5.0\text{m}$）提取纯立柱角钢点；
  2. 聚类提取 4 根物理主腿截面中心，通过 4 点交点计算全塔物理中轴 $(c_x, c_y)$；
  3. 计算 4 根腿围成的正方形外立面法向，生成铁塔基准正交单位向量 $(\vec{v}_1, \vec{v}_2)$；
  4. 若低位受灌木干扰未能提取出 4 根腿，退化为在横担下方纯立柱切片进行中轴与立面自对齐。

### 2. Physical Proportion Guard in Crossarm Slicer
- 最下层横担高程搜索与判定机制：
  1. 刚性搜索下限：切片扫描下限设置为 $z_{\text{min}} = \max(0.30 \times H, 10.0\text{m})$；
  2. 横担悬臂阶跃检测：切片层展宽 $w_{\text{span}}$ 必须相对于基准塔腰纯立柱半宽 $w_{\text{trunk}}$ 存在显著跳变（要求 $w_{\text{span}} - w_{\text{trunk}} \ge 2.5\text{m}$ 且 $w_{\text{span}} / w_{\text{trunk}} \ge 1.6$）；
  3. 剔除等边正方形粗基底：当 $w_1 \approx w_2$ 且缺乏单向外展扁平特征时，直接否决横担属性。

### 3. Tight UpperTowerBox Dimensioning
- 上半部长方体尺寸约束：
  1. 废除 `half_line_t` 的 `max(..., 5.5)` 硬编码，改为依据塔身立柱厚度加绝缘子端部裕量（允许范围收敛至 $3.0\text{m} \sim 3.8\text{m}$）；
  2. 横担半宽 `half_arm_w` 采用 $99\%$ 分位数加 $0.3\text{m} \sim 0.5\text{m}$ 制造公差，废除 $18\% + 1.2\text{m}$ 的过度外推；
  3. 保持标准的定向矩形体（OBB）几何形态。

### 4. Architectural Compatibility
- 完全保持 `TowerEntity` 接口、`detect_towers` 函数签名及下游 `PipelineExecutor` / `WireExtractor` 的契约不变，所有改动内聚在几何计算与切片过滤层。

## Testing Decisions

### What Makes a Good Test
- 好的测试仅验证铁塔提取与几何收敛的**外部行为与物理性质**，包括：
  1. 几何中轴与下塔身主腿对称中心的偏差（$< 0.5\text{m}$）；
  2. 主轴朝向与下塔腿外立面的夹角（$< 5^\circ$，严禁 45° 倒错）；
  3. 蓝黄分割线相对塔高的比例（处于 $30\% \sim 65\%$ 物理合理区间，严禁 $< 30\%$ 或 $> 85\%$）；
  4. 上半部 OBB 盒顺线厚度（$< 4.2\text{m}$，阻断跨中出线）；
  5. 既有 61 项全量单元测试（回归基准）保持 100% 通过。

### Modules Tested
- `modules/tower_detector.py`
- `tests/test_tower_detector.py`
- `tests/test_pipeline_executor.py`

### Prior Art
- `tests/test_tower_detector.py` 中针对两段式几何收敛的已有参数化单测。

## Out of Scope
- 绝缘子串挂点金具内部各零部件的微观网格重建（由阶段三/四点级分类处理）；
- 跨档导线的悬链线物理张力拟合（由 `Catenary` 模块独立负责）。

## Further Notes
- 本规范已同步记录至 ADR `docs/adr/0006-frustum-first-tower-anchor-and-bounding-box-convergence.md`，并在 `CONTEXT.md` 中注册了 `FrustumFirstAnchor` 与 `PhysicalProportionGuard` 术语。
