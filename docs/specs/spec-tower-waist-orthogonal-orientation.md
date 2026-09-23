# Spec: Tower Waist Orthogonal Orientation and RANSAC Arbitration

**Triage Label**: `ready-for-agent`

## Problem Statement

在无人机激光雷达电力巡检点云的杆塔几何分割中，部分耐张塔与转角塔（如标杆数据 `5-6(5_6).las` 2 号塔）存在明显的**上下部几何朝向剪刀差（Scissor Mismatch）与下半身菱形斜切**问题：
1. **转角耐张塔朝向扭偏（剪刀差）**：原算法在高空通过 RANSAC 拟合横担或散点 PCA 时，容易受到端部跳线环、倾斜绝缘子串及非对称悬臂的干扰，得到的横担方位角（如 130.2°）与塔身下半身物理外立面（实测为 144.5°）存在 14°~22° 的夹角偏折。此时下塔身四棱台以此扭偏方向进行放坡包络，导致俯视视角下蓝色的上部长方体与黄色的下部四棱台产生刺眼的“剪刀差”，塔身角钢被斜切；
2. **前期修复引发的世界坐标轴向死锁缺陷**：前期尝试在世界坐标系下按象限划分 4 根腿以提取立面，但该假设严重背离实际——绝大多数输电线路在地理空间上具有非零走向角。硬按世界坐标切分导致全网几乎所有正常直线塔的朝向被强行扭转为 0°/90°，放大了缺陷面；
3. **高空散点外展评判极易发生 90° 倒挂**：如果在高空仅根据散点在两轴上的投影跨度（Percentile Span）判断哪根轴是横担，在顺线跨档导线较长或耐张跳线弧垂外扩场景下（如 `68-69#` 与 `0-1#`），顺线方向的散点展宽往往超过横担宽度，导致算法误将顺线导线认定为横担，发生致命的 90° 轴向倒挂。

---

## Solution

采用**“塔腰纯净段正方形截面正交定姿 + 高空金属角钢 RANSAC 直线内积仲裁”**的双层几何融合方案：
1. **塔腰纯净段正交基底提取 (Waist-Section MinArea-OBB)**：
   在蓝黄分割线下方 $[Z_{\text{lowest\_arm}} - 3.5\text{m}, Z_{\text{lowest\_arm}} - 0.5\text{m}]$ 的纯净塔腰区间提取点云。该区间上无横担、绝缘子与导线，下无地面起伏、杂草与断腿，4 根主材角钢构成毫米级致密的刚性正方形截面。通过 2D 最小外接面积扫描（MinArea-OBB）在 $[0, 90^\circ)$ 范围内求解使外接面积最小的旋转角，解析出塔身外立面严格互相垂直的正交双主轴 $(\vec{u}_a, \vec{u}_b)$；
2. **高空横担角钢 RANSAC 直线内积仲裁 (RANSAC Dot-Product Arbitration)**：
   利用 2D RANSAC 在高空金属角钢层拟合出的笔直横担直线向量 $\vec{v}_{\text{arm}}$，分别与 $(\vec{u}_a, \vec{u}_b)$ 计算绝对点积值：$|\vec{v}_{\text{arm}} \cdot \vec{u}_a|$ vs $|\vec{v}_{\text{arm}} \cdot \vec{u}_b|$。点积更大者（与笔直角钢最平行）锁定为横担主轴 $\vec{v}_1$，另一根锁定为顺线走廊厚度轴 $\vec{v}_2$；
3. **全塔单一刚体正交轴系 (Single-Rigid-Orientation)**：
   上半部 UpperTowerBox 与下半部 LowerTowerFrustum 严格共享 $(\vec{v}_1, \vec{v}_2)$，上下包围盒同轴共体，俯视剪刀差在数学上恒等于 0°，且 100% 杜绝 90° 轴向倒挂。

---

## User Stories

1. As a power line point cloud algorithm engineer, I want the tower orientation to be derived from the pure square waist section below the lowest crossarm, so that ground bushes and high-altitude conductors do not distort the body axes.
2. As a transmission inspection technician, I want angle tension towers (such as 5-6# Tower 2) to have perfectly aligned upper and lower bounding envelopes, so that the 14°~22° visual scissor mismatch between blue and yellow segments is completely eliminated.
3. As a GIS modeling specialist, I want the tower bounding box faces to align with the physical lattice faces of the tower trunk, so that the lower frustum does not cut the 4 corner legs into diamond diagonals.
4. As an autonomous drone pipeline operator, I want the orientation solver to be completely coordinate-free and isotropic, so that lines running at any arbitrary geographic azimuth (e.g., 25°, 70°, 135°, 160°) are treated identically without bias toward East/North axes.
5. As a power grid classification analyst, I want the crossarm axis selection to be arbitrated by rigid steel RANSAC lines rather than raw point cloud dispersion, so that long overhead conductors never cause a 90-degree axis inversion.
6. As a 3D clearance auditing engineer, I want suspension towers (like 0-1# Tower 1) with equal longitudinal and transverse spans to correctly identify the crossarm, so that the corridor thickness envelope is never swapped with the arm span.
7. As an inspection QA reviewer, I want ultra-high tension towers (like 68-69#) with asymmetric cantilever arms to maintain stable body orientations within 2 degrees of physical blueprints, so that corner leg flanges are 100% enclosed within the lower frustum.
8. As a lidar data processing operator, I want the waist-section orientation algorithm to execute deterministically in sub-millisecond time (<1ms), so that batch point cloud classification throughput is not degraded.
9. As a software architect, I want the orientation fix to integrate seamlessly with the existing `PhysicalProportionGuard` (30% waist floor) and tight `UpperTowerBox` (4.2m line thickness), preserving 100% of the Q1 and Q3 verification gains.
10. As a field surveyor, I want both straight-line towers and large-angle tension towers to pass through the same unified orientation pipeline without manual mode switching or fragile heuristics.

---

## Implementation Decisions

### 1. Pure Waist Slice Sampling & MinArea-OBB Orientation
- **采样区间**：高程窗口限制在 $[Z_{\text{lowest\_arm}} - 3.5\text{m}, Z_{\text{lowest\_arm}} - 0.5\text{m}]$，平面径向限制在 $\text{dist} \le 4.0\text{m}$。若点数不足 15 点，轻微向下扩展至 $[Z_{\text{lowest\_arm}} - 4.5\text{m}, Z_{\text{lowest\_arm}}]$；
- **最小外接矩形求解**：
  在 $[0, 90^\circ)$ 范围内以 $0.5^\circ$ 步长构造正交基底 $(\vec{u}_1(\theta), \vec{u}_2(\theta))$，投影计算跨度 $w_1(\theta) = P_{98}(p \cdot \vec{u}_1) - P_2(p \cdot \vec{u}_1)$ 与 $w_2(\theta) = P_{98}(p \cdot \vec{u}_2) - P_2(p \cdot \vec{u}_2)$，寻找使包络面积 $A(\theta) = w_1(\theta) \times w_2(\theta)$ 达到全局极小值的角度 $\theta^*$；
  正方形截面在四边平行处外接面积取最小值 $L^2$，对角线处取最大值 $2L^2$，由此解析出与塔腰四面完全平行的正交基底对 $(\vec{u}_a, \vec{u}_b)$。

### 2. High-Altitude Crossarm RANSAC Dot-Product Arbitration
- 利用高空高密度金属角钢层（$Z \in [0.70H, 0.98H]$，$\text{dist} \le 9.5\text{m}$）通过 2D RANSAC 拟合出刚性金属横担直线向量 $\vec{v}_{\text{arm}}$；
- 若 RANSAC 成功拟合（$\text{inliers} \ge 8$）：
  - 计算内积绝对值：$\text{dot}_a = |\vec{u}_a \cdot \vec{v}_{\text{arm}}|$，$\text{dot}_b = |\vec{u}_b \cdot \vec{v}_{\text{arm}}|$；
  - 若 $\text{dot}_a \ge \text{dot}_b$，则 $\vec{v}_1 = \vec{u}_a, \vec{v}_2 = \vec{u}_b$；否则 $\vec{v}_1 = \vec{u}_b, \vec{v}_2 = \vec{u}_a$；
- 若 RANSAC 因点云稀疏未检出有效直线，退化为两塔跨档连线法向 $\vec{u}_{\text{arm\_prior}}$ 仲裁，仍不可用时降级为高空散点主惯性轴对比。

### 3. Rigid Orientation Propagation & Boundary Clamping
- 确定出的 $(\vec{v}_1, \vec{v}_2)$ 作为全塔（UpperTowerBox 与 LowerTowerFrustum）唯一的基准轴系；
- 保持 Q1（紧致长方体 OBB：`half_line_t <= 4.2m`，阻断跨中出线）与 Q3（30% 比例熔断：`min_waist_floor = max(h_true * 0.30, 6.0)`）完全生效。

---

## Testing Decisions

### What Makes a Good Test
- 好的测试只验证外部几何行为与物理一致性，不依赖脆弱的局部实现：
  1. **无剪刀差验证**：验证计算出的横担主轴 $\vec{v}_1$ 与塔腰正方形截面法向夹角必须为 $0^\circ$（严格平行）；
  2. **抗 90° 倒挂验证**：在长导线大跨档数据（如 68-69#）上，验证 $\vec{v}_1$ 严禁落入顺线走向（与两塔跨档连线的内积夹角严禁 $< 45^\circ$）；
  3. **各向同性验证**：对输入点云施加任意空间旋转（如 $33^\circ, 77^\circ, 142^\circ$），输出的定向包围盒朝向随之等角度旋转，无世界轴吸引现象；
  4. **全量回归保障**：既有 61 项单元测试 100% 保持通过（Exit Code: 0）。

### Modules Tested
- `modules/tower_detector.py`
- `tests/test_tower_detector.py`
- `tests/test_pipeline_executor.py`

### Prior Art
- `test_tower_detector.py` 中已有针对耐张塔切片与双轴正交性验证的测试用例。

---

## Out of Scope
- 四棱台内部单个角钢桁架的曲面重建；
- 绝缘子连接金具内部微观构件的分类与三维建模。

---

## Further Notes
- 本规范补充了 ADR 0006 中关于转角耐张塔单一刚体定姿的微观落地准则，彻底消除了上一版方案中世界坐标系与单纯外展比值带来的两个工程缺陷。
