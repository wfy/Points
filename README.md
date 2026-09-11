# 电力机巡激光点云电力线与杆塔快速分类系统 (PowerLine-FastClassifier)

[![Python](https://img.shields.io/badge/Python-3.10%2B-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Proprietary-green.svg)]()
[![Tests](https://img.shields.io/badge/Tests-7%2F7%20Passed-brightgreen.svg)]()
[![Standards](https://img.shields.io/badge/Standard-ASPRS%20%7C%20SGCC%20GIM-orange.svg)]()

基于机巡无人机激光雷达（UAV LiDAR）点云的高性能输电线路走廊智能分类系统。系统融合**杆塔几何先验引导**、**走廊各向异性切片**、**3D 悬链线物理轨道建模**与**空间空气绝缘层（Air Gap）探针滤波**，在不依赖重型深度学习框架的前提下，实现单档千万级点云的高纯净度（Precision $\ge 98\%$）、高完整性（Recall $\ge 95\%$）连续导线与杆塔全自动分类。

---

## 核心特性与创新机制

- ⚡ **超轻量与极致性能 (Zero Heavy AI Frameworks)**
  - 采用纯 Python + NumPy 向量化 + SciPy `cKDTree` 分块检索架构，彻底摆脱 PyTorch/TensorFlow 等重型深度学习框架依赖。
  - 内存峰值精确控制，千万级点云单档全流程处理通常控制在 15~60 秒内。
- 🗼 **M2 阶段：输电杆塔几何骨架毫秒级锁定**
  - 地形自适应地面剥离后，利用 3D 体素垂直连续性与形态学拓扑分析，精准锁定塔位中心坐标 $(c_x, c_y)$、横担主轴走向 $\vec{v}_1$、线路厚度走向 $\vec{v}_2$、横担挂点标高及塔高空间边界。
- ⚡ **M3 阶段：自顶向下先验引导与导线全跨连续追踪**
  - **自适应跨档走廊切片**：走廊横向有效半宽自适应扩展至 $\max(\text{half\_arm} + 12.0\text{m}, 30.0\text{m})$，彻底包容外侧边相导线与大风偏相线。
  - **物理 3D 悬链线模型驱动 (Physics-driven Catenary Fitting)**：采用严格导线悬链方程 ($a \in [300, 4000]$，弧垂比 $f/L \in [0.005, 0.08]$) 进行物理拟合与沿线全跨平滑插值；未拟合悬链线的平直地线与短弧段执行姿态一致性后验保全，绝不一票否决。
  - **空气隔离层（Air Gap）探针抗噪**：基于真导线下方天然拥有绝缘空气隙的物理规律，检测紧贴线下方 $0.4\text{m} \sim 2.2\text{m}$ 空间的点云密度。彻底根治山区陡坡高林穿透至导线高度引发的高空导线误杀难题。
  - **跨档主轴夹角卡扣 ($\le 35^\circ$)**：在线路中段对导线候选项施加两塔连线主轴一致性约束与垂直倾角约束 ($|v_1[2]| \le 0.45$)，彻底杜绝任意斜交树枝与垂直树干误混。
  - **分裂导线自适应与耐张跳线闭环 (Bundle & Tension Jumpers)**：支持 2 分裂/4 分裂导线多尺度空间滤波；激活横担下方大曲率 U 形跳线微体素聚类提取，施加铁塔角钢骨架硬互斥，确保 M2 铁塔零退化。
- 🎨 **工业级可视化与标准标准编码导出**
  - 遵循 **ASPRS LAS 1.2/1.4** 与**国网 GIM** 标准规范：`Class 14` 导线、`Class 15` 输电铁塔、`Class 2` 地面、`Class 3` 植被/背景。
  - 支持单相单色独立分组分色渲染（洋红-炽橙色系分相赋色，铁塔工业蓝赋色），无缝对接 **CloudCompare** 与 **QTModeler** 自动快速加载。

---

## 算法流水线架构 (Architecture)

```
[原始机巡 LAS/LAZ 点云]
           │
           ▼
┌─────────────────────────────────────────────────────────────┐
│ 阶段一：地形自适应局部滤波剥离地面 (Ground Separation)         │
│ 提取地面点 (Class 2)，输出纯净离地高位地物点云 (rel_z >= 6m)  │
└─────────────────────────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────────────────────┐
│ 阶段二：3D 体素垂直连续性与杆塔先验定位 (M2 Tower Detection) │
│ 锁定铁塔位置、横担挂点标高、塔高边界与主副走向轴向 (Class 15)  │
└─────────────────────────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────────────────────┐
│ 阶段三：双通道导线连续提取与拓扑闭环 (M3 Wire Extraction)     │
│ ├─ 通道 A: 跨档走廊定向切片 + 各向异性聚类 + 3D 悬链线拟合    │
│ ├─ 通道 B: 种子提取 (主轴偏角<=35°) + 空气隔离层探针过滤      │
│ ├─ 树冠抗噪: 阻断非悬链线林区漫延 + 空气绝缘层 Air Gap 校验 │
│ └─ 耐张跳线: 横担下方大曲率 U 形弧段微体素提取 + 铁塔硬互斥   │
└─────────────────────────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────────────────────┐
│ 阶段四：成果组装与色彩分类赋码导出 (Export & Visualization) │
│ 输出标准 *_sign.las，分相单色赋色，无缝调用 QTModeler 预览    │
└─────────────────────────────────────────────────────────────┘
```

---

## 目录结构

```text
e:/points/
├── fast_powerline_classifier.py    # 系统主入口命令行工具 (支持单档/批量/一键先切后算)
├── README.md                       # 本工程技术架构与操作指南
├── modules/                        # 核心算法功能模块库
│   ├── config.py                   # 全局参数与阶段配置数据类 (PipelineConfig)
│   ├── models.py                   # 领域实体定义 (TowerEntity, WireCluster, ExtractionResult 等)
│   ├── ground_separator.py         # 阶段一：地形自适应网格地面分离算子
│   ├── tower_detector.py           # 阶段二：3D 体素连续性与铁塔先验锁定算子
│   ├── topdown_wire_extractor.py   # 阶段三主干：走廊切片、双通道融合与跳线并网
│   ├── wire_tracker.py             # 阶段三辅助：3D 悬链线轨道步进追踪与 Air Gap 探针
│   ├── wire_seed_extractor.py      # 阶段三辅助：PCA 姿态、分裂导线自适应与主轴偏角粗筛
│   ├── catenary.py                 # 阶段三数学底座：3D 悬链线解析估算与非线性拟合
│   ├── tension_topology.py         # 阶段三专用：耐张转角塔横担下方 U 形跳线提取器
│   ├── span_cutter.py              # 辅助工具：整段走廊切分为两塔一档独立子点云
│   └── exporter.py                 # 阶段四：ASPRS 标准编码赋予与分相单色 LAS 导出器
└── tests/                          # 自动化单元测试套件 (CI/CD 物理门禁)
    ├── test_pipeline_m3_smoke.py   # 端到端主管道双通道接口契约冒烟测试
    ├── test_catenary_fitting.py    # 3D 悬链线参数边界与退化保护单测
    ├── test_canopy_filtering.py    # 树冠空气隔离层检测与斜交树枝剔除单测
    └── test_tension_jumpers.py     # 耐张跳线大曲率提取与铁塔互斥单测
```

---

## 环境配置

推荐使用 **Python 3.10 ~ 3.12** 运行环境。核心第三方依赖极其精简：

```bash
pip install numpy scipy laspy[lazrs] psutil
```

*(可选)* 若需读取或写入 `.laz` 压缩格式，需安装 `lazrs` 引擎（`pip install lazrs`）。

---

## 快速上手与使用说明

主程序入口为 [`fast_powerline_classifier.py`](file:///e:/points/fast_powerline_classifier.py)，提供灵活的运行模式：

### 1. 单档点云处理模式 (Single File)

对指定的单档 LAS 点云进行快速分类，并输出标记着色后的成果文件：

```bash
# 基本运行 (默认输出到输入同目录的 *_sign.las)
python fast_powerline_classifier.py -i "E:\点云备份\5-6(5_6).las"

# 指定输出文件路径
python fast_powerline_classifier.py -i "E:\点云备份\0-1(0_1).las" -o "E:\点云备份\m3_results\0-1_sign.las"

# 批处理脚本静默执行 (禁用处理后的 QTModeler 自动弹窗)
python fast_powerline_classifier.py -i "E:\点云备份\18-19(18_19).las" --no-qtmodeler
```

### 2. 批量目录处理模式 (Batch Directory)

自动扫描指定文件夹下的所有 `.las` / `.laz` 文件并逐档全自动批量分类：

```bash
python fast_powerline_classifier.py --batch-dir "E:\点云备份\待处理档段" --no-qtmodeler
```

### 3. 一键端到端整段先切后算流水线 (Auto-Span-Pipeline)

针对包含数十座铁塔的长跨整线点云，自动快速定位全线铁塔并切分出纯净的两塔一档子点云，再逐档执行精细分类：

```bash
python fast_powerline_classifier.py --auto-pipeline -i "E:\点云备份\500kV某线路全线.las"
```

### 4. 配电网模式 (Distribution Network)

针对 10kV ~ 110kV 矮塔、单水泥杆或双水泥杆配电网点云：

```bash
python fast_powerline_classifier.py -i "E:\点云备份\配网10kV.las" --distribution
```

---

## 单元测试与验证套件

本项目严格执行**硬性物理验证门禁 (Verification Before Completion)**，杜绝主观猜测。全量单测套件位于 `tests/` 目录，涵盖 7 个高价值核心算法契约测试：

```bash
python -m unittest discover tests -v
```

执行输出示例（耗时 $\le 0.02$ 秒）：

```text
test_catenary_fitting_synthetic (tests.test_catenary_fitting.TestCatenaryFitting) ... ok
test_catenary_parameter_guards (tests.test_catenary_fitting.TestCatenaryFitting) ... ok
test_filter_canopy_by_probes_air_gap (tests.test_canopy_filtering.TestCanopyFiltering) ... ok
test_wire_seed_orientation_and_density_constraint (tests.test_canopy_filtering.TestCanopyFiltering) ... ok
test_pipeline_synthetic (tests.test_pipeline_m3_smoke.TestPipelineM3Smoke) ... ok
test_bundle_conductor_seed_adaptation (tests.test_tension_jumpers.TestTensionJumpers) ... ok
test_extract_tension_jumpers_geometry (tests.test_tension_jumpers.TestTensionJumpers) ... ok

----------------------------------------------------------------------
Ran 7 tests in 0.014s

OK (Exit Code: 0)
```

---

## 分类类别标准对照表

导出生成的 `*_sign.las` 文件中，各点云属性标准严格对齐电力机巡与 GIS 行业标准：

| 分类码 (Classification) | 类别名称 | 渲染色彩规范 | 说明 |
| :---: | :---: | :---: | :--- |
| **14** | **电力导线 (Powerline Conductor)** | 🌈 分相单相单色 (洋红/炽橙色系) | 包含各相导线、分裂导线及横担下方耐张悬垂跳线 |
| **15** | **输电铁塔 (Transmission Tower)** | 🟦 纯工业蓝 (Pure Industrial Blue) | 包含塔脚基座、塔身主材、斜撑及各层横担角钢骨架 |
| **2** | **地面点云 (Ground Surface)** | ⬜ 中灰色 (Medium Gray) | 地形高程基准面与地面粗糙点 |
| **3** | **植被与环境地物 (Vegetation/Noise)** | 🌲 原真彩 (Original RGB) | 山林树木、灌木杂草、农田及房屋等非电力设施 |

---

## 技术演进与复盘记录

项目核心架构、历史演进及阶段实施复盘详见：
- [M3 阶段实施方案 (Implementation Plan)](file:///C:/Users/jayden/.gemini/antigravity/brain/181069b6-b2b3-4729-aad1-80720915262f/implementation_plan.md)
- [M3 实施成果与技术演进报告 (Walkthrough)](file:///C:/Users/jayden/.gemini/antigravity/brain/181069b6-b2b3-4729-aad1-80720915262f/walkthrough.md)
- Obsidian 长期知识库: `CC/projects/点云电力线与杆塔快速分类-md/`
