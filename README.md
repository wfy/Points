# 电力机巡激光点云电力线与杆塔快速分类系统 (PowerLine-FastClassifier)

[![Python](https://img.shields.io/badge/Python-3.10%2B-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Proprietary-green.svg)]()
[![Tests](https://img.shields.io/badge/Tests-58%2F58%20Passed-brightgreen.svg)]()
[![Architecture](https://img.shields.io/badge/Architecture-Deep%20Modules%20(ADR--0001~0004)-purple.svg)]()
[![Standards](https://img.shields.io/badge/Standard-ASPRS%20%7C%20SGCC%20GIM-orange.svg)]()

基于机巡无人机激光雷达（UAV LiDAR）点云的高性能输电线路走廊智能分类系统。系统融合**杆塔几何先验引导**、**走廊各向异性切片**、**3D 悬链线物理轨道建模**、**空间空气绝缘层（Air Gap）探针滤波**与**面向对象深度模块架构**，在不依赖重型深度学习框架的前提下，实现单档千万级点云的高纯净度（Precision $\ge 98\%$）、高完整性（Recall $\ge 95\%$）连续导线与杆塔全自动分类。

---

## 核心特性与架构亮点

- ⚡ **超轻量与极致性能 (Zero Heavy AI Frameworks)**
  - 采用纯 Python + NumPy 向量化 + SciPy `cKDTree` 分块检索架构，彻底摆脱 PyTorch/TensorFlow 等重型框架与 GPU 依赖。
  - 百万级点云全流程分类通常在 **10~15 秒** 内完成，内存峰值精确控制。
- 🏗️ **深度高内聚模块架构 (Deep Modules & Clean Seams)**
  - **`PipelineExecutor` 核心调度器**：统一管理地面剥离、杆塔定位、导线提取与成果导出四大阶段，提供纯内存矩阵接缝与文件级门面，支持基于 `PipelineStage` 的灵活提前终止（短路执行）。
  - **`WireExtractor` 双通道聚合器**：走廊切片与体素种子追踪协同互补，引入残差增量短路机制，内部严密封装并查集拓扑与单调线性编号。
  - **`CorridorCutter` 定向切片服务**：基于主轴与法向单位向量的全矢量化 OBB 定向包围盒切分；杆塔排序引入航向平滑偏转惩罚（$\cos\theta$ 夹角加权），有效抑制转角塔跳跃；分类成果直接在内存切分，消除二次磁盘 I/O。
  - **`ViewerHandler` 协议化视口**：抽象视口协议解耦操作系统进程管理（taskkill），支持 `QTModelerViewer`、`SystemDefaultViewer` 与静默 `NullViewer`；GUI 弹窗采用动态懒加载，100% 免疫无头服务器（Headless）环境崩溃。
- 🗼 **输电杆塔几何骨架锁定 (Tower Detection)**
  - 地形自适应地面剥离后，利用 3D 体素垂直连续性与形态学拓扑分析，精准锁定塔位中心坐标 $(c_x, c_y)$、横担主轴走向 $\vec{v}_1$、线路厚度走向 $\vec{v}_2$、横担挂点标高及塔高空间边界。
- ⚡ **自顶向下物理建模与导线连续追踪 (Wire Extraction)**
  - **物理 3D 悬链线模型驱动**：采用严格导线悬链方程 ($a \in [300, 4000]$，弧垂比 $f/L \in [0.005, 0.08]$) 进行物理拟合与全跨平滑插值；未拟合悬链线的平直地线与短弧段执行姿态一致性后验保全。
  - **空气隔离层（Air Gap）探针抗噪**：基于真导线下方天然拥有绝缘空气隙的物理规律，检测紧贴线下方 $0.4\text{m} \sim 2.2\text{m}$ 空间的点云密度，根治山区陡坡高林穿透引发的高空导线误分类难题。
  - **耐张跳线闭环提取**：激活横担下方大曲率 U 形跳线微体素聚类提取，施加铁塔角钢骨架硬互斥，保证铁塔与导线分界清晰。
- 🎨 **工业级可视化与标准编码导出**
  - 遵循 **ASPRS LAS 1.2/1.4** 与**国网 GIM** 标准规范：`Class 14` 导线、`Class 15` 输电铁塔、`Class 2` 地面、`Class 3` 植被/背景。
  - 成果 LAS 文件支持单相单色独立分组分色渲染，无缝对接 **CloudCompare** 与 **QTModeler**。

---

## 算法流水线架构 (Architecture)

```text
[原始机巡 LAS/LAZ 点云]
           │
           ▼  (PipelineExecutor / PipelineStage)
┌─────────────────────────────────────────────────────────────┐
│ 阶段一：地形自适应局部滤波剥离地面 (Ground Separation)         │
│ 提取地面点 (Class 2)，输出纯净离地高位地物点云 (rel_z >= 6m)  │
└─────────────────────────────────────────────────────────────┘
           │  (stop_after="ground" 短路点)
           ▼
┌─────────────────────────────────────────────────────────────┐
│ 阶段二：3D 体素垂直连续性与杆塔先验定位 (M2 Tower Detection) │
│ 锁定铁塔位置、横担挂点标高、塔高边界与主副走向轴向 (Class 15)  │
└─────────────────────────────────────────────────────────────┘
           │  (stop_after="tower" 短路点，支持纯切片模式)
           ▼
┌─────────────────────────────────────────────────────────────┐
│ 阶段三：双通道导线连续提取与拓扑闭环 (WireExtractor)         │
│ ├─ 通道 A: 跨档走廊定向切片 + 各向异性聚类 + 3D 悬链线拟合    │
│ ├─ 通道 B: 种子提取 (主轴偏角<=35°) + 空气绝缘层 Air Gap 校验 │
│ ├─ 残差协同: 通道 A 捕获点差量剔除，剩余高空点增量追踪       │
│ └─ 耐张跳线: 横担下方大曲率 U 形弧段微体素提取 + 铁塔硬互斥   │
└─────────────────────────────────────────────────────────────┘
           │  (stop_after="wire" 短路点)
           ▼
┌─────────────────────────────────────────────────────────────┐
│ 阶段四：成果组装、着色导出与分档切片 (Export & Visualization) │
│ ├─ 导出标准分类着色文件 (*_sign.las)                          │
│ ├─ 可选内存级两塔一档切片导出 (CorridorCutter -> spans/)     │
│ └─ 调用 ViewerHandler 协议安全加载至 QTModeler                │
└─────────────────────────────────────────────────────────────┘
```

---

## 代码库目录结构

```text
e:/points/
├── fast_powerline_classifier.py    # 生产主入口 CLI 脚本 (单档/批量/切片/流水线)
├── CONTEXT.md                       # 领域统一语言 (Ubiquitous Language) 与核心概念字典
├── README.md                        # 本工程技术架构与操作指南
├── docs/adr/                        # 架构决策记录 (Architecture Decision Records)
│   ├── 0001-deep-pipeline-executor-with-stage-control.md
│   ├── 0002-unified-wire-extractor-and-encapsulated-topology.md
│   ├── 0003-protocol-driven-viewer-and-gui-decoupling.md
│   └── 0004-corridor-slicing-dependency-inversion.md
├── modules/                         # 核心高内聚深度模块库
│   ├── pipeline_executor.py        # [Candidate 01] 流水线执行调度器 (PipelineExecutor)
│   ├── wire_extractor.py           # [Candidate 02] 统一导线提取深度模块 (WireExtractor)
│   ├── viewer.py                   # [Candidate 03] 视口抽象协议与处理器 (ViewerHandler)
│   ├── gui.py                      # [Candidate 03] 动态懒加载 GUI 对话框与 Headless 回退
│   ├── corridor_cutter.py          # [Candidate 04] 空间几何与 OBB 走廊切档服务 (CorridorCutter)
│   ├── models.py                   # 核心领域实体 (TowerEntity, WireCluster, SpanSegment, PipelineResult)
│   ├── config.py                   # 全局与阶段配置类 (PipelineConfig, PipelineStage)
│   ├── ground_separator.py         # 阶段一：地形自适应网格地面分离算子
│   ├── tower_detector.py           # 阶段二：3D 体素连续性与铁塔先验锁定算子
│   ├── catenary.py                 # 阶段三数学底座：3D 悬链线解析估算与拟合
│   ├── wire_tracker.py             # 阶段三辅助：3D 悬链线轨道步进追踪与 Air Gap 探针
│   ├── wire_seed_extractor.py      # 阶段三辅助：PCA 姿态与分裂导线自适应粗筛
│   ├── tension_topology.py         # 阶段三专用：耐张转角塔横担下方 U 形跳线提取器
│   └── utils.py                    # 纯净 I/O 工具函数 (分相分类 LAS 着色导出)
└── tests/                           # 自动化单元测试套件 (CI/CD 物理验证门禁)
    ├── test_pipeline_executor.py   # 流水线生命周期、阶段短路与日志回归测试
    ├── test_wire_extractor.py      # 双通道增量协同与拓扑并查集封装测试
    ├── test_wire_models.py         # 导线实体与向后兼容解包测试
    ├── test_viewer.py              # 视口协议多态、静默回退与生命周期管理测试
    ├── test_gui.py                 # GUI 对话框懒加载与 Headless 安全降级测试
    ├── test_corridor_cutter.py     # 走廊 OBB 切片、航向平滑惩罚与委托短路测试
    ├── test_span_models.py         # 档段实体与流水线挂载测试
    ├── test_catenary_fitting.py    # 3D 悬链线非线性拟合与参数约束测试
    ├── test_canopy_filtering.py    # 树冠空气隔离层检测与斜交枝剔除测试
    ├── test_tension_jumpers.py     # 耐张跳线大曲率提取测试
    └── test_pipeline_m3_smoke.py   # 真实机巡 1.37M 点云端到端冒烟测试门禁
```

---

## 环境配置

推荐使用 **Python 3.10 ~ 3.12** 运行环境。第三方依赖精简且无需编译：

```bash
pip install numpy scipy laspy[lazrs] psutil
```

*(可选)* 如需读取或写入 `.laz` 压缩格式点云，`laspy[lazrs]` 会自动安装 Rust 极速解压引擎。

---

## 用户操作指南与命令行用法 (User Guide)

主程序入口为 [`fast_powerline_classifier.py`](file:///e:/points/fast_powerline_classifier.py)，支持多种操作模式：

### 1. 标准单档全流程分类处理 (Single File)

对单档 LAS 点云进行全自动分类着色，输出标准 `*_sign.las` 成果文件：

```bash
# 基本运行 (默认输出到输入同目录的 *_sign.las，处理完自动调用 QTModeler 预览)
python fast_powerline_classifier.py -i "E:\点云备份\17-18(17_18).las"

# 显式指定输入与输出路径
python fast_powerline_classifier.py -i "input.las" -o "output_sign.las"

# 静默执行 (禁用详细阶段日志回显)
python fast_powerline_classifier.py -i "input.las" --quiet

# 无头服务器/无窗口执行 (禁用弹窗选择与处理后的 QTModeler 自动调用)
python fast_powerline_classifier.py -i "input.las" --no-gui --no-qtmodeler
```

### 2. 全流程分类 + 内存级两塔一档单档切片 (`--split-spans`)

在完成全线分类着色的同时，直接利用**内存态点云**按两塔一档 OBB 定向切分并导出各跨独立 LAS 文件（零二次磁盘 I/O）：

```bash
python fast_powerline_classifier.py -i "input.las" --split-spans
```
* **输出**：生成 `output_sign.las` 以及 `spans/` 目录下的各档已分类成果文件（例如 `input_Span1_#T1-#T2.las`）。

### 3. 纯走廊分档切片预处理 (`--split-only`)

针对多基铁塔的长跨原始点云，仅执行至杆塔锁定阶段（`stop_after=PipelineStage.TOWER` 动态短路），直接切出纯净的原始单档子点云供后续分发：

```bash
python fast_powerline_classifier.py -i "raw_corridor.las" --split-only
```
* **输出**：在 `spans_raw/` 目录下生成各档独立的原始未分类 LAS 文件。

### 4. 一键端到端先切后算自动化流水线 (`--auto-span-pipeline`)

针对超长多塔走廊，一键串联“全线找塔 -> 纯走廊快速切片 -> 各档独立精细化提取分类 -> 成果导出”：

```bash
python fast_powerline_classifier.py -i "long_corridor.las" --auto-span-pipeline
```

### 5. 批量目录处理模式 (`--batch-dir`)

自动扫描指定文件夹下的所有 `.las` / `.laz` 文件并逐档执行批量分类：

```bash
python fast_powerline_classifier.py --batch-dir "E:\点云数据\待处理档段" --no-qtmodeler
```

### 6. 配电网与矮杆塔模式 (`--distribution`)

针对 10kV ~ 110kV 矮塔、单水泥杆或双水泥杆配电网点云，自适应放宽塔高与地面净空约束：

```bash
python fast_powerline_classifier.py -i "10kV配网.las" --distribution
```

### 7. 阶段短路调试模式 (`--stop-after`)

在算法调试、参数调优或部分功能验证时，控制流水线在特定阶段完成后提前退出：

```bash
# 仅执行地面剥离 (Class 2)
python fast_powerline_classifier.py -i "input.las" --stop-after ground

# 仅执行至杆塔识别 (Class 2 + Class 15)
python fast_powerline_classifier.py -i "input.las" --stop-after tower

# 执行至导线提取，跳过拓扑挂接验证 (Class 2 + Class 15 + Class 14)
python fast_powerline_classifier.py -i "input.las" --stop-after wire
```

---

## 命令行参数完整速查表

| 参数项 | 缩写 | 默认值 | 作用与说明 |
| :--- | :---: | :---: | :--- |
| `--input` | `-i` | `None` | 输入 LAS/LAZ 点云文件路径（若未指定且未禁用 GUI，将弹出文件选择框） |
| `--output` | `-o` | `None` | 输出成果 LAS 文件路径（默认在输入同级目录下生成 `*_sign.las`） |
| `--quiet` | `-q` | `False` | 启用静默模式，关闭 1/4 ~ 4/4 阶段详细耗时与点数统计输出 |
| `--no-gui` | - | `False` | 禁用 Tkinter 图形化文件选择对话框（无头 Linux/服务器环境自动安全回退） |
| `--no-qtmodeler` | - | `False` | 处理完成后不自动启动 QTModeler 打开成果点云 |
| `--force-kill-viewer` | - | `False` | 导出成果前强制通过进程管理关闭占用文件的旧视口程序 |
| `--distribution` | - | `False` | 启用配电网模式（支持 10kV~110kV 矮塔与单双水泥杆） |
| `--stop-after` | - | `None` | 阶段短路截断，可选值：`ground`、`tower`、`wire` |
| `--split-spans` | - | `False` | 分类完成后，直接在内存中执行两塔一档 OBB 定向切分并输出至 `spans/` |
| `--split-only` | - | `False` | 仅执行纯切片预处理，锁定铁塔后短路退出，输出原始单档至 `spans_raw/` |
| `--auto-span-pipeline` | - | `False` | 一键端到端流水线：全线快速切档 -> 各档独立精细分类 |
| `--batch-dir` | - | `None` | 批量模式：遍历处理指定目录下的所有点云文件 |
| `--corridor-width` | - | `30.0` | 走廊单侧半宽（米），默认 30m（即全宽 60m 缓冲区） |
| `--span-buffer` | - | `12.0` | 档段两端沿主干走向的外延缓冲长度（米），防止塔头点云截断 |

---

## Python API SDK 二次开发调用

系统所有核心能力均封装在深模块 `modules` 中，可作为纯 Python SDK 直接集成至已有服务或分布式调度管道中：

```python
import numpy as np
import laspy
from modules import (
    PipelineExecutor,
    PipelineConfig,
    PipelineStage,
    CorridorCutter
)

# 1. 初始化配置与核心调度器
cfg = PipelineConfig()
cfg.pipeline.verbose = True
cfg.corridor.split_spans = True  # 开启内存自动分档
executor = PipelineExecutor(config=cfg)

# 2. 方式 A: 文件级执行并导出标准 LAS
result = executor.run_file("input.las", "output_sign.las")
print(f"提取杆塔: {len(result.towers)} 座, 导线簇: {len(result.wires)} 组")
if result.spans:
    print(f"自动切分单档文件: {len(result.spans)} 份")

# 3. 方式 B: 纯内存数据阵列接缝 (零文件依赖，支持流式或 Web 服务)
las = laspy.read("input.las")
points = np.column_stack([las.x, las.y, las.z])

# 仅执行至杆塔识别阶段 (毫秒级返回)
tower_result = executor.run(points, stop_after=PipelineStage.TOWER)
for tower in tower_result.towers:
    print(f"铁塔坐标: ({tower.cx:.2f}, {tower.cy:.2f}), 标高: {tower.max_z:.2f}m")

# 独立调用空间切档服务
spans = CorridorCutter.cut_spans(points, tower_result.towers, corridor_half_width=30.0)
print(f"生成双塔走廊 OBB 包围盒切片: {len(spans)} 跨")
```

---

## 自动化测试与物理验证 (Verification)

本项目遵循**硬性物理验证铁律**，代码提交前必须执行真实编译器/解释器全量套件验证并输出 `Exit Code: 0`。

### 1. 全量单元测试套件 (58 项全绿)

覆盖领域模型解包、流水线执行调度、动态短路、并查集封装、协议视口解耦、懒加载 GUI、OBB 走廊切片、航向角平滑惩罚等全量用例：

```bash
python -m unittest discover -s tests -p "test_*.py"
```

```text
..........................................................
----------------------------------------------------------------------
Ran 58 tests in 0.722s

OK (Exit Code: 0)
```

### 2. 真实机巡激光雷达端到端冒烟测试

采用实际工程采集的 500kV 线路真实机巡点云文件 `17-18(17_18).las` 进行全流程实战检验：

```bash
python tests/test_pipeline_m3_smoke.py
```

```text
[TEST 1] 合成点云端到端流水线与接口契约冒烟测试: 41,200 点 | 耗时 0.21s | [PASS]
[TEST 2] 真实机巡全流程运行测试: 1,375,048 点 | 耗时 11.24s | 导线点 79,393 | 杆塔点 76,491 | [PASS]
>>> 全部自动化验证通过 (Exit Code: 0) <<<
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

## 架构决策记录 (ADRs)

关于系统深度模块重构的详细论证、接口权衡与架构决策，参见以下 ADR 文档：
- [ADR-0001: 建立统领全流程执行的 PipelineExecutor 深度模块与阶段控制机制](file:///e:/points/docs/adr/0001-deep-pipeline-executor-with-stage-control.md)
- [ADR-0002: 整合多通道导线提取至高内聚 WireExtractor 并封装拓扑并查集](file:///e:/points/docs/adr/0002-unified-wire-extractor-and-encapsulated-topology.md)
- [ADR-0003: 基于 ViewerHandler 协议解耦视口控制与操作系统交互](file:///e:/points/docs/adr/0003-protocol-driven-viewer-and-gui-decoupling.md)
- [ADR-0004: 走廊切片依赖倒置与内存级流水线集成](file:///e:/points/docs/adr/0004-corridor-slicing-dependency-inversion.md)
- 领域通用术语字典参见：[`CONTEXT.md`](file:///e:/points/CONTEXT.md)
