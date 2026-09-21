# 0001. Deep PipelineExecutor with Stage-level Execution Control

## Status
Accepted

## Context
点云电力线分类系统此前直接在 `fast_powerline_classifier.py` 内部以过程式脚本形式串联 4 个阶段，向外暴露了 10 余个松散的 NumPy 坐标与布尔掩膜数组，形成了脆弱的浅层调用面。为了在单点样本（如 `68-69.las`）上快速调优 M2 阶段杆塔定位，开发过程中直接在源码中用 `#` 注释掉 M3 导线与 M4 拓扑阶段，导致自动化测试套件（如 `test_pipeline_m3_smoke.py`）断裂失败。

## Decision
我们将流水线阶段调度与中间状态全面收拢进单一深度模块 `PipelineExecutor`：
1. **核心接缝与测试面**：以内存级 `run(points) -> PipelineResult` 作为核心纯函数式测试面（彻底解耦磁盘 I/O），并提供 `run_file(input_path, output_path)` 便捷门面。
2. **阶段中断机制 (Stage-level Execution Control)**：引入 `PipelineStage` 枚举与 `stop_after` / `stages` 配置，支持在不修改任何生产代码的前提下优雅断点执行到指定阶段。
3. **领域结果封装**：统一输出 `PipelineResult` 对象，严格封装各类别点云索引、领域实体（`TowerEntity`, `WireCluster`）与耗时诊断，对外隐藏内部体素与掩膜细节。
4. **剔除浅层透传**：废弃纯透传模块 `powerline_extractor.py`，由执行器直接对接导线提取核心。

## Consequences
- 杜绝为了局部调优而临时修改或注释流水线源码的非规范行为。
- 单档测试与冒烟测试无需向磁盘写入临时 LAS 文件，测试执行速度大幅提升。
- 为后续 Candidate 02（统一导线提取）与 Candidate 04（走廊切片依赖反转）奠定稳定的上游契约。
