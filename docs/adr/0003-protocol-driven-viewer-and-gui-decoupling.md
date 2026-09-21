# 0003. Protocol-Driven Viewer and GUI Decoupling

## Status
Accepted

## Context
当前点云分类系统的三维可视化查看器（QTModeler）与图形用户界面（GUI）交互逻辑分散且与核心计算/导出逻辑高度耦合：
1. **进程管理污染数据落盘 (I/O & Process Coupling)**：
   `modules/utils.py` 中的 `export_colored_las` 负责核心 LAS 点云色彩与分类属性写回，但其入参包含 `force_kill_viewer: bool = False`，并在内部直接调用 `close_qtmodeler()`（执行 Windows 专有的 `taskkill /F /IM QTModeler.exe /T`）。这使得底层数据落盘操作强耦合了特定操作系统的进程杀灭逻辑。
2. **缺乏无头与测试环境隔离 (No Headless / Test Isolation)**：
   `open_in_qtmodeler` 在模块内硬编码了 8 个 Windows 绝对路径，并在无此软件时回退到 `os.startfile`。在单元测试、CI 流水线或 Linux/Docker 无头运行场景中，调用方必须人工侵入配置 `cfg.export.open_qtmodeler = False` 才能避免拉起外部进程。
3. **GUI 依赖污染核心模块 (GUI Pollution in Core Utilities)**：
   `modules/utils.py` 在模块顶层静态导入了 `tkinter` 与 `filedialog`。在无桌面环境的服务端或容器中，导入 `utils.py` 可能直接抛出异常或挂起。
4. **路径探测逻辑多处重复 (Duplicated Path Resolution)**：
   `modules/qtmodeler_protocol_handler.py` 内部硬编码了与 `utils.py` 完全相同的 8 个文件探测路径，违反了 DRY 原则。

## Decision
我们决定采用协议模式与空对象模式（Null Object Pattern），将查看器与 GUI 交互彻底从核心算法与导出工具中解耦：
1. **查看器协议与实现策略 (`modules/viewer.py`)**：
   - 定义统一抽象协议 `ViewerHandler`，声明 `open(file_path: str) -> bool`、`close() -> bool` 以及 `is_available() -> bool` 契约。
   - 提供 `QTModelerViewer`：集中收拢可执行程序探测、进程启动与进程安全关闭。
   - 提供 `SystemDefaultViewer`：使用操作系统默认关联程序打开。
   - 提供 `NullViewer`：静默无操作的空对象，专门用于自动化测试、批量批处理与无头模式。
   - 提供便捷工厂 `get_viewer(config_or_name) -> ViewerHandler`。
2. **文件导出职责纯粹化 (`export_colored_las`)**：
   从 `export_colored_las` 中剥离进程管理副作用。`force_kill_viewer` 标记为 `[DEPRECATED]` 保留参数以维持接口向下兼容（内部不再直接发起 `taskkill`）。进程独占释放的生命周期提升至 `PipelineExecutor` 落盘前或 CLI 调度层。
3. **GUI 交互抽离与环境自适应 (`modules/gui.py`)**：
   将 `select_file_gui()` 迁移至独立模块 `modules/gui.py`。采用动态惰性导入 `tkinter`；若在无桌面环境或依赖缺失时被触发，优雅降级捕获异常并提示控制台输入。
4. **URL 协议处理器轻量化 (`modules/qtmodeler_protocol_handler.py`)**：
   重构该脚本，直接调用 `modules.viewer` 中的探测与拉起逻辑，消除重复代码。

## Consequences
- `modules/utils.py` 彻底回归为纯粹的算法数学与 LAS I/O 工具库，不再包含 `tkinter`、`subprocess`、`taskkill`。
- 测试套件与无头服务可直接注入 `NullViewer`，彻底告别外部弹窗与进程占用干扰。
- 架构具备良好扩展性，未来支持接入 CloudCompare、Web3D 等新型点云查看器仅需扩充协议实现类。
- 保留 `open_in_qtmodeler`、`close_qtmodeler` 及 `select_file_gui` 别名中转，旧代码与 CLI 调用完全兼容。
