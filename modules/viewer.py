import os
import sys
import time
import subprocess
from typing import Protocol, Optional, runtime_checkable
from modules.config import ExportConfig

@runtime_checkable
class ViewerHandler(Protocol):
    """3D 点云成果查看器统一协议接口"""
    def open(self, file_path: str) -> bool:
        """打开并展示指定点云文件"""
        ...

    def close(self) -> bool:
        """关闭运行中的查看器进程以释放文件锁"""
        ...

    def is_available(self) -> bool:
        """检测当前运行环境是否支持该查看器"""
        ...


class QTModelerFinder:
    """QTModeler 可执行程序集中探测定位器"""
    DEFAULT_PATHS = [
        r"C:\Program Files\QTModeler_820_UX_TRIAL\QTModeler.exe",
        r"C:\Program Files\Applied Imagery\QT Modeler\QTModeler.exe",
        r"C:\QTModeler_840_UX\QTModeler.exe",
        r"D:\Program Files\QTModeler_820_UX_TRIAL\QTModeler.exe",
        r"D:\Program Files\Applied Imagery\QT Modeler\QTModeler.exe",
        r"D:\QTModeler_840_UX\QTModeler.exe",
        r"E:\Program Files\Applied Imagery\QT Modeler\QTModeler.exe",
        r"E:\QTModeler_840_UX\QTModeler.exe",
    ]

    @classmethod
    def find(cls) -> Optional[str]:
        env_path = os.environ.get("QTMODELER_PATH")
        if env_path and os.path.exists(env_path):
            return env_path
        for p in cls.DEFAULT_PATHS:
            if os.path.exists(p):
                return p
        return None


def find_qtmodeler() -> Optional[str]:
    """快捷查找 QTModeler.exe 安装路径"""
    return QTModelerFinder.find()


class NullViewer:
    """空对象模式查看器：专为测试、批量任务与无头环境设计的无害实现"""
    def open(self, file_path: str) -> bool:
        return True

    def close(self) -> bool:
        return True

    def is_available(self) -> bool:
        return False


class SystemDefaultViewer:
    """操作系统默认关联程序查看器"""
    def open(self, file_path: str) -> bool:
        abs_path = os.path.abspath(file_path)
        try:
            if sys.platform == "win32":
                os.startfile(abs_path)
            elif sys.platform == "darwin":
                subprocess.Popen(["open", abs_path])
            else:
                subprocess.Popen(["xdg-open", abs_path])
            print(f"[Done] 已通过系统默认查看器打开成果文件: '{os.path.basename(abs_path)}'")
            return True
        except Exception as e:
            print(f"[Warning] 无法通过系统默认查看器打开文件: {e}")
            return False

    def close(self) -> bool:
        return True

    def is_available(self) -> bool:
        return True


class QTModelerViewer:
    """Applied Imagery QTModeler 专业点云可视化实现"""
    def __init__(self, executable_path: Optional[str] = None):
        self._executable_path = executable_path

    @property
    def executable_path(self) -> Optional[str]:
        return self._executable_path or find_qtmodeler()

    def is_available(self) -> bool:
        return bool(find_qtmodeler())

    def open(self, file_path: str) -> bool:
        abs_path = os.path.abspath(file_path)
        qt_path = self.executable_path
        if qt_path and os.path.exists(qt_path):
            try:
                subprocess.Popen([qt_path, abs_path])
                print(f"[Done] 成功启动 QTModeler 并加载 '{os.path.basename(abs_path)}'！")
                return True
            except Exception as e:
                print(f"[Warning] 推送到 QTModeler 失败: {e}")
                return SystemDefaultViewer().open(abs_path)
        else:
            return SystemDefaultViewer().open(abs_path)

    def close(self) -> bool:
        """关闭运行中的 QTModeler 进程释放文件占用"""
        if sys.platform == "win32":
            try:
                cmd = 'taskkill /F /IM QTModeler.exe /T'
                subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                time.sleep(0.3)
                return True
            except Exception:
                return False
        return True


def get_viewer(config: Optional[ExportConfig] = None, name: Optional[str] = None) -> ViewerHandler:
    """
    根据运行配置或显式名称获取适用的 Viewer 实例
    """
    if config is not None and not config.open_qtmodeler:
        return NullViewer()
    if name == "null":
        return NullViewer()
    if name == "system":
        return SystemDefaultViewer()
    if name == "qtmodeler":
        return QTModelerViewer()

    # 默认策略：若检测到 QTModeler 则选用，否则回退至系统默认
    if find_qtmodeler():
        return QTModelerViewer()
    return SystemDefaultViewer()
