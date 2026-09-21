import os
import sys
from typing import Optional, List, Tuple

def select_file_gui(
    title: str = "选择要分类处理的点云文件 (LAS/LAZ)",
    filetypes: Optional[List[Tuple[str, str]]] = None,
    allow_cli_fallback: bool = True
) -> Optional[str]:
    """
    打开图形界面文件选择对话框。
    采用惰性运行时动态导入，无桌面服务或缺少 tkinter 环境时优雅降级为终端提示或返回 None。
    """
    if filetypes is None:
        filetypes = [("点云文件", "*.las *.laz"), ("所有文件", "*.*")]

    try:
        import tkinter as tk
        from tkinter import filedialog

        root = tk.Tk()
        try:
            root.withdraw()
            root.attributes('-topmost', True)
            file_path = filedialog.askopenfilename(
                title=title,
                filetypes=filetypes
            )
        finally:
            root.destroy()

        if file_path:
            return os.path.abspath(file_path)
        return None
    except Exception as e:
        print(f"[Warning] 无法拉起图形文件选择对话框: {e}")
        if allow_cli_fallback:
            try:
                cli_input = input("请输入点云文件完整路径 (直接回车取消): ").strip()
                cleaned_path = cli_input.strip('"\'')
                if cleaned_path and os.path.exists(cleaned_path):
                    return os.path.abspath(cleaned_path)
                elif cleaned_path:
                    print(f"[Warning] 输入的文件路径不存在: '{cleaned_path}'")
            except (EOFError, KeyboardInterrupt):
                pass
        return None
