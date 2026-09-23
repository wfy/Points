import os
import sys
import glob
from typing import Optional, List, Tuple, Union

def select_files_gui(
    title: str = "选择要分类处理的点云文件 (支持多选 LAS/LAZ)",
    filetypes: Optional[List[Tuple[str, str]]] = None,
    allow_cli_fallback: bool = True
) -> List[str]:
    """
    打开图形界面文件选择对话框，支持同时选择多个文件。
    采用惰性运行时动态导入，无桌面服务或缺少 tkinter 环境时优雅降级为终端提示或返回空列表。
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
            selected = filedialog.askopenfilenames(
                title=title,
                filetypes=filetypes
            )
        finally:
            root.destroy()

        if selected:
            cleaned_paths = []
            for p in selected:
                p_str = str(p).strip()
                if p_str:
                    cleaned_paths.append(os.path.abspath(os.path.normpath(p_str)))
            return cleaned_paths
        return []
    except Exception as e:
        print(f"[Warning] 无法拉起图形文件选择对话框: {e}")
        if allow_cli_fallback:
            try:
                cli_input = input("请输入点云文件路径 (支持逗号分隔多个文件或通配符，直接回车取消): ").strip()
                if not cli_input:
                    return []
                paths = []
                raw_items = [item.strip().strip('"\'') for item in cli_input.replace(';', ',').split(',') if item.strip()]
                for item in raw_items:
                    if '*' in item or '?' in item:
                        matched = glob.glob(item)
                        paths.extend([os.path.abspath(os.path.normpath(m)) for m in matched if os.path.exists(m)])
                    elif os.path.exists(item):
                        paths.append(os.path.abspath(os.path.normpath(item)))
                    else:
                        print(f"[Warning] 输入的文件路径不存在: '{item}'")
                return paths
            except (EOFError, KeyboardInterrupt):
                pass
        return []

def select_file_gui(
    title: str = "选择要分类处理的点云文件 (LAS/LAZ)",
    filetypes: Optional[List[Tuple[str, str]]] = None,
    allow_cli_fallback: bool = True,
    multiple: bool = False
) -> Union[Optional[str], List[str]]:
    """
    打开图形界面文件选择对话框。
    若 multiple=True 则返回 List[str]；默认 multiple=False 保持向下兼容返回 Optional[str]。
    """
    files = select_files_gui(title=title, filetypes=filetypes, allow_cli_fallback=allow_cli_fallback)
    if multiple:
        return files
    return files[0] if len(files) > 0 else None

