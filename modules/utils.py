import os
import time
import subprocess
import colorsys
import numpy as np
import laspy
import tkinter as tk
from tkinter import filedialog
from modules.config import ClassificationCode

def close_qtmodeler():
    """强行关闭后台运行的 QTModeler.exe 进程以释放文件独占占用"""
    try:
        cmd = 'taskkill /F /IM QTModeler.exe /T'
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.3)
    except Exception:
        pass

def open_in_qtmodeler(las_path: str):
    """
    自动将分类好的 LAS 点云导入 QTModeler.exe 打开展示
    """
    abs_las = os.path.abspath(las_path)

    qt_path = None
    possible_paths = [
        r"C:\Program Files\QTModeler_820_UX_TRIAL\QTModeler.exe",
        r"C:\Program Files\Applied Imagery\QT Modeler\QTModeler.exe",
        r"C:\QTModeler_840_UX\QTModeler.exe",
        r"D:\Program Files\QTModeler_820_UX_TRIAL\QTModeler.exe",
        r"D:\Program Files\Applied Imagery\QT Modeler\QTModeler.exe",
        r"D:\QTModeler_840_UX\QTModeler.exe",
        r"E:\Program Files\Applied Imagery\QT Modeler\QTModeler.exe",
        r"E:\QTModeler_840_UX\QTModeler.exe"
    ]
    for p in possible_paths:
        if os.path.exists(p):
            qt_path = p
            break

    if qt_path and os.path.exists(qt_path):
        try:
            subprocess.Popen([qt_path, abs_las])
            print(f"[Done] 成功启动 QTModeler 并加载 '{os.path.basename(abs_las)}'！")
        except Exception as e:
            print(f"推送到 QTModeler 失败: {e}")
    else:
        try:
            os.startfile(abs_las)
            print("[Done] 已通过 Windows 默认查看器打开成果文件。")
        except Exception as e:
            print(f"无法自动打开文件: {e}")

def select_file_gui():
    """打开文件选择 GUI 对话框"""
    root = tk.Tk()
    root.withdraw()
    root.attributes('-topmost', True)
    file_path = filedialog.askopenfilename(
        title="选择要分类处理的点云文件 (LAS/LAZ)",
        filetypes=[("点云文件", "*.las *.laz"), ("所有文件", "*.*")]
    )
    root.destroy()
    return file_path

def get_safe_output_path(target_path: str) -> str:
    """
    检查目标文件是否可写；若遇文件被外部程序锁定，自动生成增量后缀文件名
    """
    if not os.path.exists(target_path):
        return target_path
        
    try:
        with open(target_path, 'r+b'):
            pass
        return target_path
    except (PermissionError, OSError):
        dir_name, file_name = os.path.split(target_path)
        base_name, ext = os.path.splitext(file_name)
        
        counter = 1
        while True:
            candidate_path = os.path.join(dir_name, f"{base_name}_{counter}{ext}")
            if not os.path.exists(candidate_path):
                print(f"[Warning] 原目标文件 '{target_path}' 正被外部程序独占锁定，已自动安全重命名为 '{candidate_path}'")
                return candidate_path
            try:
                with open(candidate_path, 'r+b'):
                    print(f"[Warning] 原目标文件 '{target_path}' 被锁定，使用备选文件 '{candidate_path}'")
                    return candidate_path
            except (PermissionError, OSError):
                counter += 1

def export_colored_las(las_input_path: str,
                       las_output_path: str,
                       las_raw_data,
                       ground_idx: np.ndarray,
                       cable_pts_idx: np.ndarray,
                       tower_pts_idx: np.ndarray,
                       tower_arm_pts_idx: np.ndarray,
                       all_confirmed: list,
                       point_line_id: np.ndarray,
                       suspect_line_ids: set,
                       find_line_func,
                       insulator_pts_idx: np.ndarray = None,
                       force_kill_viewer: bool = False,
                       tower_below_arm_pts_idx: np.ndarray = None) -> str:
    """
    组装色彩与分类属性，写回带 RGB 的成果 LAS 文件 (Point Format 3)
    支持 Class 2(地面), Class 3(植被), Class 14(导线/跳线), Class 15(杆塔), Class 16(耐张绝缘子串)
    """
    if force_kill_viewer:
        close_qtmodeler()
        
    print("-> 正在写入色彩与分类标记至 LAS 文件...")
    
    las = las_raw_data
    num_points = len(las.x)
    
    # 严格去重：绝缘子优先，从杆塔与导线中排除
    if insulator_pts_idx is not None and len(insulator_pts_idx) > 0:
        tower_pts_idx = np.setdiff1d(tower_pts_idx, insulator_pts_idx)
        if len(tower_arm_pts_idx) > 0:
            tower_arm_pts_idx = np.setdiff1d(tower_arm_pts_idx, insulator_pts_idx)
        cable_pts_idx = np.setdiff1d(cable_pts_idx, insulator_pts_idx)
        
    if len(cable_pts_idx) > 0 and len(tower_pts_idx) > 0:
        cable_pts_idx = np.setdiff1d(cable_pts_idx, tower_pts_idx)
        
    classifications = np.full(num_points, int(ClassificationCode.LOW_VEGETATION), dtype=np.uint8)  # 默认 3 = 植被
    classifications[ground_idx] = int(ClassificationCode.GROUND)                                     # 2 = 地面
    if len(cable_pts_idx) > 0:
        classifications[cable_pts_idx] = int(ClassificationCode.WIRE_CONDUCTOR)                     # 14 = 导线/跳线
    if len(tower_pts_idx) > 0:
        classifications[tower_pts_idx] = int(ClassificationCode.TRANSMISSION_TOWER)                 # 15 = 杆塔
    if insulator_pts_idx is not None and len(insulator_pts_idx) > 0:
        classifications[insulator_pts_idx] = int(ClassificationCode.INSULATOR)                      # 16 = 耐张绝缘子串

    # 1. 植被与全量背景点直接继承原点云真彩色 (自动处理 8位/16位 色阶映射，杜绝色阶压暗或误染色)
    has_orig_rgb = hasattr(las, 'red') and hasattr(las, 'green') and hasattr(las, 'blue') and np.any(las.red > 0)
    if has_orig_rgb:
        r_raw = np.array(las.red, dtype=np.uint32)
        g_raw = np.array(las.green, dtype=np.uint32)
        b_raw = np.array(las.blue, dtype=np.uint32)
        if r_raw.max() <= 255 and g_raw.max() <= 255 and b_raw.max() <= 255:
            # 8位真彩色无损线性映射至16位规范色阶 (255 -> 65535)，确保植被原色在三维查看器中明亮清晰
            red = (r_raw * 257).astype(np.uint16)
            green = (g_raw * 257).astype(np.uint16)
            blue = (b_raw * 257).astype(np.uint16)
        else:
            red = r_raw.astype(np.uint16)
            green = g_raw.astype(np.uint16)
            blue = b_raw.astype(np.uint16)
    elif hasattr(las, 'intensity') and np.any(las.intensity > 0):
        # 无 RGB 色彩的点云基于激光反射强度渲染自然灰度
        i_min = float(np.percentile(las.intensity, 1))
        i_max = float(np.percentile(las.intensity, 99))
        i_norm = np.clip((las.intensity.astype(np.float32) - i_min) / max(i_max - i_min, 1.0), 0.0, 1.0)
        gray = (i_norm * 45000 + 10000).astype(np.uint16)
        red = gray.copy()
        green = gray.copy()
        blue = gray.copy()
    else:
        gray_val = 32768
        red = np.full(num_points, gray_val, dtype=np.uint16)
        green = np.full(num_points, gray_val, dtype=np.uint16)
        blue = np.full(num_points, gray_val, dtype=np.uint16)
    
    # 1.5 地面(灰色)与植被(绿色)分类着色：覆盖背景默认色，形成清晰分类可视化
    GROUND_GRAY = int(0.49 * 65535)   # 约 32112 灰
    VEG_GREEN = int(0.69 * 65535)     # 约 45219 绿
    red[ground_idx] = GROUND_GRAY
    green[ground_idx] = GROUND_GRAY
    blue[ground_idx] = GROUND_GRAY
    veg_idx = np.where(classifications == int(ClassificationCode.LOW_VEGETATION))[0]
    red[veg_idx] = 0
    green[veg_idx] = VEG_GREEN
    blue[veg_idx] = 0

    # 2. 导线与跳线染色 (严格剔除杆塔蓝/黄、植被绿、地面灰等色系，采用高反差独立分相色调)
    WIRE_PRESET_COLORS = [
        (65535, 15000, 0),      # 01. 烈焰橙红 (Flame Orange Red)
        (65535, 0, 48000),      # 02. 荧光洋红 (Fluorescent Magenta)
        (52000, 0, 65535),      # 03. 亮紫罗兰 (Bright Violet)
        (65535, 30000, 0),      # 04. 鲜亮琥珀橙 (Vivid Amber Orange)
        (65535, 0, 65535),      # 05. 璀璨品红 (Brilliant Magenta)
        (42000, 0, 65535),      # 06. 皇家紫 (Royal Purple)
        (65535, 0, 25000),      # 07. 深玫瑰红 (Deep Rose Red)
        (65535, 22000, 8000),   # 08. 珊瑚赤橙 (Coral Orange)
        (58000, 10000, 58000),  # 09. 兰花紫 (Orchid Purple)
        (65535, 8000, 30000),   # 10. 霓虹粉红 (Neon Hot Pink)
        (48000, 0, 55000),      # 11. 暮光紫 (Twilight Purple)
        (65535, 35000, 0),      # 12. 暖炽橙 (Warm Blaze Orange)
        (65535, 0, 38000),      # 13. 宝石红 (Ruby Pink)
        (36000, 0, 65535),      # 14. 丁香深紫 (Lilac Deep Purple)
        (65535, 18000, 18000),  # 15. 鲜桃红 (Peach Rose)
        (54000, 0, 42000),      # 16. 紫红 (Red Violet)
    ]
    
    if len(cable_pts_idx) > 0 and len(all_confirmed) > 0:
        num_lines = len(all_confirmed)
        line_color_map = {}
        for line_i in range(1, num_lines + 1):
            if line_i in suspect_line_ids:
                line_color_map[line_i] = (65535, 0, 0)  # 警示纯红
            elif line_i <= len(WIRE_PRESET_COLORS):
                line_color_map[line_i] = WIRE_PRESET_COLORS[line_i - 1]
            else:
                # 算法动态生成：严格避开杆塔黄色/蓝色区间、植被绿色区间以及地面灰度
                # 仅在 [0.0, 0.09] (红橙) 与 [0.70, 0.95] (紫/洋红/粉红) 之间生成
                t = (line_i * 0.618033988749895) % 1.0
                if t < 0.25:
                    h = (t / 0.25) * 0.09               # 红色 -> 橙色
                else:
                    h = 0.70 + ((t - 0.25) / 0.75) * 0.25  # 紫 -> 洋红 -> 粉红 -> 玫瑰红
                r, g, b = colorsys.hsv_to_rgb(h, 0.95, 1.0)
                line_color_map[line_i] = (int(r * 65535), int(g * 65535), int(b * 65535))
            
        for pt_idx in cable_pts_idx:
            raw_id = point_line_id[pt_idx]
            l_id = find_line_func(raw_id) if raw_id > 0 else raw_id
            if l_id in line_color_map:
                cr, cg, cb = line_color_map[l_id]
                red[pt_idx] = cr
                green[pt_idx] = cg
                blue[pt_idx] = cb
            else:
                red[pt_idx] = 65535
                green[pt_idx] = 20000
                blue[pt_idx] = 0
    elif len(cable_pts_idx) > 0:
        red[cable_pts_idx] = 65535
        green[cable_pts_idx] = 20000
        blue[cable_pts_idx] = 0
        
    # 3. 铁塔与横担染色 (纯正工业蓝)
    if len(tower_pts_idx) > 0:
        red[tower_pts_idx] = 0
        green[tower_pts_idx] = 0
        blue[tower_pts_idx] = 65535

    if len(tower_arm_pts_idx) > 0:
        red[tower_arm_pts_idx] = 0
        green[tower_arm_pts_idx] = 0
        blue[tower_arm_pts_idx] = 65535

    # 3b. 最下方横担以下区域标记为高亮黄色 (仅对已判定铁塔点重着色，不改分类)
    if tower_below_arm_pts_idx is not None and len(tower_below_arm_pts_idx) > 0:
        red[tower_below_arm_pts_idx] = 65535
        green[tower_below_arm_pts_idx] = 65535
        blue[tower_below_arm_pts_idx] = 0

    # 4. 耐张绝缘子串挂点染色 (高反差纯白 65535, 65535, 65535)
    if insulator_pts_idx is not None and len(insulator_pts_idx) > 0:
        red[insulator_pts_idx] = 65535
        green[insulator_pts_idx] = 65535
        blue[insulator_pts_idx] = 65535

    final_output_path = get_safe_output_path(las_output_path)
    if os.path.exists(final_output_path):
        try:
            os.remove(final_output_path)
        except OSError:
            pass

    if hasattr(las, 'red') and hasattr(las, 'green') and hasattr(las, 'blue'):
        las.classification = classifications
        las.red = red
        las.green = green
        las.blue = blue
        las.write(final_output_path)
    else:
        new_header = laspy.LasHeader(point_format=3, version="1.2")
        new_header.scales = las.header.scales
        new_header.offsets = las.header.offsets
        new_las = laspy.LasData(new_header)
        new_las.points = laspy.ScaleAwarePointRecord.zeros(len(las.x), header=new_header)
        new_las.x = las.x
        new_las.y = las.y
        new_las.z = las.z
        new_las.classification = classifications
        new_las.red = red
        new_las.green = green
        new_las.blue = blue
        new_las.write(final_output_path)

    print(f"   输出成果: '{final_output_path}'")
    return final_output_path
