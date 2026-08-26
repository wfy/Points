# -*- coding: utf-8 -*-
import os
import sys
import glob
from modules.render_preview import show_all_previews_gui, show_preview_gui

ARTIFACT_DIR = r'C:\Users\jayden\.gemini\antigravity\brain\48e5a1c5-4689-40cc-aceb-b9905cdc6d75'
POINT_CLOUD_DIR = r'E:\unity\点云'

def main():
    items = []
    
    # 7 组典型标准点云
    standard_files = [
        '0-1(0_1)',
        '17-18(17_18)',
        '18-19(18_19)',
        '7-8(7_8)',
        '5-6(5_6)',
        '125-126(125_126)',
        '68-69(68_69)'
    ]
    
    for name in standard_files:
        las_sign = os.path.join(POINT_CLOUD_DIR, f'{name}_sign.las')
        clean_name = name.replace('(', '_').replace(')', '').replace('-', '_')
        png_path = os.path.join(ARTIFACT_DIR, f'preview_{clean_name}.png')
        if not os.path.exists(png_path):
            png_path = os.path.join(ARTIFACT_DIR, f'preview_{name}.png')
            
        if os.path.exists(las_sign):
            items.append({
                'name': f'{name}_sign.las',
                'las_path': las_sign,
                'png_path': png_path,
                'info': ''
            })
            
    if len(sys.argv) > 1:
        target = sys.argv[1]
        if os.path.exists(target):
            if target.endswith('.las') or target.endswith('.laz'):
                base = os.path.splitext(os.path.basename(target))[0]
                clean_b = base.replace('(', '_').replace(')', '').replace('-', '_')
                png = os.path.join(ARTIFACT_DIR, f'preview_{clean_b}.png')
                show_preview_gui(target, png)
                return
            elif target.endswith('.png'):
                # 寻找同名 las
                show_preview_gui(target, target)
                return

    if len(items) == 0:
        print('[Warning] 未在标准目录找到 _sign.las 成果文件，正在扫描所有分类点云...')
        all_sign = glob.glob(os.path.join(POINT_CLOUD_DIR, '*_sign.las'))
        for las in all_sign:
            base = os.path.splitext(os.path.basename(las))[0]
            clean_b = base.replace('(', '_').replace(')', '').replace('-', '_')
            png = os.path.join(ARTIFACT_DIR, f'preview_{clean_b}.png')
            items.append({
                'name': os.path.basename(las),
                'las_path': las,
                'png_path': png,
                'info': ''
            })

    if len(items) > 0:
        print(f'正在打开 3D 成果预览交互面板 (共 {len(items)} 份点云)...')
        show_all_previews_gui(items)
    else:
        print('[Error] 未找到任何点云成果文件！')

if __name__ == '__main__':
    main()
