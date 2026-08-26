import os
import time
import json
import numpy as np
import laspy
from fast_powerline_classifier import fast_classify_and_color_powerline
from modules.config import PipelineConfig
from modules.render_preview import render_preview_image

FILES = [
    r"E:\unity\点云\0-1(0_1).las",
    r"E:\unity\点云\17-18(17_18).las",
    r"E:\unity\点云\18-19(18_19).las",
    r"E:\unity\点云\7-8(7_8).las",
    r"E:\unity\点云\5-6(5_6).las",
    r"E:\unity\点云\125-126(125_126).las",
    r"E:\unity\点云\68-69(68_69).las",
]

ARTIFACT_DIR = r"C:\Users\jayden\.gemini\antigravity\brain\48e5a1c5-4689-40cc-aceb-b9905cdc6d75"

def test_file(file_path):
    base_name = os.path.splitext(os.path.basename(file_path))[0]
    out_las = os.path.join(os.path.dirname(file_path), f"{base_name}_sign.las")
    out_png = os.path.join(ARTIFACT_DIR, f"preview_{base_name}.png")
    
    print(f"\n=======================================================")
    print(f"[TESTING] File: {file_path}")
    
    cfg = PipelineConfig()
    cfg.export.force_kill_viewer = False
    
    t0 = time.time()
    res_path = fast_classify_and_color_powerline(file_path, out_las, config=cfg)
    cost = time.time() - t0
    
    # 读取输出 LAS 检查杆塔信息
    las = laspy.read(res_path)
    cls = las.classification
    tower_pts_count = int(np.sum(cls == 15))
    ground_pts_count = int(np.sum(cls == 2))
    total_pts = len(las.x)
    
    # 提取实际独立的真实杆塔簇用于特写渲染和准确统计
    tower_infos = []
    if tower_pts_count > 0:
        t_pts = np.column_stack([np.array(las.x)[cls == 15], np.array(las.y)[cls == 15], np.array(las.z)[cls == 15]])
        
        # 体素化降采样至 2m 进行内存安全的连通分量分析
        from scipy.spatial import cKDTree
        vox_2d = (t_pts[:, :2] / 2.0).astype(np.int32)
        u_vox, inv = np.unique(vox_2d, axis=0, return_inverse=True)
        
        v_tree = cKDTree(u_vox * 2.0)
        pairs = v_tree.query_pairs(r=20.0)
        parent = list(range(len(u_vox)))
        def find(i):
            if parent[i] == i: return i
            parent[i] = find(parent[i])
            return parent[i]
        for i, j in pairs:
            ri, rj = find(i), find(j)
            if ri != rj: parent[ri] = rj
            
        clusters = {}
        for i in range(len(u_vox)):
            r = find(i)
            clusters.setdefault(r, []).append(i)
            
        for r, v_members in clusters.items():
            pt_mask = np.isin(inv, v_members)
            if np.sum(pt_mask) >= 500:
                sub_t = t_pts[pt_mask]
                cx, cy = float(np.mean(sub_t[:, 0])), float(np.mean(sub_t[:, 1]))
                cov = np.cov(sub_t[:, :2].T)
                ev, evec = np.linalg.eigh(cov)
                v1 = evec[:, 1]
                tower_infos.append({
                    'cx': cx,
                    'cy': cy,
                    'max_z': float(np.max(sub_t[:, 2])),
                    'pts_count': int(np.sum(pt_mask)),
                    'v1': v1
                })
                
        del las, t_pts, vox_2d, u_vox, inv, v_tree, clusters
        import gc
        gc.collect()
        
    render_preview_image(res_path, out_png, tower_infos=tower_infos)
    print(f"[PREVIEW] Saved: {out_png}")
    
    # 同时保存一份不含括号的干净文件名，防止 Markdown URL 解析截断
    clean_name = base_name.replace('(', '_').replace(')', '').replace('-', '_')
    out_png_clean = os.path.join(ARTIFACT_DIR, f"preview_{clean_name}.png")
    import shutil
    shutil.copyfile(out_png, out_png_clean)
    
    info = {
        'file_name': os.path.basename(file_path),
        'total_pts': total_pts,
        'ground_pts': ground_pts_count,
        'tower_pts': tower_pts_count,
        'tower_count': len(tower_infos),
        'tower_centers': [(round(t['cx'], 2), round(t['cy'], 2), round(t['max_z'], 2), t['pts_count']) for t in tower_infos],
        'cost_sec': round(cost, 2),
        'preview_png': out_png
    }
    print(f"[RESULT] {json.dumps(info, ensure_ascii=False, indent=2)}")
    return info

if __name__ == '__main__':
    import sys
    import subprocess
    
    if len(sys.argv) > 2 and sys.argv[1] == '--single':
        test_file(sys.argv[2])
    else:
        results = []
        py_exe = sys.executable
        for f in FILES:
            if os.path.exists(f):
                p = subprocess.run([py_exe, __file__, '--single', f], capture_output=False)
            else:
                print(f"[ERROR] File not found: {f}")

