import os
import sys
import numpy as np
import laspy
from scipy.spatial import cKDTree

def render_preview_image(las_path: str, output_img_path: str, tower_infos=None):
    """
    读取 LAS 点云并渲染高分辨率 3D 视角概览图与杆塔 3D 立体特写图 (保存为 PNG)
    - 左侧大面板：整档/全走廊 3D 俯视等轴测鸟瞰图 (Painter 深度排序)
    - 右侧子面板：两座独立主塔的 3D 等轴测超清立体特写 (含横担立柱 3D 深度透视)
    """
    las = laspy.read(las_path)
    raw_cls = getattr(las, 'classification', np.zeros(len(las.x), dtype=np.uint8))
    num_raw = len(las.x)
    
    # 智能采样：杆塔 (15) 与导地线 (13, 14) 100% 全量保留，地表植被轻量抽样 (上限 400k) 避免内存过载
    step = max(num_raw // 400000, 1)
    render_mask = (raw_cls == 15) | (raw_cls == 14) | (raw_cls == 13)
    if step > 1:
        render_mask[::step] = True
    else:
        render_mask[:] = True
        
    pts = np.column_stack([np.array(las.x)[render_mask], np.array(las.y)[render_mask], np.array(las.z)[render_mask]])
    cls = raw_cls[render_mask]
    num_pts = len(pts)
    
    # 提取真实点云颜色
    has_colors = hasattr(las, 'red') and hasattr(las, 'green') and hasattr(las, 'blue') and np.max(las.red) > 0
    if has_colors:
        r = (np.array(las.red)[render_mask] / 256).astype(np.uint8)
        g = (np.array(las.green)[render_mask] / 256).astype(np.uint8)
        b = (np.array(las.blue)[render_mask] / 256).astype(np.uint8)
        colors = np.column_stack([r, g, b])
    else:
        colors = np.full((num_pts, 3), 160, dtype=np.uint8)
        colors[cls == 2] = [70, 70, 75]                  # 地面：深灰
        colors[(cls >= 3) & (cls <= 5)] = [30, 110, 35]   # 植被：自然暗绿
        colors[cls == 14] = [0, 255, 128]                # 导线：明亮荧光绿
        colors[cls == 13] = [0, 200, 255]                # 地线：亮青
        colors[cls == 15] = [255, 215, 0]                # 杆塔：亮金黄

    H, W = 900, 1600
    canvas = np.full((H, W, 3), 20, dtype=np.uint8)  # 深色护眼背景
    
    # 提取独立主塔实体
    t_mask = cls == 15
    t_pts = pts[t_mask]
    
    if tower_infos is None or len(tower_infos) == 0:
        tower_infos = []
        if len(t_pts) >= 500:
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
                clusters.setdefault(find(i), []).append(i)
                
            for r_id, v_m in clusters.items():
                pt_m = np.isin(inv, v_m)
                if np.sum(pt_m) >= 500:
                    sub = t_pts[pt_m]
                    cx, cy = float(np.mean(sub[:, 0])), float(np.mean(sub[:, 1]))
                    cov = np.cov(sub[:, :2].T)
                    ev, evec = np.linalg.eigh(cov)
                    tower_infos.append({
                        'cx': cx, 'cy': cy, 'max_z': float(np.max(sub[:, 2])),
                        'min_z': float(np.min(sub[:, 2])), 'pts_count': int(np.sum(pt_m)),
                        'v1': evec[:, 1]
                    })
    
    # 按走廊坐标排序
    tower_infos.sort(key=lambda t: t['cx'] + t['cy'])

    # 1. 左侧面板：全走廊 3D 俯视等轴测图 (带深度排序 Painter 算法)
    W_left = 1000
    pts_xy = pts[:, :2] - np.mean(pts[:, :2], axis=0)
    cov_xy = np.cov(pts_xy.T)
    evals, evecs = np.linalg.eigh(cov_xy)
    main_dir = evecs[:, 1]
    angle_main = np.arctan2(main_dir[1], main_dir[0])
    
    theta = -angle_main + np.radians(25)
    phi = np.radians(28)
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    cos_p, sin_p = np.cos(phi), np.sin(phi)
    
    c_center = np.mean(pts, axis=0)
    p_centered = pts - c_center
    
    step = max(len(pts) // 300000, 1)
    pts_sub = pts[::step]
    cols_sub = colors[::step]
    cls_sub = cls[::step]
    p_c_sub = p_centered[::step]
    
    x_rot = p_c_sub[:, 0] * cos_t - p_c_sub[:, 1] * sin_t
    y_rot = p_c_sub[:, 0] * sin_t + p_c_sub[:, 1] * cos_t
    z_rot = p_c_sub[:, 2]
    
    u_proj = x_rot
    v_proj = y_rot * sin_p + z_rot * cos_p
    
    u_min, u_max = np.percentile(u_proj, 0.5), np.percentile(u_proj, 99.5)
    v_min, v_max = np.percentile(v_proj, 0.5), np.percentile(v_proj, 99.5)
    
    scale = min((W_left - 80) / max(u_max - u_min, 1e-3), (H - 120) / max(v_max - v_min, 1e-3))
    
    px = ((u_proj - (u_min + u_max) / 2) * scale + W_left / 2).astype(int)
    py = (H - ((v_proj - (v_min + v_max) / 2) * scale + H / 2)).astype(int)
    
    sort_depth = np.argsort(y_rot)
    px_s, py_s, cols_s, cls_s = px[sort_depth], py[sort_depth], cols_sub[sort_depth], cls_sub[sort_depth]
    
    valid = (px_s >= 2) & (px_s < W_left - 2) & (py_s >= 2) & (py_s < H - 2)
    px_v, py_v, cols_v, cls_v = px_s[valid], py_s[valid], cols_s[valid], cls_s[valid]
    
    non_t = cls_v != 15
    canvas[py_v[non_t], px_v[non_t]] = cols_v[non_t]
    
    is_t = cls_v == 15
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            canvas[np.clip(py_v[is_t] + dy, 0, H-1), np.clip(px_v[is_t] + dx, 0, W_left-1)] = cols_v[is_t]
            
    # 2. 右侧面板：每座独立主塔的 3D 等轴测高清立体特写 (W_right = 580)
    box_w = 560
    display_towers = tower_infos[:2]
    n_display = len(display_towers)
    
    if n_display > 0:
        sub_h = (H - 30) // n_display
        
        for t_i, t_info in enumerate(display_towers):
            tc_x, tc_y = float(t_info.get('cx', 0)), float(t_info.get('cy', 0))
            
            dx_box = pts_sub[:, 0] - tc_x
            dy_box = pts_sub[:, 1] - tc_y
            m_box = (dx_box >= -26.0) & (dx_box <= 26.0) & (dy_box >= -26.0) & (dy_box <= 26.0)
            
            if not np.any(m_box):
                continue
                
            t_sub_pts = pts_sub[m_box]
            t_sub_cols = cols_sub[m_box]
            t_sub_cls = cls_sub[m_box]
            
            # 以横担方向 v1 为基准旋转 35 度，倾斜 18 度获得 3D 铁塔立体透视
            v1 = np.array([float(t_info.get('v1', [1, 0])[0]), float(t_info.get('v1', [1, 0])[1])])
            v1_norm = np.linalg.norm(v1)
            v1 = v1 / v1_norm if v1_norm > 1e-3 else np.array([1.0, 0.0])
            
            v1_ang = np.arctan2(v1[1], v1[0])
            t_theta = -v1_ang + np.radians(35)
            t_phi = np.radians(18)
            
            tc_cos_t, tc_sin_t = np.cos(t_theta), np.sin(t_theta)
            tc_cos_p, tc_sin_p = np.cos(t_phi), np.sin(t_phi)
            
            min_z = float(t_info.get('min_z', np.min(t_sub_pts[:, 2])))
            max_z = float(t_info.get('max_z', np.max(t_sub_pts[:, 2])))
            p_t_c = t_sub_pts - np.array([tc_x, tc_y, (min_z + max_z) / 2.0])
            
            tx_rot = p_t_c[:, 0] * tc_cos_t - p_t_c[:, 1] * tc_sin_t
            ty_rot = p_t_c[:, 0] * tc_sin_t + p_t_c[:, 1] * tc_cos_t
            tz_rot = p_t_c[:, 2]
            
            tu_proj = tx_rot
            tv_proj = ty_rot * tc_sin_p + tz_rot * tc_cos_p
            
            t_only = t_sub_cls == 15
            if np.sum(t_only) >= 50:
                u_t_min, u_t_max = np.min(tu_proj[t_only]), np.max(tu_proj[t_only])
                v_t_min, v_t_max = np.min(tv_proj[t_only]), np.max(tv_proj[t_only])
            else:
                u_t_min, u_t_max = np.min(tu_proj), np.max(tu_proj)
                v_t_min, v_t_max = np.min(tv_proj), np.max(tv_proj)
                
            u_span = max(u_t_max - u_t_min, 10.0)
            v_span = max(v_t_max - v_t_min, 10.0)
            
            start_x = 1020
            start_y = 15 + t_i * sub_h
            cur_box_h = sub_h - 20
            
            canvas[start_y:start_y + cur_box_h, start_x:start_x + box_w] = [28, 30, 36]
            
            t_scale = min((box_w - 50) / (u_span + 4.0), (cur_box_h - 50) / (v_span + 4.0))
            
            t_px = ((tu_proj - (u_t_min + u_t_max) / 2.0) * t_scale + start_x + box_w / 2.0).astype(int)
            t_py = (start_y + cur_box_h - 25 - ((tv_proj - v_t_min) * t_scale)).astype(int)
            
            t_depth_sort = np.argsort(ty_rot)
            t_px_s, t_py_s = t_px[t_depth_sort], t_py[t_depth_sort]
            t_cols_s, t_cls_s = t_sub_cols[t_depth_sort], t_sub_cls[t_depth_sort]
            
            t_valid = (t_px_s >= start_x + 2) & (t_px_s < start_x + box_w - 2) & (t_py_s >= start_y + 2) & (t_py_s < start_y + cur_box_h - 2)
            
            non_t_mask = t_valid & (t_cls_s != 15)
            canvas[t_py_s[non_t_mask], t_px_s[non_t_mask]] = t_cols_s[non_t_mask]
            
            is_t_mask = t_valid & (t_cls_s == 15)
            for dx in (0, 1):
                for dy in (0, 1):
                    canvas[np.clip(t_py_s[is_t_mask] + dy, 0, H-1), np.clip(t_px_s[is_t_mask] + dx, 0, W-1)] = t_cols_s[is_t_mask]
                    
    os.makedirs(os.path.dirname(os.path.abspath(output_img_path)), exist_ok=True)
    import zlib
    import struct
    h_c, w_c, _ = canvas.shape
    raw_bytes = b''.join(b'\x00' + canvas[y].tobytes() for y in range(h_c))
    compressed = zlib.compress(raw_bytes, 4)
    def _chunk(tag, data):
        return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', zlib.crc32(tag + data))
    ihdr = struct.pack('>IIBBBBB', w_c, h_c, 8, 2, 0, 0, 0)
    png_bytes = b'\x89PNG\r\n\x1a\n' + _chunk(b'IHDR', ihdr) + _chunk(b'IDAT', compressed) + _chunk(b'IEND', b'')
    with open(output_img_path, 'wb') as f:
        f.write(png_bytes)
    return output_img_path

if __name__ == '__main__':
    if len(sys.argv) > 2:
        render_preview_image(sys.argv[1], sys.argv[2])

