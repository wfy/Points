import numpy as np
from scipy.spatial import cKDTree
from typing import List, Tuple
from modules.models import TowerEntity
from modules.config import PipelineConfig, DEFAULT_CONFIG

def fit_arm_ransac_2d(pts_2d: np.ndarray, max_trials: int = 100, inlier_thresh: float = 0.5) -> Tuple[np.ndarray, int]:
    """
    基于 2D RANSAC 拟合铁塔刚性金属角钢横担直线
    """
    n_pts = len(pts_2d)
    if n_pts < 5:
        return None, 0
        
    best_inliers_count = 0
    best_direction = None
    
    for _ in range(max_trials):
        idx = np.random.choice(n_pts, 2, replace=False)
        p1, p2 = pts_2d[idx[0]], pts_2d[idx[1]]
        v = p2 - p1
        dist = np.hypot(v[0], v[1])
        if dist < 1e-3:
            continue
        v_unit = v / dist
        
        normal = np.array([-v_unit[1], v_unit[0]])
        diff = pts_2d - p1
        per_dists = np.abs(diff @ normal)
        
        inliers_count = np.sum(per_dists <= inlier_thresh)
        if inliers_count > best_inliers_count:
            best_inliers_count = inliers_count
            best_direction = v_unit
            
    return best_direction, best_inliers_count

def cluster_tower_voxels(off_ground_pts: np.ndarray, 
                         rel_z: np.ndarray, 
                         t_grid_size: float = 2.0,
                         min_tower_rel_z: float = 22.0,
                         min_pts_count: int = 500,
                         continuity_ratio: float = 0.65) -> List[dict]:
    """
    基于 3D 体素垂直连续性与 2D 连通域图搜索初筛杆塔候选聚类中心
    """
    tgx = (off_ground_pts[:, 0] / t_grid_size).astype(np.int32)
    tgy = (off_ground_pts[:, 1] / t_grid_size).astype(np.int32)
    tgz = np.maximum((rel_z / 2.0).astype(np.int32), 0)
    
    min_x, min_y = int(np.min(tgx)), int(np.min(tgy))
    ny = int(np.max(tgy) - min_y + 1)
    code_2d = (tgx - min_x).astype(np.int64) * ny + (tgy - min_y).astype(np.int64)
    
    u_code, inv_2d = np.unique(code_2d, return_inverse=True)
    u_tgx = (u_code // ny + min_x).astype(np.int32)
    u_tgy = (u_code % ny + min_y).astype(np.int32)
    unique_2d = np.column_stack((u_tgx, u_tgy))
    
    grid_pts_counts = np.bincount(inv_2d, minlength=len(u_code))
    min_h_layer = int(min_tower_rel_z / 2.0)
    
    valid_grid_idx = np.where(grid_pts_counts >= min_pts_count)[0]
    if len(valid_grid_idx) == 0:
        return []
        
    candidate_tower_indices = []
    mask_pts_in_valid = np.isin(inv_2d, valid_grid_idx)
    sub_inv = inv_2d[mask_pts_in_valid]
    sub_z = tgz[mask_pts_in_valid]
    
    grid_heights = {}
    for g_i in valid_grid_idx:
        gz_layers = sub_z[sub_inv == g_i]
        if len(gz_layers) == 0:
            continue
        max_h = int(np.max(gz_layers))
        num_layers = len(np.unique(gz_layers))
        if max_h >= min_h_layer and (num_layers / (max_h + 1)) >= continuity_ratio:
            candidate_tower_indices.append(g_i)
            grid_heights[g_i] = max_h
            
    raw_tower_infos = []
    if len(candidate_tower_indices) > 0:
        cand_arr = unique_2d[candidate_tower_indices]
        cand_tree = cKDTree(cand_arr)
        pairs = cand_tree.query_pairs(r=1.5)  # 8-邻域连通
        
        parent = list(range(len(cand_arr)))
        def find(i):
            if parent[i] == i: return i
            parent[i] = find(parent[i])
            return parent[i]
        def union(i, j):
            ri, rj = find(i), find(j)
            if ri != rj: parent[ri] = rj
            
        for i, j in pairs:
            ri, rj = find(i), find(j)
            if ri != rj: parent[ri] = rj
            
        clusters = {}
        for i in range(len(cand_arr)):
            r = find(i)
            clusters.setdefault(r, []).append(i)
            
        for r, members in clusters.items():
            if len(members) >= 2:
                pts_2d = cand_arr[members]
                max_z = 0.0
                for m in members:
                    orig_idx = candidate_tower_indices[m]
                    mz = grid_heights.get(orig_idx, 0) * 2.0
                    if mz > max_z:
                        max_z = mz
                        
                peak_members = []
                for m in members:
                    orig_idx = candidate_tower_indices[m]
                    mz = grid_heights.get(orig_idx, 0) * 2.0
                    if mz >= (max_z - 4.0):
                        peak_members.append(m)
                        
                if len(peak_members) > 0:
                    peak_pts_2d = cand_arr[peak_members]
                    cx, cy = np.mean(peak_pts_2d, axis=0) * t_grid_size + t_grid_size / 2.0
                else:
                    cx, cy = np.mean(pts_2d, axis=0) * t_grid_size + t_grid_size / 2.0
                    
                raw_tower_infos.append({
                    'cx': cx, 'cy': cy, 'max_z': max_z
                })
                
    # 杆塔最小档距 NMS 合并 (< 25m)
    tower_candidates = []
    if len(raw_tower_infos) > 0:
        centers_2d = np.array([[t['cx'], t['cy']] for t in raw_tower_infos])
        tree_nms = cKDTree(centers_2d)
        nms_pairs = tree_nms.query_pairs(r=25.0)
        
        parent_nms = list(range(len(raw_tower_infos)))
        def find_nms(i):
            if parent_nms[i] == i: return i
            parent_nms[i] = find_nms(parent_nms[i])
            return parent_nms[i]
            
        for i, j in nms_pairs:
            ri, rj = find_nms(i), find_nms(j)
            if ri != rj: parent_nms[ri] = rj
            
        nms_clusters = {}
        for i in range(len(raw_tower_infos)):
            r_id = find_nms(i)
            nms_clusters.setdefault(r_id, []).append(i)
        for r_id, members in nms_clusters.items():
            best_member = max(members, key=lambda m: raw_tower_infos[m]['max_z'])
            tower_candidates.append(raw_tower_infos[best_member])
            
    return tower_candidates

def detect_towers(off_ground_pts: np.ndarray, 
                  rel_z: np.ndarray, 
                  off_ground_idx: np.ndarray, 
                  t_grid_size: float = 2.0,
                  config: PipelineConfig = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray, List[TowerEntity]]:
    """
    阶段二：3D 体素垂直连续性 + 2D 连通域聚类锁定铁塔与自适应横担拟合
    【全面防御与闭环增强机制】：
      1. 杜绝误报：真横担强约束 (Crossarm Verification)，无横担且无空腔的树林+导线直接一票否决 (解决图 4)；
      2. 杜绝漏塔：高连续性骨架保护，废除林区 Δh_relief 对超高真塔的误杀 (解决 125-126 图 1)；
      3. 完整捕获：顺线厚度放宽至 4.5m + 塔顶尖部缓冲 + 横担外展放宽至 16.5m (解决图 5、图 3)；
      4. 消除镂空：塔身放坡系数 0.25 (半宽 12m) + 3D 角钢空间区域生长 (Region Growing) (解决图 2)。
    """
    if config is None:
        config = DEFAULT_CONFIG
        
    t_cfg = config.tower
    
    tower_candidates = cluster_tower_voxels(
        off_ground_pts=off_ground_pts, 
        rel_z=rel_z, 
        t_grid_size=t_grid_size,
        min_tower_rel_z=t_cfg.min_tower_rel_z,
        min_pts_count=t_cfg.min_pts_count,
        continuity_ratio=t_cfg.continuity_ratio
    )
    
    is_tower = np.zeros(len(off_ground_pts), dtype=bool)
    is_tower_arm = np.zeros(len(off_ground_pts), dtype=bool)
    is_near_tower_high_arm = np.zeros(len(off_ground_pts), dtype=bool)
    valid_tower_entities: List[TowerEntity] = []
    
    if len(tower_candidates) > 0:
        off_ground_tree = cKDTree(off_ground_pts[:, :2])
        for cand in tower_candidates:
            cx, cy, max_z = cand['cx'], cand['cy'], cand['max_z']
            
            r_search = min(max(max_z * 0.45, 14.0), 30.0)
            indices = off_ground_tree.query_ball_point([cx, cy], r=r_search)
            
            local_pts = off_ground_pts[indices]
            local_rel_z = rel_z[indices]
            valid_mask = local_rel_z <= (max_z + 4.0)  # 容纳塔顶地线尖顶
            valid_indices = np.array(indices)[valid_mask]
            tower_z = local_rel_z[valid_mask]
            tower_pts = local_pts[valid_mask]
            
            if len(tower_z) < 200:
                continue
                
            # 【核心改进 3：真实 3D 垂直体素连通性分析 (3D Voxel CC)】
            # 采用轻量下采样点集 (上限 30,000 点) 构建体素连通图，极速验证是否存在自地面跨越至塔顶的主 3D 连通体
            v_vox_size = 1.2
            step_cc = max(len(tower_pts) // 30000, 1)
            pts_sample_cc = tower_pts[::step_cc]
            z_sample_cc = tower_z[::step_cc]
            
            vx_3d = (pts_sample_cc[:, 0] / v_vox_size).astype(np.int32)
            vy_3d = (pts_sample_cc[:, 1] / v_vox_size).astype(np.int32)
            vz_3d = (z_sample_cc / v_vox_size).astype(np.int32)
            
            min_vx, min_vy, min_vz = int(np.min(vx_3d)), int(np.min(vy_3d)), int(np.min(vz_3d))
            n_vy = int(np.max(vy_3d) - min_vy + 1)
            n_vz = int(np.max(vz_3d) - min_vz + 1)
            code_3d = (vx_3d - min_vx).astype(np.int64) * (n_vy * n_vz) + (vy_3d - min_vy).astype(np.int64) * n_vz + (vz_3d - min_vz).astype(np.int64)
            
            u_code = np.unique(code_3d)
            u_vx = (u_code // (n_vy * n_vz) + min_vx).astype(np.int32)
            u_vy = ((u_code % (n_vy * n_vz)) // n_vz + min_vy).astype(np.int32)
            u_vz = (u_code % n_vz + min_vz).astype(np.int32)
            u_vox_3d = np.column_stack((u_vx, u_vy, u_vz))
            
            v_tree = cKDTree(u_vox_3d)
            pairs_3d = v_tree.query_pairs(r=1.75)
            
            p_vox = list(range(len(u_vox_3d)))
            def find_vox(i):
                if p_vox[i] == i: return i
                p_vox[i] = find_vox(p_vox[i])
                return p_vox[i]
                
            for u, v in pairs_3d:
                ru, rv = find_vox(u), find_vox(v)
                if ru != rv: p_vox[ru] = rv
                
            vox_clusters = {}
            for i in range(len(u_vox_3d)):
                root = find_vox(i)
                vox_clusters.setdefault(root, []).append(i)
                
            max_cc_z_span = 0.0
            has_cc_bottom = False
            has_cc_top = False
            
            for root, members in vox_clusters.items():
                cc_z = u_vox_3d[members, 2] * v_vox_size
                min_cz = np.min(cc_z)
                max_cz = np.max(cc_z)
                z_span = max_cz - min_cz
                if z_span > max_cc_z_span:
                    max_cc_z_span = z_span
                    has_cc_bottom = np.any(cc_z <= 5.0)
                    has_cc_top = np.any(cc_z >= (max_z - 6.0))
                    
            # 3D 体素连通性刚性检验：排除图 4 悬空导线 + 下方独立树冠伪目标
            if not (has_cc_bottom and has_cc_top and max_cc_z_span >= max(max_z * 0.60, 16.0)):
                continue

            # 【核心优化：高层纯净钢构对称区物理中心自对齐 (Dynamic Centroid Recalibration)】
            # 在纯净塔身段 (Z in [0.45*max_z, 0.85*max_z], 且靠近粗中心 8m 内) 采用双侧外缘 [2%, 98%] 几何中点，不受迎光/背光点云密度差异干扰，彻底锁定真实对称轴心
            waist_mask = (tower_z >= max_z * 0.45) & (tower_z <= max_z * 0.85) & (np.hypot(tower_pts[:, 0] - cx, tower_pts[:, 1] - cy) <= 8.0)
            if np.sum(waist_mask) >= 30:
                w_pts = tower_pts[waist_mask]
                cx = float((np.percentile(w_pts[:, 0], 2) + np.percentile(w_pts[:, 0], 98)) / 2.0)
                cy = float((np.percentile(w_pts[:, 1], 2) + np.percentile(w_pts[:, 1], 98)) / 2.0)

            # RANSAC 横担方向拟合
            half_arm_max = min(max(max_z * 0.32, 6.0), 16.5)
            steel_z_min, steel_z_max = max_z * 0.70, max_z * 0.98
            diff_all_tower = tower_pts[:, :2] - np.array([cx, cy])
            dist_all_tower = np.hypot(diff_all_tower[:, 0], diff_all_tower[:, 1])
            lattice_arm_mask = (tower_z >= steel_z_min) & (tower_z <= steel_z_max) & (dist_all_tower <= min(half_arm_max * 0.65, 9.5))
            lattice_arm_pts = tower_pts[lattice_arm_mask]

            v_arm, inlier_count = fit_arm_ransac_2d(
                lattice_arm_pts[:, :2] - np.array([cx, cy]),
                max_trials=t_cfg.arm_ransac_trials,
                inlier_thresh=t_cfg.arm_ransac_inlier_thresh
            )
            
            if v_arm is not None and inlier_count >= 8:
                v1 = v_arm
                v2 = np.array([-v1[1], v1[0]])
            else:
                high_arm_zone_temp = tower_z >= (max_z * 0.40)
                arm_pts_temp = tower_pts[high_arm_zone_temp]
                if len(arm_pts_temp) >= 10:
                    cov_arm = np.cov(arm_pts_temp[:, :2].T)
                    evals_a, evecs_a = np.linalg.eigh(cov_arm)
                    v1 = evecs_a[:, 0]
                    v2 = evecs_a[:, 1]
                else:
                    v1, v2 = np.array([1.0, 0.0]), np.array([0.0, 1.0])
                    
            d_v1 = np.abs(diff_all_tower @ v1)
            d_v2 = np.abs(diff_all_tower @ v2)

            # 横担分层切片剖面分析
            z_slice_step = 0.6
            min_search_z = max_z * (0.35 if t_cfg.allow_distribution_poles else 0.45)
            max_search_z = max_z * 0.98
            
            slice_centers = np.arange(min_search_z, max_search_z, z_slice_step)
            crossarm_candidate_z = []

            for z_c in slice_centers:
                slice_mask = (tower_z >= z_c - z_slice_step * 0.6) & (tower_z <= z_c + z_slice_step * 0.6)
                if np.sum(slice_mask) < 6:
                    continue
                
                d_v1_slice = d_v1[slice_mask]
                d_v2_slice = d_v2[slice_mask]
                
                w1 = float(np.percentile(d_v1_slice, 95))
                w2 = float(np.percentile(d_v2_slice, 95))
                
                min_w1_thresh = 2.0 if t_cfg.allow_distribution_poles else 3.5
                min_diff_thresh = 0.8 if t_cfg.allow_distribution_poles else 1.0
                
                if w1 >= min_w1_thresh and (w1 - w2) >= min_diff_thresh and (w1 >= 1.30 * max(w2, 1.0)):
                    outer_pts_count = np.sum(d_v1_slice >= (min_w1_thresh * 0.85))
                    if outer_pts_count >= 3:
                        crossarm_candidate_z.append(z_c)

            is_robust_lattice = (max_cc_z_span >= (max_z * 0.80)) and (len(tower_z) >= 1000) and (max_z >= 22.0)
            
            # 【以 0-1(0_1) Tower 2 (30.64m) 为黄金基准点，单塔参数 = 基准参数 x 单塔真实净高倍数 k】
            core_mask = dist_all_tower <= 5.0
            if np.sum(core_mask) >= 10:
                base_ground_z = float(np.percentile(tower_z[core_mask], 3.0))
                top_ground_z = float(np.max(tower_z[core_mask]))
                h_true = max(top_ground_z - base_ground_z, 10.0)
            else:
                h_true = float(max_z)

            h0 = getattr(t_cfg, 'base_anchor_height', 30.64)
            k_height = max(float(h_true / h0), 0.5)

            slope0 = getattr(t_cfg, 'base_anchor_slope_rate', 0.0811)
            shell0 = getattr(t_cfg, 'base_anchor_shell_thick', 1.52)
            margin0 = getattr(t_cfg, 'base_anchor_margin', 0.61)

            max_slope_rate = slope0 * k_height
            shell_thick = shell0 * k_height
            margin_val = margin0 * k_height + 0.35  # 增加 0.35m 钢构角钢法兰外凸安装裕量
            
            # 横担下沿高度自适应比例
            arm_ratio = float(np.clip(0.45 - 0.03 * (k_height - 1.0), 0.36, 0.48))
            z_downward_offset = getattr(t_cfg, 'lowest_arm_downward_ratio', 0.10) * h_true
            
            if len(crossarm_candidate_z) == 0:
                if not is_robust_lattice or h_true < 22.0:
                    continue
                z_lowest_arm = max(h_true * arm_ratio, 5.0)
            else:
                z_lowest_arm = max(float(min(crossarm_candidate_z)) - 0.8 - z_downward_offset, 5.0)

            # 【精准测量塔身纯立柱半宽 (W_trunk)】：在横担纯净区 (Z >= z_lowest_arm) 测量，取 98% 分位数完整涵盖外侧角钢边缘
            clean_trunk_m = (tower_z >= z_lowest_arm) & (tower_z <= min(z_lowest_arm + 5.0, max_z)) & (d_v1 <= (4.2 * np.sqrt(k_height))) & (d_v2 <= (4.2 * np.sqrt(k_height)))
            if np.sum(clean_trunk_m) >= 5:
                w1_trunk = float(np.percentile(d_v1[clean_trunk_m], 98))
                w2_trunk = float(np.percentile(d_v2[clean_trunk_m], 98))
            else:
                w1_trunk, w2_trunk = 1.8 * np.sqrt(k_height), 1.8 * np.sqrt(k_height)
            w_waist = max(w1_trunk, w2_trunk)
            
            # 【核心改进 1：图 1 横担上部包络乘系数 (1.18x / 1.25x)，补齐少量漏点】
            high_arm_zone = tower_z >= z_lowest_arm
            arm_pts = tower_pts[high_arm_zone]

            if len(arm_pts) >= 5:
                d1_high = d_v1[high_arm_zone]
                d2_high = d_v2[high_arm_zone]
                half_arm_w = min(max(np.percentile(d1_high, 99.5) * 1.18 + 1.2, 5.5), half_arm_max * 1.20)
                half_line_t = min(max(np.percentile(d2_high, 98) * 1.25 + 0.8, 2.5), 4.8)
            else:
                half_arm_w = half_arm_max * 1.15
                half_line_t = 3.5

            mask_high = high_arm_zone & (d_v1 <= half_arm_w) & (d_v2 <= half_line_t)
            top_bracket_mask = (tower_z >= (max_z - 4.5)) & (d_v1 <= (w_waist * 1.35 + 2.0)) & (d_v2 <= 3.8)

            # 【核心优化：纯几何实体四棱台体 (Solid Quadrangular Frustum) 完整包络】
            # 从 z_lowest_arm 处的横担下腰正方形向下线性放坡延伸至塔基地面正方形
            # 彻底废除“内部死区掏空”与“塔脚立柱截断”，100% 完整捕获四棱台内的所有 X 交叉斜撑、水平隔梁与斜坡塔腿
            delta_z_all = np.maximum(z_lowest_arm - tower_z, 0.0)
            w1_allowed = w1_trunk + delta_z_all * max_slope_rate + margin_val
            w2_allowed = w2_trunk + delta_z_all * max_slope_rate + margin_val
            
            low_tower_mask = (tower_z < z_lowest_arm) & (tower_z >= 0.0) & (d_v1 <= w1_allowed) & (d_v2 <= w2_allowed)

            final_obb_mask = mask_high | low_tower_mask | top_bracket_mask
            taper_valid_indices = valid_indices[final_obb_mask]
            
            w_trunk0 = w_waist
            trunk_column_mask = high_arm_zone & (d_v1 <= w_trunk0) & (d_v2 <= w_trunk0)
            arm_wing_mask = mask_high & (~trunk_column_mask)

            final_valid_indices = taper_valid_indices

            is_tower[final_valid_indices] = True
            is_tower_arm[valid_indices[arm_wing_mask]] = True
            
            abs_max_z = float(np.max(off_ground_pts[final_valid_indices, 2])) if len(final_valid_indices) > 0 else 0.0
            
            skeleton_conf = 1.0
            
            entity = TowerEntity(
                cx=cx,
                cy=cy,
                max_z=max_z,
                abs_max_z=abs_max_z,
                v1=v1,
                v2=v2,
                half_l1=half_arm_w,
                half_l2=half_line_t,
                d_diag=2.0 * float(np.hypot(half_arm_w, half_line_t)),
                z_lowest_arm=z_lowest_arm,
                w_trunk0=w_trunk0,
                confidence=skeleton_conf,
                pts_idx=final_valid_indices
            )
            if len(final_valid_indices) > 0:
                valid_tower_entities.append(entity)
            
            dist_near_sq = (tower_pts[:, 0] - cx)**2 + (tower_pts[:, 1] - cy)**2
            near_mask = (dist_near_sq <= (20.0 ** 2)) & (tower_z >= 10.0)
            is_near_tower_high_arm[valid_indices[near_mask]] = True
            
    return is_tower, is_tower_arm, is_near_tower_high_arm, valid_tower_entities
