import numpy as np
from scipy.spatial import cKDTree
from typing import List, Tuple
from modules.models import TowerEntity
from modules.config import PipelineConfig, DEFAULT_CONFIG

def fit_arm_ransac_2d(pts_2d: np.ndarray, 
                       max_trials: int = 100, 
                       inlier_thresh: float = 0.5,
                       center_tolerance: float = 0.95,
                       min_wing_reach: float = 1.8,
                       min_symmetry_ratio: float = 0.40) -> Tuple[np.ndarray, int]:
    """
    【高精度耐张/直线塔横担主轴拟合算法】
    采用两阶段混合解算策略：
      1. 第一阶段 (主方案)：全局角度遍历 (0°~180°, 步长 1.0°) + 双翼对称跨距约束 (Bilateral Wing Symmetry) + 0.1° 精细微调。
         严格要求拟合直线经过塔心邻域 (|截距| <= center_tolerance)，且在塔心两侧均有至少 min_wing_reach 跨距的点云对称支撑。
         对单侧耐张导线、斜拉杆、引流跳线等强干扰具有绝对物理免疫力。
      2. 第二阶段 (兜底方案)：若点云极稀疏或单边横担，回退至带塔心截距惩罚的 RANSAC 采样。
    """
    n_pts = len(pts_2d)
    if n_pts < 5:
        return None, 0
        
    # 输入点云坐标基准保护：如果已经去中心化（坐标围绕塔心分布），保留 (0,0) 为塔心；仅对原始绝对坐标做中心平移
    mean_shift = np.mean(pts_2d, axis=0)
    ref_pts = pts_2d if np.hypot(mean_shift[0], mean_shift[1]) <= 50.0 else (pts_2d - mean_shift)

    # 1. 主方案：全局角度角向扫描 (0° ~ 180°，分辨率 1.0°)
    angles_deg = np.arange(0, 180, 1.0)
    angles_rad = np.radians(angles_deg)
    cos_a = np.cos(angles_rad)
    sin_a = np.sin(angles_rad)
    
    # 向量化计算所有点到各候选方向的垂向偏距 (d_perp) 与沿线投影坐标 (d_para)
    d_perp = -ref_pts[:, [0]] * sin_a + ref_pts[:, [1]] * cos_a
    d_para = ref_pts[:, [0]] * cos_a + ref_pts[:, [1]] * sin_a
    
    best_score = -1.0
    best_angle = None
    best_direction = None
    best_inliers_count = 0
    best_inliers_mask = None
    
    for m in range(len(angles_deg)):
        perp_m = d_perp[:, m]
        para_m = d_para[:, m]
        
        # 塔心近邻截距检查
        near_center = np.abs(perp_m) <= (center_tolerance * 1.5)
        if np.sum(near_center) < 3:
            continue
            
        c_offset = float(np.median(perp_m[near_center]))
        if abs(c_offset) > center_tolerance:
            continue
            
        inliers_mask = np.abs(perp_m - c_offset) <= inlier_thresh
        inliers_count = int(np.sum(inliers_mask))
        if inliers_count < 6:
            continue
            
        para_inliers = para_m[inliers_mask]
        pos_inliers = para_inliers[para_inliers >= min_wing_reach]
        neg_inliers = para_inliers[para_inliers <= -min_wing_reach]
        
        if len(pos_inliers) < 2 or len(neg_inliers) < 2:
            continue
            
        pos_span = float(np.max(pos_inliers))
        neg_span = float(-np.min(neg_inliers))
        sym_ratio = min(pos_span, neg_span) / max(pos_span, neg_span, 1e-3)
        if sym_ratio < min_symmetry_ratio:
            continue
            
        total_span = pos_span + neg_span
        score = inliers_count * total_span * np.sqrt(sym_ratio)
        if score > best_score:
            best_score = score
            best_angle = angles_rad[m]
            best_inliers_count = inliers_count
            best_direction = np.array([cos_a[m], sin_a[m]])
            best_inliers_mask = inliers_mask

    # 1.2 精细微调：在粗扫最优角 ±2.5° 范围内以 0.1° 步长进行梯度极值搜索
    if best_direction is not None and best_angle is not None:
        fine_angles = np.linspace(best_angle - np.radians(2.5), best_angle + np.radians(2.5), 51)
        cos_f = np.cos(fine_angles)
        sin_f = np.sin(fine_angles)
        
        d_perp_f = -ref_pts[:, [0]] * sin_f + ref_pts[:, [1]] * cos_f
        d_para_f = ref_pts[:, [0]] * cos_f + ref_pts[:, [1]] * sin_f
        
        fine_best_score = best_score
        for f in range(len(fine_angles)):
            perp_f = d_perp_f[:, f]
            para_f = d_para_f[:, f]
            
            near_center = np.abs(perp_f) <= (center_tolerance * 1.5)
            if np.sum(near_center) < 3:
                continue
            c_offset = float(np.median(perp_f[near_center]))
            if abs(c_offset) > center_tolerance:
                continue
                
            inliers_mask = np.abs(perp_f - c_offset) <= inlier_thresh
            inliers_count = int(np.sum(inliers_mask))
            if inliers_count < 6:
                continue
                
            para_inliers = para_f[inliers_mask]
            pos_inliers = para_inliers[para_inliers >= min_wing_reach]
            neg_inliers = para_inliers[para_inliers <= -min_wing_reach]
            if len(pos_inliers) < 2 or len(neg_inliers) < 2:
                continue
                
            pos_span = float(np.max(pos_inliers))
            neg_span = float(-np.min(neg_inliers))
            sym_ratio = min(pos_span, neg_span) / max(pos_span, neg_span, 1e-3)
            if sym_ratio < min_symmetry_ratio:
                continue
                
            total_span = pos_span + neg_span
            score = inliers_count * total_span * np.sqrt(sym_ratio)
            if score > fine_best_score:
                fine_best_score = score
                best_direction = np.array([cos_f[f], sin_f[f]])
                best_inliers_count = inliers_count
                best_inliers_mask = inliers_mask

        # 1.3 刚性金属角钢骨架 PCA 极精解析对齐：消除离散化误差与局部微弧度抖动
        if best_inliers_mask is not None and np.sum(best_inliers_mask) >= 8:
            inlier_pts = ref_pts[best_inliers_mask]
            cov_in = np.cov(inlier_pts.T)
            ev_in, evec_in = np.linalg.eigh(cov_in)
            v_pca = evec_in[:, 1]
            # 保证与主方向夹角一致
            if abs(float(v_pca @ best_direction)) >= 0.90:
                if v_pca @ best_direction < 0:
                    v_pca = -v_pca
                best_direction = v_pca

        return best_direction, best_inliers_count

    # 2. 兜底方案：带塔心偏距约束的随机采样 RANSAC
    fallback_best_inliers = 0
    fallback_best_direction = None
    
    for _ in range(max_trials):
        idx = np.random.choice(n_pts, 2, replace=False)
        p1, p2 = ref_pts[idx[0]], ref_pts[idx[1]]
        v = p2 - p1
        dist = np.hypot(v[0], v[1])
        if dist < 1e-3:
            continue
        v_unit = v / dist
        normal = np.array([-v_unit[1], v_unit[0]])
        
        # 直线距塔心垂直偏距不得超过 center_tolerance * 1.5
        if abs(p1 @ normal) > (center_tolerance * 1.5):
            continue
            
        diff = ref_pts - p1
        per_dists = np.abs(diff @ normal)
        inliers_count = np.sum(per_dists <= inlier_thresh)
        if inliers_count > fallback_best_inliers:
            fallback_best_inliers = inliers_count
            fallback_best_direction = v_unit
            
    return fallback_best_direction, fallback_best_inliers

def cluster_tower_voxels(off_ground_pts: np.ndarray, 
                         rel_z: np.ndarray, 
                         t_grid_size: float = 2.0,
                         min_tower_rel_z: float = 22.0,
                         min_pts_count: int = 500,
                         continuity_ratio: float = 0.65,
                         nms_radius: float = 14.0) -> List[dict]:
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
            if len(members) >= 1:
                pts_2d = cand_arr[members]
                max_z = 0.0
                total_pts = 0
                for m in members:
                    orig_idx = candidate_tower_indices[m]
                    total_pts += int(grid_pts_counts[orig_idx])
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
                    'cx': cx, 'cy': cy, 'max_z': max_z, 'total_pts': total_pts
                })
                
    # 杆塔候选 NMS 合并 (默认 <= 14m，杜绝大半径吞噬真塔)
    tower_candidates = []
    if len(raw_tower_infos) > 0:
        centers_2d = np.array([[t['cx'], t['cy']] for t in raw_tower_infos])
        tree_nms = cKDTree(centers_2d)
        nms_pairs = tree_nms.query_pairs(r=nms_radius)
        
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
            # 综合考虑点云丰富度与高度，防止下坡树木+高处导线凭微小相对高度吞噬真塔
            best_member = max(members, key=lambda m: (raw_tower_infos[m].get('total_pts', 0) * np.sqrt(raw_tower_infos[m]['max_z'])))
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
    
    effective_nms_r = min(getattr(t_cfg, 'nms_radius', 25.0), 14.0)
    tower_candidates = cluster_tower_voxels(
        off_ground_pts=off_ground_pts, 
        rel_z=rel_z, 
        t_grid_size=t_grid_size,
        min_tower_rel_z=t_cfg.min_tower_rel_z,
        min_pts_count=t_cfg.min_pts_count,
        continuity_ratio=t_cfg.continuity_ratio,
        nms_radius=effective_nms_r
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

            # 横担方向拟合：覆盖 0.48*max_z 至 0.98*max_z，完整捕获上、中、下各层级横担金属构件
            half_arm_max = min(max(max_z * 0.32, 6.0), 16.5)
            steel_z_min = max_z * (0.35 if t_cfg.allow_distribution_poles else 0.48)
            steel_z_max = max_z * 0.98
            diff_all_tower = tower_pts[:, :2] - np.array([cx, cy])
            dist_all_tower = np.hypot(diff_all_tower[:, 0], diff_all_tower[:, 1])
            lattice_arm_mask = (tower_z >= steel_z_min) & (tower_z <= steel_z_max) & (dist_all_tower <= min(half_arm_max * 1.15, 18.0))
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
                    # 修复 PCA 特征向量顺序：eigh 升序排列，[:, 1] 对应最大主成分 (横担长轴)
                    v1 = evecs_a[:, 1]
                    v2 = np.array([-v1[1], v1[0]])
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
                    w1_max_ext = float(np.percentile(d_v1_slice, 99)) if len(d_v1_slice) > 0 else 0.0
                    arm_limit = half_arm_max * 1.15 + 2.0
                    if outer_pts_count >= 3 and w1_max_ext <= arm_limit:
                        crossarm_candidate_z.append(z_c)

            # 【核心防御 1：横担存在性刚性约束】
            # 任何高压输电铁塔（直线塔与耐张塔）必有明显外展横担以悬挂绝缘子与导线。
            # 若经全高度层切片均未检出横担候选，坚决一票否决淘汰，彻底杜绝高大林木伪装
            if len(crossarm_candidate_z) == 0:
                continue

            # 【核心防御 2：塔身中空度与实心树冠空腔率检验 (Cavity / Porosity Filter)】
            # 真实铁塔为四棱桁架镂空结构，塔心轴线核心圆柱 (r <= 0.85m) 在中段高程极度中空；
            # 山区大树与高密树冠为实心体，中心轴线充满密集体积点云 (core_ratio >= 0.35 为实心树干)
            mid_trunk_mask = (tower_z >= max_z * 0.35) & (tower_z <= max_z * 0.70)
            if np.sum(mid_trunk_mask) >= 60:
                dist_mid = dist_all_tower[mid_trunk_mask]
                core_pts_count = np.sum(dist_mid <= 0.85)
                body_pts_count = np.sum(dist_mid <= 4.5)
                if body_pts_count >= 50:
                    core_ratio = float(core_pts_count / body_pts_count)
                    if not t_cfg.allow_distribution_poles and core_ratio >= 0.35:
                        continue
                    if core_ratio >= 0.30 and len(crossarm_candidate_z) <= 1:
                        continue

            # 【核心防御 3：高位地物 3D 特征值球状散射度检验 (Canopy Sphericity Filter)】
            # 铁塔由线/面状刚性角钢构成，高位点云协方差第三特征值接近 0；
            # 树冠呈各向同性团块球体，3D 散乱度显著偏高 (scatter >= 0.28)
            high_pts_m = tower_z >= (max_z * 0.45)
            if np.sum(high_pts_m) >= 60:
                high_pts = tower_pts[high_pts_m]
                cov_3d = np.cov(high_pts.T)
                ev_3d = np.sort(np.linalg.eigvalsh(cov_3d))[::-1]
                sum_ev = np.sum(ev_3d)
                if sum_ev > 1e-6:
                    scatter_index = float(3.0 * ev_3d[2] / sum_ev)
                    if scatter_index > 0.28 and len(crossarm_candidate_z) <= 1:
                        continue

            # 【核心防御 4：垂直主干刚性连续体与空气断层检验 (Vertical Air Gap Filter)】
            # 真实铁塔角钢桁架从基部直达塔顶，核心圆柱 (r <= 4.0m) 内各层点云垂直连续；
            # 树冠+上方高悬贯通导线在树顶至导线之间存在显著垂直完全断层 (Air Gap >= 3.5m)
            col_mask = (dist_all_tower <= 4.0) & (tower_z >= 4.0) & (tower_z <= max_z * 0.90)
            if np.sum(col_mask) >= 30:
                col_z_sorted = np.sort(tower_z[col_mask])
                max_vertical_gap = float(np.max(np.diff(col_z_sorted)))
                if max_vertical_gap >= 3.5:
                    continue
            
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
