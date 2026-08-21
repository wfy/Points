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
    tgz = (rel_z / 2.0).astype(np.int32)
    tgz = np.maximum(tgz, 0)
    
    coords_3d = np.column_stack((tgx, tgy, tgz))
    unique_voxels, inv_3d = np.unique(coords_3d, axis=0, return_inverse=True)
    coords_2d = unique_voxels[:, :2]
    unique_2d, inv_2d = np.unique(coords_2d, axis=0, return_inverse=True)
    
    sort_idx = np.lexsort((unique_voxels[:, 2], inv_2d))
    sorted_inv_2d = inv_2d[sort_idx]
    sorted_z = unique_voxels[sort_idx, 2]
    
    _, start_indices = np.unique(sorted_inv_2d, return_index=True)
    z_voxels_per_grid = np.split(sorted_z, start_indices[1:])
    
    grid_heights = [z[-1] for z in z_voxels_per_grid]
    grid_counts = [len(z) for z in z_voxels_per_grid]
    
    pts_counts = np.bincount(inv_3d)
    grid_pts_counts = np.bincount(inv_2d, weights=pts_counts).astype(np.int32)
    
    min_h_layer = int(min_tower_rel_z / 2.0)
    candidate_tower_indices = []
    for i in range(len(unique_2d)):
        h_idx = grid_heights[i]
        c_idx = grid_counts[i]
        pt_c = grid_pts_counts[i]
        
        # 杆塔初筛门槛：高度 >= min_tower_rel_z, 点数 >= min_pts_count, 垂直占空比 >= continuity_ratio
        if h_idx >= min_h_layer and pt_c >= min_pts_count and (c_idx / (h_idx + 1)) >= continuity_ratio:
            candidate_tower_indices.append(i)
            
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
            if ri != rj: parent_ri = rj
            
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
                    mz = grid_heights[orig_idx] * 2.0
                    if mz > max_z:
                        max_z = mz
                        
                peak_members = []
                for m in members:
                    orig_idx = candidate_tower_indices[m]
                    mz = grid_heights[orig_idx] * 2.0
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
    """
    np.random.seed(42)  # fixed RANSAC seed for reproducibility
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
            
            r_search = min(max(max_z * 0.45, 12.0), 25.0)
            indices = off_ground_tree.query_ball_point([cx, cy], r=r_search)
            
            local_pts = off_ground_pts[indices]
            local_rel_z = rel_z[indices]
            valid_mask = local_rel_z <= (max_z + 2.0)
            valid_indices = np.array(indices)[valid_mask]
            tower_z = local_rel_z[valid_mask]
            tower_pts = local_pts[valid_mask]
            
            if len(tower_z) < 200:
                continue
                
            # 全高度 3D 垂直空间连续性与结构完整性校验
            v_step = 2.0
            layer_indices = (tower_z / v_step).astype(np.int32)
            max_layer = int(np.max(tower_z) / v_step)
            min_layer = int(np.min(tower_z) / v_step)
            
            total_layers = max(max_layer - min_layer + 1, 1)
            occupied_layers = len(np.unique(layer_indices))
            vertical_continuity = occupied_layers / total_layers
            
            has_bottom = np.any(tower_z <= 6.0)
            has_top = np.any(tower_z >= (max_z - 6.0))
            
            if vertical_continuity >= 0.60 and has_bottom and has_top:
                # 突兀高差与林冠突出度校验
                outer_ring_indices = off_ground_tree.query_ball_point([cx, cy], r=25.0)
                delta_h_relief = 10.0
                if len(outer_ring_indices) >= 20:
                    outer_pts_2d = off_ground_pts[outer_ring_indices, :2]
                    outer_dist = np.hypot(outer_pts_2d[:, 0] - cx, outer_pts_2d[:, 1] - cy)
                    ring_mask_25 = (outer_dist >= 12.0) & (outer_dist <= 25.0)
                    
                    if np.sum(ring_mask_25) >= 15:
                        ring_z = rel_z[np.array(outer_ring_indices)[ring_mask_25]]
                        h_outer_canopy = np.percentile(ring_z, 90)
                        delta_h_relief = max_z - h_outer_canopy
                        
                        if not t_cfg.allow_distribution_poles:
                            if max_z < t_cfg.high_voltage_min_z or delta_h_relief < t_cfg.delta_h_relief:
                                continue

                # 杆塔受力与结构力学比例约束
                half_arm_max = min(max(max_z * 0.28, 4.5), 14.0)

                # 塔心纯角钢区 RANSAC 采样
                steel_z_min = max_z * 0.75
                steel_z_max = max_z * 0.98
                r_steel_search = min(half_arm_max * 0.65, 8.5)

                diff_all_tower = tower_pts[:, :2] - np.array([cx, cy])
                dist_all_tower = np.hypot(diff_all_tower[:, 0], diff_all_tower[:, 1])

                lattice_arm_mask = (tower_z >= steel_z_min) & (tower_z <= steel_z_max) & (dist_all_tower <= r_steel_search)
                lattice_arm_pts = tower_pts[lattice_arm_mask]

                # 2D RANSAC 拟合横担主轴
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
                        
                # 最下方横担高度解算
                d_v1 = np.abs(diff_all_tower @ v1)
                d_v2 = np.abs(diff_all_tower @ v2)

                arm_outer_mask = (d_v1 >= 4.0) & (tower_z >= max_z * 0.35) & (tower_z <= max_z * 0.85)
                if np.sum(arm_outer_mask) >= 10:
                    z_lowest_arm = max(np.percentile(tower_z[arm_outer_mask], 5) - 0.8, max_z * 0.35)
                else:
                    z_lowest_arm = max_z * 0.40

                high_arm_zone = tower_z >= z_lowest_arm
                arm_pts = tower_pts[high_arm_zone]

                if len(arm_pts) >= 5:
                    diff_arm = arm_pts[:, :2] - np.array([cx, cy])
                    proj1 = diff_arm @ v1
                    proj2 = diff_arm @ v2
                    half_arm_w = min(max(np.percentile(np.abs(proj1), 99.5) + 2.0, 5.0), half_arm_max)
                    half_line_t = min(max(np.percentile(np.abs(proj2), 98) + 0.6, 1.8), 2.8)
                else:
                    half_arm_w = half_arm_max
                    half_line_t = 2.8

                mask_high = high_arm_zone & (d_v1 <= half_arm_w) & (d_v2 <= half_line_t)

                # 塔身塔脚区平滑外扩
                near_lowest_mask = (tower_z >= (z_lowest_arm - 1.5)) & (tower_z <= (z_lowest_arm + 1.0))
                if np.sum(near_lowest_mask) >= 5:
                    w_trunk0 = min(max(np.percentile(d_v2[near_lowest_mask], 95), 1.2), 2.5)
                else:
                    w_trunk0 = min(max(half_line_t * 0.75, 1.2), 2.2)

                depth_z = np.maximum(z_lowest_arm - tower_z, 0.0)
                low_allowed_half_w = np.minimum(w_trunk0 + depth_z * 0.14, 7.5)
                mask_low = (~high_arm_zone) & (d_v1 <= low_allowed_half_w) & (d_v2 <= low_allowed_half_w)

                obb_mask = mask_high | mask_low
                taper_valid_indices = valid_indices[obb_mask]
                
                trunk_column_mask = high_arm_zone & (d_v1 <= w_trunk0) & (d_v2 <= w_trunk0)
                arm_wing_mask = mask_high & (~trunk_column_mask)

                final_valid_indices = taper_valid_indices

                is_tower[final_valid_indices] = True
                is_tower_arm[valid_indices[arm_wing_mask]] = True
                
                abs_max_z = float(np.max(off_ground_pts[final_valid_indices, 2])) if len(final_valid_indices) > 0 else 0.0
                
                # 计算几何骨架置信度 (综合垂直占空比与突出度)
                skeleton_conf = min(max(vertical_continuity * 0.7 + (delta_h_relief / 15.0) * 0.3, 0.5), 1.0)
                
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
                
                near_indices = off_ground_tree.query_ball_point([cx, cy], r=20.0)
                high_mask_zone = rel_z[near_indices] >= 10.0
                valid_near_indices = np.array(near_indices)[high_mask_zone]
                is_near_tower_high_arm[valid_near_indices] = True
                
    return is_tower, is_tower_arm, is_near_tower_high_arm, valid_tower_entities
