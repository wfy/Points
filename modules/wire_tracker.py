import numpy as np
from scipy.spatial import cKDTree
from typing import List, Tuple, Set, Callable, Union, Optional
from modules.models import TowerEntity, WireCluster
from modules.catenary import fit_catenary_3d, CatenaryModel
from modules.config import PipelineConfig, DEFAULT_CONFIG

def cluster_wire_candidates(points: np.ndarray,
                            high_pts: np.ndarray,
                            high_indices: np.ndarray,
                            cable_seed_indices: np.ndarray,
                            c_voxel_size: float = 0.5) -> Tuple[np.ndarray, np.ndarray, List[WireCluster]]:
    """
    基于体素连通图聚类、共线姿态验证与 3D 悬链线物理建模，聚合导线候选段
    """
    if len(cable_seed_indices) == 0:
        return np.empty((0, 3)), np.array([], dtype=int), []
        
    seed_pts = points[cable_seed_indices]
    seed_tree = cKDTree(seed_pts.astype(np.float32))
    
    dists, _ = seed_tree.query(high_pts, distance_upper_bound=2.5)
    in_range_mask = dists <= 2.5
    
    candidate_cable_indices = high_indices[in_range_mask]
    if len(candidate_cable_indices) == 0:
        return np.empty((0, 3)), np.array([], dtype=int), []
        
    cand_pts = points[candidate_cable_indices]
    cand_tree = cKDTree(cand_pts.astype(np.float32))
    dense_counts = cand_tree.query_ball_point(cand_pts, r=2.5, return_length=True)
    valid_cable_mask = dense_counts >= 3
    
    cable_pts_idx = candidate_cable_indices[valid_cable_mask]
    if len(cable_pts_idx) == 0:
        return np.empty((0, 3)), np.array([], dtype=int), []
        
    final_cable_pts = points[cable_pts_idx]
    
    # 体素连通聚类
    cvx = (final_cable_pts[:, 0] / c_voxel_size).astype(np.int32)
    cvy = (final_cable_pts[:, 1] / c_voxel_size).astype(np.int32)
    cvz = (final_cable_pts[:, 2] / c_voxel_size).astype(np.int32)
    c_coords = np.column_stack((cvx, cvy, cvz))
    
    unique_cvoxels, inv_c = np.unique(c_coords, axis=0, return_inverse=True)
    cvoxel_centers = unique_cvoxels * c_voxel_size + c_voxel_size / 2.0
    
    c_tree = cKDTree(cvoxel_centers.astype(np.float32))
    c_pairs = c_tree.query_pairs(r=2.0)
    
    parent_c = list(range(len(unique_cvoxels)))
    def find_c(i):
        root = i
        while parent_c[root] != root:
            root = parent_c[root]
        curr = i
        while curr != root:
            nxt = parent_c[curr]
            parent_c[curr] = root
            curr = nxt
        return root
    def union_c(i, j):
        ri, rj = find_c(i), find_c(j)
        if ri != rj: parent_c[ri] = rj
        
    for i, j in c_pairs:
        union_c(i, j)
        
    c_clusters = {}
    for i in range(len(final_cable_pts)):
        voxel_idx = inv_c[i]
        r = find_c(voxel_idx)
        c_clusters.setdefault(r, []).append(i)
        
    sure_wire_clusters = []
    candidate_clusters = []
    
    for r, members in c_clusters.items():
        if len(members) < 10:
            continue
        c_pts = final_cable_pts[members]
        ptp = np.ptp(c_pts, axis=0)
        diag_span = float(np.linalg.norm(ptp))
        
        cov = np.cov(c_pts.T)
        evals, evecs = np.linalg.eigh(cov)
        linearity = 0.0
        if evals[-1] > 0:
            linearity = float((evals[2] - evals[1]) / (evals[2] + 1e-6))
        
        min_var = float(evals[0])
        
        # 尝试拟合 3D 悬链线物理模型
        cat_model = None
        if len(c_pts) >= 12 and diag_span >= 8.0:
            cat_model = fit_catenary_3d(c_pts)
            
        cluster_info = WireCluster(
            members=members,
            center=np.mean(c_pts, axis=0),
            dir=evecs[:, 2] if evals[-1] > 0 else np.array([1.0, 0.0, 0.0]),
            span=diag_span,
            linearity=linearity,
            min_var=min_var,
            catenary=cat_model
        )
        
        if diag_span > 30.0 and linearity > 0.8:
            sure_wire_clusters.append(cluster_info)
        elif diag_span > 15.0 and linearity > 0.75 and min_var < 1.5:
            sure_wire_clusters.append(cluster_info)
        elif diag_span > 8.0 and linearity > 0.85 and min_var < 1.0:
            sure_wire_clusters.append(cluster_info)
        elif cat_model is not None and cat_model.residual_rmse < 0.35 and diag_span > 10.0:
            sure_wire_clusters.append(cluster_info)
        elif diag_span > 3.0 and linearity > 0.6 and min_var < 3.0:
            candidate_clusters.append(cluster_info)
            
    validated_cands = []
    for cand in candidate_clusters:
        is_valid = False
        for sure in sure_wire_clusters:
            dir_xy_cand = cand.dir[:2]
            dir_xy_sure = sure.dir[:2]
            
            norm_cand = np.linalg.norm(dir_xy_cand)
            norm_sure = np.linalg.norm(dir_xy_sure)
            
            if norm_cand > 0 and norm_sure > 0:
                cos_theta_xy = abs(np.dot(dir_xy_cand, dir_xy_sure) / (norm_cand * norm_sure))
            else:
                cos_theta_xy = 0.0
                
            v_xy = cand.center[:2] - sure.center[:2]
            n_xy = np.array([-dir_xy_sure[1], dir_xy_sure[0]])
            if np.linalg.norm(n_xy) > 0:
                n_xy = n_xy / np.linalg.norm(n_xy)
            dist_to_line_xy = abs(np.dot(v_xy, n_xy))
            
            dist_along = np.linalg.norm(cand.center[:2] - sure.center[:2])
            allowed_dist_xy = 3.0 + dist_along * 0.04
            
            v = cand.center - sure.center
            cross = np.cross(v, sure.dir)
            dist_to_line_3d = np.linalg.norm(cross)
            
            if dist_to_line_xy < allowed_dist_xy and cos_theta_xy > 0.92 and dist_to_line_3d < 25.0:
                is_valid = True
                break
        if is_valid:
            validated_cands.append(cand)
            
    all_confirmed = sure_wire_clusters + validated_cands
    return final_cable_pts, cable_pts_idx, all_confirmed

def filter_canopy_by_probes(final_cable_pts: np.ndarray,
                            off_ground_pts: np.ndarray,
                            all_confirmed: List[WireCluster],
                            tower_infos: List[Union[TowerEntity, dict]]) -> Set[int]:
    """
    通过 3D 垂直下探探针阵列检测线路下方是否存在连续茂密树冠，剔除树冠杂波
    """
    suspect_line_ids = set()
    if len(all_confirmed) == 0:
        return suspect_line_ids
        
    off_tree = cKDTree(off_ground_pts.astype(np.float32))
    
    for cluster_idx, cluster in enumerate(all_confirmed):
        l_id = cluster_idx + 1
        c_pts = final_cable_pts[cluster.members]
        if len(c_pts) < 10:
            continue
            
        # 规则 0：顶层地线防误杀保护
        is_topmost_ground_wire = False
        if len(tower_infos) > 0:
            max_tower_abs_z = max(t['abs_max_z'] for t in tower_infos)
            if np.max(c_pts[:, 2]) >= max_tower_abs_z - 3.5:
                is_topmost_ground_wire = True
        if is_topmost_ground_wire:
            continue
            
        # 规则 1：塔底斜坡/塔身下部树木排除区
        is_under_tower_veg = False
        if len(tower_infos) > 0:
            for t_info in tower_infos:
                d_xy = np.hypot(cluster.center[0] - t_info['cx'], cluster.center[1] - t_info['cy'])
                if d_xy < 22.0 and cluster.center[2] < t_info['abs_max_z'] - 22.0:
                    is_under_tower_veg = True
                    break
        if is_under_tower_veg:
            suspect_line_ids.add(l_id)
            continue
            
        proj = np.dot(c_pts - cluster.center, cluster.dir)
        p_min, p_max = np.min(proj), np.max(proj)
        span_len = p_max - p_min
        if span_len < 3.0 or span_len > 30.0:
            continue
            
        sample_fractions = np.linspace(0.1, 0.9, 9)
        sample_positions = cluster.center + np.outer(p_min + sample_fractions * span_len, cluster.dir)
        
        steps = np.arange(1, 13)[:, None]
        probe_offsets = np.zeros((12, 3))
        probe_offsets[:, 2] = -steps[:, 0] * 1.0
        
        all_probe_centers = (sample_positions[:, None, :] + probe_offsets[None, :, :]).reshape(-1, 3)
        probe_neighbor_lists = off_tree.query_ball_point(all_probe_centers.astype(np.float32), r=1.5)
        
        deep_probe_count = 0
        for f_idx in range(9):
            sample_pos = sample_positions[f_idx]
            consecutive_steps = 0
            max_consecutive = 0
            
            for step_idx in range(12):
                probe_flat_idx = f_idx * 12 + step_idx
                near_idx = probe_neighbor_lists[probe_flat_idx]
                
                if len(near_idx) > 0:
                    near_pts = off_ground_pts[near_idx]
                    xy_dists = np.linalg.norm(near_pts[:, :2] - sample_pos[:2], axis=1)
                    below_mask = (xy_dists <= 1.5) & (near_pts[:, 2] <= sample_pos[2] - 0.8)
                    
                    if np.any(below_mask):
                        consecutive_steps += 1
                        if consecutive_steps > max_consecutive:
                            max_consecutive = consecutive_steps
                    else:
                        consecutive_steps = 0
                else:
                    consecutive_steps = 0
                    
            if max_consecutive >= 3:
                deep_probe_count += 1
                
        if deep_probe_count >= 4:
            suspect_line_ids.add(l_id)
            
    return suspect_line_ids

def track_and_bridge_powerlines(points: np.ndarray,
                                high_pts: np.ndarray,
                                high_indices: np.ndarray,
                                final_cable_pts: np.ndarray,
                                cable_pts_idx: np.ndarray,
                                all_confirmed: List[WireCluster],
                                suspect_line_ids: Set[int],
                                tower_infos: List[Union[TowerEntity, dict]],
                                max_tracking_steps: int = 60,
                                step_size: float = 3.0,
                                use_catenary_tracking: bool = True) -> Tuple[np.ndarray, np.ndarray, Callable[[int], int]]:
    """
    3D 自适应悬链线物理轨道引导步进追踪与跨塔缝合算子
    """
    num_points = len(points)
    point_line_id = np.zeros(num_points, dtype=int)
    
    parent_line = list(range(len(all_confirmed) + 1))
    def find_line(i):
        if i <= 0 or i >= len(parent_line): return i
        root = i
        while parent_line[root] != root:
            root = parent_line[root]
        curr = i
        while curr != root:
            nxt = parent_line[curr]
            parent_line[curr] = root
            curr = nxt
        return root
    def union_line(i, j):
        if i <= 0 or j <= 0 or i >= len(parent_line) or j >= len(parent_line): return
        ri, rj = find_line(i), find_line(j)
        if ri != rj: parent_line[ri] = rj

    valid_refined_cable_idx_local = []
    for cluster_idx, cluster in enumerate(all_confirmed):
        l_id = cluster_idx + 1
        if l_id not in suspect_line_ids:
            valid_refined_cable_idx_local.extend(cluster.members)
            
    refined_cable_idx_local = valid_refined_cable_idx_local
    extra_cable_indices = []

    if len(refined_cable_idx_local) > 0:
        all_high_tree = cKDTree(high_pts)
        
        for cluster_idx, cluster in enumerate(all_confirmed):
            line_id = cluster_idx + 1
            if line_id in suspect_line_ids:
                continue
                
            member_local = cluster.members
            member_global = cable_pts_idx[member_local]
            point_line_id[member_global] = line_id
            
            pts = final_cable_pts[cluster.members]
            if len(pts) < 2: 
                continue
                
            proj = np.dot(pts - cluster.center, cluster.dir)
            end1 = pts[np.argmin(proj)]
            end2 = pts[np.argmax(proj)]
            
            cat_model: Optional[CatenaryModel] = cluster.catenary if use_catenary_tracking else None
            
            for start_pt, init_dir in [(end1, -cluster.dir), (end2, cluster.dir)]:
                curr_pt = start_pt
                curr_dir = init_dir
                jumped_towers = set()
                tracked_pts_history = [curr_pt]
                
                for step in range(max_tracking_steps):
                    if cat_model is not None:
                        s_curr = float(cat_model.project_to_s(curr_pt[None, :])[0])
                        dir_sign = 1.0 if np.dot(curr_dir[:2], cat_model.direction_2d) > 0 else -1.0
                        s_next = s_curr + dir_sign * step_size
                        cat_tan = cat_model.get_tangent_3d(s_next) * dir_sign
                        curr_dir = 0.6 * curr_dir + 0.4 * cat_tan
                        norm_d = np.linalg.norm(curr_dir)
                        if norm_d > 1e-3:
                            curr_dir /= norm_d
                            
                    probe_pos = curr_pt + step_size * curr_dir
                    max_proj_limit = 14.0
                    z_diff_limit = 2.5
                    dists_limit = 1.2
                    
                    if len(tower_infos) > 0:
                        d2d_towers = [np.hypot(probe_pos[0] - t['cx'], probe_pos[1] - t['cy']) for t in tower_infos]
                        min_t_idx = np.argmin(d2d_towers)
                        min_t_dist = d2d_towers[min_t_idx]
                        t_info = tower_infos[min_t_idx]
                        
                        if min_t_dist < 8.0:
                            curr_t_dist = np.hypot(curr_pt[0] - t_info['cx'], curr_pt[1] - t_info['cy'])
                            probe_t_dist = np.hypot(probe_pos[0] - t_info['cx'], probe_pos[1] - t_info['cy'])
                            is_moving_towards_tower = probe_t_dist < curr_t_dist
                            
                            if is_moving_towards_tower:
                                if min_t_idx not in jumped_towers:
                                    if curr_pt[2] >= t_info['abs_max_z'] - 25.0:
                                        cx, cy = t_info['cx'], t_info['cy']
                                        half_d = t_info.get('d_diag', 12.0) / 2.0
                                        r_search_max = max(half_d + 4.0, 8.0)
                                        
                                        near_t_indices = all_high_tree.query_ball_point([cx, cy, curr_pt[2]], r=r_search_max)
                                        if len(near_t_indices) > 0:
                                            near_t_pts = high_pts[near_t_indices]
                                            
                                            z_diffs_cyl = np.abs(near_t_pts[:, 2] - curr_pt[2])
                                            vec_from_cxcy = near_t_pts[:, :2] - np.array([cx, cy])
                                            r_2d = np.linalg.norm(vec_from_cxcy, axis=1)
                                            
                                            dir_2d = curr_dir[:2]
                                            norm_dir = np.linalg.norm(dir_2d)
                                            if norm_dir > 1e-3: dir_2d = dir_2d / norm_dir
                                            dot_proj = np.dot(vec_from_cxcy, dir_2d)
                                            
                                            side_vec = np.array([-dir_2d[1], dir_2d[0]])
                                            curr_side = np.dot(curr_pt[:2] - np.array([cx, cy]), side_vec)
                                            cand_sides = np.dot(vec_from_cxcy, side_vec)
                                            side_diffs = np.abs(cand_sides - curr_side)
                                            
                                            out_mask = (z_diffs_cyl <= 2.0) & (r_2d >= 1.5) & (r_2d <= r_search_max) & \
                                                       (dot_proj > 0.3) & (side_diffs <= 2.5)
                                            valid_out_idx = np.array(near_t_indices)[out_mask]
                                            
                                            if len(valid_out_idx) > 0:
                                                valid_out_pts = high_pts[valid_out_idx]
                                                best_out_sub_idx = np.argmax(dot_proj[out_mask])
                                                next_pt = valid_out_pts[best_out_sub_idx]
                                                
                                                new_dir = next_pt - curr_pt
                                                norm_new = np.linalg.norm(new_dir)
                                                if norm_new > 1e-3:
                                                    curr_dir = 0.5 * curr_dir + 0.5 * (new_dir / norm_new)
                                                    curr_dir /= np.linalg.norm(curr_dir)
                                                curr_pt = next_pt
                                                
                                                extra_cable_indices.extend(valid_out_idx.tolist())
                                                extra_global_pts = high_indices[valid_out_idx]
                                                
                                                for g_pt in extra_global_pts:
                                                    old_id = point_line_id[g_pt]
                                                    if old_id > 0 and old_id != line_id:
                                                        union_line(line_id, old_id)
                                                    point_line_id[g_pt] = line_id
                                                    
                                                jumped_towers.add(min_t_idx)
                                                continue
                                        
                                        probe_pos = curr_pt + (half_d * 2.0 + 2.0) * curr_dir
                                        max_proj_limit = half_d * 2.0 + 5.0
                                        z_diff_limit = 4.5
                                        dists_limit = 3.0
                                        jumped_towers.add(min_t_idx)
                                    else:
                                        break
                                
                    search_r = 6.0 if dists_limit > 2.0 else 4.5
                    idx_near = all_high_tree.query_ball_point(probe_pos, r=search_r)
                    
                    if len(idx_near) > 0:
                        near_pts = high_pts[idx_near]
                        vecs = near_pts - curr_pt
                        proj_len = np.dot(vecs, curr_dir)
                        dists = np.linalg.norm(vecs - np.outer(proj_len, curr_dir), axis=1)
                        z_diffs = np.abs(near_pts[:, 2] - curr_pt[2])
                        
                        valid_mask = (dists < dists_limit) & (proj_len > 0.1) & (proj_len < max_proj_limit) & (z_diffs <= z_diff_limit)
                        valid_idx = np.array(idx_near)[valid_mask]
                        
                        if len(valid_idx) > 0:
                            if len(valid_idx) >= 4:
                                std_dist = np.std(dists[valid_mask])
                                if std_dist > 1.0:
                                    break
                                    
                            extra_cable_indices.extend(valid_idx.tolist())
                            extra_global_pts = high_indices[valid_idx]
                            point_line_id[extra_global_pts] = line_id
                            
                            valid_pts = high_pts[valid_idx]
                            max_proj_idx = np.argmax(proj_len[valid_mask])
                            next_pt = valid_pts[max_proj_idx]
                            
                            new_dir = next_pt - curr_pt
                            norm_new = np.linalg.norm(new_dir)
                            if norm_new > 1e-3:
                                new_dir /= norm_new
                                curr_dir = 0.5 * curr_dir + 0.5 * new_dir
                                curr_dir /= np.linalg.norm(curr_dir)
                            
                            curr_pt = next_pt
                            tracked_pts_history.append(curr_pt)
                            
                            if len(tracked_pts_history) >= 15 and len(tracked_pts_history) % 10 == 0:
                                new_cat = fit_catenary_3d(np.array(tracked_pts_history))
                                if new_cat is not None:
                                    cat_model = new_cat
                        else:
                            break
                    else:
                        break
                        
        final_cable_indices = cable_pts_idx[refined_cable_idx_local].tolist()
        if len(extra_cable_indices) > 0:
            extra_global = high_indices[extra_cable_indices]
            final_cable_indices.extend(extra_global.tolist())
            
        final_cable_pts_idx = np.unique(final_cable_indices)
    else:
        final_cable_pts_idx = np.array([], dtype=int)

    return final_cable_pts_idx, point_line_id, find_line
