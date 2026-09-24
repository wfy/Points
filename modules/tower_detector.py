import numpy as np
from scipy.spatial import cKDTree
from typing import List, Tuple, Optional
from collections import deque
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

def extract_waist_orthogonal_axes(
    waist_pts_2d: np.ndarray,
    deg_step: float = 0.5
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """
    基于 2D MinArea-OBB (最小外接矩形面积旋转扫描) 提取塔腰纯净截面的严格物理正交基底对 (u_a, u_b)。
    对于正方形截面，外接包络面积在四边与旋转轴平行时取全局极小值 L^2，在 45° 对角线时取极大值 2L^2。
    在 [0, 90°) 范围内以 deg_step 步长扫描，不受世界绝对坐标系或象限先验的束缚，满足各向同性。

    参数:
        waist_pts_2d: shape (N, 2) 的塔腰 2D 点坐标
        deg_step: 角度搜索步长，默认 0.5 度
    返回:
        (u_a, u_b): 相互严格正交的单位方向向量对 (shape: (2,))，若点数不足 (N < 8) 则返回 None
    """
    if waist_pts_2d is None or len(waist_pts_2d) < 8:
        return None

    deg = np.arange(0.0, 90.0, deg_step)
    rad = np.radians(deg)
    cos_t = np.cos(rad)
    sin_t = np.sin(rad)
    
    # 构造旋转基底矩阵: U1 对应角度 theta 的主轴, U2 对应垂直轴
    U1 = np.vstack([cos_t, sin_t])       # shape (2, len(deg))
    U2 = np.vstack([-sin_t, cos_t])      # shape (2, len(deg))
    
    # 批量投影计算两轴展宽 (使用 2% 与 98% 分位数排除离群噪点)
    proj1 = waist_pts_2d @ U1            # shape (N, len(deg))
    proj2 = waist_pts_2d @ U2            # shape (N, len(deg))
    
    w1 = np.percentile(proj1, 98.0, axis=0) - np.percentile(proj1, 2.0, axis=0)
    w2 = np.percentile(proj2, 98.0, axis=0) - np.percentile(proj2, 2.0, axis=0)
    
    areas = w1 * w2
    best_idx = int(np.argmin(areas))
    
    u_a = np.array([cos_t[best_idx], sin_t[best_idx]])
    u_b = np.array([-sin_t[best_idx], cos_t[best_idx]])
    
    return u_a, u_b

def cluster_tower_voxels(off_ground_pts: np.ndarray, 
                         rel_z: np.ndarray, 
                         t_grid_size: float = 2.0,
                         min_tower_rel_z: float = 22.0,
                         min_pts_count: int = 500,
                         continuity_ratio: float = 0.65,
                         nms_radius: float = 14.0) -> List[dict]:
    """
    基于 3D 体素垂直连续性与 2D 连通域图搜索初筛杆塔候选聚类中心
    【高精度演进 (CC D11)】：
      1. 绝对高程 + 高位金属骨架点数加权：杜绝坡地下沉处架空线以相对净高优势向林区磁吸漂移塔心；
      2. 兼容单格高耸塔身 (len(members) >= 1)；
      3. 物理级 14m NMS 截断与 Score = N_pts * sqrt(max_z) 综合评分优胜仲裁。
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
    sub_abs_z = off_ground_pts[mask_pts_in_valid, 2]
    
    grid_heights = {}
    grid_abs_max = {}
    for g_i in valid_grid_idx:
        m = (sub_inv == g_i)
        gz_layers = sub_z[m]
        if len(gz_layers) == 0:
            continue
        max_h = int(np.max(gz_layers))
        num_layers = len(np.unique(gz_layers))
        if max_h >= min_h_layer and (num_layers / (max_h + 1)) >= continuity_ratio:
            candidate_tower_indices.append(g_i)
            grid_heights[g_i] = max_h
            grid_abs_max[g_i] = float(np.max(sub_abs_z[m]))
            
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
                # 统计聚类簇最高绝对高程
                cluster_abs_max_z = max(grid_abs_max[candidate_tower_indices[m]] for m in members)
                
                # 统计高位（顶部 6.0m 内）骨架点数作为重心权重，彻底压制下坡低密导线与树木
                weights = []
                coords = []
                for m in members:
                    g_i = candidate_tower_indices[m]
                    gx, gy = cand_arr[m]
                    gcx = (gx + 0.5) * t_grid_size
                    gcy = (gy + 0.5) * t_grid_size
                    coords.append([gcx, gcy])
                    
                    m_pts = (sub_inv == g_i)
                    high_pts = np.sum(m_pts & (sub_abs_z >= (cluster_abs_max_z - 6.0)))
                    weights.append(high_pts)
                    
                weights = np.array(weights, dtype=np.float64)
                coords = np.array(coords, dtype=np.float64)
                
                if np.sum(weights) > 50:
                    w_thresh = np.max(weights) * 0.05
                    valid_w_mask = weights >= w_thresh
                    sub_w = weights[valid_w_mask]
                    sub_c = coords[valid_w_mask]
                    cx = float(np.sum(sub_c[:, 0] * sub_w) / np.sum(sub_w))
                    cy = float(np.sum(sub_c[:, 1] * sub_w) / np.sum(sub_w))
                else:
                    all_w = np.array([grid_pts_counts[candidate_tower_indices[m]] for m in members], dtype=np.float64)
                    cx = float(np.sum(coords[:, 0] * all_w) / np.sum(all_w))
                    cy = float(np.sum(coords[:, 1] * all_w) / np.sum(all_w))
                    
                # 锁定加权中心最近的核心网格真实高度
                d_to_c = np.hypot(coords[:, 0] - cx, coords[:, 1] - cy)
                best_m = members[int(np.argmin(d_to_c))]
                best_gi = candidate_tower_indices[best_m]
                core_max_z = float(grid_heights.get(best_gi, 0) * 2.0)
                
                total_cluster_pts = sum(grid_pts_counts[candidate_tower_indices[m]] for m in members)
                score = float(total_cluster_pts * np.sqrt(core_max_z))
                
                raw_tower_infos.append({
                    'cx': cx, 'cy': cy, 'max_z': core_max_z,
                    'abs_max_z': cluster_abs_max_z,
                    'score': score,
                    'total_pts': total_cluster_pts
                })
                
    # 物理 NMS 截断 (默认 <= 14.0m，按 Score 优胜仲裁，符合 CC D11)
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
            best_member = max(members, key=lambda m: raw_tower_infos[m]['score'])
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
        continuity_ratio=t_cfg.continuity_ratio,
        nms_radius=min(getattr(t_cfg, 'nms_radius', 14.0), 14.0)
    )
    
    is_tower = np.zeros(len(off_ground_pts), dtype=bool)
    is_tower_arm = np.zeros(len(off_ground_pts), dtype=bool)
    is_near_tower_high_arm = np.zeros(len(off_ground_pts), dtype=bool)
    valid_tower_entities: List[TowerEntity] = []
    
    if len(tower_candidates) > 0:
        # 【走廊走向与横担正交基准 (Corridor Span & Arm Orthogonal Reference)】
        # 在输电线路巡检中，走廊走向 u_span 与横担法向 u_arm 具有强先验几何正交性。
        # 优先从初筛铁塔候选点集 (>= 2 座高塔) 的空间分布主轴中提取走廊先验基准：
        u_span_prior = None
        u_arm_prior = None
        high_cands = [c for c in tower_candidates if c.get('max_z', 0.0) >= 20.0]
        if len(high_cands) >= 2:
            # 优先从相距 >= 45m 的真正跨档杆塔对提取走廊轴向，杜绝同一座铁塔旁的孤立树冠候选干扰走廊走向
            max_dist = 0.0
            best_pair = None
            for i in range(len(high_cands)):
                for j in range(i + 1, len(high_cands)):
                    d = np.hypot(high_cands[i]['cx'] - high_cands[j]['cx'], high_cands[i]['cy'] - high_cands[j]['cy'])
                    if d > max_dist:
                        max_dist = d
                        best_pair = (high_cands[i], high_cands[j])
            if best_pair is not None and max_dist >= 45.0:
                c1, c2 = best_pair
                span_vec = np.array([c2['cx'] - c1['cx'], c2['cy'] - c1['cy']])
                u_span_prior = span_vec / np.linalg.norm(span_vec)
                u_arm_prior = np.array([-u_span_prior[1], u_span_prior[0]])
            else:
                cand_xy = np.array([[c['cx'], c['cy']] for c in high_cands])
                span_vec = cand_xy[1] - cand_xy[0]
                norm_span = np.linalg.norm(span_vec)
                if norm_span > 1e-3:
                    u_span_prior = span_vec / norm_span
                    u_arm_prior = np.array([-u_span_prior[1], u_span_prior[0]])

        off_ground_tree = cKDTree(off_ground_pts[:, :2])
        for cand in tower_candidates:
            cx, cy, max_z = cand['cx'], cand['cy'], cand['max_z']
            
            r_search = min(max(max_z * 0.45, 14.0), 30.0)
            indices = off_ground_tree.query_ball_point([cx, cy], r=r_search)
            
            local_pts = off_ground_pts[indices]
            local_rel_z = rel_z[indices]
            abs_max_z = float(cand.get('abs_max_z', 0.0))
            relief_margin = float(getattr(t_cfg, 'delta_h_relief', 8.0) * 2.0)
            max_top_allowance = 8.0 + max(relief_margin, 12.0)
            if abs_max_z > 0.0:
                # 【ADR 0006 & Ticket 02: 陡坡塔顶绝对高程解耦防护 (Slope-Adaptive Absolute Z Gate)】
                # 容纳猫头塔羊角高出塔身中轴(可达6~8m)以及山地陡坡下陷引起的相对高程膨胀(可达15m)
                valid_mask = (local_pts[:, 2] <= (abs_max_z + 8.5)) & (local_rel_z <= (max_z + max_top_allowance))
            else:
                valid_mask = local_rel_z <= (max_z + max_top_allowance)
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

            # 初始中心校准
            waist_mask = (tower_z >= max_z * 0.45) & (tower_z <= max_z * 0.85) & (np.hypot(tower_pts[:, 0] - cx, tower_pts[:, 1] - cy) <= 8.0)
            if np.sum(waist_mask) >= 30:
                w_pts = tower_pts[waist_mask]
                cx = float((np.percentile(w_pts[:, 0], 2) + np.percentile(w_pts[:, 0], 98)) / 2.0)
                cy = float((np.percentile(w_pts[:, 1], 2) + np.percentile(w_pts[:, 1], 98)) / 2.0)

            diff_all_tower = tower_pts[:, :2] - np.array([cx, cy])
            dist_all_tower = np.hypot(diff_all_tower[:, 0], diff_all_tower[:, 1])

            # 【ADR 0006, Ticket 01 & 02: 跨档走向参考向量 v_span】
            # 从 candidate clusters 中查找相距 >= 45m 的相邻高塔（代表真实跨档走廊走向）
            other_cands = [c for c in tower_candidates if np.hypot(c['cx'] - cx, c['cy'] - cy) >= 45.0]
            if len(other_cands) > 0:
                dists_other = [np.hypot(c['cx'] - cx, c['cy'] - cy) for c in other_cands]
                nearest_cand = other_cands[int(np.argmin(dists_other))]
                v_span = np.array([nearest_cand['cx'] - cx, nearest_cand['cy'] - cy])
                v_span /= np.linalg.norm(v_span)
            elif u_span_prior is not None:
                v_span = u_span_prior
            else:
                v_span = None

            # 拟合高位金属横担直线特征 (RANSAC)
            half_arm_max = min(max(max_z * 0.32, 6.0), 16.5)
            steel_z_min, steel_z_max = max_z * 0.70, max_z * 0.98
            lattice_arm_mask = (tower_z >= steel_z_min) & (tower_z <= steel_z_max) & (dist_all_tower <= min(half_arm_max * 0.65, 9.5))
            lattice_arm_pts = tower_pts[lattice_arm_mask]

            v_arm, inlier_count = fit_arm_ransac_2d(
                lattice_arm_pts[:, :2] - np.array([cx, cy]),
                max_trials=t_cfg.arm_ransac_trials,
                inlier_thresh=t_cfg.arm_ransac_inlier_thresh
            )

            if v_arm is not None and inlier_count >= 8:
                v_a = v_arm
                v_b = np.array([-v_a[1], v_a[0]])
                if v_span is not None:
                    # 走廊正交门禁：真实高压横担必须与跨档走廊垂直 (|v_arm · v_span| <= 0.40)
                    # 若 RANSAC 拟合到的是架空导线，其与 v_span 平行 (|v_a · v_span| > 0.60)，
                    # 则垂直于走廊的法向 v_b 才是真实横担轴向
                    dot_span_a = abs(float(v_a @ v_span))
                    dot_span_b = abs(float(v_b @ v_span))
                    if dot_span_a <= dot_span_b:
                        v1, v2 = v_a, v_b
                    else:
                        v1, v2 = v_b, v_a
                else:
                    d_a = np.abs((lattice_arm_pts[:, :2] - np.array([cx, cy])) @ v_a)
                    d_b = np.abs((lattice_arm_pts[:, :2] - np.array([cx, cy])) @ v_b)
                    if np.percentile(d_b, 98) > np.percentile(d_a, 98):
                        v1, v2 = v_b, v_a
                    else:
                        v1, v2 = v_a, v_b
            else:
                high_arm_zone_temp = tower_z >= (max_z * 0.40)
                arm_pts_temp = tower_pts[high_arm_zone_temp]
                if v_span is not None:
                    # 若无 RANSAC 结果，优先采用与跨档走廊垂直的方向作为初始横担轴 v1
                    v1 = np.array([-v_span[1], v_span[0]])
                    v2 = v_span
                elif len(arm_pts_temp) >= 10:
                    cov_arm = np.cov(arm_pts_temp[:, :2].T)
                    evals_a, evecs_a = np.linalg.eigh(cov_arm)
                    v1 = evecs_a[:, 1]
                    v2 = evecs_a[:, 0]
                else:
                    v1, v2 = np.array([1.0, 0.0]), np.array([0.0, 1.0])

            d_v1 = np.abs(diff_all_tower @ v1)
            d_v2 = np.abs(diff_all_tower @ v2)

            # 【ADR 0006 & Ticket 02: 输电网横担高程硬门禁与横担/跳线几何先验约束】
            # 1. 高压输电塔横担绝不可能悬挂在塔身下部 50% 甚至触地位置，最低搜索高度必须 >= 0.60 * max_z (配网单双杆除外)
            min_arm_ratio = 0.30 if getattr(t_cfg, 'allow_distribution_poles', False) else 0.60
            min_arm_floor = 6.0 if getattr(t_cfg, 'allow_distribution_poles', False) else 15.0
            min_search_z = max(max_z * min_arm_ratio, min_arm_floor)
            max_search_z = max_z * 0.98
            z_slice_step = 0.6
            slice_centers = np.arange(min_search_z, max_search_z, z_slice_step)
            raw_candidates_z = []

            r_trunk_search = min(max(0.06 * max_z, 2.5), 3.8)
            waist_est_m = (tower_z >= max_z * 0.30) & (tower_z <= max_z * 0.50) & (d_v1 <= r_trunk_search) & (d_v2 <= r_trunk_search)
            w_trunk_est = float(np.percentile(d_v1[waist_est_m], 90)) if np.sum(waist_est_m) >= 10 else min(max(0.045 * max_z, 1.8), 3.5)

            min_arm_aspect = 1.3 if getattr(t_cfg, 'allow_distribution_poles', False) else 2.3
            min_arm_span = 2.5 if getattr(t_cfg, 'allow_distribution_poles', False) else max(max_z * 0.10, 4.0)

            beam_slices = []
            jumper_slices = []

            r_arm_eval = min(half_arm_max * 1.15, 15.0)
            for z_c in slice_centers:
                slice_mask = (tower_z >= z_c - z_slice_step * 0.6) & (tower_z <= z_c + z_slice_step * 0.6) & (dist_all_tower <= r_arm_eval)
                if np.sum(slice_mask) < 6:
                    continue
                
                d_v1_slice = d_v1[slice_mask]
                d_v2_slice = d_v2[slice_mask]
                
                w1 = float(np.percentile(d_v1_slice, 95))
                w2 = float(np.percentile(d_v2_slice, 95))
                w_span = max(w1, w2)
                w_thick = min(w1, w2)
                outer_pts_count = np.sum((d_v1_slice >= 4.0) | (d_v2_slice >= 4.0))

                # 真实横担角钢必须显著沿横担轴向 v1 展开 (w1 明显大于躯干且相对顺线厚度具备清晰长宽比)
                max_allowed_w1 = min(half_arm_max * 1.15, 18.0)
                max_allowed_w2 = max(0.08 * max_z, 3.8)
                is_beam = (w1 >= max(w_trunk_est + 2.0, min_arm_span)) and (w1 <= max_allowed_w1) and (w2 <= max_allowed_w2) and (w1 >= min_arm_aspect * max(w2, 0.5)) and (outer_pts_count >= 10)
                # 耐张跳线仅存在于高位横担下方附近区间 (>= 0.65 * max_z)，绝不可能悬垂在接近地面的低空树冠处
                is_tension_jumper = (w1 >= 5.5) and (w2 >= 5.5) and (outer_pts_count >= 50) and (w_span >= max(w_trunk_est + 3.0, 7.0)) and (z_c >= max_z * 0.65)
                
                if is_beam:
                    beam_slices.append(z_c)
                elif is_tension_jumper:
                    jumper_slices.append(z_c)

            # 【Ticket 02 准入条件：真实高压铁塔严禁“仅有跳线而无横担横梁”】
            # 耐张跳线必须依附于横担梁附近区间，禁止孤立圆形树冠仅凭各向同性厚度直接注册为横担候选
            if len(beam_slices) >= 1:
                beam_min_z = min(beam_slices)
                beam_max_z = max(beam_slices)
                valid_jumpers = [
                    jz for jz in jumper_slices 
                    if (beam_min_z - 4.5) <= jz <= (beam_max_z + 1.2)
                ]
                raw_candidates_z = sorted(beam_slices + valid_jumpers)
            else:
                raw_candidates_z = []

            max_allowed_gap = min(max(0.10 * max_z, 3.5), 4.5)
            if len(raw_candidates_z) > 0:
                sorted_cand = sorted(raw_candidates_z, reverse=True)
                valid_arm_cluster = [sorted_cand[0]]
                for c_z in sorted_cand[1:]:
                    if (valid_arm_cluster[-1] - c_z) <= max_allowed_gap:
                        valid_arm_cluster.append(c_z)
                    else:
                        break
                crossarm_candidate_z = valid_arm_cluster
            else:
                crossarm_candidate_z = []

            is_robust_lattice = (max_cc_z_span >= (max_z * 0.80)) and (len(tower_z) >= 1000) and (max_z >= 22.0)
            
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

            max_slope_rate = np.clip(slope0, 0.045, 0.075)
            shell_thick = 1.3 * np.clip(k_height, 0.8, 1.2)
            margin_val = min(margin0, 0.45)
            
            # 【ADR 0005, 0006 & Ticket 01, 02: 塔身分界高程基准与无横担兜底收紧】
            min_waist_floor = max(h_true * 0.30, 6.0)
            if len(crossarm_candidate_z) == 0:
                if not is_robust_lattice or h_true < 22.0:
                    continue
                # 【Ticket 02 & 03: 无横担兜底校验】真实高压铁塔若无横担，仅能为细长立柱（半径 <= 3.8m）
                # 严禁在树冠高位区间 (z >= 0.50 * max_z) 存在宽大膨胀球冠 (半径 > 6.0m)
                mid_canopy_pts = np.sum((dist_all_tower > 6.0) & (tower_z >= max_z * 0.50))
                if mid_canopy_pts >= 500:
                    continue
                arm_ratio = float(np.clip(0.55 - 0.03 * (k_height - 1.0), 0.45, 0.60))
                waist_boundary_z = max(h_true * arm_ratio, min_waist_floor)
                z_lowest_arm = waist_boundary_z + 1.2
            else:
                z_lowest_arm = float(min(crossarm_candidate_z))
                waist_boundary_z = max(z_lowest_arm - 1.2, min_waist_floor)

            # 【阶段二核心优化：纯净塔身多层切片几何轴心自校准 (Pure Trunk Multi-Slice Centroid Recalibration)】
            # 彻底摒弃在包含悬臂横担与非对称导线的高位区间粗暴估算中心；
            # 严格在最下方横担以下、地面以上的纯净塔腰区间采样。
            # 此区间绝对没有悬臂横担，也没有架空导线，四根主材角钢在空间中构成严格对称的几何四棱台。
            # 逐层提取外缘几何中点并取中位数，锁定毫米级精度的铁塔真正物理对称中轴！
            trunk_z_min = max(z_lowest_arm - 8.0, max(z_lowest_arm * 0.55, 6.0))
            trunk_z_max = max(z_lowest_arm - 1.0, 7.0)
            if trunk_z_max <= trunk_z_min + 1.0:
                trunk_z_min = max(z_lowest_arm * 0.40, 3.0)
                trunk_z_max = max(z_lowest_arm * 0.85, 5.0)

            r_trunk_cyl = min(max(h_true * 0.12, 5.5), 8.5)
            centers_x, centers_y = [], []
            for z_s in np.arange(trunk_z_min, trunk_z_max, 0.8):
                sl_m = (tower_z >= z_s) & (tower_z < z_s + 0.8) & (np.hypot(tower_pts[:, 0] - cx, tower_pts[:, 1] - cy) <= r_trunk_cyl)
                if np.sum(sl_m) >= 15:
                    sub_x = tower_pts[sl_m, 0]
                    sub_y = tower_pts[sl_m, 1]
                    cx_s = (np.percentile(sub_x, 2) + np.percentile(sub_x, 98)) / 2.0
                    cy_s = (np.percentile(sub_y, 2) + np.percentile(sub_y, 98)) / 2.0
                    centers_x.append(cx_s)
                    centers_y.append(cy_s)

            if len(centers_x) >= 3:
                cx = float(np.median(centers_x))
                cy = float(np.median(centers_y))

            # 依据精准物理中轴重新度量全塔横担与顺线走廊距离
            diff_all_tower = tower_pts[:, :2] - np.array([cx, cy])

            # 【ADR 0006, Ticket 01: 跨档输电线路走向向量 v_span】
            # 从 candidate clusters 中查找相距 >= 45m 的相邻杆塔（代表真实跨档走廊走向）
            other_cands = [c for c in tower_candidates if np.hypot(c['cx'] - cx, c['cy'] - cy) >= 45.0]
            if len(other_cands) > 0:
                dists_other = [np.hypot(c['cx'] - cx, c['cy'] - cy) for c in other_cands]
                nearest_cand = other_cands[int(np.argmin(dists_other))]
                v_span = np.array([nearest_cand['cx'] - cx, nearest_cand['cy'] - cy])
                v_span /= np.linalg.norm(v_span)
            elif u_span_prior is not None:
                v_span = u_span_prior
            else:
                v_span = None

            # 【ADR 0006 & Ticket 01: 塔腰纯净截面正交定姿与走廊先验防倒挂仲裁】
            # 1. 提取蓝黄分割线下方纯净塔腰截面点云 [z_lowest_arm - 2.2, z_lowest_arm - 0.4]，近轴限径 3.8m，避免混入低空斜坡树冠
            waist_slice_m = (tower_z >= max(z_lowest_arm - 2.2, 0.0)) & (tower_z <= max(z_lowest_arm - 0.4, 0.0)) & (np.hypot(diff_all_tower[:, 0], diff_all_tower[:, 1]) <= 3.8)
            if np.sum(waist_slice_m) < 15:
                waist_slice_m = (tower_z >= max(z_lowest_arm - 3.0, 0.0)) & (tower_z <= max(z_lowest_arm - 0.2, 0.0)) & (np.hypot(diff_all_tower[:, 0], diff_all_tower[:, 1]) <= 4.5)

            waist_pts_raw = diff_all_tower[waist_slice_m]
            waist_axes = extract_waist_orthogonal_axes(waist_pts_raw, deg_step=0.5)
            if waist_axes is not None:
                u_a, u_b = waist_axes
                # 仲裁横担主轴 v1 与顺线走廊轴 v2：
                # 优先跨档先验准则：若存在相邻杆塔跨档走向 v_span，横担轴必与跨档垂直（内积更小者锁定为横担主轴 v1）
                if v_span is not None:
                    dot_span_a = abs(float(u_a @ v_span))
                    dot_span_b = abs(float(u_b @ v_span))
                    if dot_span_a <= dot_span_b:
                        v1 = u_a
                        v2 = np.array([-v1[1], v1[0]])
                    else:
                        v1 = u_b
                        v2 = np.array([-v1[1], v1[0]])
                else:
                    ref_arm = v_arm if (v_arm is not None and inlier_count >= 8) else v1
                    dot_a = abs(float(u_a @ ref_arm))
                    dot_b = abs(float(u_b @ ref_arm))
                    if dot_a >= dot_b:
                        v1 = u_a
                        v2 = np.array([-v1[1], v1[0]])
                    else:
                        v1 = u_b
                        v2 = np.array([-v1[1], v1[0]])

                # 【Ticket 01: 塔腰正交对称中轴再校准】
                # 在锁定严格正交基底 (v1, v2) 后，在纯净塔腰截面内重新度量物理对称中心
                # 彻底消除低位单侧茂密树冠对中轴造成的漂移 (解决 0-1# 图 2 中轴偏心缺陷)
                p1_w = waist_pts_raw @ v1
                p2_w = waist_pts_raw @ v2
                shift_v1 = (float(np.percentile(p1_w, 98.0)) + float(np.percentile(p1_w, 2.0))) / 2.0
                shift_v2 = (float(np.percentile(p2_w, 98.0)) + float(np.percentile(p2_w, 2.0))) / 2.0
                cx += float(shift_v1 * v1[0] + shift_v2 * v2[0])
                cy += float(shift_v1 * v1[1] + shift_v2 * v2[1])

            # 依据全新物理中轴与正交主轴重新度量全塔横担与顺线走廊距离
            diff_all_tower = tower_pts[:, :2] - np.array([cx, cy])
            d_v1 = np.abs(diff_all_tower @ v1)
            d_v2 = np.abs(diff_all_tower @ v2)

            # 【Ticket 03: 塔身腰部收窄与树冠扩散比过滤 (Waist Necking & Canopy Spread Filter)】
            # 严格在最下横担下方 [z_lowest_arm - 3.0, z_lowest_arm - 0.4] 纯净立柱区间采样
            # 真实铁塔在腰身区间仅包含四根主立柱 (r <= 3.8m)，外部空间 (3.8m < r <= 15m) 空旷无点；
            # 档中山坡大树林呈各向同性实心球冠向外大幅膨胀，外缘扩散点数远超阈值，坚决予以剔除
            waist_eval_m = (tower_z >= max(z_lowest_arm - 3.0, 0.0)) & (tower_z <= max(z_lowest_arm - 0.4, 0.0))
            if np.sum(waist_eval_m) >= 20:
                dist_waist_eval = np.hypot(diff_all_tower[waist_eval_m, 0], diff_all_tower[waist_eval_m, 1])
                inner_waist = np.sum(dist_waist_eval <= 3.8)
                outer_waist = np.sum((dist_waist_eval > 3.8) & (dist_waist_eval <= 15.0))
                if outer_waist >= 500 and outer_waist > 0.60 * inner_waist:
                    continue

            # 【阶段二优化 2：精准测量塔身纯立柱半宽 (W_trunk)】
            # 严格在最下横担下方 [z_lowest_arm - 2.2, z_lowest_arm - 0.4] 纯立柱区间测量，近轴限制 d <= 3.8m
            # 避免窗口下延过大侵入山坡低空树冠，并施加物理几何上限 (w_waist_cap)
            w_waist_cap = max(min(0.045 * h_true + 0.6, 3.2), 1.8)
            waist_band = (tower_z >= max(z_lowest_arm - 2.2, 0.0)) & (tower_z <= max(z_lowest_arm - 0.4, 0.0)) & (d_v1 <= 3.8) & (d_v2 <= 3.8)
            if np.sum(waist_band) >= 10:
                w1_trunk = min(float(np.percentile(d_v1[waist_band], 98)), w_waist_cap)
                w2_trunk = min(float(np.percentile(d_v2[waist_band], 98)), w_waist_cap)
            else:
                clean_trunk_m = (tower_z >= max(z_lowest_arm - 3.0, 0.0)) & (tower_z <= max(z_lowest_arm - 0.2, 0.0)) & (d_v1 <= 3.8) & (d_v2 <= 3.8)
                if np.sum(clean_trunk_m) >= 5:
                    w1_trunk = min(float(np.percentile(d_v1[clean_trunk_m], 98)), w_waist_cap)
                    w2_trunk = min(float(np.percentile(d_v2[clean_trunk_m], 98)), w_waist_cap)
                else:
                    w1_trunk, w2_trunk = 1.6 * np.sqrt(k_height), 1.6 * np.sqrt(k_height)
            w_waist = max(w1_trunk, w2_trunk)
            
            # 【ADR 0005, 0006 & Ticket 02: 上半部定向矩形包围盒 (UpperTowerBox) 自适应分路】
            # 耐张塔大弧垂跳线常下垂至最下横担下方 1.5m~2.5m
            high_cand_m = tower_z >= max(z_lowest_arm - 2.5, 0.0)
            d2_cand = d_v2[high_cand_m] if np.any(high_cand_m) else np.array([])
            is_tension_tower = (len(d2_cand) >= 20) and (float(np.percentile(d2_cand, 95.0)) >= 3.8) and (np.sum(d2_cand >= 3.8) >= 20)

            z_upper_floor = max(z_lowest_arm - 2.5, 0.0) if is_tension_tower else waist_boundary_z
            high_arm_zone = tower_z >= z_upper_floor
            arm_pts = tower_pts[high_arm_zone]

            if len(arm_pts) >= 5:
                d1_high = d_v1[high_arm_zone]
                d2_high = d_v2[high_arm_zone]
                # 紧致横担半宽：采用 99 分位数加 0.35m 安全安装公差，取消 18% 过度外推
                half_arm_w = min(max(np.percentile(d1_high, 99.0) + 0.35, 5.5), half_arm_max * 1.10)
                
                # 【Ticket 02: 动态识别耐张塔与直线塔，放开耐张塔跳线/耐张串厚度】
                p95_v2 = float(np.percentile(d2_high, 95.0))
                p98_v2 = float(np.percentile(d2_high, 98.0))

                insulator_margin = min(0.18 * half_arm_w, 0.08 * h_true)
                if is_tension_tower:
                    # 耐张塔放开顺线厚度上限至 5.5m~6.0m，完整包含大弧垂跳线与耐张绝缘子串，同时严密阻断 >= 7.0m 跨中导线
                    max_ceiling = min(max(0.12 * h_true, 5.5), 6.0)
                    half_line_limit = w2_trunk + max(insulator_margin * 1.6, 4.2)
                    half_line_t = min(max(p98_v2 + 0.5, half_line_limit), max_ceiling)
                else:
                    # 直线塔保持严格紧致上限（<= 4.2m），自然阻断非挂点出线跨中导线
                    max_ceiling = max(0.08 * h_true, 4.2)
                    half_line_limit = w2_trunk + max(insulator_margin, 2.0)
                    half_line_t = min(max(p98_v2 + 0.35, 2.5), min(max(half_line_limit, 3.2), max_ceiling))
            else:
                half_arm_w = half_arm_max * 1.05
                half_line_t = 3.2

            mask_high = high_arm_zone & (d_v1 <= half_arm_w) & (d_v2 <= half_line_t)
            # 【ADR 0006 & Ticket 03: 猫头塔顶羊角与地线支架包围盒自适应放宽 (Cathead Horn Bracket Mask Expansion)】
            top_arm_w = max(w_waist * 1.6 + 2.5, half_arm_w)
            is_top_bracket_zone = (tower_z >= (max_z - 5.5))
            if abs_max_z > 0.0:
                is_top_bracket_zone = is_top_bracket_zone | (tower_pts[:, 2] >= (abs_max_z - 5.5))
            top_bracket_mask = is_top_bracket_zone & (d_v1 <= top_arm_w) & (d_v2 <= half_line_t)

            # 【ADR 0005 & Ticket 03: 下半部四棱台放坡包围盒 (LowerTowerFrustum)】
            # 从 z_upper_floor (WaistBoundaryElevation) 向下以真实物理放坡斜率线性延伸至塔基地面
            delta_z_all = np.maximum(z_upper_floor - tower_z, 0.0)
            w1_allowed = w1_trunk + delta_z_all * max_slope_rate + margin_val
            w2_allowed = w2_trunk + delta_z_all * max_slope_rate + margin_val
            
            low_tower_cand = (tower_z < z_upper_floor) & (tower_z >= 0.0) & (d_v1 <= w1_allowed) & (d_v2 <= w2_allowed)

            # 以腰部纯净立柱点为种子向下连通，步长 0.85m 贯穿钢构角钢，阻断方盒边缘不连通的山坡独立植被与悬空杂块
            seed_mask = high_arm_zone & (tower_z <= (z_lowest_arm + 3.0)) & (d_v1 <= (w_waist + 0.8)) & (d_v2 <= (w_waist + 0.8))
            if np.sum(seed_mask) >= 5 and np.sum(low_tower_cand) >= 10:
                sub_pts_for_cc = np.vstack([tower_pts[seed_mask], tower_pts[low_tower_cand]])
                v_size_cc = 0.45
                v_coords = (sub_pts_for_cc / v_size_cc).astype(np.int32)
                u_v, inv_v = np.unique(v_coords, axis=0, return_inverse=True)
                seed_v_ids = np.unique(inv_v[:np.sum(seed_mask)])
                
                v_tree = cKDTree(u_v * v_size_cc)
                pairs_cc = v_tree.query_pairs(r=0.85)
                
                adj_cc = {}
                for u_id, v_id in pairs_cc:
                    adj_cc.setdefault(u_id, []).append(v_id)
                    adj_cc.setdefault(v_id, []).append(u_id)
                    
                visited_v = np.zeros(len(u_v), dtype=bool)
                q = deque(seed_v_ids)
                for s in seed_v_ids:
                    visited_v[s] = True
                while q:
                    curr = q.popleft()
                    for nbr in adj_cc.get(curr, []):
                        if not visited_v[nbr]:
                            visited_v[nbr] = True
                            q.append(nbr)
                            
                # 仅保留与塔身高层连通的下部点
                connected_low_mask = visited_v[inv_v[np.sum(seed_mask):]]
                low_tower_mask = np.zeros(len(tower_pts), dtype=bool)
                low_cand_indices = np.where(low_tower_cand)[0]
                low_tower_mask[low_cand_indices[connected_low_mask]] = True
            else:
                low_tower_mask = low_tower_cand

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
