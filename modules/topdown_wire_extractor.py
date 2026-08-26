import numpy as np
from scipy.spatial import cKDTree
from typing import List, Tuple, Set, Dict, Optional, Union, Callable

from modules.models import TowerEntity, WireCluster, ExtractionResult
from modules.catenary import fit_catenary_3d, CatenaryModel
from modules.config import PipelineConfig, DEFAULT_CONFIG
from modules.wire_seed_extractor import extract_wire_seeds
from modules.wire_tracker import cluster_wire_candidates, filter_canopy_by_probes, track_and_bridge_powerlines
from modules.tension_topology import extract_tension_jumpers, solve_tension_topology


def extract_wires_by_corridor_slices(points: np.ndarray,
                                     off_ground_pts: np.ndarray,
                                     off_ground_idx: np.ndarray,
                                     rel_z: np.ndarray,
                                     is_tower: np.ndarray,
                                     tower_infos: List[Union[TowerEntity, dict]],
                                     config: PipelineConfig = None) -> Tuple[List[int], np.ndarray, List[WireCluster]]:
    """
    【工业级核心算法】多通道走廊定向切片 + PCA 姿态滤波 + 3D 悬链线数据驱动分解
    全场景防御与覆盖机制：
      1. 主档走廊：精准切分相邻两塔跨间导线；
      2. 首末端出线走廊：自动外延提取点云首末端铁塔外侧导线与地线，杜绝截断漏检；
      3. 转角出线走廊：自动提取耐张/转角塔变向出线段；
      4. 沿走廊各向异性加权体素聚类，解耦多层多横担；
      5. 3D 悬链线强校验与端头引桥吸附。
    """
    if config is None:
        config = DEFAULT_CONFIG
        
    num_points = len(points)
    point_line_id = np.zeros(num_points, dtype=int)
    all_wire_clusters: List[WireCluster] = []
    collected_cable_indices: List[int] = []

    if len(tower_infos) == 0:
        return [], point_line_id, []

    line_id_counter = 1
    p_cfg = config.powerline

    # 1. 沿主走向对杆塔进行拓扑排序
    if len(tower_infos) >= 2:
        tower_centers = np.array([[float(t['cx']), float(t['cy'])] for t in tower_infos])
        cov_t = np.cov(tower_centers.T)
        evals, evecs = np.linalg.eigh(cov_t)
        v_main = evecs[:, 1]
        proj_t = tower_centers @ v_main
        sort_order = np.argsort(proj_t)
        sorted_towers = [tower_infos[i] for i in sort_order]
    else:
        sorted_towers = list(tower_infos)

    # 种子 z 下限: 塔脚绝对 z + 最低横担相对高度 - 弧垂下探余量 (排除树冠主体扫描线伪种子)
    z_seed_floor = -1e9
    if len(tower_infos) >= 1:
        arm_zs = []
        for t_info in sorted_towers:
            t_idx_local = np.array(t_info.get('pts_idx', np.array([], dtype=int)), dtype=int)
            if len(t_idx_local) > 0:
                arm_zs.append(float(np.min(off_ground_pts[t_idx_local, 2])) +
                              float(t_info.get('z_lowest_arm', 10.0)) - 16.0)
        if len(arm_zs) > 0:
            z_seed_floor = float(np.min(arm_zs))

    # 2. 遍历每一个相邻两塔主档段 (Main Spans)
    if len(sorted_towers) >= 2:
        for span_idx in range(len(sorted_towers) - 1):
            t1, t2 = sorted_towers[span_idx], sorted_towers[span_idx + 1]
            c1 = np.array([float(t1['cx']), float(t1['cy'])])
            c2 = np.array([float(t2['cx']), float(t2['cy'])])

            diff_span = c2 - c1
            span_length = float(np.hypot(diff_span[0], diff_span[1]))
            if span_length < 10.0:
                continue

            u_span = diff_span / span_length               # 档段纵向走向单位矢量 (2D)
            n_span = np.array([-u_span[1], u_span[0]])     # 档段水平法向单位矢量 (2D)

            half_arm_max = max(float(t1.get('half_l1', 10.0)), float(t2.get('half_l1', 10.0)), 8.0)
            corridor_half_w = half_arm_max + 4.5           # 电气走廊横向有效半宽 (m)

            # 提取走廊内候选点云
            diff_pts_2d = off_ground_pts[:, :2] - c1
            s_coords = diff_pts_2d @ u_span
            d_coords = diff_pts_2d @ n_span
            z_coords = off_ground_pts[:, 2]

            # 走廊纵向与横向硬包络，排除走廊外大面积树林
            in_corridor_mask = (s_coords >= -8.0) & (s_coords <= span_length + 8.0) & \
                               (np.abs(d_coords) <= corridor_half_w) & \
                               (rel_z >= p_cfg.min_rel_z) & (~is_tower)

            cand_indices_in_off = np.where(in_corridor_mask)[0]
            if len(cand_indices_in_off) < 30:
                continue

            cand_pts = off_ground_pts[cand_indices_in_off]
            cand_global = off_ground_idx[cand_indices_in_off]
            cand_s = s_coords[cand_indices_in_off]
            cand_d = d_coords[cand_indices_in_off]
            cand_z = z_coords[cand_indices_in_off]

            span_collected, span_clusters, line_id_counter = _extract_wires_in_corridor(
                cand_pts=cand_pts,
                cand_global=cand_global,
                cand_s=cand_s,
                cand_d=cand_d,
                cand_z=cand_z,
                u_span=u_span,
                span_length=span_length,
                point_line_id=point_line_id,
                line_id_counter=line_id_counter,
                config=config,
                z_seed_floor=z_seed_floor
            )
            collected_cable_indices.extend(span_collected)
            all_wire_clusters.extend(span_clusters)

    # 3. 首末端档外延伸出线走廊 (Terminal Out-Corridors) 与 转角塔出线走廊
    #    针对单塔或首末端铁塔外侧导线，自动外延搜索，确保不漏检档外相导线与地线
    terminal_directions = []
    if len(sorted_towers) >= 2:
        # 首端塔向外：反向 -u_first
        c0, c1 = np.array([float(sorted_towers[0]['cx']), float(sorted_towers[0]['cy'])]), np.array([float(sorted_towers[1]['cx']), float(sorted_towers[1]['cy'])])
        d01 = c1 - c0
        u01 = d01 / max(np.linalg.norm(d01), 1e-3)
        terminal_directions.append((sorted_towers[0], -u01))

        # 末端塔向外：正向 +u_last
        cn_1, cn = np.array([float(sorted_towers[-2]['cx']), float(sorted_towers[-2]['cy'])]), np.array([float(sorted_towers[-1]['cx']), float(sorted_towers[-1]['cy'])])
        dn = cn - cn_1
        un = dn / max(np.linalg.norm(dn), 1e-3)
        terminal_directions.append((sorted_towers[-1], un))

        # 转角塔出线走廊（当塔 v2 夹角偏离 >= 15°）
        for t_info in sorted_towers:
            v2_t = np.array([float(t_info['v2'][0]), float(t_info['v2'][1])], dtype=float)
            norm_v2 = np.linalg.norm(v2_t)
            if norm_v2 < 1e-6:
                continue
            v2_t /= norm_v2
            c_t = np.array([float(t_info['cx']), float(t_info['cy'])])

            best_cos = 0.0
            for t_other in sorted_towers:
                c_o = np.array([float(t_other['cx']), float(t_other['cy'])])
                if np.hypot(c_o[0] - c_t[0], c_o[1] - c_t[1]) < 10.0:
                    continue
                d_o = c_o - c_t
                u_o = d_o / np.linalg.norm(d_o)
                cos_vo = abs(float(np.dot(u_o, v2_t)))
                if cos_vo > best_cos:
                    best_cos = cos_vo
            angle_deg = float(np.degrees(np.arccos(min(best_cos, 1.0))))
            if angle_deg >= 15.0:
                terminal_directions.append((t_info, v2_t))
                terminal_directions.append((t_info, -v2_t))
    elif len(sorted_towers) == 1:
        t_single = sorted_towers[0]
        v2_t = np.array([float(t_single['v2'][0]), float(t_single['v2'][1])], dtype=float)
        norm_v2 = np.linalg.norm(v2_t)
        if norm_v2 > 1e-6:
            v2_t /= norm_v2
            terminal_directions.append((t_single, v2_t))
            terminal_directions.append((t_single, -v2_t))

    # 提取所有外延走廊
    for t_info, u_out in terminal_directions:
        norm_u = np.linalg.norm(u_out)
        if norm_u < 1e-6:
            continue
        u_out = u_out / norm_u
        n_out = np.array([-u_out[1], u_out[0]])

        c_t = np.array([float(t_info['cx']), float(t_info['cy'])])
        exit_half_w = max(float(t_info.get('half_l1', 8.0)), 8.0) + 4.5
        z_coords = off_ground_pts[:, 2]
        diff_t = off_ground_pts[:, :2] - c_t
        s_out = diff_t @ u_out
        d_out = diff_t @ n_out

        # 出线走廊：从塔心沿 u_out 延伸，纵向范围覆盖 2.0m ~ 300.0m
        in_exit = (s_out >= 2.0) & (s_out <= 300.0) & \
                  (np.abs(d_out) <= exit_half_w) & \
                  (rel_z >= p_cfg.min_rel_z) & (~is_tower)
        e_idx = np.where(in_exit)[0]
        if len(e_idx) < 20:
            continue

        exit_span = max(40.0, float(np.ptp(s_out[e_idx])))
        exit_collected, exit_clusters, line_id_counter = _extract_wires_in_corridor(
            cand_pts=off_ground_pts[e_idx],
            cand_global=off_ground_idx[e_idx],
            cand_s=s_out[e_idx],
            cand_d=d_out[e_idx],
            cand_z=z_coords[e_idx],
            u_span=u_out,
            span_length=exit_span,
            point_line_id=point_line_id,
            line_id_counter=line_id_counter,
            config=config,
            z_seed_floor=z_seed_floor
        )
        collected_cable_indices.extend(exit_collected)
        all_wire_clusters.extend(exit_clusters)

    return collected_cable_indices, point_line_id, all_wire_clusters

def _extract_wires_in_corridor(cand_pts: np.ndarray,
                               cand_global: np.ndarray,
                               cand_s: np.ndarray,
                               cand_d: np.ndarray,
                               cand_z: np.ndarray,
                               u_span: np.ndarray,
                               span_length: float,
                               point_line_id: np.ndarray,
                               line_id_counter: int,
                               config: PipelineConfig,
                               z_seed_floor: float = -1e9) -> Tuple[List[int], List[WireCluster], int]:
    """
    走廊内四道防线核心提取：局部 PCA 姿态滤波 -> 纯净种子辐射扩散 -> 各向异性体素聚类
    -> 3D 悬链线物理拟合与全跨搜集。可复用于主档走廊与转角塔出线走廊。
    """
    p_cfg = config.powerline
    collected_cable_indices: List[int] = []
    all_wire_clusters: List[WireCluster] = []

    if len(cand_pts) < 30:
        return collected_cable_indices, all_wire_clusters, line_id_counter

    # -------------------------------------------------------------
    # 第一道防线：局部 PCA 线型度 + 水平方向滤波 (杜绝树冠与直立树干)
    # -------------------------------------------------------------
    cand_tree = cKDTree(cand_pts.astype(np.float32))
    
    # 降采样采样点计算 PCA (0.8m 体素抽取代表点，极速执行)
    sgx = (cand_s / 0.8).astype(np.int32)
    sgy = (cand_d / 0.8).astype(np.int32)
    sgz = (cand_z / 0.8).astype(np.int32)
    sample_coords = np.column_stack([sgx, sgy, sgz])
    _, sample_idx = np.unique(sample_coords, axis=0, return_index=True)
    sample_pts = cand_pts[sample_idx]

    r_pca = max(p_cfg.pca_radius, 1.0)
    neighbors_list = cand_tree.query_ball_point(sample_pts, r=r_pca)
    # 低密度区自适应扩大 PCA 球：1.0m 内不足 3 点时用 2.2m 补足 (稀疏扫描线)
    sparse_mask = np.array([len(n) < 3 for n in neighbors_list])
    if np.any(sparse_mask):
        sparse_nbrs = cand_tree.query_ball_point(sample_pts[sparse_mask], r=2.2)
        sparse_idx_list = np.where(sparse_mask)[0]
        for k, sn in enumerate(sparse_nbrs):
            if len(sn) >= 3:
                neighbors_list[sparse_idx_list[k]] = sn
    
    pure_wire_sample_idx = []
    for s_i, nbrs in enumerate(neighbors_list):
        n_nbr = len(nbrs)
        if n_nbr >= 3:
            local_pts = cand_pts[nbrs]
            cov = np.cov(local_pts.T)
            evals_l, evecs_l = np.linalg.eigh(cov)
            l1, l2, l3 = evals_l[2], evals_l[1], evals_l[0]
            if l1 > 0:
                linearity = (l1 - l2) / l1
                v1 = evecs_l[:, 2] # 主方向
                
                # 导线物理准则：
                # 1. 线性度充足 (linearity >= 0.60)
                # 2. 主方向绝不能为竖直树干 (|v1[2]| <= 0.65)
                # 3. 水平方向必须高度平行于线路主走向 (|v1_xy . u_span| >= 0.70)
                # 4. 局部截面尺度 sqrt(l3) 必须纤细：单导线 <= 0.30m；
                #    分裂导线 (子线间距 0.45m) 放宽至 0.60m 但邻域点数受限 (<= 60)；
                #    (树冠 1.0m 球内通常 100+ 点，真导线 30~60 点，可物理分离)
                v1_xy = v1[:2]
                norm_xy = np.linalg.norm(v1_xy)
                if norm_xy > 1e-3:
                    v1_xy_unit = v1_xy / norm_xy
                    cos_align = abs(float(np.dot(v1_xy_unit, u_span)))
                else:
                    cos_align = 0.0

                l3_scale = float(np.sqrt(max(l3, 0.0)))
                is_slim_seed = l3_scale <= p_cfg.wire_seed_l3_max
                is_bundle_seed = (l3_scale <= p_cfg.wire_seed_l3_bundle_max) and (n_nbr <= p_cfg.wire_seed_density_max)

                # 种子 z 下限: 排除树冠主体扫描线伪种子 (导线最低不低于塔最低横担-弧垂下探)
                z_ok = cand_pts[sample_idx[s_i], 2] >= z_seed_floor

                if linearity >= 0.60 and abs(v1[2]) <= 0.65 and cos_align >= 0.70 and (is_slim_seed or is_bundle_seed) and z_ok:
                    pure_wire_sample_idx.append(sample_idx[s_i])

    if len(pure_wire_sample_idx) < 3:
        return collected_cable_indices, all_wire_clusters, line_id_counter

    # -------------------------------------------------------------
    # 第二道防线：纯净种子点辐射扩散过滤 (消灭一切孤立树木)
    # -------------------------------------------------------------
    pure_seeds = cand_pts[pure_wire_sample_idx]
    pure_tree = cKDTree(pure_seeds.astype(np.float32))
    
    dists_to_seed, _ = pure_tree.query(cand_pts, distance_upper_bound=1.5)
    is_wire_cand_mask = dists_to_seed <= 1.5

    wire_pts = cand_pts[is_wire_cand_mask]
    wire_global = cand_global[is_wire_cand_mask]
    wire_s = cand_s[is_wire_cand_mask]
    wire_d = cand_d[is_wire_cand_mask]
    wire_z = cand_z[is_wire_cand_mask]

    if len(wire_pts) < 10:
        return collected_cable_indices, all_wire_clusters, line_id_counter

    # -------------------------------------------------------------
    # 第三道防线：走廊各向异性加权体素聚类 (解耦多层横担)
    # -------------------------------------------------------------
    vx = (wire_s / 0.8).astype(np.int32)
    vy = (wire_d / 0.4).astype(np.int32)
    vz = (wire_z / 0.4).astype(np.int32)
    v_coords = np.column_stack([vx, vy, vz])

    unique_v, inv_map = np.unique(v_coords, axis=0, return_inverse=True)
    num_v = len(unique_v)
    if num_v < 4:
        return collected_cable_indices, all_wire_clusters, line_id_counter

    # 加权体素中心点 (纵向 1.0, 横向 2.0, 垂向 2.0，天然拉开多层横担)
    v_centers_weighted = np.column_stack([
        unique_v[:, 0] * 0.8,
        unique_v[:, 1] * (0.4 * 2.0),
        unique_v[:, 2] * (0.4 * p_cfg.cluster_weight_v)
    ])

    v_tree = cKDTree(v_centers_weighted.astype(np.float32))
    pairs = v_tree.query_pairs(r=3.2)

    parent_v = list(range(num_v))
    def find_v(i):
        root = i
        while parent_v[root] != root:
            root = parent_v[root]
        curr = i
        while curr != root:
            nxt = parent_v[curr]
            parent_v[curr] = root
            curr = nxt
        return root
    def union_v(i, j):
        ri, rj = find_v(i), find_v(j)
        if ri != rj: parent_v[ri] = rj

    for i, j in pairs:
        union_v(i, j)

    clusters_dict: Dict[int, List[int]] = {}
    for pt_i in range(len(wire_pts)):
        v_idx = inv_map[pt_i]
        r = find_v(v_idx)
        clusters_dict.setdefault(r, []).append(pt_i)

    # -------------------------------------------------------------
    # 第四道防线：3D 悬链线物理拟合与全跨平滑搜集
    # -------------------------------------------------------------
    for r_root, member_indices in clusters_dict.items():
        if len(member_indices) < 10:
            continue

        sub_pts = wire_pts[member_indices]
        sub_global = wire_global[member_indices]
        sub_s = wire_s[member_indices]
        sub_d = wire_d[member_indices]

        s_span = float(np.ptp(sub_s))
        if s_span < max(span_length * 0.15, 8.0):
            continue

        # 拟合 3D 悬链线模型
        cat = fit_catenary_3d(sub_pts, min_points=6, max_rmse_thresh=1.2)
        if cat is not None and cat.residual_rmse <= 0.45:
            # ---- 悬链线物理参数校验：拒绝树冠/建筑表面伪拟合 ----
            s_sub = cat.project_to_s(sub_pts)
            z_pred_sub = cat.predict_z(s_sub)
            pred_min, pred_max = float(np.min(z_pred_sub)), float(np.max(z_pred_sub))
            z_cl_min, z_cl_max = float(np.min(sub_pts[:, 2])), float(np.max(sub_pts[:, 2]))
            # 1. 悬链线必须真实穿过簇点 (树冠伪拟合的悬链线常偏离簇点数米至数十米)
            covers_cluster = (pred_min <= z_cl_max + 0.5) and (pred_max >= z_cl_min - 0.5)
            # 2. 张力参数 a 处于物理范围 (短跨/出线段/地线张力很大，上限放宽至 50000)
            a_max_allowed = 50000.0 if s_span < 80.0 else p_cfg.catenary_a_max
            a_ok = p_cfg.catenary_a_min <= cat.a <= a_max_allowed
            # 3. 弧垂比例必须处于设计规范范围；
            #    短跨 (档外延伸段/跳线段 < 80m) 弧垂绝对值天然小，下限放宽至 0.0
            s_span_sub = float(np.ptp(s_sub))
            sag_ratio = 0.0
            if s_span_sub >= 10.0:
                sag = cat.compute_sag(float(np.min(s_sub)), float(np.max(s_sub)))
                sag_ratio = max(sag / s_span_sub, 0.0)
            sag_min_ratio = 0.0 if s_span_sub < 80.0 else p_cfg.min_sag_ratio
            sag_ok = (sag_min_ratio <= sag_ratio <= p_cfg.max_sag_ratio)

            fit_ok = covers_cluster and a_ok and sag_ok
        else:
            fit_ok = False
            cat = None

        # 拟合失败/校验失败: 按 z 薄层 (0.6m) 子聚类重试，
        # 解耦低层导线与贴近树冠顶的杂波 (树冠垂直厚度 3m+，导线 < 0.4m)
        retry_found = False
        if not fit_ok:
            vz_idx = np.round(sub_pts[:, 2] / 0.6).astype(np.int32)
            for vz_val in np.unique(vz_idx):
                zm = vz_idx == vz_val
                if np.sum(zm) < 10:
                    continue
                zsub = sub_pts[zm]
                zglobal = sub_global[zm]
                zsub_s = sub_s[zm]
                zsub_d = sub_d[zm]
                z_span = float(np.ptp(zsub_s))
                if z_span < max(span_length * 0.15, 8.0):
                    continue
                zcat = fit_catenary_3d(zsub, min_points=6, max_rmse_thresh=1.2)
                if zcat is None or zcat.residual_rmse > 0.45:
                    continue
                zs_proj = zcat.project_to_s(zsub)
                zz_pred = zcat.predict_z(zs_proj)
                zpred_min, zpred_max = float(np.min(zz_pred)), float(np.max(zz_pred))
                zz_cl_min, zz_cl_max = float(np.min(zsub[:, 2])), float(np.max(zsub[:, 2]))
                zcovers = (zpred_min <= zz_cl_max + 0.5) and (zpred_max >= zz_cl_min - 0.5)
                za_max_allowed = 50000.0 if z_span < 80.0 else p_cfg.catenary_a_max
                za_ok = p_cfg.catenary_a_min <= zcat.a <= za_max_allowed
                z_span_sub = float(np.ptp(zs_proj))
                z_ratio = 0.0
                if z_span_sub >= 10.0:
                    z_ratio = max(zcat.compute_sag(float(np.min(zs_proj)), float(np.max(zs_proj))) / z_span_sub, 0.0)
                z_min_ratio = 0.0 if z_span_sub < 80.0 else p_cfg.min_sag_ratio
                if not (zcovers and za_ok and (z_min_ratio <= z_ratio <= p_cfg.max_sag_ratio)):
                    continue
                _sub_pts = zsub
                _sub_global = zglobal
                sub_d_used = zsub_d
                cat_used = zcat
                retry_found = True
                fit_ok = True
                break
            if not retry_found:
                # 检查是否为极高线性度的平直外延导线簇 (PCA 线型度 >= 0.82 且截面纤细)
                if len(sub_pts) >= 8:
                    cov_raw = np.cov(sub_pts.T)
                    evs_raw = np.linalg.eigvalsh(cov_raw)
                    if evs_raw[2] > 0 and (evs_raw[2] - evs_raw[1]) / evs_raw[2] >= 0.80:
                        _sub_pts = sub_pts
                        _sub_global = sub_global
                        sub_d_used = sub_d
                        cat_used = None
                    else:
                        continue
                else:
                    continue
        else:
            _sub_pts = sub_pts
            _sub_global = sub_global
            sub_d_used = sub_d
            cat_used = cat

        if fit_ok:
                # 在走廊候选点内沿该悬链线模型进行全跨搜集 (残差 <= 0.45m)
                s_proj_all = cat_used.project_to_s(cand_pts)
                z_pred_all = cat_used.predict_z(s_proj_all)
                mean_d = float(np.mean(sub_d_used))
                d_pred_diff = np.abs(cand_d - mean_d)
                z_res = np.abs(cand_z - z_pred_all)

                # 【挂点引桥吸附机制】主跨内严格搜集；在两端挂点附近 (cand_s 接近 0 或 span_length) 放宽残差吸附过渡点
                is_end_zone = (cand_s <= 5.0) | (cand_s >= span_length - 5.0)
                allowed_z_res = np.where(is_end_zone, 0.75, 0.45)
                allowed_d_diff = np.where(is_end_zone, 1.80, 1.45)

                inlier_mask = (d_pred_diff <= allowed_d_diff) & (z_res <= allowed_z_res) & \
                              (cand_s >= -6.0) & (cand_s <= span_length + 6.0)
                # 全跨搜集的候选点做局部线性度复查，杜绝树冠/建筑表面混入
                gather_idx_in_cand = np.where(inlier_mask)[0]
                if len(gather_idx_in_cand) < 10:
                    continue
                gather_pts = cand_pts[gather_idx_in_cand]
                gather_tree = cKDTree(gather_pts.astype(np.float32))
                gl_nbrs = gather_tree.query_ball_point(gather_pts, r=1.0)
                # 稀疏区自适应扩大复查球 (低密度扫描线 1.0m 内不足 3 点时用 3.0m 补足)
                sparse_g = np.array([len(n) < 3 for n in gl_nbrs])
                if np.any(sparse_g):
                    sg_nbrs = gather_tree.query_ball_point(gather_pts[sparse_g], r=3.0)
                    sg_idx_list = np.where(sparse_g)[0]
                    for k, sn in enumerate(sg_nbrs):
                        if len(sn) >= 3:
                            gl_nbrs[sg_idx_list[k]] = sn
                keep_mask = np.zeros(len(gather_idx_in_cand), dtype=bool)
                for gi, g_nbrs in enumerate(gl_nbrs):
                    orig_cand_i = gather_idx_in_cand[gi]
                    # 若点极其精确地吻合悬链线 (残差 <= 0.35m)，或者在挂点端区，则直接保留，防止端部被误杀
                    if z_res[orig_cand_i] <= 0.35 or is_end_zone[orig_cand_i]:
                        keep_mask[gi] = True
                    elif len(g_nbrs) >= 3:
                        gcov = np.cov(cand_pts[gather_idx_in_cand[g_nbrs]].T)
                        gev = np.linalg.eigvalsh(gcov)
                        if gev[2] > 0:
                            g_lin = (gev[2] - gev[1]) / gev[2]
                            if g_lin >= 0.45:
                                keep_mask[gi] = True
                inlier_mask[gather_idx_in_cand] = keep_mask
                final_pts = cand_pts[inlier_mask]
                final_global = cand_global[inlier_mask]
        else:
                final_pts = _sub_pts
                final_global = _sub_global

        if len(final_global) >= 10:
                curr_line_id = line_id_counter
                line_id_counter += 1

                point_line_id[final_global] = curr_line_id
                collected_cable_indices.extend(final_global.tolist())

                ptp = np.ptp(final_pts, axis=0)
                span_d = float(np.linalg.norm(ptp))
                all_wire_clusters.append(WireCluster(
                    members=list(range(len(final_pts))),
                    center=np.mean(final_pts, axis=0),
                    dir=np.array([u_span[0], u_span[1], 0.0]),
                    span=span_d,
                    linearity=0.95,
                    min_var=0.04,
                    catenary=cat_used if fit_ok else None
                ))

    return collected_cable_indices, all_wire_clusters, line_id_counter



def extract_wires_topdown(points: np.ndarray,
                          off_ground_pts: np.ndarray,
                          off_ground_idx: np.ndarray,
                          rel_z: np.ndarray,
                          is_tower: np.ndarray,
                          is_tower_arm: np.ndarray,
                          is_near_tower_high_arm: np.ndarray,
                          tower_infos: List[Union[TowerEntity, dict]],
                          config: PipelineConfig = None) -> ExtractionResult:
    """
    基于杆塔先验引导的自顶向下 (Top-down) 导线提取主入口函数
    """
    if config is None:
        config = DEFAULT_CONFIG

    p_cfg = config.powerline
    num_points = len(points)
    topdown_cable_indices = []
    topdown_point_line_id = np.zeros(num_points, dtype=int)
    topdown_clusters: List[WireCluster] = []

    # 1. 走廊定向截面多通道 3D 悬链线高精提取
    if config.powerline.enable_topdown_prior and len(tower_infos) >= 1:
        topdown_cable_indices, topdown_point_line_id, topdown_clusters = extract_wires_by_corridor_slices(
            points=points,
            off_ground_pts=off_ground_pts,
            off_ground_idx=off_ground_idx,
            rel_z=rel_z,
            is_tower=is_tower,
            tower_infos=tower_infos,
            config=config
        )

    # 2. 兜底与跳线追踪器 (Tracker)
    high_mask = rel_z >= p_cfg.min_rel_z
    high_pts = off_ground_pts[high_mask]
    high_indices = off_ground_idx[high_mask]
    high_is_near_arm = is_near_tower_high_arm[high_mask]
    high_is_tower = is_tower[high_mask]

    cable_seed_indices = extract_wire_seeds(
        high_pts=high_pts,
        high_indices=high_indices,
        high_is_near_arm=high_is_near_arm,
        high_is_tower=high_is_tower,
        tower_infos=tower_infos,
        seed_grid_size=p_cfg.seed_grid_size,
        pca_radius=p_cfg.pca_radius,
        linearity_thresh=p_cfg.linearity_thresh,
        arm_linearity_thresh=p_cfg.arm_linearity_thresh,
        bundle_adapt_linearity_thresh=p_cfg.bundle_adapt_linearity_thresh,
        wire_seed_l3_max=p_cfg.wire_seed_l3_max,
        wire_seed_l3_bundle_max=p_cfg.wire_seed_l3_bundle_max,
        wire_seed_density_max=p_cfg.wire_seed_density_max,
        enable_bundle_adapt=p_cfg.enable_bundle_conductor_adapt
    )

    final_cable_pts, cable_pts_idx, all_confirmed = cluster_wire_candidates(
        points=points,
        high_pts=high_pts,
        high_indices=high_indices,
        cable_seed_indices=cable_seed_indices,
        c_voxel_size=p_cfg.voxel_cluster_size,
        tower_infos=tower_infos
    )

    suspect_line_ids = filter_canopy_by_probes(
        final_cable_pts=final_cable_pts,
        off_ground_pts=off_ground_pts,
        all_confirmed=all_confirmed,
        tower_infos=tower_infos
    )

    tracker_cable_pts_idx, tracker_point_line_id, find_line_func = track_and_bridge_powerlines(
        points=points,
        high_pts=high_pts,
        high_indices=high_indices,
        final_cable_pts=final_cable_pts,
        cable_pts_idx=cable_pts_idx,
        all_confirmed=all_confirmed,
        suspect_line_ids=suspect_line_ids,
        tower_infos=tower_infos,
        max_tracking_steps=p_cfg.max_tracking_steps,
        step_size=p_cfg.tracking_step,
        use_catenary_tracking=p_cfg.use_catenary_tracking
    )

    # 3. 结果合并：topdown 与 tracker 双通道 union (不再丢弃 tracker 档间完整导线)
    n_top_clusters = len(topdown_clusters)
    topdown_ok = len(topdown_cable_indices) >= 30

    if topdown_ok:
        merged_cable_indices = np.union1d(topdown_cable_indices, tracker_cable_pts_idx).astype(int)
        final_point_line_id = topdown_point_line_id.copy()

        # 补全 tracker 通道点的线 ID (偏移到 topdown 簇之后，避免 ID 冲突)
        missing_mask = (final_point_line_id[merged_cable_indices] == 0)
        missing_global_pts = merged_cable_indices[missing_mask]
        for g_idx in missing_global_pts:
            t_id = tracker_point_line_id[g_idx]
            final_point_line_id[g_idx] = (t_id + n_top_clusters) if t_id > 0 else 1

        # tracker 簇并入输出
        merged_confirmed = list(topdown_clusters) + list(all_confirmed)

        # find_line 包装：tracker 线 ID 偏移后映射回 tracker 的并查集
        def merged_find_line(raw_id: int) -> int:
            if raw_id <= n_top_clusters or n_top_clusters == 0:
                return raw_id
            return find_line_func(raw_id - n_top_clusters) + n_top_clusters

        merged_suspect = {sid + n_top_clusters for sid in suspect_line_ids}
    else:
        merged_cable_indices = tracker_cable_pts_idx
        final_point_line_id = tracker_point_line_id
        merged_confirmed = list(all_confirmed)
        merged_suspect = set(suspect_line_ids)
        merged_find_line = find_line_func

    # 4. 【耐张跳线专用 3D 弧段提取器】提取横担下方引流跳线并并入成果
    if config.powerline.enable_jumper_extraction and len(tower_infos) > 0:
        jumper_idx = extract_tension_jumpers(
            points=points,
            off_ground_pts=off_ground_pts,
            off_ground_idx=off_ground_idx,
            rel_z=rel_z,
            is_tower=is_tower,
            tower_infos=tower_infos,
            cable_pts_idx=merged_cable_indices,
            config=config
        )
        if len(jumper_idx) > 0:
            new_jumpers = np.setdiff1d(jumper_idx, merged_cable_indices)
            if len(new_jumpers) > 0:
                merged_cable_indices = np.union1d(merged_cable_indices, new_jumpers).astype(int)
                jumper_line_id = len(merged_confirmed) + 1
                for j_pt in new_jumpers:
                    final_point_line_id[j_pt] = jumper_line_id
                merged_confirmed.append(WireCluster(
                    members=list(range(len(new_jumpers))),
                    center=np.mean(points[new_jumpers], axis=0),
                    dir=np.array([1.0, 0.0, 0.0]),
                    span=float(np.linalg.norm(np.ptp(points[new_jumpers], axis=0))),
                    linearity=0.6,
                    min_var=0.2,
                    catenary=None
                ))

    return ExtractionResult(
        cable_pts_idx=merged_cable_indices,
        point_line_id=final_point_line_id,
        all_confirmed=merged_confirmed,
        suspect_line_ids=merged_suspect,
        find_line_func=merged_find_line,
        insulator_pts_idx=np.array([], dtype=int)
    )
