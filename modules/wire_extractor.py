"""
点云电力线与架空地线提取深度核心模块 (Deep Wire Extractor)

统一统筹走廊定向切片高精提取、点集差量剔除、种子体素追踪兜底及耐张跳线抽取。
内部就地压平并查集与拓扑图关系，对外输出高内聚的 WireExtractionResult 领域成果，
彻底消灭对外泄露并查集闭包与长达 N 的全局临时大数组。
"""

from typing import List, Optional, Union, Set, Dict, Any
import numpy as np

from modules.config import PipelineConfig, DEFAULT_CONFIG
from modules.models import TowerEntity, WireCluster, WireExtractionResult
from modules.catenary import fit_catenary_3d
from modules.topdown_wire_extractor import extract_wires_by_corridor_slices
from modules.wire_seed_extractor import extract_wire_seeds
from modules.wire_tracker import cluster_wire_candidates, filter_canopy_by_probes, track_and_bridge_powerlines
from modules.tension_topology import extract_tension_jumpers


class WireExtractor:
    """
    点云电力线提取深度执行器
    """

    def __init__(self, config: Optional[PipelineConfig] = None):
        self.config = config if config is not None else DEFAULT_CONFIG

    def extract(
        self,
        points: np.ndarray,
        off_ground_pts: np.ndarray,
        off_ground_idx: np.ndarray,
        rel_z: np.ndarray,
        is_tower: np.ndarray,
        is_tower_arm: Optional[np.ndarray] = None,
        is_near_tower_high_arm: Optional[np.ndarray] = None,
        tower_infos: Optional[List[Union[TowerEntity, dict]]] = None,
        config: Optional[PipelineConfig] = None
    ) -> WireExtractionResult:
        """
        统一导线提取入口：调度走廊切片、残差种子追踪与跳线提取

        Parameters:
        -----------
        points : np.ndarray
            (N, 3) 原始点云全局空间坐标
        off_ground_pts : np.ndarray
            (M, 3) 离地目标点空间坐标
        off_ground_idx : np.ndarray
            (M,) 离地点在原始点云中的全局索引
        rel_z : np.ndarray
            (M,) 离地相对高程
        is_tower : np.ndarray
            (M,) bool 铁塔点掩膜
        is_tower_arm : Optional[np.ndarray]
            (M,) bool 铁塔横担点掩膜
        is_near_tower_high_arm : Optional[np.ndarray]
            (M,) bool 铁塔高位横担邻域掩膜
        tower_infos : Optional[List[Union[TowerEntity, dict]]]
            已检测锁定的铁塔实体列表
        config : Optional[PipelineConfig]
            运行时参数配置

        Returns:
        --------
        WireExtractionResult
            封装最终导线簇实体、点索引及分类元数据的领域成果
        """
        cfg = config if config is not None else self.config
        p_cfg = cfg.powerline
        towers = tower_infos if tower_infos is not None else []
        num_points = len(points)

        confirmed_wires: List[WireCluster] = []
        collected_indices_set: Set[int] = set()
        next_line_id = 1

        # =====================================================================
        # 通道一：走廊定向切片高精度提取 (Top-down Corridor Slices)
        # =====================================================================
        if p_cfg.enable_topdown_prior and len(towers) >= 1:
            topdown_cable_indices, topdown_point_line_id, topdown_clusters = extract_wires_by_corridor_slices(
                points=points,
                off_ground_pts=off_ground_pts,
                off_ground_idx=off_ground_idx,
                rel_z=rel_z,
                is_tower=is_tower,
                tower_infos=towers,
                config=cfg
            )
            if len(topdown_cable_indices) > 0 and len(topdown_clusters) > 0:
                for i, cluster in enumerate(topdown_clusters):
                    cluster_lid = i + 1
                    cluster.line_id = cluster_lid
                    # 获取属于该簇的全局点索引
                    c_gidx = np.where(topdown_point_line_id == cluster_lid)[0]
                    if len(c_gidx) > 0:
                        cluster.global_indices = c_gidx
                        cluster.members = list(range(len(c_gidx)))
                    confirmed_wires.append(cluster)
                    collected_indices_set.update(c_gidx.tolist())

                next_line_id = len(confirmed_wires) + 1

        # =====================================================================
        # 通道二：点集差量排除 + 残差种子追踪兜底 (Residual Seed Tracker)
        # =====================================================================
        high_mask = rel_z >= p_cfg.min_rel_z
        # 排除铁塔点
        high_mask = high_mask & (~is_tower)

        # 若通道一已提取导线点，对高空候选点做差量剔除
        if len(collected_indices_set) > 0:
            claimed_in_off = np.isin(off_ground_idx, list(collected_indices_set))
            residual_high_mask = high_mask & (~claimed_in_off)
        else:
            residual_high_mask = high_mask

        n_residual = int(np.sum(residual_high_mask))

        # 仅当残差高空点数 >= 50 时触发通道二；否则短路以消除冗余计算
        if n_residual >= 50:
            res_high_pts = off_ground_pts[residual_high_mask]
            res_high_indices = off_ground_idx[residual_high_mask]
            res_high_is_near_arm = (
                is_near_tower_high_arm[residual_high_mask]
                if is_near_tower_high_arm is not None
                else np.zeros(len(res_high_pts), dtype=bool)
            )
            res_high_is_tower = np.zeros(len(res_high_pts), dtype=bool)

            cable_seed_indices = extract_wire_seeds(
                high_pts=res_high_pts,
                high_indices=res_high_indices,
                high_is_near_arm=res_high_is_near_arm,
                high_is_tower=res_high_is_tower,
                tower_infos=towers,
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

            if len(cable_seed_indices) > 0:
                final_cable_pts, cable_pts_idx, tracker_clusters = cluster_wire_candidates(
                    points=points,
                    high_pts=res_high_pts,
                    high_indices=res_high_indices,
                    cable_seed_indices=cable_seed_indices,
                    c_voxel_size=p_cfg.voxel_cluster_size,
                    tower_infos=towers
                )

                if len(cable_pts_idx) > 0 and len(tracker_clusters) > 0:
                    suspect_line_ids = filter_canopy_by_probes(
                        final_cable_pts=final_cable_pts,
                        off_ground_pts=off_ground_pts,
                        all_confirmed=tracker_clusters,
                        tower_infos=towers
                    )

                    tracker_cable_pts_idx, tracker_point_line_id, find_line_func = track_and_bridge_powerlines(
                        points=points,
                        high_pts=res_high_pts,
                        high_indices=res_high_indices,
                        final_cable_pts=final_cable_pts,
                        cable_pts_idx=cable_pts_idx,
                        all_confirmed=tracker_clusters,
                        suspect_line_ids=suspect_line_ids,
                        tower_infos=towers,
                        max_tracking_steps=p_cfg.max_tracking_steps,
                        step_size=p_cfg.tracking_step,
                        use_catenary_tracking=p_cfg.use_catenary_tracking
                    )

                    # -------------------------------------------------------------
                    # 就地压平并查集 (Flatten Union-Find) 并直接构造 WireCluster
                    # -------------------------------------------------------------
                    root_to_new_lid: Dict[int, int] = {}
                    for g_idx in tracker_cable_pts_idx:
                        raw_id = tracker_point_line_id[g_idx]
                        if raw_id <= 0:
                            continue
                        root_id = find_line_func(raw_id)
                        if root_id not in root_to_new_lid:
                            root_to_new_lid[root_id] = next_line_id
                            next_line_id += 1

                    new_lid_to_pts: Dict[int, List[int]] = {nlid: [] for nlid in root_to_new_lid.values()}
                    for g_idx in tracker_cable_pts_idx:
                        raw_id = tracker_point_line_id[g_idx]
                        if raw_id <= 0:
                            continue
                        root_id = find_line_func(raw_id)
                        nlid = root_to_new_lid[root_id]
                        new_lid_to_pts[nlid].append(g_idx)

                    for nlid, g_indices in new_lid_to_pts.items():
                        if len(g_indices) < 6:
                            continue
                        g_arr = np.array(g_indices, dtype=int)
                        c_pts = points[g_arr]
                        ptp = np.ptp(c_pts, axis=0)
                        span_d = float(np.linalg.norm(ptp))
                        cov = np.cov(c_pts.T)
                        evs, evecs = np.linalg.eigh(cov)
                        lin = float((evs[2] - evs[1]) / evs[2]) if evs[2] > 0 else 0.8

                        cat_m = None
                        if len(c_pts) >= 12 and span_d >= 8.0:
                            cat_m = fit_catenary_3d(c_pts)

                        cluster = WireCluster(
                            members=list(range(len(c_pts))),
                            center=np.mean(c_pts, axis=0),
                            dir=evecs[:, 2] if evs[-1] > 0 else np.array([1.0, 0.0, 0.0]),
                            span=span_d,
                            linearity=lin,
                            min_var=float(evs[0]),
                            catenary=cat_m,
                            line_id=nlid,
                            global_indices=g_arr,
                            is_suspect=False,
                            is_jumper=False
                        )
                        confirmed_wires.append(cluster)
                        collected_indices_set.update(g_indices)

        # =====================================================================
        # 通道三：耐张跳线专用 3D 弧段提取 (Tension Jumpers)
        # =====================================================================
        jumper_indices_arr = np.array([], dtype=int)
        if p_cfg.enable_jumper_extraction and len(towers) > 0:
            all_cable_so_far = np.array(list(collected_indices_set), dtype=int)
            jumper_idx = extract_tension_jumpers(
                points=points,
                off_ground_pts=off_ground_pts,
                off_ground_idx=off_ground_idx,
                rel_z=rel_z,
                is_tower=is_tower,
                tower_infos=towers,
                cable_pts_idx=all_cable_so_far,
                config=cfg
            )
            if len(jumper_idx) > 0:
                new_jumpers = np.setdiff1d(jumper_idx, all_cable_so_far)
                if len(new_jumpers) > 0:
                    jumper_indices_arr = new_jumpers
                    j_pts = points[new_jumpers]
                    if len(j_pts) >= 4:
                        j_cov = np.cov(j_pts.T)
                        j_evals, j_evecs = np.linalg.eigh(j_cov)
                        j_dir = j_evecs[:, 2]
                        j_lin = float((j_evals[2] - j_evals[1]) / j_evals[2]) if j_evals[2] > 0 else 0.6
                    else:
                        j_dir = np.array([1.0, 0.0, 0.0])
                        j_lin = 0.6

                    j_cluster = WireCluster(
                        members=list(range(len(new_jumpers))),
                        center=np.mean(j_pts, axis=0),
                        dir=j_dir,
                        span=float(np.linalg.norm(np.ptp(j_pts, axis=0))),
                        linearity=j_lin,
                        min_var=0.2,
                        catenary=None,
                        line_id=next_line_id,
                        global_indices=new_jumpers,
                        is_suspect=False,
                        is_jumper=True
                    )
                    next_line_id += 1
                    confirmed_wires.append(j_cluster)
                    collected_indices_set.update(new_jumpers.tolist())

        # =====================================================================
        # 成果融合与铁塔硬互斥保障 (Tower Mutual Exclusivity)
        # =====================================================================
        tower_pts_idx = off_ground_idx[is_tower] if np.any(is_tower) else np.array([], dtype=int)
        final_cable_indices = np.array(list(collected_indices_set), dtype=int)

        if len(tower_pts_idx) > 0 and len(final_cable_indices) > 0:
            final_cable_indices = np.setdiff1d(final_cable_indices, tower_pts_idx)
            for w in confirmed_wires:
                if len(w.global_indices) > 0:
                    w.global_indices = np.setdiff1d(w.global_indices, tower_pts_idx)
                    w.members = list(range(len(w.global_indices)))

        return WireExtractionResult(
            wires=confirmed_wires,
            cable_indices=final_cable_indices,
            jumper_indices=jumper_indices_arr,
            insulator_indices=np.array([], dtype=int),
            metadata={'num_points': num_points}
        )
