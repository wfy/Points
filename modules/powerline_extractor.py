import numpy as np
from typing import List, Union
from modules.models import TowerEntity, ExtractionResult
from modules.config import PipelineConfig, DEFAULT_CONFIG
from modules.wire_seed_extractor import extract_wire_seeds
from modules.wire_tracker import (
    cluster_wire_candidates,
    filter_canopy_by_probes,
    track_and_bridge_powerlines
)

def extract_and_track_powerlines(points: np.ndarray,
                                 off_ground_pts: np.ndarray,
                                 off_ground_idx: np.ndarray,
                                 rel_z: np.ndarray,
                                 is_tower: np.ndarray,
                                 is_near_tower_high_arm: np.ndarray,
                                 tower_infos: List[Union[TowerEntity, dict]],
                                 config: PipelineConfig = None) -> ExtractionResult:
    """
    阶段三门面函数 (Facade)：PCA 特征姿态分析 + 连续 3D 悬链线物理轨道追踪缝合 (纯净稳定基线)
    """
    if config is None:
        config = DEFAULT_CONFIG
        
    p_cfg = config.powerline

    # 1. 提取高空非地面点云切片
    high_mask = rel_z >= p_cfg.min_rel_z
    high_pts = off_ground_pts[high_mask]
    high_indices = off_ground_idx[high_mask]
    high_is_near_arm = is_near_tower_high_arm[high_mask]
    high_is_tower = is_tower[high_mask]

    # 2. 粗筛候选导线种子点 (支持分裂导线多尺度自适应)
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
        enable_bundle_adapt=p_cfg.enable_bundle_conductor_adapt
    )

    # 3. 体素连通图聚类与 3D 悬链线物理初筛
    final_cable_pts, cable_pts_idx, all_confirmed = cluster_wire_candidates(
        points=points,
        high_pts=high_pts,
        high_indices=high_indices,
        cable_seed_indices=cable_seed_indices,
        c_voxel_size=p_cfg.voxel_cluster_size
    )

    # 4. 探针阵列下探树冠连续性过滤
    suspect_line_ids = filter_canopy_by_probes(
        final_cable_pts=final_cable_pts,
        off_ground_pts=off_ground_pts,
        all_confirmed=all_confirmed,
        tower_infos=tower_infos
    )

    # 5. 3D 自适应悬链线步进追踪与跨塔缝合 (完整连续相导线)
    final_cable_pts_idx, point_line_id, find_line_func = track_and_bridge_powerlines(
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

    return ExtractionResult(
        cable_pts_idx=final_cable_pts_idx,
        point_line_id=point_line_id,
        all_confirmed=all_confirmed,
        suspect_line_ids=suspect_line_ids,
        find_line_func=find_line_func,
        insulator_pts_idx=np.array([], dtype=int)
    )
