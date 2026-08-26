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

from modules.topdown_wire_extractor import extract_wires_topdown

def extract_and_track_powerlines(points: np.ndarray,
                                 off_ground_pts: np.ndarray,
                                 off_ground_idx: np.ndarray,
                                 rel_z: np.ndarray,
                                 is_tower: np.ndarray,
                                 is_near_tower_high_arm: np.ndarray,
                                 tower_infos: List[Union[TowerEntity, dict]],
                                 is_tower_arm: np.ndarray = None,
                                 config: PipelineConfig = None) -> ExtractionResult:
    """
    阶段三门面函数 (Facade)：基于杆塔先验的自顶向下导线提取 + 多层耐张引流跳线拓扑解算
    """
    if config is None:
        config = DEFAULT_CONFIG

    arm_mask = is_tower_arm if is_tower_arm is not None else is_near_tower_high_arm

    return extract_wires_topdown(
        points=points,
        off_ground_pts=off_ground_pts,
        off_ground_idx=off_ground_idx,
        rel_z=rel_z,
        is_tower=is_tower,
        is_tower_arm=arm_mask,
        is_near_tower_high_arm=is_near_tower_high_arm,
        tower_infos=tower_infos,
        config=config
    )
