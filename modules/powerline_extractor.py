import warnings
import numpy as np
from typing import List, Union
from modules.models import TowerEntity, ExtractionResult
from modules.config import PipelineConfig, DEFAULT_CONFIG
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
    [DEPRECATED] 阶段三门面函数：请直接使用 PipelineExecutor 或 modules.topdown_wire_extractor.extract_wires_topdown
    """
    warnings.warn(
        "extract_and_track_powerlines is deprecated; use PipelineExecutor or extract_wires_topdown directly.",
        DeprecationWarning,
        stacklevel=2
    )
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
