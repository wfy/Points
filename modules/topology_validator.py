import numpy as np
from scipy.spatial import cKDTree
from typing import List, Tuple, Union
from modules.models import TowerEntity
from modules.config import PipelineConfig, DEFAULT_CONFIG

def validate_tower_topology(points: np.ndarray,
                            off_ground_idx: np.ndarray,
                            rel_z: np.ndarray,
                            tower_infos: List[Union[TowerEntity, dict]],
                            cable_pts_idx: np.ndarray,
                            config: PipelineConfig = None) -> Tuple[np.ndarray, int, np.ndarray]:
    """
    阶段四：导线依附拓扑校验 (Topology Validation)
    基于导线与塔顶的依附挂接关系与双向几何骨架置信度校验剔除伪装假塔
    
    Parameters:
    -----------
    points : np.ndarray
        (N, 3) 全局空间点云坐标
    off_ground_idx : np.ndarray
        非地面点全局索引
    rel_z : np.ndarray
        非地面点相对高度
    tower_infos : list
        候选铁塔实体列表 (TowerEntity 或 dict)
    cable_pts_idx : np.ndarray
        提取的导线点全局索引
    config : PipelineConfig, optional
        全局参数配置
        
    Returns:
    --------
    tower_pts_idx : np.ndarray
        拓扑校验通过的铁塔点全局索引
    valid_tower_count : int
        校验通过的铁塔数量
    demoted_pts_idx : np.ndarray
        未通过拓扑校验、降级为非塔(植被/杂波)的点全局索引
    """
    if config is None:
        config = DEFAULT_CONFIG
        
    topo_cfg = config.topology
    final_tower_pts_idx = []
    demoted_tower_pts_idx = []
    valid_tower_count = 0
    
    if len(tower_infos) > 0:
        cable_tree_3d = cKDTree(points[cable_pts_idx]) if len(cable_pts_idx) > 0 else None
        
        for info in tower_infos:
            t_idx_local = info['pts_idx']
            if len(t_idx_local) == 0: 
                continue
            
            global_t_idx = off_ground_idx[t_idx_local]
            tower_pts_3d = points[global_t_idx]
            
            local_rel_z = rel_z[t_idx_local]
            top_mask = local_rel_z >= (info['max_z'] - topo_cfg.top_depth)
            top_pts_3d = tower_pts_3d[top_mask]
            
            confidence = info.get('confidence', 0.8)
            num_tower_pts = len(t_idx_local)
            
            is_valid_tower = False
            
            if cable_tree_3d is not None and len(top_pts_3d) >= 5:
                # 校验塔顶挂载导线距离与点数
                dists, _ = cable_tree_3d.query(top_pts_3d)
                near_top_cable_count = np.sum(dists < topo_cfg.cable_search_radius)
                
                if near_top_cable_count >= topo_cfg.min_anchor_cable_points:
                    is_valid_tower = True
                elif topo_cfg.enable_bidirectional_weight and confidence >= 0.85 and num_tower_pts >= 800 and near_top_cable_count >= 2:
                    # 双向加权保护：骨架置信度极高且点数充足的真塔，放宽挂接点数要求
                    is_valid_tower = True
            elif topo_cfg.enable_bidirectional_weight and confidence >= 0.90 and num_tower_pts >= 1200:
                # 若整档导线极度稀疏但铁塔骨架极完整
                is_valid_tower = True
                
            if is_valid_tower:
                final_tower_pts_idx.extend(global_t_idx.tolist())
                valid_tower_count += 1
            else:
                demoted_tower_pts_idx.extend(global_t_idx.tolist())
                
        tower_pts_idx = np.array(final_tower_pts_idx, dtype=int)
        demoted_pts_idx = np.array(demoted_tower_pts_idx, dtype=int)
    else:
        tower_pts_idx = np.array([], dtype=int)
        demoted_pts_idx = np.array([], dtype=int)

    return tower_pts_idx, valid_tower_count, demoted_pts_idx
