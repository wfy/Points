import numpy as np
from scipy.spatial import cKDTree
from modules.models import GroundResult

def separate_ground(points: np.ndarray, 
                    grid_size: float = 3.0, 
                    height_threshold: float = 1.2,
                    opening_radius: float = 12.0,
                    idw_k: int = 3,
                    batch_size: int = 500000) -> GroundResult:
    """
    阶段一：高适应性地面物理建模与剥离 (Ground Separation)
    执行地形自适应局部滤波剥离地面
    
    Parameters:
    -----------
    points : np.ndarray
        (N, 3) 空间点云坐标 (x, y, z)
    grid_size : float
        网格切片分辨率 (m)，默认 3.0m
    height_threshold : float
        相对高程地面判定门槛 (m)，默认 1.2m
    opening_radius : float
        形态学开运算搜索半径 (m)，默认 12.0m
    idw_k : int
        IDW 邻域插值点数，默认 3
    batch_size : int
        内存分块处理批大小，默认 500,000
        
    Returns:
    --------
    result : GroundResult
        包含 is_ground, ground_idx, off_ground_idx, off_ground_pts, rel_z，支持元组解包
    """
    gx = (points[:, 0] / grid_size).astype(np.int32)
    gy = (points[:, 1] / grid_size).astype(np.int32)
    g_coords = np.column_stack((gx, gy))
    unique_grids, inv = np.unique(g_coords, axis=0, return_inverse=True)
    
    grid_mins = np.full(len(unique_grids), np.inf)
    np.minimum.at(grid_mins, inv, points[:, 2])
    
    grid_centers = unique_grids * grid_size + grid_size / 2.0
    grid_tree = cKDTree(grid_centers)
    
    # 形态学开运算 (Morphological Opening) 提取真实地面
    neighbors_list = grid_tree.query_ball_point(grid_centers, r=opening_radius)
    
    eroded_z = np.zeros(len(grid_centers))
    for i, neighbors in enumerate(neighbors_list):
        eroded_z[i] = np.min(grid_mins[neighbors])
        
    opened_z = np.zeros(len(grid_centers))
    for i, neighbors in enumerate(neighbors_list):
        opened_z[i] = np.max(eroded_z[neighbors])
    
    # IDW 插值平滑，基于真实的开运算地面 (内存分块防护 Chunking Query)
    interp_tree = cKDTree(grid_centers)
    num_pts = len(points)
    ground_z = np.zeros(num_pts, dtype=np.float32)
    
    for start_idx in range(0, num_pts, batch_size):
        end_idx = min(start_idx + batch_size, num_pts)
        pts_chunk = points[start_idx:end_idx, :2]
        dists, idxs = interp_tree.query(pts_chunk, k=idw_k)
        dists = np.maximum(dists, 1e-3)
        weights = 1.0 / dists
        weights /= np.sum(weights, axis=1, keepdims=True)
        ground_z[start_idx:end_idx] = np.sum(opened_z[idxs] * weights, axis=1)
    
    is_ground = (points[:, 2] - ground_z) < height_threshold
    ground_idx = np.where(is_ground)[0]
    off_ground_idx = np.where(~is_ground)[0]
    off_ground_pts = points[off_ground_idx]
    
    rel_z = off_ground_pts[:, 2] - ground_z[off_ground_idx]
    
    return GroundResult(
        is_ground=is_ground,
        ground_idx=ground_idx,
        off_ground_idx=off_ground_idx,
        off_ground_pts=off_ground_pts,
        rel_z=rel_z
    )
