import os
import time
import numpy as np
import laspy
from dataclasses import dataclass, field
from typing import List, Union, Optional
from modules.models import TowerEntity
from modules.config import PipelineConfig, DEFAULT_CONFIG
from modules.ground_separator import separate_ground
from modules.tower_detector import detect_towers

@dataclass
class SpanSegment:
    """两塔一档走廊切片数据模型"""
    span_index: int                       # 档段顺序编号 (0-based)
    tower_from_idx: int                   # 起始杆塔全局索引
    tower_to_idx: int                     # 终止杆塔全局索引
    tower_from_pos: np.ndarray            # 起始杆塔 2D 坐标 [cx, cy]
    tower_to_pos: np.ndarray              # 终止杆塔 2D 坐标 [cx, cy]
    span_length: float                    # 档距水平跨度 (m)
    point_indices: np.ndarray             # 属于本档走廊的点云全局索引
    output_las_path: str = ""             # 输出 LAS 文件完整路径


def order_towers_along_line(tower_infos: List[Union[TowerEntity, dict]]) -> List[int]:
    """
    沿输电线路主干走向，将无序/离散的杆塔拓扑排序为线性链条 (T1 -> T2 -> ... -> Tn)
    """
    n_towers = len(tower_infos)
    if n_towers <= 2:
        return list(range(n_towers))
        
    t_xy = np.array([[t['cx'], t['cy']] for t in tower_infos])
    
    # 1. 计算两两杆塔之间的欧氏距离矩阵
    dists = np.linalg.norm(t_xy[:, None, :] - t_xy[None, :, :], axis=2)
    
    # 2. 找到相距最远的两个杆塔作为线路的两个端点 (Start / End)
    start_idx, end_idx = np.unravel_index(np.argmax(dists), dists.shape)
    
    # 3. 从起点出发，通过最近邻贪心遍历依次串联整条走廊
    ordered = [start_idx]
    visited = set(ordered)
    curr = start_idx
    
    while len(ordered) < n_towers:
        # 在未访问节点中寻找距离当前杆塔最近的下一个铁塔
        unvisited = [i for i in range(n_towers) if i not in visited]
        next_t = min(unvisited, key=lambda idx: dists[curr, idx])
        ordered.append(next_t)
        visited.add(next_t)
        curr = next_t
        
    return ordered


def cut_corridors_by_spans(points: np.ndarray,
                           tower_infos: List[Union[TowerEntity, dict]],
                           corridor_half_width: float = 30.0,
                           buffer_length: float = 12.0) -> List[SpanSegment]:
    """
    基于双塔定向走廊包围盒 (Corridor OBB) 向量化快速切分各跨档距点云
    """
    n_towers = len(tower_infos)
    if n_towers < 2:
        return []
        
    ordered_indices = order_towers_along_line(tower_infos)
    t_xy = np.array([[tower_infos[i]['cx'], tower_infos[i]['cy']] for i in range(n_towers)])
    pts_xy = points[:, :2]
    
    spans = []
    for span_i in range(len(ordered_indices) - 1):
        idx_a = ordered_indices[span_i]
        idx_b = ordered_indices[span_i + 1]
        
        p_a = t_xy[idx_a]
        p_b = t_xy[idx_b]
        
        v_span = p_b - p_a
        span_len = float(np.linalg.norm(v_span))
        if span_len < 1e-3:
            continue
            
        u_span = v_span / span_len                      # 沿线主轴方向单位向量
        n_span = np.array([-u_span[1], u_span[0]])     # 走廊横向法向单位向量
        
        # 向量化投影计算每个点相对起点 P_a 的 (纵向投影 s, 横向偏距 d_perp)
        delta_p = pts_xy - p_a
        s_proj = delta_p @ u_span
        d_perp = np.abs(delta_p @ n_span)
        
        # OBB 定向走廊范围判定
        in_corridor_mask = (s_proj >= -buffer_length) & \
                           (s_proj <= (span_len + buffer_length)) & \
                           (d_perp <= corridor_half_width)
                           
        span_pt_indices = np.where(in_corridor_mask)[0]
        
        spans.append(SpanSegment(
            span_index=span_i,
            tower_from_idx=idx_a,
            tower_to_idx=idx_b,
            tower_from_pos=p_a,
            tower_to_pos=p_b,
            span_length=span_len,
            point_indices=span_pt_indices
        ))
        
    return spans


def export_split_spans(las_classified_path: str,
                       spans: List[SpanSegment],
                       output_dir: Optional[str] = None) -> List[str]:
    """
    将切分好的各档点云高保真无损输出为独立的 LAS 文件
    """
    if not os.path.exists(las_classified_path) or len(spans) == 0:
        return []
        
    if output_dir is None:
        base_dir = os.path.dirname(las_classified_path)
        output_dir = os.path.join(base_dir, "spans")
        
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"-> 正在将成果按两塔一档切分为 {len(spans)} 份独立 LAS 文件...")
    las = laspy.read(las_classified_path)
    base_name = os.path.splitext(os.path.basename(las_classified_path))[0]
    clean_base = base_name.replace("_sign", "")
    
    generated_paths = []
    for span in spans:
        if len(span.point_indices) == 0:
            continue
            
        span_filename = f"{clean_base}_Span{span.span_index + 1}_#T{span.tower_from_idx + 1}-#T{span.tower_to_idx + 1}.las"
        span_out_path = os.path.join(output_dir, span_filename)
        
        span_las = las[span.point_indices]
        span_las.write(span_out_path)
        
        span.output_las_path = os.path.abspath(span_out_path)
        generated_paths.append(span.output_las_path)
        print(f"   [Span {span.span_index + 1}] 输出档段: '{os.path.basename(span_out_path)}' (包含点数: {len(span.point_indices):,} 点 | 档距: {span.span_length:.1f}m)")
        
    print(f"[Done] 成功切分并导出 {len(generated_paths)} 个单档 LAS 文件至目录: '{output_dir}'")
    return generated_paths


def split_raw_corridor(las_input_path: str,
                       output_dir: Optional[str] = None,
                       config: Optional[PipelineConfig] = None) -> List[str]:
    """
    【独立步骤一：纯走廊切片预处理 (Split-Only Preprocessing)】
    仅执行快速地面初筛与 3D 体素杆塔锁定（耗时 2~3 秒），直接将原始未分类大点云切分为各档独立 LAS。
    
    Parameters:
    -----------
    las_input_path : str
        原始整线输入 LAS/LAZ 点云路径
    output_dir : str, optional
        原始切片输出目录，默认保存在原文件所在目录下的 'spans_raw/'
    config : PipelineConfig, optional
        配置实例
        
    Returns:
    --------
    generated_paths : List[str]
        生成的各档原始未分类 LAS 文件绝对路径列表
    """
    if config is None:
        config = DEFAULT_CONFIG
        
    t_start = time.time()
    print(f"[Split-Only] 开始执行纯走廊切片预处理: {las_input_path}")
    
    # 0. 读取原始点云
    las = laspy.read(las_input_path)
    points = np.vstack((las.x, las.y, las.z)).T
    num_points = len(points)
    print(f"   读取原始点云总数: {num_points:,} 点")
    
    # 1. 快速地面物理分离
    t1 = time.time()
    print("-> 1/2 执行快速地面物理分离与高程归一化...")
    _, _, off_ground_idx, off_ground_pts, rel_z = separate_ground(
        points,
        grid_size=config.ground.grid_size,
        height_threshold=config.ground.height_threshold,
        opening_radius=config.ground.opening_radius,
        idw_k=config.ground.idw_k,
        batch_size=config.ground.idw_batch_size
    )
    print(f"   地面剥离完成 (耗时: {time.time() - t1:.2f}s)")
    
    # 2. 快速 3D 体素锁定全线铁塔骨架
    t2 = time.time()
    print("-> 2/2 执行 3D 体素连续性快速提取全线杆塔...")
    _, _, _, tower_infos = detect_towers(
        off_ground_pts=off_ground_pts,
        rel_z=rel_z,
        off_ground_idx=off_ground_idx,
        t_grid_size=config.tower.t_grid_size,
        config=config
    )
    n_towers = len(tower_infos)
    print(f"   杆塔锁定完成 (耗时: {time.time() - t2:.2f}s) | 锁定铁塔: {n_towers} 座")
    
    if n_towers < 2:
        print("[Warning] 检测到的铁塔数量少于 2 座，无法构成跨越档段，切分取消。")
        return []
        
    # 3. 定向 OBB 走廊包围盒切分
    spans = cut_corridors_by_spans(
        points=points,
        tower_infos=tower_infos,
        corridor_half_width=config.corridor.corridor_half_width,
        buffer_length=config.corridor.buffer_length
    )
    
    if output_dir is None:
        base_dir = os.path.dirname(las_input_path)
        output_dir = os.path.join(base_dir, "spans_raw")
    os.makedirs(output_dir, exist_ok=True)
    
    base_name = os.path.splitext(os.path.basename(las_input_path))[0]
    generated_paths = []
    
    for span in spans:
        if len(span.point_indices) == 0:
            continue
        span_filename = f"{base_name}_Span{span.span_index + 1}_#T{span.tower_from_idx + 1}-#T{span.tower_to_idx + 1}.las"
        span_out_path = os.path.join(output_dir, span_filename)
        
        # 零拷贝切片，保留 100% 原始数据属性
        span_las = las[span.point_indices]
        span_las.write(span_out_path)
        
        span.output_las_path = os.path.abspath(span_out_path)
        generated_paths.append(span.output_las_path)
        print(f"   [Span {span.span_index + 1}] 导出原始单档: '{os.path.basename(span_out_path)}' (点数: {len(span.point_indices):,} | 档距: {span.span_length:.1f}m)")
        
    total_time = time.time() - t_start
    print(f"[Done] 纯切片预处理完成！总耗时: {total_time:.2f} 秒 | 导出单档文件: {len(generated_paths)} 份")
    return generated_paths
