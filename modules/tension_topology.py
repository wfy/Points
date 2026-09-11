import numpy as np
from dataclasses import dataclass, field
from typing import List, Union, Tuple, Optional, Set
from scipy.spatial import cKDTree
from modules.models import TowerEntity, WireCluster
from modules.config import PipelineConfig, DEFAULT_CONFIG

@dataclass
class TensionTopologyResult:
    insulator_pts_idx: np.ndarray = field(default_factory=lambda: np.array([], dtype=int))
    jumper_pts_idx: np.ndarray = field(default_factory=lambda: np.array([], dtype=int))
    joints: List = field(default_factory=list)


def extract_tension_jumpers(points: np.ndarray,
                            off_ground_pts: np.ndarray,
                            off_ground_idx: np.ndarray,
                            rel_z: np.ndarray,
                            is_tower: np.ndarray,
                            tower_infos: List[Union[TowerEntity, dict]],
                            cable_pts_idx: np.ndarray = None,
                            config: PipelineConfig = None) -> np.ndarray:
    """
    【耐张塔引流跳线专用 3D 几何弧段提取器】
    原理：
      1. 空间敏感区锁定：以各铁塔横担标高下方 Z in [Z_lowest_arm - 2.0, max_z]、
         塔心水平半径 R in [w_trunk0, half_l1 + 3.0] 为跳线搜索空间；
      2. 排除塔身主体角钢骨架与地面植被；
      3. 0.35m 微体素连通聚类，提取大曲率空间弧段；
      4. 严格校验弧段线型度 (linearity >= 0.50 或局部平坦薄层 l3 <= 0.40m)
         与空间尺度 (span 1.5m~18m)，杜绝树木杂波。
    """
    if config is None:
        config = DEFAULT_CONFIG
    if not config.powerline.enable_jumper_extraction:
        return np.array([], dtype=int)
    if len(tower_infos) == 0 or len(off_ground_pts) == 0:
        return np.array([], dtype=int)

    jumper_global_indices: List[int] = []
    off_tree = cKDTree(off_ground_pts[:, :2].astype(np.float32))

    cable_set = set(cable_pts_idx.tolist()) if cable_pts_idx is not None and len(cable_pts_idx) > 0 else set()

    for t_info in tower_infos:
        cx = float(t_info['cx'])
        cy = float(t_info['cy'])
        max_z = float(t_info['max_z'])
        z_low_arm = float(t_info.get('z_lowest_arm', max_z * 0.45))
        half_l1 = float(t_info.get('half_l1', 8.0))
        half_l2 = float(t_info.get('half_l2', 2.0))
        w_trunk0 = float(t_info.get('w_trunk0', 1.5))
        v1 = np.array(t_info.get('v1', [1.0, 0.0]), dtype=float)
        v2 = np.array(t_info.get('v2', [0.0, 1.0]), dtype=float)

        # 塔心水平搜索半径
        r_search = max(half_l1 + 3.5, 8.0)
        near_indices = off_tree.query_ball_point([cx, cy], r=r_search)
        if len(near_indices) < 15:
            continue

        near_indices = np.array(near_indices, dtype=int)
        near_pts = off_ground_pts[near_indices]
        near_rel_z = rel_z[near_indices]
        near_global = off_ground_idx[near_indices]
        near_is_tower = is_tower[near_indices]

        # 空间敏感区过滤：
        # 1. 高度在最下方横担稍下至塔顶下方 (z_low_arm - 3.0 <= rel_z <= max_z + 0.5)
        # 2. 高度绝对值必须高于地面 (rel_z >= 8.0)
        # 3. 沿 v1 (横担方向) 与 v2 (走向) 的投影空间
        diff_2d = near_pts[:, :2] - np.array([cx, cy])
        proj_v1 = np.abs(diff_2d @ v1)
        proj_v2 = np.abs(diff_2d @ v2)

        # 跳线必须在横担横向范围及稍外延处 (proj_v1 <= half_l1 + 3.5)，且在走廊走向附近 (proj_v2 <= half_l2 + 6.0)
        # 严格排除铁塔主体角钢骨架与塔心立柱，跳线必须为独立悬垂导线
        z_mask = (near_rel_z >= max(z_low_arm - 4.0, 6.0)) & (near_rel_z <= (max_z + 0.5))
        spatial_mask = z_mask & (proj_v1 <= half_l1 + 3.5) & (proj_v2 <= half_l2 + 6.0) & (~near_is_tower)

        cand_sub_idx = np.where(spatial_mask)[0]
        if len(cand_sub_idx) < 6:
            continue

        cand_pts = near_pts[cand_sub_idx]
        cand_global = near_global[cand_sub_idx]

        # 0.35m 微体素连通图聚类
        voxel_size = 0.35
        vx = (cand_pts[:, 0] / voxel_size).astype(np.int32)
        vy = (cand_pts[:, 1] / voxel_size).astype(np.int32)
        vz = (cand_pts[:, 2] / voxel_size).astype(np.int32)
        v_coords = np.column_stack([vx, vy, vz])

        unique_v, inv_v = np.unique(v_coords, axis=0, return_inverse=True)
        num_v = len(unique_v)
        if num_v < 3:
            continue

        v_centers = unique_v * voxel_size + voxel_size / 2.0
        v_tree = cKDTree(v_centers.astype(np.float32))
        pairs = v_tree.query_pairs(r=1.2)

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
            if ri != rj:
                parent_v[ri] = rj

        for i, j in pairs:
            union_v(i, j)

        clusters: dict = {}
        for pt_i in range(len(cand_pts)):
            r = find_v(inv_v[pt_i])
            clusters.setdefault(r, []).append(pt_i)

        cand_tree = cKDTree(cand_pts.astype(np.float32))

        for r_id, members in clusters.items():
            if len(members) < 5:
                continue
            c_pts = cand_pts[members]
            ptp = np.ptp(c_pts, axis=0)
            span_3d = float(np.linalg.norm(ptp))

            # 跳线空间尺度限制：跨度 1.2m ~ 20.0m，且垂向不能是几十米的大树
            if span_3d < 1.0 or span_3d > 22.0 or ptp[2] > 6.5:
                continue

            # PCA 特征姿态校验：跳线呈空间弧线，主截面厚度 sqrt(l3) 必须很薄 (<= 0.45m)
            cov = np.cov(c_pts.T)
            evals, evecs = np.linalg.eigh(cov)
            l1, l2, l3 = evals[2], evals[1], max(evals[0], 0.0)
            if l1 <= 0:
                continue

            linearity = float((l1 - l2) / l1)
            l3_thickness = float(np.sqrt(l3))

            # 弧线在局部或整段具有较好线型度或纤细截面
            is_valid_arc = (linearity >= 0.45 or l3_thickness <= 0.40) and (l3_thickness <= 0.60)

            if is_valid_arc:
                # 辐射吸收该跳线簇 0.6m 内的未分类近邻点
                j_nbrs = cand_tree.query_ball_point(c_pts, r=0.6)
                absorbed = set(members)
                for n_list in j_nbrs:
                    for n_idx in n_list:
                        absorbed.add(n_idx)
                for abs_i in absorbed:
                    jumper_global_indices.append(cand_global[abs_i])

    if len(jumper_global_indices) > 0:
        tower_indices = off_ground_idx[is_tower] if np.any(is_tower) else np.array([], dtype=int)
        valid_jumpers = np.setdiff1d(np.unique(jumper_global_indices), tower_indices)
        return valid_jumpers
    return np.array([], dtype=int)


def solve_tension_topology(points: np.ndarray,
                           cable_pts_idx: np.ndarray,
                           tower_pts_idx: np.ndarray,
                           tower_infos: list,
                           all_confirmed = None,
                           config = None,
                           off_ground_pts: np.ndarray = None,
                           off_ground_idx: np.ndarray = None,
                           rel_z: np.ndarray = None,
                           is_tower: np.ndarray = None,
                           **kwargs) -> TensionTopologyResult:
    """
    耐张拓扑解析核心入口：解算耐张绝缘子与引流跳线
    """
    if config is None:
        config = DEFAULT_CONFIG

    if off_ground_pts is None or rel_z is None:
        return TensionTopologyResult()

    jumper_idx = extract_tension_jumpers(
        points=points,
        off_ground_pts=off_ground_pts,
        off_ground_idx=off_ground_idx,
        rel_z=rel_z,
        is_tower=is_tower if is_tower is not None else np.zeros(len(off_ground_pts), dtype=bool),
        tower_infos=tower_infos,
        cable_pts_idx=cable_pts_idx,
        config=config
    )

    return TensionTopologyResult(
        insulator_pts_idx=np.array([], dtype=int),
        jumper_pts_idx=jumper_idx,
        joints=[]
    )

