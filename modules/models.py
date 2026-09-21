from dataclasses import dataclass, field
from typing import List, Set, Callable, Optional, Any, Iterator, Dict
import numpy as np

@dataclass
class GroundResult:
    """
    地面自适应滤波剥离结果数据结构 (支持元组解包以保持 100% 向后兼容)
    """
    is_ground: np.ndarray
    ground_idx: np.ndarray
    off_ground_idx: np.ndarray
    off_ground_pts: np.ndarray
    rel_z: np.ndarray

    def __iter__(self) -> Iterator[Any]:
        return iter((
            self.is_ground,
            self.ground_idx,
            self.off_ground_idx,
            self.off_ground_pts,
            self.rel_z
        ))

    def __getitem__(self, index: int) -> Any:
        return (
            self.is_ground,
            self.ground_idx,
            self.off_ground_idx,
            self.off_ground_pts,
            self.rel_z
        )[index]

@dataclass
class TowerEntity:
    """
    检测锁定的输电杆塔结构化领域实体
    """
    cx: float
    cy: float
    max_z: float
    abs_max_z: float = 0.0
    v1: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0]))  # 横担主轴方向
    v2: np.ndarray = field(default_factory=lambda: np.array([0.0, 1.0]))  # 线路/塔厚走向
    half_l1: float = 8.0                                                  # 横担半长
    half_l2: float = 2.5                                                  # 塔侧厚度半宽
    d_diag: float = 16.0                                                  # 塔顶横担对角线跨度
    z_lowest_arm: float = 10.0                                            # 最低横担挂点相对高度
    w_trunk0: float = 1.8                                                 # 塔身初始半宽
    confidence: float = 1.0                                               # 杆塔骨架几何置信度 [0.0, 1.0]
    pts_idx: np.ndarray = field(default_factory=lambda: np.array([], dtype=int)) # 铁塔在非地面点中的局部索引

    def __getitem__(self, key: str) -> Any:
        if hasattr(self, key):
            return getattr(self, key)
        raise KeyError(f"TowerEntity has no attribute '{key}'")

    def __setitem__(self, key: str, value: Any):
        setattr(self, key, value)

    def get(self, key: str, default: Any = None) -> Any:
        return getattr(self, key, default)

    def to_dict(self) -> dict:
        return {
            'cx': self.cx,
            'cy': self.cy,
            'max_z': self.max_z,
            'abs_max_z': self.abs_max_z,
            'v1': self.v1,
            'v2': self.v2,
            'half_l1': self.half_l1,
            'half_l2': self.half_l2,
            'd_diag': self.d_diag,
            'z_lowest_arm': self.z_lowest_arm,
            'w_trunk0': self.w_trunk0,
            'confidence': self.confidence,
            'pts_idx': self.pts_idx
        }

@dataclass
class WireCluster:
    """
    导线分段体素连通簇实体 (含可选三维物理悬链线模型)
    """
    members: List[int]
    center: np.ndarray
    dir: np.ndarray
    span: float
    linearity: float
    min_var: float
    catenary: Optional[Any] = None

    def __getitem__(self, key: str) -> Any:
        if hasattr(self, key):
            return getattr(self, key)
        raise KeyError(f"WireCluster has no attribute '{key}'")

    def get(self, key: str, default: Any = None) -> Any:
        return getattr(self, key, default)

@dataclass
class ExtractionResult:
    """
    导线提取与追踪阶段成果容器 (支持元组解包及绝缘子挂点索引)
    """
    cable_pts_idx: np.ndarray
    point_line_id: np.ndarray
    all_confirmed: List[WireCluster]
    suspect_line_ids: Set[int]
    find_line_func: Callable[[int], int]
    insulator_pts_idx: np.ndarray = field(default_factory=lambda: np.array([], dtype=int))

    def __iter__(self) -> Iterator[Any]:
        # 兼容 5 元组或 6 元组解包
        return iter((
            self.cable_pts_idx,
            self.point_line_id,
            self.all_confirmed,
            self.suspect_line_ids,
            self.find_line_func,
            self.insulator_pts_idx
        ))

    def __getitem__(self, index: int) -> Any:
        return (
            self.cable_pts_idx,
            self.point_line_id,
            self.all_confirmed,
            self.suspect_line_ids,
            self.find_line_func,
            self.insulator_pts_idx
        )[index]

@dataclass
class PipelineResult:
    """
    点云分类流水线整体执行成果对象 (Domain Result Container)
    """
    num_points: int
    classification: np.ndarray
    towers: List[TowerEntity] = field(default_factory=list)
    wires: List[WireCluster] = field(default_factory=list)
    stage_timings: Dict[str, float] = field(default_factory=dict)
    ground_result: Optional[GroundResult] = None
    extraction_result: Optional[ExtractionResult] = None
    metadata: Dict[str, Any] = field(default_factory=dict)

    @property
    def ground_indices(self) -> np.ndarray:
        return np.where(self.classification == 2)[0]

    @property
    def tower_indices(self) -> np.ndarray:
        return np.where(self.classification == 15)[0]

    @property
    def wire_indices(self) -> np.ndarray:
        return np.where((self.classification == 14) | (self.classification == 13))[0]

    @property
    def unclassified_indices(self) -> np.ndarray:
        return np.where(self.classification == 1)[0]

    @property
    def total_time(self) -> float:
        return float(sum(self.stage_timings.values()))

    def summary(self) -> dict:
        return {
            'total_points': self.num_points,
            'ground_count': int(len(self.ground_indices)),
            'tower_count': int(len(self.tower_indices)),
            'wire_count': int(len(self.wire_indices)),
            'unclassified_count': int(len(self.unclassified_indices)),
            'towers_detected': len(self.towers),
            'wires_detected': len(self.wires),
            'total_time_s': round(self.total_time, 2),
            'stage_timings': {k: round(v, 3) for k, v in self.stage_timings.items()}
        }
