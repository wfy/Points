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
    abs_z_boundary: float = 0.0                                           # 绝对物理水平分界标高
    abs_base_ground_z: float = 0.0                                        # 塔基绝对物理标高
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
            'abs_z_boundary': self.abs_z_boundary,
            'abs_base_ground_z': self.abs_base_ground_z,
            'w_trunk0': self.w_trunk0,
            'confidence': self.confidence,
            'pts_idx': self.pts_idx
        }

@dataclass
class WireCluster:
    """
    导线分段体素连通簇实体 (含可选三维物理悬链线模型与拓扑标识)
    """
    members: List[int]
    center: np.ndarray
    dir: np.ndarray
    span: float
    linearity: float
    min_var: float
    catenary: Optional[Any] = None
    line_id: int = 0
    global_indices: np.ndarray = field(default_factory=lambda: np.array([], dtype=int))
    is_suspect: bool = False
    is_jumper: bool = False

    def __getitem__(self, key: str) -> Any:
        if hasattr(self, key):
            return getattr(self, key)
        raise KeyError(f"WireCluster has no attribute '{key}'")

    def __setitem__(self, key: str, value: Any):
        setattr(self, key, value)

    def get(self, key: str, default: Any = None) -> Any:
        return getattr(self, key, default)

    def to_dict(self) -> dict:
        return {
            'members': self.members,
            'center': self.center,
            'dir': self.dir,
            'span': self.span,
            'linearity': self.linearity,
            'min_var': self.min_var,
            'catenary': self.catenary,
            'line_id': self.line_id,
            'global_indices': self.global_indices,
            'is_suspect': self.is_suspect,
            'is_jumper': self.is_jumper
        }

@dataclass
class WireExtractionResult:
    """
    导线提取阶段的领域成果容器 (Domain Result Container)
    内部密封并查集与拓扑图结构，对外交付扁平化的 WireCluster 集合，并保持 100% 向下兼容
    """
    wires: List[WireCluster] = field(default_factory=list)
    cable_indices: np.ndarray = field(default_factory=lambda: np.array([], dtype=int))
    jumper_indices: np.ndarray = field(default_factory=lambda: np.array([], dtype=int))
    insulator_indices: np.ndarray = field(default_factory=lambda: np.array([], dtype=int))
    metadata: Dict[str, Any] = field(default_factory=dict)
    _cached_point_line_id: Optional[np.ndarray] = field(default=None, repr=False)
    _custom_find_line_func: Optional[Callable[[int], int]] = field(default=None, repr=False)
    _custom_suspect_line_ids: Optional[Set[int]] = field(default=None, repr=False)

    def __init__(
        self,
        wires: Optional[List[WireCluster]] = None,
        cable_indices: Optional[np.ndarray] = None,
        jumper_indices: Optional[np.ndarray] = None,
        insulator_indices: Optional[np.ndarray] = None,
        metadata: Optional[Dict[str, Any]] = None,
        # 兼容旧版参数:
        cable_pts_idx: Optional[np.ndarray] = None,
        point_line_id: Optional[np.ndarray] = None,
        all_confirmed: Optional[List[WireCluster]] = None,
        suspect_line_ids: Optional[Set[int]] = None,
        find_line_func: Optional[Callable[[int], int]] = None,
        insulator_pts_idx: Optional[np.ndarray] = None,
        **kwargs
    ):
        self.wires = wires if wires is not None else (all_confirmed if all_confirmed is not None else [])
        self.cable_indices = cable_indices if cable_indices is not None else (cable_pts_idx if cable_pts_idx is not None else np.array([], dtype=int))
        self.jumper_indices = jumper_indices if jumper_indices is not None else np.array([], dtype=int)
        self.insulator_indices = insulator_indices if insulator_indices is not None else (insulator_pts_idx if insulator_pts_idx is not None else np.array([], dtype=int))
        self.metadata = metadata if metadata is not None else {}
        self._cached_point_line_id = point_line_id
        self._custom_find_line_func = find_line_func
        self._custom_suspect_line_ids = suspect_line_ids

    @property
    def cable_pts_idx(self) -> np.ndarray:
        return self.cable_indices

    @property
    def all_confirmed(self) -> List[WireCluster]:
        return self.wires

    @property
    def insulator_pts_idx(self) -> np.ndarray:
        return self.insulator_indices

    @property
    def suspect_line_ids(self) -> Set[int]:
        if self._custom_suspect_line_ids is not None:
            return self._custom_suspect_line_ids
        return {w.line_id for w in self.wires if w.is_suspect and w.line_id > 0}

    @property
    def find_line_func(self) -> Callable[[int], int]:
        if self._custom_find_line_func is not None:
            return self._custom_find_line_func
        return lambda x: x

    @property
    def point_line_id(self) -> np.ndarray:
        """
        向后兼容属性：延迟按需生成全局点到线路 ID 的映射向量
        """
        if self._cached_point_line_id is not None:
            return self._cached_point_line_id
        if 'point_line_id' in self.metadata:
            return self.metadata['point_line_id']

        if len(self.cable_indices) == 0:
            return np.array([], dtype=int)

        max_idx = int(np.max(self.cable_indices)) if len(self.cable_indices) > 0 else 0
        arr_len = max(max_idx + 1, self.metadata.get('num_points', max_idx + 1))
        arr = np.zeros(arr_len, dtype=int)
        for w in self.wires:
            if len(w.global_indices) > 0 and w.line_id > 0:
                arr[w.global_indices] = w.line_id
        self._cached_point_line_id = arr
        return arr

    def __iter__(self) -> Iterator[Any]:
        # 兼容 5 元组或 6 元组解包
        return iter((
            self.cable_indices,
            self.point_line_id,
            self.wires,
            self.suspect_line_ids,
            self.find_line_func,
            self.insulator_indices
        ))

    def __getitem__(self, index: int) -> Any:
        return (
            self.cable_indices,
            self.point_line_id,
            self.wires,
            self.suspect_line_ids,
            self.find_line_func,
            self.insulator_indices
        )[index]

# 向后兼容类型别名
ExtractionResult = WireExtractionResult

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
    spans: Optional[List[SpanSegment]] = None
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
        info = {
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
        if self.spans is not None:
            info['span_count'] = len(self.spans)
        return info
