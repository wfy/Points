from dataclasses import dataclass, field
from enum import IntEnum

class ClassificationCode(IntEnum):
    """
    ASPRS LAS 1.2-1.4 & 国家电网点云分类标准编码
    """
    UNCLASSIFIED = 1
    GROUND = 2
    LOW_VEGETATION = 3
    MEDIUM_VEGETATION = 4
    HIGH_VEGETATION = 5
    WIRE_GUARD = 13          # 地线 / 避雷线 / OPGW
    WIRE_CONDUCTOR = 14      # 导线 / 相导线
    TRANSMISSION_TOWER = 15  # 输电铁塔 / 钢管塔 / 杆塔
    INSULATOR = 16           # 绝缘子 / 连接金具
    HIGH_NOISE = 18          # 高噪点 / 悬空杂波

@dataclass
class GroundConfig:
    grid_size: float = 3.0
    height_threshold: float = 1.2
    opening_radius: float = 12.0
    idw_k: int = 3
    idw_batch_size: int = 500000

@dataclass
class TowerConfig:
    t_grid_size: float = 2.0
    min_tower_rel_z: float = 22.0
    min_pts_count: int = 500
    continuity_ratio: float = 0.65
    nms_radius: float = 25.0
    high_voltage_min_z: float = 28.0
    delta_h_relief: float = 8.0
    arm_ransac_trials: int = 100
    arm_ransac_inlier_thresh: float = 0.5
    voltage_mode: str = 'auto'               # 'auto', 'high_voltage', 'distribution'
    allow_distribution_poles: bool = False   # 是否放宽门槛支持 10kV~110kV 配电单双杆

@dataclass
class PowerlineConfig:
    min_rel_z: float = 6.0
    seed_grid_size: float = 0.8
    pca_radius: float = 2.0
    linearity_thresh: float = 0.82
    arm_linearity_thresh: float = 0.65
    bundle_adapt_linearity_thresh: float = 0.70  # 分裂导线自适应粗筛门槛
    voxel_cluster_size: float = 0.5
    cluster_conn_radius: float = 2.0
    tracking_step: float = 3.0
    max_tracking_steps: int = 60
    use_catenary_tracking: bool = True          # 是否启用 3D 悬链线物理轨道追踪
    enable_bundle_conductor_adapt: bool = True  # 是否启用分裂导线多尺度自适应
    extract_insulator_class: bool = False       # 禁用绝缘子抽取
    enable_tension_topology_solver: bool = False # 禁用耐张拓扑解析

@dataclass
class TopologyConfig:
    top_depth: float = 6.0
    cable_search_radius: float = 2.5
    min_anchor_cable_points: int = 5
    enable_bidirectional_weight: bool = True   # 是否启用双向加权校验

@dataclass
class CorridorConfig:
    split_spans: bool = False                  # 是否将点云按两塔一档切分为多段 LAS
    corridor_half_width: float = 30.0          # 走廊半宽 (m)，全宽 60m 满足 110kV~500kV 规范
    buffer_length: float = 12.0                # 塔位纵向外延重叠保护区长度 (m)

@dataclass
class ExportConfig:
    safe_overwrite: bool = True
    auto_rename_on_conflict: bool = True
    force_kill_viewer: bool = False
    open_qtmodeler: bool = True

@dataclass
class PipelineConfig:
    ground: GroundConfig = field(default_factory=GroundConfig)
    tower: TowerConfig = field(default_factory=TowerConfig)
    powerline: PowerlineConfig = field(default_factory=PowerlineConfig)
    topology: TopologyConfig = field(default_factory=TopologyConfig)
    corridor: CorridorConfig = field(default_factory=CorridorConfig)
    export: ExportConfig = field(default_factory=ExportConfig)

DEFAULT_CONFIG = PipelineConfig()
