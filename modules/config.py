from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import Optional, Set

class PipelineStage(str, Enum):
    """
    点云分类流水线执行生命周期阶段枚举
    """
    GROUND = "ground"
    TOWER = "tower"
    WIRE = "wire"
    TOPOLOGY = "topology"
    EXPORT = "export"

    @classmethod
    def from_string(cls, val: str) -> "PipelineStage":
        clean = val.strip().lower()
        for member in cls:
            if member.value == clean or member.name.lower() == clean:
                return member
        raise ValueError(f"Unknown PipelineStage: '{val}'")

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
    min_tower_rel_z: float = 28.0
    min_pts_count: int = 500
    continuity_ratio: float = 0.65
    nms_radius: float = 25.0
    high_voltage_min_z: float = 28.0
    delta_h_relief: float = 8.0
    arm_ransac_trials: int = 100
    arm_ransac_inlier_thresh: float = 0.5
    voltage_mode: str = 'auto'               # 'auto', 'high_voltage', 'distribution'
    allow_distribution_poles: bool = False   # 是否放宽门槛支持 10kV~110kV 配电单双杆
    lowest_arm_downward_ratio: float = 0.10  # 最下方横担高度默认下降杆塔实际高度的比例 (默认 10%)
    base_2m_cutoff_ratio: float = 2.0        # 地面上方 2 米处四棱台宽度硬截断倍数 (默认 2.0 倍)
    frustum_slope_scale: float = 1.20        # 四棱台放坡斜率放大系数 (默认 1.20)
    frustum_upward_angle: float = 80.0       # 以塔基正方形为底面向上延伸的斜率倾角 (默认 80 度)
    tracking_search_angle: float = 60.0      # 自顶向下逐层跟踪搜索斜率角度 (默认 60 度)
    tracking_expand_ratio: float = 1.10      # 逐层递推边长放大倍数 (默认 1.10 倍)
    tracking_max_slope: float = 0.15         # 单层最大物理放坡斜率保护 (默认 0.15 m/m)
    layer_max_slope_rate: float = 0.08       # 真实铁塔物理放坡增长率基准值 (默认 0.08 m/m)
    # 0-1(0_1) Tower 2 黄金基准点与单塔净高线性倍数放缩模型
    base_anchor_height: float = 30.64        # 黄金基准塔净高 H0 (30.64m)
    base_anchor_slope_rate: float = 0.0811   # 黄金基准放坡率 slope0 (0.0811 m/m)
    base_anchor_shell_thick: float = 1.52    # 黄金基准外框厚度 shell0 (1.52m)
    base_anchor_margin: float = 0.61         # 黄金基准基础裕量 margin0 (0.61m)

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
    enable_topdown_prior: bool = True           # 是否启用杆塔先验引导的自顶向下导线提取
    wire_seed_l3_max: float = 0.30              # 单导线局部截面尺度 sqrt(l3) 上限 (m)
    wire_seed_l3_bundle_max: float = 0.60       # 分裂导线局部截面尺度 sqrt(l3) 上限 (m)
    wire_seed_density_max: int = 60             # 分裂导线邻域点数上限
    cluster_weight_v: float = 2.0               # 体素聚类垂直权重
    catenary_a_min: float = 300.0               # 悬链线参数 a 最小值
    catenary_a_max: float = 4000.0              # 悬链线参数 a 最大值
    min_sag_ratio: float = 0.005                # 悬链线最小弧垂比
    max_sag_ratio: float = 0.08                 # 悬链线最大弧垂比
    enable_jumper_extraction: bool = True       # 是否启用耐张塔横担下方跳线专用弧段提取
    enable_hang_point_bridging: bool = True     # 是否启用横担耐张挂点端头导线引桥吸附
    hang_point_bridge_radius: float = 2.5       # 挂点引桥吸附搜索半径 (m)

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
class PipelineExecutionConfig:
    stop_after: Optional[PipelineStage] = None
    stages: Optional[Set[PipelineStage]] = None

@dataclass
class PipelineConfig:
    pipeline: PipelineExecutionConfig = field(default_factory=PipelineExecutionConfig)
    ground: GroundConfig = field(default_factory=GroundConfig)
    tower: TowerConfig = field(default_factory=TowerConfig)
    powerline: PowerlineConfig = field(default_factory=PowerlineConfig)
    topology: TopologyConfig = field(default_factory=TopologyConfig)
    corridor: CorridorConfig = field(default_factory=CorridorConfig)
    export: ExportConfig = field(default_factory=ExportConfig)

DEFAULT_CONFIG = PipelineConfig()
