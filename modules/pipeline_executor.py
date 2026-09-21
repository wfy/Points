import os
import time
from typing import Optional, List, Dict, Tuple, Set
import numpy as np
import laspy

from modules.config import (
    PipelineConfig,
    PipelineStage,
    DEFAULT_CONFIG,
    ClassificationCode
)
from modules.models import (
    GroundResult,
    TowerEntity,
    WireCluster,
    ExtractionResult,
    PipelineResult
)
from modules.ground_separator import separate_ground
from modules.tower_detector import detect_towers
from modules.wire_extractor import WireExtractor
from modules.topdown_wire_extractor import extract_wires_topdown
from modules.topology_validator import validate_tower_topology
from modules.utils import export_colored_las

class PipelineExecutor:
    """
    点云电力线与杆塔分类调度深度核心模块 (Deep Pipeline Executor)
    
    统一调度4个核心算法阶段，对外提供内存态纯计算接缝 run(points) 与便利落盘门面 run_file(in, out)，
    并支持通过 PipelineStage 与 stop_after 进行细粒度断点调试与阶段短路。
    """
    
    def __init__(self, config: Optional[PipelineConfig] = None):
        self.config = config if config is not None else DEFAULT_CONFIG

    def run(self, 
            points: np.ndarray, 
            config: Optional[PipelineConfig] = None,
            verbose: Optional[bool] = None) -> PipelineResult:
        """
        核心纯内存接缝：对空间点云坐标矩阵执行完整的电力线与杆塔提取流水线
        
        Parameters:
        -----------
        points : np.ndarray
            (N, 3) float 点云全局笛卡尔空间坐标
        config : Optional[PipelineConfig]
            本次调度的参数配置，默认使用类初始化时的配置
        verbose : Optional[bool]
            是否输出各阶段执行日志，默认遵循 config.pipeline.verbose
            
        Returns:
        --------
        PipelineResult
            封装各分类代码、索引掩膜、几何实体及分步耗时的领域成果
        """
        cfg = config if config is not None else self.config
        is_verbose = verbose if verbose is not None else getattr(cfg.pipeline, 'verbose', True)
        stop_after = cfg.pipeline.stop_after
        num_points = len(points)
        stage_timings: Dict[str, float] = {}
        
        # 初始化分类标签：默认 Class 1 (Unclassified)
        classification = np.full(num_points, int(ClassificationCode.UNCLASSIFIED), dtype=np.uint8)
        
        # =====================================================================
        # 阶段一：地形自适应局部滤波剥离地面 (Ground Separation)
        # =====================================================================
        if is_verbose:
            print("-> 1/4 执行地形自适应局部滤波剥离地面...")
        t1 = time.time()
        is_ground, ground_idx, off_ground_idx, off_ground_pts, rel_z = separate_ground(
            points,
            grid_size=cfg.ground.grid_size,
            height_threshold=cfg.ground.height_threshold,
            opening_radius=cfg.ground.opening_radius,
            idw_k=cfg.ground.idw_k,
            batch_size=cfg.ground.idw_batch_size
        )
        stage_timings["ground"] = float(time.time() - t1)
        if is_verbose:
            print(f"   阶段一完成 (耗时: {stage_timings['ground']:.2f}s) | 地面点: {len(ground_idx):,} 点 | 剩余目标点: {len(off_ground_idx):,} 点")
        
        ground_res = GroundResult(
            is_ground=is_ground,
            ground_idx=ground_idx,
            off_ground_idx=off_ground_idx,
            off_ground_pts=off_ground_pts,
            rel_z=rel_z
        )
        
        if len(ground_idx) > 0:
            classification[ground_idx] = int(ClassificationCode.GROUND)
            
        # 短路检查：仅执行地面剥离
        if stop_after == PipelineStage.GROUND:
            if is_verbose:
                print("   [提前终止] 已根据配置 stop_after=ground 终止后续阶段")
            return PipelineResult(
                num_points=num_points,
                classification=classification,
                towers=[],
                wires=[],
                stage_timings=stage_timings,
                ground_result=ground_res
            )

        # =====================================================================
        # 阶段二：3D 体素垂直连续性 + 聚类滤波锁定铁塔 (Tower Detection)
        # =====================================================================
        if is_verbose:
            print("-> 2/4 执行 3D 体素垂直连续性 + 聚类滤波锁定铁塔...")
        t2 = time.time()
        is_tower, is_tower_arm, is_near_tower_high_arm, tower_infos = detect_towers(
            off_ground_pts=off_ground_pts,
            rel_z=rel_z,
            off_ground_idx=off_ground_idx,
            t_grid_size=cfg.tower.t_grid_size,
            config=cfg
        )
        stage_timings["tower"] = float(time.time() - t2)
        if is_verbose:
            print(f"   阶段二完成 (耗时: {stage_timings['tower']:.2f}s) | 检测到候选铁塔: {len(tower_infos)} 座")
        
        # 收集有效铁塔点全局索引
        tower_pts_idx = off_ground_idx[is_tower] if np.any(is_tower) else np.array([], dtype=int)
        if len(tower_pts_idx) > 0:
            classification[tower_pts_idx] = int(ClassificationCode.TRANSMISSION_TOWER)
            
        # 收集铁塔横担以下区域（用于后续特征标色参考）
        tower_below_arm_pts_idx = np.array([], dtype=int)
        if len(tower_infos) > 0:
            _parts = []
            for _info in tower_infos:
                _local = _info.get('pts_idx', np.array([], dtype=int))
                if len(_local) == 0:
                    continue
                _z_low = float(_info.get('z_lowest_arm', 10.0))
                _below = _local[rel_z[_local] < _z_low]
                if len(_below) > 0:
                    _parts.append(off_ground_idx[_below])
            if _parts:
                tower_below_arm_pts_idx = np.unique(np.concatenate(_parts))
                
        # 短路检查：仅执行至杆塔检测
        if stop_after == PipelineStage.TOWER:
            if is_verbose:
                print("   [提前终止] 已根据配置 stop_after=tower 终止后续阶段")
            return PipelineResult(
                num_points=num_points,
                classification=classification,
                towers=tower_infos,
                wires=[],
                stage_timings=stage_timings,
                ground_result=ground_res,
                metadata={'tower_below_arm_pts_idx': tower_below_arm_pts_idx}
            )

        # =====================================================================
        # 阶段三：自顶向下双通道导线提取与悬链线物理轨道拟合 (Wire Extraction)
        # =====================================================================
        if is_verbose:
            print("-> 3/4 执行 PCA 特征姿态分析与 3D 悬链线完整连续追踪...")
        t3 = time.time()
        arm_mask = is_tower_arm if is_tower_arm is not None else is_near_tower_high_arm
        wire_extractor = WireExtractor(config=cfg)
        ext_res = wire_extractor.extract(
            points=points,
            off_ground_pts=off_ground_pts,
            off_ground_idx=off_ground_idx,
            rel_z=rel_z,
            is_tower=is_tower,
            is_tower_arm=arm_mask,
            is_near_tower_high_arm=is_near_tower_high_arm,
            tower_infos=tower_infos,
            config=cfg
        )
        stage_timings["wire"] = float(time.time() - t3)
        
        cable_pts_idx = ext_res.cable_indices
        all_confirmed = ext_res.wires
        point_line_id = ext_res.point_line_id
        suspect_line_ids = ext_res.suspect_line_ids
        find_line_func = ext_res.find_line_func

        if is_verbose:
            print(f"   阶段三完成 (耗时: {stage_timings['wire']:.2f}s) | 提取导线点: {len(cable_pts_idx):,} 点 | 聚合线路簇: {len(all_confirmed)} 组")

        # 短路检查：仅执行至导线提取
        if stop_after == PipelineStage.WIRE:
            if is_verbose:
                print("   [提前终止] 已根据配置 stop_after=wire 终止后续阶段")
            if len(tower_pts_idx) > 0 and len(cable_pts_idx) > 0:
                cable_pts_idx = np.setdiff1d(cable_pts_idx, tower_pts_idx)
                point_line_id[tower_pts_idx] = 0
            if len(ground_idx) > 0:
                classification[ground_idx] = int(ClassificationCode.GROUND)
            if len(tower_pts_idx) > 0:
                classification[tower_pts_idx] = int(ClassificationCode.TRANSMISSION_TOWER)
            if len(cable_pts_idx) > 0:
                classification[cable_pts_idx] = int(ClassificationCode.WIRE_CONDUCTOR)
            if len(tower_below_arm_pts_idx) > 0:
                tower_below_arm_pts_idx = np.intersect1d(tower_below_arm_pts_idx, tower_pts_idx)
            tower_arm_pts_idx = off_ground_idx[is_tower_arm & is_tower] if (is_tower_arm is not None and np.any(is_tower_arm & is_tower)) else np.array([], dtype=int)
            return PipelineResult(
                num_points=num_points,
                classification=classification,
                towers=tower_infos,
                wires=all_confirmed,
                stage_timings=stage_timings,
                ground_result=ground_res,
                extraction_result=ext_res,
                metadata={
                    'point_line_id': point_line_id,
                    'suspect_line_ids': suspect_line_ids,
                    'find_line_func': find_line_func,
                    'tower_arm_pts_idx': tower_arm_pts_idx,
                    'tower_below_arm_pts_idx': tower_below_arm_pts_idx
                }
            )

        # =====================================================================
        # 阶段四：导线依附拓扑校验 (Topology Validation)
        # =====================================================================
        t4 = time.time()
        valid_tower_count = len(tower_infos)
        # 若配置启用拓扑双向校验，则执行真塔挂接验证与伪装塔降级
        if getattr(cfg.topology, 'enable_bidirectional_weight', True) and len(tower_infos) > 0:
            validated_tower_idx, valid_tower_count, demoted_pts_idx = validate_tower_topology(
                points=points,
                off_ground_idx=off_ground_idx,
                rel_z=rel_z,
                tower_infos=tower_infos,
                cable_pts_idx=cable_pts_idx,
                config=cfg
            )
            tower_pts_idx = validated_tower_idx
        stage_timings["topology"] = float(time.time() - t4)
        if is_verbose:
            print(f"-> 4/4 杆塔检测确认: 检测到 {valid_tower_count} 座杆塔 (杆塔点数: {len(tower_pts_idx):,})")

        # =====================================================================
        # 成果融合与刚性互斥保障 (Mutual Exclusivity)
        # =====================================================================
        # 1. 铁塔点优先于导线点
        if len(tower_pts_idx) > 0 and len(cable_pts_idx) > 0:
            cable_pts_idx = np.setdiff1d(cable_pts_idx, tower_pts_idx)
            point_line_id[tower_pts_idx] = 0
            
        # 2. 地面点绝对互斥
        if len(ground_idx) > 0:
            classification[ground_idx] = int(ClassificationCode.GROUND)
            
        if len(tower_pts_idx) > 0:
            classification[tower_pts_idx] = int(ClassificationCode.TRANSMISSION_TOWER)
            
        if len(cable_pts_idx) > 0:
            classification[cable_pts_idx] = int(ClassificationCode.WIRE_CONDUCTOR)

        # 校验并截断横担以下标记
        if len(tower_below_arm_pts_idx) > 0:
            tower_below_arm_pts_idx = np.intersect1d(tower_below_arm_pts_idx, tower_pts_idx)
            
        tower_arm_pts_idx = off_ground_idx[is_tower_arm & is_tower] if (is_tower_arm is not None and np.any(is_tower_arm & is_tower)) else np.array([], dtype=int)

        # 3. 走廊分档内存切片 (当配置开启且检测到至少 2 座杆塔时)
        spans = None
        if getattr(cfg.corridor, 'split_spans', False) and len(tower_infos) >= 2:
            from modules.corridor_cutter import CorridorCutter
            spans = CorridorCutter.cut_spans(
                points=points,
                tower_infos=tower_infos,
                corridor_half_width=cfg.corridor.corridor_half_width,
                buffer_length=cfg.corridor.buffer_length
            )

        return PipelineResult(
            num_points=num_points,
            classification=classification,
            towers=tower_infos,
            wires=all_confirmed,
            stage_timings=stage_timings,
            ground_result=ground_res,
            extraction_result=ext_res,
            spans=spans,
            metadata={
                'point_line_id': point_line_id,
                'suspect_line_ids': suspect_line_ids,
                'find_line_func': find_line_func,
                'tower_arm_pts_idx': tower_arm_pts_idx,
                'tower_below_arm_pts_idx': tower_below_arm_pts_idx
            }
        )

    def run_file(self, 
                 las_input_path: str, 
                 las_output_path: str, 
                 config: Optional[PipelineConfig] = None,
                 verbose: Optional[bool] = None) -> PipelineResult:
        """
        文件级执行门面：读取 LAS/LAZ 点云，调用内存核心计算并导出标准分类着色 LAS
        
        Parameters:
        -----------
        las_input_path : str
            输入点云路径
        las_output_path : str
            成果输出路径
        config : Optional[PipelineConfig]
            运行时配置
        verbose : Optional[bool]
            是否输出各阶段执行日志，默认遵循 config.pipeline.verbose
            
        Returns:
        --------
        PipelineResult
            完整的管道成果对象
        """
        cfg = config if config is not None else self.config
        is_verbose = verbose if verbose is not None else getattr(cfg.pipeline, 'verbose', True)
        
        # 1. 读取原始 LAS 点云
        las = laspy.read(las_input_path)
        points = np.column_stack([np.array(las.x), np.array(las.y), np.array(las.z)])
        num_points = len(points)
        if is_verbose:
            print(f"   读取点云总数: {num_points:,} 点")
        
        # 2. 执行内存核心流水线
        result = self.run(points, config=cfg, verbose=is_verbose)
        
        # 3. 如果需要落盘成果文件
        if cfg.export.force_kill_viewer:
            from modules.viewer import get_viewer
            get_viewer().close()

        t_exp = time.time()
        meta = result.metadata
        point_line_id = meta.get('point_line_id', np.zeros(len(points), dtype=int))
        suspect_line_ids = meta.get('suspect_line_ids', set())
        find_line_func = meta.get('find_line_func', lambda x: x)
        tower_arm_pts_idx = meta.get('tower_arm_pts_idx', np.array([], dtype=int))
        tower_below_arm_pts_idx = meta.get('tower_below_arm_pts_idx', np.array([], dtype=int))
        
        actual_path = export_colored_las(
            las_input_path=las_input_path,
            las_output_path=las_output_path,
            las_raw_data=las,
            ground_idx=result.ground_indices,
            cable_pts_idx=result.wire_indices,
            tower_pts_idx=result.tower_indices,
            tower_arm_pts_idx=tower_arm_pts_idx,
            all_confirmed=result.wires,
            point_line_id=point_line_id,
            suspect_line_ids=suspect_line_ids,
            find_line_func=find_line_func,
            force_kill_viewer=cfg.export.force_kill_viewer,
            tower_below_arm_pts_idx=tower_below_arm_pts_idx
        )
        result.stage_timings["export"] = float(time.time() - t_exp)
        result.metadata["actual_output_path"] = actual_path

        # 4. 如果包含切档成果，导出独立单档 LAS 文件
        if result.spans is not None and len(result.spans) > 0:
            from modules.corridor_cutter import CorridorCutter
            split_paths = CorridorCutter.export_spans(actual_path, result.spans)
            result.metadata["split_span_paths"] = split_paths
            if is_verbose:
                print(f"   两塔一档切分完成 | 生成独立档段 LAS 文件: {len(split_paths)} 份")

        return result
