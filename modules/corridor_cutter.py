import os
import time
from typing import List, Union, Optional
import numpy as np
import laspy

from modules.models import TowerEntity, SpanSegment
from modules.config import PipelineConfig, DEFAULT_CONFIG

class CorridorCutter:
    """
    走廊分档切片与空间几何变换深度服务类。
    负责铁塔走向拓扑排序、双塔定向走廊 (OBB) 点云切分与单档高保真导出。
    """

    @staticmethod
    def order_towers(tower_infos: List[Union[TowerEntity, dict]], heading_penalty: float = 0.5) -> List[int]:
        """
        沿输电线路主干走向，将无序/离散的杆塔拓扑排序为线性链条 (T1 -> T2 -> ... -> Tn)。
        引入航向角偏转平滑惩罚，抑制山区大转角与局部误检引起的顺序乱跳。
        """
        n_towers = len(tower_infos)
        if n_towers <= 2:
            return list(range(n_towers))

        t_xy = np.array([[t['cx'], t['cy']] for t in tower_infos])

        # 1. 计算两两杆塔之间的欧氏距离矩阵
        dists = np.linalg.norm(t_xy[:, None, :] - t_xy[None, :, :], axis=2)

        # 2. 找到相距最远的两个杆塔作为线路的两个端点 (Start / End)
        start_idx, _ = np.unravel_index(np.argmax(dists), dists.shape)

        # 3. 从起点出发，通过航向平滑加权的贪心遍历串联整条走廊
        ordered = [start_idx]
        visited = set(ordered)
        curr = start_idx
        prev_vec = None

        while len(ordered) < n_towers:
            unvisited = [i for i in range(n_towers) if i not in visited]
            if len(unvisited) == 1:
                ordered.append(unvisited[0])
                break

            prev_norm = np.linalg.norm(prev_vec) if prev_vec is not None else 0.0
            has_valid_prev = prev_norm > 1e-3

            best_candidate = None
            min_cost = float('inf')

            for cand in unvisited:
                d = dists[curr, cand]
                cand_vec = t_xy[cand] - t_xy[curr]
                cand_norm = np.linalg.norm(cand_vec)

                if has_valid_prev and cand_norm > 1e-3:
                    cos_theta = np.dot(prev_vec, cand_vec) / (prev_norm * cand_norm)
                    cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
                    # 夹角偏转惩劳项: 方向越一致 (cos~1) 惩罚越小; 大角度掉头 (cos~-1) 惩罚最大
                    penalty = 1.0 + heading_penalty * (1.0 - cos_theta)
                else:
                    penalty = 1.0

                cost = d * penalty
                if cost < min_cost:
                    min_cost = cost
                    best_candidate = cand

            if best_candidate is not None:
                prev_vec = t_xy[best_candidate] - t_xy[curr]
                ordered.append(best_candidate)
                visited.add(best_candidate)
                curr = best_candidate
            else:
                fallback = min(unvisited, key=lambda idx: dists[curr, idx])
                prev_vec = t_xy[fallback] - t_xy[curr]
                ordered.append(fallback)
                visited.add(fallback)
                curr = fallback

        return ordered

    @staticmethod
    def cut_spans(points: np.ndarray,
                  tower_infos: List[Union[TowerEntity, dict]],
                  corridor_half_width: float = 30.0,
                  buffer_length: float = 12.0) -> List[SpanSegment]:
        """
        基于双塔定向走廊包围盒 (Corridor OBB) 向量化快速切分各跨档距点云
        """
        n_towers = len(tower_infos)
        if n_towers < 2:
            return []

        ordered_indices = CorridorCutter.order_towers(tower_infos)
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

            u_span = v_span / span_len                  # 沿线主轴方向单位向量
            n_span = np.array([-u_span[1], u_span[0]]) # 走廊横向法向单位向量

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

    @staticmethod
    def export_spans(las_classified_or_data: Union[str, laspy.LasData],
                     spans: List[SpanSegment],
                     output_dir: Optional[str] = None,
                     base_name: Optional[str] = None) -> List[str]:
        """
        将切分好的各档点云高保真无损输出为独立的 LAS 文件。
        支持传入已分类 LAS 文件路径 (str) 或内存中 LasData 对象。
        """
        if len(spans) == 0:
            return []

        if isinstance(las_classified_or_data, str):
            if not os.path.exists(las_classified_or_data):
                return []
            if output_dir is None:
                base_dir = os.path.dirname(las_classified_or_data)
                output_dir = os.path.join(base_dir, "spans")
            if base_name is None:
                raw_name = os.path.splitext(os.path.basename(las_classified_or_data))[0]
                clean_base = raw_name.removesuffix("_sign")
            else:
                clean_base = base_name.removesuffix("_sign")
            las = laspy.read(las_classified_or_data)
        else:
            las = las_classified_or_data
            if output_dir is None:
                output_dir = "spans"
            clean_base = base_name.removesuffix("_sign") if base_name is not None else "corridor"

        os.makedirs(output_dir, exist_ok=True)

        print(f"-> 正在将成果按两塔一档切分为 {len(spans)} 份独立 LAS 文件...")
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

    @staticmethod
    def split_raw_corridor(las_input_path: str,
                           output_dir: Optional[str] = None,
                           config: Optional[PipelineConfig] = None) -> List[str]:
        """
        【独立步骤一：纯走廊切片预处理 (Split-Only Preprocessing)】
        委托 PipelineExecutor 在锁定全线杆塔后 (PipelineStage.TOWER) 快速返回，
        直接将原始未分类大点云切分为各档独立 LAS。
        """
        from modules.pipeline_executor import PipelineExecutor
        from modules.config import PipelineStage

        cfg = config if config is not None else DEFAULT_CONFIG

        t_start = time.time()
        print(f"[Split-Only] 开始执行纯走廊切片预处理: {las_input_path}")

        # 1. 委托 PipelineExecutor 读取点云并执行至 TOWER 阶段短路返回
        las = laspy.read(las_input_path)
        points = np.column_stack([np.array(las.x), np.array(las.y), np.array(las.z)])
        num_points = len(points)
        print(f"   读取原始点云总数: {num_points:,} 点")

        executor = PipelineExecutor(config=cfg)
        res = executor.run(points, config=cfg, stop_after=PipelineStage.TOWER)
        tower_infos = res.towers
        n_towers = len(tower_infos)

        if n_towers < 2:
            print("[Warning] 检测到的铁塔数量少于 2 座，无法构成跨越档段，切分取消。")
            return []

        # 2. 定向 OBB 走廊包围盒切分
        spans = CorridorCutter.cut_spans(
            points=points,
            tower_infos=tower_infos,
            corridor_half_width=cfg.corridor.corridor_half_width,
            buffer_length=cfg.corridor.buffer_length
        )

        if output_dir is None:
            base_dir = os.path.dirname(las_input_path)
            output_dir = os.path.join(base_dir, "spans_raw")

        base_name = os.path.splitext(os.path.basename(las_input_path))[0]
        # 直接复用 export_spans 高保真无损输出逻辑 (零拷贝切片，保留 100% 原始数据属性)
        generated_paths = CorridorCutter.export_spans(
            las_classified_or_data=las,
            spans=spans,
            output_dir=output_dir,
            base_name=base_name
        )

        total_time = time.time() - t_start
        print(f"[Done] 纯切片预处理完成！总耗时: {total_time:.2f} 秒 | 导出单档文件: {len(generated_paths)} 份")
        return generated_paths


# =============================================================================
# 向后兼容顶层过程式函数转调垫片 (Shims)
# =============================================================================

def order_towers_along_line(tower_infos: List[Union[TowerEntity, dict]]) -> List[int]:
    """[DEPRECATED] 沿输电线路主干走向对杆塔进行拓扑排序。请使用 CorridorCutter.order_towers()"""
    return CorridorCutter.order_towers(tower_infos)


def cut_corridors_by_spans(points: np.ndarray,
                           tower_infos: List[Union[TowerEntity, dict]],
                           corridor_half_width: float = 30.0,
                           buffer_length: float = 12.0) -> List[SpanSegment]:
    """[DEPRECATED] 基于双塔定向走廊包围盒向量化切分各跨档距点云。请使用 CorridorCutter.cut_spans()"""
    return CorridorCutter.cut_spans(points, tower_infos, corridor_half_width, buffer_length)


def export_split_spans(las_classified_path: str,
                       spans: List[SpanSegment],
                       output_dir: Optional[str] = None) -> List[str]:
    """[DEPRECATED] 将切分好的各档点云输出为独立 LAS 文件。请使用 CorridorCutter.export_spans()"""
    return CorridorCutter.export_spans(las_classified_path, spans, output_dir)


def split_raw_corridor(las_input_path: str,
                       output_dir: Optional[str] = None,
                       config: Optional[PipelineConfig] = None) -> List[str]:
    """[DEPRECATED] 纯走廊切片预处理。请使用 CorridorCutter.split_raw_corridor()"""
    return CorridorCutter.split_raw_corridor(las_input_path, output_dir, config)
