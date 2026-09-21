import os
import glob
import sys
import time
import argparse
from typing import List, Optional
import numpy as np
import laspy

from modules.config import PipelineConfig, DEFAULT_CONFIG, PipelineStage
from modules.pipeline_executor import PipelineExecutor
from modules.corridor_cutter import cut_corridors_by_spans, export_split_spans, split_raw_corridor
from modules.utils import select_file_gui, open_in_qtmodeler

def fast_classify_and_color_powerline(las_input_path: str, 
                                      las_output_path: str, 
                                      config: PipelineConfig = None) -> str:
    """
    点云电力线分类与着色主入口管道 (委托给深度模块 PipelineExecutor)
    
    Parameters:
    -----------
    las_input_path : str
        输入 LAS/LAZ 点云路径
    las_output_path : str
        期望输出的分类着色 LAS 路径
    config : PipelineConfig, optional
        全局参数配置中心实例，默认使用 DEFAULT_CONFIG
        
    Returns:
    --------
    actual_output_path : str
        实际成功写入并落盘的成果 LAS 路径
    """
    if config is None:
        config = DEFAULT_CONFIG
        
    print(f"[Start] 开始电力线分类处理: {las_input_path}")
    start_time = time.time()
    
    executor = PipelineExecutor(config=config)
    result = executor.run_file(las_input_path, las_output_path, config=config)
    actual_out_path = result.metadata.get("actual_output_path", las_output_path)
    
    total_cost = time.time() - start_time
    summary = result.summary()
    print(f"[Done] 全流程处理完成！总耗时: {total_cost:.2f} 秒")
    print(f"   分类统计: 地面={summary['ground_count']:,} | 杆塔={summary['tower_count']:,} | 导线={summary['wire_count']:,} | 植被/杂波={summary['unclassified_count']:,}")
    
    # 后置切分 (仅在整段模式下指定 --split-spans 时触发)
    if config.corridor.split_spans and len(result.towers) >= 2:
        t_split = time.time()
        las = laspy.read(las_input_path)
        points = np.column_stack([np.array(las.x), np.array(las.y), np.array(las.z)])
        spans = cut_corridors_by_spans(
            points=points,
            tower_infos=result.towers,
            corridor_half_width=config.corridor.corridor_half_width,
            buffer_length=config.corridor.buffer_length
        )
        split_paths = export_split_spans(actual_out_path, spans)
        print(f"   两塔一档切分完成 (耗时: {time.time() - t_split:.2f}s) | 生成独立档段 LAS 文件: {len(split_paths)} 份")
        
    return actual_out_path


def run_batch_directory(input_dir: str, config: PipelineConfig = None):
    """
    【第二步：批量单档分类处理器】
    扫描指定目录下的所有 .las / .laz 文件，逐个执行高精度分类与着色
    """
    if config is None:
        config = DEFAULT_CONFIG
        
    las_files = glob.glob(os.path.join(input_dir, "*.las")) + glob.glob(os.path.join(input_dir, "*.laz"))
    # 过滤掉已经带有 _sign 标记的文件，避免死循环重复跑
    las_files = [f for f in las_files if not f.endswith("_sign.las") and not f.endswith("_sign.laz")]
    
    if len(las_files) == 0:
        print(f"[Warning] 目录 '{input_dir}' 下未找到任何待处理的 LAS/LAZ 点云文件。")
        return
        
    print(f"\n========================================================")
    print(f"  [批量处理模式] 扫描到待处理档段文件: {len(las_files)} 份")
    print(f"========================================================")
    
    t_batch_start = time.time()
    success_count = 0
    
    for idx, fpath in enumerate(las_files, 1):
        print(f"\n>>> [{idx}/{len(las_files)}] 正在处理单档: '{os.path.basename(fpath)}'")
        base_name = os.path.splitext(fpath)[0]
        out_path = f"{base_name}_sign.las"
        try:
            fast_classify_and_color_powerline(fpath, out_path, config=config)
            success_count += 1
        except Exception as e:
            print(f"[Error] 处理文件 '{fpath}' 失败: {e}")
            
    total_batch_time = time.time() - t_batch_start
    print(f"\n========================================================")
    print(f"  [批量处理完成] 成功处理: {success_count}/{len(las_files)} 份 | 总耗时: {total_batch_time:.2f} 秒")
    print(f"========================================================")


def run_auto_span_pipeline(las_input_path: str, config: PipelineConfig = None) -> List[str]:
    """
    【模式 B：一键端到端先切后算流水线 (Auto-Span-Pipeline)】
    1. 快速定位全线铁塔并切分出原始单档子点云；
    2. 逐档在纯净 60m 走廊内独立执行高精度分类；
    3. 自动生成各档独立成果并加载到 QTModeler。
    """
    if config is None:
        config = DEFAULT_CONFIG
        
    t_all_start = time.time()
    print(f"\n========================================================")
    print(f"  [一键端到端流水线] 启动: {las_input_path}")
    print(f"========================================================")
    
    base_dir = os.path.dirname(las_input_path)
    base_name = os.path.splitext(os.path.basename(las_input_path))[0]
    spans_raw_dir = os.path.join(base_dir, f"{base_name}_spans_raw")
    
    # 1. 快速切片预处理
    raw_spans = split_raw_corridor(las_input_path, output_dir=spans_raw_dir, config=config)
    if len(raw_spans) == 0:
        print("[Warning] 未生成有效档段切片，退化为整段直接分类模式...")
        out_full = f"{os.path.splitext(las_input_path)[0]}_sign.las"
        return [fast_classify_and_color_powerline(las_input_path, out_full, config=config)]
        
    # 2. 对每个切片独立执行精细分类
    classified_paths = []
    for idx, raw_span_path in enumerate(raw_spans, 1):
        print(f"\n>>> [{idx}/{len(raw_spans)}] 正在执行单档精细分类: '{os.path.basename(raw_span_path)}'")
        span_base = os.path.splitext(raw_span_path)[0]
        span_out = f"{span_base}_sign.las"
        actual_span_out = fast_classify_and_color_powerline(raw_span_path, span_out, config=config)
        classified_paths.append(actual_span_out)
        
    total_time = time.time() - t_all_start
    print(f"\n========================================================")
    print(f"  [端到端先切后算全部完成] 共生成分类档段: {len(classified_paths)} 份 | 总耗时: {total_time:.2f} 秒")
    print(f"========================================================")
    
    if len(classified_paths) > 0 and config.export.open_qtmodeler:
        open_in_qtmodeler(classified_paths[0])
        
    return classified_paths


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="激光点云电力线分类与杆塔提取系统")
    parser.add_argument("--input", "-i", type=str, help="输入 LAS 文件路径")
    parser.add_argument("--output", "-o", type=str, help="输出 LAS 文件路径")
    parser.add_argument("--no-gui", action="store_true", help="禁用 GUI 选择框")
    parser.add_argument("--no-qtmodeler", action="store_true", help="禁用处理完成后的 QTModeler 自动可视化")
    parser.add_argument("--force-kill-viewer", action="store_true", help="强制关闭后台运行的 QTModeler 线程")
    parser.add_argument("--distribution", action="store_true", help="启用配电网模式 (支持 10kV~110kV 矮塔/单双水泥杆)")
    
    parser.add_argument("--stop-after", type=str, choices=["ground", "tower", "wire"], default=None,
                        help="仅执行至指定阶段并提前终止 (可选: ground, tower, wire)")
    parser.add_argument("--quiet", "-q", action="store_true", help="静默模式，不输出阶段详细执行日志")
    
    # 走廊切片与批量模式参数
    parser.add_argument("--split-only", action="store_true", help="【步骤一】：仅执行纯走廊切片预处理，快速生成原始单档点云")
    parser.add_argument("--split-spans", action="store_true", help="分类完成后在后台自动切分各档独立成果 LAS")
    parser.add_argument("--auto-span-pipeline", action="store_true", help="【端到端】：一键自动串联“找塔 -> 快速切档 -> 各档分别精细分类”")
    parser.add_argument("--batch-dir", type=str, help="【步骤二】：批量对指定目录下的所有单档 LAS 文件执行精细分类")
    parser.add_argument("--corridor-width", type=float, default=30.0, help="走廊单侧半宽 (m)，默认 30m (全宽 60m)")
    parser.add_argument("--span-buffer", type=float, default=12.0, help="档段两端外延缓冲长度 (m)，默认 12m")

    args = parser.parse_args()

    cfg = PipelineConfig()
    if args.stop_after:
        cfg.pipeline.stop_after = PipelineStage.from_string(args.stop_after)
    if args.quiet:
        cfg.pipeline.verbose = False
    cfg.export.force_kill_viewer = args.force_kill_viewer
    cfg.export.open_qtmodeler = not args.no_qtmodeler
    cfg.corridor.split_spans = args.split_spans
    cfg.corridor.corridor_half_width = args.corridor_width
    cfg.corridor.buffer_length = args.span_buffer
    
    if args.distribution:
        cfg.tower.allow_distribution_poles = True
        cfg.tower.min_tower_rel_z = 10.0
        cfg.tower.high_voltage_min_z = 10.0
        cfg.tower.delta_h_relief = 3.0

    # 1. 批量目录处理模式
    if args.batch_dir:
        run_batch_directory(args.batch_dir, config=cfg)
        sys.exit(0)

    # 2. 单文件/GUI 输入处理
    INPUT_LAS = args.input
    if not INPUT_LAS and not args.no_gui:
        print("正在打开 Windows 文件选择对话框...")
        INPUT_LAS = select_file_gui()

    if INPUT_LAS and os.path.exists(INPUT_LAS):
        # 模式 A: 纯切片预处理 (--split-only)
        if args.split_only:
            split_raw_corridor(INPUT_LAS, config=cfg)
            sys.exit(0)
            
        # 模式 B: 一键端到端先切后算 (--auto-span-pipeline)
        if args.auto_span_pipeline:
            run_auto_span_pipeline(INPUT_LAS, config=cfg)
            sys.exit(0)

        # 模式 C: 标准分类模式 (支持单档或整段)
        if args.output:
            OUTPUT_LAS = args.output
        else:
            base_name = os.path.splitext(INPUT_LAS)[0]
            OUTPUT_LAS = f"{base_name}_sign.las"
            
        out_path = fast_classify_and_color_powerline(INPUT_LAS, OUTPUT_LAS, config=cfg)
        if cfg.export.open_qtmodeler:
            open_in_qtmodeler(out_path)
            
    elif not INPUT_LAS:
        print("未选择任何文件，操作已取消。")
    else:
        print(f"选择的文件不存在: {INPUT_LAS}")
