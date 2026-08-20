import os
import sys
import glob
import time
import argparse
from fast_powerline_classifier import fast_classify_and_color_powerline
from modules.config import PipelineConfig
from modules.evaluator import PointCloudEvaluator, print_evaluation_report

def process_file(input_las: str, output_dir: str = None, run_eval: bool = False, config: PipelineConfig = None):
    """
    处理单个 LAS 文件并根据选型输出评估报表
    """
    base_name = os.path.basename(input_las)
    name_no_ext = os.path.splitext(base_name)[0]
    
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        out_las = os.path.join(output_dir, f"{name_no_ext}_sign.las")
    else:
        out_las = os.path.join(os.path.dirname(input_las), f"{name_no_ext}_sign.las")
        
    print(f"\n[Batch] 处理文件: {input_las}")
    res_path = fast_classify_and_color_powerline(input_las, out_las, config=config)
    
    if run_eval:
        try:
            report = PointCloudEvaluator.evaluate_las_files(input_las, res_path)
            print_evaluation_report(report, title=f"评估报告: {base_name}")
        except Exception as e:
            print(f"[Eval Error] 对照真值评估失败: {e}")

def main():
    parser = argparse.ArgumentParser(description="点云电力线分类批处理与 Benchmark 报表引擎")
    parser.add_argument("--input", "-i", required=True, help="输入 LAS 文件路径或包含 LAS 文件的目录路径")
    parser.add_argument("--output-dir", "-o", help="输出成果保存目录")
    parser.add_argument("--eval", action="store_true", help="是否与输入 LAS 的原始 Classification 标签比对算指标")
    
    args = parser.parse_args()
    
    start_time = time.time()
    cfg = PipelineConfig()
    
    if os.path.isfile(args.input):
        process_file(args.input, args.output_dir, args.eval, config=cfg)
    elif os.path.isdir(args.input):
        las_files = glob.glob(os.path.join(args.input, "*.las")) + glob.glob(os.path.join(args.input, "*.laz"))
        if not las_files:
            print(f"目录 {args.input} 下未找到任何 .las / .laz 文件！")
            return
            
        print(f"[Batch Mode] 找到 {len(las_files)} 个点云文件，开始批量分类评估...")
        for idx, file_path in enumerate(las_files, 1):
            print(f"\n({idx}/{len(las_files)}) 正在调度: {file_path}")
            process_file(file_path, args.output_dir, args.eval, config=cfg)
    else:
        print(f"路径不存在: {args.input}")
        return
        
    print(f"\n[All Done] 所有任务执行完毕，总耗时: {time.time() - start_time:.2f} 秒！")

if __name__ == "__main__":
    main()
