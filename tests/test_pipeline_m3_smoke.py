import os
import sys
import time
import tempfile
import numpy as np
import laspy

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from fast_powerline_classifier import fast_classify_and_color_powerline
from modules.config import PipelineConfig

def create_synthetic_span_las(output_path: str):
    header = laspy.LasHeader(point_format=3, version="1.2")
    header.offsets = [0.0, 0.0, 0.0]
    header.scales = [0.001, 0.001, 0.001]
    
    # 1. 地面点
    np.random.seed(42)
    gx = np.random.uniform(-20, 120, 25000)
    gy = np.random.uniform(-30, 30, 25000)
    gz = np.random.uniform(0.0, 0.4, 25000)
    
    # 2. 铁塔模型 (2座: x=0, y=0 与 x=100, y=0; 塔高 35m)
    def generate_tower(cx, cy, base_w=6.0, top_w=2.0, h=35.0, n_pts=3500):
        zs = np.random.uniform(1.5, h, n_pts)
        ws = base_w - (base_w - top_w) * (zs / h)
        xs = cx + np.random.uniform(-ws/2, ws/2, n_pts)
        ys = cy + np.random.uniform(-ws/2, ws/2, n_pts)
        
        # 横担点云
        arm_pts = 1000
        z_arm1 = 28.0 + np.random.uniform(-0.4, 0.4, arm_pts // 2)
        y_arm1 = cy + np.random.uniform(-8.0, 8.0, arm_pts // 2)
        x_arm1 = cx + np.random.uniform(-1.0, 1.0, arm_pts // 2)
        
        z_arm2 = 32.0 + np.random.uniform(-0.4, 0.4, arm_pts // 2)
        y_arm2 = cy + np.random.uniform(-6.0, 6.0, arm_pts // 2)
        x_arm2 = cx + np.random.uniform(-1.0, 1.0, arm_pts // 2)
        
        tx = np.concatenate([xs, x_arm1, x_arm2])
        ty = np.concatenate([ys, y_arm1, y_arm2])
        tz = np.concatenate([zs, z_arm1, z_arm2])
        return tx, ty, tz

    t1x, t1y, t1z = generate_tower(0.0, 0.0)
    t2x, t2y, t2z = generate_tower(100.0, 0.0)
    
    # 3. 悬链线导线 (4根单相导线, a=500m)
    cable_xs = []
    cable_ys = []
    cable_zs = []
    
    cables_cfg = [
        (-6.0, 28.0), (6.0, 28.0),
        (-4.5, 32.0), (4.5, 32.0)
    ]
    a = 500.0
    for cy, cz_hang in cables_cfg:
        xs = np.linspace(0.0, 100.0, 1800)
        b = cz_hang - a * np.cosh(50.0 / a)
        zs = a * np.cosh((xs - 50.0) / a) + b
        xs += np.random.normal(0, 0.02, len(xs))
        ys = cy + np.random.normal(0, 0.02, len(xs))
        zs += np.random.normal(0, 0.02, len(xs))
        cable_xs.append(xs)
        cable_ys.append(ys)
        cable_zs.append(zs)
        
    all_x = np.concatenate([gx, t1x, t2x] + cable_xs)
    all_y = np.concatenate([gy, t1y, t2y] + cable_ys)
    all_z = np.concatenate([gz, t1z, t2z] + cable_zs)
    
    las = laspy.LasData(header)
    las.x = all_x
    las.y = all_y
    las.z = all_z
    las.classification = np.full(len(all_x), 1, dtype=np.uint8)
    las.red = np.full(len(all_x), 200, dtype=np.uint16)
    las.green = np.full(len(all_x), 200, dtype=np.uint16)
    las.blue = np.full(len(all_x), 200, dtype=np.uint16)
    las.write(output_path)
    return len(all_x)

def test_pipeline_synthetic():
    print("============================================================")
    print("[TEST 1] 合成点云端到端流水线与接口契约冒烟测试 (Synthetic Test)")
    print("============================================================")
    with tempfile.TemporaryDirectory() as tmp_dir:
        input_las = os.path.join(tmp_dir, "synthetic_span.las")
        output_las = os.path.join(tmp_dir, "synthetic_span_sign.las")
        
        n_pts = create_synthetic_span_las(input_las)
        print(f"-> 合成点云构建完成: {n_pts} 点")
        
        cfg = PipelineConfig()
        cfg.export.open_qtmodeler = False
        cfg.export.force_kill_viewer = False
        
        t0 = time.time()
        res_path = fast_classify_and_color_powerline(input_las, output_las, config=cfg)
        cost = time.time() - t0
        
        assert os.path.exists(res_path), f"成果文件不存在: {res_path}"
        print(f"-> 流水线成功执行完成，耗时: {cost:.2f}s")
        
        out_las = laspy.read(res_path)
        cls = np.array(out_las.classification)
        
        n_ground = int(np.sum(cls == 2))
        n_tower = int(np.sum(cls == 15))
        n_cable = int(np.sum(cls == 14))
        
        print(f"-> 分类统计校验: 地面={n_ground}, 铁塔={n_tower}, 导线={n_cable}")
        
        assert n_ground > 0, "地面点 (Class 2) 未识别！"
        assert n_tower > 0, "铁塔点 (Class 15) 未识别！"
        assert n_cable > 0, "导线点 (Class 14) 未识别！"
        
        # 验证点集互斥性
        g_idx = np.where(cls == 2)[0]
        t_idx = np.where(cls == 15)[0]
        c_idx = np.where(cls == 14)[0]
        
        assert len(np.intersect1d(g_idx, t_idx)) == 0, "地面与铁塔点集重叠！"
        assert len(np.intersect1d(t_idx, c_idx)) == 0, "铁塔与导线点集重叠！"
        assert len(np.intersect1d(g_idx, c_idx)) == 0, "地面与导线点集重叠！"
        
        # 验证导线点色彩分配非黑
        c_red = np.array(out_las.red)[c_idx]
        c_green = np.array(out_las.green)[c_idx]
        c_blue = np.array(out_las.blue)[c_idx]
        is_black = (c_red == 0) & (c_green == 0) & (c_blue == 0)
        assert np.sum(is_black) == 0, "导线点存在纯黑异常赋色！"
        
        print("[PASS] 合成测试全部契约断言通过！")

def test_pipeline_real_file():
    real_path = os.path.join("E:", os.sep, "点云备份", "1-66", "17-18(17_18).las")
    if not os.path.exists(real_path):
        print(f"[SKIP] 真实点云样本不存在: {real_path}")
        return
        
    print("")
    print("============================================================")
    print(f"[TEST 2] 真实机巡档段端到端全流程运行测试: {os.path.basename(real_path)}")
    print("============================================================")
    
    with tempfile.TemporaryDirectory() as tmp_dir:
        output_las = os.path.join(tmp_dir, "17_18_sign.las")
        cfg = PipelineConfig()
        cfg.export.open_qtmodeler = False
        cfg.export.force_kill_viewer = False
        
        t0 = time.time()
        res_path = fast_classify_and_color_powerline(real_path, output_las, config=cfg)
        cost = time.time() - t0
        
        assert os.path.exists(res_path)
        out_las = laspy.read(res_path)
        cls = np.array(out_las.classification)
        
        n_ground = int(np.sum(cls == 2))
        n_tower = int(np.sum(cls == 15))
        n_cable = int(np.sum(cls == 14))
        
        print(f"-> 真实数据完成: 耗时 {cost:.2f}s | 地面={n_ground} | 铁塔={n_tower} | 导线={n_cable}")
        assert n_tower > 0, "真实数据中铁塔未检出！"
        assert n_cable > 0, "真实数据中导线未检出！"
        print("[PASS] 真实数据全链路运行通过！")

if __name__ == '__main__':
    test_pipeline_synthetic()
    test_pipeline_real_file()
    print("")
    print(">>> Task 1 全部自动化测试用例均验证通过！<<<")
