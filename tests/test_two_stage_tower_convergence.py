import unittest
import numpy as np
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from modules.tower_detector import detect_towers
from modules.config import PipelineConfig

class TestTwoStageTowerConvergence(unittest.TestCase):
    def setUp(self):
        self.config = PipelineConfig()

    def test_ticket01_waist_boundary_preserves_diagonal_underbracing(self):
        """
        Ticket 01: 验证分界高程基准面 (WaistBoundaryElevation) 下延 1.5m 裕量，
        确保横担下方的倾斜下斜撑角钢 (下延 1.2m) 100% 被捕获，绝不被窄四棱台切除。
        """
        np.random.seed(42)
        h = 35.0
        n_trunk = 3000
        zs = np.random.uniform(1.0, h, n_trunk)
        w = 4.0 - 2.0 * (zs / h)
        
        legs = np.random.choice(4, n_trunk)
        signs = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        xs = np.array([signs[l][0] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.04, n_trunk)
        ys = np.array([signs[l][1] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.04, n_trunk)
        
        # 最低横担在 z=25m, 翼展 14m (y 方向 [-7, 7])
        n_arm = 500
        arm_z = 25.0 + np.random.uniform(-0.2, 0.2, n_arm)
        arm_y = np.random.uniform(-7.0, 7.0, n_arm)
        arm_x = np.random.uniform(-0.2, 0.2, n_arm)
        
        # 横担下斜撑角钢：位于 z=23.8m ~ 24.8m (在横担中心下方 0.2m ~ 1.2m 处)，
        # 沿 y 轴展开至 [-5.0, 5.0]，此宽度远超塔腰立柱宽度 (2.5m)
        n_brace = 200
        brace_z = np.random.uniform(23.8, 24.8, n_brace)
        brace_y = np.random.uniform(-5.0, 5.0, n_brace)
        brace_x = np.random.uniform(-0.2, 0.2, n_brace)
        
        all_x = 100.0 + np.concatenate([xs, arm_x, brace_x])
        all_y = 100.0 + np.concatenate([ys, arm_y, brace_y])
        all_z = np.concatenate([zs, arm_z, brace_z])
        
        pts = np.column_stack([all_x, all_y, all_z])
        rel_z = all_z.copy()
        off_ground_idx = np.arange(len(pts))
        
        is_tower, is_arm, is_near_arm, entities = detect_towers(
            off_ground_pts=pts,
            rel_z=rel_z,
            off_ground_idx=off_ground_idx,
            config=self.config
        )
        
        self.assertEqual(len(entities), 1, "铁塔未能检出！")
        
        # 下斜撑角钢索引位于最后 n_brace 个点
        brace_indices = np.arange(len(pts) - n_brace, len(pts))
        captured_brace_count = np.sum(is_tower[brace_indices])
        capture_ratio = captured_brace_count / n_brace
        
        self.assertGreater(capture_ratio, 0.95, f"下斜撑角钢捕获率仅为 {capture_ratio:.1%}，低于 95%！")

    def test_ticket02_adaptive_line_direction_thickness(self):
        """
        Ticket 02: 验证顺线厚度自适应机制：
        对于 500kV 大跨度塔 (高 75m, 横担翼展 26m, 顺线绝缘子串长 4.2m)，
        顺线包围盒能够自适应包裹 4.2m 绝缘子，同时坚决阻断 >6.5m 的跨中悬空导线。
        """
        np.random.seed(42)
        h = 75.0
        n_trunk = 5000
        zs = np.random.uniform(1.0, h, n_trunk)
        w = 8.0 - 4.0 * (zs / h)
        
        legs = np.random.choice(4, n_trunk)
        signs = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        xs = np.array([signs[l][0] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.05, n_trunk)
        ys = np.array([signs[l][1] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.05, n_trunk)
        
        # 500kV 横担 (z=55m, 沿 y 展开 26m)
        n_arm = 1200
        arm_z = 55.0 + np.random.uniform(-0.3, 0.3, n_arm)
        arm_y = np.random.uniform(-13.0, 13.0, n_arm)
        arm_x = np.random.uniform(-0.2, 0.2, n_arm)
        
        # 500kV 耐张绝缘子串 (顺线沿 x 轴向外延伸 4.2m)
        n_ins = 300
        ins_z = 55.0 + np.random.uniform(-0.5, 0.5, n_ins)
        ins_y = np.random.choice([-12.0, 12.0], n_ins) + np.random.normal(0, 0.2, n_ins)
        ins_x = np.random.uniform(-4.2, 4.2, n_ins)  # 顺线半厚度达到 4.2m
        
        # 跨中悬空导线 (顺线沿 x 轴延伸至 7m ~ 25m)
        n_wire = 400
        wire_z = 54.5 + np.random.uniform(-0.2, 0.2, n_wire)
        wire_y = np.random.choice([-12.0, 12.0], n_wire)
        wire_x = np.random.uniform(7.0, 25.0, n_wire)  # 远离铁塔的跨中导线
        
        all_x = 200.0 + np.concatenate([xs, arm_x, ins_x, wire_x])
        all_y = 200.0 + np.concatenate([ys, arm_y, ins_y, wire_y])
        all_z = np.concatenate([zs, arm_z, ins_z, wire_z])
        
        pts = np.column_stack([all_x, all_y, all_z])
        rel_z = all_z.copy()
        off_ground_idx = np.arange(len(pts))
        
        is_tower, is_arm, is_near_arm, entities = detect_towers(
            off_ground_pts=pts,
            rel_z=rel_z,
            off_ground_idx=off_ground_idx,
            config=self.config
        )
        
        self.assertEqual(len(entities), 1)
        
        # 验证 1：4.2m 耐张绝缘子串点必须被高比例包裹在杆塔内
        ins_start = n_trunk + n_arm
        ins_end = ins_start + n_ins
        ins_captured = np.sum(is_tower[ins_start:ins_end])
        self.assertGreater(ins_captured / n_ins, 0.85, f"500kV 绝缘子串捕获率不足: {ins_captured}/{n_ins}")
        
        # 验证 2：x >= 7m 的跨中大跨度导线绝不能被染为杆塔
        wire_start = ins_end
        wire_captured = np.sum(is_tower[wire_start:])
        self.assertLess(wire_captured / n_wire, 0.05, f"跨中导线被错误包入杆塔: {wire_captured}/{n_wire}")

    def test_ticket03_ticket04_pure_centroid_and_ground_vegetation_rejection(self):
        """
        Ticket 03 & 04: 
        1. 验证非对称导线牵拉下，纯净塔身多层切片锁定真中轴，四棱台塔腿零漏识；
        2. 验证四棱台贴地端 3D 骨架体素连通生长能够剔除四棱台底角覆盖的孤立地表树冠。
        """
        np.random.seed(42)
        h = 32.0
        n_trunk = 3200
        zs = np.random.uniform(1.0, h, n_trunk)
        base_w, top_w = 4.5, 1.8
        w = base_w - (base_w - top_w) * (zs / h)
        
        legs = np.random.choice(4, n_trunk)
        signs = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        xs = np.array([signs[l][0] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.04, n_trunk)
        ys = np.array([signs[l][1] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.04, n_trunk)
        
        # 横担 (z=26m)
        n_arm = 600
        arm_z = 26.0 + np.random.uniform(-0.2, 0.2, n_arm)
        arm_y = np.random.uniform(-7.0, 7.0, n_arm)
        arm_x = np.random.uniform(-0.2, 0.2, n_arm)
        
        # 极端非对称强导线牵拉点 (仅在 +y 方向 8~25m 延伸，模拟单侧导线)
        n_asym = 1000
        asym_z = 25.8 + np.random.uniform(-0.2, 0.2, n_asym)
        asym_y = np.random.uniform(8.0, 25.0, n_asym)
        asym_x = np.random.uniform(-0.2, 0.2, n_asym)
        
        # 四棱台底角山坡孤立树木 (位于地面 z=0.5~3m 处，在空腔处 (x=0, y=3.2)，在四棱台底半宽范围内但无钢构相连)
        n_tree = 800
        tree_z = np.random.uniform(0.5, 3.0, n_tree)
        tree_r = np.random.uniform(0.0, 0.8, n_tree)
        tree_theta = np.random.uniform(0.0, 2 * np.pi, n_tree)
        tree_x = 0.0 + tree_r * np.cos(tree_theta)
        tree_y = 3.2 + tree_r * np.sin(tree_theta)
        
        all_x = 300.0 + np.concatenate([xs, arm_x, asym_x, tree_x])
        all_y = 300.0 + np.concatenate([ys, arm_y, asym_y, tree_y])
        all_z = np.concatenate([zs, arm_z, asym_z, tree_z])
        
        pts = np.column_stack([all_x, all_y, all_z])
        rel_z = all_z.copy()
        off_ground_idx = np.arange(len(pts))
        
        is_tower, is_arm, is_near_arm, entities = detect_towers(
            off_ground_pts=pts,
            rel_z=rel_z,
            off_ground_idx=off_ground_idx,
            config=self.config
        )
        
        self.assertEqual(len(entities), 1)
        ent = entities[0]
        
        # 验证中轴精度 (真中心位于 (300.0, 300.0))
        self.assertLess(abs(ent.cx - 300.0), 0.25, f"非对称导线导致中轴偏心: cx={ent.cx:.2f}")
        self.assertLess(abs(ent.cy - 300.0), 0.25, f"非对称导线导致中轴偏心: cy={ent.cy:.2f}")
        
        # 验证真塔腿点云召回率 (主材角钢)
        trunk_captured = np.sum(is_tower[:n_trunk])
        self.assertGreater(trunk_captured / n_trunk, 0.95, f"塔腿角钢召回率不足: {trunk_captured}/{n_trunk}")
        
        # 验证四棱台底角孤立地表树木被成功剔除
        tree_start = n_trunk + n_arm + n_asym
        tree_captured = np.sum(is_tower[tree_start:])
        self.assertLess(tree_captured / n_tree, 0.05, f"四棱台底角树木被误判为塔: {tree_captured}/{n_tree}")

if __name__ == '__main__':
    unittest.main()
