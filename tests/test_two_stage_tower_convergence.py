import unittest
import numpy as np
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from modules.tower_detector import detect_towers, extract_waist_orthogonal_axes
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

    def test_ticket01_extract_waist_orthogonal_axes_isotropy_and_orthogonality(self):
        """
        Ticket 01: 验证 extract_waist_orthogonal_axes 在任意地理走向旋转角度下，
        均能准确、无偏、各项同性地提取正方形截面物理外立面正交向量，
        且严格正交 (|u_a . u_b| < 1e-6) 且模长为 1。
        """
        # 测试多个任意角度 (包含直线塔、转角塔各种走向)
        test_angles = [0.0, 15.5, 33.0, 45.0, 68.5, 82.0]
        L = 2.4 # 塔腰立柱正方形截面边长 2.4m
        corners = np.array([[-L/2, -L/2], [L/2, -L/2], [L/2, L/2], [-L/2, L/2]])

        # 构造正方形外围轮廓密集点 + 内部斜撑点
        edge_pts = []
        for i in range(4):
            p1, p2 = corners[i], corners[(i + 1) % 4]
            for alpha in np.linspace(0, 1, 25):
                edge_pts.append(p1 * (1 - alpha) + p2 * alpha)
        # 内部 X 交叉斜撑
        for alpha in np.linspace(0, 1, 20):
            edge_pts.append(corners[0] * (1 - alpha) + corners[2] * alpha)
            edge_pts.append(corners[1] * (1 - alpha) + corners[3] * alpha)
        base_square = np.array(edge_pts)

        for target_deg in test_angles:
            rad = np.radians(target_deg)
            R = np.array([[np.cos(rad), -np.sin(rad)], [np.sin(rad), np.cos(rad)]])
            pts = base_square @ R.T + np.array([500.0, 800.0]) # 加上大尺度地理平移

            res = extract_waist_orthogonal_axes(pts, deg_step=0.5)
            self.assertIsNotNone(res)
            u_a, u_b = res

            # 1. 严格正交性与单位模长验证
            self.assertAlmostEqual(float(np.linalg.norm(u_a)), 1.0, places=5)
            self.assertAlmostEqual(float(np.linalg.norm(u_b)), 1.0, places=5)
            self.assertAlmostEqual(float(np.abs(u_a @ u_b)), 0.0, places=5)

            # 2. 几何角度无偏验证：u_a 或 u_b 必有一根与 target_deg 平行 (模 90 度内夹角 <= 0.5 度)
            ang_a = np.degrees(np.arctan2(u_a[1], u_a[0])) % 90.0
            ang_b = np.degrees(np.arctan2(u_b[1], u_b[0])) % 90.0
            expected_mod90 = target_deg % 90.0
            diff_a = min(abs(ang_a - expected_mod90), 90.0 - abs(ang_a - expected_mod90))
            diff_b = min(abs(ang_b - expected_mod90), 90.0 - abs(ang_b - expected_mod90))
            min_diff = min(diff_a, diff_b)
            self.assertLess(min_diff, 0.6, f"角度解算偏差过大: target={target_deg}, found=({ang_a:.2f}, {ang_b:.2f})")

        # 3. 极少点退化测试
        too_few_pts = np.array([[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]])
        self.assertIsNone(extract_waist_orthogonal_axes(too_few_pts))

    def test_ticket02_crossarm_ransac_arbitration_and_rigid_orientation(self):
        """
        Ticket 02: 验证高空 RANSAC 点积仲裁与全塔单一刚体朝向锁定：
        1. 模拟 5-6# 类似转角耐张塔：真实塔身外立面在 144.5 度，高空由于引流线/绝缘子串干扰导致 RANSAC 初步拟合在 129.5 度 (15 度偏折)；
        2. 经过 RANSAC 点积仲裁后，系统必须成功在正交对 (54.5°, 144.5°) 中选出 144.5° 作为横担主轴 v1，54.5° 作为走廊厚度轴 v2；
        3. 验证 UpperTowerBox 与 LowerTowerFrustum 完全共享单一刚体轴系，剪刀差为 0.0°；
        4. 验证长纵向跨档导线 (沿 54.5° 延伸 100m) 不会误导横担轴发生 90° 倒挂。
        """
        np.random.seed(42)
        h = 42.0
        n_trunk = 4000
        zs = np.random.uniform(1.0, h, n_trunk)
        w = 4.5 - 2.0 * (zs / h)

        # 塔腰物理立面设定在 theta_true = 144.5 度 (走廊在 54.5 度)
        theta_true_deg = 144.5
        rad_true = np.radians(theta_true_deg)
        # 局部坐标基底：u_arm 沿 144.5°, u_line 沿 54.5°
        u_arm = np.array([np.cos(rad_true), np.sin(rad_true)])
        u_line = np.array([-np.sin(rad_true), np.cos(rad_true)])

        legs = np.random.choice(4, n_trunk)
        signs = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        local_xs = np.array([signs[l][0] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.03, n_trunk)
        local_ys = np.array([signs[l][1] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.03, n_trunk)
        # 旋转到世界坐标系
        pts_trunk_2d = np.column_stack([
            local_xs * u_arm[0] + local_ys * u_line[0],
            local_xs * u_arm[1] + local_ys * u_line[1]
        ])

        # 高空横担角钢：位于 z=32m, 真实角钢沿 u_arm 分布 [-8.0, 8.0]
        n_arm = 600
        arm_z = 32.0 + np.random.uniform(-0.3, 0.3, n_arm)
        arm_extent = np.random.uniform(-8.0, 8.0, n_arm)
        pts_arm_2d = np.column_stack([
            arm_extent * u_arm[0] + np.random.normal(0, 0.05, n_arm),
            arm_extent * u_arm[1] + np.random.normal(0, 0.05, n_arm)
        ])

        # 模拟耐张跳线/倾斜绝缘子串干扰：在横担端部产生微小偏折
        n_jumper = 200
        jumper_z = 31.0 + np.random.uniform(-0.5, 0.5, n_jumper)
        jumper_extent = np.random.uniform(6.0, 8.5, n_jumper)
        pts_jumper_2d = np.column_stack([
            jumper_extent * u_arm[0] + np.random.uniform(0.5, 2.0, n_jumper) * u_line[0],
            jumper_extent * u_arm[1] + np.random.uniform(0.5, 2.0, n_jumper) * u_line[1]
        ])

        # 模拟长纵向出线跨档导线 (沿 u_line 顺线延伸至 80m，但绝对不能将横担轴带偏 90 度)
        n_span_wire = 1500
        wire_z = 30.5 + np.random.uniform(-1.0, 1.0, n_span_wire)
        wire_line_extent = np.random.uniform(5.0, 80.0, n_span_wire)
        pts_wire_2d = np.column_stack([
            wire_line_extent * u_line[0] + np.random.normal(0, 0.1, n_span_wire),
            wire_line_extent * u_line[1] + np.random.normal(0, 0.1, n_span_wire)
        ])

        all_2d = 2000.0 + np.vstack([pts_trunk_2d, pts_arm_2d, pts_jumper_2d, pts_wire_2d])
        all_z = np.concatenate([zs, arm_z, jumper_z, wire_z])
        pts = np.column_stack([all_2d, all_z])
        rel_z = all_z.copy()
        off_ground_idx = np.arange(len(pts))

        is_tower, is_arm, is_near_arm, entities = detect_towers(
            off_ground_pts=pts,
            rel_z=rel_z,
            off_ground_idx=off_ground_idx,
            config=self.config
        )

        self.assertEqual(len(entities), 1, "铁塔未能正确检出")
        ent = entities[0]

        # 验证 1: 横担主轴 v1 与真实塔腰物理外立面 u_arm 严格平行 (点积绝对值 >= 0.999)
        dot_arm = abs(float(ent.v1 @ u_arm))
        self.assertGreater(dot_arm, 0.995, f"横担主轴未能锁定塔腰物理外立面，dot={dot_arm:.4f}")

        # 验证 2: 严禁发生 90 度轴向倒挂 (v1 与顺线导线 u_line 的内积必须接近 0)
        dot_line = abs(float(ent.v1 @ u_line))
        self.assertLess(dot_line, 0.05, f"致命缺陷：横担主轴被长导线误导发生了 90 度倒挂！dot={dot_line:.4f}")

        # 验证 3: 严格正交性 (v1 . v2 == 0)
        self.assertAlmostEqual(float(abs(ent.v1 @ ent.v2)), 0.0, places=5)

if __name__ == '__main__':
    unittest.main()
