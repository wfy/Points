# -*- coding: utf-8 -*-
import unittest
import numpy as np
from modules.config import PipelineConfig
from modules.tower_detector import detect_towers

class TestSlopeTreeClusterRejection(unittest.TestCase):
    """
    Q2 Ticket 01 & 02 & 03: 跨档山坡独立与成片高大树林防误识别回归测试
    """
    def setUp(self):
        self.config = PipelineConfig()
        self.config.tower.voltage_mode = 'high_voltage'
        self.config.tower.allow_distribution_poles = False

    def test_isolated_tall_tree_canopy_rejection(self):
        """
        验证跨档山坡上 28m 高度、横向各向同性展宽达 18m 的独立密实树冠
        由于不存在任何刚性金属横担主梁且缺乏塔身收腰结构，
        绝不被误判为杆塔实体，仅检测到真正的输电铁塔。
        """
        np.random.seed(42)
        
        # 1. 真实输电铁塔 (位于 cx=100.0, cy=100.0, 高度 42m)
        cx_tower, cy_tower = 100.0, 100.0
        h_tower = 42.0
        n_trunk = 4500
        zs = np.random.uniform(0.5, h_tower, n_trunk)
        # 四棱台放坡：底宽 5.5m，腰宽 2.4m
        w = np.where(zs <= 25.0, 5.5 - (5.5 - 2.4) * (zs / 25.0), 2.4)
        legs = np.random.choice(4, n_trunk)
        signs = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        xs = np.array([signs[l][0] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.04, n_trunk)
        ys = np.array([signs[l][1] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.04, n_trunk)
        pts_trunk = np.column_stack([cx_tower + xs, cy_tower + ys, zs])
        
        # 真实横担 (Z: 32.0m ~ 38.0m, 沿 Y 方向横向展宽 16m, 顺线 X 方向厚度仅 1.0m)
        n_arm = 1500
        arm_z = np.random.uniform(32.0, 38.0, n_arm)
        arm_y = np.random.uniform(-8.0, 8.0, n_arm)
        arm_x = np.random.uniform(-0.5, 0.5, n_arm)
        pts_arm = np.column_stack([cx_tower + arm_x, cy_tower + arm_y, arm_z])
        
        # 2. 档中山坡独立密实高大树木 (位于 cx=220.0, cy=100.0，与杆塔相距 120m，类似 cloud2 档中伪目标)
        # 高度 32m，冠幅展宽 18m (各向同性半径 9m)，密实实心，无横担横梁
        cx_tree, cy_tree = 220.0, 100.0
        n_tree_trunk = 1500
        z_tree_trunk = np.random.uniform(0.5, 32.0, n_tree_trunk)
        r_tree_trunk = np.random.uniform(0, 0.9, n_tree_trunk)
        th_tree_trunk = np.random.uniform(0, 2*np.pi, n_tree_trunk)
        pts_tree_trunk = np.column_stack([cx_tree + r_tree_trunk * np.cos(th_tree_trunk), cy_tree + r_tree_trunk * np.sin(th_tree_trunk), z_tree_trunk])

        n_tree_canopy = 8000
        z_tree_canopy = np.random.uniform(18.0, 32.0, n_tree_canopy)
        r_tree_canopy = np.random.uniform(0, 8.5, n_tree_canopy)
        th_tree_canopy = np.random.uniform(0, 2*np.pi, n_tree_canopy)
        pts_tree_canopy = np.column_stack([cx_tree + r_tree_canopy * np.cos(th_tree_canopy), cy_tree + r_tree_canopy * np.sin(th_tree_canopy), z_tree_canopy])
        pts_tree = np.vstack([pts_tree_trunk, pts_tree_canopy])
        n_tree = len(pts_tree)
        
        all_pts = np.vstack([pts_trunk, pts_arm, pts_tree])
        rel_z = all_pts[:, 2].copy()
        off_ground_idx = np.arange(len(all_pts))
        
        is_tower, is_arm, is_near, entities = detect_towers(
            off_ground_pts=all_pts,
            rel_z=rel_z,
            off_ground_idx=off_ground_idx,
            t_grid_size=2.0,
            config=self.config
        )
        
        print(f"\n[TestSlopeTreeCluster] 检测到的杆塔总数: {len(entities)}")
        for idx, ent in enumerate(entities):
            print(f"  Entity {idx}: center=({ent.cx:.2f}, {ent.cy:.2f}), max_z={ent.max_z:.2f}, z_lowest_arm={ent.z_lowest_arm:.2f}")
            
        # 核心断言 1: 必须且仅能检测到 1 座真实铁塔
        self.assertEqual(len(entities), 1, "跨档山坡独立树丛绝不能被误检测为第 2 座杆塔！")
        
        # 核心断言 2: 检测到的杆塔中心必须是真实铁塔 (cx~100, cy~100)
        tower = entities[0]
        dist_to_real = np.hypot(tower.cx - cx_tower, tower.cy - cy_tower)
        self.assertLess(dist_to_real, 5.0, "检测到的杆塔必须为真实铁塔坐标")
        
        # 核心断言 3: 树丛点云误识别为杆塔比例必须极其微小 (< 2%)
        tree_start_idx = len(pts_trunk) + len(pts_arm)
        tree_is_tower = is_tower[tree_start_idx:]
        false_positive_rate = np.sum(tree_is_tower) / n_tree
        print(f"[TestSlopeTreeCluster] 独立树丛误识别比例: {false_positive_rate:.2%} (目标 < 2%)")
        self.assertLess(false_positive_rate, 0.02, "跨档大树丛不应被分类为杆塔")

    def test_tree_with_overhead_conductors_rejection(self):
        """
        验证双塔跨档走廊（Span 250m）中间山坡上生长的 28m 高大树木，
        上方即便有顺线架空导线穿过并与树冠相互交织（类似 cloud2 档中 Candidate 1 与 cloud7 图 4），
        由于顺线导线与跨档走廊平行而非垂直、且树木缺乏铁塔收腰与四立柱骨架，
        绝不被误判为第 3 座杆塔。
        """
        np.random.seed(42)
        
        # 1. 真实杆塔 A (cx=100.0, cy=100.0, H=42m)
        cx_a, cy_a = 100.0, 100.0
        h_a = 42.0
        n_trunk_a = 4000
        zs_a = np.random.uniform(0.5, h_a, n_trunk_a)
        w_a = np.where(zs_a <= 25.0, 5.5 - (5.5 - 2.4) * (zs_a / 25.0), 2.4)
        legs_a = np.random.choice(4, n_trunk_a)
        signs = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        xs_a = np.array([signs[l][0] * w_a[i]/2 for i, l in enumerate(legs_a)]) + np.random.normal(0, 0.04, n_trunk_a)
        ys_a = np.array([signs[l][1] * w_a[i]/2 for i, l in enumerate(legs_a)]) + np.random.normal(0, 0.04, n_trunk_a)
        pts_trunk_a = np.column_stack([cx_a + xs_a, cy_a + ys_a, zs_a])
        
        n_arm_a = 1200
        arm_z_a = np.random.uniform(32.0, 38.0, n_arm_a)
        arm_y_a = np.random.uniform(-8.0, 8.0, n_arm_a)
        arm_x_a = np.random.uniform(-0.5, 0.5, n_arm_a)
        pts_arm_a = np.column_stack([cx_a + arm_x_a, cy_a + arm_y_a, arm_z_a])
        
        # 2. 真实杆塔 B (cx=350.0, cy=100.0, H=42m，档距 250m，走向沿 X 轴)
        cx_b, cy_b = 350.0, 100.0
        h_b = 42.0
        n_trunk_b = 4000
        zs_b = np.random.uniform(0.5, h_b, n_trunk_b)
        w_b = np.where(zs_b <= 25.0, 5.5 - (5.5 - 2.4) * (zs_b / 25.0), 2.4)
        legs_b = np.random.choice(4, n_trunk_b)
        xs_b = np.array([signs[l][0] * w_b[i]/2 for i, l in enumerate(legs_b)]) + np.random.normal(0, 0.04, n_trunk_b)
        ys_b = np.array([signs[l][1] * w_b[i]/2 for i, l in enumerate(legs_b)]) + np.random.normal(0, 0.04, n_trunk_b)
        pts_trunk_b = np.column_stack([cx_b + xs_b, cy_b + ys_b, zs_b])
        
        n_arm_b = 1200
        arm_z_b = np.random.uniform(32.0, 38.0, n_arm_b)
        arm_y_b = np.random.uniform(-8.0, 8.0, n_arm_b)
        arm_x_b = np.random.uniform(-0.5, 0.5, n_arm_b)
        pts_arm_b = np.column_stack([cx_b + arm_x_b, cy_b + arm_y_b, arm_z_b])
        
        # 3. 档中密集山坡大树丛 (位于 cx=225.0, cy=100.0，H=28m，半径 9m)
        cx_tree, cy_tree = 225.0, 100.0
        n_tree = 6000
        tree_z = np.random.uniform(0.5, 28.0, n_tree)
        tree_r_max = np.where(tree_z < 10.0, 3.0 + 3.0 * (tree_z / 10.0), 9.0)
        tree_angle = np.random.uniform(0, 2 * np.pi, n_tree)
        tree_r = np.random.uniform(0, 1.0, n_tree) ** 0.5 * tree_r_max
        tree_x = cx_tree + tree_r * np.cos(tree_angle)
        tree_y = cy_tree + tree_r * np.sin(tree_angle)
        pts_tree = np.column_stack([tree_x, tree_y, tree_z])
        
        # 4. 跨中架空导线 (沿 X 轴延伸，穿过树林上方 Z=28m 附近)
        n_wire = 2000
        wire_x = np.random.uniform(105.0, 345.0, n_wire)
        wire_y = cy_a + np.random.choice([-6.0, 0.0, 6.0], n_wire) + np.random.normal(0, 0.05, n_wire)
        wire_z = 32.0 - 6.0 * (1.0 - ((wire_x - 225.0) / 125.0) ** 2) + np.random.normal(0, 0.05, n_wire)
        pts_wire = np.column_stack([wire_x, wire_y, wire_z])
        
        all_pts = np.vstack([pts_trunk_a, pts_arm_a, pts_trunk_b, pts_arm_b, pts_tree, pts_wire])
        rel_z = all_pts[:, 2].copy()
        off_ground_idx = np.arange(len(all_pts))
        
        is_tower, is_arm, is_near, entities = detect_towers(
            off_ground_pts=all_pts,
            rel_z=rel_z,
            off_ground_idx=off_ground_idx,
            t_grid_size=2.0,
            config=self.config
        )
        
        print(f"\n[TestSlopeTreeWithWires] 检测到的杆塔总数: {len(entities)} (期望为 2)")
        for idx, ent in enumerate(entities):
            print(f"  Entity {idx}: center=({ent.cx:.2f}, {ent.cy:.2f}), max_z={ent.max_z:.2f}, z_lowest_arm={ent.z_lowest_arm:.2f}")
            
        # 核心断言 1: 必须且仅能检测到 2 座真实铁塔，档中山坡树木+导线不得成为第 3 座伪塔
        self.assertEqual(len(entities), 2, "跨档山坡树木+导线绝不能被误判为第 3 座杆塔！")
        
        # 核心断言 2: 两座杆塔坐标分别对应 A 塔与 B 塔
        centers_x = sorted([e.cx for e in entities])
        self.assertAlmostEqual(centers_x[0], 100.0, delta=5.0)
        self.assertAlmostEqual(centers_x[1], 350.0, delta=5.0)

if __name__ == '__main__':
    unittest.main()
