# -*- coding: utf-8 -*-
import unittest
import numpy as np
from modules.config import PipelineConfig
from modules.tower_detector import detect_towers

class TestSlopeVegetationStripping(unittest.TestCase):
    """
    Ticket 01 & 02 & 03: 边坡低空植被防误判横担与四棱台树枝剔除测试
    """
    def setUp(self):
        self.config = PipelineConfig()
        self.config.tower.voltage_mode = 'high_voltage'
        self.config.tower.allow_distribution_poles = False

    def test_low_altitude_hillside_tree_rejection(self):
        """
        验证 34m 输电铁塔旁在 10m~23m 低空处生长的边坡茂密树木（宽达 13m）
        绝不被误判为最低横担或耐张跳线，z_lowest_arm 应正确锁定在真实高空横担 (>= 24.5m)，
        且该低空边坡树木绝大多数不被误分类为杆塔 (误识别率 < 10%)。
        """
        np.random.seed(42)
        cx, cy = 100.0, 100.0
        h_tower = 34.0
        
        # 1. 真实 34m 铁塔四立柱骨架 (Z: 0 ~ 34m, 腰宽 ~2.5m, 底部放坡至 ~4.5m)
        n_trunk = 4000
        zs = np.random.uniform(0.5, h_tower, n_trunk)
        w = 4.5 - 2.0 * (zs / h_tower)
        legs = np.random.choice(4, n_trunk)
        signs = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        xs = np.array([signs[l][0] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.04, n_trunk)
        ys = np.array([signs[l][1] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.04, n_trunk)
        pts_trunk = np.column_stack([cx + xs, cy + ys, zs])
        
        # 2. 真实输电铁塔高空横担 (Z: 26.0m ~ 32.0m, 横向翼展 14.0m 即 y 方向 [-7, 7])
        n_arm = 1200
        arm_z = np.random.uniform(26.0, 32.0, n_arm)
        arm_y = np.random.uniform(-7.0, 7.0, n_arm)
        arm_x = np.random.uniform(-0.6, 0.6, n_arm)
        pts_arm = np.column_stack([cx + arm_x, cy + arm_y, arm_z])
        
        # 3. 边坡左右两侧红框茂密树冠 (位于 Z: 10.0m ~ 23.0m, 对应图 1 左右两侧红框边坡 [-6.5, -2.5] 与 [2.5, 6.5])
        n_tree = 3000
        tree_z = np.random.uniform(10.0, 23.0, n_tree)
        tree_side = np.random.choice([-1.0, 1.0], size=n_tree)
        tree_y = tree_side * np.random.uniform(2.5, 6.5, n_tree)
        tree_x = np.random.uniform(-3.0, 3.0, n_tree)
        pts_tree = np.column_stack([cx + tree_x, cy + tree_y, tree_z])
        
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
        
        self.assertEqual(len(entities), 1, "应准确检测到 1 座铁塔")
        tower = entities[0]
        
        print(f"\n[TestSlopeVegetation] 检测到铁塔最低横担高度 z_lowest_arm: {tower.z_lowest_arm:.2f}m (真实高空横担 >= 24.5m)")
        # 验证 1: 最低横担绝对不应被拉低至 10m 的低空树冠处
        self.assertGreaterEqual(tower.z_lowest_arm, 24.5, "最低横担不应下陷至 10m 低空树冠处！")
        
        # 验证 2: 低空边坡树木绝大多数不应被误分类为杆塔
        tree_start_idx = len(pts_trunk) + len(pts_arm)
        tree_is_tower = is_tower[tree_start_idx:]
        false_positive_rate = np.sum(tree_is_tower) / n_tree
        print(f"[TestSlopeVegetation] 低空边坡树木误识别为杆塔比例: {false_positive_rate:.2%} (目标 < 10%)")
        self.assertLess(false_positive_rate, 0.10, "边坡低空树木误识别率应小于 10%")

if __name__ == '__main__':
    unittest.main()
