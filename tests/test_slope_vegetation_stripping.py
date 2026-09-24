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

    def test_bilateral_symmetry_rejects_single_side_hillside_tree_patch(self):
        """
        Ticket 02 (cloud0): 验证单侧向上隆起的边坡树木（外伸至 12m~14m）绝不拉偏横担半宽，
        左右横担严格保持对称约束，右侧边坡孤立高位树斑被彻底剔除
        """
        np.random.seed(42)
        cx, cy = 200.0, 200.0
        h_tower = 45.0
        
        # 1. 铁塔主躯干 (Z: 0 ~ 45m)
        n_trunk = 4500
        zs = np.random.uniform(0.5, h_tower, n_trunk)
        w = 4.0 - 2.0 * (zs / h_tower)
        legs = np.random.choice(4, n_trunk)
        signs = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        xs = np.array([signs[l][0] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.04, n_trunk)
        ys = np.array([signs[l][1] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.04, n_trunk)
        pts_trunk = np.column_stack([cx + xs, cy + ys, zs])
        
        # 2. 真实对称横担：左翼 -4.6m，右翼 +4.6m，位于 Z in [28.0, 42.0]m
        n_arm = 1500
        arm_z = np.random.uniform(28.0, 42.0, n_arm)
        arm_y = np.random.uniform(-4.6, 4.6, n_arm)
        arm_x = np.random.normal(0, 0.25, n_arm)
        pts_arm = np.column_stack([cx + arm_x, cy + arm_y, arm_z])
        
        # 3. 右侧边坡孤立高位树冠 (Z: 27.0m ~ 33.0m, y 方向 [9.0, 14.0]m，模拟 cloud0 右侧边坡悬空树斑)
        n_tree = 1000
        tree_z = np.random.uniform(27.0, 33.0, n_tree)
        tree_y = np.random.uniform(9.0, 14.0, n_tree)
        tree_x = np.random.uniform(-2.0, 2.0, n_tree)
        pts_tree = np.column_stack([cx + tree_x, cy + tree_y, tree_z])
        
        all_pts = np.vstack([pts_trunk, pts_arm, pts_tree])
        rel_z = all_pts[:, 2].copy()
        off_ground_idx = np.arange(len(all_pts))
        
        is_tower, is_arm, is_near, entities = detect_towers(
            off_ground_pts=all_pts,
            rel_z=rel_z,
            off_ground_idx=off_ground_idx,
            config=self.config
        )
        
        self.assertEqual(len(entities), 1)
        tower = entities[0]
        
        # 验证 1: 横担半宽不得被右侧 14m 树木拉偏膨胀，应严格约束在 ~5.0m 以内
        print(f"[TestBilateralSymmetry] 检测横担半宽 half_l1: {tower.half_l1:.2f}m (真实物理翼展 4.6m)")
        self.assertLessEqual(tower.half_l1, 5.5, f"横担半宽被单侧树木拉偏膨胀: {tower.half_l1:.2f}m")
        self.assertGreaterEqual(tower.half_l1, 4.5, f"横担半宽被过度压缩: {tower.half_l1:.2f}m")
        
        # 验证 2: 右侧 9m~14m 孤立边坡高位树斑 0 误判 (误识别率 < 2%)
        tree_start_idx = len(pts_trunk) + len(pts_arm)
        tree_is_tower = is_tower[tree_start_idx:]
        false_tree_rate = np.sum(tree_is_tower) / n_tree
        print(f"[TestBilateralSymmetry] 边坡高位悬空树斑误判比例: {false_tree_rate:.2%}")
        self.assertLess(false_tree_rate, 0.02, "边坡高位悬空树斑应被彻底剔除！")

    def test_tension_tower_multi_tier_thick_crossarm_detected(self):
        """
        Ticket 01 (cloud7): 验证耐张塔多层大翼展宽厚横担 (顺线厚度 4.0m~5.0m, 横向翼展 16m)
        不被 w2 <= 2.07m 误杀，最低横担精准定位在 ~26m 真实下横担处，杜绝分割线虚高至塔顶 43m
        """
        np.random.seed(42)
        cx, cy = 300.0, 300.0
        h_tower = 46.0
        
        # 1. 铁塔主躯干 (Z: 0 ~ 46m)
        n_trunk = 5000
        zs = np.random.uniform(0.5, h_tower, n_trunk)
        w = 4.6 - 2.2 * (zs / h_tower)
        legs = np.random.choice(4, n_trunk)
        signs = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        xs = np.array([signs[l][0] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.04, n_trunk)
        ys = np.array([signs[l][1] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.04, n_trunk)
        pts_trunk = np.column_stack([cx + xs, cy + ys, zs])
        
        # 2. 三层耐张横担 (相对高程 26m, 32m, 38m；顺线耐张绝缘子串/跳线厚度 4.5m 即 x in [-2.25, 2.25], 翼展 16m 即 y in [-8, 8])
        tier_heights = [26.0, 32.0, 38.0]
        arm_pts_list = []
        for zh in tier_heights:
            n_t = 600
            t_z = np.random.uniform(zh - 0.6, zh + 0.6, n_t)
            t_y = np.random.uniform(-8.0, 8.0, n_t)
            t_x = np.random.uniform(-2.25, 2.25, n_t)
            arm_pts_list.append(np.column_stack([cx + t_x, cy + t_y, t_z]))
        pts_arms = np.vstack(arm_pts_list)
        
        # 3. 塔顶地线羊角架 (Z: 43.5m ~ 45.0m, 横向翼展 4.5m)
        n_top = 300
        top_z = np.random.uniform(43.5, 45.0, n_top)
        top_y = np.random.uniform(-2.25, 2.25, n_top)
        top_x = np.random.uniform(-0.5, 0.5, n_top)
        pts_top = np.column_stack([cx + top_x, cy + top_y, top_z])
        
        all_pts = np.vstack([pts_trunk, pts_arms, pts_top])
        rel_z = all_pts[:, 2].copy()
        off_ground_idx = np.arange(len(all_pts))
        
        is_tower, is_arm, is_near, entities = detect_towers(
            off_ground_pts=all_pts,
            rel_z=rel_z,
            off_ground_idx=off_ground_idx,
            config=self.config
        )
        
        self.assertEqual(len(entities), 1)
        tower = entities[0]
        
        print(f"[TestTensionThickArm] 检测到耐张塔最低横担高度 z_lowest_arm: {tower.z_lowest_arm:.2f}m (真实下横担 ~26m)")
        # 验证 1: 最低横担必须精准锁定在真实下横担 (<= 28m)，绝对不可虚高至 43m 羊角处
        self.assertLessEqual(tower.z_lowest_arm, 28.0, f"耐张横担被误杀，最低横担虚高至塔顶: {tower.z_lowest_arm:.2f}m")
        self.assertGreaterEqual(tower.z_lowest_arm, 24.0, f"最低横担偏低: {tower.z_lowest_arm:.2f}m")
        
        # 验证 2: 3层宽大横担必须 100% 位于分割线 abs_z_boundary 以上 (归入 UpperTowerBox 蓝色)
        arm_start_idx = len(pts_trunk)
        arm_end_idx = arm_start_idx + len(pts_arms)
        arms_is_tower = is_tower[arm_start_idx:arm_end_idx]
        arm_capture_rate = np.sum(arms_is_tower) / len(pts_arms)
        print(f"[TestTensionThickArm] 耐张横担捕获率: {arm_capture_rate:.2%}")
        self.assertGreater(arm_capture_rate, 0.95, "耐张横担点云捕获率应 >= 95%")

if __name__ == '__main__':
    unittest.main()
