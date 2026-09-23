# -*- coding: utf-8 -*-
import unittest
import numpy as np
from modules.config import PipelineConfig
from modules.tower_detector import detect_towers

class TestSlopeTowerTop(unittest.TestCase):
    """
    Ticket 01 & 02 & 03: 陡坡地形猫头塔顶部羊角/地线支架防截断与完整捕获测试
    """
    def setUp(self):
        self.config = PipelineConfig()
        self.config.tower.voltage_mode = 'high_voltage'
        self.config.tower.allow_distribution_poles = False
        self.config.tower.min_pts_count = 300

    def test_steep_slope_tower_top_capture(self):
        """
        验证陡坡地形（基底高程落差 12m~20m）下，猫头塔外展羊角角钢在塔顶
        不因坡底相对高程膨胀 (local_rel_z > max_z + 4.0) 被切除，
        且不因 top_bracket_mask 狭窄被截断。
        """
        np.random.seed(42)
        cx, cy = 200.0, 200.0
        
        # 1. 构造塔身四根立柱 (长短腿落地 3045m ~ 3090m)
        n_trunk = 3500
        zs_abs = np.random.uniform(3045.0, 3090.0, n_trunk)
        w = 4.5 - 2.5 * ((zs_abs - 3045.0) / 45.0)
        legs = np.random.choice(4, n_trunk)
        signs = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        xs = np.array([signs[l][0] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.05, n_trunk)
        ys = np.array([signs[l][1] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.05, n_trunk)

        # 2. 中部横担 (3080m ~ 3093m, 横向翼展半宽 6.5m)
        n_arm = 1500
        arm_z = np.random.uniform(3080.0, 3093.0, n_arm)
        arm_x = np.random.uniform(-6.5, 6.5, n_arm)
        arm_y = np.random.uniform(-1.0, 1.0, n_arm)

        # 3. 塔顶猫头羊角 (3093m ~ 3100m，外展至半宽 5.8m)
        n_horn = 800
        horn_z = np.random.uniform(3093.0, 3100.0, n_horn)
        horn_u = (horn_z - 3093.0) / 7.0
        horn_w = 1.5 + horn_u * 4.3  # 从 1.5m 外展至 5.8m
        horn_side = np.random.choice([-1.0, 1.0], size=n_horn)
        horn_x = horn_side * (horn_w + np.random.normal(0, 0.05, n_horn))
        horn_y = np.random.uniform(-0.8, 0.8, n_horn)

        all_x = cx + np.concatenate([xs, arm_x, horn_x])
        all_y = cy + np.concatenate([ys, arm_y, horn_y])
        all_z = np.concatenate([zs_abs, arm_z, horn_z])

        # 4. 陡坡地面：塔中心山梁地面高程为 3050.0m，向下坡 -X 方向迅速降至 3038.0m (落差 12m)
        ground_z = 3050.0 + 0.8 * (all_x - cx)
        rel_z = all_z - ground_z

        tower_pts = np.column_stack([all_x, all_y, all_z])
        off_ground_idx = np.arange(len(tower_pts))

        is_tower, is_tower_arm, is_near, entities = detect_towers(
            off_ground_pts=tower_pts,
            rel_z=rel_z,
            off_ground_idx=off_ground_idx,
            t_grid_size=2.0,
            config=self.config
        )

        self.assertEqual(len(entities), 1, "应准确检测到 1 座陡坡猫头塔")

        # 验证塔顶尖端羊角捕获率 (绝对高程 >= 3097m)
        top_mask = tower_pts[:, 2] >= 3097.0
        n_top = np.sum(top_mask)
        captured_top = np.sum(is_tower[top_mask])
        top_capture_rate = captured_top / max(n_top, 1)

        print(f"\n[TestSlopeTowerTop] 塔顶高空角钢总数: {n_top}, 捕获数: {captured_top}, 捕获率: {top_capture_rate:.2%}")
        # 修复前仅捕获约 49.0%（下坡侧羊角被全部误杀），修复后应达到 > 95%
        self.assertGreater(top_capture_rate, 0.95, "陡坡塔顶羊角捕获率应大于 95%")

if __name__ == '__main__':
    unittest.main()
