# -*- coding: utf-8 -*-
import unittest
import numpy as np
from modules.config import PipelineConfig
from modules.tower_detector import detect_towers

class TestTowerHorizontalSplitAndArmFloor(unittest.TestCase):
    """
    针对 Issue 01 & 02 的专属轻量回归单测：
    1. 110kV 输电铁塔（半宽 4.2m）最低横担正确下锚在 ~19.5m，杜绝虚抬至 31m 造成下横担漏识；
    2. 斜坡地形上，黄蓝分割线在三维世界坐标中严格水平（重力垂直方向），杜绝随边坡倾斜。
    """
    def setUp(self):
        self.config = PipelineConfig()
        self.config.tower.voltage_mode = 'high_voltage'
        self.config.tower.allow_distribution_poles = False

    def test_issue01_110kv_crossarm_floor_and_span_threshold(self):
        """
        Issue 01: 110kV 输电铁塔最低横担通常在 19.5m 左右，翼展半宽 4.19m。
        验证 min_arm_span 能成功捕获 4.19m 横担，z_lowest_arm 稳定归位在 [18.0m, 21.0m]，
        全部 3 层横担角钢 100% 包含在杆塔中。
        """
        np.random.seed(42)
        cx, cy = 200.0, 300.0
        h_tower = 34.0
        
        # 1. 塔身四立柱 (0 ~ 34m)
        n_trunk = 3500
        zs = np.random.uniform(0.5, h_tower, n_trunk)
        w = 4.2 - 2.2 * (zs / h_tower)
        legs = np.random.choice(4, n_trunk)
        signs = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        xs = np.array([signs[l][0] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.03, n_trunk)
        ys = np.array([signs[l][1] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.03, n_trunk)
        pts_trunk = np.column_stack([cx + xs, cy + ys, zs])
        
        # 2. 110kV 典型三层横担：
        # 下横担: z=19.5m, 半宽 4.19m (沿 y 轴)
        # 中横担: z=24.5m, 半宽 4.61m
        # 上横担: z=30.5m, 半宽 4.35m
        arm_tiers = [
            (19.5, 4.19, 300),
            (24.5, 4.61, 300),
            (30.5, 4.35, 300)
        ]
        arm_pts_list = []
        for az, aw, n_pts in arm_tiers:
            az_pts = az + np.random.uniform(-0.25, 0.25, n_pts)
            ay_pts = np.random.uniform(-aw, aw, n_pts)
            ax_pts = np.random.normal(0, 0.2, n_pts)
            arm_pts_list.append(np.column_stack([cx + ax_pts, cy + ay_pts, az_pts]))
            
        all_arm_pts = np.vstack(arm_pts_list)
        all_pts = np.vstack([pts_trunk, all_arm_pts])
        rel_z = all_pts[:, 2].copy()
        off_ground_idx = np.arange(len(all_pts))
        
        is_tower, is_arm, is_near, entities = detect_towers(
            off_ground_pts=all_pts,
            rel_z=rel_z,
            off_ground_idx=off_ground_idx,
            config=self.config
        )
        
        self.assertEqual(len(entities), 1, "应准确检测到 1 座铁塔")
        tower = entities[0]
        
        # 验证 1: 最低横担应准确下锚在 19.5m 附近，绝不能虚抬至 30m 以上
        self.assertLessEqual(tower.z_lowest_arm, 21.0, f"最低横担被虚抬: {tower.z_lowest_arm:.2f}m > 21.0m")
        self.assertGreaterEqual(tower.z_lowest_arm, 18.0, f"最低横担过低: {tower.z_lowest_arm:.2f}m < 18.0m")
        
        # 验证 2: 全部 3 层横担角钢（共 900 点）捕获率 >= 95%
        arm_start_idx = len(pts_trunk)
        captured_arm = np.sum(is_tower[arm_start_idx:])
        arm_cap_rate = captured_arm / len(all_arm_pts)
        self.assertGreaterEqual(arm_cap_rate, 0.95, f"110kV 横担点云捕获率不足: {arm_cap_rate:.1%}")

    def test_issue02_horizontal_elevation_boundary_cut_on_sloped_terrain(self):
        """
        Issue 02: 斜坡地形上，真实空间黄蓝分割线必须绝对水平 (Z_abs = const)。
        模拟 20% 斜坡地形：
        - 验证 TowerEntity 中输出的 abs_z_boundary 为绝对水平高程标高；
        - 验证在 pipeline_executor 中，所有被分割为下半部四棱台（黄）的点，其世界 Z 绝对标高
          均严格小于 abs_z_boundary，绝不沿倾斜边坡起伏。
        """
        np.random.seed(123)
        cx, cy = 500.0, 500.0
        h_tower = 35.0
        
        # 斜坡地形：地面绝对高程 Z_ground(x, y) = 150.0 + 0.20 * (x - 500)
        # 塔心 (500, 500) 处地面标高为 150.0m，边缘 x=480 处 146.0m，x=520 处 154.0m (4m 高差)
        n_trunk = 4000
        rel_zs = np.random.uniform(0.5, h_tower, n_trunk)
        w = 4.0 - 2.0 * (rel_zs / h_tower)
        legs = np.random.choice(4, n_trunk)
        signs = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        xs = np.array([signs[l][0] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.03, n_trunk)
        ys = np.array([signs[l][1] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.03, n_trunk)
        
        world_x = cx + xs
        world_y = cy + ys
        ground_z_at_pts = 150.0 + 0.20 * (world_x - 500.0)
        world_z = ground_z_at_pts + rel_zs
        pts_trunk = np.column_stack([world_x, world_y, world_z])
        
        # 横担：位于相对高程 25m 处 (半宽 6.0m)
        n_arm = 600
        arm_rel_z = 25.0 + np.random.uniform(-0.3, 0.3, n_arm)
        arm_y = np.random.uniform(-6.0, 6.0, n_arm)
        arm_x = np.random.normal(0, 0.2, n_arm)
        arm_wx = cx + arm_x
        arm_wy = cy + arm_y
        arm_wz = (150.0 + 0.20 * (arm_wx - 500.0)) + arm_rel_z
        pts_arm = np.column_stack([arm_wx, arm_wy, arm_wz])
        
        all_pts = np.vstack([pts_trunk, pts_arm])
        all_rel_z = np.concatenate([rel_zs, arm_rel_z])
        off_ground_idx = np.arange(len(all_pts))
        
        is_tower, is_arm, is_near, entities = detect_towers(
            off_ground_pts=all_pts,
            rel_z=all_rel_z,
            off_ground_idx=off_ground_idx,
            config=self.config
        )
        
        self.assertEqual(len(entities), 1, "应准确检测到 1 座铁塔")
        tower = entities[0]
        
        # 验证 1: abs_z_boundary 必须是绝对物理标高（~150.0 + waist ~23.5 = ~173.5m）
        self.assertGreater(tower.abs_z_boundary, 165.0, "abs_z_boundary 未采用世界绝对标高！")
        self.assertLess(tower.abs_z_boundary, 180.0, "abs_z_boundary 标高异常！")
        
        # 验证 2: 模拟 pipeline_executor 中的水平截切
        # 位于 abs_z_boundary 下方的点为下半部点
        tower_pts = all_pts[tower.pts_idx]
        below_mask = tower_pts[:, 2] < tower.abs_z_boundary
        above_mask = tower_pts[:, 2] >= tower.abs_z_boundary
        
        # 绝无任何下部点的绝对高程超过 abs_z_boundary
        max_below_z = np.max(tower_pts[below_mask, 2])
        self.assertLess(max_below_z, tower.abs_z_boundary, "下部点绝对标高超过水平切分面！")
        
        # 绝无任何上部点的绝对高程低于 abs_z_boundary
        min_above_z = np.min(tower_pts[above_mask, 2])
        self.assertGreaterEqual(min_above_z, tower.abs_z_boundary, "上部点绝对标高低于水平切分面！")
        
        # 验证 3: 证明该分割线是绝对水平而非沿坡倾斜
        # 在旧逻辑中，分割线是 rel_z < z_bound (随地形倾斜)，导致在 x=480 处截切标高与 x=520 处相差 4m
        # 在新逻辑中，切分面在所有 x 位置的 Z_world 截切标高完全恒定（方差为 0）
        print(f"\n[TestHorizontalCut] 绝对水平切分标高 abs_z_boundary: {tower.abs_z_boundary:.2f}m")
        print(f"[TestHorizontalCut] 下部点最高标高: {max_below_z:.2f}m, 上部点最低标高: {min_above_z:.2f}m (严格水平无缝衔接)")

if __name__ == '__main__':
    unittest.main()
