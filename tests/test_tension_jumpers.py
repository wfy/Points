# -*- coding: utf-8 -*-
import unittest
import numpy as np
from modules.config import PipelineConfig, DEFAULT_CONFIG
from modules.models import TowerEntity
from modules.tension_topology import extract_tension_jumpers
from modules.wire_seed_extractor import extract_wire_seeds

class TestTensionJumpers(unittest.TestCase):
    """
    Task 4: 分裂导线自适应与耐张跳线专用提取单元测试
    """
    def setUp(self):
        self.config = PipelineConfig()
        self.config.powerline.enable_jumper_extraction = True
        self.config.powerline.enable_bundle_conductor_adapt = True

    def test_extract_tension_jumpers_geometry(self):
        """
        验证耐张转角塔横担下方 U 形悬垂跳线提取：
        - 位于横担挂点下方且弯曲的跳线弧段 -> 100% 提取
        - 铁塔主体角钢骨架点 -> 0% 误入跳线
        """
        # 1. 构造铁塔实体 (cx=0, cy=0, 塔高 35m, 横担半长 8m, 最低横担 25m)
        tower = TowerEntity(
            cx=0.0,
            cy=0.0,
            max_z=35.0,
            abs_max_z=35.0,
            v1=np.array([0.0, 1.0]),  # 横担沿 Y 轴
            v2=np.array([1.0, 0.0]),  # 线路沿 X 轴
            half_l1=8.0,
            half_l2=2.5,
            z_lowest_arm=25.0,
            w_trunk0=1.5
        )
        tower_infos = [tower]

        # 2. 构造铁塔角钢骨架点
        np.random.seed(42)
        n_tower = 1000
        tower_x = np.random.uniform(-1.5, 1.5, n_tower)
        tower_y = np.random.uniform(-1.5, 1.5, n_tower)
        tower_z = np.random.uniform(0.0, 35.0, n_tower)
        tower_pts = np.column_stack([tower_x, tower_y, tower_z])

        # 3. 构造 2 根位于横担挂点下方的 U 形悬垂跳线 (横担端部 y=6.0, y=-6.0)
        # 弧线方程：z = z0 + c * x^2
        n_jumper_pts = 60
        jumper_pts_list = []
        for sign in [-1.0, 1.0]:
            y_j = sign * 6.0
            xs = np.linspace(-3.0, 3.0, n_jumper_pts)
            zs = 24.0 + 0.3 * (xs ** 2)  # 最低点 24.0m, 端点 26.7m
            ys = np.full(n_jumper_pts, y_j) + np.random.normal(0, 0.02, n_jumper_pts)
            jumper_pts_list.append(np.column_stack([xs, ys, zs]))

        all_jumpers = np.vstack(jumper_pts_list)
        off_ground_pts = np.vstack([tower_pts, all_jumpers])
        off_ground_idx = np.arange(len(off_ground_pts))
        rel_z = off_ground_pts[:, 2]

        is_tower = np.zeros(len(off_ground_pts), dtype=bool)
        is_tower[:n_tower] = True

        # 4. 执行跳线专用提取
        jumper_idx = extract_tension_jumpers(
            points=off_ground_pts,
            off_ground_pts=off_ground_pts,
            off_ground_idx=off_ground_idx,
            rel_z=rel_z,
            is_tower=is_tower,
            tower_infos=tower_infos,
            config=self.config
        )

        jumper_set = set(jumper_idx.tolist())
        true_jumper_indices = set(range(n_tower, len(off_ground_pts)))
        tower_indices = set(range(0, n_tower))

        detected_true = len(jumper_set.intersection(true_jumper_indices))
        detected_tower = len(jumper_set.intersection(tower_indices))

        self.assertGreater(detected_true, int(0.70 * len(true_jumper_indices)), "耐张跳线点大部分必须被成功检出")
        self.assertEqual(detected_tower, 0, "铁塔角钢骨架绝不能被误识别为跳线点")

    def test_bundle_conductor_seed_adaptation(self):
        """
        验证双分裂/四分裂导线多尺度空间滤波：
        - 子线间距 0.45m 的分裂导线点簇 -> 在 enable_bundle_adapt 下成功提取种子
        - 截面厚度 l3 处于 0.3m~0.6m 之间依然保持识别
        """
        tower_infos = [
            TowerEntity(cx=0.0, cy=0.0, max_z=35.0, abs_max_z=35.0, v2=np.array([1.0, 0.0])),
            TowerEntity(cx=100.0, cy=0.0, max_z=35.0, abs_max_z=35.0, v2=np.array([1.0, 0.0]))
        ]

        # 构造双分裂导线：两条平行子导线，间距 0.45m，沿 X 轴走向
        xs = np.linspace(30.0, 70.0, 100)
        # 子线 1: y = -0.225, z = 25.0
        sub1 = np.column_stack([xs, np.full(100, -0.225), np.full(100, 25.0)])
        # 子线 2: y = +0.225, z = 25.0
        sub2 = np.column_stack([xs, np.full(100, 0.225), np.full(100, 25.0)])
        bundle_pts = np.vstack([sub1, sub2])

        high_indices = np.arange(len(bundle_pts))
        high_is_near_arm = np.zeros(len(bundle_pts), dtype=bool)
        high_is_tower = np.zeros(len(bundle_pts), dtype=bool)

        seeds = extract_wire_seeds(
            high_pts=bundle_pts,
            high_indices=high_indices,
            high_is_near_arm=high_is_near_arm,
            high_is_tower=high_is_tower,
            tower_infos=tower_infos,
            seed_grid_size=0.8,
            pca_radius=2.0,
            linearity_thresh=0.82,
            bundle_adapt_linearity_thresh=0.70,
            wire_seed_l3_max=0.30,
            wire_seed_l3_bundle_max=0.60,
            wire_seed_density_max=60,
            enable_bundle_adapt=True
        )

        self.assertGreater(len(seeds), 0, "启用分裂导线自适应时，双分裂导线点必须被检出种子")

if __name__ == '__main__':
    unittest.main()
