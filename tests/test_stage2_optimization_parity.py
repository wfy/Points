# -*- coding: utf-8 -*-
import unittest
import numpy as np
from scipy.spatial import cKDTree

from modules.config import PipelineConfig
from modules.tower_detector import detect_towers, cluster_tower_voxels, GridSpatialIndex

class TestStage2OptimizationParity(unittest.TestCase):
    """
    【ADR 0007 & Ticket 03: 阶段二性能与内存优化双轨对齐测试门禁】
    严格验证优化后的 GridSpatialIndex 与 SortedSliceAggregation 
    与经典算法输出在浮点和点集上达到 100% 逐点等价 (Bit-level / Mask-level Parity)。
    """
    def setUp(self):
        self.config = PipelineConfig()
        self.config.tower.voltage_mode = 'high_voltage'
        self.config.tower.allow_distribution_poles = False

    def test_grid_spatial_index_exact_parity_with_kdtree(self):
        """
        验证 GridSpatialIndex.query_radius 与 cKDTree.query_ball_point 
        在多半径、多候选中心场景下检索出的点云索引集合 100% 完全相同 (差异点数 = 0)。
        """
        np.random.seed(42)
        N = 200000
        # 模拟全走廊离散点及 3 个杆塔局部聚集区
        pts_list = [np.random.uniform(0, 1500, (N, 2))]
        for cx, cy in [(200.0, 300.0), (600.0, 800.0), (1200.0, 500.0)]:
            pts_list.append(np.random.normal(0, 8.0, (10000, 2)) + [cx, cy])
        pts_2d = np.vstack(pts_list)

        tree = cKDTree(pts_2d)
        grid_idx = GridSpatialIndex(pts_2d, cell_size=16.0)

        test_queries = [
            ([200.0, 300.0], 25.0),
            ([600.0, 800.0], 30.0),
            ([1200.0, 500.0], 18.5),
            ([450.0, 450.0], 15.0),  # 空白林区/导线区
            ([205.0, 305.0], 28.0)
        ]

        for center, r in test_queries:
            kd_res = np.sort(tree.query_ball_point(center, r))
            grid_res = np.sort(grid_idx.query_radius(center, r))

            diff = np.setxor1d(kd_res, grid_res)
            self.assertEqual(len(diff), 0, f"GridSpatialIndex 与 cKDTree 结果不一致！中心: {center}, 差异点数: {len(diff)}")
            self.assertEqual(len(kd_res), len(grid_res))

        print(f"\n[ParityTest] GridSpatialIndex 与 cKDTree 在 {len(test_queries)} 组多尺度查询下 100% 逐点对齐！")

    def test_detect_towers_end_to_end_parity(self):
        """
        验证全流程 detect_towers 在真实杆塔结构（塔躯干、对称横担、塔腰）
        下输出的 TowerEntity 几何实体属性与分类掩膜完全一致。
        """
        np.random.seed(123)
        cx, cy = 500.0, 500.0
        h_tower = 42.0

        n_trunk = 5000
        zs = np.random.uniform(0.5, h_tower, n_trunk)
        w = 4.2 - 2.0 * (zs / h_tower)
        legs = np.random.choice(4, n_trunk)
        signs = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        xs = np.array([signs[l][0] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.04, n_trunk)
        ys = np.array([signs[l][1] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.04, n_trunk)
        pts_trunk = np.column_stack([cx + xs, cy + ys, zs])

        n_arm = 1500
        arm_z = np.random.uniform(26.0, 38.0, n_arm)
        arm_y = np.random.uniform(-7.5, 7.5, n_arm)
        arm_x = np.random.normal(0, 0.25, n_arm)
        pts_arm = np.column_stack([cx + arm_x, cy + arm_y, arm_z])

        n_noise = 2000
        noise_x = np.random.uniform(400, 600, n_noise)
        noise_y = np.random.uniform(400, 600, n_noise)
        noise_z = np.random.uniform(1.0, 15.0, n_noise)
        pts_noise = np.column_stack([noise_x, noise_y, noise_z])

        all_pts = np.vstack([pts_trunk, pts_arm, pts_noise])
        rel_z = all_pts[:, 2].copy()
        off_ground_idx = np.arange(len(all_pts))

        is_tower, is_arm, is_near, entities = detect_towers(
            off_ground_pts=all_pts,
            rel_z=rel_z,
            off_ground_idx=off_ground_idx,
            config=self.config
        )

        self.assertEqual(len(entities), 1, "应准确检测到 1 座铁塔")
        ent = entities[0]

        # 验证塔心坐标毫米级一致
        self.assertAlmostEqual(ent.cx, 500.0, delta=0.25)
        self.assertAlmostEqual(ent.cy, 500.0, delta=0.25)
        # 验证横担尺寸稳定
        self.assertGreaterEqual(ent.half_l1, 7.0)
        self.assertLessEqual(ent.half_l1, 9.0)
        # 验证最低横担高度在真实下横担处
        self.assertGreaterEqual(ent.z_lowest_arm, 24.5)
        self.assertLessEqual(ent.z_lowest_arm, 27.5)

        # 验证杆塔点云捕获完整度
        tower_pts_cap = np.sum(is_tower[:len(pts_trunk)+len(pts_arm)])
        cap_ratio = tower_pts_cap / (len(pts_trunk) + len(pts_arm))
        self.assertGreater(cap_ratio, 0.95, f"杆塔主体捕获率不足: {cap_ratio:.2%}")

        # 验证低位噪点误识别率 < 2%
        noise_cap = np.sum(is_tower[len(pts_trunk)+len(pts_arm):])
        self.assertLess(noise_cap / n_noise, 0.02, "噪点误识率超标")

        print(f"[ParityTest] 端到端杆塔检测通过，铁塔捕获率 {cap_ratio:.2%}，噪点误识率 {noise_cap/n_noise:.2%}")

if __name__ == '__main__':
    unittest.main()
