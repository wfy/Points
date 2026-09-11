# -*- coding: utf-8 -*-
import unittest
import numpy as np
from modules.config import PipelineConfig, DEFAULT_CONFIG
from modules.models import TowerEntity, WireCluster
from modules.wire_tracker import filter_canopy_by_probes
from modules.wire_seed_extractor import extract_wire_seeds

class TestCanopyFiltering(unittest.TestCase):
    """
    Task 3: 树冠尖顶伪种子探针滤波与高空抗噪单元测试
    """
    def setUp(self):
        self.config = PipelineConfig()

    def test_filter_canopy_by_probes_air_gap(self):
        """
        验证探针空气隔离层：
        - 真实空中悬挂导线 (下方为空气) -> 绝不误判为 suspect
        - 假导线/树梢伪脊线 (下方 1.2m 为密集实心树冠) -> 必须被准确标记为 suspect
        """
        # 1. 构建环境非地面点云：包含一座茂密树木
        # 树冠中心 (50, 0, 20)，半径 3m 的实心点球
        np.random.seed(42)
        n_tree = 500
        tree_r = np.random.uniform(0, 2.5, n_tree)
        tree_theta = np.random.uniform(0, 2 * np.pi, n_tree)
        tree_phi = np.random.uniform(0, np.pi, n_tree)
        tree_x = 50.0 + tree_r * np.sin(tree_phi) * np.cos(tree_theta)
        tree_y = 0.0 + tree_r * np.sin(tree_phi) * np.sin(tree_theta)
        tree_z = 20.0 + tree_r * np.cos(tree_phi)
        tree_pts = np.column_stack([tree_x, tree_y, tree_z])

        # 2. 真实导线：空中悬挂 (x: 20~80, y: 0, z: 30)
        n_wire = 100
        wire_x = np.linspace(20, 80, n_wire)
        wire_y = np.zeros(n_wire)
        wire_z = np.full(n_wire, 30.0)
        real_wire_pts = np.column_stack([wire_x, wire_y, wire_z])

        # 3. 树冠尖顶伪线：紧贴树冠顶部 (x: 48~52, y: 0, z: 22.5) 下方 1.2m 正好是密集树冠内部 (z=21.3)
        n_fake = 30
        fake_x = np.linspace(48.5, 51.5, n_fake)
        fake_y = np.zeros(n_fake)
        fake_z = np.full(n_fake, 22.5)
        fake_wire_pts = np.column_stack([fake_x, fake_y, fake_z])

        # 综合非地面点云
        off_ground_pts = np.vstack([tree_pts, real_wire_pts, fake_wire_pts])

        # 构造聚类实体
        cluster_real = WireCluster(
            members=list(range(len(tree_pts), len(tree_pts) + len(real_wire_pts))),
            center=np.mean(real_wire_pts, axis=0),
            dir=np.array([1.0, 0.0, 0.0]),
            span=60.0,
            linearity=0.98,
            min_var=0.01,
            catenary=None
        )

        cluster_fake = WireCluster(
            members=list(range(len(tree_pts) + len(real_wire_pts), len(off_ground_pts))),
            center=np.mean(fake_wire_pts, axis=0),
            dir=np.array([1.0, 0.0, 0.0]),
            span=3.0,
            linearity=0.92,
            min_var=0.02,
            catenary=None
        )

        final_cable_pts = off_ground_pts
        all_confirmed = [cluster_real, cluster_fake]
        tower_infos = [
            TowerEntity(cx=0.0, cy=0.0, max_z=35.0, abs_max_z=35.0),
            TowerEntity(cx=100.0, cy=0.0, max_z=35.0, abs_max_z=35.0)
        ]

        suspect_ids = filter_canopy_by_probes(
            final_cable_pts=final_cable_pts,
            off_ground_pts=off_ground_pts,
            all_confirmed=all_confirmed,
            tower_infos=tower_infos
        )

        # 验证：真实的空中导线 (id=1) 绝不能被判定为 suspect
        self.assertNotIn(1, suspect_ids, "真导线下方为空气，不应被探针误杀")
        # 验证：紧贴实心树冠顶部的伪线 (id=2) 必须被准确标记为 suspect
        self.assertIn(2, suspect_ids, "树冠顶部伪导线下方为实心树木，必须被探针剔除")

    def test_wire_seed_orientation_and_density_constraint(self):
        """
        验证种子点提取阶段：
        - 走向与跨档轴线平行的候选线段 -> 保留为种子
        - 走向与跨档轴线夹角 > 35 度的斜向树枝 -> 剔除
        - 局部体密度过大/厚度过大的实心树梢 -> 剔除
        """
        # 两座铁塔，跨档沿 X 轴 (y=0)
        tower_infos = [
            TowerEntity(cx=0.0, cy=0.0, max_z=35.0, abs_max_z=35.0, v2=np.array([1.0, 0.0])),
            TowerEntity(cx=100.0, cy=0.0, max_z=35.0, abs_max_z=35.0, v2=np.array([1.0, 0.0]))
        ]

        # 1. 平行于跨档的纤细导线段 (走向 [1, 0, 0], 夹角 0度)
        x_wire = np.linspace(40, 60, 50)
        y_wire = np.zeros(50)
        z_wire = np.full(50, 25.0)
        pts_wire = np.column_stack([x_wire, y_wire, z_wire])

        # 2. 走向偏离 45 度的树枝 (走向 [1, 1, 0] / sqrt(2), 夹角 45度 > 35度)
        branch_t = np.linspace(-10, 10, 50)
        x_branch = 50.0 + branch_t * np.cos(np.radians(45.0))
        y_branch = 15.0 + branch_t * np.sin(np.radians(45.0))
        z_branch = np.full(50, 25.0)
        pts_branch = np.column_stack([x_branch, y_branch, z_branch])

        # 3. 走向偏离 90 度的横向树枝 (走向 [0, 1, 0], 夹角 90度)
        x_cross = np.full(50, 50.0)
        y_cross = np.linspace(-25, -5, 50)
        z_cross = np.full(50, 25.0)
        pts_cross = np.column_stack([x_cross, y_cross, z_cross])

        # 4. 密集树冠 (高密度、高厚度)
        np.random.seed(123)
        n_dense = 150
        pts_dense = np.column_stack([
            50.0 + np.random.uniform(-0.8, 0.8, n_dense),
            5.0 + np.random.uniform(-0.8, 0.8, n_dense),
            25.0 + np.random.uniform(-0.8, 0.8, n_dense)
        ])

        high_pts = np.vstack([pts_wire, pts_branch, pts_cross, pts_dense])
        high_indices = np.arange(len(high_pts))
        high_is_near_arm = np.zeros(len(high_pts), dtype=bool)
        high_is_tower = np.zeros(len(high_pts), dtype=bool)

        seeds = extract_wire_seeds(
            high_pts=high_pts,
            high_indices=high_indices,
            high_is_near_arm=high_is_near_arm,
            high_is_tower=high_is_tower,
            tower_infos=tower_infos,
            seed_grid_size=0.8,
            pca_radius=2.0,
            linearity_thresh=0.80,
            wire_seed_l3_max=0.30,
            wire_seed_l3_bundle_max=0.60,
            wire_seed_density_max=60,
            enable_bundle_adapt=False
        )

        wire_indices = set(range(0, 50))
        branch_indices = set(range(50, 100))
        cross_indices = set(range(100, 150))
        dense_indices = set(range(150, len(high_pts)))

        extracted_set = set(seeds)
        wire_detected = len(extracted_set.intersection(wire_indices))
        branch_detected = len(extracted_set.intersection(branch_indices))
        cross_detected = len(extracted_set.intersection(cross_indices))
        dense_detected = len(extracted_set.intersection(dense_indices))

        self.assertGreater(wire_detected, 0, "平行于档距主轴的真导线必须检出种子")
        self.assertEqual(branch_detected, 0, "走向偏离 > 35 度的斜向树枝不得检出为种子")
        self.assertEqual(cross_detected, 0, "走向垂直于档距主轴的树枝不得检出为种子")
        self.assertEqual(dense_detected, 0, "高密度/大厚度的实心树冠顶不得检出为种子")

if __name__ == '__main__':
    unittest.main()
