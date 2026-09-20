import unittest
import numpy as np
from modules.tower_detector import cluster_tower_voxels

class TestTowerCenterLocalization(unittest.TestCase):
    def test_downhill_wire_offset_suppression(self):
        """
        验证当山顶存在真塔(Z=140~178m, 地面140m, rel_z=38m)、
        坡下存在大高差导线(Z=178m, 地面134m, rel_z=44m)时，
        初筛聚类中心坚决锁定在山顶真塔几何中心，绝不向坡下导线漂移。
        """
        np.random.seed(42)
        
        # 1. 模拟山顶真实铁塔 (中心 739386.0, 3380336.0, 地面 140m, 净高 38m)
        # 高位塔头与横担密集点云 (Z in [170, 178])
        n_tower_top = 8000
        tower_top_x = np.random.uniform(739384.0, 739388.0, n_tower_top)
        tower_top_y = np.random.uniform(3380334.0, 3380338.0, n_tower_top)
        tower_top_z = np.random.uniform(170.0, 178.5, n_tower_top)
        tower_top_rel_z = tower_top_z - 140.0
        
        # 塔身与塔腿点云 (Z in [140, 170])
        n_tower_body = 12000
        tower_body_x = np.random.uniform(739382.0, 739390.0, n_tower_body)
        tower_body_y = np.random.uniform(3380332.0, 3380340.0, n_tower_body)
        tower_body_z = np.random.uniform(140.0, 170.0, n_tower_body)
        tower_body_rel_z = tower_body_z - 140.0
        
        # 2. 模拟坡下林木与贯通悬空导线 (中心 739401.0, 3380343.0, 地面 134m)
        # 悬空导线 (Z in [174, 178], 净高高达 44m，但点数稀疏且沿线分布)
        n_wire = 1500
        wire_x = np.random.uniform(739390.0, 739410.0, n_wire)
        wire_y = 3380343.0 + (wire_x - 739401.0) * 0.7 + np.random.normal(0, 0.2, n_wire)
        wire_z = np.random.uniform(174.0, 178.0, n_wire)
        wire_rel_z = wire_z - 134.0  # 相对地高达 40~44m，高于真塔净高 38m
        
        # 下方林木 (Z in [135, 150])
        n_trees = 10000
        tree_x = np.random.uniform(739395.0, 739407.0, n_trees)
        tree_y = np.random.uniform(3380338.0, 3380348.0, n_trees)
        tree_z = np.random.uniform(135.0, 150.0, n_trees)
        tree_rel_z = tree_z - 134.0
        
        all_pts = np.vstack([
            np.column_stack([tower_top_x, tower_top_y, tower_top_z]),
            np.column_stack([tower_body_x, tower_body_y, tower_body_z]),
            np.column_stack([wire_x, wire_y, wire_z]),
            np.column_stack([tree_x, tree_y, tree_z])
        ])
        
        all_rel_z = np.concatenate([tower_top_rel_z, tower_body_rel_z, wire_rel_z, tree_rel_z])
        
        candidates = cluster_tower_voxels(
            off_ground_pts=all_pts,
            rel_z=all_rel_z,
            t_grid_size=2.0,
            min_tower_rel_z=22.0,
            min_pts_count=300,
            continuity_ratio=0.50,
            nms_radius=14.0
        )
        
        self.assertGreaterEqual(len(candidates), 1)
        best_cand = max(candidates, key=lambda c: c.get('score', 0))
        
        # 验证塔心中心锁定在真实塔心 (739386.0, 3380336.0) 1.5m 误差半径内
        dist_to_true_tower = np.hypot(best_cand['cx'] - 739386.0, best_cand['cy'] - 3380336.0)
        self.assertLess(dist_to_true_tower, 1.5, f"塔心中心偏移过大: {dist_to_true_tower:.2f}m")
        
        # 验证没有漂移到坡下导线 (739401.0, 3380343.0) 处
        dist_to_false_wire = np.hypot(best_cand['cx'] - 739401.0, best_cand['cy'] - 3380343.0)
        self.assertGreater(dist_to_false_wire, 10.0, "塔心被错误吸附到坡下导线处！")

if __name__ == '__main__':
    unittest.main()
