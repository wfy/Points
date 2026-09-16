import unittest
import numpy as np
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from modules.tower_detector import detect_towers
from modules.config import PipelineConfig

class TestTreeFalsePositiveRejection(unittest.TestCase):
    def test_reject_tall_dense_hillside_tree(self):
        """
        测试高大陡坡茂密林木防误识能力 (复现图 2 场景: 高度 26m, 点数 4500 点, 垂向体素全连通)
        在无横担且树冠为实心体时，必须被坚决淘汰，绝不能被误判为杆塔 (Class 15)
        """
        np.random.seed(42)
        n_pts = 4500
        max_z = 32.0
        
        # 模拟高大树木：树干在中心 (r <= 0.85m)，树冠呈半球形实心漫延
        z = np.random.uniform(0.5, max_z, n_pts)
        r = np.random.uniform(0.0, 4.0, n_pts) * (z / max_z)**0.5
        theta = np.random.uniform(0.0, 2 * np.pi, n_pts)
        
        tree_x = 500.0 + r * np.cos(theta)
        tree_y = 500.0 + r * np.sin(theta)
        pts = np.column_stack([tree_x, tree_y, z])
        
        rel_z = z.copy()
        off_ground_idx = np.arange(len(pts))
        
        cfg = PipelineConfig()
        is_tower, is_arm, is_near_arm, tower_entities = detect_towers(
            off_ground_pts=pts,
            rel_z=rel_z,
            off_ground_idx=off_ground_idx,
            config=cfg
        )
        
        # 严正验证：树木不能被识别为铁塔实体，杆塔标记点数应为 0
        self.assertEqual(len(tower_entities), 0, "高大树木被错误识别为杆塔实体！")
        self.assertEqual(np.sum(is_tower), 0, "高大树木点云被错误染为 Class 15 杆塔！")

    def test_accept_real_transmission_tower(self):
        """
        测试真实高压空腔角钢输电铁塔 (高度 35m, 4 根立柱中空四棱台 + 双层横担)
        必须被 100% 准确识别为杆塔
        """
        np.random.seed(42)
        h = 35.0
        n_trunk = 3500
        zs = np.random.uniform(1.0, h, n_trunk)
        base_w, top_w = 5.0, 2.0
        w = base_w - (base_w - top_w) * (zs / h)
        
        # 四根角钢立柱 (中间中空)
        legs = np.random.choice(4, n_trunk)
        signs = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        xs = np.array([signs[l][0] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.05, n_trunk)
        ys = np.array([signs[l][1] * w[i]/2 for i, l in enumerate(legs)]) + np.random.normal(0, 0.05, n_trunk)
        
        # 横担构件 (双层对称横担, y 向外展 8m)
        n_arm = 800
        arm_z1 = 26.0 + np.random.uniform(-0.3, 0.3, n_arm // 2)
        arm_y1 = np.random.uniform(-8.0, 8.0, n_arm // 2)
        arm_x1 = np.random.uniform(-0.6, 0.6, n_arm // 2)
        
        arm_z2 = 30.0 + np.random.uniform(-0.3, 0.3, n_arm // 2)
        arm_y2 = np.random.uniform(-6.0, 6.0, n_arm // 2)
        arm_x2 = np.random.uniform(-0.5, 0.5, n_arm // 2)
        
        all_x = 200.0 + np.concatenate([xs, arm_x1, arm_x2])
        all_y = 200.0 + np.concatenate([ys, arm_y1, arm_y2])
        all_z = np.concatenate([zs, arm_z1, arm_z2])
        
        pts = np.column_stack([all_x, all_y, all_z])
        rel_z = all_z.copy()
        off_ground_idx = np.arange(len(pts))
        
        cfg = PipelineConfig()
        is_tower, is_arm, is_near_arm, tower_entities = detect_towers(
            off_ground_pts=pts,
            rel_z=rel_z,
            off_ground_idx=off_ground_idx,
            config=cfg
        )
        
        self.assertEqual(len(tower_entities), 1, "真铁塔未能成功识别！")
        self.assertGreater(np.sum(is_tower), 1000, "真铁塔点云识别量不足！")

    def test_reject_wire_over_tree_combo_and_preserve_nearby_tower(self):
        """
        测试真塔身侧 18m 处陡坡大树+上方贯通导线场景 (精确复现图 2 问题: 蓝框真塔被 NMS 吞噬，红框假塔误识别)
        期望行为：
          1. 蓝框真铁塔 (x=0, y=0) 必须成功检出，绝不能被身侧假目标通过 NMS 抹杀；
          2. 红框假目标 (x=18, y=0) 的大树+导线组合必须被严格过滤，严禁识别为杆塔！
        """
        np.random.seed(42)
        
        # 1. 真实铁塔 (x=0, y=0, 净高 30m)
        h1 = 30.0
        n_trunk = 3000
        zs1 = np.random.uniform(1.0, h1, n_trunk)
        w1 = 4.0 - 2.0 * (zs1 / h1)
        legs1 = np.random.choice(4, n_trunk)
        signs1 = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        xs1 = np.array([signs1[l][0] * w1[i]/2 for i, l in enumerate(legs1)]) + np.random.normal(0, 0.04, n_trunk)
        ys1 = np.array([signs1[l][1] * w1[i]/2 for i, l in enumerate(legs1)]) + np.random.normal(0, 0.04, n_trunk)
        
        # 真塔横担 (沿 Y 轴展开)
        n_arm = 600
        arm_z = 24.0 + np.random.uniform(-0.3, 0.3, n_arm)
        arm_y = np.random.uniform(-7.0, 7.0, n_arm)
        arm_x = np.random.uniform(-0.5, 0.5, n_arm)
        
        t1_x = np.concatenate([xs1, arm_x])
        t1_y = np.concatenate([ys1, arm_y])
        t1_z = np.concatenate([zs1, arm_z])
        
        # 2. 距离真塔 18m 处的斜坡实心大树 (x=18, y=0, 树高 0.5~16m)
        n_tree = 2500
        tree_z = np.random.uniform(0.5, 16.0, n_tree)
        tree_r = np.random.uniform(0.0, 3.5, n_tree)
        tree_theta = np.random.uniform(0.0, 2 * np.pi, n_tree)
        tree_x = 18.0 + tree_r * np.cos(tree_theta)
        tree_y = 0.0 + tree_r * np.sin(tree_theta)
        
        # 3. 贯通导线 (从 x=-15 延伸到 x=40, 穿过真塔与假塔上方，高度 24m 与 28m)
        n_wire_pts = 1200
        wire_xs = np.linspace(-15.0, 40.0, n_wire_pts // 2)
        # 上层导线 (z=28m) 与下层导线 (z=24m)
        w_x = np.concatenate([wire_xs, wire_xs])
        w_y = np.concatenate([np.full(len(wire_xs), 3.0), np.full(len(wire_xs), -3.0)])
        w_z = np.concatenate([np.full(len(wire_xs), 28.0), np.full(len(wire_xs), 24.0)])
        
        all_x = np.concatenate([t1_x, tree_x, w_x])
        all_y = np.concatenate([t1_y, tree_y, w_y])
        all_z = np.concatenate([t1_z, tree_z, w_z])
        
        pts = np.column_stack([all_x, all_y, all_z])
        rel_z = all_z.copy()
        off_ground_idx = np.arange(len(pts))
        
        cfg = PipelineConfig()
        is_tower, is_arm, is_near_arm, tower_entities = detect_towers(
            off_ground_pts=pts,
            rel_z=rel_z,
            off_ground_idx=off_ground_idx,
            config=cfg
        )
        
        print(f"Detected tower count: {len(tower_entities)}")
        for i, ent in enumerate(tower_entities):
            print(f"  Tower {i+1}: center=({ent.cx:.1f}, {ent.cy:.1f}), max_z={ent.max_z:.1f}")
            
        # 必须仅检出 1 座铁塔 (位于 x=0 处真塔)，绝不能检出位于 x=18 处的假塔
        self.assertEqual(len(tower_entities), 1, "未能准确识别单一真铁塔！")
        self.assertLess(abs(tower_entities[0].cx - 0.0), 3.0, "真铁塔中心定位偏离！")

if __name__ == '__main__':
    unittest.main()

