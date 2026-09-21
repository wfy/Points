import unittest
import numpy as np

from modules.config import PipelineConfig
from modules.models import TowerEntity, WireExtractionResult
from modules.wire_extractor import WireExtractor

class TestWireExtractor(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        np.random.seed(42)
        # Ground
        gx = np.random.uniform(-10, 50, 1000)
        gy = np.random.uniform(-10, 10, 1000)
        gz = np.random.uniform(0.0, 0.2, 1000)

        # 2 Towers at (0, 0) and (40, 0), h=35m
        tz1 = np.random.uniform(1.0, 35.0, 600)
        tx1 = np.random.uniform(-1.5, 1.5, 600)
        ty1 = np.random.uniform(-1.5, 1.5, 600)

        tz2 = np.random.uniform(1.0, 35.0, 600)
        tx2 = 40.0 + np.random.uniform(-1.5, 1.5, 600)
        ty2 = np.random.uniform(-1.5, 1.5, 600)

        # 2 Catenary wires between towers at y=-4 and y=4, z=30m
        xs = np.linspace(0.0, 40.0, 400)
        a = 300.0
        zs = a * np.cosh((xs - 20.0) / a) + (30.0 - a * np.cosh(20.0 / a))
        w1_x = xs + np.random.normal(0, 0.02, len(xs))
        w1_y = np.full(len(xs), -4.0) + np.random.normal(0, 0.02, len(xs))
        w1_z = zs + np.random.normal(0, 0.02, len(xs))

        w2_x = xs + np.random.normal(0, 0.02, len(xs))
        w2_y = np.full(len(xs), 4.0) + np.random.normal(0, 0.02, len(xs))
        w2_z = zs + np.random.normal(0, 0.02, len(xs))

        all_x = np.concatenate([gx, tx1, tx2, w1_x, w2_x])
        all_y = np.concatenate([gy, ty1, ty2, w1_y, w2_y])
        all_z = np.concatenate([gz, tz1, tz2, w1_z, w2_z])
        cls.points = np.column_stack([all_x, all_y, all_z])

        # Off ground mask: everything except ground
        n_ground = len(gx)
        n_total = len(all_x)
        cls.off_ground_idx = np.arange(n_ground, n_total, dtype=int)
        cls.off_ground_pts = cls.points[cls.off_ground_idx]
        cls.rel_z = cls.off_ground_pts[:, 2]  # Ground z is ~0

        # Tower mask inside off_ground
        n_tower_pts = len(tx1) + len(tx2)
        cls.is_tower = np.zeros(len(cls.off_ground_idx), dtype=bool)
        cls.is_tower[:n_tower_pts] = True

        cls.towers = [
            TowerEntity(cx=0.0, cy=0.0, max_z=35.0, half_l1=6.0, half_l2=2.0, z_lowest_arm=28.0),
            TowerEntity(cx=40.0, cy=0.0, max_z=35.0, half_l1=6.0, half_l2=2.0, z_lowest_arm=28.0)
        ]

    def test_extract_returns_valid_wire_extraction_result(self):
        extractor = WireExtractor()
        cfg = PipelineConfig()
        cfg.powerline.enable_topdown_prior = True

        result = extractor.extract(
            points=self.points,
            off_ground_pts=self.off_ground_pts,
            off_ground_idx=self.off_ground_idx,
            rel_z=self.rel_z,
            is_tower=self.is_tower,
            tower_infos=self.towers,
            config=cfg
        )

        self.assertIsInstance(result, WireExtractionResult)
        self.assertTrue(len(result.cable_indices) > 0)
        self.assertTrue(len(result.wires) > 0)

        # Check line_id uniqueness and valid global_indices
        line_ids = [w.line_id for w in result.wires]
        self.assertEqual(len(line_ids), len(set(line_ids)), "Line IDs must be unique")
        for w in result.wires:
            self.assertTrue(w.line_id > 0)
            self.assertTrue(len(w.global_indices) > 0)

        # Check mutual exclusivity with tower points
        tower_global = self.off_ground_idx[self.is_tower]
        overlap = np.intersect1d(result.cable_indices, tower_global)
        self.assertEqual(len(overlap), 0, "Wire indices must not overlap with tower points")

    def test_fallback_without_towers(self):
        extractor = WireExtractor()
        cfg = PipelineConfig()

        # Run with no towers detected
        result = extractor.extract(
            points=self.points,
            off_ground_pts=self.off_ground_pts,
            off_ground_idx=self.off_ground_idx,
            rel_z=self.rel_z,
            is_tower=np.zeros(len(self.off_ground_idx), dtype=bool),
            tower_infos=[],
            config=cfg
        )

        self.assertIsInstance(result, WireExtractionResult)
        # Should fall back to seed tracker and still identify wires
        self.assertTrue(len(result.cable_indices) > 0)
        self.assertTrue(len(result.wires) > 0)

if __name__ == '__main__':
    unittest.main()
