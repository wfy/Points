import unittest
import numpy as np
from modules.config import PipelineStage, PipelineConfig
from modules.models import PipelineResult, TowerEntity, WireCluster

class TestPipelineResultAndStage(unittest.TestCase):
    def test_pipeline_stage_enum(self):
        self.assertEqual(PipelineStage.GROUND.value, "ground")
        self.assertEqual(PipelineStage.TOWER.value, "tower")
        self.assertEqual(PipelineStage.WIRE.value, "wire")
        self.assertEqual(PipelineStage.TOPOLOGY.value, "topology")
        self.assertEqual(PipelineStage.EXPORT.value, "export")
        
        # Test from_string helper
        self.assertEqual(PipelineStage.from_string("tower"), PipelineStage.TOWER)
        self.assertEqual(PipelineStage.from_string("TOWER"), PipelineStage.TOWER)

    def test_pipeline_config_stage_controls(self):
        cfg = PipelineConfig()
        self.assertIsNone(cfg.pipeline.stop_after)
        
        cfg.pipeline.stop_after = PipelineStage.TOWER
        self.assertEqual(cfg.pipeline.stop_after, PipelineStage.TOWER)

    def test_pipeline_result_encapsulation(self):
        n_pts = 100
        classification = np.full(n_pts, 1, dtype=np.uint8) # 1: unclassified
        classification[0:20] = 2   # Ground
        classification[20:30] = 15 # Tower
        classification[30:50] = 14 # Wire

        towers = [TowerEntity(cx=0.0, cy=0.0, max_z=35.0)]
        wires = [WireCluster(members=[0, 1], center=np.zeros(3), dir=np.array([1, 0, 0]), span=10.0, linearity=0.9, min_var=0.05)]
        timings = {"ground": 0.05, "tower": 0.10}

        result = PipelineResult(
            num_points=n_pts,
            classification=classification,
            towers=towers,
            wires=wires,
            stage_timings=timings
        )

        self.assertEqual(len(result.ground_indices), 20)
        self.assertEqual(len(result.tower_indices), 10)
        self.assertEqual(len(result.wire_indices), 20)
        self.assertEqual(len(result.towers), 1)
        self.assertEqual(len(result.wires), 1)
        self.assertAlmostEqual(result.total_time, 0.15)
        
        summary = result.summary()
        self.assertEqual(summary['ground_count'], 20)
        self.assertEqual(summary['tower_count'], 10)
        self.assertEqual(summary['wire_count'], 20)

if __name__ == '__main__':
    unittest.main()
