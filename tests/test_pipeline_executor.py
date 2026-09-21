import os
import unittest
import tempfile
import numpy as np
import laspy

from modules.config import PipelineConfig, PipelineStage
from modules.models import PipelineResult
from modules.pipeline_executor import PipelineExecutor

class TestPipelineExecutor(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Create a compact synthetic point cloud (Ground + 1 Tower + 2 Wire lines)
        np.random.seed(42)
        # 1. Ground: 2000 points
        gx = np.random.uniform(-10, 50, 2000)
        gy = np.random.uniform(-10, 10, 2000)
        gz = np.random.uniform(0.0, 0.3, 2000)
        
        # 2. Tower: at (0, 0), h=35m, 1500 points
        tz = np.random.uniform(1.0, 35.0, 1200)
        tw = 4.0 - (4.0 - 1.5) * (tz / 35.0)
        tx = np.random.uniform(-tw/2, tw/2, 1200)
        ty = np.random.uniform(-tw/2, tw/2, 1200)
        # Arm: z=30m, y span [-6, 6]
        arm_z = np.random.uniform(29.8, 30.2, 400)
        arm_y = np.random.uniform(-6.0, 6.0, 400)
        arm_x = np.random.uniform(-0.5, 0.5, 400)
        
        # 3. Second Tower: at (40, 0), h=35m
        t2z = np.random.uniform(1.0, 35.0, 1200)
        t2w = 4.0 - (4.0 - 1.5) * (t2z / 35.0)
        t2x = 40.0 + np.random.uniform(-t2w/2, t2w/2, 1200)
        t2y = np.random.uniform(-t2w/2, t2w/2, 1200)
        arm2_z = np.random.uniform(29.8, 30.2, 400)
        arm2_y = np.random.uniform(-6.0, 6.0, 400)
        arm2_x = 40.0 + np.random.uniform(-0.5, 0.5, 400)
        
        # 4. Wires: 2 catenary lines between towers at y=-4 and y=4, z=30m
        xs = np.linspace(0.0, 40.0, 600)
        a = 300.0
        zs = a * np.cosh((xs - 20.0) / a) + (30.0 - a * np.cosh(20.0 / a))
        w1_x = xs + np.random.normal(0, 0.02, len(xs))
        w1_y = np.full(len(xs), -4.0) + np.random.normal(0, 0.02, len(xs))
        w1_z = zs + np.random.normal(0, 0.02, len(xs))
        
        w2_x = xs + np.random.normal(0, 0.02, len(xs))
        w2_y = np.full(len(xs), 4.0) + np.random.normal(0, 0.02, len(xs))
        w2_z = zs + np.random.normal(0, 0.02, len(xs))
        
        all_x = np.concatenate([gx, tx, arm_x, t2x, arm2_x, w1_x, w2_x])
        all_y = np.concatenate([gy, ty, arm_y, t2y, arm2_y, w1_y, w2_y])
        all_z = np.concatenate([gz, tz, arm_z, t2z, arm2_z, w1_z, w2_z])
        cls.synthetic_points = np.column_stack([all_x, all_y, all_z])

    def test_short_circuit_ground(self):
        cfg = PipelineConfig()
        cfg.pipeline.stop_after = PipelineStage.GROUND
        
        executor = PipelineExecutor(config=cfg)
        res = executor.run(self.synthetic_points)
        
        self.assertIsInstance(res, PipelineResult)
        self.assertTrue(len(res.ground_indices) > 0)
        self.assertEqual(len(res.tower_indices), 0)
        self.assertEqual(len(res.wire_indices), 0)
        self.assertIn("ground", res.stage_timings)
        self.assertNotIn("tower", res.stage_timings)
        self.assertNotIn("wire", res.stage_timings)

    def test_short_circuit_tower(self):
        cfg = PipelineConfig()
        cfg.pipeline.stop_after = PipelineStage.TOWER
        
        executor = PipelineExecutor(config=cfg)
        res = executor.run(self.synthetic_points)
        
        self.assertIsInstance(res, PipelineResult)
        self.assertTrue(len(res.ground_indices) > 0)
        self.assertTrue(len(res.tower_indices) > 0)
        self.assertEqual(len(res.wire_indices), 0)
        self.assertIn("ground", res.stage_timings)
        self.assertIn("tower", res.stage_timings)
        self.assertNotIn("wire", res.stage_timings)

    def test_full_pipeline_execution(self):
        cfg = PipelineConfig()
        cfg.pipeline.stop_after = None # Run full pipeline
        
        executor = PipelineExecutor(config=cfg)
        res = executor.run(self.synthetic_points)
        
        self.assertIsInstance(res, PipelineResult)
        self.assertTrue(len(res.ground_indices) > 0)
        self.assertTrue(len(res.tower_indices) > 0)
        self.assertTrue(len(res.wire_indices) > 0)
        
        # Test mutual exclusivity of point indices
        g_set = set(res.ground_indices)
        t_set = set(res.tower_indices)
        w_set = set(res.wire_indices)
        self.assertEqual(len(g_set.intersection(t_set)), 0)
        self.assertEqual(len(t_set.intersection(w_set)), 0)
        self.assertEqual(len(g_set.intersection(w_set)), 0)

    def test_short_circuit_wire(self):
        cfg = PipelineConfig()
        cfg.pipeline.stop_after = PipelineStage.WIRE
        
        executor = PipelineExecutor(config=cfg)
        res = executor.run(self.synthetic_points, verbose=False)
        
        self.assertIsInstance(res, PipelineResult)
        self.assertTrue(len(res.ground_indices) > 0)
        self.assertTrue(len(res.tower_indices) > 0)
        self.assertTrue(len(res.wire_indices) > 0)
        self.assertIn("ground", res.stage_timings)
        self.assertIn("tower", res.stage_timings)
        self.assertIn("wire", res.stage_timings)
        self.assertNotIn("topology", res.stage_timings)

    def test_verbose_logging(self):
        import io
        from contextlib import redirect_stdout
        
        cfg = PipelineConfig()
        cfg.pipeline.stop_after = PipelineStage.GROUND
        
        executor = PipelineExecutor(config=cfg)
        
        # Test verbose=True
        buf_verbose = io.StringIO()
        with redirect_stdout(buf_verbose):
            executor.run(self.synthetic_points, verbose=True)
        out_verbose = buf_verbose.getvalue()
        self.assertIn("-> 1/4 执行地形自适应局部滤波剥离地面...", out_verbose)
        self.assertIn("阶段一完成", out_verbose)
        
        # Test verbose=False
        buf_silent = io.StringIO()
        with redirect_stdout(buf_silent):
            executor.run(self.synthetic_points, verbose=False)
        out_silent = buf_silent.getvalue()
        self.assertEqual(out_silent.strip(), "")

    def test_run_file_and_export(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            in_las = os.path.join(tmp_dir, "input.las")
            out_las = os.path.join(tmp_dir, "output_sign.las")
            
            header = laspy.LasHeader(point_format=3, version="1.2")
            header.offsets = [0.0, 0.0, 0.0]
            header.scales = [0.001, 0.001, 0.001]
            las_data = laspy.LasData(header)
            las_data.x = self.synthetic_points[:, 0]
            las_data.y = self.synthetic_points[:, 1]
            las_data.z = self.synthetic_points[:, 2]
            las_data.classification = np.full(len(self.synthetic_points), 1, dtype=np.uint8)
            las_data.write(in_las)
            
            cfg = PipelineConfig()
            cfg.export.open_qtmodeler = False
            cfg.export.force_kill_viewer = False
            
            executor = PipelineExecutor(config=cfg)
            res = executor.run_file(in_las, out_las)
            
            self.assertTrue(os.path.exists(out_las))
            self.assertIsInstance(res, PipelineResult)
            self.assertTrue(len(res.ground_indices) > 0)

if __name__ == '__main__':
    unittest.main()
