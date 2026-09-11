import unittest
import numpy as np
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from modules.catenary import fit_catenary_3d, CatenaryModel
from modules.config import PipelineConfig

class TestCatenaryFitting(unittest.TestCase):
    def test_standard_catenary_fit(self):
        """测试常规档距 (200m) 悬链线拟合精度与 RMSE"""
        s = np.linspace(0.0, 200.0, 500)
        a_true = 800.0
        s0_true = 100.0
        z_offset_true = 25.0
        
        # z(s) = a * cosh((s - s0) / a) + z_offset
        z_true = a_true * np.cosh((s - s0_true) / a_true) + z_offset_true
        # 旋转到 3D 空间 (沿 45 度方向)
        ux, uy = np.cos(np.pi / 4), np.sin(np.pi / 4)
        origin = np.array([1000.0, 2000.0, 0.0])
        pts_3d = np.zeros((len(s), 3))
        pts_3d[:, 0] = origin[0] + s * ux
        pts_3d[:, 1] = origin[1] + s * uy
        pts_3d[:, 2] = z_true + np.random.normal(0.0, 0.02, len(s)) # 加入 2cm 测量噪声
        
        cat = fit_catenary_3d(pts_3d, min_points=10, max_rmse_thresh=0.5)
        self.assertIsNotNone(cat)
        self.assertLess(cat.residual_rmse, 0.10)
        self.assertAlmostEqual(cat.a, a_true, delta=50.0)
        
        # 验证弧垂计算
        sag = cat.compute_sag(0.0, 200.0)
        self.assertGreater(sag, 3.0)
        self.assertLess(sag, 15.0)

    def test_high_tension_flat_wire(self):
        """测试高张力平直段/短档距 (a >= 8000) 悬链线拟合"""
        s = np.linspace(0.0, 80.0, 200)
        a_true = 12000.0
        s0_true = 40.0
        z_offset_true = 30.0
        
        z_true = a_true * np.cosh((s - s0_true) / a_true) + z_offset_true
        pts_3d = np.zeros((len(s), 3))
        pts_3d[:, 0] = s
        pts_3d[:, 1] = 0.0
        pts_3d[:, 2] = z_true + np.random.normal(0.0, 0.01, len(s))
        
        cat = fit_catenary_3d(pts_3d, min_points=10, max_rmse_thresh=0.5)
        self.assertIsNotNone(cat)
        self.assertLess(cat.residual_rmse, 0.10)
        self.assertGreater(cat.a, 5000.0)

    def test_dual_bundle_dispersion(self):
        """测试双分裂导线 (间距 0.45m) 弥散点云拟合容差"""
        s = np.linspace(0.0, 150.0, 400)
        a_true = 1000.0
        s0_true = 75.0
        z_offset_true = 20.0
        z_base = a_true * np.cosh((s - s0_true) / a_true) + z_offset_true
        
        # 两个子导线，间距 0.45m
        sub1 = np.column_stack([s, np.full_like(s, -0.225), z_base])
        sub2 = np.column_stack([s, np.full_like(s, 0.225), z_base])
        bundle_pts = np.vstack([sub1, sub2])
        
        cat = fit_catenary_3d(bundle_pts, min_points=10, max_rmse_thresh=0.8)
        self.assertIsNotNone(cat)
        # RMSE 理论上在 0.2m 左右
        self.assertLess(cat.residual_rmse, 0.35)

if __name__ == '__main__':
    unittest.main()
