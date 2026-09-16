import unittest
import numpy as np
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from modules.tower_detector import fit_arm_ransac_2d

class TestTowerOrientation(unittest.TestCase):
    def test_symmetric_crossarm(self):
        """测试标准对称横担方向拟合精度"""
        theta_true = np.radians(35.0)
        v_true = np.array([np.cos(theta_true), np.sin(theta_true)])
        
        # 生成双翼对称横担点云 (span = [-8m, 8m])
        s = np.linspace(-8.0, 8.0, 100)
        pts = np.zeros((len(s), 2))
        pts[:, 0] = s * v_true[0] + np.random.normal(0, 0.05, len(s))
        pts[:, 1] = s * v_true[1] + np.random.normal(0, 0.05, len(s))
        
        v_fit, inliers = fit_arm_ransac_2d(pts, inlier_thresh=0.4)
        self.assertIsNotNone(v_fit)
        self.assertGreater(inliers, 50)
        
        # 验证拟合方向与真实方向同向或反向的夹角极小 (< 1.5 度)
        cos_sim = abs(float(v_fit @ v_true))
        angle_err = np.degrees(np.arccos(np.clip(cos_sim, -1.0, 1.0)))
        self.assertLess(angle_err, 1.5)

    def test_crossarm_with_heavy_side_wire(self):
        """测试横担一侧存在高密度顺线导线/跳线杂波时的抗偏转鲁棒性 (如图 1 耐张塔场景)"""
        theta_true = np.radians(70.0) # 真实横担方向 70 度
        v_true = np.array([np.cos(theta_true), np.sin(theta_true)])
        n_true = np.array([-v_true[1], v_true[0]]) # 顺线导线方向 160 度
        
        # 1. 真实横担点：从 -7m 到 +7m，50 个点
        s_arm = np.linspace(-7.0, 7.0, 50)
        arm_pts = np.column_stack([
            s_arm * v_true[0] + np.random.normal(0, 0.04, len(s_arm)),
            s_arm * v_true[1] + np.random.normal(0, 0.04, len(s_arm))
        ])
        
        # 2. 模拟耐张绝缘子串/单侧导线：位于横担右侧端头 (s = 5m 处)，沿顺线方向延伸，且点密度高达 120 点！
        # 若采用简单 RANSAC，极易被导线的 120 点或者横担端头与导线的连线绑架发生 15~20 度偏转
        s_wire = np.linspace(-6.0, 6.0, 120)
        wire_origin = 5.0 * v_true
        wire_pts = np.column_stack([
            wire_origin[0] + s_wire * n_true[0] + np.random.normal(0, 0.03, len(s_wire)),
            wire_origin[1] + s_wire * n_true[1] + np.random.normal(0, 0.03, len(s_wire))
        ])
        
        all_pts = np.vstack([arm_pts, wire_pts])
        
        v_fit, inliers = fit_arm_ransac_2d(all_pts, inlier_thresh=0.5)
        self.assertIsNotNone(v_fit)
        
        cos_sim = abs(float(v_fit @ v_true))
        angle_err = np.degrees(np.arccos(np.clip(cos_sim, -1.0, 1.0)))
        print(f"Angle error with heavy side wire: {angle_err:.2f} deg")
        self.assertLess(angle_err, 2.5)

    def test_crossarm_with_diagonal_jumper_loop(self):
        """测试耐张塔大曲率斜向引流跳线弧段干扰下的方向稳定性"""
        theta_true = np.radians(0.0) # 真实横担沿 X 轴 (0 度)
        v_true = np.array([1.0, 0.0])
        
        # 真实横担点
        s_arm = np.linspace(-6.0, 6.0, 60)
        arm_pts = np.column_stack([s_arm, np.random.normal(0, 0.04, len(s_arm))])
        
        # 斜向跳线环 (从横担一侧 (-4, 0) 绕向下方 (-2, -3))
        t = np.linspace(0, np.pi, 40)
        jumper_pts = np.column_stack([-3.0 + 1.5 * np.cos(t), -2.0 + 1.5 * np.sin(t)])
        
        all_pts = np.vstack([arm_pts, jumper_pts])
        v_fit, inliers = fit_arm_ransac_2d(all_pts, inlier_thresh=0.5)
        self.assertIsNotNone(v_fit)
        
        cos_sim = abs(float(v_fit @ v_true))
        angle_err = np.degrees(np.arccos(np.clip(cos_sim, -1.0, 1.0)))
        self.assertLess(angle_err, 2.0)

if __name__ == '__main__':
    unittest.main()
