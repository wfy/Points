import unittest
import numpy as np
from modules.config import DEFAULT_CONFIG
from modules.tower_detector import detect_towers

class TestLowerTowerCapture(unittest.TestCase):
    def test_disconnected_vegetation_rejection_by_3d_connectivity(self):
        """
        验证下塔身捕获时：
        1. 塔身桁架角钢从腰部连续延伸至地面，100% 被捕获为杆塔；
        2. 虽在四棱台放坡方盒允许范围内、但与塔身存在空间空气间隙的独立林木/杂物，被 3D 体素连通生长成功阻断并滤除。
        """
        np.random.seed(42)
        cx, cy = 100.0, 100.0
        max_z = 35.0
        
        # 1. 构造真实铁塔点云
        # 塔头与横担 (Z: 25 ~ 35m)
        n_arm = 2000
        arm_x = np.random.uniform(cx - 7.0, cx + 7.0, n_arm)
        arm_y = np.random.uniform(cy - 1.2, cy + 1.2, n_arm)
        arm_z = np.random.uniform(25.0, 35.0, n_arm)
        
        # 塔腰 (Z: 18 ~ 25m, 半宽 ~2.0m)
        n_waist = 2000
        waist_x = np.random.uniform(cx - 2.0, cx + 2.0, n_waist)
        waist_y = np.random.uniform(cy - 2.0, cy + 2.0, n_waist)
        waist_z = np.random.uniform(18.0, 25.0, n_waist)
        
        # 连续下塔身立柱与斜撑 (Z: 0 ~ 18m, 线性放坡至半宽 ~2.5m)
        # 密集连续分布，确保体素 0.45m/0.85m 步长完全连通
        n_legs = 4000
        leg_z = np.random.uniform(0.5, 18.0, n_legs)
        slope = (3.0 - 2.0) / 18.0
        allowed_w = 2.0 + (18.0 - leg_z) * slope
        leg_x = cx + (np.random.uniform(-1.0, 1.0, n_legs) * allowed_w * 0.8)
        leg_y = cy + (np.random.uniform(-1.0, 1.0, n_legs) * allowed_w * 0.8)
        
        # 2. 构造四棱台方盒边缘的独立悬空植被丛 (Z: 3.0 ~ 6.5m, 与塔身间距 > 1.2m 空气隙)
        # 位于方盒边缘允许范围内 (w_allowed ~4.5m, veg_x 在 3.8 ~ 4.2m)
        n_veg = 500
        veg_x = np.random.uniform(cx + 3.8, cx + 4.2, n_veg)
        veg_y = np.random.uniform(cy + 0.0, cy + 1.0, n_veg)
        veg_z = np.random.uniform(3.0, 6.5, n_veg)
        
        tower_pts = np.vstack([
            np.column_stack([arm_x, arm_y, arm_z]),
            np.column_stack([waist_x, waist_y, waist_z]),
            np.column_stack([leg_x, leg_y, leg_z]),
        ])
        
        veg_pts = np.column_stack([veg_x, veg_y, veg_z])
        
        all_pts = np.vstack([tower_pts, veg_pts])
        rel_z = all_pts[:, 2].copy()
        off_ground_idx = np.arange(len(all_pts))
        
        cfg = DEFAULT_CONFIG
        is_tower, is_tower_arm, is_near_tower, entities = detect_towers(
            off_ground_pts=all_pts,
            rel_z=rel_z,
            off_ground_idx=off_ground_idx,
            config=cfg
        )
        
        self.assertGreaterEqual(len(entities), 1, "未检测到铁塔实体")
        
        # 验证塔身真点绝大部分被保留 (>= 95%)
        tower_mask = is_tower[:len(tower_pts)]
        tower_recall = np.sum(tower_mask) / len(tower_pts)
        self.assertGreater(tower_recall, 0.95, f"真塔点召回率过低: {tower_recall:.2%}")
        
        # 验证不连通的边缘植被丛被 3D 连通生长彻底剔除 (误识率 <= 1%)
        veg_mask = is_tower[len(tower_pts):]
        veg_false_positive_rate = np.sum(veg_mask) / len(veg_pts)
        self.assertLess(veg_false_positive_rate, 0.01, f"边缘不连通植被未被剔除，误报率: {veg_false_positive_rate:.2%}")

    def test_waist_trunk_measurement_isolation(self):
        """
        验证横担下腰过渡段的立柱半宽测量：
        即使上方横担展开达到 12m，测得的立柱腰宽依然严格收敛在立柱真实物理宽度 (~2.0m)，
        绝不被横担大宽度外展所污染。
        """
        np.random.seed(42)
        cx, cy = 200.0, 200.0
        max_z = 32.0
        
        # 横担很宽 (半宽 10m)
        n_arm = 1500
        arm_x = np.random.uniform(cx - 10.0, cx + 10.0, n_arm)
        arm_y = np.random.uniform(cy - 1.0, cy + 1.0, n_arm)
        arm_z = np.random.uniform(22.0, 32.0, n_arm)
        
        # 纯净立柱段 (半宽 1.8m)
        n_body = 3000
        body_x = np.random.uniform(cx - 1.8, cx + 1.8, n_body)
        body_y = np.random.uniform(cy - 1.8, cy + 1.8, n_body)
        body_z = np.random.uniform(0.5, 22.0, n_body)
        
        all_pts = np.vstack([
            np.column_stack([arm_x, arm_y, arm_z]),
            np.column_stack([body_x, body_y, body_z])
        ])
        rel_z = all_pts[:, 2].copy()
        off_ground_idx = np.arange(len(all_pts))
        
        is_tower, is_tower_arm, is_near_tower, entities = detect_towers(
            off_ground_pts=all_pts,
            rel_z=rel_z,
            off_ground_idx=off_ground_idx,
            config=DEFAULT_CONFIG
        )
        
        self.assertGreaterEqual(len(entities), 1)
        ent = entities[0]
        # 验证腰宽紧收在 2.5m 内，绝不膨胀至横担宽度 (5m~10m)
        self.assertLess(ent.w_trunk0, 2.5, f"塔身纯立柱腰宽膨胀: {ent.w_trunk0:.2f}m")

if __name__ == '__main__':
    unittest.main()
