import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple

@dataclass
class CatenaryModel:
    """
    三维空间电力线物理悬链线模型
    物理方程 (铅垂投影剖面):
        z(s) = a * cosh((s - s0) / a) + z_offset
    """
    origin: np.ndarray       # 3D 锚点 (x0, y0, z0)
    direction_2d: np.ndarray # 水平方向单位矢量 (ux, uy)
    a: float                 # 悬链线水平张力/线重比参数 (a = H / q)
    s0: float                # 悬链线弧垂最低点对应的水平投影距离
    z_offset: float          # 垂向偏置常数
    residual_rmse: float     # 拟合均方根误差 (m)

    def project_to_s(self, pts_3d: np.ndarray) -> np.ndarray:
        """计算 3D 点在水平走向上的投影标量距离 s"""
        diff_2d = pts_3d[:, :2] - self.origin[:2]
        return diff_2d @ self.direction_2d

    def predict_z(self, s: np.ndarray) -> np.ndarray:
        """基于悬链线方程预测水平距离 s 处的高程 z"""
        arg = np.clip((s - self.s0) / max(abs(self.a), 1e-3), -10.0, 10.0)
        return self.a * np.cosh(arg) + self.z_offset

    def predict_3d(self, s: np.ndarray) -> np.ndarray:
        """基于水平距离 s 预测空间 3D 点坐标 (x, y, z)"""
        s_arr = np.atleast_1d(s)
        pts_xy = self.origin[:2] + np.outer(s_arr, self.direction_2d)
        pts_z = self.predict_z(s_arr)[:, None]
        return np.hstack((pts_xy, pts_z))

    def get_tangent_3d(self, s: float) -> np.ndarray:
        """计算在水平距离 s 处的 3D 单位切线矢量 (dx, dy, dz)"""
        arg = np.clip((s - self.s0) / max(abs(self.a), 1e-3), -10.0, 10.0)
        dz_ds = np.sinh(arg)
        
        tangent = np.array([
            self.direction_2d[0],
            self.direction_2d[1],
            dz_ds
        ])
        norm = np.linalg.norm(tangent)
        return tangent / max(norm, 1e-6)

    def compute_sag(self, s_start: float, s_end: float) -> float:
        """计算在 [s_start, s_end] 跨度内的最大物理弧垂 (Sag)"""
        z_start = self.predict_z(np.array([s_start]))[0]
        z_end = self.predict_z(np.array([s_end]))[0]
        chord_mid_z = 0.5 * (z_start + z_end)
        
        s_mid = 0.5 * (s_start + s_end)
        wire_mid_z = self.predict_z(np.array([s_mid]))[0]
        return float(chord_mid_z - wire_mid_z)


def fit_catenary_3d(pts_3d: np.ndarray, 
                    min_points: int = 6,
                    max_rmse_thresh: float = 1.0) -> Optional[CatenaryModel]:
    """
    鲁棒拟合 3D 点云的悬链线模型 (内存安全与自适应降采样优化)
    """
    n_pts = len(pts_3d)
    if n_pts < min_points:
        return None
        
    # 若点数过大，进行均匀网格降采样以加速拟合并消除大矩阵内存分配
    if n_pts > 300:
        step = max(n_pts // 200, 1)
        sample_pts = pts_3d[::step]
    else:
        sample_pts = pts_3d
        
    origin = np.mean(sample_pts, axis=0)
    
    # 1. 2D 水平主轴方向估计 (PCA)
    cov_2d = np.cov((sample_pts[:, :2] - origin[:2]).T)
    evals, evecs = np.linalg.eigh(cov_2d)
    direction_2d = evecs[:, 1]
    norm_d = np.linalg.norm(direction_2d)
    if norm_d < 1e-5:
        return None
    direction_2d /= norm_d
    
    # 2. 投影至 (s, z) 铅垂剖面
    diff_2d = sample_pts[:, :2] - origin[:2]
    s = diff_2d @ direction_2d
    z = sample_pts[:, 2]
    
    span_s = np.ptp(s)
    if span_s < 2.0:
        return None
        
    # 3. 抛物线解析初值估计: z(s) = A * s^2 + B * s + C
    try:
        poly_coeffs = np.polyfit(s, z, deg=2)
        A, B, C = poly_coeffs[0], poly_coeffs[1], poly_coeffs[2]
    except Exception:
        return None
        
    if A <= 1e-7:
        A = 1e-4
        
    a_init = 1.0 / (2.0 * max(A, 1e-6))
    a_init = float(np.clip(a_init, 20.0, 50000.0))
    s0_init = float(-B / (2.0 * max(A, 1e-6)))
    z_offset_init = float(C - (s0_init ** 2) / (2.0 * a_init) - a_init)
    
    # 4. 快速非线性 Gauss-Newton / Huber 鲁棒加权精修 (O(N) 内存)
    a, s0, z_offset = a_init, s0_init, z_offset_init
    
    for _ in range(5):
        arg = np.clip((s - s0) / a, -8.0, 8.0)
        cosh_val = np.cosh(arg)
        sinh_val = np.sinh(arg)
        
        z_pred = a * cosh_val + z_offset
        residuals = z - z_pred
        
        abs_res = np.abs(residuals)
        weights = np.ones_like(residuals)
        huber_k = 0.5
        outliers = abs_res > huber_k
        weights[outliers] = huber_k / abs_res[outliers]
        
        # 雅可比矩阵 J
        J_a = cosh_val - arg * sinh_val
        J_s0 = -sinh_val
        J_z0 = np.ones_like(s)
        
        J = np.column_stack((J_a, J_s0, J_z0))
        # 向量化加权: JT_W = J.T * weights (无需构造庞大的 N x N 对角矩阵)
        JT_W = J.T * weights[None, :]
        
        try:
            H = JT_W @ J + 1e-4 * np.eye(3)
            g = JT_W @ residuals
            delta = np.linalg.solve(H, g)
            
            a += float(np.clip(delta[0], -500.0, 500.0))
            a = float(np.clip(a, 20.0, 50000.0))
            s0 += float(np.clip(delta[1], -10.0, 10.0))
            z_offset += float(np.clip(delta[2], -5.0, 5.0))
            
            if np.linalg.norm(delta) < 1e-3:
                break
        except Exception:
            break
            
    final_pred_z = a * np.cosh(np.clip((s - s0) / a, -8.0, 8.0)) + z_offset
    rmse = float(np.sqrt(np.mean((z - final_pred_z) ** 2)))
    
    if rmse > max_rmse_thresh:
        return None
        
    return CatenaryModel(
        origin=origin,
        direction_2d=direction_2d,
        a=a,
        s0=s0,
        z_offset=z_offset,
        residual_rmse=rmse
    )
