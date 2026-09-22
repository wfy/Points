# 04: Ground-Level Voxel Connectivity Gating and Full Regression Verification

**What to build:**
Integrate `UpperTowerBox` and `LowerTowerFrustum` geometric envelopes with downward 3D skeleton voxel connectivity growth (step 0.85m) seeded from pure waist columns. Completely eliminate hillside terrain and unattached tree clusters enclosed in the wide frustum base. Run the full unit test suite and execute benchmark verification across the 5 primary point cloud datasets (`68-69.las`, `18-19.las`, `126-127.las`, `0-1.las`, `53-54.las`) with zero regressions.

**Blocked by:** 02 (Voltage-Adaptive Line-Direction Thickness for UpperTowerBox), 03 (LowerTowerFrustum Pure Centroid Symmetry and Slope Envelope)

**Status:** ready-for-agent

- [x] Combine `UpperTowerBox` and `LowerTowerFrustum` with downward 3D voxel connectivity growth.
- [x] Ensure disconnected hillside vegetation inside the wide base is rejected.
- [x] Run full test suite via `python -m unittest discover -s tests` and verify 100% pass rate.
- [x] Execute pipeline benchmark runs against primary LAS point clouds and verify exit code 0 and metrics.
