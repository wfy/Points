# 02: Voltage-Adaptive Line-Direction Thickness for UpperTowerBox

**What to build:**
Upgrade `UpperTowerBox` in `modules/tower_detector.py` by removing the historical hardcoded line-direction thickness limit (`min(..., 4.8)`). Implement the voltage-adaptive insulator allowance formula:
$$W_{\text{line\_half}} = \frac{W_{\text{trunk\_line}}}{2} + \min(0.18 \times W_{\text{arm\_half}},\ 0.08 \times H)$$
Cleanly encapsulate insulators and hanging hardware for 110kV, 220kV, and 500kV towers while rigidly barring mid-span hanging conductors from entering the tower envelope.

**Blocked by:** 01 (Anchor WaistBoundaryElevation and Full Jumper Inclusion)

**Status:** ready-for-agent

- [x] Remove hardcoded `4.8m` line-direction thickness bound in `modules/tower_detector.py`.
- [x] Implement adaptive line-direction thickness scaling with crossarm span and tower height.
- [x] Add unit test verifying insulator string preservation across synthetic 110kV (1.5m), 220kV (2.5m), and 500kV (4.2m) test configurations.
- [x] Verify that mid-span conductors extending beyond the adaptive boundary are rigidly excluded.
