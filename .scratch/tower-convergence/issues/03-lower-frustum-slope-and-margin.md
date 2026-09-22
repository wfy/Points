# 03: LowerTowerFrustum Pure Centroid Symmetry and Slope Envelope

**What to build:**
Ensure `LowerTowerFrustum` in `modules/tower_detector.py` is centered strictly on the multi-slice pure-trunk median centroid (D15). Linearly expand the frustum downwards from `WaistBoundaryElevation` to ground level using the height-scaled physical slope rate ($0.0811 \times k_{\text{height}}$), with independent margin parameters along crossarm and line directions. Ensure zero missed leg points under asymmetric conductor tension.

**Blocked by:** 01 (Anchor WaistBoundaryElevation and Full Jumper Inclusion)

**Status:** ready-for-agent

- [x] Align `LowerTowerFrustum` horizontal center with D15 multi-slice pure-trunk median centroid.
- [x] Enforce linear slope expansion $W_{\text{allowed}} = W_{\text{trunk}} + \Delta z \times \text{slope} + \text{margin}$ symmetrically.
- [x] Maintain independent installation margin parameters for crossarm ($v_1$) and line ($v_2$) directions.
- [x] Verify zero missed tower leg steel points under single-sided asymmetric wire pull in regression suite.
