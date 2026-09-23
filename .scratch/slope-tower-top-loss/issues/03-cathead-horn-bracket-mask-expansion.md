# 03: Cathead horn bracket mask expansion

**What to build:**
在 `modules/tower_detector.py` 的包围盒构造逻辑中，放宽 `top_bracket_mask` 允许的最大横向半宽。支持猫头塔羊角（地线支架）外展角钢尖端的完整包含（`d_v1 <= max(w_waist * 1.6 + 2.5, half_arm_w)`），同时严格维持顺线方向走廊厚度约束（`d_v2 <= half_line_t`），杜绝跨中导线或出线悬挂导线渗透进杆塔类别。

**Blocked by:** 02: Slope adaptive top absolute Z gate

**Status:** closed

- [x] 优化 `top_bracket_mask` 横向半宽计算逻辑，完整捕获大外展角度的猫头塔羊角角钢点（如 `cloud4` 中外展达 5.71m 的尖端）
- [x] 保持顺线厚度严格截断，导线点（`d_v2 > half_line_t`）不发生回流或误分类
- [x] 针对真实 `cloud4.las` 运行完整 pipeline，验证原先漏识的 1,012 个塔顶角钢点被成功分类为 Class 15（杆塔）
- [x] 全套测试用例 `python -m unittest discover tests` 保持 Exit Code: 0，无任何回归
