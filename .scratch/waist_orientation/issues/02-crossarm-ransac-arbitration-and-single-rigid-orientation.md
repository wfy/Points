# 02: 高空横担角钢 RANSAC 点积仲裁与全塔单一刚体朝向锁定 (Crossarm RANSAC Arbitration & Rigid Orientation)

**What to build:** 在高空金属角钢层拟合 2D RANSAC 刚性横担直线向量 $\vec{v}_{\text{arm}}$，与塔腰正交基底 $(\vec{u}_a, \vec{u}_b)$ 计算绝对点积值仲裁出横担主轴 $\vec{v}_1$ 与走廊厚度轴 $\vec{v}_2$；将 $(\vec{v}_1, \vec{v}_2)$ 注入全塔，锁定 UpperTowerBox 与 LowerTowerFrustum 为单一刚体朝向；杜绝高空导线展宽引发的 90° 倒挂，彻底消除 5-6# 俯视剪刀差。

**Blocked by:** 01: 塔腰纯净段正方形截面正交定姿 (Waist MinArea-OBB)

**Status:** closed

- [x] 在 `detect_towers` 中调用 `extract_waist_orthogonal_axes` 获取纯净塔腰截面的正交基底对 $(\vec{u}_a, \vec{u}_b)$
- [x] 实现基于高空横担角钢 2D RANSAC 向量 $\vec{v}_{\text{arm}}$ 的绝对点积仲裁逻辑，确定横担主轴 $\vec{v}_1$ 与顺线走廊轴 $\vec{v}_2$
- [x] 设置防退化降级保护（优先 RANSAC 直线，次选跨档走向法向 $\vec{u}_{\text{arm\_prior}}$，兜底为高空散点惯性轴），100% 杜绝 90° 倒挂
- [x] 将仲裁后的 $(\vec{v}_1, \vec{v}_2)$ 赋给整个杆塔模型（UpperTowerBox 与 LowerTowerFrustum 同轴共体）
- [x] 保持 Q1（紧致长方体 OBB：`half_line_t <= 4.2m`）与 Q3（30% 物理比例熔断：`PhysicalProportionGuard`）完全生效
- [x] 编写针对点积仲裁、抗 90° 倒挂与刚体轴向一致性的单元测试
