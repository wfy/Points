# 01: PhysicalProportionGuard 物理几何比例熔断与横担阶跃检测

**What to build:** 在横担切片检测中植入刚性物理下限 $z \ge \max(0.30 \times H, 10.0\text{m})$，并增加相对下层塔腰立柱的展宽阶跃突变检测（$\Delta w \ge 2.5\text{m}$ 且 $w_{\text{span}} / w_{\text{trunk}} \ge 1.6$），杜绝 56m+ 大塔平滑放坡角钢误判为横担，解决 68-69# 分割线下坠。

**Blocked by:** None (can start immediately)

**Status:** closed

- [x] 在切片扫描中植入刚性搜索底线 $z_{\text{min}} = \max(0.30 \times H, 8.0\text{m})$
- [x] 增加横担外展相对下层立柱的阶跃突变判定（$\Delta w \ge 2.0\text{m}$ 且 $(w_1-w_2) \ge 2.0\text{m}$）
- [x] 排除正方形且无展宽的平滑放坡切片
- [x] 单元测试验证：模拟大塔放坡角钢不被误判为横担，68-69# Tower 1 分割线稳定在 30.30m (58.3%)
