# 02: FrustumFirstAnchor 四棱台自底向上定姿与全塔单一刚体朝向锁定

**What to build:** 在地面上方纯净四棱台区间（$z \in [0.15H, 0.40H]$）自底向上提取 4 根刚性主腿角钢，以 4 根主腿外围矩形对称中心作为铁塔唯一绝对中轴 $(c_x, c_y)$，彻底消除不对称横担对塔心的拉偏；以 4 根主腿外立面法向提取正交基准主轴 $(\vec{v}_1, \vec{v}_2)$，消除转角耐张塔（5-6#）因粗先验误锁定到 45° 对角线引发的菱形错位；上下两部分严格共享同一正交主轴，保证 UpperTowerBox 与 LowerTowerFrustum 朝向浑然一体。

**Blocked by:** 01: PhysicalProportionGuard 物理几何比例熔断与横担阶跃检测

**Status:** closed

- [x] 实现 `extract_frustum_first_anchor` 函数，自底向上提取 4 根主腿中心与立面
- [x] 锁定全塔统一物理轴心 $(c_x, c_y)$，杜绝高位非对称拉偏
- [x] 锁定全塔正交外立面朝向 $(\vec{v}_1, \vec{v}_2)$，杜绝 45° 对角线倒挂（5-6# Tower 2 朝向成功锁定至 -90.4°/179.6°）
- [x] UpperTowerBox 与 LowerTowerFrustum 严格共享此基准朝向
- [x] 单元测试验证：验证非对称横担下中心稳定性与四立面正交性
