# V11 CAD 交付摘要

- 坐标：X 左右，+Y 朝显示器，+Z 向上，单位 mm。
- 键盘托行程：+30 Y / -15 Z；妙控板托行程：-115 Y / -43 Z。
- 两托盘均为 279.7 × 115.7 的连续输入平面，妙控板靠右定位。
- 运动拓扑：双侧固定直线槽 + 双侧交叉长槽；每条受双向力的槽使用固定轮和偏心轮接触相对壁。
- 回位：两根带 Ø3 导杆和一体端部导向套的压缩弹簧，初始安装长度 80；OD 14、线径 1.10、自由长 111、刚度 0.200 N/mm。
- 限位：TPU 缓冲环承担端位，轴承不撞槽端。
- 右侧 0.6 轴向浮动吸收左右打印误差；左侧为定位基准。
- 三种滚轮轴改为可采购的 Ø4/M3 低头精密肩轴，肩长 20/16/12，头部 Ø6 × 2，螺纹长 4。
- STEP 主文件：`keyboard_mode.step`、`mid_mode.step`、`trackpad_mode.step`、`master_assembly.step`。
- 观察文件：`roller_detail_assembled.step`、`roller_detail_exploded.step`。
- 制造文件：`printables/` 下分件 STEP/STL 与六张单材料 3MF。
- 权威验证输出：`validation_report.json`。

当前强度结果属于 20 N 静载的一阶梁模型筛查，不是 FEA，也不证明层间疲劳寿命。首件必须执行 `ASSEMBLY.md` 的空载 20 次和带设备 100 次实体循环。
