# V11 可打印联动支架 BOM

单位均为 mm。肩轴使用可直接采购的低头精密规格；不要用全螺纹 M4 螺钉代替滚轮轴。

## 打印件

| 材料 | 零件 | 数量 | 文件 |
|---|---:|---:|---|
| PETG-HF | 开放式底座 | 1 | `printables/base.*` |
| PETG-HF | 左/右固定侧框 | 各 1 | `printables/side_frame_left.*`, `side_frame_right.*` |
| PETG-HF | 键盘托、妙控板托 | 各 1 | `printables/keyboard_tray.*`, `trackpad_tray.*` |
| PETG-HF | 键盘左右侧架 | 各 1 | `printables/keyboard_carriage_left.*`, `keyboard_carriage_right.*` |
| PETG-HF | 妙控板左右侧架 | 各 1 | `printables/trackpad_carriage_left.*`, `trackpad_carriage_right.*` |
| PETG-HF | 交叉偏心套 | 2 | `printables/cross_eccentric_bushing.*` |
| PETG-HF | 导向偏心套 | 4 | `printables/guide_eccentric_bushing.*` |
| TPU 95A | 端位缓冲环 | 4 | `printables/tpu_stop_collar.*` |
| TPU 95A | 右侧浮动垫片 | 4 | `printables/right_float_washer.*` |
| TPU 95A | 键盘设备垫 | 4 | `printables/keyboard_tpu_pad.*` |
| TPU 95A | 妙控板设备垫 | 4 | `printables/trackpad_tpu_pad.*` |
| TPU 95A | 底座防滑脚 | 4 | `printables/tpu_base_foot.*` |
| PETG-HF | 尺寸试片 | 1 | `printables/calibration_coupon.*` |

## 轴承、肩轴与锁紧件

| 零件 | 数量 | 精确规格 | 用途 |
|---|---:|---|---|
| 604ZZ | 8 | 4 × 12 × 4，双面防尘 | 两托盘双侧直线导向 |
| MR84ZZ | 4 | 4 × 8 × 3，双面防尘 | 交叉长槽联动 |
| 低头精密肩轴 | 4 | Ø4 光肩长 20；M3 螺纹长 4；头部 Ø6 × 2 | 键盘导向 |
| 低头精密肩轴 | 4 | Ø4 光肩长 16；M3 螺纹长 4；头部 Ø6 × 2 | 妙控板导向 |
| 低头精密肩轴 | 4 | Ø4 光肩长 12；M3 螺纹长 4；头部 Ø6 × 2 | 交叉滚轮 |
| M3 平垫片 | 12 | 3.2 ID × 7 OD × 0.5 | 仅压轴承内圈 |
| DIN 934 M3 螺母 | 12 | 厚 2.4，5.5 对边 | 肩轴端部保持 |
| 中强度可拆螺纹锁固剂 | 少量 | 适用于 M3 金属螺纹 | 防止螺母松脱 |

## 回位与结构紧固

| 零件 | 数量 | 精确规格 | 用途 |
|---|---:|---|---|
| 定制压缩弹簧 | 2 | SUS304-WPB；OD 14，自由长 111，线径 1.10，约 32.8 有效圈，刚度 0.200 N/mm，压并长不大于 38 | 双侧回位 |
| 不锈钢精密杆 | 2 | Ø3.0 × 122.0，端面倒钝 | 弹簧防屈曲导杆兼端位环载体 |
| M4 热熔螺母 | 12 | OD 6.2 × 长 5.2 | 底座 4、托盘 8 |
| M4 × 8 内六角螺钉 | 4 | 普通圆柱头 | 底座到侧框 |
| M4 × 10 内六角螺钉 | 4 | 普通圆柱头 | 键盘托到侧架 |
| M4 × 16 内六角螺钉 | 4 | 普通圆柱头 | 妙控板托到侧架 |

## 不可替代项

- 604ZZ、MR84ZZ 的内孔都是 4 mm；轴承不得在螺纹段上转动。
- 三种肩长采用 20/16/12 标准档，装配层间距由这些现货尺寸反推，左右误差为 0；替换长度会改变轴向夹紧或留下间隙。
- 弹簧不是仅按外形选取。供应商图纸必须同时确认自由长、刚度、线径、有效圈数和压并长；若任一项变化，必须重新运行 `validation.py` 的回位力、压并和 Wahl 修正剪应力检查。
- TPU 浮动垫片只装在右侧托盘连接处，左侧是定位基准侧。
