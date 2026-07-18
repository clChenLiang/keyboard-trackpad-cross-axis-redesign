# V12 真实 CAD 拆解图

此目录是固定交付入口，避免在多版本审查目录中找不到产物。

- `exploded.step`：可旋转查看的真实 CAD 爆炸装配。
- `exploded.glb`：轻量三维预览文件。
- `exploded.png`：1600 × 1200 静态预览。
- `exploded-annotated.png`：2400 × 1700 中文编号说明图。
- `exploded_source.py`：从 V12 装配实体生成爆炸坐标的参数化源文件。
- `annotate_exploded.py`：只在真实渲染上叠加标题、引线、编号和运动链。

几何来源为 `cross-axis-slider-v12-usability-audit/assembly.py`。爆炸视图只对原装配
实体施加刚性显示位移，没有增加替代零件或 AI 生成的机械结构。
