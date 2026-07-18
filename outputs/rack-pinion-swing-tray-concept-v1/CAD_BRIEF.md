# Rack-pinion swing tray concept V1 — CAD brief

- Model: independent concept assembly for a keyboard-driven rack, vertical-axis pinion, and eccentric swing-arm Magic Trackpad tray.
- Units: millimetres.
- Coordinate system: X left/right; +Y toward the display; +Z up. Assembly origin remains at the original keyboard tray centre in keyboard mode.
- Primary outputs: `keyboard_mode.step`, `trackpad_mode.step`; low-cost additions: `mid_mode.step` and `cylinder_section_exploded.step`.
- Keyboard tray: continuous 279.7 × 115.7 × 3.5 mm plate; keyboard device envelope is intentionally hidden.
- Trackpad tray: continuous 160.8 × 115.7 × 3.5 mm plate with low locating lips; right edge aligns to X = +139.85 mm in trackpad mode.
- Base: continuous 310 × 300 × 4 mm plate centered at Y = 90.9 mm, on six labeled replaceable 16 mm diameter, 3 mm high non-slip foot interfaces. It contains the sampled swing projection, keyboard front edge, and parked-trackpad rear edge.
- Motion assumptions: keyboard travel +20.42 mm Y / -15 mm Z; vertical shaft rotates +90 degrees; rack displacement follows `s = r * theta` with pitch radius 13 mm.
- Gear concept: module 1, 26 teeth, 20-degree pressure angle, 13 mm pitch radius, 22 mm face width. The pinion uses a generated involute approximation; the rack uses matching 20-degree trapezoidal teeth with 0.12 mm conceptual backlash and phase derived from the same displacement equation.
- Fixed housing: 38 mm OD cylinder at the keyboard's right-rear; includes a full-height rack access window through the shell and mounting flange, lower and upper bearing seats, lower thrust interface, and independent endpoint hard stops.
- Shaft/arm: 8 mm vertical shaft at the right-rear position (120, 99.5). The keyboard and trackpad work target keep their original coordinates; the cylinder clears the keyboard by Y separation instead of cutting or shifting the continuous tray. Trackpad work centre is (59.45, 0), its right edge is +139.85 mm, and the eccentric arm radius is about 116.45 mm.
- Compactness refinement: the right-rear rack is 60 mm long and begins 2.15 mm behind the moving keyboard tray; swing arm width is 10 mm for the concept view.
- Labels: keyboard tray, rack, pinion, shaft, lower/upper bearing seats, thrust interface, fixed cylinder, swing arm, trackpad tray, two fixed hard stops, and rotating stop lug are individually selectable.
- Validation targets: STEP generation succeeds; assemblies contain labeled positive-volume solids; work target and rack/pinion travel relation are asserted in source; Explorer links and one diagnostic snapshot packet are produced.
- Explicitly deferred: continuous-pose collision report, final tooth geometry/backlash, tolerances, strength, deflection, fatigue, spring design, STL/3MF, BOM, print layout, ZIP, and patent work.
