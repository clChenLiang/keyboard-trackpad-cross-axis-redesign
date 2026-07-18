# Cross-axis slider concept v7 low-lift design

## Scope and versioning

Create an independent `cross-axis-slider-concept-v7-low-lift-stop` concept. Preserve every v1-v6 source and generated artifact. Deliver keyboard, mid-travel, and trackpad-mode STEP files, then stop for manual review.

This is still concept geometry. It does not claim final tolerance, friction, spring selection, strength, manufacturability, or print readiness.

## Ergonomic target

- Keyboard final top surface: Z = 10.5 mm, unchanged.
- Trackpad final bottom surface: Z = 25.0 mm.
- Trackpad final top surface: Z = 28.5 mm.
- Final keyboard-to-trackpad working-surface difference: 18.0 mm.
- Preserve the 115 mm trackpad travel toward the user and the 30 mm keyboard travel toward the display.

## Constraint-derived motion

- Keyboard travel: +30 Y / -15 Z.
- Trackpad travel: -115 Y / -43 Z.
- Roller motion relative to each blue link: +145 Y / +28 Z.
- Straight slot angle: `atan2(28, 145) = +10.929488°` toward +Y.
- Input path: 33.541 mm; output path: 122.776 mm; displacement ratio approximately 3.66:1.
- The positive slot direction is intentional: lowering the trackpad takes priority over the earlier visual preference for a negative slot, and the steeper positive slot reduces roller side loading.

## Bilateral linkage and guidance

- Retain the v6 mirrored active linkage on both sides.
- Raise the crossover roller center from `keyboard_z + 21 mm` to `keyboard_z + 37 mm`. This 16 mm side-only height increase preserves the -43 mm trackpad endpoint while lifting the complete slot line above the base.
- Retain the left datum stack and the right 0.7 mm axial-floating stack.
- Retain the two extended keyboard guides, rigid shoe connectors, and seven-turn compression springs.
- Regenerate the fixed trackpad guides from the new -43 mm Z travel.
- Both trays remain level throughout their constrained translations.

## Terminal stop

- Add one fixed 10.0 mm stop core inside each keyboard compression spring.
- Core outside diameter: 1.8 mm maximum, below the 2.3 mm spring inside diameter.
- Each core is fixed to the forward/lower spring seat and points backward along the keyboard guide axis.
- At `travel=1`, the moving guide shoe contacts the end of the stop core while the spring envelope is 10.0 mm long.
- The stop core, not spring coil bind, defines the final keyboard and trackpad positions.
- Label the parts `keyboard_terminal_stop_core_left_datum` and `keyboard_terminal_stop_core_right_floating`.

## Clearance target

- All geometry must remain at or above the base bottom plane Z = 0.
- Moving linkage geometry should remain at least 2.0 mm above the base top plane Z = 4 in all three exported poses.
- If the exact -43 mm endpoint violates the 2.0 mm moving-link clearance, stop and report the measured conflict instead of silently changing the endpoint.

## Visual language

- Preserve v6 colors: keyboard and drivers red; trackpad and slotted links blue; active rollers gold; fixed guides dark gray; shoes, shafts, seats, and stop cores neutral metallic gray; springs bronze.
- Both terminal cores must be individually selectable in CAD Explorer.
- Do not add keyboard or trackpad device reference solids.

## Validation gates

- Sample at least 201 motion positions.
- Maximum roller-to-slot centerline error below 0.001 mm on both sides.
- Positive roller-center reserve at both slot ends.
- Monotonic bilateral roller travel and spring compression.
- Exact trackpad final bottom/top planes of 25.0/28.5 mm.
- Exact 10.0 mm terminal spring envelope and stop-core length.
- Both stop cores present; obsolete three-hole anchors absent.
- Generate and inspect three non-empty STEP files.
- Review side and isometric snapshots for slot direction, low final height, bilateral symmetry, stop-core placement, and base clearance.
- Confirm the raised crossover axes and taller red triangular drivers remain outside the input-surface envelope.

## Manual review boundary

After publishing the three v7 CAD Explorer links, stop. Do not start final interference, tolerance, strength, STL/3MF, BOM, layout, ZIP, or patent work.
