# Cross-axis slider concept v6 design

## Scope

Create a new, independent `cross-axis-slider-concept-v6-symmetric-spring` concept. Preserve all v1-v5 files. Deliver keyboard, mid-travel, and trackpad-mode STEP files only, then stop for manual review.

This remains concept geometry. It does not claim final interference clearance, tolerance compliance, spring selection, friction performance, strength, print readiness, BOM, or manufacturing release.

## Motion and coordinate convention

- Units: millimeters.
- X: left/right; +Y: toward the display; +Z: upward.
- Keyboard tray: +30 Y and -15 Z from keyboard mode to trackpad mode.
- Trackpad tray: -115 Y and -7.400872 Z.
- The roller motion relative to each blue slot is +145 Y and -7.599128 Z.
- Therefore each straight slot is exactly -3.000° when read toward +Y. It descends in the keyboard push direction.
- Both trays remain level during their constrained translations.

## Bilateral active linkage

- Mirror the active red driver, shoulder shaft, captured roller, blue slotted cheeks, and trackpad ties on both sides.
- Both sides transmit load; neither side is merely decorative.
- The left side is the axial datum stack.
- The right side has 0.7 mm total X-direction axial float in its roller/cheek stack. This float absorbs print and assembly mismatch without changing the YZ slot constraint.
- Slot centerline length remains 170 mm, slot width 7.2 mm, and roller OD 6.0 mm unless geometry validation requires a larger endpoint reserve.
- The two endpoint poses and all sampled intermediate poses must keep each roller center on its slot centerline and inside the slot end limits.

## Keyboard guidance

- Replace visually ambiguous point rollers with one elongated captured guide shoe per side.
- Each shoe must have an explicit rigid connector to the keyboard side carriage and remain inside its fixed guide slot through the full stroke.
- The left guide is the primary lateral datum.
- The right guide receives additional X-direction assembly clearance so the bilateral system is not fully rigidly overconstrained.

## Compression-spring return concept

- Use two symmetric compression-spring axes, one near each keyboard side mechanism and parallel to the keyboard guide trajectory.
- Each spring sits between a fixed forward/lower seat and a moving collar rigidly attached to the keyboard carriage.
- Pressing the keyboard +Y/-Z shortens both spring envelopes by the keyboard path length of 33.541 mm; releasing it pushes the keyboard back and returns the trackpad through the captured slots.
- Show simplified spring solids and both seats in STEP so the load path is understandable. Their 43.541-to-10.000 mm envelope stays above the base plane. They are conceptual spring envelopes, not selected catalog springs.
- Reserve an initial concept range of 0.08-0.12 N/mm per spring. Two springs would add approximately 5.4-8.0 N over the full stroke, excluding preload, gravity, and friction.

## Base cleanup

- Reuse the compact 308 × 280 mm four-window base.
- Remove both three-hole return-anchor tabs. They have no function once the guide-axis compression springs are shown.
- Do not add desk clamps, palm rests, device reference solids, or unrelated hardware.

## STEP labels and visual language

- Keyboard tray and drivers: red.
- Trackpad tray and captured slotted linkage: blue.
- Active rollers: gold.
- Fixed guides and structural supports: dark neutral gray.
- Guide shoes, shafts, spring seats, and spring guides: light/medium neutral gray.
- Compression springs: a distinct metallic neutral color.
- Give left/right and fixed/floating roles explicit labels so CAD Explorer can isolate them.

## Validation gates

- Generate all three STEP poses successfully with non-empty labeled solids.
- Sample at least 201 motion positions.
- Maximum roller-to-slot centerline error below 0.001 mm.
- Positive roller-center reserve at both slot ends.
- Monotonic roller travel in both left and right slots.
- Spring compression equals keyboard path travel and is monotonic.
- Old three-hole anchor labels and solids are absent.
- Both active linkage sides and both spring assemblies are present.
- Run CAD inspection for facts, planes, and positioning, then visually review side and isometric snapshots.

## Manual review boundary

After publishing the three CAD Explorer links, stop for user review. Do not start final interference, tolerance, strength, manufacturing, STL/3MF, BOM, layout, ZIP, or patent work.
