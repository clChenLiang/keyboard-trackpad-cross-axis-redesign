# Cross-axis slider concept v1 — CAD brief

- Model: independent new concept assembly; no V7 geometry reused or overwritten.
- Primary output: three labeled STEP assemblies for keyboard, mid, and trackpad modes.
- Units and axes: millimetres; X left/right, +Y toward display, +Z up.
- Base: 310 × 300 × 4 mm, one-piece conceptual plate with slim side guide supports.
- Trays: two independent, continuous 279.7 × 115.7 × 3.5 mm plates; no device proxy solids.
- Motion: keyboard tray translates (+30, -15) mm; trackpad tray translates (-115, -18) mm.
- Constraint concept: two rollers per tray run in a straight side guide on each side, preserving tray pitch. A fixed roller pin on each keyboard drive arm crosses and slides inside a real slot in the trackpad link.
- Slot derivation: relative tray travel is (+145, +3) mm, so the slot axis is parallel to that vector. The pin is centred in the slot travel at mid-stroke.
- Return: three-hole, 5 mm pitch anchor tabs only; no spring is modeled in phase 1.
- Visual labels/colors: keyboard members red, trackpad members blue, fixed guides/base grey, coupling rollers gold.
- Validation targets: valid positive-volume solids, labeled STEP children, overall width no greater than 310 mm, three visibly distinct poses, and the coupling pin remaining within the real slot at t = 0, 0.5, 1.
- Deliberate phase-1 omissions: device envelopes, final collision/tolerance/strength checks, spring sizing, print files, BOM, and manufacturing release work.

