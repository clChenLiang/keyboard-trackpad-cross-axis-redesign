# Rack–pinion swing tray V2 — supported keyboard concept

This directory is an independent V2. The approved V1 files are not modified.

## Added keyboard support

- Four independent parallel inclined guide pods: left/right × front/rear.
- Each fixed guide has a slotted plate and two integral piers grounded at the
  main base top face.
- Each moving carriage has a 5 mm cross roller and an 8 mm under-tray overlap.
- All four carriage axes translate exactly +20.42 mm in Y and −15.00 mm in Z.
- The rack remains the drive element; it is not treated as the sole structural
  support for the keyboard tray.
- Guide posts sit beyond both travel endpoints so the carriage cannot hit a
  post during the stroke.

## Selectable STEP entities

- `fixed_keyboard_guide_left_front`
- `fixed_keyboard_guide_right_front`
- `fixed_keyboard_guide_left_rear`
- `fixed_keyboard_guide_right_rear`
- `keyboard_carriage_left_front`
- `keyboard_carriage_right_front`
- `keyboard_carriage_left_rear`
- `keyboard_carriage_right_rear`

The rack, pinion, shaft, bearing seats, fixed cylinder, swing arm, trackpad
tray, keyboard tray, stops, base, and feet retain their V1 labels.

## Scope boundary

This is still a low-cost kinematic concept. Roller hardware, fasteners,
bearing selection, tolerance stack, strength, stiffness, fatigue, return
spring, manufacturability, BOM, mesh exports, and printing layout are not yet
finalized.
