# V3 Integrated Return Drum and Rack Retention Design

## Scope

Quickly freeze the next V3 concept without changing the previously approved tray envelopes, four inclined keyboard supports, rack/pinion ratio, swing geometry, endpoint locations, hard stops, or trackpad arm layout.

This revision adds only:

- two rack back-pressure rollers at the pinion mesh;
- compliant preload on one roller;
- the existing coaxial torsion-return cartridge as a standard part of the main assembly.

It deliberately does not add endpoint detents, latches, production tolerances, spring sizing, strength calculations, manufacturing outputs, or continuous collision reporting.

## Rack retention

The four existing inclined guide assemblies continue to carry and locate the keyboard tray. The rack transmits motion only.

Two selectable rollers sit on the back side of the rack, opposite the pinion. Their axes are perpendicular to the rack's approximately 20 mm forward and 15 mm downward travel vector in the YZ plane, so the rack rolls across them during diagonal motion instead of being constrained to pure horizontal travel.

- The first roller establishes the nominal rack-to-pinion center distance.
- The second roller is mounted on a short selectable flexure arm and applies light preload toward the pinion.
- The roller bracket is fixed to the cylinder/base structure, not to the moving tray.
- Roller height and placement must clear the rack transition, cylinder shell, pinion, base, and tray in keyboard, mid, and trackpad poses.

The concept models the preload geometry and travel allowance, not a finalized spring rate.

## Integrated return drum

The already modeled bottom-mounted coaxial torsion cartridge becomes part of the standard V3 assembly rather than a separate optional derivative.

- The vertical drive shaft extends into the return cartridge.
- A keyed rotating clamp couples the spring moving end to the shaft.
- The spring fixed end anchors into one of the existing preload holes in the fixed cup.
- The fixed cup mounts beneath the base with its removable bottom cover retained.
- Replacement feet preserve clearance below the cartridge.
- Spring bias returns the mechanism from trackpad mode toward keyboard mode.

The cup, cover, spring, anchor, clamp, key, shaft, feet, both rollers, flexure, and bracket remain separately selectable STEP entities for inspection and maintenance.

## Outputs

Create a new directory so the earlier V3 and optional-module files remain untouched:

`outputs/rack-pinion-swing-tray-main-v3-integrated-return-v1/`

First-round outputs:

- `keyboard_mode.step`
- `mid_mode.step`
- `trackpad_mode.step`
- `cylinder_return_section_exploded.step`

## Focused validation

Only checks directly affected by this revision are required:

1. New parts and labels exist in all relevant outputs.
2. Both rollers maintain contact/near-contact with the rack back face at keyboard, mid, and trackpad poses.
3. The roller system does not create positive-volume interference with the rack, pinion, cylinder, transition, tray, or base at those poses.
4. The integrated cup, shaft, clamp, key, spring ends, cover, and feet retain their physical interfaces from the previously passing optional module.
5. The spring rotates with the shaft and biases toward keyboard mode.
6. The four STEP files load as valid solids/assemblies and receive a short visual review packet.

Previously validated, untouched V3 geometry is not re-tested.
