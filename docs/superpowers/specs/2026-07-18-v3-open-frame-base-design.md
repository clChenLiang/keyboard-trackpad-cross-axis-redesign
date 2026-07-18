# V3 Open-Frame Base Design

## Objective

Replace the current `310 × 300 × 4 mm` continuous plate in the integrated-return V3 assembly with a smaller FDM-oriented open frame. Preserve the approved keyboard/trackpad motion, guide positions, rack/pinion geometry, return cartridge, rack-retention system, and all top-side mounting datums.

Create the revision in a new output directory. Do not overwrite the integrated-return V1 files.

## Envelope and datum

- Outer width: `310 mm`.
- Outer depth: `225 mm`.
- Y range: `-55 … 170 mm`; center `57.5 mm`.
- Nominal structural depth: `6 mm`.
- Existing top mounting datum remains at `Z=7 mm`.
- The main frame therefore occupies `Z=1 … 7 mm`.

Width is retained because the guide mount feet and rack-retention bracket already reach approximately `X=-153 … 153.5 mm`. Depth is reduced because fixed mechanism hardware occupies approximately `Y=-43.6 … 125.3 mm`. The parked trackpad may overhang the rear of the frame; its centre remains near the rear support line.

## Frame topology

Build the base as a union of explicit load-path solids rather than a full plate with decorative perforations:

- `12 mm` perimeter rails;
- three transverse cross-members near the front guide group, rear guide group, and shaft pod;
- full local islands below all four guide-foot groups;
- a widened right-side structural spine below the fixed cylinder, rack-retention bracket, and return cartridge;
- a local annular mounting pad around the return-cartridge bolt circle.

Keep the central low-load regions open. Openings must not cross the direct paths between guide mounts, cylinder pod, rack-retention bracket, and feet.

## Cartridge and foot interfaces

The 6 mm frame extends downward so the established top datum does not move.

- Cut a `2 mm`-deep underside recess around the torsion-cartridge flange. The flange continues to seat at `Z=3 mm`; the remaining local roof is `4 mm` thick up to `Z=7 mm`.
- Preserve the shaft bore and three cartridge fastener holes.
- Use four primary TPU feet near the frame corners to define the table plane.
- Add two centre auxiliary TPU pads that are `0.3 mm` shorter and engage only after small frame deflection.
- Model shallow underside foot pockets so the feet have real lateral retention and do not overlap the base solid.
- Preserve at least `2 mm` clearance between the return-cartridge bottom cover and the table plane.

## Manufacturing intent

- FDM printing only; no metal plate or folded-sheet assumptions.
- The base remains a separately selectable part.
- Mounting islands and rails may be split into printable subassemblies in a later manufacturing phase, but this revision remains a one-piece concept STEP.
- No heat-set insert sizing, slicer settings, print layout, BOM, or final material calculation in this round.

## Output

Create:

`outputs/rack-pinion-swing-tray-main-v3-open-frame-base-v1/`

Generate only:

- `keyboard_mode.step`
- `mid_mode.step`
- `trackpad_mode.step`
- `base_and_return_interface.step`

## Focused validation

Run only checks affected by the base revision:

1. The new base is a valid positive-volume solid with the specified envelope and unchanged `Z=7 mm` top datum.
2. All guide-foot, cylinder, rack-retention, cartridge, and fastener interfaces are supported by positive base material.
3. The flange recess and foot pockets remove base overlap while preserving seating contact.
4. All four primary feet share one table plane; the two auxiliary pads are `0.3 mm` shorter.
5. The cartridge bottom cover retains at least `2 mm` table clearance.
6. The base does not interfere with moving trays, rack, pinion, arm, guides, cartridge, or retention hardware at keyboard, midpoint, and trackpad poses.
7. The four new STEP files reload as valid solids/assemblies and receive a compact visual review.

Do not rerun unrelated historical mechanism, strength, fatigue, or continuous-pose test suites.
