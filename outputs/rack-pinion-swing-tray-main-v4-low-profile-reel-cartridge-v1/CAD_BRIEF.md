# V4 low-profile reel-cartridge concept

## Confirmed concept geometry

- The base is a single restrained, rounded open frame, `310 × 225 mm`. Its structural body runs from `Z=1` to `Z=7 mm`; the primary table pads reach `Z=-1.5 mm`. The reel cartridge and all return hardware begin at or above `Z=7 mm`, so no cylinder, shaft, spring cup, or cover hangs below the structural underside.
- The keyboard tray keeps a continuous `278.9 × 114.9 mm` keyboard-support plane. It carries only a short drive tongue and floating fork, not a long rigid rack or fence.
- The Magic Trackpad tray is sized for the `160.0 × 114.9 × 10.9 mm` envelope and is carried by an eccentric horizontal arm on the vertical output shaft.
- The drive uses a separately selectable, purchased-style open-ended `2M`, approximately `12 mm` wide toothed-belt envelope. The modeled pitch-line length is `68 mm`: `46 mm` initially straight plus `22 mm` initially wrapped.
- Four keyboard guide supports use inward transverse dovetails, solid end-wall datums, and top-access flexible snap tabs. Screws are not used in this concept.

## Motion relationship

The reel has `41` teeth at `2 mm` pitch, giving pitch radius `41 × 2 / (2π) = 13.05 mm`. A `20.50 mm` belt feed therefore gives `20.50 / 13.05 = π/2 rad = 90°` of shaft and swing-arm rotation. Across that command, the keyboard moves approximately `20.42 mm` forward and `15.00 mm` down. The belt-end slider stays on the fixed `Z=35 mm` belt plane while the short keyboard tongue follows the sloped tray path; the floating fork provides the required relative Z travel without forcing the belt to slope.

## Service interfaces

### Belt replacement

1. Remove the cartridge top cap.
2. Pull the screwless C-shaped reel retainer radially in `+X`, then lift the reel stack along the shaft.
3. Lift the reel-end tooth wedge in `+Z`; it releases five captured belt teeth from the reel pocket.
4. Withdraw the slider-end tooth wedge along `+X`, then remove the open-ended belt from its fixed planar entry guide.
5. Install the replacement belt at the same tooth phase. Seat the slider wedge from `+X` toward `-X`, seat the reel wedge from `+Z` toward `-Z`, reinstall the reel/retainer, and refit the cap.

Each modeled wedge captures five teeth and reaches a solid load datum. These are concept service directions, not production assembly instructions.

### Guide-support removal

Press the orange top-access release tab downward by the modeled `0.9 mm`, then slide the support transversely outward, opposite its inward insertion direction. Left supports insert in `+X` and withdraw in `-X`; right supports insert in `-X` and withdraw in `+X`. The dovetail faces and end wall carry the working loads; the tab provides retention only.

## Selectable component labels

The STEP assembly preserves separate labels for the base, four receivers, four removable guides, keyboard tray/carriages, short tongue, floating fork, belt-slider adapter, belt backing, belt teeth, both tooth wedges, belt slider, 41-tooth reel, lower thrust interface, lower and upper radial bearing seats, return spring and both anchors, vertical shaft, C-retainer, housing, planar entry guide, top cap, both fixed stops, rotating stop lug, eccentric arm, and trackpad tray.

Key labels include `flexible_belt_backing`, `flexible_belt_teeth`, `reel_drum_41t`, `screwless_belt_end_slider`, `keyboard_short_drive_tongue`, `floating_z_drive_fork`, `belt_slider_drive_adapter`, `above_base_return_spring`, `lower_radial_bearing_seat`, `upper_radial_bearing_seat`, `vertical_output_shaft`, `fixed_cartridge_housing`, `fixed_planar_belt_entry_guide`, `removable_cartridge_top_cap`, `independent_hard_stop_keyboard`, and `independent_hard_stop_trackpad`.

## Provisional assumptions and acceptance boundary

This is a STEP-first visual and kinematic concept. The helical spring is an explicit placeholder with provisional `1.2 mm` wire, `4` turns, and `0°` modeled preload; its rate, working preload, material, fatigue life, and manufacturable anchor details have not been sized. The exact belt product, tooth-form compatibility, belt endurance, production clearances/tolerances, tooth and wedge strength, shaft deflection, bearing selection, print material/process, print orientation, and life-cycle performance remain unvalidated.

Acceptance for this round is limited to visual approval of the low table profile, restrained rounded open base, absence of underside return hardware, independent belt and keyboard tray, belt winding into the supported reel cartridge, constant-height belt entry, eccentric trackpad motion, and screwless guide-support direction. Continuous-pose certification, production engineering, STL/3MF, BOM, print layout, ZIP packaging, and patent work are explicitly outside this round.
