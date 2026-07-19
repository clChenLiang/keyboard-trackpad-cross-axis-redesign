# V4 Low-Profile Reel-Cartridge Design

## Objective

Create a new derivative that combines the approved V3 keyboard/trackpad motion with the independently validated A-P flat-reel mechanism. The revision removes the long rigid rack from the keyboard tray, relocates all return hardware above the base plane, eliminates the guide-support screws, and gives the base a restrained rounded appearance.

This is a new V4 concept. It must not overwrite the existing V3, integrated-return, open-frame-base, or A-P coupon files.

## Confirmed design language

Use the selected **C · restrained radius** direction:

- retain a clearly engineered open-frame layout;
- round the outer base corners and the corners of the three main openings;
- use modest radii rather than flowing or decorative ribs;
- soften the transitions into guide lands and the right-side cylinder pod;
- preserve straight mounting datums and printable flat faces wherever a part locates another part.

The first concept should use approximately `8–12 mm` outer-corner radii and `6–10 mm` opening radii. These are visual starting values, not production tolerances.

## Low-profile base

Retain the current `310 × 225 mm` plan envelope and the approved mechanism top datum at `Z=7 mm`.

- Keep up to `6 mm` of structural depth on direct load paths below the four guide stations and the cylinder pod.
- Allow the visible perimeter and low-load transitions to taper or step down locally so the frame does not read as a uniformly thick slab.
- Remove the underside torsion-cup recess, shaft extension, removable bottom cover, and long clearance feet.
- Use short TPU pads retained by shallow printed pockets. A first concept may place the table plane near `Z=-1.5 mm`, reducing the nominal table-to-base-top height from about `22 mm` to about `8.5 mm` while retaining the `Z=7 mm` mechanism datum.
- Keep four primary pads as the table-defining contacts. Two centre auxiliary pads may remain `0.3 mm` shorter if the open frame still benefits from anti-sag support.

No return component may extend below the structural base underside.

## Keyboard tray and drive decoupling

The keyboard tray is no longer integral with a rigid rack or long toothed fence.

- The tray remains a continuous keyboard-supporting plane.
- Add only a short, rigid drive tongue at the tray's right side.
- The tongue engages a separate floating fork.
- The floating fork follows the keyboard's approximately `20.5 mm` forward and `15 mm` downward motion.
- Two vertical drive keys transmit the forward component while allowing the belt-end slider to remain at a fixed world Z.
- The keyboard tray, floating fork, belt-end slider, flexible toothed belt, and reel cartridge remain separately selectable entities.

This preserves the A-P coupon's proven separation between the diagonal keyboard motion and the planar belt feed.

## Flexible toothed belt

Replace the module-1 rigid rack and pinion with the validated planar, single-layer synchronous belt concept:

- nominal profile: `2M-GT2`-compatible concept geometry;
- belt pitch: `2.00 mm`;
- belt width: approximately `12 mm`;
- reel tooth count: `41`;
- pitch diameter: `41 × 2 / π = 26.1014 mm`;
- nominal feed: `20.50 mm` for `90°` reel rotation;
- belt centreline height remains fixed relative to the cylinder entry guide;
- the belt winds in one planar layer rather than spiralling axially.

The previous rigid rack, module-1 pinion, and rack back-pressure rollers are therefore deleted rather than retained as redundant transmission parts.

The production tooth profile, belt reinforcement, fatigue life, and final material remain outside this concept round.

## Reel cartridge

Treat the belt and reel as one replaceable **functional cartridge**, but not as one same-material monolithic print.

- The reel, shaft interfaces, bearing seats, hard stops, and housing use rigid material.
- The repeatedly flexed toothed belt uses a flexible, fatigue-capable material.
- The belt root terminates in a printed enlarged tongue or dovetail wedge.
- The tongue slides into a matching reel slot and becomes geometrically self-locking under belt tension.
- No screw is used at the belt-to-reel interface.
- A service opening or removable top cap must permit belt replacement without accessing the underside of the base.

The existing fixed cylinder becomes the cartridge housing. Target the previous compact `38–42 mm` outside-diameter range where practical; the first model may grow only if the `26.10 mm` pitch reel, belt thickness, walls, and bearings cannot fit honestly.

## Above-base return stack

Relocate the coaxial return spring into the lower portion of the fixed cylinder.

The conceptual vertical order is:

1. base-integrated lower thrust/radial seat;
2. above-base coaxial return-spring chamber;
3. toothed reel and planar belt-entry level;
4. upper radial support and output-shaft interface;
5. eccentric trackpad swing arm.

The former bottom-mounted spring cup, bottom cover, and extended shaft are deleted. The return spring biases the reel toward keyboard mode. The spring remains a separately selectable component, and its rate and preload remain provisional.

Independent `0°` and `90°` hard stops remain inside the fixed housing. The return spring must not serve as the motion stop.

## Screwless guide-support mounting

Replace the base fasteners for the four inclined keyboard-support assemblies with printed top-side receivers.

- Each base guide land contains a transverse female dovetail slide. Left supports insert inward along `+X`; right supports insert inward along `-X`. Keyboard operating force is primarily along Y and therefore cannot drive a support back out of its receiver.
- Each support foot contains the matching male dovetail.
- A solid end wall defines the installed datum.
- A printable cantilever snap tab drops into a window at full insertion.
- Pressing the tab from above releases the support for reverse sliding removal.
- Dovetail faces provide vertical and lateral constraint; the end wall carries the longitudinal reaction. The snap tab is retention only and must not carry the main keyboard load.

The same no-screw principle applies to the belt-end slider: an enlarged belt-end key inserts transversely into a captured slot, reaches a solid end datum, and is retained by geometry rather than clamp screws.

## Component boundaries

The first STEP assembly must expose at least these selectable labels:

- rounded low-profile base;
- four primary TPU pads and optional two auxiliary pads;
- four female base slide receivers;
- four removable guide-support feet with snap tabs;
- continuous keyboard tray with short drive tongue;
- floating-Z drive fork;
- screwless belt-end slider;
- flexible toothed belt backing and transverse teeth;
- 41T reel drum;
- fixed cartridge housing and entry guide;
- lower and upper shaft supports;
- above-base return spring and anchors;
- vertical output shaft;
- independent hard stops;
- eccentric trackpad swing arm;
- reduced Magic Trackpad tray.

## New output directory

Create:

`outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/`

First-round deliverables:

- `keyboard_mode.step`;
- `mid_mode.step`;
- `trackpad_mode.step`;
- `reel_cartridge_section_exploded.step`;
- `screwless_guide_mount_detail.step` if it is low-cost after the main assembly works.

Do not overwrite or delete the prior A-P coupon; reuse its validated kinematic functions and geometry assumptions where practical.

## Focused validation

Validate only the interfaces changed by this architecture:

1. `20.50 mm` planar belt feed produces `90°` reel rotation.
2. The belt-end slider remains at constant world Z while the keyboard tongue moves forward and down.
3. Belt centreline length remains constant at keyboard, midpoint, and trackpad poses.
4. At least five belt teeth occupy matching reel grooves at the stored endpoint without positive reel penetration.
5. The return spring, reel, shaft, bearings, belt, and housing have real supporting interfaces and remain above the base underside.
6. The independent hard stops contact only at their intended endpoints.
7. Each dovetail support foot reaches its solid end datum, is vertically and laterally captured, and does not rely on the snap tab for primary load transfer.
8. The base and moving assembly have no unintended positive-volume interference at the three exported poses.
9. All generated STEP files reload with their required selectable labels.
10. A compact render packet confirms the low table height, restrained radii, belt entry, cartridge stack, and screwless mount orientation.

The already passing A-P tests may be reused rather than rewritten. Do not perform final belt fatigue, spring sizing, tooth stress, shaft deflection, production tolerance, STL/3MF, BOM, print-layout, or continuous-pose certification in this first concept.

## Acceptance boundary

This design round is accepted when the user can clearly see and approve:

- the lower table profile;
- the restrained rounded base;
- the absence of underside return hardware;
- the separation of keyboard tray and toothed belt;
- the belt winding into the reel cartridge;
- the removable, screwless guide-support slides.

Material and endurance decisions follow only after that visual and kinematic approval.
