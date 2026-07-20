# V4 Low-Profile Reel Cartridge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a new STEP-first V4 concept that preserves the approved keyboard/trackpad motion while replacing the rigid rack with a purchased open-ended 2M toothed belt, a serviceable above-base return reel cartridge, a lower rounded base, and screwless guide-support mounts.

**Architecture:** Reuse the proven V3 moving geometry and A-P coupon kinematics through thin import adapters, then compose new V4-only parts in focused modules. Keep the diagonal keyboard motion separate from the constant-Z belt path with a floating fork. Model the reel, spring, bearings, stops, housing, belt wedges, and belt entry as one replaceable functional cartridge, while keeping every physical item as a separately selectable STEP entity.

**Tech Stack:** Python, build123d, pytest, OpenCascade STEP, repository CAD launcher/inspector, CAD Explorer renderer

---

## Working rules

- Work only in `/Users/bytedance/Documents/Codex/2026-07-15/keyboard-trackpad-cross-axis-redesign/.worktrees/v3-open-frame-base` on branch `agent/v3-open-frame-base`.
- Create only `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/`; do not overwrite V3, V3 open-frame, integrated-return, or A-P coupon files.
- Use `apply_patch` for source edits.
- Use the `cad` skill before generating or inspecting STEP and the `render` skill before starting CAD Explorer or producing review images.
- Keep first-round scope to three assembly poses, cartridge section/explosion, and the low-cost mount detail. Do not add fatigue, final spring sizing, strength, tolerances, STL/3MF, BOM, print layout, ZIP, or patent work.
- Every commit must end with a blank line and the literal trailer `Co-Authored-By: Riff`.

## Task 1: Establish V4 kinematics and a constant-Z belt interface

**Files:**

- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/__init__.py`
- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/v4_kinematics.py`
- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/test_v4_kinematics.py`
- Reference: `outputs/ap-flat-reel-floating-z-coupon-v1/ap_coupon.py`
- Reference: `outputs/rack-pinion-swing-tray-main-v3/rack_pinion_swing_main_v3.py`

- [ ] **Step 1: Write failing kinematic tests**

Cover all three named poses and the changed interface, not the already proven full A-P sweep:

```python
import math
import pytest

from v4_kinematics import BELT_Z, pose_state


@pytest.mark.parametrize(
    ("travel", "feed", "angle_deg", "keyboard_dy", "keyboard_dz"),
    [(0.0, 0.0, 0.0, 0.0, 0.0),
     (0.5, 10.25, 45.0, 10.21, -7.5),
     (1.0, 20.50, 90.0, 20.42, -15.0)],
)
def test_pose_state(travel, feed, angle_deg, keyboard_dy, keyboard_dz):
    state = pose_state(travel)
    assert state.belt_feed == pytest.approx(feed, abs=0.01)
    assert math.degrees(state.reel_angle) == pytest.approx(angle_deg, abs=0.01)
    assert state.keyboard_dy == pytest.approx(keyboard_dy, abs=0.02)
    assert state.keyboard_dz == pytest.approx(keyboard_dz, abs=0.01)
    assert state.belt_z == BELT_Z


def test_belt_length_is_constant_at_exported_poses():
    lengths = [pose_state(t).belt_centerline_length for t in (0.0, 0.5, 1.0)]
    assert max(lengths) - min(lengths) < 1e-6
```

- [ ] **Step 2: Run the test and confirm the intended failure**

Run:

```bash
cd outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1
python3 -m pytest -q test_v4_kinematics.py
```

Expected: collection fails because `v4_kinematics` does not exist.

- [ ] **Step 3: Implement the minimal shared state**

Use a frozen dataclass, clamp travel to `[0, 1]`, and derive reel angle from pitch radius rather than hard-coding both values independently:

```python
from dataclasses import dataclass
from math import pi

BELT_PITCH = 2.0
REEL_TEETH = 41
REEL_PITCH_RADIUS = REEL_TEETH * BELT_PITCH / (2.0 * pi)
FEED_TRAVEL = 20.50
KEYBOARD_DY = 20.42
KEYBOARD_DZ = -15.0
BELT_Z = 35.0
FREE_BELT_CENTERLINE = 46.0


@dataclass(frozen=True)
class V4PoseState:
    travel: float
    belt_feed: float
    reel_angle: float
    keyboard_dy: float
    keyboard_dz: float
    belt_z: float
    belt_centerline_length: float


def pose_state(travel: float) -> V4PoseState:
    if not 0.0 <= travel <= 1.0:
        raise ValueError("travel must be within [0, 1]")
    feed = FEED_TRAVEL * travel
    return V4PoseState(
        travel=travel,
        belt_feed=feed,
        reel_angle=feed / REEL_PITCH_RADIUS,
        keyboard_dy=KEYBOARD_DY * travel,
        keyboard_dz=KEYBOARD_DZ * travel,
        belt_z=BELT_Z,
        belt_centerline_length=FREE_BELT_CENTERLINE,
    )
```

- [ ] **Step 4: Run focused tests**

Run:

```bash
python3 -m pytest -q test_v4_kinematics.py
```

Expected: all tests pass; full travel evaluates to `90.0°` within `0.01°`.

- [ ] **Step 5: Commit the kinematic seam**

```bash
git add outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1
git commit -m $'Add V4 reel cartridge kinematics\n\nCo-Authored-By: Riff'
```

## Task 2: Model the purchased belt and both screwless tooth wedges

**Files:**

- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/v4_reel_cartridge.py`
- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/test_v4_reel_cartridge.py`
- Reference: `outputs/ap-flat-reel-floating-z-coupon-v1/ap_coupon.py`

- [ ] **Step 1: Write failing belt and retention tests**

Test semantic reports in addition to raw geometry so failures explain the interface that broke:

```python
import pytest

from v4_reel_cartridge import (
    BELT_WIDTH,
    build_belt_system,
    belt_retention_report,
    reel_engagement_report,
)


@pytest.mark.parametrize("travel", [0.0, 0.5, 1.0])
def test_reel_has_minimum_engagement_without_penetration(travel):
    report = reel_engagement_report(travel)
    assert report.engaged_teeth >= 5
    assert report.positive_penetration_volume < 1e-3


def test_both_wedges_capture_four_teeth_and_seat():
    report = belt_retention_report()
    assert report.reel_wedge_captured_teeth >= 4
    assert report.slider_wedge_captured_teeth >= 4
    assert report.reel_wedge_at_datum
    assert report.slider_wedge_at_datum
    assert report.reel_working_direction_blocked
    assert report.slider_working_direction_blocked


def test_belt_width_remains_supplier_replaceable_envelope():
    assert BELT_WIDTH == pytest.approx(12.0)
    labels = {part.label for part in build_belt_system(0.5).children}
    assert {"flexible_belt_backing", "flexible_belt_teeth",
            "reel_end_tooth_wedge", "slider_end_tooth_wedge"} <= labels
```

- [ ] **Step 2: Run and confirm failure**

```bash
python3 -m pytest -q test_v4_reel_cartridge.py
```

Expected: import failure because `v4_reel_cartridge.py` is not yet present.

- [ ] **Step 3: Build the belt envelope and 41T reel**

Port only the proven A-P geometric assumptions: `2.0 mm` pitch, `12 mm` width, `1.2 mm` backing, `0.9 mm` tooth depth, `41` reel teeth, and a planar single-layer wrap. Keep backing and teeth as distinct solids. Generate teeth from indexed pitch positions, and pose the straight segment from `pose_state(travel)`.

The public constructor must remain stable:

```python
def build_belt_system(travel: float) -> Compound:
    state = pose_state(travel)
    return Compound(children=[
        _straight_belt_backing(state, label="flexible_belt_backing"),
        _straight_and_wrapped_teeth(state, label="flexible_belt_teeth"),
        _reel_drum(state, label="reel_drum_41t"),
        _tooth_wedge(REEL_WEDGE, label="reel_end_tooth_wedge"),
        _tooth_wedge(SLIDER_WEDGE, label="slider_end_tooth_wedge"),
        _belt_end_slider(state, label="screwless_belt_end_slider"),
    ])
```

- [ ] **Step 4: Add real wedge seating geometry**

Model each wedge with four or five matching tooth pockets, a tapered non-tooth back face, a solid insertion datum, and an opening direction orthogonal to working tension. The reel wedge inserts along `-Z` and is blocked by the removable cap. The slider wedge inserts transversely into its captured constant-Z slot. Do not substitute friction clamps or screws.

- [ ] **Step 5: Implement BREP-backed reports and run tests**

`reel_engagement_report()` must count groove occupancy and compute intersection volume between belt teeth and reel land solids. `belt_retention_report()` must derive captured-tooth count from pocket occupancy and datum contact, not return unconditional constants.

Run:

```bash
python3 -m pytest -q test_v4_reel_cartridge.py
```

Expected: all belt, engagement, label, and wedge tests pass.

- [ ] **Step 6: Commit the belt system**

```bash
git add outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1
git commit -m $'Model serviceable V4 toothed belt interfaces\n\nCo-Authored-By: Riff'
```

## Task 3: Complete the honest above-base cartridge stack

**Files:**

- Modify: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/v4_reel_cartridge.py`
- Modify: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/test_v4_reel_cartridge.py`
- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/reel_cartridge_section_exploded.py`

- [ ] **Step 1: Add failing support, spring, and stop tests**

```python
from v4_reel_cartridge import cartridge_stack_report, stop_contact_report


def test_every_rotating_part_has_a_real_support_and_is_above_base():
    report = cartridge_stack_report()
    assert report.minimum_z >= report.base_structural_underside_z
    assert report.lower_radial_supported
    assert report.lower_axial_supported
    assert report.upper_radial_supported
    assert report.spring_inner_anchor_contact
    assert report.spring_outer_anchor_contact
    assert report.top_cap_blocks_reel_wedge_z_exit


def test_independent_stops_contact_only_at_endpoints():
    assert stop_contact_report(0.0).keyboard_stop_contact
    assert not stop_contact_report(0.0).trackpad_stop_contact
    assert not stop_contact_report(0.5).any_contact
    assert stop_contact_report(1.0).trackpad_stop_contact
    assert not stop_contact_report(1.0).keyboard_stop_contact
```

- [ ] **Step 2: Confirm tests fail for missing reports**

```bash
python3 -m pytest -q test_v4_reel_cartridge.py
```

Expected: import or assertion failures for the unimplemented cartridge stack.

- [ ] **Step 3: Build the vertical stack in physical order**

Construct and label these separate entities, all above the structural underside:

1. `lower_axial_thrust_interface`
2. `lower_radial_bearing_seat`
3. `above_base_return_spring`
4. `return_spring_inner_anchor`
5. `return_spring_outer_anchor`
6. `reel_drum_41t`
7. `upper_radial_bearing_seat`
8. `vertical_output_shaft`
9. `fixed_cartridge_housing`
10. `fixed_planar_belt_entry_guide`
11. `removable_cartridge_top_cap`
12. both independent stops and the shaft stop lug

Use a helical solid for the conceptual spring, but keep wire diameter, turns, and preload explicitly provisional in `CAD_BRIEF.md`. The spring must visibly connect its inner and outer anchors. The reel and shaft must not float between bearing seats.

- [ ] **Step 4: Add the sectional/exploded assembly entry point**

`reel_cartridge_section_exploded.py` must call one shared cartridge builder with `sectioned=True, exploded=True`; it must not duplicate the cartridge geometry. Offset the top cap, upper bearing seat, reel, spring, and lower interfaces vertically while retaining alignment witness axes.

- [ ] **Step 5: Run cartridge tests**

```bash
python3 -m pytest -q test_v4_reel_cartridge.py
```

Expected: support/anchor tests pass, midpoint has no stop contact, and only the intended stop contacts at each endpoint.

- [ ] **Step 6: Commit the cartridge stack**

```bash
git add outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1
git commit -m $'Complete V4 above-base return cartridge\n\nCo-Authored-By: Riff'
```

## Task 4: Build the rounded low-profile base and screwless supports

**Files:**

- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/v4_screwless_base.py`
- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/test_v4_screwless_mounts.py`
- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/screwless_guide_mount_detail.py`

- [ ] **Step 1: Write failing base and mount tests**

```python
import pytest

from v4_screwless_base import build_low_profile_base, mount_report


def test_low_base_keeps_approved_datums():
    base = build_low_profile_base()
    assert base.bounding_box().size.X == pytest.approx(310.0, abs=0.1)
    assert base.bounding_box().size.Y == pytest.approx(225.0, abs=0.1)
    assert base.bounding_box().max.Z == pytest.approx(7.0, abs=0.1)
    assert base.bounding_box().min.Z >= -1.5


def test_four_supports_are_captured_by_load_bearing_geometry():
    report = mount_report()
    assert report.receiver_count == 4
    assert report.all_at_end_datum
    assert report.all_vertically_captured
    assert report.all_laterally_captured
    assert report.snap_tabs_clear_primary_load_path
    assert report.left_insertion_direction == "+X"
    assert report.right_insertion_direction == "-X"
```

- [ ] **Step 2: Run and confirm failure**

```bash
python3 -m pytest -q test_v4_screwless_mounts.py
```

Expected: import failure because the low-profile base module is missing.

- [ ] **Step 3: Model the restrained rounded open frame**

Start from the approved `310 × 225 mm` plan, keep structural depth only under the guide stations and cartridge pod, and use approximately `10 mm` outer and `8 mm` opening radii. Preserve flat XY datums at support receivers. Use four primary TPU pockets to define a table plane near `Z=-1.5 mm`; if retained, make two centre auxiliary pads `0.3 mm` shorter.

Required labels:

```python
BASE_LABELS = {
    "rounded_low_profile_base",
    "primary_tpu_pad_front_left", "primary_tpu_pad_front_right",
    "primary_tpu_pad_rear_left", "primary_tpu_pad_rear_right",
    "female_slide_receiver_front_left", "female_slide_receiver_front_right",
    "female_slide_receiver_rear_left", "female_slide_receiver_rear_right",
}
```

- [ ] **Step 4: Model each transverse dovetail and release tab**

Use a solid end wall for Y reaction, dovetail flanks for Z/lateral capture, and a top-access cantilever tab only for reverse-slide retention. Verify the installed part cannot exit along keyboard operating force. Keep male foot, guide support, and snap tab together as one replaceable support part, while keeping each of the four supports separately labeled.

- [ ] **Step 5: Create the mount detail entry point and run tests**

`screwless_guide_mount_detail.py` must show one receiver, one installed foot, one exploded foot, insertion arrow/witness, end wall, and release window.

```bash
python3 -m pytest -q test_v4_screwless_mounts.py
```

Expected: base height and envelope pass; all four mounts reach the end datum and are geometrically captured.

- [ ] **Step 6: Commit the base and mounts**

```bash
git add outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1
git commit -m $'Add rounded V4 base and screwless supports\n\nCo-Authored-By: Riff'
```

## Task 5: Compose the complete three-pose V4 mechanism

**Files:**

- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/v4_assembly.py`
- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/test_v4_integration.py`
- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/keyboard_mode.py`
- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/mid_mode.py`
- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/trackpad_mode.py`
- Reference: `outputs/rack-pinion-swing-tray-main-v3/rack_pinion_swing_main_v3.py`

- [ ] **Step 1: Write failing integration tests**

```python
import pytest

from v4_assembly import REQUIRED_LABELS, build_pose, interference_report


@pytest.mark.parametrize("travel", [0.0, 0.5, 1.0])
def test_pose_has_required_selectable_entities(travel):
    labels = {part.label for part in build_pose(travel).children}
    assert REQUIRED_LABELS <= labels
    assert "keyboard_right_straight_rack" not in labels
    assert "module_1_26t_pinion" not in labels
    assert not any("back_pressure_roller" in label for label in labels)


def test_changed_parts_do_not_interfere_at_exported_poses():
    report = interference_report((0.0, 0.5, 1.0))
    assert report.unintended_positive_volume_pairs == ()


def test_keyboard_and_belt_are_separate_entities():
    assembly = build_pose(0.5)
    labels = {part.label for part in assembly.children}
    assert "keyboard_tray_continuous" in labels
    assert "keyboard_short_drive_tongue" in labels
    assert "floating_z_drive_fork" in labels
    assert "flexible_belt_backing" in labels
```

- [ ] **Step 2: Run and confirm failure**

```bash
python3 -m pytest -q test_v4_integration.py
```

Expected: import failure because `v4_assembly.py` is missing.

- [ ] **Step 3: Adapt, do not edit, the V3 moving mechanism**

Import the V3 builder through `importlib.util.spec_from_file_location`. Reuse the continuous keyboard tray envelope, four inclined guides/carriages, vertical shaft, eccentric arm, and reduced trackpad tray. Filter out the V3 base, rigid rack, 26T pinion, rack rollers, old fixed cylinder, old bearing/stop stack, and all underside return pieces before adding V4 replacements.

- [ ] **Step 4: Add the short tongue and floating-Z fork**

The keyboard tray receives only a short rigid tongue. The fork follows keyboard `Y/Z`; two vertical drive keys transmit Y while the belt-end slider remains exactly at `BELT_Z`. Leave visible sliding engagement at both endpoints, including the full `15 mm` Z differential.

The assembly seam must be explicit:

```python
def build_pose(travel: float) -> Compound:
    state = pose_state(travel)
    legacy_parts = _filtered_v3_parts(state)
    return Compound(children=[
        *legacy_parts,
        *build_low_profile_base().children,
        _keyboard_drive_tongue(state),
        _floating_drive_fork(state),
        *build_belt_system(travel).children,
        *build_cartridge(travel).children,
    ], label=f"v4_pose_{travel:.2f}")
```

- [ ] **Step 5: Implement focused interference classification**

Check only changed interfaces: tray/tongue vs housing, fork vs entry guide, belt vs reel lands/housing, swing arm vs cap, moving guides vs base, and support feet vs receivers. Maintain an explicit allowlist for designed contacts such as belt-tooth/groove occupancy and dovetail face contact; never suppress an entire component pair without a named contact purpose.

- [ ] **Step 6: Run all source-level tests**

```bash
python3 -m pytest -q \
  test_v4_kinematics.py \
  test_v4_reel_cartridge.py \
  test_v4_screwless_mounts.py \
  test_v4_integration.py
```

Expected: all tests pass at keyboard, midpoint, and trackpad poses; no removed V3 transmission label survives.

- [ ] **Step 7: Commit the composed assembly**

```bash
git add outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1
git commit -m $'Compose V4 keyboard trackpad mechanism\n\nCo-Authored-By: Riff'
```

## Task 6: Generate and inspect the five STEP deliverables

**Files:**

- Generate: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/keyboard_mode.step`
- Generate: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/mid_mode.step`
- Generate: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/trackpad_mode.step`
- Generate: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/reel_cartridge_section_exploded.step`
- Generate: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/screwless_guide_mount_detail.step`
- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/test_v4_step_outputs.py`

- [ ] **Step 1: Add failing STEP reload/label tests**

The test must skip only when files are absent before generation; once any STEP exists, absence or missing labels must fail. Reload each STEP through build123d/OCP and verify non-zero solids, valid bounding boxes, and required assembly-specific labels.

- [ ] **Step 2: Run the CAD preflight**

```bash
python3 -c 'import build123d; print(build123d.__version__)'
python3 /Users/bytedance/.agents/skills/cad/scripts/step --help
```

Expected: both commands exit `0`. If the repository launcher reports an interpreter/API mismatch, use the CAD skill's configured compatible Python runtime before changing model code.

- [ ] **Step 3: Generate every STEP through the CAD launcher**

From the repository root:

```bash
python3 /Users/bytedance/.agents/skills/cad/scripts/step \
  outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/keyboard_mode.py
python3 /Users/bytedance/.agents/skills/cad/scripts/step \
  outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/mid_mode.py
python3 /Users/bytedance/.agents/skills/cad/scripts/step \
  outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/trackpad_mode.py
python3 /Users/bytedance/.agents/skills/cad/scripts/step \
  outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/reel_cartridge_section_exploded.py
python3 /Users/bytedance/.agents/skills/cad/scripts/step \
  outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/screwless_guide_mount_detail.py
```

Expected: five same-stem `.step` files are written in the V4 directory.

- [ ] **Step 4: Inspect references, facts, planes, and positioning**

Run for each generated file; this example is the main keyboard pose:

```bash
python3 /Users/bytedance/.agents/skills/cad/scripts/inspect refs \
  outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/keyboard_mode.step \
  --facts --planes --positioning
```

Expected: valid solids, `310 × 225 mm` base envelope, table-to-base-top target near `8.5 mm`, all return hardware above the base underside, and selectable labels for the changed parts. Repeat for the other four files and correct any missing/merged entity.

- [ ] **Step 5: Run STEP reload tests**

```bash
cd outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1
python3 -m pytest -q test_v4_step_outputs.py
```

Expected: all five STEP files reload successfully and expose their required labels.

- [ ] **Step 6: Commit generated STEP outputs**

```bash
git add outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1
git commit -m $'Export and validate V4 STEP deliverables\n\nCo-Authored-By: Riff'
```

## Task 7: Produce the compact visual review packet and handoff brief

**Files:**

- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/CAD_BRIEF.md`
- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/render-keyboard-job.json`
- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/render-trackpad-job.json`
- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/render-cartridge-job.json`
- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/render-review/keyboard_iso.png`
- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/render-review/keyboard_low_side.png`
- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/render-review/trackpad_iso.png`
- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/render-review/cartridge_section_iso.png`
- Create: `outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/render-review/mount_detail_iso.png`

- [ ] **Step 1: Write the concise CAD brief**

Record confirmed geometry, provisional assumptions, component labels, how the 20.5 mm feed produces 90°, how the floating fork holds belt Z, belt replacement sequence, support removal sequence, and the exact acceptance boundary. Explicitly state that actual belt product, spring rate/preload, endurance, tolerances, strength, and print production remain unvalidated.

- [ ] **Step 2: Render the minimum useful views**

Use the render skill's snapshot job format and generate:

- keyboard isometric: overall architecture and separate belt;
- low side view: table height and absence of underside hardware;
- trackpad isometric: final tray target and arm clearance;
- cartridge section: spring, two bearing seats, reel, shaft, wedges, cap, and stops;
- mount detail: insertion direction, dovetail faces, end wall, and release tab.

Run each job with:

```bash
python3 /Users/bytedance/.agents/skills/render/scripts/snapshot --job \
  outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/render-keyboard-job.json
```

Expected: each PNG exists, is non-empty, and clearly shows the intended interface. Repeat for the trackpad and cartridge job files.

- [ ] **Step 3: Perform a visual review against the spec**

Confirm from the images:

- base reads as restrained rounded open frame rather than a slab;
- no cylinder, spring cup, shaft, or cover hangs below the structural underside;
- keyboard tray does not carry a long rack or fence;
- belt enters the cartridge at fixed Z without housing collision;
- reel stack is understandable and physically supported;
- supports slide inward transversely and are retained without screws.

If a view exposes a real modeling problem, add a focused failing test before changing geometry, regenerate only affected STEP files, and rerender only affected views.

- [ ] **Step 4: Run the complete focused suite and repository checks**

```bash
python3 -m pytest -q outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1
git diff --check
git status --short
```

Expected: tests pass, no whitespace errors, and only intentional V4 files are modified/untracked.

- [ ] **Step 5: Start CAD Explorer at the keyboard pose**

```bash
npm --prefix /Users/bytedance/.agents/skills/render/scripts/viewer run dev:ensure -- \
  --workspace-root /Users/bytedance/Documents/Codex/2026-07-15/keyboard-trackpad-cross-axis-redesign/.worktrees/v3-open-frame-base \
  --root-dir outputs \
  --file outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1/keyboard_mode.step
```

Expected: the command returns a local Explorer URL that opens the V4 keyboard pose. Keep the direct STEP paths in the handoff in case the server later expires.

- [ ] **Step 6: Commit the review packet**

```bash
git add outputs/rack-pinion-swing-tray-main-v4-low-profile-reel-cartridge-v1
git commit -m $'Document and render V4 concept review\n\nCo-Authored-By: Riff'
```

- [ ] **Step 7: Push without expanding scope**

```bash
git push origin agent/v3-open-frame-base
```

Expected: the existing public PR updates with the V4 plan and concept outputs. Stop for user visual confirmation; do not begin production engineering work.

## Final implementation acceptance checklist

- [ ] The V4 directory is new and all earlier outputs remain unchanged.
- [ ] `20.50 mm` belt feed maps to `90°` reel motion.
- [ ] The keyboard travels about `20.42 mm` forward and `15 mm` down while belt Z remains fixed.
- [ ] A purchased open-ended `2M`, approximately `12 mm` belt envelope is represented honestly.
- [ ] Both removable printed wedges capture at least four teeth and cannot escape under working load.
- [ ] The reel retains at least five engaged teeth without positive penetration.
- [ ] Lower thrust/radial support, upper radial support, shaft, spring anchors, and independent stops are real geometry.
- [ ] All return hardware is above the structural base underside.
- [ ] The base is `310 × 225 mm`, visually rounded, open, and materially lower than V3.
- [ ] Four guide supports use inward transverse dovetails, end datums, and non-load-bearing snap retention.
- [ ] Keyboard tray, tongue, fork, slider, belt, reel, cartridge parts, arm, and trackpad tray are separately selectable.
- [ ] Three assembly poses plus cartridge and mount details reload as valid STEP.
- [ ] The review packet makes table height, belt entry, cartridge internals, and mount direction immediately legible.
- [ ] Work stops after visual handoff for user confirmation.
