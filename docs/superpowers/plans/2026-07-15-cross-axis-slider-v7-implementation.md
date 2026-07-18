# Cross-axis Slider V7 Low-lift Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generate a new v7 CAD concept that lowers the final trackpad working surface to Z=28.5 mm, uses a constraint-derived +10.929488° bilateral slot, and includes spring-protecting terminal stop cores.

**Architecture:** Import the validated v6 source as the geometry basis but override all trackpad-dependent motion and linkage calculations in a new v7 module. Generate keyboard, mid, and trackpad poses from one normalized travel value, with deterministic checks for slot closure, terminal planes, stop-core geometry, and moving-link clearance.

**Tech Stack:** Python 3.14, build123d 0.11.1, CAD STEP/inspection launchers, CAD Explorer render tools.

---

### Task 1: Create the v7 parameter and geometry layer

**Files:**
- Create: `outputs/cross-axis-slider-concept-v7-low-lift-stop/cross_axis_slider_concept_v7.py`
- Create: `outputs/cross-axis-slider-concept-v7-low-lift-stop/keyboard_mode.py`
- Create: `outputs/cross-axis-slider-concept-v7-low-lift-stop/mid_mode.py`
- Create: `outputs/cross-axis-slider-concept-v7-low-lift-stop/trackpad_mode.py`
- Create: `outputs/cross-axis-slider-concept-v7-low-lift-stop/CAD_BRIEF.md`

- [ ] **Step 1: Define exact motion constants**

```python
TRACKPAD_DZ_V7 = -43.0
RELATIVE_DY = KEYBOARD_DY - TRACKPAD_DY
RELATIVE_DZ = KEYBOARD_DZ - TRACKPAD_DZ_V7
SLOT_ANGLE_DEG = degrees(atan2(RELATIVE_DZ, RELATIVE_DY))
PIN_Z_OFFSET = 37.0
```

- [ ] **Step 2: Rebuild both captured linkage sides from v7 slot geometry**

Reuse the v6 X stacks, colors, rollers, shafts, spacers, ties, guides, shoes, connectors, and springs. Raise both crossover axes to `keyboard_z + 37 mm`, rebuild the taller red drivers, and recreate both blue cheek pairs using the v7 slot center and angle so no v6 -3° linkage remains.

- [ ] **Step 3: Add the two terminal cores**

```python
STOP_CORE_LENGTH = 10.0
STOP_CORE_DIAMETER = 1.8
core_start = fixed_seat_point
core_end = (
    fixed_y - STOP_CORE_LENGTH * KEYBOARD_UY,
    fixed_z - STOP_CORE_LENGTH * KEYBOARD_UZ,
)
```

Create one fixed beam/cylinder along each keyboard guide axis, inside its spring, with explicit datum/floating labels.

- [ ] **Step 4: Assemble three pose generators**

Filter inherited v1/v6 linkage children, regenerate fixed trackpad guides with -43 mm Z travel, retain the compact base and v6 keyboard spring guides, then add v7 bilateral linkage and stop cores.

### Task 2: Validate motion, heights, and clearance

**Files:**
- Modify: `outputs/cross-axis-slider-concept-v7-low-lift-stop/cross_axis_slider_concept_v7.py`

- [ ] **Step 1: Implement the normalized kinematic sample**

```python
def kinematic_sample(travel):
    t = max(0.0, min(1.0, float(travel)))
    key_y = KEYBOARD_Y0 + t * KEYBOARD_DY
    key_z = KEYBOARD_Z0 + t * KEYBOARD_DZ
    track_y = TRACKPAD_Y0 + t * TRACKPAD_DY
    track_z = TRACKPAD_Z0 + t * TRACKPAD_DZ_V7
    pin_y, pin_z, slot_y, slot_z = _slot_geometry(key_y, key_z, track_y, track_z)
    rel_len = hypot(RELATIVE_DY, RELATIVE_DZ)
    along = ((pin_y - slot_y) * RELATIVE_DY + (pin_z - slot_z) * RELATIVE_DZ) / rel_len
    cross = ((pin_y - slot_y) * RELATIVE_DZ - (pin_z - slot_z) * RELATIVE_DY) / rel_len
    margin = CAPTURED_SLOT_LENGTH / 2 - ROLLER_OD / 2 - abs(along)
    return {"travel": t, "along": along, "cross": cross, "margin": margin}
```

- [ ] **Step 2: Sample 201 positions**

Assert slot cross error below 0.001 mm, positive endpoint reserve, bilateral monotonic travel, and exact +10.929488° slot angle.

- [ ] **Step 3: Validate terminal planes and stop cores**

At `travel=1`, assert keyboard top Z=10.5 mm, trackpad bottom/top Z=25.0/28.5 mm, spring envelope=10.0 mm, two stop cores exist, and obsolete three-hole anchors do not exist.

- [ ] **Step 4: Measure moving-link clearance**

For keyboard, mid, and trackpad poses, calculate the minimum Z of all moving linkage children while excluding the base and fixed supports. Require minimum Z at least 6.0 mm, which is 2.0 mm above the base top plane. Stop without export if this assertion fails.

### Task 3: Generate, inspect, and render STEP files

**Files:**
- Generate: `outputs/cross-axis-slider-concept-v7-low-lift-stop/keyboard_mode.step`
- Generate: `outputs/cross-axis-slider-concept-v7-low-lift-stop/mid_mode.step`
- Generate: `outputs/cross-axis-slider-concept-v7-low-lift-stop/trackpad_mode.step`

- [ ] **Step 1: Generate all three sources**

```bash
work/cad-venv314/bin/python /Users/bytedance/.agents/skills/cad/scripts/step \
  outputs/cross-axis-slider-concept-v7-low-lift-stop/keyboard_mode.py \
  outputs/cross-axis-slider-concept-v7-low-lift-stop/mid_mode.py \
  outputs/cross-axis-slider-concept-v7-low-lift-stop/trackpad_mode.py
```

- [ ] **Step 2: Inspect facts, planes, and positioning**

Run `scripts/inspect refs <pose.step> --facts --planes --positioning` for each file. Confirm non-empty labeled solids, exact terminal tray planes, and no warnings.

- [ ] **Step 3: Start or reuse CAD Explorer and render diagnostics**

Return live links for all three poses. Render left-side and isometric stills for each pose and visually check the low terminal height, positive slot, bilateral symmetry, terminal cores, and base clearance.

- [ ] **Step 4: Stop at manual review**

Report only verified concept results. Do not create STL, 3MF, BOM, ZIP, or manufacturing outputs.
