# Cross-axis Slider V6 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and validate an independent v6 CAD concept with a -3° bilateral captured linkage, symmetric compression-spring return geometry, floating right-side stacks, and no obsolete three-hole anchors.

**Architecture:** Reuse the v3 compact-base geometry through imports but derive all v6 motion and linkage geometry in a new source module. Generate three static STEP poses from one normalized travel variable; calculate trays, rollers, slots, guide shoes, and spring length from that single variable.

**Tech Stack:** Python 3.14, build123d 0.11.1, CAD skill STEP/inspection launchers, CAD Explorer render tools.

---

### Task 1: Create the v6 parametric source

**Files:**
- Create: `outputs/cross-axis-slider-concept-v6-symmetric-spring/cross_axis_slider_concept_v6.py`
- Create: `outputs/cross-axis-slider-concept-v6-symmetric-spring/keyboard_mode.py`
- Create: `outputs/cross-axis-slider-concept-v6-symmetric-spring/mid_mode.py`
- Create: `outputs/cross-axis-slider-concept-v6-symmetric-spring/trackpad_mode.py`
- Create: `outputs/cross-axis-slider-concept-v6-symmetric-spring/CAD_BRIEF.md`

- [ ] **Step 1: Define constraint-derived motion constants**

```python
SLOT_ANGLE_DEG = -3.0
RELATIVE_DY = KEYBOARD_DY - TRACKPAD_DY
RELATIVE_DZ = RELATIVE_DY * tan(radians(SLOT_ANGLE_DEG))
TRACKPAD_DZ_V6 = KEYBOARD_DZ - RELATIVE_DZ
```

- [ ] **Step 2: Build mirrored active linkage stacks**

```python
for side_name, sign in (("left_datum", -1), ("right_floating", 1)):
    axial_float = 0.0 if sign < 0 else 0.7
    children.extend(_captured_linkage_side(
        side_name, sign, axial_float, key_y, key_z, track_y, track_z
    ))
```

Each side must contain a red two-point driver, two blue slot cheeks, two gold roller rings, a shoulder shaft, spacers, and two trackpad ties. Right-side cheek/roller spacing must visibly include the 0.7 mm axial float without changing YZ kinematics.

- [ ] **Step 3: Build captured keyboard guide shoes**

```python
shoe_widths = {
    "left_datum": GUIDE_SLOT_WIDTH - 0.8,
    "right_floating": GUIDE_SLOT_WIDTH - 1.4,
}
```

Give each shoe explicit overlap with its keyboard side carriage. Keep the left shoe as datum and increase only the right shoe's X/slot assembly freedom.

- [ ] **Step 4: Build symmetric spring return geometry**

```python
KEYBOARD_PATH = hypot(KEYBOARD_DY, KEYBOARD_DZ)
SPRING_FREE_LENGTH = 43.54101966249684
spring_length = SPRING_FREE_LENGTH - travel * KEYBOARD_PATH
```

Create two extended guide slots parallel to keyboard travel, fixed forward/lower seats, rigid shoe-to-carriage connectors, and simplified seven-turn helical spring solids whose length decreases monotonically from 43.541 mm to 10.000 mm without crossing below the base plane.

- [ ] **Step 5: Remove obsolete v1 parts and assemble the pose**

```python
obsolete_prefixes = (
    "keyboard_drive_arm_",
    "trackpad_slotted_link_",
    "cross_axis_roller_pin_",
    "keyboard_guide_roller_",
    "return_anchor_3x5mm_",
)
```

Return a labeled `Compound` for each pose and preserve the v3 compact open base.

### Task 2: Add deterministic mechanism validation

**Files:**
- Modify: `outputs/cross-axis-slider-concept-v6-symmetric-spring/cross_axis_slider_concept_v6.py`

- [ ] **Step 1: Implement one normalized kinematic sample**

```python
def kinematic_sample(travel):
    t = max(0.0, min(1.0, float(travel)))
    key_y = KEYBOARD_Y0 + t * KEYBOARD_DY
    key_z = KEYBOARD_Z0 + t * KEYBOARD_DZ
    track_y = TRACKPAD_Y0 + t * TRACKPAD_DY
    track_z = TRACKPAD_Z0 + t * TRACKPAD_DZ_V6
    pin_y, pin_z, slot_y, slot_z = _slot_geometry(key_y, key_z, track_y, track_z)
    rel_len = hypot(RELATIVE_DY, RELATIVE_DZ)
    along = ((pin_y - slot_y) * RELATIVE_DY
             + (pin_z - slot_z) * RELATIVE_DZ) / rel_len
    cross = ((pin_y - slot_y) * RELATIVE_DZ
             - (pin_z - slot_z) * RELATIVE_DY) / rel_len
    margin = CAPTURED_SLOT_LENGTH / 2.0 - ROLLER_OD / 2.0 - abs(along)
    return {
        "travel": t,
        "along": along,
        "cross": cross,
        "margin": margin,
        "spring_length": SPRING_FREE_LENGTH - t * KEYBOARD_PATH,
    }
```

- [ ] **Step 2: Implement the 201-position validation summary**

```python
def validate_kinematics(sample_count=201):
    samples = [kinematic_sample(i / (sample_count - 1))
               for i in range(sample_count)]
    return {
        "max_cross_error": max(abs(s["cross"]) for s in samples),
        "minimum_roller_center_margin": min(s["margin"] for s in samples),
        "roller_monotonic": all(samples[i + 1]["along"] > samples[i]["along"]
                                for i in range(len(samples) - 1)),
        "spring_monotonic": all(samples[i + 1]["spring_length"] < samples[i]["spring_length"]
                                for i in range(len(samples) - 1)),
    }
```

- [ ] **Step 3: Run source validation**

Run:

```bash
work/cad-venv314/bin/python -m py_compile outputs/cross-axis-slider-concept-v6-symmetric-spring/*.py
work/cad-venv314/bin/python -c 'import sys; sys.path.insert(0,"outputs/cross-axis-slider-concept-v6-symmetric-spring"); import cross_axis_slider_concept_v6 as v6; print(v6.validate_kinematics())'
```

Expected: 201 samples, -3° slot, `TRACKPAD_DZ=-7.400872003959025`, cross error below 0.001 mm, positive endpoint margin, monotonic roller travel, and monotonic spring compression.

### Task 3: Generate and inspect three STEP poses

**Files:**
- Generate: `outputs/cross-axis-slider-concept-v6-symmetric-spring/keyboard_mode.step`
- Generate: `outputs/cross-axis-slider-concept-v6-symmetric-spring/mid_mode.step`
- Generate: `outputs/cross-axis-slider-concept-v6-symmetric-spring/trackpad_mode.step`

- [ ] **Step 1: Generate explicit sources**

```bash
work/cad-venv314/bin/python /Users/bytedance/.agents/skills/cad/scripts/step \
  outputs/cross-axis-slider-concept-v6-symmetric-spring/keyboard_mode.py \
  outputs/cross-axis-slider-concept-v6-symmetric-spring/mid_mode.py \
  outputs/cross-axis-slider-concept-v6-symmetric-spring/trackpad_mode.py
```

Expected: three successful `generated part STEP` messages.

- [ ] **Step 2: Inspect each STEP**

```bash
work/cad-venv314/bin/python /Users/bytedance/.agents/skills/cad/scripts/inspect refs <pose.step> --facts --planes --positioning
```

Expected: non-empty shape/face counts, 308 × 280 mm base envelope in keyboard mode, no warnings, and correct tray Z planes.

- [ ] **Step 3: Verify labels and phase scope**

Confirm both datum/floating active stacks and both spring assemblies are present, while `return_anchor_3x5mm_*` is absent. Confirm no STL, 3MF, BOM, ZIP, or manufacturing artifacts were created.

### Task 4: Render and hand off for manual review

**Files:**
- Review: all three v6 STEP files

- [ ] **Step 1: Start or reuse CAD Explorer**

```bash
npm --prefix /Users/bytedance/.agents/skills/render/scripts/viewer run dev:ensure -- \
  --workspace-root /Users/bytedance/Documents/Codex/2026-07-15/keyboard-trackpad-cross-axis-redesign \
  --file outputs/cross-axis-slider-concept-v6-symmetric-spring/<pose>.step
```

Expected: a working Explorer URL on the reused local server for each pose.

- [ ] **Step 2: Render a diagnostic packet**

Create left-side and isometric stills for keyboard, mid, and trackpad modes. Check slot slope, bilateral symmetry, spring compression, carriage attachment, and removal of three-hole tabs.

- [ ] **Step 3: Stop at the manual-review boundary**

Return the three links, source directory, actual validation results, and concept-stage caveats. Do not perform final interference, tolerance, strength, manufacturing, or print-package work.
