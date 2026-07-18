# V3 Open-Frame Base Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the integrated-return V3 continuous plate with a `310 × 225 × 6 mm` FDM open frame while preserving every existing mechanism datum and interface.

**Architecture:** Create a new derivative module that imports the passing integrated-return V1 assembly and replaces only its base and clearance feet. Construct the base from perimeter rails, cross-members, guide islands, and a right-side mechanism spine, then subtract the cartridge flange recess, fastener paths, and foot pockets.

**Tech Stack:** Python 3, build123d, unittest, CAD STEP/inspection launchers, CAD Explorer.

---

### Task 1: Add focused failing tests

**Files:**
- Create: `outputs/rack-pinion-swing-tray-main-v3-open-frame-base-v1/test_open_frame_base.py`

- [ ] **Step 1: Test the new envelope and unchanged top datum**

```python
base = concept.make_open_frame_base()
box = base.bounding_box()
self.assertAlmostEqual(box.size.X, 310.0, places=5)
self.assertAlmostEqual(box.size.Y, 225.0, places=5)
self.assertAlmostEqual(box.min.Z, 1.0, places=5)
self.assertAlmostEqual(box.max.Z, 7.0, places=5)
```

- [ ] **Step 2: Test supported mounting probes and selectable labels**

Require positive base material beneath each guide-foot mounting centre, the fixed cylinder footprint, rack-retention bracket root, and cartridge bolt land. Require `open_frame_base_310x225`, `primary_tpu_feet_4x`, and `auxiliary_tpu_pads_2x` in all three poses.

- [ ] **Step 3: Test recesses, feet, and table clearance**

Assert no positive-volume overlap between the base and cartridge flange, primary feet, or auxiliary pads. Assert the four primary feet share `Z=-15 mm`; auxiliary pads stop at `Z=-14.7 mm`; the cartridge bottom cover remains at least `2 mm` above the primary table plane.

- [ ] **Step 4: Test only base-related interferences**

For keyboard, midpoint, and trackpad poses, check the new base against the moving trays, rack, pinion, swing arm, cartridge, retention rollers, and flexure. Intentional seating contacts remain zero-volume contacts.

- [ ] **Step 5: Run the test and observe the missing-module failure**

Run from the new output directory:

```bash
python -m unittest test_open_frame_base.py -v
```

Expected: failure because `open_frame_base_v1` does not exist.

### Task 2: Implement the base derivative

**Files:**
- Create: `outputs/rack-pinion-swing-tray-main-v3-open-frame-base-v1/open_frame_base_v1.py`
- Create: `outputs/rack-pinion-swing-tray-main-v3-open-frame-base-v1/keyboard_mode.py`
- Create: `outputs/rack-pinion-swing-tray-main-v3-open-frame-base-v1/mid_mode.py`
- Create: `outputs/rack-pinion-swing-tray-main-v3-open-frame-base-v1/trackpad_mode.py`
- Create: `outputs/rack-pinion-swing-tray-main-v3-open-frame-base-v1/base_and_return_interface.py`
- Create: `outputs/rack-pinion-swing-tray-main-v3-open-frame-base-v1/CAD_BRIEF.md`

- [ ] **Step 1: Build named frame solids**

Use `BASE_X=(-155,155)`, `BASE_Y=(-55,170)`, `BASE_Z=(1,7)`, `RAIL_WIDTH=12`, and three transverse cross-members. Add full guide islands and a widened right mechanism spine that contains the cylinder, retention bracket, and cartridge bolt circle.

- [ ] **Step 2: Cut physical underside interfaces**

Subtract a `2 mm`-deep flange recess ending at `Z=3`, the shaft path, three cartridge fastener paths, and six shallow foot pockets. Keep the base as one valid selectable solid.

- [ ] **Step 3: Add retained TPU feet**

Create four primary feet from `Z=-15…3` and two auxiliary pads from `Z=-14.7…3`. Their upper plugs occupy the matching pockets without overlapping the base solid.

- [ ] **Step 4: Replace only base and feet in the imported assembly**

Call the integrated-return V1 pose builder, replace `continuous_main_base_integrated_return_drilled` and `integrated_return_clearance_feet`, and preserve every other child unchanged.

- [ ] **Step 5: Implement pose and diagnostic generators**

Generate `t=0`, `t=0.5`, `t=1`, and a base/cartridge interface diagnostic with the base, feet, cup, cover, shaft, fixed cylinder, guide feet, and retention bracket.

- [ ] **Step 6: Run the focused test**

Expected: all new tests pass. Do not run historical mechanism suites.

### Task 3: Generate and review STEP artifacts

**Files:**
- Create: `outputs/rack-pinion-swing-tray-main-v3-open-frame-base-v1/keyboard_mode.step`
- Create: `outputs/rack-pinion-swing-tray-main-v3-open-frame-base-v1/mid_mode.step`
- Create: `outputs/rack-pinion-swing-tray-main-v3-open-frame-base-v1/trackpad_mode.step`
- Create: `outputs/rack-pinion-swing-tray-main-v3-open-frame-base-v1/base_and_return_interface.step`

- [ ] **Step 1: Generate the four explicit Python targets**

Use the CAD STEP launcher without directory-wide generation and without skipping Explorer sidecars.

- [ ] **Step 2: Inspect validity and labels**

Reload every STEP, report solid counts and bounding boxes, and confirm the new base/foot labels survive export.

- [ ] **Step 3: Create a compact visual packet**

Capture keyboard isometric, base top, base bottom, and base/cartridge interface views. Convert any visual concern into a geometry check before changing source.

- [ ] **Step 4: Start or reuse CAD Explorer**

Return the viewer link for the new keyboard-mode STEP.

### Task 4: Publish the isolated change

**Files:**
- Track only: `docs/superpowers/plans/2026-07-18-v3-open-frame-base.md` and `outputs/rack-pinion-swing-tray-main-v3-open-frame-base-v1/`

- [ ] **Step 1: Verify focused tests and STEP reloads once more**

- [ ] **Step 2: Commit with the required trailer**

Every commit message must end with:

```text
Co-Authored-By: Riff
```

- [ ] **Step 3: Push a feature branch and create a Draft PR**

The PR description must also end with the literal `Co-Authored-By: Riff` trailer.
