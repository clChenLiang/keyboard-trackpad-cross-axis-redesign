# V3 Integrated Return and Rack Retention Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce a new STEP-first V3 derivative with rack back-pressure rollers and the coaxial torsion-return drum integrated into the standard assembly.

**Architecture:** Preserve all existing V3 source and generated files. A new output module imports the passing V3 assembly and torsion cartridge, replaces only the base/shaft/feet required by the cartridge, and adds a fixed roller bracket, one datum roller, and one flexure-preloaded roller beside the rack/pinion mesh.

**Tech Stack:** Python 3, build123d, unittest, CAD skill STEP/inspection launchers, CAD Explorer.

---

### Task 1: Create focused integration tests

**Files:**
- Create: `outputs/rack-pinion-swing-tray-main-v3-integrated-return-v1/test_integrated_return.py`

- [ ] **Step 1: Write tests that identify every new selectable entity**

The test must require these labels in keyboard, mid, and trackpad poses:

```python
REQUIRED = {
    "rack_retention_fixed_bracket",
    "rack_back_pressure_datum_roller",
    "rack_back_pressure_preload_roller",
    "rack_back_pressure_flexure",
    "torsion_fixed_mounting_cup",
    "torsion_removable_bottom_cover",
    "torsion_helical_spring",
    "torsion_rotating_shaft_clamp",
    "torsion_fixed_end_anchor_pin",
    "torsion_shaft_drive_key",
    "vertical_drive_shaft_integrated_return",
    "integrated_return_clearance_feet",
}
for t in (0.0, 0.5, 1.0):
    labels = {child.label for child in concept.build_pose(t).children}
    self.assertTrue(REQUIRED <= labels)
```

- [ ] **Step 2: Add only revision-focused geometry checks**

For each endpoint and midpoint, assert valid solids; roller-to-rack-back distance at or below the modeled preload allowance; no positive-volume intersections between rollers/bracket/flexure and rack teeth, pinion, cylinder, base, tray, or rack transition. Reuse the already passing torsion tests for cup access, spring-cover clearance, keyed coupling, and two-millimetre table clearance.

- [ ] **Step 3: Run the focused test and confirm it fails because the new module does not exist**

Run:

```bash
python -m unittest outputs/rack-pinion-swing-tray-main-v3-integrated-return-v1/test_integrated_return.py -v
```

Expected: import failure for `integrated_return_v1`.

### Task 2: Build the integrated parametric assembly

**Files:**
- Create: `outputs/rack-pinion-swing-tray-main-v3-integrated-return-v1/integrated_return_v1.py`
- Create: `outputs/rack-pinion-swing-tray-main-v3-integrated-return-v1/keyboard_mode.py`
- Create: `outputs/rack-pinion-swing-tray-main-v3-integrated-return-v1/mid_mode.py`
- Create: `outputs/rack-pinion-swing-tray-main-v3-integrated-return-v1/trackpad_mode.py`
- Create: `outputs/rack-pinion-swing-tray-main-v3-integrated-return-v1/cylinder_return_section_exploded.py`

- [ ] **Step 1: Import, do not edit, the existing V3 and torsion sources**

Load both modules by absolute path derived from `Path(__file__).resolve().parent.parent`. Keep the earlier output directories untouched.

- [ ] **Step 2: Add the rack-retention datums and geometry**

Use the rack travel vector `(0, KEYBOARD_TRAVEL_Y, KEYBOARD_TRAVEL_Z)` and a roller-axis vector perpendicular to it in the YZ plane. Place the datum roller against the rack back face opposite the pinion. Place a second roller along the mesh span on a short fixed-root flexure. Keep the nominal concept preload between `0.15` and `0.30 mm`; represent compliance geometrically without claiming a spring rate.

- [ ] **Step 3: Merge the torsion cartridge into the standard pose builder**

Start with `v3.build_pose(t, ...)`, replace the original base and shaft with the drilled base and keyed extended shaft, add clearance feet, then append every cartridge entity and rack-retention entity as separately labeled children. The spring direction must bias `t=1` toward `t=0`.

- [ ] **Step 4: Implement the four generators**

The three pose files call `build_pose(0.0)`, `build_pose(0.5)`, and `build_pose(1.0)`. The exploded generator offsets the cover, spring, clamp, key, anchor, rollers, and bracket while retaining their labels.

- [ ] **Step 5: Run only the focused test**

Run the Task 1 unittest command. Expected: all tests pass.

### Task 3: Generate and inspect the four STEP outputs

**Files:**
- Create: `outputs/rack-pinion-swing-tray-main-v3-integrated-return-v1/keyboard_mode.step`
- Create: `outputs/rack-pinion-swing-tray-main-v3-integrated-return-v1/mid_mode.step`
- Create: `outputs/rack-pinion-swing-tray-main-v3-integrated-return-v1/trackpad_mode.step`
- Create: `outputs/rack-pinion-swing-tray-main-v3-integrated-return-v1/cylinder_return_section_exploded.step`

- [ ] **Step 1: Generate each explicit Python target through the CAD STEP launcher**

Do not run directory-wide generation and do not skip Explorer sidecars.

- [ ] **Step 2: Inspect all four STEP files**

Confirm each file loads, contains positive-volume solids, and exposes the required labels. Do not rerun unrelated historical V3 tests.

- [ ] **Step 3: Create one compact visual review packet**

Capture keyboard isometric, trackpad isometric, rack/pinion close-up, and return-drum section/exploded views.

### Task 4: Publish the workspace and hand off the viewer

**Files:**
- Create: `.gitignore`
- Track: source, design/plan documents, STEP outputs, and review images that fit GitHub limits.

- [ ] **Step 1: Audit before public publication**

Scan filenames and text for credentials, tokens, private keys, machine-specific secrets, and files at or above GitHub's per-file limit. Exclude caches, virtual environments, CAD Explorer runtime files, and OS metadata.

- [ ] **Step 2: Initialize Git and create the public repository**

Use repository name `keyboard-trackpad-cross-axis-redesign` in the currently authenticated GitHub account. Every commit message must end with:

```text
Co-Authored-By: Riff
```

- [ ] **Step 3: Push the initial branch**

Verify the remote repository is public and the required commit trailer is present before pushing.

- [ ] **Step 4: Start or reuse CAD Explorer**

Open the new `keyboard_mode.step` and return its localhost review link, direct STEP paths, focused validation result, and public GitHub repository URL.
