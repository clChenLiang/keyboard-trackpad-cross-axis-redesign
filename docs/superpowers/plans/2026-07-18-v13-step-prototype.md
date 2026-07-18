# V13 Under-100 STEP Prototype Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an independent, reviewable V13 STEP prototype that proves the four-bearing cross-slot linkage, printed shoe guide paths, spring mounting geometry, and three valid end/intermediate poses without developing print-production details.

**Architecture:** A small build123d package in `outputs/cross-axis-slider-v13-under-100/` separates named parameters, analytical motion constraints, printable concept solids, and pose assembly. The red keyboard and blue trackpad trajectories remain affine; two MR84ZZ rollers per side occupy a real blue long slot, while broad printed shoe/rail pairs replace the eight V12 guide bearings. Static pose generators export keyboard, midpoint, trackpad, and master STEP assemblies.

**Tech Stack:** Python 3.9, build123d, unittest, STEP export, CAD Explorer.

---

The workspace is not a Git repository, so the plan uses test and artifact checkpoints instead of commits.

### Task 1: Parameters and analytical motion tests

**Files:**
- Create: `outputs/cross-axis-slider-v13-under-100/parameters.py`
- Create: `outputs/cross-axis-slider-v13-under-100/mechanism.py`
- Create: `outputs/cross-axis-slider-v13-under-100/test_mechanism.py`

- [ ] **Step 1: Write failing tests for three poses, slot closure, end margin, follower float, and spring window**

```python
import unittest
from mechanism import pose, validate_motion, validate_bilateral_parallelism, validate_spring_window

class MotionTests(unittest.TestCase):
    def test_three_pose_endpoints(self):
        self.assertEqual((30.0, -17.0), pose(1.0).keyboard_delta)
        self.assertEqual((-115.0, -40.0), pose(1.0).trackpad_delta)
        self.assertEqual(0.5, pose(0.5).travel)

    def test_cross_slot_is_real_and_monotonic(self):
        report = validate_motion(401)
        self.assertLess(report["max_cross_error_mm"], 0.01)
        self.assertGreaterEqual(report["min_cross_end_margin_mm"], 2.0)
        self.assertTrue(report["monotonic"])

    def test_right_side_is_follower(self):
        report = validate_bilateral_parallelism(401)
        self.assertEqual("left_datum_right_follower", report["strategy"])
        self.assertGreater(report["normal_float_margin_mm"], 0.0)

    def test_extension_spring_window(self):
        report = validate_spring_window()
        self.assertGreaterEqual(report["start_total_force_n"], 3.0)
        self.assertLessEqual(report["start_total_force_n"], 5.0)
        self.assertLessEqual(report["end_total_force_n"], 8.0)
        self.assertGreaterEqual(report["stroke_reserve_ratio"], 0.20)
```

- [ ] **Step 2: Run the tests and verify import failure**

Run: `cd outputs/cross-axis-slider-v13-under-100 && python3 -m unittest -v test_mechanism.py`  
Expected: FAIL because `mechanism.py` does not exist.

- [ ] **Step 3: Add named V13 parameters**

Define the existing device/tray envelopes, the V12-proven endpoint deltas, a 178–184 mm cross slot, 18 mm cross-roller spacing, 0.8 mm right follower reserve, 14 mm shoe length, 0.35 mm shoe running clearance, and a provisional two-spring model with 2 N start force per side and no more than 4 N end force per side. Keep all units in millimetres and newtons.

- [ ] **Step 4: Implement the affine pose and validation functions**

```python
@dataclass(frozen=True)
class Pose:
    travel: float
    keyboard_delta: tuple[float, float]
    trackpad_delta: tuple[float, float]
    cross_slot_center: tuple[float, float]
    cross_roller_centers: tuple[tuple[float, float], tuple[float, float]]

def pose(travel: float) -> Pose:
    t = max(0.0, min(1.0, float(travel)))
    keyboard_delta = (t * p.KEYBOARD_DY, t * p.KEYBOARD_DZ)
    trackpad_delta = (t * p.TRACKPAD_DY, t * p.TRACKPAD_DZ)
    # Derive slot direction from relative tray motion; never introduce a
    # telescoping rigid link.
    relative = (p.KEYBOARD_DY - p.TRACKPAD_DY,
                p.KEYBOARD_DZ - p.TRACKPAD_DZ)
    relative_length = hypot(*relative)
    slot_u = (relative[0] / relative_length, relative[1] / relative_length)
    slot_n = (-slot_u[1], slot_u[0])
    keyboard_position = (p.KEYBOARD_Y0 + keyboard_delta[0],
                         p.KEYBOARD_Z0 + keyboard_delta[1])
    yoke_center = (keyboard_position[0] + 30.0,
                   keyboard_position[1] + 37.0)
    initial_yoke = (p.KEYBOARD_Y0 + 30.0, p.KEYBOARD_Z0 + 37.0)
    initial_slot = (initial_yoke[0] + relative[0] / 2.0,
                    initial_yoke[1] + relative[1] / 2.0)
    slot_center = (initial_slot[0] + trackpad_delta[0],
                   initial_slot[1] + trackpad_delta[1])
    half = p.CROSS_ROLLER_SPACING / 2.0
    wall = (p.CROSS_SLOT_WIDTH - p.MR84ZZ.od) / 2.0
    rollers = (
        (yoke_center[0] - half * slot_u[0] + wall * slot_n[0],
         yoke_center[1] - half * slot_u[1] + wall * slot_n[1]),
        (yoke_center[0] + half * slot_u[0] - wall * slot_n[0],
         yoke_center[1] + half * slot_u[1] - wall * slot_n[1]),
    )
    return Pose(t, keyboard_delta, trackpad_delta, slot_center, rollers)
```

Implement `validate_motion(401)`, `validate_bilateral_parallelism(401)`, and `validate_spring_window()` with numerical reports matching the test keys.

- [ ] **Step 5: Run the motion tests**

Run: `cd outputs/cross-axis-slider-v13-under-100 && python3 -m unittest -v test_mechanism.py`  
Expected: 4 tests PASS.

### Task 2: Concept part factories

**Files:**
- Create: `outputs/cross-axis-slider-v13-under-100/parts.py`
- Create: `outputs/cross-axis-slider-v13-under-100/test_geometry.py`

- [ ] **Step 1: Write geometry tests**

```python
import unittest
import parts

class GeometryTests(unittest.TestCase):
    def test_primary_parts_are_positive_solids(self):
        shapes = [
            parts.make_base(), parts.make_tray("keyboard"),
            parts.make_tray("trackpad"), parts.make_side_frame("left"),
            parts.make_side_frame("right"), parts.make_carriage("keyboard", "left"),
            parts.make_carriage("trackpad", "left"), parts.make_guide_shoe(),
        ]
        for shape in shapes:
            self.assertGreater(shape.volume, 0.0)
            self.assertEqual(1, len(shape.solids()))

    def test_trackpad_carriage_has_true_long_slot(self):
        self.assertTrue(parts.cross_slot_clearance_report()["roller_fits"])
```

- [ ] **Step 2: Run tests and confirm failure**

Run: `cd outputs/cross-axis-slider-v13-under-100 && python3 -m unittest -v test_geometry.py`  
Expected: FAIL because `parts.py` does not exist.

- [ ] **Step 3: Implement simplified concept solids**

Create a compact open base, two dark fixed side frames with shallow shoe rails, complete continuous red/blue trays, red keyboard carriages with two reinforced printed axle bosses per side, blue trackpad carriages with actual elongated slots, four replaceable guide shoes, TPU stop blocks, and simple extension-spring envelopes. Do not model 604ZZ, shoulder screws, CFS drums, side guards, devices, or production fastener details.

- [ ] **Step 4: Run geometry tests**

Run: `cd outputs/cross-axis-slider-v13-under-100 && python3 -m unittest -v test_geometry.py`  
Expected: all tests PASS.

### Task 3: Three-pose labeled assemblies

**Files:**
- Create: `outputs/cross-axis-slider-v13-under-100/assembly.py`
- Create: `outputs/cross-axis-slider-v13-under-100/keyboard_mode.py`
- Create: `outputs/cross-axis-slider-v13-under-100/mid_mode.py`
- Create: `outputs/cross-axis-slider-v13-under-100/trackpad_mode.py`
- Create: `outputs/cross-axis-slider-v13-under-100/master_assembly.py`
- Create: `outputs/cross-axis-slider-v13-under-100/test_assembly.py`

- [ ] **Step 1: Write assembly-label and interface tests**

```python
import unittest
from assembly import build_pose

class AssemblyTests(unittest.TestCase):
    def test_required_groups_exist(self):
        model = build_pose(0.5)
        for name in ("base", "keyboard_tray", "trackpad_tray",
                     "keyboard_carriage_left", "trackpad_carriage_left",
                     "cross_bearing_left_1", "guide_shoe_keyboard_left"):
            self.assertIn(name, model.bodies)

    def test_no_removed_v12_hardware_exists(self):
        names = set(build_pose(0.5).bodies)
        self.assertFalse(any("604" in name or "shoulder" in name or "CFS" in name for name in names))
```

- [ ] **Step 2: Implement `build_pose(travel)`**

Use explicit named colors: graphite fixed structure, red keyboard group, blue trackpad group, gold four-bearing crosspoint, silver cheap M3 retention proxies, champagne extension-spring envelopes, and dark TPU stops. Each guide shoe moves with its carriage inside a fixed frame rail. Each MR84ZZ center must come directly from `mechanism.pose(travel)`.

- [ ] **Step 3: Add pose generators**

```python
# keyboard_mode.py
from assembly import build_pose
def gen_step():
    return build_pose(0.0).compound
```

Repeat with travel `0.5` and `1.0`; `master_assembly.py` returns the keyboard pose for a stable default Explorer entry.

- [ ] **Step 4: Run assembly tests**

Run: `cd outputs/cross-axis-slider-v13-under-100 && python3 -m unittest -v test_assembly.py`  
Expected: all tests PASS.

### Task 4: Collision screen and STEP generation

**Files:**
- Create: `outputs/cross-axis-slider-v13-under-100/validation.py`
- Create: `outputs/cross-axis-slider-v13-under-100/VALIDATION.md`
- Generate: `outputs/cross-axis-slider-v13-under-100/keyboard_mode.step`
- Generate: `outputs/cross-axis-slider-v13-under-100/mid_mode.step`
- Generate: `outputs/cross-axis-slider-v13-under-100/trackpad_mode.step`
- Generate: `outputs/cross-axis-slider-v13-under-100/master_assembly.step`

- [ ] **Step 1: Implement a 41-sample broad-phase collision screen**

Test fixed base/frame versus moving tray/carriage/shoe groups, and keyboard moving group versus trackpad moving group. Exclude intended interfaces: shoe-to-rail, MR84ZZ-to-long-slot, tray-to-own-carriage, and TPU-stop contact at endpoints. Report every unexpected pair with travel and overlap volume.

- [ ] **Step 2: Run all unit tests and validation**

Run: `cd outputs/cross-axis-slider-v13-under-100 && python3 -m unittest -v`  
Expected: all tests PASS.  
Run: `cd outputs/cross-axis-slider-v13-under-100 && python3 validation.py`  
Expected: motion sample count 401, collision sample count 41, zero unexpected collisions.

- [ ] **Step 3: Export the four explicit STEP targets**

Use the active build123d exporter for each generator. Confirm each STEP is non-empty and imports as a compound with named children.

- [ ] **Step 4: Write the stage-limited validation note**

Document checks actually run, provisional spring envelope assumptions, the unverified printed-shoe friction, and that no detailed tolerance, strength, STL/3MF, BOM finalization, or print layout work is included before user approval.

### Task 5: Viewer handoff and stop gate

**Files:**
- Review: `outputs/cross-axis-slider-v13-under-100/keyboard_mode.step`
- Review: `outputs/cross-axis-slider-v13-under-100/mid_mode.step`
- Review: `outputs/cross-axis-slider-v13-under-100/trackpad_mode.step`

- [ ] **Step 1: Start or reuse CAD Explorer**

Run `dev:ensure` with the workspace root and `keyboard_mode.step`, then prepare links for all three pose files.

- [ ] **Step 2: Capture one diagnostic snapshot per pose**

Use the render snapshot workflow where supported; verify visually that the two trays remain separate, the four gold cross bearings occupy the blue slots, the broad printed shoes sit in the fixed rails, and no side guard appears.

- [ ] **Step 3: Stop for user review**

Return the STEP paths, Explorer links, validation summary, and explicit provisional assumptions. Do not proceed to production tolerances, strength, detailed fasteners, STL/3MF, print plates, final BOM, ZIP, or patent work.
