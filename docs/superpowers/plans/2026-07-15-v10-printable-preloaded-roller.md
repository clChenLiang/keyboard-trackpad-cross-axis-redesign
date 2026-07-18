# V10 Printable Preloaded Roller Mechanism Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and verify a printable keyboard/trackpad switching stand with bilateral anti-backlash roller constraints, real return springs, connected printable parts, and complete STEP/STL/3MF handoff files.

**Architecture:** Create V10 independently from V1–V9. Keep motion equations in a geometry-free module, build printable solids in focused part factories, assemble labeled poses from explicit placements, and run numerical plus OpenCascade validation before exporting manufacturing files.

**Tech Stack:** Python 3.9, build123d/OpenCascade, unittest, CAD skill launchers, CAD Explorer.

---

## File map

- Create `outputs/cross-axis-slider-v10-printable-preloaded/parameters.py`: dimensions, print compensation, bearing and hardware data.
- Create `outputs/cross-axis-slider-v10-printable-preloaded/mechanism.py`: pose equations, roller centers, force and spring calculations.
- Create `outputs/cross-axis-slider-v10-printable-preloaded/parts.py`: all printable and purchased-part geometry.
- Create `outputs/cross-axis-slider-v10-printable-preloaded/assembly.py`: labeled pose assembly and part placements.
- Create `outputs/cross-axis-slider-v10-printable-preloaded/validation.py`: collision, connection, stability, printability and assembly checks.
- Create `outputs/cross-axis-slider-v10-printable-preloaded/export_printables.py`: explicit STEP/STL/3MF and report export.
- Create `outputs/cross-axis-slider-v10-printable-preloaded/test_mechanism.py`: pure mathematical tests.
- Create `outputs/cross-axis-slider-v10-printable-preloaded/test_geometry.py`: solid and connection tests.
- Create `outputs/cross-axis-slider-v10-printable-preloaded/test_full_validation.py`: sampled motion and final-deliverable tests.
- Create `outputs/cross-axis-slider-v10-printable-preloaded/BOM.md`: exact hardware and printed part counts.
- Create `outputs/cross-axis-slider-v10-printable-preloaded/ASSEMBLY.md`: print orientation, calibration and assembly order.
- Generate pose STEP files, master assembly STEP, individual printable STEP/STL files, a 3MF package and `validation_report.json`.

The workspace is not a Git repository. Replace each normally required commit with a fresh test run and a written checkpoint in `validation_report.json`; do not initialize Git without user authorization.

### Task 1: Parameter and motion contract

**Files:**
- Create: `outputs/cross-axis-slider-v10-printable-preloaded/parameters.py`
- Create: `outputs/cross-axis-slider-v10-printable-preloaded/mechanism.py`
- Test: `outputs/cross-axis-slider-v10-printable-preloaded/test_mechanism.py`

- [ ] **Step 1: Write the failing motion tests**

```python
import unittest
from mechanism import pose, validate_motion, spring_requirement

class MotionTests(unittest.TestCase):
    def test_end_positions_and_constraint(self):
        self.assertEqual((30.0, -15.0), pose(1.0).keyboard_delta)
        self.assertEqual((-115.0, -43.0), pose(1.0).trackpad_delta)
        report = validate_motion(401)
        self.assertLess(report["max_cross_error_mm"], 0.01)
        self.assertGreaterEqual(report["min_cross_end_margin_mm"], 5.0)
        self.assertTrue(report["monotonic"])

    def test_spring_window_is_feasible(self):
        result = spring_requirement(
            moving_mass_kg=0.90,
            rolling_resistance_n=2.0,
            spring_count=2,
        )
        self.assertGreaterEqual(result["start_return_margin_n"], 2.0)
        self.assertLessEqual(result["maximum_user_force_n"], 20.0)
        self.assertGreaterEqual(result["coil_bind_margin_mm"], 2.0)
```

- [ ] **Step 2: Run the tests and verify RED**

Run:

```bash
cd outputs/cross-axis-slider-v10-printable-preloaded
../../work/cad-venv/bin/python -m unittest -v test_mechanism.py
```

Expected: import failure because `mechanism.py` does not exist.

- [ ] **Step 3: Implement named parameter records and exact affine kinematics**

`parameters.py` must define immutable records for the two device envelopes, tray size, paths, bearing dimensions, slot widths, eccentricity, print compensation, base candidate size, safety margins and material density. `mechanism.py` must expose:

```python
@dataclass(frozen=True)
class Pose:
    travel: float
    keyboard_delta: tuple[float, float]
    trackpad_delta: tuple[float, float]
    cross_slot_center: tuple[float, float]
    cross_roller_centers: tuple[tuple[float, float], tuple[float, float]]

def pose(travel: float) -> Pose: ...
def validate_motion(sample_count: int = 401) -> dict: ...
def spring_requirement(moving_mass_kg: float,
                       rolling_resistance_n: float,
                       spring_count: int = 2) -> dict: ...
```

Compute the slot direction from relative displacement, not from a typed angle. Compute spring energy from both device gravitational potential changes, printed moving mass, rolling resistance and a two-spring linear force curve. Reject a spring result if force or coil-bind constraints conflict.

- [ ] **Step 4: Run the motion tests and verify GREEN**

Run the Step 2 command. Expected: two passing tests and no warnings.

### Task 2: Printable part factories and calibration coupons

**Files:**
- Create: `outputs/cross-axis-slider-v10-printable-preloaded/parts.py`
- Test: `outputs/cross-axis-slider-v10-printable-preloaded/test_geometry.py`

- [ ] **Step 1: Write failing solid tests**

```python
import unittest
from parts import printable_parts, calibration_coupon

class GeometryTests(unittest.TestCase):
    def test_every_printable_is_one_valid_solid(self):
        for name, part in printable_parts().items():
            self.assertEqual(1, len(part.solids()), name)
            self.assertTrue(part.is_valid(), name)
            self.assertGreater(part.volume, 0.0, name)

    def test_coupon_contains_required_interfaces(self):
        coupon = calibration_coupon()
        self.assertEqual(1, len(coupon.solids()))
        self.assertGreater(coupon.volume, 0.0)
```

- [ ] **Step 2: Run and verify RED**

Run:

```bash
cd outputs/cross-axis-slider-v10-printable-preloaded
../../work/cad-venv/bin/python -m unittest -v test_geometry.py
```

Expected: import failure because `parts.py` does not exist.

- [ ] **Step 3: Implement the factories**

`parts.py` must provide closed solids with stable labels:

```python
def make_open_base() -> Part: ...
def make_side_frame(side: str) -> Part: ...
def make_tray(kind: str) -> Part: ...
def make_carriage(kind: str, side: str) -> Part: ...
def make_eccentric_bushing(kind: str) -> Part: ...
def make_tpu_stop(kind: str) -> Part: ...
def make_tpu_device_pad(kind: str) -> Part: ...
def calibration_coupon() -> Part: ...
def printable_parts() -> dict[str, Part]: ...
```

Build the side frame as one union of both guide plates, base feet and triangular gussets. Cut real M4 clearance holes and real closed slots. Build tray top skins plus underside ribs as one solid. Add the specified recessed text, device pockets, cable clearance and TPU pad sockets. The coupon must include compensated 4 mm shaft holes, MR84/604 bearing pockets, both eccentric sockets and a TPU dovetail socket.

- [ ] **Step 4: Run geometry tests and verify GREEN**

Run the Step 2 command. Expected: all named printable parts are valid single solids.

### Task 3: Anti-backlash bearing placements and connected assembly

**Files:**
- Create: `outputs/cross-axis-slider-v10-printable-preloaded/assembly.py`
- Modify: `outputs/cross-axis-slider-v10-printable-preloaded/test_geometry.py`

- [ ] **Step 1: Add failing assembly interface tests**

```python
from assembly import build_pose

def test_required_interfaces_touch_or_overlap(self):
    for travel in (0.0, 0.5, 1.0):
        assembly = build_pose(travel)
        report = assembly.interface_report()
        self.assertEqual([], report["disconnected"], report)
        self.assertGreaterEqual(report["minimum_cross_end_margin_mm"], 5.0)
        self.assertGreaterEqual(report["minimum_stop_before_slot_end_mm"], 2.0)

def test_opposed_rollers_contact_opposite_walls(self):
    for travel in (0.0, 0.5, 1.0):
        report = build_pose(travel).preload_report()
        self.assertLessEqual(report["maximum_adjusted_wall_gap_mm"], 0.05)
        self.assertGreaterEqual(report["minimum_adjustment_reserve_mm"], 0.10)
```

- [ ] **Step 2: Run and verify RED**

Run the geometry test command. Expected: `assembly` import failure.

- [ ] **Step 3: Implement explicit part placements**

`assembly.py` must define a small wrapper that retains both the labeled build123d compound and numeric placement metadata:

```python
@dataclass
class PoseAssembly:
    compound: Compound
    bodies: dict[str, Shape]
    metadata: dict
    def interface_report(self) -> dict: ...
    def preload_report(self) -> dict: ...

def build_pose(travel: float, include_devices: bool = False) -> PoseAssembly: ...
```

Place two MR84 bearings per side in the cross slot and two 604 bearings in each tray guide slot. Offset fixed and eccentric rollers toward opposite walls. Model the eccentric bushing, M4 shoulder, washer and retaining head as separate labeled bodies. Implement the right-side X float geometrically with a wider axial pocket, not only a label. Add guided compression-spring envelopes and TPU stops at both endpoints.

- [ ] **Step 4: Run geometry tests and verify GREEN**

Expected: connection and preload checks pass at all three poses.

### Task 4: Sampled collision, clearance and stability validation

**Files:**
- Create: `outputs/cross-axis-slider-v10-printable-preloaded/validation.py`
- Create: `outputs/cross-axis-slider-v10-printable-preloaded/test_full_validation.py`

- [ ] **Step 1: Write failing full validation tests**

```python
import unittest
from validation import validate_all

class FullValidationTests(unittest.TestCase):
    def test_full_validation(self):
        report = validate_all(motion_samples=401, collision_samples=101)
        self.assertEqual([], report["errors"], report)
        self.assertGreaterEqual(report["stability"]["minimum_margin_mm"], 15.0)
        self.assertGreaterEqual(report["clearance"]["bearing_to_slot_end_mm"], 2.0)
        self.assertTrue(report["spring"]["returns_with_10_percent_mismatch"])
        self.assertLessEqual(report["spring"]["maximum_user_force_n"], 20.0)
```

- [ ] **Step 2: Run and verify RED**

Run:

```bash
cd outputs/cross-axis-slider-v10-printable-preloaded
../../work/cad-venv/bin/python -m unittest -v test_full_validation.py
```

Expected: `validation` import failure.

- [ ] **Step 3: Implement broad-phase and exact checks**

`validate_all()` must:

1. Run the 401-point mathematical motion check.
2. Build 101 exact solid poses.
3. Use bounding boxes only to prune pairs, then OpenCascade common volume and distance for the decision.
4. Allow only enumerated shaft/bearing, bearing/slot-wall, spring/seat and TPU/stop design contacts.
5. Check base/side-frame and tray/carriage connections separately from collision exclusions.
6. Compute printed-part masses from volume and configured material density; add device and hardware point masses.
7. Compute total CG in both endpoints and the minimum distance to the four-foot support polygon.
8. Test nominal spring forces and `±10%` left/right mismatch.
9. Check H2D single-part bounding boxes and minimum isolated load-bearing thickness.

- [ ] **Step 4: Run full validation and repair failures one cause at a time**

Run the Step 2 command after each smallest geometry repair. Do not weaken allowed-collision lists or safety thresholds to make the test pass.

### Task 5: Exact spring and hardware baseline

**Files:**
- Modify: `outputs/cross-axis-slider-v10-printable-preloaded/parameters.py`
- Create: `outputs/cross-axis-slider-v10-printable-preloaded/BOM.md`
- Modify: `outputs/cross-axis-slider-v10-printable-preloaded/test_full_validation.py`

- [ ] **Step 1: Add a failing exact-BOM test**

```python
from parameters import BOM

def test_bom_is_exact(self):
    self.assertEqual(4, BOM["MR84ZZ"]["quantity"])
    self.assertEqual(8, BOM["604ZZ"]["quantity"])
    self.assertEqual(2, BOM["compression_spring"]["quantity"])
    for key, item in BOM.items():
        self.assertNotIn("range", item, key)
        self.assertIsNotNone(item.get("spec"), key)
```

- [ ] **Step 2: Run and verify RED**

Expected: `BOM` is absent or the spring specification is not exact.

- [ ] **Step 3: Select one spring specification from calculated requirements**

Choose one free length, OD, wire diameter, solid length and rate that satisfies the verified spring window. Store the exact values in `parameters.py`; update the spring envelope and seats to that part. Record all bearing, shoulder screw, fastener, washer, spring and TPU counts in `BOM.md`, including which interfaces use printed eccentric bushings.

- [ ] **Step 4: Re-run full validation and verify GREEN**

Expected: exact BOM test and the 101-pose validation pass.

### Task 6: STEP-first export and secondary print files

**Files:**
- Create: `outputs/cross-axis-slider-v10-printable-preloaded/export_printables.py`
- Create: `outputs/cross-axis-slider-v10-printable-preloaded/ASSEMBLY.md`
- Generate: `outputs/cross-axis-slider-v10-printable-preloaded/*.step`
- Generate: `outputs/cross-axis-slider-v10-printable-preloaded/printables/*.step`
- Generate: `outputs/cross-axis-slider-v10-printable-preloaded/printables/*.stl`
- Generate: `outputs/cross-axis-slider-v10-printable-preloaded/printables/v10_printables.3mf`
- Generate: `outputs/cross-axis-slider-v10-printable-preloaded/validation_report.json`

- [ ] **Step 1: Add failing deliverable tests**

```python
from pathlib import Path

def test_required_deliverables_exist(self):
    root = Path(__file__).parent
    required = [
        root / "keyboard_mode.step",
        root / "mid_mode.step",
        root / "trackpad_mode.step",
        root / "master_assembly.step",
        root / "printables" / "v10_printables.3mf",
        root / "validation_report.json",
        root / "BOM.md",
        root / "ASSEMBLY.md",
    ]
    self.assertEqual([], [str(p) for p in required if not p.exists()])
```

- [ ] **Step 2: Run and verify RED**

Expected: generated deliverables are missing.

- [ ] **Step 3: Implement deterministic exports**

`export_printables.py` must refuse to export if `validate_all()` has errors. It must export STEP first, then one STEP and STL per unique printable part, then assemble the 3MF without silently rescaling units. `ASSEMBLY.md` must list calibration coupon printing first, part orientations, eccentric adjustment order, fastener access, spring installation and endpoint checks.

- [ ] **Step 4: Run the exporter and deliverable tests**

Run:

```bash
cd outputs/cross-axis-slider-v10-printable-preloaded
../../work/cad-venv/bin/python export_printables.py
../../work/cad-venv/bin/python -m unittest -v
```

Expected: all tests pass; all files exist and have positive size.

### Task 7: CAD inspection, render review and completion audit

**Files:**
- Inspect all generated STEP files and `validation_report.json`.
- Modify only the smallest responsible V10 source file if a check fails.

- [ ] **Step 1: Generate CAD Explorer sidecars for explicit targets**

Run the CAD skill `scripts/step` launcher separately for `keyboard_mode.step`, `mid_mode.step`, `trackpad_mode.step` and `master_assembly.step`. Do not run directory-wide generation and do not skip Explorer.

- [ ] **Step 2: Inspect facts and positioning**

For the master assembly and all three poses, run:

```bash
python /Users/bytedance/.agents/skills/cad/scripts/inspect refs <absolute-step-path> --facts --planes --positioning
```

Confirm valid solids, expected occurrences, millimetre-scale bounds, stable labels and no empty bodies.

- [ ] **Step 3: Render risk-focused views**

Use the render skill to make isometric and side snapshots of keyboard, midpoint and trackpad modes. Review the cross roller pairs, guide roller pairs, spring seats, four base connections, both tray connections, hard stops and tool access.

- [ ] **Step 4: Run a fresh final verification**

Run:

```bash
cd outputs/cross-axis-slider-v10-printable-preloaded
../../work/cad-venv/bin/python -m unittest -v
../../work/cad-venv/bin/python export_printables.py --verify-only
```

Read the complete output. Completion requires zero failed tests, zero validation errors, all requested artifacts, and no unresolved visual connection or collision finding.

- [ ] **Step 5: Return Explorer links and limitations**

Report the four CAD Explorer links, exact artifact paths, executed sample counts, minimum clearances, stability margin, spring force window, exact BOM and the mandatory calibration/empty-assembly sequence. Do not describe real-world durability as proven until a physical prototype has completed cycling tests.
