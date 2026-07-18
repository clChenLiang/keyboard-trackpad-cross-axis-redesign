# V9 Force-Closed Cross Slider Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce a V9 three-pose STEP concept with continuous input/output rigid bodies, real follower contact, connected guide axles, and a connected return spring.

**Architecture:** Build V9 independently while reusing only primitive geometry helpers and the compact open base. Model one continuous red side carriage and one continuous blue side carriage per side, then add separate bearing rollers and fixed slotted guides. Validate kinematics, contact/connectivity, transmission facts, and collision before export.

**Tech Stack:** Python 3.14, build123d, OpenCascade booleans/distances, unittest, CAD Explorer.

---

### Task 1: Force-chain regression tests

**Files:**
- Create: `outputs/cross-axis-slider-concept-v9-force-closed/test_v9_mechanics.py`
- Create: `outputs/cross-axis-slider-concept-v9-force-closed/cross_axis_slider_concept_v9.py`

- [ ] Add a 201-point kinematic constraint test.
- [ ] Add three-pose rigid connectivity and nominal-contact tests.
- [ ] Add transmission-ratio and transmission-projection tests.
- [ ] Add a 21-pose forbidden-collision test.
- [ ] Run the tests before the generator exists and confirm import failure.

### Task 2: Continuous moving rigid bodies

**Files:**
- Create: `outputs/cross-axis-slider-concept-v9-force-closed/cross_axis_slider_concept_v9.py`

- [ ] Build each red side carriage as one fused solid connected to the keyboard tray.
- [ ] Build each blue side carriage as one fused solid connected to the trackpad tray.
- [ ] Add cross-axis and guide axles at locations that overlap their owning side carriage.
- [ ] Add nominal-contact follower and guide rollers as separately labeled bearing bodies.
- [ ] Add moving and fixed spring seats with a spring spanning the actual seat distance.
- [ ] Run connectivity tests until every required force-path edge is present.

### Task 3: Collision repair and STEP handoff

**Files:**
- Create: `outputs/cross-axis-slider-concept-v9-force-closed/keyboard_mode.py`
- Create: `outputs/cross-axis-slider-concept-v9-force-closed/mid_mode.py`
- Create: `outputs/cross-axis-slider-concept-v9-force-closed/trackpad_mode.py`
- Create: `outputs/cross-axis-slider-concept-v9-force-closed/CAD_BRIEF.md`
- Generate: three matching `.step` files and Explorer sidecars.

- [ ] Repair only collision-producing geometry while retaining all force-path connections.
- [ ] Generate the three explicit STEP targets.
- [ ] Inspect exported facts, planes, bounds, and occurrence counts.
- [ ] Render diagnostic isometric and side views.
- [ ] Re-run all mechanics tests immediately before handoff.

