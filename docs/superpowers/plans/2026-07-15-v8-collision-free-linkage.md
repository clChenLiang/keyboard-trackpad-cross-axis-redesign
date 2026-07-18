# V8 Collision-Free Linkage Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a new V8 STEP concept whose rigid bodies can traverse the complete low-lift stroke without hard interference.

**Architecture:** Create a standalone V8 assembly generator using the existing V1/V3 primitive helpers and compact open base, but do not inherit V7's colliding assembly children. Separate each side into four X layers and validate motion and collision through reusable Python functions.

**Tech Stack:** Python 3.14, build123d, OpenCascade boolean intersection, CAD Explorer.

---

### Task 1: Collision regression harness

**Files:**
- Create: `outputs/cross-axis-slider-concept-v8-collision-free/test_v8_collision.py`
- Create: `outputs/cross-axis-slider-concept-v8-collision-free/cross_axis_slider_concept_v8.py`

- [ ] Write tests requiring 201 valid kinematic samples and zero forbidden collision pairs over 21 poses.
- [ ] Run the tests before the V8 generator exists and confirm import failure.
- [ ] Implement labeled rigid-body groups and exact boolean collision reporting.
- [ ] Run the tests and retain any geometric collision as a failing result.

### Task 2: Layered mechanism geometry

**Files:**
- Create: `outputs/cross-axis-slider-concept-v8-collision-free/cross_axis_slider_concept_v8.py`

- [ ] Build the compact open base and independent continuous trays.
- [ ] Build mirrored red drivers at the inner layer and single blue slotted cheeks at the next layer.
- [ ] Add short crossover shafts and one real roller per side.
- [ ] Add outer-layer keyboard and trackpad slotted guides, rollers, supports, compression springs, and terminal stops.
- [ ] Iterate only the responsible dimensions until the collision regression is green.

### Task 3: STEP generation and review

**Files:**
- Create: `outputs/cross-axis-slider-concept-v8-collision-free/keyboard_mode.py`
- Create: `outputs/cross-axis-slider-concept-v8-collision-free/mid_mode.py`
- Create: `outputs/cross-axis-slider-concept-v8-collision-free/trackpad_mode.py`
- Create: `outputs/cross-axis-slider-concept-v8-collision-free/CAD_BRIEF.md`
- Generate: three matching `.step` files and Explorer sidecars.

- [ ] Generate each explicit STEP target with the CAD launcher.
- [ ] Run STEP facts/planes/positioning inspection and compare the three bounding boxes.
- [ ] Run a diagnostic snapshot packet because this is a multi-body moving assembly.
- [ ] Re-run the full kinematic and collision suite immediately before reporting completion.

