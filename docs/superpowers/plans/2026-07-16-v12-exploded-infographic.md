# V12 Exploded Infographic Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generate one portrait Chinese exploded-view infographic that accurately communicates the existing V12 keyboard/trackpad mechanism and saves it beside the CAD deliverables.

**Architecture:** Use the existing CAD screenshots as geometry/color references and the downloaded Mac mini infographic only as a layout/style reference. Generate one bitmap through the built-in image generation tool, then visually inspect it against the approved component order and save the selected result non-destructively in the V12 output directory.

**Tech Stack:** CAD Explorer screenshots, built-in image generation, local visual inspection, PNG output.

---

### Task 1: Prepare and verify references

**Files:**
- Read: `outputs/cross-axis-slider-v12-usability-audit/screenshots/01_keyboard_mode.png`
- Read: `outputs/cross-axis-slider-v12-usability-audit/screenshots/02_mid_mode.png`
- Read: `outputs/cross-axis-slider-v12-usability-audit/screenshots/03_roller_detail_exploded.png`
- Read: `outputs/cross-axis-slider-v12-usability-audit/references/exploded-reference.jpg`

- [ ] **Step 1: Confirm all reference images exist**

Run:

```bash
test -s outputs/cross-axis-slider-v12-usability-audit/screenshots/01_keyboard_mode.png
test -s outputs/cross-axis-slider-v12-usability-audit/screenshots/02_mid_mode.png
test -s outputs/cross-axis-slider-v12-usability-audit/screenshots/03_roller_detail_exploded.png
test -s outputs/cross-axis-slider-v12-usability-audit/references/exploded-reference.jpg
```

Expected: exit code 0.

- [ ] **Step 2: Inspect the four references visually**

Expected: the CAD screenshots show the real red/blue/gray mechanism and the style reference shows a white, blue-accented Chinese product infographic.

### Task 2: Generate the infographic

**Files:**
- Create: `outputs/cross-axis-slider-v12-usability-audit/exploded-infographic-v1.png`

- [ ] **Step 1: Generate one portrait infographic with the built-in image tool**

Use case: `infographic-diagram`.

Prompt constraints:

```text
Create a portrait Chinese product exploded-view infographic for the exact keyboard/trackpad switching stand shown in the CAD references. Use the external Mac mini image only for white technical-manual layout, blue numbered callouts, information density, and typography hierarchy. Preserve the CAD mechanism: separate blue trackpad tray, separate red keyboard tray, blue long-slot carriages, red cross-roller carriages, gold bearings, dark-gray fixed side frames, champagne constant-force spring strips, open perforated base, and six TPU feet. Explode vertically in the approved eight-layer order. Include a lower-right close-up of the shoulder-screw/ eccentric-bushing/bearing/washer/M3-nut stack. Do not invent guards, gears, motors, clamps, extra trays, logos, or fixed-center cross pivots. Clean white background, soft technical shadows, crisp Chinese callouts, no watermark.
```

Exact principal labels:

```text
键盘 / 妙控板联动支架｜结构拆解图
完整连续妙控板托盘
完整连续键盘托盘
妙控板长槽滑架
键盘交叉滚轮滑架
604ZZ 双侧斜槽导向
MR84ZZ 双滚轮滑动交点
MISUMI CFS0.2 恒力回位
开放式镂空底座 / 六点防滑
```

- [ ] **Step 2: Copy the generated output into the workspace**

Expected: the final PNG exists at the exact create path and does not overwrite any STEP, STL, 3MF, screenshot, or reference image.

### Task 3: Validate and hand off

**Files:**
- Verify: `outputs/cross-axis-slider-v12-usability-audit/exploded-infographic-v1.png`

- [ ] **Step 1: Inspect the final PNG**

Check all of the following:

```text
portrait white technical layout
one blue tray and one red tray
two blue and two red side carriages
real visible long slots and roller stacks
two dark-gray fixed side frames
one open base and six feet
no invented guard, gear, motor, clamp, or logo
no duplicated tray or impossible solid intersection
principal Chinese text is readable
```

- [ ] **Step 2: If one targeted defect is found, make one focused image edit**

Expected: change only the identified defect while preserving subject, viewpoint, colors, component order, typography hierarchy, and all other validated content.

- [ ] **Step 3: Verify the saved file**

Run:

```bash
file outputs/cross-axis-slider-v12-usability-audit/exploded-infographic-v1.png
test -s outputs/cross-axis-slider-v12-usability-audit/exploded-infographic-v1.png
```

Expected: PNG image data and exit code 0.

- [ ] **Step 4: Return the image inline and provide its absolute saved path**

Expected: the user can see the infographic directly in the final response and can open the project-local PNG.
