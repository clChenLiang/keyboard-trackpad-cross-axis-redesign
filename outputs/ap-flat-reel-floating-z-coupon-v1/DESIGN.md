# A-P flat-reel + floating-Z validation coupon v1

## CAD brief

- Model: an independent kinematic validation coupon for a planar, single-layer toothed belt winding onto a vertical drum through a 15 mm Z-floating fork. It does not reuse or modify any V1/V2/V3 bracket body.
- Units and frame: millimetres; XY is the horizontal plane; +Y is the belt feed direction; +Z is up; drum axis is world Z at the origin.
- Input motion: a small horizontal coupon carrier translates from `(0, 0, 0)` to `(0, +20.50, -15.00)` relative to its extended pose. No keyboard or trackpad body is represented.
- Isolation rule: the carrier and fork take the full Z stroke while the belt-end clamp/slider stays at a constant world Z. Relative to the drum, the belt therefore feeds only along +Y.
- Transmission: a 41-groove `2M-GT2` synchronous drum uses exact 2.00 mm circular pitch, giving pitch diameter `41 × 2 / π = 26.1014 mm`; 20.50 mm feed is therefore exactly 90 degrees. The nominal outside diameter is 0.50 mm below pitch diameter, consistent with published 2 mm GT2 pulley examples. The coupon uses rounded BREP groove/tooth silhouettes and explicit phase; it does not claim the proprietary production cutter geometry or a final tolerance class.
- Belt concept: 12 mm wide backing with rounded transverse `2M-GT2` nominal teeth at 2.00 mm pitch. The posed BREP shows the changing straight run plus a planar single-layer circular wrap. Engagement is counted only when a tooth BREP occupies a matching groove BREP with no pulley penetration; it is kinematic CAD, not flexible-material or load/contact simulation.
- Fixed enclosure: 46 mm target outer diameter, 58 mm height, with a closed entry tunnel, upper/lower radial-bearing seats, and a lower thrust interface.
- Interfaces: separate one-sided 0/90-degree hard stops touch the rotary lug only at their respective endpoints. Two vertical rounded drive keys run in matching clamp-slider keyways: coincident Y faces transmit feed, while X clearance and long Z capture permit the full 15 mm float. Entry guide, bearing seats, thrust interface, and stop bodies carry explicit webs/bosses into the protective housing. Three spring-anchor holes remain uncommitted. Spring rate, spring type, preload, belt material, production tooth cutter, fit, and tolerance are intentionally not selected.
- Output abstraction: only the vertical shaft and a short pointer arm are shown downstream of the drum.

## Deliverables and validation gates

- `extended_0deg.step`, `mid_45deg.step`, `stored_90deg.step`, and `cylinder_section_exploded.step`.
- Every functional entity remains a separately labelled solid/compound child.
- Regression checks cover endpoint displacement, constant clamp Z, pitch-radius conversion, constant belt centreline length, BREP/phased engagement of at least five teeth, endpoint-only stop contact, Z-guide capture, fixed-feature connectivity, required labels, positive solids, and an all-top-level-pair 11-pose unintended-interference sweep.
- Final checks include STEP round-trip readability, BREP and re-imported STEP wrap-angle measurement, CAD CLI inspection, and a CAD Explorer review link plus a small diagnostic snapshot packet.

## Scope boundary

This coupon validates only topology, nominal 2M-GT2-compatible pitch/groove phase, geometry, motion mapping, and obvious nominal BREP interference. It does not validate material flexure, tooth stress/contact, fatigue, spring choice, load capacity, proprietary production cutter geometry, final clearances/tolerances, manufacturability, or print layout.
