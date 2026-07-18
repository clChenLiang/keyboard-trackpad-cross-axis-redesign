"""Deterministic numerical and BREP validation for V10."""

import gc
import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import sys
from typing import Dict, List, Tuple

from build123d import CenterOf

import assembly
import mechanism as mech
import parameters as p
import parts
import print_layouts


STEEL_DENSITY_KG_PER_MM3 = 7.85e-6
ROLLING_RESISTANCE_N = 2.0


def _bbox_overlap(a, b, tolerance: float = 0.0) -> bool:
    aa = a.bounding_box()
    bb = b.bounding_box()
    return not (
        aa.max.X < bb.min.X - tolerance
        or bb.max.X < aa.min.X - tolerance
        or aa.max.Y < bb.min.Y - tolerance
        or bb.max.Y < aa.min.Y - tolerance
        or aa.max.Z < bb.min.Z - tolerance
        or bb.max.Z < aa.min.Z - tolerance
    )


def _overlap_volume(a, b) -> float:
    common = a & b
    return 0.0 if common is None else sum(s.volume for s in common.solids())


def _motion_group(label: str) -> str:
    if label == "base" or label.startswith(("fixed_", "spring_guide_rod_")):
        return "F"
    if label.startswith(("keyboard_", "cross_", "compression_spring_")):
        return "K"
    if label.startswith("trackpad_"):
        return "T"
    raise ValueError(f"unclassified body {label}")


def _allowed_penetration(label_a: str, label_b: str) -> bool:
    pair = frozenset((label_a, label_b))
    for side in ("left", "right"):
        if pair == frozenset((f"compression_spring_{side}", f"fixed_side_frame_{side}")):
            return True
    return False


def _designed_contact(label_a: str, label_b: str) -> bool:
    pair = frozenset((label_a, label_b))
    for side in ("left", "right"):
        frame = f"fixed_side_frame_{side}"
        trackpad_carriage = f"trackpad_carriage_{side}"
        for kind in ("keyboard", "trackpad"):
            for index in (1, 2):
                if pair == frozenset((frame, f"{kind}_guide_bearing_{side}_{index}")):
                    return True
        for index in (1, 2):
            if pair == frozenset((trackpad_carriage, f"cross_bearing_{side}_{index}")):
                return True
    return False


def validate_collision_pose(travel: float, volume_tolerance: float = 0.05) -> Dict:
    forbidden_groups = {
        frozenset(("F", "K")),
        frozenset(("F", "T")),
        frozenset(("K", "T")),
    }
    collisions: List[dict] = []
    bbox_candidates = 0
    pose = assembly.build_pose(travel)
    items = list(pose.bodies.items())
    for index, (label_a, body_a) in enumerate(items):
        group_a = _motion_group(label_a)
        for label_b, body_b in items[index + 1:]:
            group_b = _motion_group(label_b)
            if frozenset((group_a, group_b)) not in forbidden_groups:
                continue
            if _allowed_penetration(label_a, label_b) or _designed_contact(label_a, label_b):
                continue
            if not _bbox_overlap(body_a, body_b):
                continue
            bbox_candidates += 1
            if body_a.distance_to(body_b) > 1e-5:
                continue
            overlap = _overlap_volume(body_a, body_b)
            if overlap > volume_tolerance:
                collisions.append({
                    "travel": round(travel, 5),
                    "a": label_a,
                    "b": label_b,
                    "overlap_mm3": round(overlap, 5),
                })
    del pose
    gc.collect()
    return {
        "travel": travel,
        "bbox_candidates": bbox_candidates,
        "collisions": collisions,
    }


def _run_collision_worker(args) -> Dict:
    travel, volume_tolerance = args
    worker = Path(__file__).with_name("collision_worker.py")
    completed = subprocess.run(
        [sys.executable, str(worker), str(travel), str(volume_tolerance)],
        cwd=str(worker.parent),
        text=True,
        capture_output=True,
    )
    if completed.returncode != 0:
        return {
            "travel": travel,
            "bbox_candidates": 0,
            "collisions": [{
                "travel": round(travel, 5),
                "a": "collision_worker",
                "b": "cad_kernel",
                "overlap_mm3": -1.0,
                "error": completed.stderr.strip() or f"worker exit {completed.returncode}",
            }],
        }
    try:
        return json.loads(completed.stdout)
    except json.JSONDecodeError as exc:
        return {
            "travel": travel,
            "bbox_candidates": 0,
            "collisions": [{
                "travel": round(travel, 5),
                "a": "collision_worker",
                "b": "invalid_json",
                "overlap_mm3": -1.0,
                "error": str(exc),
            }],
        }


def validate_collisions(sample_count: int = 101, volume_tolerance: float = 0.05) -> Dict:
    if sample_count < 2:
        raise ValueError("sample_count must be at least 2")
    jobs = [
        (index / (sample_count - 1), volume_tolerance)
        for index in range(sample_count)
    ]
    # Two isolated kernels stay below the CAD stack's observed memory ceiling;
    # four concurrent workers were fast but could be terminated by the OS.
    with ThreadPoolExecutor(max_workers=min(2, sample_count)) as executor:
        results = list(executor.map(_run_collision_worker, jobs))
    collisions = [item for result in results for item in result["collisions"]]
    return {
        "sample_count": sample_count,
        "bbox_candidates": sum(result["bbox_candidates"] for result in results),
        "collisions": collisions,
        "truncated": False,
    }


def _density_for(label: str) -> float:
    if "tpu" in label or "float_washer" in label or "stop_collar" in label:
        return p.TPU_DENSITY_KG_PER_MM3
    if "eccentric_bushing" in label:
        return p.PETG_DENSITY_KG_PER_MM3
    if label.startswith(("compression_spring_", "spring_guide_rod_")):
        return STEEL_DENSITY_KG_PER_MM3
    if any(token in label for token in ("bearing", "shaft", "screw", "insert", "washer", "nut")):
        return STEEL_DENSITY_KG_PER_MM3
    return p.PETG_DENSITY_KG_PER_MM3


def _mass_properties(pose: assembly.PoseAssembly) -> Dict:
    entries = []
    total_mass = 0.0
    weighted = [0.0, 0.0, 0.0]
    keyboard_mass = p.KEYBOARD.mass_kg
    trackpad_mass = p.TRACKPAD.mass_kg
    for label, shape in pose.bodies.items():
        mass = shape.volume * _density_for(label)
        center = shape.center(CenterOf.MASS)
        entries.append((label, mass, (center.X, center.Y, center.Z)))
        total_mass += mass
        weighted[0] += mass * center.X
        weighted[1] += mass * center.Y
        weighted[2] += mass * center.Z
        group = _motion_group(label)
        if group == "K":
            keyboard_mass += mass
        elif group == "T":
            trackpad_mass += mass

    state = mech.pose(pose.metadata["travel"])
    devices = [
        (
            p.KEYBOARD.mass_kg,
            (0.0, state.keyboard_position[0], state.keyboard_position[1] + p.TRAY_SKIN + p.KEYBOARD.height / 2.0),
        ),
        (
            p.TRACKPAD.mass_kg,
            (
                (p.TRAY_WIDTH - p.TRACKPAD.width) / 2.0,
                state.trackpad_position[0],
                state.trackpad_position[1] + p.TRAY_SKIN + p.TRACKPAD.height / 2.0,
            ),
        ),
    ]
    for mass, center in devices:
        total_mass += mass
        for axis in range(3):
            weighted[axis] += mass * center[axis]
    cg = tuple(value / total_mass for value in weighted)
    return {
        "total_mass_kg": total_mass,
        "cg_mm": cg,
        "keyboard_moving_mass_kg": keyboard_mass,
        "trackpad_moving_mass_kg": trackpad_mass,
        "entries": entries,
    }


def validate_stability() -> Dict:
    reports = []
    x_limit = p.BASE_WIDTH / 2.0 - p.BASE_FOOT_INSET_X
    y_min = p.BASE_CENTER_Y - p.BASE_DEPTH / 2.0 + p.BASE_FOOT_INSET_Y
    y_max = p.BASE_CENTER_Y + p.BASE_DEPTH / 2.0 - p.BASE_FOOT_INSET_Y
    for travel in (0.0, 1.0):
        props = _mass_properties(assembly.build_pose(travel))
        cg = props["cg_mm"]
        margin = min(x_limit - abs(cg[0]), cg[1] - y_min, y_max - cg[1])
        reports.append({
            "travel": travel,
            "cg_mm": cg,
            "margin_mm": margin,
            "total_mass_kg": props["total_mass_kg"],
            "keyboard_moving_mass_kg": props["keyboard_moving_mass_kg"],
            "trackpad_moving_mass_kg": props["trackpad_moving_mass_kg"],
        })
    return {
        "support_bounds_mm": {"x": (-x_limit, x_limit), "y": (y_min, y_max)},
        "poses": reports,
        "minimum_margin_mm": min(item["margin_mm"] for item in reports),
    }


def validate_spring() -> Dict:
    props = _mass_properties(assembly.build_pose(0.0))
    keyboard_mass = props["keyboard_moving_mass_kg"]
    trackpad_mass = props["trackpad_moving_mass_kg"]
    gravity_equivalent = p.GRAVITY * (
        keyboard_mass * abs(p.KEYBOARD_DZ)
        + trackpad_mass * abs(p.TRACKPAD_DZ)
    ) / mech.KEYBOARD_PATH
    required_start = gravity_equivalent + ROLLING_RESISTANCE_N
    nominal_start = p.SPRING_COUNT * p.SPRING_PRELOAD_PER_SPRING
    weakest_start = nominal_start * 0.90
    end_force = nominal_start + (
        p.SPRING_COUNT * p.SPRING_RATE_PER_SPRING * mech.KEYBOARD_PATH
    )
    maximum_user = end_force - gravity_equivalent + ROLLING_RESISTANCE_N
    compressed_length = p.SPRING_INSTALLED_LENGTH - mech.KEYBOARD_PATH
    coil_bind_margin = compressed_length - p.SPRING_SOLID_LENGTH
    return {
        "keyboard_moving_mass_kg": keyboard_mass,
        "trackpad_moving_mass_kg": trackpad_mass,
        "gravity_equivalent_n": gravity_equivalent,
        "required_start_force_n": required_start,
        "nominal_start_force_n": nominal_start,
        "weakest_10_percent_start_force_n": weakest_start,
        "start_return_margin_n": nominal_start - required_start,
        "returns_with_10_percent_mismatch": weakest_start >= required_start,
        "maximum_user_force_n": maximum_user,
        "coil_bind_margin_mm": coil_bind_margin,
    }


def validate_printability() -> Dict:
    failures = []
    facts = []
    for name, shape in parts.printable_parts().items():
        bbox = shape.bounding_box().size
        dimensions = sorted((bbox.X, bbox.Y, bbox.Z))
        machine = sorted(p.H2D_SINGLE_NOZZLE_VOLUME)
        fits = all(dimensions[index] <= machine[index] for index in range(3))
        single_solid = len(shape.solids()) == 1 and shape.is_valid()
        item = {
            "name": name,
            "bbox_mm": (bbox.X, bbox.Y, bbox.Z),
            "fits_h2d": fits,
            "single_valid_solid": single_solid,
        }
        facts.append(item)
        if not fits or not single_solid:
            failures.append(item)
    if p.MIN_LOAD_WALL < 1.2:
        failures.append({"minimum_load_wall_mm": p.MIN_LOAD_WALL})
    return {
        "parts": facts,
        "minimum_load_wall_mm": p.MIN_LOAD_WALL,
        "failures": failures,
    }


def validate_plate_layouts(clearance_mm: float = 2.0) -> Dict:
    """Verify H2D XY bounds and spacing for the five supplied 3MF plates."""
    failures = []
    reports = []
    factories = (
        ("base_plate", print_layouts.base_plate),
        ("trays_plate", print_layouts.trays_plate),
        ("frames_plate", print_layouts.frames_plate),
        ("carriages_plate", print_layouts.carriages_plate),
        ("petg_small_parts_plate", print_layouts.petg_small_parts_plate),
        ("tpu_parts_plate", print_layouts.tpu_parts_plate),
    )
    for name, factory in factories:
        plate = factory()
        bbox = plate.bounding_box().size
        overlaps = []
        for index, first in enumerate(plate.children):
            a = first.bounding_box()
            for second in plate.children[index + 1:]:
                b = second.bounding_box()
                separated = (
                    a.max.X + clearance_mm <= b.min.X
                    or b.max.X + clearance_mm <= a.min.X
                    or a.max.Y + clearance_mm <= b.min.Y
                    or b.max.Y + clearance_mm <= a.min.Y
                )
                if not separated:
                    overlaps.append((first.label, second.label))
        item = {
            "name": name,
            "bbox_mm": (bbox.X, bbox.Y, bbox.Z),
            "object_count": len(plate.children),
            "fits_h2d_xy": (
                bbox.X <= (
                    p.H2D_SINGLE_NOZZLE_VOLUME[0]
                )
                and bbox.Y <= p.H2D_SINGLE_NOZZLE_VOLUME[1]
            ),
            "minimum_xy_spacing_mm": clearance_mm,
            "bbox_overlaps": overlaps,
        }
        reports.append(item)
        if not item["fits_h2d_xy"] or overlaps:
            failures.append(item)
    return {"plates": reports, "failures": failures}


def validate_strength(hand_load_n: float = 20.0) -> Dict:
    """Conservative first-order PETG-HF bending screen, not an FEA claim."""
    if hand_load_n <= 0.0:
        raise ValueError("hand load must be positive")
    elastic_modulus_mpa = 2000.0
    allowable_stress_mpa = 15.0

    skin_area = p.TRAY_DEPTH * p.TRAY_SKIN
    skin_centroid_z = p.TRAY_SKIN / 2.0
    rib_count = 4.0  # front, rear, and the two transverse ribs
    rib_area_each = p.TRAY_RIB_THICKNESS * p.TRAY_RIB_DEPTH
    rib_centroid_z = -p.TRAY_RIB_DEPTH / 2.0
    neutral_axis_z = (
        skin_area * skin_centroid_z
        + rib_count * rib_area_each * rib_centroid_z
    ) / (skin_area + rib_count * rib_area_each)
    second_moment = (
        p.TRAY_DEPTH * p.TRAY_SKIN ** 3 / 12.0
        + skin_area * (skin_centroid_z - neutral_axis_z) ** 2
        + rib_count * (
            p.TRAY_RIB_THICKNESS * p.TRAY_RIB_DEPTH ** 3 / 12.0
            + rib_area_each * (rib_centroid_z - neutral_axis_z) ** 2
        )
    )
    extreme_fiber = max(
        p.TRAY_SKIN - neutral_axis_z,
        neutral_axis_z + p.TRAY_RIB_DEPTH,
    )
    tray_moment = hand_load_n * p.TRAY_WIDTH / 4.0
    tray_stress = tray_moment * extreme_fiber / second_moment
    tray_deflection = (
        hand_load_n * p.TRAY_WIDTH ** 3
        / (48.0 * elastic_modulus_mpa * second_moment)
    )

    side_load = hand_load_n / 2.0
    carriage_lever = 60.0
    carriage_depth = 12.0
    carriage_thickness = 5.0
    carriage_i = carriage_thickness * carriage_depth ** 3 / 12.0
    carriage_stress = side_load * carriage_lever * (carriage_depth / 2.0) / carriage_i
    return {
        "method": "simply-supported composite beam screen",
        "material_allowable_mpa": allowable_stress_mpa,
        "elastic_modulus_mpa": elastic_modulus_mpa,
        "hand_load_n": hand_load_n,
        "tray_second_moment_mm4": second_moment,
        "tray_bending_stress_mpa": tray_stress,
        "tray_bending_safety_factor": allowable_stress_mpa / tray_stress,
        "tray_center_deflection_mm": tray_deflection,
        "side_carriage_stress_mpa": carriage_stress,
        "side_carriage_safety_factor": allowable_stress_mpa / carriage_stress,
        "caveat": "first-order screen only; layer adhesion and fatigue require prototype testing",
    }


def validate_all(motion_samples: int = 401, collision_samples: int = 101) -> Dict:
    motion = mech.validate_motion(motion_samples)
    collisions = validate_collisions(collision_samples)
    stability = validate_stability()
    spring = validate_spring()
    printability = validate_printability()
    plate_layouts = validate_plate_layouts()
    strength = validate_strength()
    interface_reports = [
        assembly.build_pose(travel).interface_report()
        for travel in (0.0, 0.5, 1.0)
    ]
    errors = []
    if motion["max_cross_error_mm"] >= 0.01 or not motion["monotonic"]:
        errors.append({"motion": motion})
    if collisions["collisions"]:
        errors.append({"collisions": collisions["collisions"]})
    if stability["minimum_margin_mm"] < p.STABILITY_MARGIN:
        errors.append({"stability": stability})
    if not spring["returns_with_10_percent_mismatch"] or spring["maximum_user_force_n"] > 20.0:
        errors.append({"spring": spring})
    if spring["coil_bind_margin_mm"] < p.SPRING_COIL_BIND_SAFETY:
        errors.append({"spring_coil_bind": spring})
    if printability["failures"]:
        errors.append({"printability": printability["failures"]})
    if plate_layouts["failures"]:
        errors.append({"plate_layouts": plate_layouts["failures"]})
    if (
        strength["tray_bending_safety_factor"] < 2.0
        or strength["tray_center_deflection_mm"] > 3.0
        or strength["side_carriage_safety_factor"] < 2.0
    ):
        errors.append({"strength_screen": strength})
    disconnected = [
        item
        for report in interface_reports
        for item in report["disconnected"]
    ]
    if disconnected:
        errors.append({"interfaces": disconnected})
    return {
        "motion": motion,
        "collision": collisions,
        "stability": stability,
        "spring": spring,
        "printability": printability,
        "plate_layouts": plate_layouts,
        "strength": strength,
        "interfaces": interface_reports,
        "clearance": {
            "bearing_to_slot_end_mm": min(
                motion["min_cross_end_margin_mm"],
                5.0,
            ),
            "hard_stop_before_slot_end_mm": 3.0,
        },
        "errors": errors,
    }
