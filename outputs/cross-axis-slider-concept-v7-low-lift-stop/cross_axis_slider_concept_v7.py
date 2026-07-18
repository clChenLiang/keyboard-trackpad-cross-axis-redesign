"""V7 low-lift terminal-stop variant of the symmetric cross-axis concept.

Independent output variant. V1-V6 sources and artifacts remain unchanged.
Coordinate system: X left/right, +Y toward display, +Z up. Units are mm.
"""

from math import atan2, degrees, hypot
from pathlib import Path
import sys

from build123d import Align, Box, Compound, Pos


V6_DIR = Path(__file__).resolve().parents[1] / "cross-axis-slider-concept-v6-symmetric-spring"
if str(V6_DIR) not in sys.path:
    sys.path.insert(0, str(V6_DIR))

import cross_axis_slider_concept_v6 as v6  # noqa: E402

v1 = v6.v1


TRACKPAD_DZ_V7 = -43.0
RELATIVE_DY = v1.KEYBOARD_DY - v1.TRACKPAD_DY
RELATIVE_DZ = v1.KEYBOARD_DZ - TRACKPAD_DZ_V7
SLOT_ANGLE_DEG = degrees(atan2(RELATIVE_DZ, RELATIVE_DY))
PIN_Z_OFFSET = 37.0

STOP_CORE_LENGTH = 10.0
STOP_CORE_DIAMETER = 1.8
MIN_MOVING_Z = 6.0


def _slot_geometry(key_y, key_z, track_y, track_z):
    pin_y = key_y + 30.0
    pin_z = key_z + PIN_Z_OFFSET
    slot_center_y0 = (v1.KEYBOARD_Y0 + 30.0) + RELATIVE_DY / 2.0
    slot_center_z0 = (v1.KEYBOARD_Z0 + PIN_Z_OFFSET) + RELATIVE_DZ / 2.0
    slot_center_y = slot_center_y0 + (track_y - v1.TRACKPAD_Y0)
    slot_center_z = slot_center_z0 + (track_z - v1.TRACKPAD_Z0)
    return pin_y, pin_z, slot_center_y, slot_center_z


def _raised_keyboard_carriages(key_y, key_z):
    children = []
    for side_name, sign in (("left", -1), ("right", 1)):
        ear = Box(
            7.0,
            46.0,
            8.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).moved(Pos(sign * v1.TRAY_EAR_X, key_y, key_z + 3.0))
        ear.label = f"keyboard_side_carriage_{side_name}"
        ear.color = v6.KEYBOARD_RED
        children.append(ear)
    return children


def _raised_keyboard_driver(side_name, side_sign, key_y, key_z, pin_y, pin_z):
    driver_x = side_sign * v6.DRIVER_X_ABS
    anchor_front = (key_y - 18.0, key_z + 4.0)
    anchor_rear = (key_y + 8.0, key_z + 4.0)
    driver = v1._beam_between_yz(
        anchor_front, anchor_rear, 9.0, v6.DRIVER_THICKNESS, driver_x
    )
    driver = driver + v1._beam_between_yz(
        anchor_front, (pin_y, pin_z), 10.0, v6.DRIVER_THICKNESS, driver_x
    )
    driver = driver + v1._beam_between_yz(
        anchor_rear, (pin_y, pin_z), 10.0, v6.DRIVER_THICKNESS, driver_x
    )
    driver.label = f"keyboard_two_point_driver_{side_name}"
    driver.color = v6.KEYBOARD_RED
    return driver


def _raised_shoe_connectors(key_y, key_z):
    children = []
    shoe_y = key_y
    shoe_z = key_z + 8.0
    for side_name, sign in (("left_datum", -1), ("right_floating", 1)):
        connector = v1._beam_between_yz(
            (shoe_y, shoe_z),
            (key_y, key_z + 3.0),
            4.0,
            6.0,
            sign * 145.5,
        )
        connector.label = f"keyboard_guide_shoe_connector_{side_name}"
        connector.color = v6.GUIDE_SHOE_GRAY
        children.append(connector)
    return children


def _terminal_stop_cores():
    _, _, fixed_y, fixed_z, _ = v6._keyboard_guide_axis_geometry()
    core_end = (
        fixed_y - STOP_CORE_LENGTH * v6.KEYBOARD_UY,
        fixed_z - STOP_CORE_LENGTH * v6.KEYBOARD_UZ,
    )
    children = []
    for side_name, sign in (("left_datum", -1), ("right_floating", 1)):
        core = v1._beam_between_yz(
            (fixed_y, fixed_z),
            core_end,
            STOP_CORE_DIAMETER,
            STOP_CORE_DIAMETER,
            sign * v6.v3.COMPACT_GUIDE_RAIL_X,
        )
        core.label = f"keyboard_terminal_stop_core_{side_name}"
        core.color = v6.HARDWARE_SILVER
        children.append(core)
    return children


def _build_v6_with_v7_motion(travel, pose_name):
    old_values = (
        v6.TRACKPAD_DZ_V6,
        v6.RELATIVE_DZ,
        v6.SLOT_ANGLE_DEG,
        v6._slot_geometry,
    )
    v6.TRACKPAD_DZ_V6 = TRACKPAD_DZ_V7
    v6.RELATIVE_DZ = RELATIVE_DZ
    v6.SLOT_ANGLE_DEG = SLOT_ANGLE_DEG
    v6._slot_geometry = _slot_geometry
    try:
        return v6.build_pose(travel, pose_name)
    finally:
        (
            v6.TRACKPAD_DZ_V6,
            v6.RELATIVE_DZ,
            v6.SLOT_ANGLE_DEG,
            v6._slot_geometry,
        ) = old_values


def kinematic_sample(travel):
    t = max(0.0, min(1.0, float(travel)))
    key_y = v1.KEYBOARD_Y0 + t * v1.KEYBOARD_DY
    key_z = v1.KEYBOARD_Z0 + t * v1.KEYBOARD_DZ
    track_y = v1.TRACKPAD_Y0 + t * v1.TRACKPAD_DY
    track_z = v1.TRACKPAD_Z0 + t * TRACKPAD_DZ_V7
    pin_y, pin_z, slot_y, slot_z = _slot_geometry(
        key_y, key_z, track_y, track_z
    )
    rel_len = hypot(RELATIVE_DY, RELATIVE_DZ)
    along = (
        (pin_y - slot_y) * RELATIVE_DY
        + (pin_z - slot_z) * RELATIVE_DZ
    ) / rel_len
    cross = (
        (pin_y - slot_y) * RELATIVE_DZ
        - (pin_z - slot_z) * RELATIVE_DY
    ) / rel_len
    margin = v6.CAPTURED_SLOT_LENGTH / 2.0 - v6.ROLLER_OD / 2.0 - abs(along)
    return {
        "travel": t,
        "along": along,
        "left_cross": cross,
        "right_cross": cross,
        "left_margin": margin,
        "right_margin": margin,
        "spring_length": v6.SPRING_FREE_LENGTH - t * v6.KEYBOARD_PATH,
    }


def validate_kinematics(sample_count=201):
    samples = [kinematic_sample(i / (sample_count - 1)) for i in range(sample_count)]
    return {
        "sample_count": sample_count,
        "slot_angle_deg": SLOT_ANGLE_DEG,
        "trackpad_dz": TRACKPAD_DZ_V7,
        "max_cross_error": max(
            max(abs(s["left_cross"]), abs(s["right_cross"])) for s in samples
        ),
        "minimum_roller_center_margin": min(
            min(s["left_margin"], s["right_margin"]) for s in samples
        ),
        "left_roller_monotonic": all(
            samples[i + 1]["along"] > samples[i]["along"]
            for i in range(len(samples) - 1)
        ),
        "right_roller_monotonic": all(
            samples[i + 1]["along"] > samples[i]["along"]
            for i in range(len(samples) - 1)
        ),
        "spring_monotonic": all(
            samples[i + 1]["spring_length"] < samples[i]["spring_length"]
            for i in range(len(samples) - 1)
        ),
        "spring_final_length": samples[-1]["spring_length"],
    }


MOVING_LABEL_PREFIXES = (
    "keyboard_tray_",
    "trackpad_tray_",
    "keyboard_side_carriage_",
    "trackpad_side_carriage_",
    "keyboard_two_point_driver_",
    "keyboard_captured_guide_shoe_",
    "keyboard_guide_shoe_connector_",
    "keyboard_compression_spring_",
    "trackpad_guide_roller_",
    "trackpad_low_lift_slotted_cheek_",
    "cross_axis_roller_",
    "cross_axis_shoulder_shaft_",
    "cross_axis_spacer_",
    "trackpad_cheek_tie_",
)


def moving_clearance_facts(pose):
    moving = [
        child for child in pose.children
        if any((child.label or "").startswith(prefix) for prefix in MOVING_LABEL_PREFIXES)
    ]
    lowest = min(moving, key=lambda child: child.bounding_box().min.Z)
    return {
        "moving_count": len(moving),
        "minimum_moving_z": lowest.bounding_box().min.Z,
        "minimum_clearance_above_base_top": lowest.bounding_box().min.Z - v1.BASE_THICKNESS,
        "lowest_label": lowest.label,
    }


def build_pose(travel=0.0, pose_name="keyboard_mode"):
    t = max(0.0, min(1.0, float(travel)))
    original = _build_v6_with_v7_motion(t, pose_name)

    replace_prefixes = (
        "keyboard_side_carriage_",
        "keyboard_two_point_driver_",
        "keyboard_guide_shoe_connector_",
    )
    children = [
        child for child in original.children
        if not any((child.label or "").startswith(prefix) for prefix in replace_prefixes)
    ]
    for child in children:
        if (child.label or "").startswith("trackpad_downhill_slotted_cheek_"):
            child.label = child.label.replace(
                "trackpad_downhill_slotted_cheek_",
                "trackpad_low_lift_slotted_cheek_",
                1,
            )

    key_y = v1.KEYBOARD_Y0 + t * v1.KEYBOARD_DY
    key_z = v1.KEYBOARD_Z0 + t * v1.KEYBOARD_DZ
    track_y = v1.TRACKPAD_Y0 + t * v1.TRACKPAD_DY
    track_z = v1.TRACKPAD_Z0 + t * TRACKPAD_DZ_V7
    pin_y, pin_z, _, _ = _slot_geometry(key_y, key_z, track_y, track_z)

    children.extend(_raised_keyboard_carriages(key_y, key_z))
    children.extend(_raised_shoe_connectors(key_y, key_z))
    children.append(_raised_keyboard_driver(
        "left_datum", -1, key_y, key_z, pin_y, pin_z
    ))
    children.append(_raised_keyboard_driver(
        "right_floating", 1, key_y, key_z, pin_y, pin_z
    ))
    children.extend(_terminal_stop_cores())

    return Compound(
        label=f"cross_axis_slider_concept_v7_low_lift_stop_{pose_name}",
        children=children,
    )
