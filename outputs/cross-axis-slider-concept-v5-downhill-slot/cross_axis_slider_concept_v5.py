"""V5 downhill-slot linkage for the keyboard / trackpad cross-axis concept.

Independent output variant. V1-V4 sources and generated artifacts remain unchanged.
Coordinate system: X left/right, +Y toward display, +Z up. Units are mm.
"""

from math import atan2, degrees, hypot, radians, tan
from pathlib import Path
import sys

from build123d import Color, Compound


V3_DIR = Path(__file__).resolve().parents[1] / "cross-axis-slider-concept-v3-compact-base"
if str(V3_DIR) not in sys.path:
    sys.path.insert(0, str(V3_DIR))

import cross_axis_slider_concept_v3 as v3  # noqa: E402

v1 = v3.v1


# Confirmed v5 motion: a visibly downhill, still-straight captured slot.
SLOT_ANGLE_DEG = 3.0
RELATIVE_DY = v1.KEYBOARD_DY - v1.TRACKPAD_DY
RELATIVE_DZ = RELATIVE_DY * tan(radians(SLOT_ANGLE_DEG))
TRACKPAD_DZ_V5 = v1.KEYBOARD_DZ - RELATIVE_DZ

# Captured linkage axial stack (left side only).
DRIVER_X = -142.5
DRIVER_THICKNESS = 4.0
OUTER_CHEEK_X = -147.0
INNER_CHEEK_X = -138.0
CHEEK_THICKNESS = 3.0

CAPTURED_SLOT_LENGTH = 170.0
CAPTURED_SLOT_WIDTH = 7.2
CAPTURED_LINK_WIDTH = 14.0
ROLLER_OD = 6.0
ROLLER_BORE = 4.0
ROLLER_THICKNESS = 2.8
SHOULDER_SHAFT_DIAMETER = 3.6
SHOULDER_SHAFT_LENGTH = 13.5
SPACER_OD = 7.6
SPACER_THICKNESS = 0.8

# One long captured shoe replaces each pair of red keyboard guide dots.
GUIDE_SHOE_LENGTH = 28.0
GUIDE_SHOE_WIDTH = 4.6
GUIDE_SHOE_THICKNESS_X = 7.0

TRACKPAD_BLUE = Color(0.10, 0.38, 0.92)
KEYBOARD_RED = Color(0.88, 0.16, 0.13)
ROLLER_GOLD = Color(0.96, 0.65, 0.12)
HARDWARE_SILVER = Color(0.72, 0.75, 0.78)
GUIDE_SHOE_GRAY = Color(0.48, 0.51, 0.54)


def _slot_geometry(key_y, key_z, track_y, track_z):
    """Derive the slot from the exact relative endpoint displacement."""
    pin_y = key_y + 30.0
    pin_z = key_z + 21.0
    slot_center_y0 = (v1.KEYBOARD_Y0 + 30.0) + RELATIVE_DY / 2.0
    slot_center_z0 = (v1.KEYBOARD_Z0 + 21.0) + RELATIVE_DZ / 2.0
    slot_center_y = slot_center_y0 + (track_y - v1.TRACKPAD_Y0)
    slot_center_z = slot_center_z0 + (track_z - v1.TRACKPAD_Z0)
    return pin_y, pin_z, slot_center_y, slot_center_z, SLOT_ANGLE_DEG


def _keyboard_driver(key_y, key_z, pin_y, pin_z):
    anchor_front = (key_y - 18.0, key_z + 1.0)
    anchor_rear = (key_y + 8.0, key_z + 1.0)
    driver = v1._beam_between_yz(
        anchor_front, anchor_rear, 9.0, DRIVER_THICKNESS, DRIVER_X
    )
    driver = driver + v1._beam_between_yz(
        anchor_front, (pin_y, pin_z), 10.0, DRIVER_THICKNESS, DRIVER_X
    )
    driver = driver + v1._beam_between_yz(
        anchor_rear, (pin_y, pin_z), 10.0, DRIVER_THICKNESS, DRIVER_X
    )
    driver.label = "keyboard_two_point_driver_left"
    driver.color = KEYBOARD_RED
    return driver


def _trackpad_cheek(cheek_x, cheek_name, track_y, track_z,
                    slot_center_y, slot_center_z, slot_angle):
    cheek = v1._slotted_beam(
        CAPTURED_SLOT_LENGTH + 14.0,
        CAPTURED_LINK_WIDTH,
        CAPTURED_SLOT_LENGTH,
        CAPTURED_SLOT_WIDTH,
        CHEEK_THICKNESS,
        slot_center_y,
        slot_center_z,
        slot_angle,
        cheek_x,
    )

    # Two independent necks make the slotted member rigid with the trackpad carriage.
    slope = RELATIVE_DZ / RELATIVE_DY
    anchor_front = (track_y - 15.0, track_z - 2.0)
    anchor_rear = (track_y + 15.0, track_z - 2.0)
    attach_front = (slot_center_y + 4.0, slot_center_z + 4.0 * slope)
    attach_rear = (slot_center_y + 38.0, slot_center_z + 38.0 * slope)
    cheek = cheek + v1._beam_between_yz(
        attach_front, anchor_front, 10.0, CHEEK_THICKNESS, cheek_x
    )
    cheek = cheek + v1._beam_between_yz(
        attach_rear, anchor_rear, 10.0, CHEEK_THICKNESS, cheek_x
    )
    cheek.label = cheek_name
    cheek.color = TRACKPAD_BLUE
    return cheek


def _roller_ring(center_x, pin_y, pin_z, label):
    roller = v1._x_cylinder(
        ROLLER_OD / 2.0, ROLLER_THICKNESS, center_x, pin_y, pin_z
    )
    bore = v1._x_cylinder(
        ROLLER_BORE / 2.0, ROLLER_THICKNESS + 1.0, center_x, pin_y, pin_z
    )
    roller = roller - bore
    roller.label = label
    roller.color = ROLLER_GOLD
    return roller


def _captured_linkage(key_y, key_z, track_y, track_z):
    pin_y, pin_z, slot_y, slot_z, slot_angle = _slot_geometry(
        key_y, key_z, track_y, track_z
    )
    children = [
        _keyboard_driver(key_y, key_z, pin_y, pin_z),
        _trackpad_cheek(
            OUTER_CHEEK_X,
            "trackpad_downhill_slotted_cheek_outer_left",
            track_y,
            track_z,
            slot_y,
            slot_z,
            slot_angle,
        ),
        _trackpad_cheek(
            INNER_CHEEK_X,
            "trackpad_downhill_slotted_cheek_inner_left",
            track_y,
            track_z,
            slot_y,
            slot_z,
            slot_angle,
        ),
    ]

    shaft = v1._x_cylinder(
        SHOULDER_SHAFT_DIAMETER / 2.0,
        SHOULDER_SHAFT_LENGTH,
        DRIVER_X,
        pin_y,
        pin_z,
    )
    shaft.label = "cross_axis_shoulder_shaft_left"
    shaft.color = HARDWARE_SILVER
    children.append(shaft)

    children.append(_roller_ring(
        OUTER_CHEEK_X, pin_y, pin_z, "cross_axis_roller_outer_left"
    ))
    children.append(_roller_ring(
        INNER_CHEEK_X, pin_y, pin_z, "cross_axis_roller_inner_left"
    ))

    for x, label in ((-145.0, "cross_axis_spacer_outer_left"),
                     (-140.0, "cross_axis_spacer_inner_left")):
        spacer = v1._x_cylinder(
            SPACER_OD / 2.0, SPACER_THICKNESS, x, pin_y, pin_z
        )
        spacer.label = label
        spacer.color = HARDWARE_SILVER
        children.append(spacer)

    for index, anchor_y in enumerate((track_y - 15.0, track_y + 15.0), 1):
        tie = v1._x_cylinder(3.0, 12.0, DRIVER_X, anchor_y, track_z - 2.0)
        tie.label = f"trackpad_cheek_tie_left_{index}"
        tie.color = TRACKPAD_BLUE
        children.append(tie)

    return children


def _keyboard_guide_shoes(travel):
    """Long gray captured sliders replace visually ambiguous red roller pairs."""
    t = max(0.0, min(1.0, float(travel)))
    travel_len = hypot(v1.KEYBOARD_DY, v1.KEYBOARD_DZ)
    angle = degrees(atan2(v1.KEYBOARD_DZ, v1.KEYBOARD_DY))
    center_y = v1.KEYBOARD_Y0 + t * v1.KEYBOARD_DY
    center_z = v1.KEYBOARD_Z0 + 8.0 + t * v1.KEYBOARD_DZ
    shoes = []
    for side_name, sign in (("left", -1), ("right", 1)):
        shoe = v1._capsule_yz(
            GUIDE_SHOE_LENGTH,
            GUIDE_SHOE_WIDTH,
            GUIDE_SHOE_THICKNESS_X,
            center_y,
            center_z,
            angle,
            sign * 148.5,
        )
        shoe.label = f"keyboard_captured_guide_shoe_{side_name}"
        shoe.color = GUIDE_SHOE_GRAY
        shoes.append(shoe)

    # Source-level sanity: the shoe is meaningful only inside the keyboard rail.
    assert GUIDE_SHOE_LENGTH <= v1.GUIDE_PIN_SPACING + 1e-9
    assert GUIDE_SHOE_WIDTH < v1.GUIDE_SLOT_WIDTH
    assert travel_len > 0.0
    return shoes


def kinematic_sample(travel):
    t = max(0.0, min(1.0, float(travel)))
    key_y = v1.KEYBOARD_Y0 + t * v1.KEYBOARD_DY
    key_z = v1.KEYBOARD_Z0 + t * v1.KEYBOARD_DZ
    track_y = v1.TRACKPAD_Y0 + t * v1.TRACKPAD_DY
    track_z = v1.TRACKPAD_Z0 + t * TRACKPAD_DZ_V5
    pin_y, pin_z, slot_y, slot_z, _ = _slot_geometry(
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
    center_limit = CAPTURED_SLOT_LENGTH / 2.0 - ROLLER_OD / 2.0
    return {
        "travel": t,
        "along": along,
        "cross": cross,
        "roller_center_margin": center_limit - abs(along),
    }


def validate_kinematics(sample_count=201):
    samples = [kinematic_sample(i / (sample_count - 1)) for i in range(sample_count)]
    return {
        "sample_count": sample_count,
        "slot_angle_deg": SLOT_ANGLE_DEG,
        "trackpad_dz": TRACKPAD_DZ_V5,
        "max_cross_error": max(abs(sample["cross"]) for sample in samples),
        "minimum_roller_center_margin": min(
            sample["roller_center_margin"] for sample in samples
        ),
        "monotonic": all(
            samples[i + 1]["along"] > samples[i]["along"]
            for i in range(len(samples) - 1)
        ),
    }


def build_pose(travel=0.0, pose_name="keyboard_mode"):
    t = max(0.0, min(1.0, float(travel)))

    # Regenerate the inherited fixed trackpad guides from the v5 endpoint.
    original_trackpad_dz = v1.TRACKPAD_DZ
    v1.TRACKPAD_DZ = TRACKPAD_DZ_V5
    try:
        original = v3.build_pose(t, pose_name)
    finally:
        v1.TRACKPAD_DZ = original_trackpad_dz

    obsolete_prefixes = (
        "keyboard_drive_arm_",
        "trackpad_slotted_link_",
        "cross_axis_roller_pin_",
        "keyboard_guide_roller_",
    )
    children = [
        child for child in original.children
        if not any((child.label or "").startswith(prefix) for prefix in obsolete_prefixes)
    ]

    key_y = v1.KEYBOARD_Y0 + t * v1.KEYBOARD_DY
    key_z = v1.KEYBOARD_Z0 + t * v1.KEYBOARD_DZ
    track_y = v1.TRACKPAD_Y0 + t * v1.TRACKPAD_DY
    track_z = v1.TRACKPAD_Z0 + t * TRACKPAD_DZ_V5
    children.extend(_keyboard_guide_shoes(t))
    children.extend(_captured_linkage(key_y, key_z, track_y, track_z))

    return Compound(
        label=f"cross_axis_slider_concept_v5_downhill_slot_{pose_name}",
        children=children,
    )

