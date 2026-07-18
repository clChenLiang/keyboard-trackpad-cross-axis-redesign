"""V6 symmetric spring-return cross-axis slider concept.

Independent output variant. V1-V5 sources and artifacts remain unchanged.
Coordinate system: X left/right, +Y toward display, +Z up. Units are mm.
"""

from math import atan2, degrees, hypot, radians, tan
from pathlib import Path
import sys

from build123d import Color, Compound, Edge, Face, Plane, Solid, Wire


V3_DIR = Path(__file__).resolve().parents[1] / "cross-axis-slider-concept-v3-compact-base"
if str(V3_DIR) not in sys.path:
    sys.path.insert(0, str(V3_DIR))

import cross_axis_slider_concept_v3 as v3  # noqa: E402

v1 = v3.v1


# Constraint-derived v6 motion.
SLOT_ANGLE_DEG = -3.0
RELATIVE_DY = v1.KEYBOARD_DY - v1.TRACKPAD_DY
RELATIVE_DZ = RELATIVE_DY * tan(radians(SLOT_ANGLE_DEG))
TRACKPAD_DZ_V6 = v1.KEYBOARD_DZ - RELATIVE_DZ

KEYBOARD_PATH = hypot(v1.KEYBOARD_DY, v1.KEYBOARD_DZ)
KEYBOARD_UY = v1.KEYBOARD_DY / KEYBOARD_PATH
KEYBOARD_UZ = v1.KEYBOARD_DZ / KEYBOARD_PATH
KEYBOARD_GUIDE_ANGLE_DEG = degrees(atan2(v1.KEYBOARD_DZ, v1.KEYBOARD_DY))

# Bilateral captured linkage.
DRIVER_X_ABS = 142.5
DRIVER_THICKNESS = 4.0
OUTER_CHEEK_X_ABS = 147.0
INNER_CHEEK_X_ABS = 138.0
CHEEK_THICKNESS = 3.0
RIGHT_AXIAL_FLOAT = 0.7

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

# Keyboard guide and coaxial compression spring.
GUIDE_SHOE_LENGTH = 14.0
GUIDE_SHOE_WIDTH = 4.6
LEFT_SHOE_THICKNESS_X = 7.0
RIGHT_SHOE_THICKNESS_X = 6.2
SPRING_REAR_CLEARANCE = 4.0
SPRING_COMPRESSED_LENGTH = 10.0
SPRING_FREE_LENGTH = KEYBOARD_PATH + SPRING_COMPRESSED_LENGTH
KEYBOARD_GUIDE_SLOT_LENGTH = (
    KEYBOARD_PATH
    + GUIDE_SHOE_LENGTH
    + SPRING_REAR_CLEARANCE
    + SPRING_COMPRESSED_LENGTH
)
KEYBOARD_GUIDE_BODY_LENGTH = KEYBOARD_GUIDE_SLOT_LENGTH + 6.0
SPRING_COIL_RADIUS = 1.6
SPRING_WIRE_RADIUS = 0.45
SPRING_TURNS = 7

TRACKPAD_BLUE = Color(0.10, 0.38, 0.92)
KEYBOARD_RED = Color(0.88, 0.16, 0.13)
ROLLER_GOLD = Color(0.96, 0.65, 0.12)
HARDWARE_SILVER = Color(0.72, 0.75, 0.78)
GUIDE_SHOE_GRAY = Color(0.48, 0.51, 0.54)
SPRING_METAL = Color(0.70, 0.58, 0.34)


def _slot_geometry(key_y, key_z, track_y, track_z):
    """Derive the straight slot from the exact relative displacement."""
    pin_y = key_y + 30.0
    pin_z = key_z + 21.0
    slot_center_y0 = (v1.KEYBOARD_Y0 + 30.0) + RELATIVE_DY / 2.0
    slot_center_z0 = (v1.KEYBOARD_Z0 + 21.0) + RELATIVE_DZ / 2.0
    slot_center_y = slot_center_y0 + (track_y - v1.TRACKPAD_Y0)
    slot_center_z = slot_center_z0 + (track_z - v1.TRACKPAD_Z0)
    return pin_y, pin_z, slot_center_y, slot_center_z


def _side_stack_x(side_sign, axial_float):
    """Return explicit X layers for datum or floating captured stacks."""
    driver_x = side_sign * DRIVER_X_ABS
    outer_abs = OUTER_CHEEK_X_ABS + axial_float / 2.0
    inner_abs = INNER_CHEEK_X_ABS - axial_float / 2.0
    outer_x = side_sign * outer_abs
    inner_x = side_sign * inner_abs

    outer_spacer_abs = (
        DRIVER_X_ABS + DRIVER_THICKNESS / 2.0
        + outer_abs - CHEEK_THICKNESS / 2.0
    ) / 2.0
    inner_spacer_abs = (
        DRIVER_X_ABS - DRIVER_THICKNESS / 2.0
        + inner_abs + CHEEK_THICKNESS / 2.0
    ) / 2.0
    return {
        "driver": driver_x,
        "outer_cheek": outer_x,
        "inner_cheek": inner_x,
        "outer_spacer": side_sign * outer_spacer_abs,
        "inner_spacer": side_sign * inner_spacer_abs,
    }


def _keyboard_driver(side_name, driver_x, key_y, key_z, pin_y, pin_z):
    anchor_front = (key_y - 18.0, key_z + 1.0)
    anchor_rear = (key_y + 8.0, key_z + 1.0)
    driver = v1._beam_between_yz(
        anchor_front, anchor_rear, 9.0, DRIVER_THICKNESS, driver_x
    )
    driver = driver + v1._beam_between_yz(
        anchor_front, (pin_y, pin_z), 10.0, DRIVER_THICKNESS, driver_x
    )
    driver = driver + v1._beam_between_yz(
        anchor_rear, (pin_y, pin_z), 10.0, DRIVER_THICKNESS, driver_x
    )
    driver.label = f"keyboard_two_point_driver_{side_name}"
    driver.color = KEYBOARD_RED
    return driver


def _trackpad_cheek(cheek_x, label, track_y, track_z, slot_y, slot_z):
    cheek = v1._slotted_beam(
        CAPTURED_SLOT_LENGTH + 14.0,
        CAPTURED_LINK_WIDTH,
        CAPTURED_SLOT_LENGTH,
        CAPTURED_SLOT_WIDTH,
        CHEEK_THICKNESS,
        slot_y,
        slot_z,
        SLOT_ANGLE_DEG,
        cheek_x,
    )

    slope = RELATIVE_DZ / RELATIVE_DY
    anchor_front = (track_y - 15.0, track_z - 2.0)
    anchor_rear = (track_y + 15.0, track_z - 2.0)
    attach_front = (slot_y + 4.0, slot_z + 4.0 * slope)
    attach_rear = (slot_y + 38.0, slot_z + 38.0 * slope)
    cheek = cheek + v1._beam_between_yz(
        attach_front, anchor_front, 10.0, CHEEK_THICKNESS, cheek_x
    )
    cheek = cheek + v1._beam_between_yz(
        attach_rear, anchor_rear, 10.0, CHEEK_THICKNESS, cheek_x
    )
    cheek.label = label
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


def _captured_linkage_side(
    side_name, side_sign, axial_float, key_y, key_z, track_y, track_z
):
    pin_y, pin_z, slot_y, slot_z = _slot_geometry(
        key_y, key_z, track_y, track_z
    )
    stack = _side_stack_x(side_sign, axial_float)
    children = [
        _keyboard_driver(
            side_name, stack["driver"], key_y, key_z, pin_y, pin_z
        ),
        _trackpad_cheek(
            stack["outer_cheek"],
            f"trackpad_downhill_slotted_cheek_outer_{side_name}",
            track_y,
            track_z,
            slot_y,
            slot_z,
        ),
        _trackpad_cheek(
            stack["inner_cheek"],
            f"trackpad_downhill_slotted_cheek_inner_{side_name}",
            track_y,
            track_z,
            slot_y,
            slot_z,
        ),
    ]

    shaft = v1._x_cylinder(
        SHOULDER_SHAFT_DIAMETER / 2.0,
        SHOULDER_SHAFT_LENGTH + axial_float,
        stack["driver"],
        pin_y,
        pin_z,
    )
    shaft.label = f"cross_axis_shoulder_shaft_{side_name}"
    shaft.color = HARDWARE_SILVER
    children.append(shaft)

    children.append(_roller_ring(
        stack["outer_cheek"],
        pin_y,
        pin_z,
        f"cross_axis_roller_outer_{side_name}",
    ))
    children.append(_roller_ring(
        stack["inner_cheek"],
        pin_y,
        pin_z,
        f"cross_axis_roller_inner_{side_name}",
    ))

    for role in ("outer", "inner"):
        spacer = v1._x_cylinder(
            SPACER_OD / 2.0,
            SPACER_THICKNESS,
            stack[f"{role}_spacer"],
            pin_y,
            pin_z,
        )
        spacer.label = f"cross_axis_spacer_{role}_{side_name}"
        spacer.color = HARDWARE_SILVER
        children.append(spacer)

    for index, anchor_y in enumerate((track_y - 15.0, track_y + 15.0), 1):
        tie = v1._x_cylinder(
            3.0,
            12.0 + axial_float,
            stack["driver"],
            anchor_y,
            track_z - 2.0,
        )
        tie.label = f"trackpad_cheek_tie_{side_name}_{index}"
        tie.color = TRACKPAD_BLUE
        children.append(tie)

    return children


def _keyboard_guide_axis_geometry():
    pin_y0 = v1.KEYBOARD_Y0
    pin_z0 = v1.KEYBOARD_Z0 + 8.0
    rear_q = -GUIDE_SHOE_LENGTH / 2.0 - SPRING_REAR_CLEARANCE
    fixed_q = KEYBOARD_PATH + GUIDE_SHOE_LENGTH / 2.0 + SPRING_COMPRESSED_LENGTH
    center_q = (rear_q + fixed_q) / 2.0
    center_y = pin_y0 + center_q * KEYBOARD_UY
    center_z = pin_z0 + center_q * KEYBOARD_UZ
    fixed_y = pin_y0 + fixed_q * KEYBOARD_UY
    fixed_z = pin_z0 + fixed_q * KEYBOARD_UZ
    return center_y, center_z, fixed_y, fixed_z, fixed_q


def _compression_spring(side_name, center_x, moving_front_y, moving_front_z,
                        fixed_y, fixed_z, spring_length):
    end_gap = 1.0
    coil_height = spring_length - 2.0 * end_gap
    start = (
        center_x,
        moving_front_y + end_gap * KEYBOARD_UY,
        moving_front_z + end_gap * KEYBOARD_UZ,
    )
    path = Edge.make_helix(
        coil_height / SPRING_TURNS,
        coil_height,
        SPRING_COIL_RADIUS,
        center=start,
        normal=(0.0, KEYBOARD_UY, KEYBOARD_UZ),
    )
    profile = Face(Wire.make_circle(
        SPRING_WIRE_RADIUS,
        Plane(path.position_at(0), z_dir=path.tangent_at(0)),
    ))
    spring = Solid.sweep(profile, path, is_frenet=True)
    spring.label = f"keyboard_compression_spring_{side_name}"
    spring.color = SPRING_METAL

    seat = v1._capsule_yz(
        6.0,
        5.0,
        6.0,
        fixed_y,
        fixed_z,
        KEYBOARD_GUIDE_ANGLE_DEG + 90.0,
        center_x,
    )
    seat.label = f"keyboard_spring_fixed_seat_{side_name}"
    seat.color = HARDWARE_SILVER
    return spring, seat


def _keyboard_guides_and_springs(travel):
    t = max(0.0, min(1.0, float(travel)))
    rail_y, rail_z, fixed_y, fixed_z, _ = _keyboard_guide_axis_geometry()
    shoe_y = v1.KEYBOARD_Y0 + t * v1.KEYBOARD_DY
    shoe_z = v1.KEYBOARD_Z0 + 8.0 + t * v1.KEYBOARD_DZ
    spring_length = SPRING_FREE_LENGTH - t * KEYBOARD_PATH
    moving_front_y = shoe_y + GUIDE_SHOE_LENGTH / 2.0 * KEYBOARD_UY
    moving_front_z = shoe_z + GUIDE_SHOE_LENGTH / 2.0 * KEYBOARD_UZ

    children = []
    for side_name, sign, shoe_thickness in (
        ("left_datum", -1, LEFT_SHOE_THICKNESS_X),
        ("right_floating", 1, RIGHT_SHOE_THICKNESS_X),
    ):
        rail_x = sign * v3.COMPACT_GUIDE_RAIL_X
        rail = v1._slotted_beam(
            KEYBOARD_GUIDE_BODY_LENGTH,
            v1.GUIDE_BODY_WIDTH,
            KEYBOARD_GUIDE_SLOT_LENGTH,
            v1.GUIDE_SLOT_WIDTH,
            v1.GUIDE_THICKNESS,
            rail_y,
            rail_z,
            KEYBOARD_GUIDE_ANGLE_DEG,
            rail_x,
        )
        rail.label = f"keyboard_extended_fixed_guide_{side_name}"
        rail.color = v1.GUIDE_COLOR
        children.append(rail)

        shoe = v1._capsule_yz(
            GUIDE_SHOE_LENGTH,
            GUIDE_SHOE_WIDTH,
            shoe_thickness,
            shoe_y,
            shoe_z,
            KEYBOARD_GUIDE_ANGLE_DEG,
            sign * 148.5,
        )
        shoe.label = f"keyboard_captured_guide_shoe_{side_name}"
        shoe.color = GUIDE_SHOE_GRAY
        children.append(shoe)

        connector = v1._beam_between_yz(
            (shoe_y, shoe_z),
            (
                v1.KEYBOARD_Y0 + t * v1.KEYBOARD_DY,
                v1.KEYBOARD_Z0 - 2.0 + t * v1.KEYBOARD_DZ,
            ),
            5.0,
            6.0,
            sign * 145.5,
        )
        connector.label = f"keyboard_guide_shoe_connector_{side_name}"
        connector.color = GUIDE_SHOE_GRAY
        children.append(connector)

        spring, seat = _compression_spring(
            side_name,
            rail_x,
            moving_front_y,
            moving_front_z,
            fixed_y,
            fixed_z,
            spring_length,
        )
        children.extend((spring, seat))

    return children


def kinematic_sample(travel):
    t = max(0.0, min(1.0, float(travel)))
    key_y = v1.KEYBOARD_Y0 + t * v1.KEYBOARD_DY
    key_z = v1.KEYBOARD_Z0 + t * v1.KEYBOARD_DZ
    track_y = v1.TRACKPAD_Y0 + t * v1.TRACKPAD_DY
    track_z = v1.TRACKPAD_Z0 + t * TRACKPAD_DZ_V6
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
    margin = CAPTURED_SLOT_LENGTH / 2.0 - ROLLER_OD / 2.0 - abs(along)
    return {
        "travel": t,
        "along": along,
        "left_cross": cross,
        "right_cross": cross,
        "left_margin": margin,
        "right_margin": margin,
        "spring_length": SPRING_FREE_LENGTH - t * KEYBOARD_PATH,
    }


def validate_kinematics(sample_count=201):
    samples = [kinematic_sample(i / (sample_count - 1)) for i in range(sample_count)]
    return {
        "sample_count": sample_count,
        "slot_angle_deg": SLOT_ANGLE_DEG,
        "trackpad_dz": TRACKPAD_DZ_V6,
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
        "spring_free_length": samples[0]["spring_length"],
        "spring_compressed_length": samples[-1]["spring_length"],
        "spring_compression": (
            samples[0]["spring_length"] - samples[-1]["spring_length"]
        ),
    }


def build_pose(travel=0.0, pose_name="keyboard_mode"):
    t = max(0.0, min(1.0, float(travel)))

    original_trackpad_dz = v1.TRACKPAD_DZ
    v1.TRACKPAD_DZ = TRACKPAD_DZ_V6
    try:
        original = v3.build_pose(t, pose_name)
    finally:
        v1.TRACKPAD_DZ = original_trackpad_dz

    obsolete_prefixes = (
        "keyboard_drive_arm_",
        "trackpad_slotted_link_",
        "cross_axis_roller_pin_",
        "keyboard_guide_roller_",
        "keyboard_fixed_guide_",
        "return_anchor_3x5mm_",
    )
    children = [
        child for child in original.children
        if not any((child.label or "").startswith(prefix) for prefix in obsolete_prefixes)
    ]

    key_y = v1.KEYBOARD_Y0 + t * v1.KEYBOARD_DY
    key_z = v1.KEYBOARD_Z0 + t * v1.KEYBOARD_DZ
    track_y = v1.TRACKPAD_Y0 + t * v1.TRACKPAD_DY
    track_z = v1.TRACKPAD_Z0 + t * TRACKPAD_DZ_V6

    children.extend(_keyboard_guides_and_springs(t))
    children.extend(_captured_linkage_side(
        "left_datum", -1, 0.0, key_y, key_z, track_y, track_z
    ))
    children.extend(_captured_linkage_side(
        "right_floating", 1, RIGHT_AXIAL_FLOAT, key_y, key_z, track_y, track_z
    ))

    return Compound(
        label=f"cross_axis_slider_concept_v6_symmetric_spring_{pose_name}",
        children=children,
    )
