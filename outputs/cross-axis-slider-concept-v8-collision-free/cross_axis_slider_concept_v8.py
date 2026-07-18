"""V8 collision-cleared bilateral cross-axis slider concept.

This is an independent geometry implementation. V1-V7 remain unchanged.
Coordinate system: X left/right, +Y toward display, +Z up. Units are mm.
"""

from math import atan2, degrees, hypot
from pathlib import Path
import sys

from build123d import Align, Box, Color, Compound, Edge, Face, Plane, Pos, Solid, Wire


V3_DIR = Path(__file__).resolve().parents[1] / "cross-axis-slider-concept-v3-compact-base"
if str(V3_DIR) not in sys.path:
    sys.path.insert(0, str(V3_DIR))

import cross_axis_slider_concept_v3 as v3  # noqa: E402

v1 = v3.v1


# Confirmed endpoint motion retained from V7.
TRACKPAD_DZ = -43.0
RELATIVE_DY = v1.KEYBOARD_DY - v1.TRACKPAD_DY
RELATIVE_DZ = v1.KEYBOARD_DZ - TRACKPAD_DZ
RELATIVE_PATH = hypot(RELATIVE_DY, RELATIVE_DZ)
SLOT_ANGLE_DEG = degrees(atan2(RELATIVE_DZ, RELATIVE_DY))
PIN_Y_OFFSET = 30.0
PIN_Z_OFFSET = 37.0

# Explicit side layers, from tray edge outward.
DRIVER_X = 142.0
DRIVER_THICKNESS = 3.0
CHEEK_X = 146.0
CHEEK_THICKNESS = 3.0
TRACKPAD_GUIDE_X = 150.0
TRACKPAD_GUIDE_THICKNESS = 2.5
KEYBOARD_GUIDE_X = 153.0
KEYBOARD_GUIDE_THICKNESS = 2.0

# Cross-axis slider.
SLOT_TRAVEL_LENGTH = 158.0
SLOT_BODY_LENGTH = 170.0
SLOT_BODY_WIDTH = 13.0
SLOT_WIDTH = 7.0
CROSS_ROLLER_OD = 6.0
CROSS_ROLLER_THICKNESS = 2.4
CROSS_SHAFT_DIAMETER = 3.2

# Linear guides.
GUIDE_BODY_WIDTH = 15.0
GUIDE_SLOT_WIDTH = 12.0
GUIDE_ROLLER_OD = 5.6
GUIDE_ROLLER_THICKNESS = 2.0
GUIDE_AXLE_DIAMETER = 3.0
TRACKPAD_ROLLER_SPACING = 18.0
KEYBOARD_ROLLER_SPACING = 14.0

# Keyboard return spring.
KEYBOARD_PATH = hypot(v1.KEYBOARD_DY, v1.KEYBOARD_DZ)
KEYBOARD_UY = v1.KEYBOARD_DY / KEYBOARD_PATH
KEYBOARD_UZ = v1.KEYBOARD_DZ / KEYBOARD_PATH
KEYBOARD_GUIDE_ANGLE = degrees(atan2(v1.KEYBOARD_DZ, v1.KEYBOARD_DY))
SPRING_FREE_LENGTH = KEYBOARD_PATH + 14.0
SPRING_COMPRESSED_LENGTH = 14.0
SPRING_TURNS = 7
SPRING_COIL_RADIUS = 1.55
SPRING_WIRE_RADIUS = 0.42

BASE_COLOR = Color(0.29, 0.31, 0.34)
FIXED_COLOR = Color(0.18, 0.20, 0.23)
KEYBOARD_RED = Color(0.88, 0.16, 0.13)
TRACKPAD_BLUE = Color(0.10, 0.38, 0.92)
ROLLER_GOLD = Color(0.96, 0.65, 0.12)
HARDWARE_SILVER = Color(0.72, 0.75, 0.78)
SPRING_COLOR = Color(0.70, 0.58, 0.34)


def _label(shape, name, color):
    shape.label = name
    shape.color = color
    return shape


def _ring_x(center_x, y, z, outer_diameter, bore_diameter, thickness):
    ring = v1._x_cylinder(outer_diameter / 2.0, thickness, center_x, y, z)
    bore = v1._x_cylinder(bore_diameter / 2.0, thickness + 1.0, center_x, y, z)
    return ring - bore


def _slot_geometry(key_y, key_z, track_y, track_z):
    pin_y = key_y + PIN_Y_OFFSET
    pin_z = key_z + PIN_Z_OFFSET
    center_y0 = (v1.KEYBOARD_Y0 + PIN_Y_OFFSET) + RELATIVE_DY / 2.0
    center_z0 = (v1.KEYBOARD_Z0 + PIN_Z_OFFSET) + RELATIVE_DZ / 2.0
    center_y = center_y0 + (track_y - v1.TRACKPAD_Y0)
    center_z = center_z0 + (track_z - v1.TRACKPAD_Z0)
    return pin_y, pin_z, center_y, center_z


def _tray(label, y, z, color):
    return _label(v1._tray(label, y, z, color), label, color)


def _lateral_bracket(label, sign, outer_x, y, z, depth, height, color):
    inner_x = sign * (v1.TRAY_WIDTH / 2.0 - 1.0)
    outer_edge = sign * (outer_x + 1.5)
    center_x = (inner_x + outer_edge) / 2.0
    width_x = abs(outer_edge - inner_x)
    bracket = Box(
        width_x,
        depth,
        height,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Pos(center_x, y, z))
    return _label(bracket, label, color)


def _keyboard_driver(sign, side, key_y, key_z, pin_y, pin_z):
    x = sign * DRIVER_X
    front = (key_y - 25.0, key_z + 6.0)
    rear = (key_y + 5.0, key_z + 6.0)
    driver = v1._beam_between_yz(front, rear, 8.0, DRIVER_THICKNESS, x)
    driver = driver + v1._beam_between_yz(front, (pin_y, pin_z), 8.0, DRIVER_THICKNESS, x)
    driver = driver + v1._beam_between_yz(rear, (pin_y, pin_z), 8.0, DRIVER_THICKNESS, x)
    return _label(driver, f"keyboard_driver_{side}", KEYBOARD_RED)


def _trackpad_slotted_cheek(sign, side, track_y, track_z, slot_y, slot_z):
    x = sign * CHEEK_X
    cheek = v1._slotted_beam(
        SLOT_BODY_LENGTH,
        SLOT_BODY_WIDTH,
        SLOT_TRAVEL_LENGTH,
        SLOT_WIDTH,
        CHEEK_THICKNESS,
        slot_y,
        slot_z,
        SLOT_ANGLE_DEG,
        x,
    )
    # Join only at the closed front end cap. Joining on the slot centerline
    # would refill the slot and create a false solid obstruction.
    slope = RELATIVE_DZ / RELATIVE_DY
    attach = (slot_y - 84.0, slot_z - 84.0 * slope)
    anchor = (track_y - 50.0, track_z + 4.0)
    cheek = cheek + v1._beam_between_yz(
        attach, anchor, 8.0, CHEEK_THICKNESS, x
    )
    return _label(cheek, f"trackpad_slotted_cheek_{side}", TRACKPAD_BLUE)


def _cross_axis_parts(sign, side, pin_y, pin_z):
    driver_x = sign * DRIVER_X
    cheek_x = sign * CHEEK_X
    shaft_center_x = (driver_x + cheek_x) / 2.0
    shaft_length = abs(cheek_x - driver_x) + DRIVER_THICKNESS
    shaft = v1._x_cylinder(
        CROSS_SHAFT_DIAMETER / 2.0,
        shaft_length,
        shaft_center_x,
        pin_y,
        pin_z,
    )
    roller = _ring_x(
        cheek_x,
        pin_y,
        pin_z,
        CROSS_ROLLER_OD,
        CROSS_SHAFT_DIAMETER + 0.3,
        CROSS_ROLLER_THICKNESS,
    )
    return [
        _label(shaft, f"cross_axis_shaft_{side}", HARDWARE_SILVER),
        _label(roller, f"cross_axis_roller_{side}", ROLLER_GOLD),
    ]


def _guide_rail(prefix, side, sign, center, travel, roller_spacing, center_x, thickness):
    dy, dz = travel
    path = hypot(dy, dz)
    angle = degrees(atan2(dz, dy))
    if prefix == "keyboard":
        q_min = -roller_spacing / 2.0 - GUIDE_ROLLER_OD / 2.0 - 2.0
        fixed_q = (
            KEYBOARD_PATH + roller_spacing / 2.0 + 4.5
            + SPRING_COMPRESSED_LENGTH
        )
        q_max = fixed_q + 3.0
        slot_length = q_max - q_min
        center_q = (q_min + q_max) / 2.0
        uy, uz = dy / path, dz / path
        rail_center_y = center[0] + center_q * uy
        rail_center_z = center[1] + center_q * uz
    else:
        slot_length = path + roller_spacing + GUIDE_ROLLER_OD + 5.0
        rail_center_y = center[0] + dy / 2.0
        rail_center_z = center[1] + dz / 2.0
    body_length = slot_length + 8.0
    rail = v1._slotted_beam(
        body_length,
        GUIDE_BODY_WIDTH,
        slot_length,
        GUIDE_SLOT_WIDTH,
        thickness,
        rail_center_y,
        rail_center_z,
        angle,
        sign * center_x,
    )
    return (
        _label(rail, f"fixed_{prefix}_guide_{side}", FIXED_COLOR),
        body_length,
        (rail_center_y, rail_center_z),
    )


def _guide_supports(prefix, side, sign, travel, body_length, rail_center, center_x):
    dy, dz = travel
    path = hypot(dy, dz)
    uy, uz = dy / path, dz / path
    rail_center_y, rail_center_z = rail_center
    children = []
    for index, q in enumerate((-body_length / 2.0 + 2.0, body_length / 2.0 - 2.0), 1):
        y = rail_center_y + q * uy
        z = rail_center_z + q * uz
        if prefix == "trackpad" and index == 1:
            # Move the base foot behind the keyboard sweep and reach the high
            # rail end with a diagonal strut above that sweep.
            support = v1._beam_between_yz(
                (y + 25.0, v1.BASE_THICKNESS + 2.5),
                (y, z - GUIDE_BODY_WIDTH / 2.0),
                5.0,
                2.0,
                sign * (center_x - 0.5),
            )
            children.append(_label(
                support,
                f"fixed_{prefix}_guide_support_{side}_{index}",
                FIXED_COLOR,
            ))
            continue
        height = max(z - GUIDE_BODY_WIDTH / 2.0 - v1.BASE_THICKNESS, 0.5)
        support = Box(
            3.0,
            4.0,
            height,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Pos(sign * center_x, y, v1.BASE_THICKNESS))
        children.append(_label(
            support,
            f"fixed_{prefix}_guide_support_{side}_{index}",
            FIXED_COLOR,
        ))
    return children


def _guide_rollers(prefix, side, sign, moving_center, travel, spacing, guide_x, source_x):
    dy, dz = travel
    path = hypot(dy, dz)
    uy, uz = dy / path, dz / path
    children = []
    for index, q in enumerate((-spacing / 2.0, spacing / 2.0), 1):
        y = moving_center[0] + q * uy
        z = moving_center[1] + q * uz
        axle_center_x = sign * ((source_x + guide_x) / 2.0)
        axle_length = abs(guide_x - source_x) + 3.0
        axle = v1._x_cylinder(
            GUIDE_AXLE_DIAMETER / 2.0,
            axle_length,
            axle_center_x,
            y,
            z,
        )
        roller = _ring_x(
            sign * guide_x,
            y,
            z,
            GUIDE_ROLLER_OD,
            GUIDE_AXLE_DIAMETER + 0.25,
            GUIDE_ROLLER_THICKNESS,
        )
        children.extend((
            _label(axle, f"{prefix}_guide_axle_{side}_{index}", HARDWARE_SILVER),
            _label(roller, f"{prefix}_guide_roller_{side}_{index}", ROLLER_GOLD),
        ))
    return children


def _compression_spring(side, sign, travel):
    t = max(0.0, min(1.0, float(travel)))
    pin0_y = v1.KEYBOARD_Y0 + 48.0
    pin0_z = v1.KEYBOARD_Z0 + 12.0
    moving_q = t * KEYBOARD_PATH + KEYBOARD_ROLLER_SPACING / 2.0 + 4.5
    fixed_q = KEYBOARD_PATH + KEYBOARD_ROLLER_SPACING / 2.0 + 4.5 + SPRING_COMPRESSED_LENGTH
    spring_length = fixed_q - moving_q
    end_gap = 0.8
    coil_height = spring_length - 2.0 * end_gap
    start = (
        sign * KEYBOARD_GUIDE_X,
        pin0_y + (moving_q + end_gap) * KEYBOARD_UY,
        pin0_z + (moving_q + end_gap) * KEYBOARD_UZ,
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

    fixed_y = pin0_y + fixed_q * KEYBOARD_UY
    fixed_z = pin0_z + fixed_q * KEYBOARD_UZ
    seat = v1._capsule_yz(
        5.0,
        4.5,
        KEYBOARD_GUIDE_THICKNESS,
        fixed_y,
        fixed_z,
        KEYBOARD_GUIDE_ANGLE + 90.0,
        sign * KEYBOARD_GUIDE_X,
    )
    core_start_q = fixed_q - SPRING_COMPRESSED_LENGTH + 1.0
    core_start = (
        pin0_y + core_start_q * KEYBOARD_UY,
        pin0_z + core_start_q * KEYBOARD_UZ,
    )
    core_end = (
        fixed_y - 1.0 * KEYBOARD_UY,
        fixed_z - 1.0 * KEYBOARD_UZ,
    )
    core = v1._beam_between_yz(
        core_start,
        core_end,
        1.4,
        1.4,
        sign * KEYBOARD_GUIDE_X,
    )
    return [
        _label(spring, f"keyboard_compression_spring_{side}", SPRING_COLOR),
        _label(seat, f"fixed_keyboard_spring_seat_{side}", HARDWARE_SILVER),
        _label(core, f"fixed_keyboard_terminal_stop_core_{side}", HARDWARE_SILVER),
    ]


def kinematic_sample(travel):
    t = max(0.0, min(1.0, float(travel)))
    key_y = v1.KEYBOARD_Y0 + t * v1.KEYBOARD_DY
    key_z = v1.KEYBOARD_Z0 + t * v1.KEYBOARD_DZ
    track_y = v1.TRACKPAD_Y0 + t * v1.TRACKPAD_DY
    track_z = v1.TRACKPAD_Z0 + t * TRACKPAD_DZ
    pin_y, pin_z, slot_y, slot_z = _slot_geometry(key_y, key_z, track_y, track_z)
    along = ((pin_y - slot_y) * RELATIVE_DY + (pin_z - slot_z) * RELATIVE_DZ) / RELATIVE_PATH
    cross = ((pin_y - slot_y) * RELATIVE_DZ - (pin_z - slot_z) * RELATIVE_DY) / RELATIVE_PATH
    margin = SLOT_TRAVEL_LENGTH / 2.0 - CROSS_ROLLER_OD / 2.0 - abs(along)
    return {
        "travel": t,
        "along": along,
        "cross": cross,
        "margin": margin,
        "spring_length": SPRING_FREE_LENGTH - t * KEYBOARD_PATH,
    }


def validate_kinematics(sample_count=201):
    samples = [kinematic_sample(i / (sample_count - 1)) for i in range(sample_count)]
    return {
        "sample_count": sample_count,
        "slot_angle_deg": SLOT_ANGLE_DEG,
        "max_cross_error": max(abs(sample["cross"]) for sample in samples),
        "minimum_roller_center_margin": min(sample["margin"] for sample in samples),
        "roller_monotonic": all(samples[i + 1]["along"] > samples[i]["along"] for i in range(len(samples) - 1)),
        "spring_monotonic": all(samples[i + 1]["spring_length"] < samples[i]["spring_length"] for i in range(len(samples) - 1)),
        "spring_final_length": samples[-1]["spring_length"],
    }


def build_pose(travel=0.0, pose_name="keyboard_mode"):
    t = max(0.0, min(1.0, float(travel)))
    key_y = v1.KEYBOARD_Y0 + t * v1.KEYBOARD_DY
    key_z = v1.KEYBOARD_Z0 + t * v1.KEYBOARD_DZ
    track_y = v1.TRACKPAD_Y0 + t * v1.TRACKPAD_DY
    track_z = v1.TRACKPAD_Z0 + t * TRACKPAD_DZ
    pin_y, pin_z, slot_y, slot_z = _slot_geometry(key_y, key_z, track_y, track_z)

    base = _label(v3.make_compact_open_base(), "base_openwork_308x280", BASE_COLOR)
    children = [
        base,
        _tray("keyboard_tray_continuous", key_y, key_z, KEYBOARD_RED),
        _tray("trackpad_tray_continuous_right_aligned", track_y, track_z, TRACKPAD_BLUE),
    ]

    for side, sign in (("left_datum", -1), ("right_floating", 1)):
        children.append(_keyboard_driver(sign, side, key_y, key_z, pin_y, pin_z))
        children.append(_trackpad_slotted_cheek(sign, side, track_y, track_z, slot_y, slot_z))
        children.extend(_cross_axis_parts(sign, side, pin_y, pin_z))

        keyboard_bracket_y = key_y + 48.0
        keyboard_bracket_z = key_z + 2.0
        children.append(_lateral_bracket(
            f"keyboard_rear_bracket_{side}", sign, DRIVER_X,
            keyboard_bracket_y, keyboard_bracket_z, 7.0, 4.0, KEYBOARD_RED,
        ))
        keyboard_center = (key_y + 48.0, key_z + 12.0)
        keyboard_rail, keyboard_body_length, keyboard_rail_center = _guide_rail(
            "keyboard", side, sign,
            (v1.KEYBOARD_Y0 + 48.0, v1.KEYBOARD_Z0 + 12.0),
            (v1.KEYBOARD_DY, v1.KEYBOARD_DZ),
            KEYBOARD_ROLLER_SPACING, KEYBOARD_GUIDE_X, KEYBOARD_GUIDE_THICKNESS,
        )
        children.append(keyboard_rail)
        children.extend(_guide_supports(
            "keyboard", side, sign,
            (v1.KEYBOARD_DY, v1.KEYBOARD_DZ),
            keyboard_body_length, keyboard_rail_center, KEYBOARD_GUIDE_X,
        ))
        children.extend(_guide_rollers(
            "keyboard", side, sign, keyboard_center,
            (v1.KEYBOARD_DY, v1.KEYBOARD_DZ), KEYBOARD_ROLLER_SPACING,
            KEYBOARD_GUIDE_X, DRIVER_X,
        ))
        children.extend(_compression_spring(side, sign, t))

        trackpad_bracket_y = track_y - 50.0
        trackpad_bracket_z = track_z + 4.0
        children.append(_lateral_bracket(
            f"trackpad_front_bracket_{side}", sign, CHEEK_X,
            trackpad_bracket_y, trackpad_bracket_z, 7.0, 4.0, TRACKPAD_BLUE,
        ))
        trackpad_center = (track_y - 25.0, track_z + 8.0)
        guide_carriage = v1._beam_between_yz(
            (trackpad_bracket_y, trackpad_bracket_z),
            trackpad_center,
            7.0,
            CHEEK_THICKNESS,
            sign * CHEEK_X,
        )
        children.append(_label(
            guide_carriage,
            f"trackpad_guide_carriage_{side}",
            TRACKPAD_BLUE,
        ))
        trackpad_rail, trackpad_body_length, trackpad_rail_center = _guide_rail(
            "trackpad", side, sign,
            (v1.TRACKPAD_Y0 - 25.0, v1.TRACKPAD_Z0 + 8.0),
            (v1.TRACKPAD_DY, TRACKPAD_DZ),
            TRACKPAD_ROLLER_SPACING, TRACKPAD_GUIDE_X, TRACKPAD_GUIDE_THICKNESS,
        )
        children.append(trackpad_rail)
        children.extend(_guide_supports(
            "trackpad", side, sign,
            (v1.TRACKPAD_DY, TRACKPAD_DZ),
            trackpad_body_length, trackpad_rail_center, TRACKPAD_GUIDE_X,
        ))
        children.extend(_guide_rollers(
            "trackpad", side, sign, trackpad_center,
            (v1.TRACKPAD_DY, TRACKPAD_DZ), TRACKPAD_ROLLER_SPACING,
            TRACKPAD_GUIDE_X, CHEEK_X,
        ))

    return Compound(
        label=f"cross_axis_slider_concept_v8_collision_free_{pose_name}",
        children=children,
    )


def _motion_group(label):
    if label.startswith("base_"):
        return "B"
    if label.startswith("fixed_"):
        return "F"
    if label.startswith(("keyboard_", "cross_axis_")):
        return "K"
    if label.startswith("trackpad_"):
        return "T"
    raise ValueError(f"Unclassified body: {label}")


def _bbox_overlaps(a, b, tol=1e-7):
    aa = a.bounding_box()
    bb = b.bounding_box()
    return not (
        aa.max.X <= bb.min.X + tol or bb.max.X <= aa.min.X + tol
        or aa.max.Y <= bb.min.Y + tol or bb.max.Y <= aa.min.Y + tol
        or aa.max.Z <= bb.min.Z + tol or bb.max.Z <= aa.min.Z + tol
    )


def _allowed_collision(label_a, label_b):
    pair = frozenset((label_a, label_b))
    for side in ("left_datum", "right_floating"):
        if pair == frozenset((
            f"keyboard_compression_spring_{side}",
            f"fixed_keyboard_spring_seat_{side}",
        )):
            return True
        if pair == frozenset((
            f"keyboard_compression_spring_{side}",
            f"fixed_keyboard_terminal_stop_core_{side}",
        )):
            return True
    return False


def validate_collisions(sample_count=21, volume_tolerance=1e-4):
    collisions = []
    checked_pairs = 0
    for index in range(sample_count):
        t = index / (sample_count - 1)
        pose = build_pose(t, f"collision_sample_{index}")
        bodies = [(child.label or "", child) for child in pose.children]
        for i, (label_a, body_a) in enumerate(bodies):
            group_a = _motion_group(label_a)
            for label_b, body_b in bodies[i + 1:]:
                group_b = _motion_group(label_b)
                if group_a == group_b or frozenset((group_a, group_b)) not in {
                    frozenset(("B", "K")),
                    frozenset(("B", "T")),
                    frozenset(("F", "K")),
                    frozenset(("F", "T")),
                    frozenset(("K", "T")),
                }:
                    continue
                if _allowed_collision(label_a, label_b) or not _bbox_overlaps(body_a, body_b):
                    continue
                checked_pairs += 1
                common = body_a & body_b
                volume = 0.0 if common is None else sum(
                    solid.volume for solid in common.solids()
                )
                if volume > volume_tolerance:
                    collisions.append({
                        "travel": round(t, 6),
                        "groups": group_a + group_b,
                        "a": label_a,
                        "b": label_b,
                        "volume": round(volume, 6),
                    })
    return {
        "sample_count": sample_count,
        "checked_bbox_candidates": checked_pairs,
        "collisions": collisions,
    }
