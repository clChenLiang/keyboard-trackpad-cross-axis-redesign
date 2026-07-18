"""V9 force-closed bilateral cross-axis slider concept.

V9 replaces V8's disconnected visual motion with continuous input and output
side carriages. Coordinate system: X left/right, +Y toward display, +Z up.
Units are millimetres.
"""

from math import atan2, degrees, hypot
from pathlib import Path
import sys

from build123d import Align, Box, Color, Compound, Pos


V8_DIR = Path(__file__).resolve().parents[1] / "cross-axis-slider-concept-v8-collision-free"
if str(V8_DIR) not in sys.path:
    sys.path.insert(0, str(V8_DIR))

import cross_axis_slider_concept_v8 as v8  # noqa: E402

v1 = v8.v1
v3 = v8.v3


TRACKPAD_DZ = -43.0
RELATIVE_DY = v1.KEYBOARD_DY - v1.TRACKPAD_DY
RELATIVE_DZ = v1.KEYBOARD_DZ - TRACKPAD_DZ
RELATIVE_PATH = hypot(RELATIVE_DY, RELATIVE_DZ)
SLOT_ANGLE_DEG = degrees(atan2(RELATIVE_DZ, RELATIVE_DY))

KEYBOARD_PATH = hypot(v1.KEYBOARD_DY, v1.KEYBOARD_DZ)
TRACKPAD_PATH = hypot(v1.TRACKPAD_DY, TRACKPAD_DZ)
KEYBOARD_U = (v1.KEYBOARD_DY / KEYBOARD_PATH, v1.KEYBOARD_DZ / KEYBOARD_PATH)
TRACKPAD_U = (v1.TRACKPAD_DY / TRACKPAD_PATH, TRACKPAD_DZ / TRACKPAD_PATH)
SLOT_U = (RELATIVE_DY / RELATIVE_PATH, RELATIVE_DZ / RELATIVE_PATH)
SLOT_N = (-SLOT_U[1], SLOT_U[0])

DRIVER_X = 142.0
DRIVER_THICKNESS = 3.0
CHEEK_X = 146.0
CHEEK_THICKNESS = 3.0
TRACKPAD_GUIDE_X = 150.0
TRACKPAD_GUIDE_THICKNESS = 2.5
KEYBOARD_GUIDE_X = 153.0
KEYBOARD_GUIDE_THICKNESS = 2.0

PIN_Y_OFFSET = 30.0
PIN_Z_OFFSET = 37.0
SLOT_LENGTH = 158.0
SLOT_BODY_LENGTH = 170.0
SLOT_BODY_WIDTH = 14.0
SLOT_WIDTH = 8.0
CROSS_SHAFT_DIAMETER = 4.0
CROSS_ROLLER_OD = SLOT_WIDTH
CROSS_ROLLER_THICKNESS = 2.6

GUIDE_BODY_WIDTH = 16.0
GUIDE_SLOT_WIDTH = 12.0
GUIDE_ROLLER_OD = 11.6
GUIDE_AXLE_DIAMETER = 4.0
GUIDE_ROLLER_THICKNESS = 2.0
KEYBOARD_ROLLER_SPACING = 16.0
TRACKPAD_ROLLER_SPACING = 22.0

SPRING_COMPRESSED_LENGTH = 14.0
SPRING_FREE_LENGTH = KEYBOARD_PATH + SPRING_COMPRESSED_LENGTH
SPRING_AMPLITUDE = 2.2
SPRING_SEGMENTS = 14

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


def _slot_geometry(key_y, key_z, track_y, track_z):
    pin_y = key_y + PIN_Y_OFFSET
    pin_z = key_z + PIN_Z_OFFSET
    center_y0 = (v1.KEYBOARD_Y0 + PIN_Y_OFFSET) + RELATIVE_DY / 2.0
    center_z0 = (v1.KEYBOARD_Z0 + PIN_Z_OFFSET) + RELATIVE_DZ / 2.0
    center_y = center_y0 + (track_y - v1.TRACKPAD_Y0)
    center_z = center_z0 + (track_z - v1.TRACKPAD_Z0)
    return pin_y, pin_z, center_y, center_z


def _guide_centers(prefix, y, z):
    if prefix == "keyboard":
        uy, uz = KEYBOARD_U
        spacing = KEYBOARD_ROLLER_SPACING
    else:
        uy, uz = TRACKPAD_U
        spacing = TRACKPAD_ROLLER_SPACING
    return [
        (y - spacing / 2.0 * uy, z - spacing / 2.0 * uz),
        (y + spacing / 2.0 * uy, z + spacing / 2.0 * uz),
    ]


def _keyboard_side_carriage(sign, side, key_y, key_z, pin_y, pin_z):
    x = sign * DRIVER_X
    bridge = v8._lateral_bracket(
        f"keyboard_tray_bridge_{side}", sign, DRIVER_X,
        key_y, key_z + 4.0, 12.0, 8.0, KEYBOARD_RED,
    )
    front = (key_y - 25.0, key_z + 6.0)
    rear = (key_y + 5.0, key_z + 6.0)
    carriage = bridge
    carriage = carriage + v1._beam_between_yz(front, rear, 8.0, DRIVER_THICKNESS, x)
    carriage = carriage + v1._beam_between_yz(front, (pin_y, pin_z), 8.0, DRIVER_THICKNESS, x)
    carriage = carriage + v1._beam_between_yz(rear, (pin_y, pin_z), 8.0, DRIVER_THICKNESS, x)

    guide_center = (key_y + 48.0, key_z + 12.0)
    axle_centers = _guide_centers("keyboard", *guide_center)
    node = (key_y, key_z + 6.0)
    for center in axle_centers:
        carriage = carriage + v1._beam_between_yz(
            node, center, 7.0, DRIVER_THICKNESS, x
        )

    moving_seat_q = KEYBOARD_ROLLER_SPACING / 2.0 + 6.0
    moving_seat = (
        guide_center[0] + moving_seat_q * KEYBOARD_U[0],
        guide_center[1] + moving_seat_q * KEYBOARD_U[1],
    )
    carriage = carriage + v1._beam_between_yz(
        node, moving_seat, 7.0, DRIVER_THICKNESS, x
    )
    carriage = carriage + v1._capsule_yz(
        7.0, 6.0, DRIVER_THICKNESS,
        moving_seat[0], moving_seat[1],
        degrees(atan2(KEYBOARD_U[1], KEYBOARD_U[0])) + 90.0,
        x,
    )
    return (
        _label(carriage, f"keyboard_rigid_side_carriage_{side}", KEYBOARD_RED),
        axle_centers,
        moving_seat,
    )


def _trackpad_side_carriage(sign, side, track_y, track_z, slot_y, slot_z):
    x = sign * CHEEK_X
    bridge_y = track_y - 50.0
    bridge_z = track_z + 4.0
    bridge = v8._lateral_bracket(
        f"trackpad_tray_bridge_{side}", sign, CHEEK_X,
        bridge_y, bridge_z, 10.0, 8.0, TRACKPAD_BLUE,
    )
    carriage = bridge
    carriage = carriage + v1._slotted_beam(
        SLOT_BODY_LENGTH,
        SLOT_BODY_WIDTH,
        SLOT_LENGTH,
        SLOT_WIDTH,
        CHEEK_THICKNESS,
        slot_y,
        slot_z,
        SLOT_ANGLE_DEG,
        x,
    )
    # Attach through the positive-normal outer wall, not through the slot
    # centerline. The bridge node lies on this same side for the full stroke.
    attach = (
        slot_y - 80.0 * SLOT_U[0] + 7.0 * SLOT_N[0],
        slot_z - 80.0 * SLOT_U[1] + 7.0 * SLOT_N[1],
    )
    bridge_node = (bridge_y, bridge_z)
    carriage = carriage + v1._beam_between_yz(
        attach, bridge_node, 6.0, CHEEK_THICKNESS, x
    )

    guide_center = (track_y - 25.0, track_z + 8.0)
    axle_centers = _guide_centers("trackpad", *guide_center)
    for center in axle_centers:
        carriage = carriage + v1._beam_between_yz(
            bridge_node, center, 8.0, CHEEK_THICKNESS, x
        )
    return (
        _label(carriage, f"trackpad_rigid_side_carriage_{side}", TRACKPAD_BLUE),
        axle_centers,
    )


def _axle_and_roller(prefix, side, sign, index, center, source_x, guide_x):
    y, z = center
    axle_center_x = sign * ((source_x + guide_x) / 2.0)
    axle_length = abs(guide_x - source_x) + 3.0
    axle = v1._x_cylinder(
        GUIDE_AXLE_DIAMETER / 2.0,
        axle_length,
        axle_center_x,
        y,
        z,
    )
    roller = v8._ring_x(
        sign * guide_x,
        y,
        z,
        GUIDE_ROLLER_OD,
        GUIDE_AXLE_DIAMETER,
        GUIDE_ROLLER_THICKNESS,
    )
    return [
        _label(axle, f"{prefix}_guide_axle_{side}_{index}", HARDWARE_SILVER),
        _label(roller, f"{prefix}_guide_roller_{side}_{index}", ROLLER_GOLD),
    ]


def _cross_axis(sign, side, pin_y, pin_z):
    driver_x = sign * DRIVER_X
    cheek_x = sign * CHEEK_X
    center_x = (driver_x + cheek_x) / 2.0
    length = abs(cheek_x - driver_x) + DRIVER_THICKNESS
    shaft = v1._x_cylinder(
        CROSS_SHAFT_DIAMETER / 2.0,
        length,
        center_x,
        pin_y,
        pin_z,
    )
    roller = v8._ring_x(
        cheek_x,
        pin_y,
        pin_z,
        CROSS_ROLLER_OD,
        CROSS_SHAFT_DIAMETER,
        CROSS_ROLLER_THICKNESS,
    )
    return [
        _label(shaft, f"cross_axis_shaft_{side}", HARDWARE_SILVER),
        _label(roller, f"cross_axis_contact_roller_{side}", ROLLER_GOLD),
    ]


def _guide_rail(prefix, side, sign, start, travel, spacing, guide_x, thickness):
    dy, dz = travel
    path = hypot(dy, dz)
    uy, uz = dy / path, dz / path
    slot_length = path + spacing + GUIDE_ROLLER_OD + 4.0
    body_length = slot_length + 8.0
    center_q = path / 2.0
    center_y = start[0] + center_q * uy
    center_z = start[1] + center_q * uz
    rail = v1._slotted_beam(
        body_length,
        GUIDE_BODY_WIDTH,
        slot_length,
        GUIDE_SLOT_WIDTH,
        thickness,
        center_y,
        center_z,
        degrees(atan2(dz, dy)),
        sign * guide_x,
    )
    return (
        _label(rail, f"fixed_{prefix}_guide_{side}", FIXED_COLOR),
        body_length,
        (center_y, center_z),
    )


def _guide_supports(prefix, side, sign, travel, body_length, rail_center, guide_x):
    dy, dz = travel
    path = hypot(dy, dz)
    uy, uz = dy / path, dz / path
    children = []
    for index, q in enumerate((-body_length / 2.0 + 2.0, body_length / 2.0 - 2.0), 1):
        y = rail_center[0] + q * uy
        z = rail_center[1] + q * uz
        if prefix == "trackpad" and index == 1:
            support = v1._beam_between_yz(
                (y + 27.0, v1.BASE_THICKNESS + 2.5),
                (y, z - GUIDE_BODY_WIDTH / 2.0),
                5.0,
                2.0,
                sign * (guide_x - 0.5),
            )
        else:
            height = max(z - GUIDE_BODY_WIDTH / 2.0 - v1.BASE_THICKNESS, 0.5)
            support = Box(
                3.0, 4.0, height,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            ).moved(Pos(sign * guide_x, y, v1.BASE_THICKNESS))
        children.append(_label(
            support,
            f"fixed_{prefix}_guide_support_{side}_{index}",
            FIXED_COLOR,
        ))
    return children


def _spring_geometry(side, sign, moving_seat, travel):
    t = max(0.0, min(1.0, float(travel)))
    spring_length = SPRING_FREE_LENGTH - t * KEYBOARD_PATH
    fixed_seat = (
        moving_seat[0] + spring_length * KEYBOARD_U[0],
        moving_seat[1] + spring_length * KEYBOARD_U[1],
    )
    angle = degrees(atan2(KEYBOARD_U[1], KEYBOARD_U[0])) + 90.0
    moving_pad = v1._capsule_yz(
        7.0, 6.0, DRIVER_THICKNESS,
        moving_seat[0], moving_seat[1], angle, sign * DRIVER_X,
    )
    fixed_pad = v1._capsule_yz(
        7.0, 6.0, 3.0,
        fixed_seat[0], fixed_seat[1], angle, sign * DRIVER_X,
    )

    uy, uz = KEYBOARD_U
    ny, nz = -uz, uy
    points = []
    for index in range(SPRING_SEGMENTS + 1):
        f = index / SPRING_SEGMENTS
        offset = 0.0 if index in (0, SPRING_SEGMENTS) else (
            SPRING_AMPLITUDE if index % 2 else -SPRING_AMPLITUDE
        )
        points.append((
            moving_seat[0] + f * spring_length * uy + offset * ny,
            moving_seat[1] + f * spring_length * uz + offset * nz,
        ))
    spring = v1._beam_between_yz(points[0], points[1], 1.0, 1.0, sign * DRIVER_X)
    for p1, p2 in zip(points[1:], points[2:]):
        spring = spring + v1._beam_between_yz(p1, p2, 1.0, 1.0, sign * DRIVER_X)
    return [
        _label(moving_pad, f"keyboard_moving_spring_seat_{side}", KEYBOARD_RED),
        _label(fixed_pad, f"fixed_keyboard_spring_seat_{side}", HARDWARE_SILVER),
        _label(spring, f"keyboard_compression_spring_{side}", SPRING_COLOR),
    ]


def kinematic_sample(travel):
    t = max(0.0, min(1.0, float(travel)))
    key_y = v1.KEYBOARD_Y0 + t * v1.KEYBOARD_DY
    key_z = v1.KEYBOARD_Z0 + t * v1.KEYBOARD_DZ
    track_y = v1.TRACKPAD_Y0 + t * v1.TRACKPAD_DY
    track_z = v1.TRACKPAD_Z0 + t * TRACKPAD_DZ
    pin_y, pin_z, slot_y, slot_z = _slot_geometry(key_y, key_z, track_y, track_z)
    along = (pin_y - slot_y) * SLOT_U[0] + (pin_z - slot_z) * SLOT_U[1]
    cross = (pin_y - slot_y) * SLOT_N[0] + (pin_z - slot_z) * SLOT_N[1]
    return {
        "travel": t,
        "along": along,
        "cross": cross,
        "margin": SLOT_LENGTH / 2.0 - CROSS_ROLLER_OD / 2.0 - abs(along),
    }


def validate_kinematics(sample_count=201):
    samples = [kinematic_sample(i / (sample_count - 1)) for i in range(sample_count)]
    output_projection = abs(SLOT_N[0] * TRACKPAD_U[0] + SLOT_N[1] * TRACKPAD_U[1])
    input_projection = abs(SLOT_N[0] * KEYBOARD_U[0] + SLOT_N[1] * KEYBOARD_U[1])
    return {
        "sample_count": sample_count,
        "slot_angle_deg": SLOT_ANGLE_DEG,
        "max_cross_error": max(abs(s["cross"]) for s in samples),
        "minimum_slot_margin": min(s["margin"] for s in samples),
        "roller_monotonic": all(samples[i + 1]["along"] > samples[i]["along"] for i in range(len(samples) - 1)),
        "stroke_ratio": TRACKPAD_PATH / KEYBOARD_PATH,
        "minimum_output_projection": output_projection,
        "input_projection": input_projection,
        "ideal_input_force_per_output_resistance": TRACKPAD_PATH / KEYBOARD_PATH,
    }


def build_pose(travel=0.0, pose_name="keyboard_mode"):
    t = max(0.0, min(1.0, float(travel)))
    key_y = v1.KEYBOARD_Y0 + t * v1.KEYBOARD_DY
    key_z = v1.KEYBOARD_Z0 + t * v1.KEYBOARD_DZ
    track_y = v1.TRACKPAD_Y0 + t * v1.TRACKPAD_DY
    track_z = v1.TRACKPAD_Z0 + t * TRACKPAD_DZ
    pin_y, pin_z, slot_y, slot_z = _slot_geometry(key_y, key_z, track_y, track_z)

    base = _label(v3.make_compact_open_base(), "base_openwork_308x280", BASE_COLOR)
    keyboard_tray = _label(
        v1._tray("keyboard_tray_continuous", key_y, key_z, KEYBOARD_RED),
        "keyboard_tray_continuous", KEYBOARD_RED,
    )
    trackpad_tray = _label(
        v1._tray("trackpad_tray_continuous", track_y, track_z, TRACKPAD_BLUE),
        "trackpad_tray_continuous", TRACKPAD_BLUE,
    )
    children = [base, keyboard_tray, trackpad_tray]

    for side, sign in (("left_datum", -1), ("right_floating", 1)):
        keyboard_carriage, keyboard_axle_centers, moving_seat = _keyboard_side_carriage(
            sign, side, key_y, key_z, pin_y, pin_z
        )
        trackpad_carriage, trackpad_axle_centers = _trackpad_side_carriage(
            sign, side, track_y, track_z, slot_y, slot_z
        )
        children.extend((keyboard_carriage, trackpad_carriage))
        children.extend(_cross_axis(sign, side, pin_y, pin_z))

        for index, center in enumerate(keyboard_axle_centers, 1):
            children.extend(_axle_and_roller(
                "keyboard", side, sign, index, center,
                DRIVER_X, KEYBOARD_GUIDE_X,
            ))
        for index, center in enumerate(trackpad_axle_centers, 1):
            children.extend(_axle_and_roller(
                "trackpad", side, sign, index, center,
                CHEEK_X, TRACKPAD_GUIDE_X,
            ))

        keyboard_rail, keyboard_body_length, keyboard_rail_center = _guide_rail(
            "keyboard", side, sign,
            (v1.KEYBOARD_Y0 + 48.0, v1.KEYBOARD_Z0 + 12.0),
            (v1.KEYBOARD_DY, v1.KEYBOARD_DZ),
            KEYBOARD_ROLLER_SPACING, KEYBOARD_GUIDE_X, KEYBOARD_GUIDE_THICKNESS,
        )
        trackpad_rail, trackpad_body_length, trackpad_rail_center = _guide_rail(
            "trackpad", side, sign,
            (v1.TRACKPAD_Y0 - 25.0, v1.TRACKPAD_Z0 + 8.0),
            (v1.TRACKPAD_DY, TRACKPAD_DZ),
            TRACKPAD_ROLLER_SPACING, TRACKPAD_GUIDE_X, TRACKPAD_GUIDE_THICKNESS,
        )
        children.extend((keyboard_rail, trackpad_rail))
        children.extend(_guide_supports(
            "keyboard", side, sign,
            (v1.KEYBOARD_DY, v1.KEYBOARD_DZ),
            keyboard_body_length, keyboard_rail_center, KEYBOARD_GUIDE_X,
        ))
        children.extend(_guide_supports(
            "trackpad", side, sign,
            (v1.TRACKPAD_DY, TRACKPAD_DZ),
            trackpad_body_length, trackpad_rail_center, TRACKPAD_GUIDE_X,
        ))
        children.extend(_spring_geometry(side, sign, moving_seat, t))

    return Compound(
        label=f"cross_axis_slider_concept_v9_force_closed_{pose_name}",
        children=children,
    )


def _overlap_volume(a, b):
    common = a & b
    return 0.0 if common is None else sum(s.volume for s in common.solids())


def validate_force_closure(travel=0.0, tolerance=1e-4):
    pose = build_pose(travel, "force_closure")
    bodies = {child.label or "": child for child in pose.children}
    failures = []
    checks = []
    for side in ("left_datum", "right_floating"):
        required_overlap = [
            ("keyboard_tray_continuous", f"keyboard_rigid_side_carriage_{side}"),
            (f"keyboard_rigid_side_carriage_{side}", f"cross_axis_shaft_{side}"),
            (f"keyboard_rigid_side_carriage_{side}", f"keyboard_guide_axle_{side}_1"),
            (f"keyboard_rigid_side_carriage_{side}", f"keyboard_guide_axle_{side}_2"),
            (f"keyboard_rigid_side_carriage_{side}", f"keyboard_moving_spring_seat_{side}"),
            ("trackpad_tray_continuous", f"trackpad_rigid_side_carriage_{side}"),
            (f"trackpad_rigid_side_carriage_{side}", f"trackpad_guide_axle_{side}_1"),
            (f"trackpad_rigid_side_carriage_{side}", f"trackpad_guide_axle_{side}_2"),
        ]
        required_contact = [
            (f"cross_axis_shaft_{side}", f"cross_axis_contact_roller_{side}"),
            (f"cross_axis_contact_roller_{side}", f"trackpad_rigid_side_carriage_{side}"),
            (f"keyboard_guide_axle_{side}_1", f"keyboard_guide_roller_{side}_1"),
            (f"keyboard_guide_axle_{side}_2", f"keyboard_guide_roller_{side}_2"),
            (f"trackpad_guide_axle_{side}_1", f"trackpad_guide_roller_{side}_1"),
            (f"trackpad_guide_axle_{side}_2", f"trackpad_guide_roller_{side}_2"),
            (f"keyboard_moving_spring_seat_{side}", f"keyboard_compression_spring_{side}"),
            (f"fixed_keyboard_spring_seat_{side}", f"keyboard_compression_spring_{side}"),
        ]
        for a, b in required_overlap:
            volume = _overlap_volume(bodies[a], bodies[b])
            checks.append((a, b, "overlap", volume))
            if volume <= tolerance:
                failures.append({"a": a, "b": b, "mode": "overlap", "value": volume})
        for a, b in required_contact:
            distance = bodies[a].distance_to(bodies[b])
            checks.append((a, b, "contact", distance))
            if distance > tolerance:
                failures.append({"a": a, "b": b, "mode": "contact", "value": distance})

        for rigid_label in (
            f"keyboard_rigid_side_carriage_{side}",
            f"trackpad_rigid_side_carriage_{side}",
        ):
            solid_count = len(bodies[rigid_label].solids())
            checks.append((rigid_label, "", "solid_count", solid_count))
            if solid_count != 1:
                failures.append({"a": rigid_label, "b": "", "mode": "solid_count", "value": solid_count})
    return {"travel": travel, "checks": checks, "failures": failures}


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


def _allowed_collision(label_a, label_b):
    pair = frozenset((label_a, label_b))
    for side in ("left_datum", "right_floating"):
        if pair == frozenset((
            f"keyboard_compression_spring_{side}",
            f"fixed_keyboard_spring_seat_{side}",
        )):
            return True
    return False


def validate_collisions(sample_count=21, volume_tolerance=1e-3):
    collisions = []
    candidates = 0
    forbidden = {
        frozenset(("B", "K")),
        frozenset(("B", "T")),
        frozenset(("F", "K")),
        frozenset(("F", "T")),
        frozenset(("K", "T")),
    }
    for index in range(sample_count):
        t = index / (sample_count - 1)
        pose = build_pose(t, f"collision_{index}")
        bodies = [(child.label or "", child) for child in pose.children]
        for i, (label_a, body_a) in enumerate(bodies):
            group_a = _motion_group(label_a)
            for label_b, body_b in bodies[i + 1:]:
                group_b = _motion_group(label_b)
                if frozenset((group_a, group_b)) not in forbidden:
                    continue
                if _allowed_collision(label_a, label_b) or not v8._bbox_overlaps(body_a, body_b):
                    continue
                candidates += 1
                volume = _overlap_volume(body_a, body_b)
                if volume > volume_tolerance:
                    collisions.append({
                        "travel": round(t, 6),
                        "a": label_a,
                        "b": label_b,
                        "volume": round(volume, 6),
                    })
    return {
        "sample_count": sample_count,
        "bbox_candidates": candidates,
        "collisions": collisions,
    }
