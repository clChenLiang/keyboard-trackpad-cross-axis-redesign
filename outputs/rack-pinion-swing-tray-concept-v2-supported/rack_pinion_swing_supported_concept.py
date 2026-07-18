"""Supported V2 of the rack-pinion swing-tray concept.

V1 remains untouched.  Four independent, parallel inclined guide pods carry
the keyboard tray; the rack transmits motion but no longer acts as its only
structural support.  Coordinates are millimetres.
"""

from importlib.util import module_from_spec, spec_from_file_location
from math import atan2, degrees, hypot
from pathlib import Path

from build123d import Align, Box, Color, Compound, Cylinder, Pos, Rot


_V1_PATH = (
    Path(__file__).resolve().parent.parent
    / "rack-pinion-swing-tray-concept-v1"
    / "rack_pinion_swing_concept.py"
)
_V1_SPEC = spec_from_file_location("rack_pinion_swing_concept_v1", _V1_PATH)
v1 = module_from_spec(_V1_SPEC)
_V1_SPEC.loader.exec_module(v1)


GUIDE_X = 146.0
FRONT_PIN_Y0 = -32.0
REAR_PIN_Y0 = 28.0
PIN_Z_BELOW_TRAY = 3.5
GUIDE_PLATE_THICKNESS = 4.0
GUIDE_BODY_WIDTH = 12.0
GUIDE_END_MARGIN = 7.0
SLOT_WIDTH = 6.2
ROLLER_DIAMETER = 5.0
ROLLER_LENGTH = 12.0
POST_Y_DEPTH = 3.0

GUIDE_BLUE = Color(0.10, 0.48, 0.72)
CARRIAGE_GREEN = Color(0.18, 0.68, 0.38)


def _label(shape, name, color):
    shape.label = name
    shape.color = color
    return shape


def _x_cylinder(radius, length, x, y, z):
    return Cylinder(
        radius,
        length,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Rot(0, 90, 0)).moved(Pos(x - length / 2.0, y, z))


def _inclined_box(x_size, y_length, z_width, center_y, center_z):
    angle = degrees(atan2(v1.KEYBOARD_TRAVEL_Z, v1.KEYBOARD_TRAVEL_Y))
    return Box(
        x_size,
        y_length,
        z_width,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Rot(angle, 0, 0)).moved(Pos(0, center_y, center_z))


def _inclined_capsule(x_size, straight_length, width, center_y, center_z):
    angle = degrees(atan2(v1.KEYBOARD_TRAVEL_Z, v1.KEYBOARD_TRAVEL_Y))
    body = Box(
        x_size,
        straight_length,
        width,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    end_a = _x_cylinder(width / 2.0, x_size, 0, -straight_length / 2.0, 0)
    end_b = _x_cylinder(width / 2.0, x_size, 0, straight_length / 2.0, 0)
    return (body + end_a + end_b).moved(Rot(angle, 0, 0)).moved(
        Pos(0, center_y, center_z)
    )


def carriage_pin_center(position, t):
    if position not in ("left_front", "right_front", "left_rear", "right_rear"):
        raise ValueError(f"unknown carriage position: {position}")
    if not 0.0 <= t <= 1.0:
        raise ValueError("pose t must be within [0, 1]")
    x = -GUIDE_X if position.startswith("left") else GUIDE_X
    y0 = FRONT_PIN_Y0 if position.endswith("front") else REAR_PIN_Y0
    return (
        x,
        y0 + v1.KEYBOARD_TRAVEL_Y * t,
        v1.KEYBOARD_TRAY_Z0 + v1.KEYBOARD_TRAVEL_Z * t - PIN_Z_BELOW_TRAY,
    )


def _fixed_guide(position):
    start = carriage_pin_center(position, 0.0)
    end = carriage_pin_center(position, 1.0)
    x = start[0]
    center_y = (start[1] + end[1]) / 2.0
    center_z = (start[2] + end[2]) / 2.0
    travel = hypot(v1.KEYBOARD_TRAVEL_Y, v1.KEYBOARD_TRAVEL_Z)

    plate = _inclined_box(
        GUIDE_PLATE_THICKNESS,
        travel + 2.0 * GUIDE_END_MARGIN,
        GUIDE_BODY_WIDTH,
        center_y,
        center_z,
    ).moved(Pos(x, 0, 0))
    slot = _inclined_capsule(
        GUIDE_PLATE_THICKNESS + 2.0,
        travel,
        SLOT_WIDTH,
        center_y,
        center_z,
    ).moved(Pos(x, 0, 0))
    rail = plate - slot

    # Two grounded piers make each guide pod a real fixed support rather than a
    # floating slot.  Their tops overlap the rail at each travel endpoint.
    unit_y = v1.KEYBOARD_TRAVEL_Y / travel
    unit_z = v1.KEYBOARD_TRAVEL_Z / travel
    post_offset = GUIDE_END_MARGIN
    post_centers = (
        (start[1] - unit_y * post_offset, start[2] - unit_z * post_offset),
        (end[1] + unit_y * post_offset, end[2] + unit_z * post_offset),
    )
    posts = []
    for y, z in post_centers:
        post_top = z + 1.0
        post_height = post_top - v1.BASE_TOP_Z
        posts.append(
            Box(
                GUIDE_PLATE_THICKNESS,
                POST_Y_DEPTH,
                post_height,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            ).moved(Pos(x, y, v1.BASE_TOP_Z))
        )
    guide = rail + posts[0] + posts[1]
    return _label(guide, f"fixed_keyboard_guide_{position}", GUIDE_BLUE)


def _moving_carriage(position, t):
    x, y, z = carriage_pin_center(position, t)
    side = -1.0 if position.startswith("left") else 1.0
    tray_edge_x = side * v1.KEYBOARD_TRAY_WIDTH / 2.0
    tray_attach_x = tray_edge_x - side * 8.0
    inner_rail_face = x - side * (GUIDE_PLATE_THICKNESS / 2.0 + 0.5)
    bracket_center_x = (tray_attach_x + inner_rail_face) / 2.0
    bracket_width_x = abs(inner_rail_face - tray_attach_x)

    bracket = Box(
        bracket_width_x,
        8.0,
        6.5,
        align=(Align.CENTER, Align.CENTER, Align.MAX),
    ).moved(Pos(bracket_center_x, y, v1.KEYBOARD_TRAY_Z0 + v1.KEYBOARD_TRAVEL_Z * t))
    roller = _x_cylinder(ROLLER_DIAMETER / 2.0, ROLLER_LENGTH, x, y, z)
    carriage = bracket + roller
    return _label(carriage, f"keyboard_carriage_{position}", CARRIAGE_GREEN)


def build_pose(t, label):
    base_assembly = v1.build_pose(t, label)
    positions = ("left_front", "right_front", "left_rear", "right_rear")
    children = list(base_assembly.children)
    children.extend(_fixed_guide(position) for position in positions)
    children.extend(_moving_carriage(position, t) for position in positions)
    return Compound(label=label, children=children)


def build_section_exploded():
    return v1.build_section_exploded()
