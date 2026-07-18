"""STEP-first rack-pinion + vertical-axis eccentric swing-tray concept.

Independent V1 concept. It does not import or modify cross-axis V1-V11 geometry.
Coordinate system: X left/right, +Y toward display, +Z up. Units are mm.
"""

from math import acos, atan2, cos, degrees, hypot, pi, radians, sin, sqrt, tan

from build123d import (
    Align,
    Box,
    Color,
    Compound,
    Cylinder,
    Face,
    Pos,
    Rot,
    Solid,
    Vector,
    Wire,
)


# Device/tray envelopes
KEYBOARD_TRAY_WIDTH = 279.7
KEYBOARD_TRAY_DEPTH = 115.7
TRACKPAD_TRAY_WIDTH = 160.8
TRACKPAD_TRAY_DEPTH = 115.7
TRAY_THICKNESS = 3.5

# Continuous desktop base and six replaceable non-slip feet.
BASE_WIDTH = 310.0
BASE_DEPTH = 300.0
BASE_CENTER_Y = 90.9
BASE_THICKNESS = 4.0
FOOT_HEIGHT = 3.0
BASE_Z0 = FOOT_HEIGHT
BASE_TOP_Z = BASE_Z0 + BASE_THICKNESS

# Endpoint motion, chosen so 20.42 / 13 ~= pi/2.
KEYBOARD_TRAVEL_Y = 20.42
KEYBOARD_TRAVEL_Z = -15.0
SWING_TRAVEL_DEG = degrees(KEYBOARD_TRAVEL_Y / 13.0)
KEYBOARD_TRAY_Z0 = 40.0 + BASE_TOP_Z
TRACKPAD_TRAY_Z = 54.5 + BASE_TOP_Z

# Right-side vertical shaft and exact right-aligned trackpad target.
KEYBOARD_CENTER_X = 0.0
SHAFT_X = 120.0
SHAFT_Y = 99.5
TRACKPAD_WORK_CENTER_X = (
    KEYBOARD_CENTER_X
    + KEYBOARD_TRAY_WIDTH / 2.0
    - TRACKPAD_TRAY_WIDTH / 2.0
)
TRACKPAD_WORK_CENTER_Y = 0.0
WORK_VECTOR_X = TRACKPAD_WORK_CENTER_X - SHAFT_X
WORK_VECTOR_Y = TRACKPAD_WORK_CENTER_Y - SHAFT_Y
ARM_RADIUS = hypot(WORK_VECTOR_X, WORK_VECTOR_Y)

# Rack and pinion concept geometry.
GEAR_MODULE = 1.0
PINION_TEETH = 26
PITCH_RADIUS = GEAR_MODULE * PINION_TEETH / 2.0
PRESSURE_ANGLE_DEG = 20.0
PRESSURE_ANGLE_RAD = radians(PRESSURE_ANGLE_DEG)
GEAR_ROOT_RADIUS = PITCH_RADIUS - 1.25 * GEAR_MODULE
GEAR_BASE_RADIUS = PITCH_RADIUS * cos(PRESSURE_ANGLE_RAD)
GEAR_OUTER_RADIUS = 14.0
GEAR_FACE_Z0 = 19.0 + BASE_TOP_Z
GEAR_FACE_WIDTH = 22.0
RACK_PITCH_X = SHAFT_X + PITCH_RADIUS
RACK_BODY_MIN_X = RACK_PITCH_X + 1.15
RACK_BODY_MAX_X = 137.5
RACK_BODY_X = (RACK_BODY_MIN_X + RACK_BODY_MAX_X) / 2.0
RACK_BODY_WIDTH = RACK_BODY_MAX_X - RACK_BODY_MIN_X
RACK_LENGTH = 60.0
RACK_Z0 = 15.5 + BASE_TOP_Z
RACK_TOOTH_HEIGHT_Z = 30.0
TOOTH_PITCH = pi * GEAR_MODULE
RACK_BACKLASH = 0.12

# Fixed cylinder and internal support stack.
CYLINDER_OD = 38.0
CYLINDER_ID = 33.0
CYLINDER_Z0 = 8.0 + BASE_TOP_Z
CYLINDER_HEIGHT = 46.0
SHAFT_DIAMETER = 8.0
LOWER_BEARING_Z0 = 12.0 + BASE_TOP_Z
UPPER_BEARING_Z0 = 42.0 + BASE_TOP_Z
BEARING_SEAT_HEIGHT = 5.0
BEARING_SEAT_OD = 28.0
BEARING_BORE = 16.2  # visual 688-style bearing pocket/interface

# Colors make the motion relationship legible in Explorer while labels remain primary.
KEYBOARD_RED = Color(0.82, 0.16, 0.12)
TRACKPAD_BLUE = Color(0.08, 0.38, 0.90)
RACK_ORANGE = Color(0.95, 0.42, 0.08)
GEAR_GOLD = Color(0.98, 0.70, 0.10)
SHAFT_SILVER = Color(0.72, 0.76, 0.80)
BEARING_CYAN = Color(0.12, 0.66, 0.72)
CYLINDER_GRAY = Color(0.22, 0.25, 0.29)
ARM_PURPLE = Color(0.48, 0.22, 0.75)
STOP_MAGENTA = Color(0.90, 0.18, 0.55)
THRUST_GREEN = Color(0.18, 0.68, 0.32)
BASE_DARK = Color(0.16, 0.18, 0.21)
FOOT_BLACK = Color(0.06, 0.07, 0.08)


def _label(shape, name, color):
    shape.label = name
    shape.color = color
    return shape


def _z_cylinder(radius, height, x, y, z0):
    return Cylinder(
        radius,
        height,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(x, y, z0))


def _radial_block(angle_deg, r0, r1, tangential_width, height, z0):
    radial_length = r1 - r0
    block = Box(
        radial_length,
        tangential_width,
        height,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos((r0 + r1) / 2.0, 0, z0))
    return block.moved(Rot(0, 0, angle_deg)).moved(Pos(SHAFT_X, SHAFT_Y, 0))


def _beam_between_xy(p1, p2, width, height, z0):
    x1, y1 = p1
    x2, y2 = p2
    length = hypot(x2 - x1, y2 - y1)
    angle = degrees(atan2(y2 - y1, x2 - x1))
    local = Box(
        length,
        width,
        height,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    local = local.moved(Rot(0, 0, angle)).moved(
        Pos((x1 + x2) / 2.0, (y1 + y2) / 2.0, z0)
    )
    end1 = _z_cylinder(width / 2.0, height, x1, y1, z0)
    end2 = _z_cylinder(width / 2.0, height, x2, y2, z0)
    return local + end1 + end2


def _main_base():
    base = Box(
        BASE_WIDTH,
        BASE_DEPTH,
        BASE_THICKNESS,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(0, BASE_CENTER_Y, BASE_Z0))
    return _label(base, "continuous_main_base", BASE_DARK)


def _replaceable_feet():
    feet = []
    index = 1
    for x in (-130.0, 0.0, 130.0):
        for y in (-39.1, 220.9):
            foot = _z_cylinder(8.0, FOOT_HEIGHT, x, y, 0.0)
            foot = _label(foot, f"replaceable_tpu_foot_{index}", FOOT_BLACK)
            feet.append(foot)
            index += 1
    return Compound(label="replaceable_tpu_feet_6x", children=feet)


def _keyboard_tray(t):
    y = KEYBOARD_TRAVEL_Y * t
    z = KEYBOARD_TRAY_Z0 + KEYBOARD_TRAVEL_Z * t
    plate = Box(
        KEYBOARD_TRAY_WIDTH,
        KEYBOARD_TRAY_DEPTH,
        TRAY_THICKNESS,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(KEYBOARD_CENTER_X, y, z))
    # Low rear lip preserves a continuous keyboard support plane.
    rear_lip = Box(
        KEYBOARD_TRAY_WIDTH,
        1.8,
        2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Pos(
            KEYBOARD_CENTER_X,
            y + KEYBOARD_TRAY_DEPTH / 2.0 - 0.9,
            z + TRAY_THICKNESS,
        )
    )
    return _label(plate + rear_lip, "keyboard_tray_continuous", KEYBOARD_RED)


def _rack(t):
    y = 90.0 + KEYBOARD_TRAVEL_Y * t
    z0 = RACK_Z0 + KEYBOARD_TRAVEL_Z * t
    spine_z0 = z0 + 12.5
    body = Box(
        RACK_BODY_WIDTH,
        RACK_LENGTH,
        5.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(RACK_BODY_X, y, spine_z0))

    rack = body

    # Standard 20-degree full-depth rack teeth. A small backlash reduction is
    # applied to the rack tooth thickness; the pinion remains nominal size.
    addendum = GEAR_MODULE
    dedendum = 1.25 * GEAR_MODULE
    pitch_thickness = TOOTH_PITCH / 2.0 - RACK_BACKLASH
    tip_half_width = (
        pitch_thickness - 2.0 * addendum * tan(PRESSURE_ANGLE_RAD)
    ) / 2.0
    root_half_width = (
        pitch_thickness + 2.0 * dedendum * tan(PRESSURE_ANGLE_RAD)
    ) / 2.0
    tip_x = RACK_PITCH_X - addendum
    root_x = RACK_PITCH_X + dedendum
    rack_min_y = y - RACK_LENGTH / 2.0
    rack_max_y = y + RACK_LENGTH / 2.0
    rack_displacement = KEYBOARD_TRAVEL_Y * t
    tooth_phase_y = SHAFT_Y + TOOTH_PITCH / 2.0 + rack_displacement
    for index in range(-64, 65):
        ty = tooth_phase_y + index * TOOTH_PITCH
        if not (
            rack_min_y + root_half_width
            <= ty
            <= rack_max_y - root_half_width
        ):
            continue
        tooth_points = [
            (root_x, ty - root_half_width, z0),
            (tip_x, ty - tip_half_width, z0),
            (tip_x, ty + tip_half_width, z0),
            (root_x, ty + root_half_width, z0),
        ]
        tooth_face = Face(Wire.make_polygon(tooth_points, close=True))
        tooth = Solid.extrude(
            tooth_face, Vector(0, 0, RACK_TOOTH_HEIGHT_Z)
        )
        rack = rack + tooth

    # A rear-edge bridge ties the rack to the moving tray. The rack begins
    # 2.15 mm behind the tray in both endpoints, so neither moving body enters
    # the fixed cylinder envelope before the dedicated rack window.
    tray_z = KEYBOARD_TRAY_Z0 + KEYBOARD_TRAVEL_Z * t
    tray_rear_y = KEYBOARD_TRAVEL_Y * t + KEYBOARD_TRAY_DEPTH / 2.0
    rack_front_y = y - RACK_LENGTH / 2.0
    bridge_depth = rack_front_y - tray_rear_y + 0.4
    bridge = Box(
        12.0,
        bridge_depth,
        max(tray_z - spine_z0, 3.0),
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Pos(
            RACK_BODY_MAX_X - 6.0,
            (rack_front_y + tray_rear_y) / 2.0,
            spine_z0,
        )
    )
    rack = rack + bridge
    return _label(rack, "keyboard_right_straight_rack", RACK_ORANGE)


def _polar_point(radius, angle_rad, z=0.0):
    return (radius * cos(angle_rad), radius * sin(angle_rad), z)


def _involute_value(radius):
    alpha = acos(GEAR_BASE_RADIUS / radius)
    return tan(alpha) - alpha


def _pinion_outline_points(rotation_rad):
    """Closed standard involute-like outline for a 20-degree spur pinion."""
    points = []
    tooth_pitch_angle = 2.0 * pi / PINION_TEETH
    half_tooth_angle = pi / (2.0 * PINION_TEETH)
    pitch_involute = tan(PRESSURE_ANGLE_RAD) - PRESSURE_ANGLE_RAD
    base_flank_angle = half_tooth_angle + pitch_involute
    flank_steps = 5

    for tooth_index in range(PINION_TEETH):
        center = rotation_rad + tooth_index * tooth_pitch_angle
        valley_left = center - tooth_pitch_angle / 2.0
        valley_right = center + tooth_pitch_angle / 2.0
        points.append(_polar_point(GEAR_ROOT_RADIUS, valley_left))
        points.append(_polar_point(GEAR_ROOT_RADIUS, center - base_flank_angle))
        points.append(_polar_point(GEAR_BASE_RADIUS, center - base_flank_angle))

        flank_radii = [
            GEAR_BASE_RADIUS
            + (GEAR_OUTER_RADIUS - GEAR_BASE_RADIUS) * step / flank_steps
            for step in range(1, flank_steps + 1)
        ]
        for radius in flank_radii:
            flank_angle = (
                half_tooth_angle
                + pitch_involute
                - _involute_value(radius)
            )
            points.append(_polar_point(radius, center - flank_angle))
        for radius in reversed(flank_radii):
            flank_angle = (
                half_tooth_angle
                + pitch_involute
                - _involute_value(radius)
            )
            points.append(_polar_point(radius, center + flank_angle))

        points.append(_polar_point(GEAR_BASE_RADIUS, center + base_flank_angle))
        points.append(_polar_point(GEAR_ROOT_RADIUS, center + base_flank_angle))
        points.append(_polar_point(GEAR_ROOT_RADIUS, valley_right))
    return points


def _pinion(t, z_offset=0.0):
    gear_angle_rad = KEYBOARD_TRAVEL_Y * t / PITCH_RADIUS
    outline = Wire.make_polygon(
        _pinion_outline_points(gear_angle_rad), close=True
    )
    gear = Solid.extrude(Face(outline), Vector(0, 0, GEAR_FACE_WIDTH))
    gear = gear.moved(Pos(SHAFT_X, SHAFT_Y, GEAR_FACE_Z0 + z_offset))
    bore = _z_cylinder(
        SHAFT_DIAMETER / 2.0 + 0.15,
        GEAR_FACE_WIDTH + 2.0,
        SHAFT_X,
        SHAFT_Y,
        GEAR_FACE_Z0 - 1.0 + z_offset,
    )
    return _label(gear - bore, "module_1_26t_pinion", GEAR_GOLD)


def _fixed_cylinder(sectioned=False):
    outer = _z_cylinder(CYLINDER_OD / 2.0, CYLINDER_HEIGHT, SHAFT_X, SHAFT_Y, CYLINDER_Z0)
    inner = _z_cylinder(CYLINDER_ID / 2.0, CYLINDER_HEIGHT - 2.0, SHAFT_X, SHAFT_Y, CYLINDER_Z0 + 1.0)
    shell = outer - inner

    # Full-height right-side access window admits the tall moving rack through
    # the shell, pedestal, and flange throughout the 15 mm downward stroke.
    rack_slot = Box(
        18.0,
        50.0,
        CYLINDER_Z0 + CYLINDER_HEIGHT - BASE_TOP_Z,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    ).moved(Pos(SHAFT_X + 7.0, SHAFT_Y, BASE_TOP_Z))

    foot = Box(
        40.0,
        50.0,
        4.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(SHAFT_X - 1.0, SHAFT_Y, BASE_TOP_Z))
    pedestal = _z_cylinder(CYLINDER_OD / 2.0, 4.0, SHAFT_X, SHAFT_Y, BASE_TOP_Z + 4.0)
    shaft_clearance = _z_cylinder(5.0, 8.5, SHAFT_X, SHAFT_Y, BASE_TOP_Z)
    fixed = shell + foot + pedestal - shaft_clearance - rack_slot

    if sectioned:
        # Remove the front half only in the diagnostic assembly.
        cut = Box(
            50.0,
            30.0,
            70.0,
            align=(Align.CENTER, Align.MAX, Align.MIN),
        ).moved(Pos(SHAFT_X, SHAFT_Y, 0))
        fixed = fixed - cut
    return _label(fixed, "fixed_protective_cylinder", CYLINDER_GRAY)


def _bearing_seat(label, z0, z_offset=0.0):
    ring = _z_cylinder(BEARING_SEAT_OD / 2.0, BEARING_SEAT_HEIGHT, SHAFT_X, SHAFT_Y, z0 + z_offset)
    pocket = _z_cylinder(BEARING_BORE / 2.0, BEARING_SEAT_HEIGHT + 2.0, SHAFT_X, SHAFT_Y, z0 - 1.0 + z_offset)
    return _label(ring - pocket, label, BEARING_CYAN)


def _thrust_interface(z_offset=0.0):
    ring = _z_cylinder(10.0, 1.8, SHAFT_X, SHAFT_Y, 9.8 + BASE_TOP_Z + z_offset)
    bore = _z_cylinder(4.25, 3.0, SHAFT_X, SHAFT_Y, 9.2 + BASE_TOP_Z + z_offset)
    return _label(ring - bore, "lower_axial_thrust_interface", THRUST_GREEN)


def _shaft(z_offset=0.0):
    shaft = _z_cylinder(SHAFT_DIAMETER / 2.0, 46.0, SHAFT_X, SHAFT_Y, 8.0 + BASE_TOP_Z + z_offset)
    return _label(shaft, "vertical_drive_shaft", SHAFT_SILVER)


def _pose_vectors(t):
    delta_deg = -SWING_TRAVEL_DEG * (1.0 - t)
    angle = radians(delta_deg)
    vx = WORK_VECTOR_X * cos(angle) - WORK_VECTOR_Y * sin(angle)
    vy = WORK_VECTOR_X * sin(angle) + WORK_VECTOR_Y * cos(angle)
    return delta_deg, vx, vy


def _swing_arm(t, z_offset=0.0):
    _, vx, vy = _pose_vectors(t)
    center = (SHAFT_X + vx, SHAFT_Y + vy)
    arm = _beam_between_xy((SHAFT_X, SHAFT_Y), center, 10.0, 3.5, 50.0 + BASE_TOP_Z + z_offset)
    hub = _z_cylinder(9.0, 3.5, SHAFT_X, SHAFT_Y, 50.0 + BASE_TOP_Z + z_offset)
    return _label(arm + hub, "eccentric_horizontal_swing_arm", ARM_PURPLE)


def _trackpad_tray(t, z_offset=0.0):
    delta_deg, vx, vy = _pose_vectors(t)
    cx = SHAFT_X + vx
    cy = SHAFT_Y + vy
    plate = Box(
        TRACKPAD_TRAY_WIDTH,
        TRACKPAD_TRAY_DEPTH,
        TRAY_THICKNESS,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    # 1.6 mm locating lips on left/right/rear; front stays open for the hand.
    left_lip = Box(1.8, TRACKPAD_TRAY_DEPTH, 1.6, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Pos(-TRACKPAD_TRAY_WIDTH / 2.0 + 0.9, 0, TRAY_THICKNESS)
    )
    right_lip = Box(1.8, TRACKPAD_TRAY_DEPTH, 1.6, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Pos(TRACKPAD_TRAY_WIDTH / 2.0 - 0.9, 0, TRAY_THICKNESS)
    )
    rear_lip = Box(TRACKPAD_TRAY_WIDTH, 1.8, 1.6, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Pos(0, TRACKPAD_TRAY_DEPTH / 2.0 - 0.9, TRAY_THICKNESS)
    )
    tray = plate + left_lip + right_lip + rear_lip
    tray = tray.moved(Rot(0, 0, delta_deg)).moved(Pos(cx, cy, TRACKPAD_TRAY_Z + z_offset))
    return _label(tray, "magic_trackpad_small_tray", TRACKPAD_BLUE)


def _hard_stops(t, explode_offset=0.0):
    # Rotating lug follows the same 90-degree shaft motion; fixed blocks carry endpoint loads.
    lug_angle = 27.4 + SWING_TRAVEL_DEG * t
    lug = _radial_block(lug_angle, 4.0, 12.0, 3.2, 2.8, 47.2 + BASE_TOP_Z + explode_offset)
    lug = _label(lug, "shaft_rotating_stop_lug", STOP_MAGENTA)
    stop_keyboard = _radial_block(23.5, 11.5, 16.0, 4.2, 3.2, 47.0 + BASE_TOP_Z)
    stop_keyboard = _label(stop_keyboard, "independent_hard_stop_keyboard", STOP_MAGENTA)
    stop_trackpad = _radial_block(121.3, 11.5, 16.0, 4.2, 3.2, 47.0 + BASE_TOP_Z)
    stop_trackpad = _label(stop_trackpad, "independent_hard_stop_trackpad", STOP_MAGENTA)
    return lug, stop_keyboard, stop_trackpad


def build_pose(t, label):
    if not 0.0 <= t <= 1.0:
        raise ValueError("pose t must be within [0, 1]")
    lug, stop_keyboard, stop_trackpad = _hard_stops(t)
    children = [
        _main_base(),
        _replaceable_feet(),
        _fixed_cylinder(),
        _thrust_interface(),
        _bearing_seat("lower_radial_bearing_seat", LOWER_BEARING_Z0),
        _pinion(t),
        _bearing_seat("upper_radial_bearing_seat", UPPER_BEARING_Z0),
        _shaft(),
        stop_keyboard,
        stop_trackpad,
        lug,
        _swing_arm(t),
        _keyboard_tray(t),
        _rack(t),
        _trackpad_tray(t),
    ]
    assembly = Compound(label=label, children=children)
    return assembly


def build_section_exploded():
    t = 0.5
    lug, stop_keyboard, stop_trackpad = _hard_stops(t, explode_offset=17.0)
    children = [
        _fixed_cylinder(sectioned=True),
        _rack(t),
        _thrust_interface(),
        _bearing_seat("lower_radial_bearing_seat", LOWER_BEARING_Z0),
        _pinion(t),
        _shaft(),
        _bearing_seat("upper_radial_bearing_seat_exploded", UPPER_BEARING_Z0, 12.0),
        stop_keyboard,
        stop_trackpad,
        lug,
        _swing_arm(t, z_offset=25.0),
    ]
    return Compound(label="cylinder_section_exploded", children=children)


# Source-level concept assertions: endpoint target and rack/pinion ratio only.
assert abs(
    TRACKPAD_WORK_CENTER_X + TRACKPAD_TRAY_WIDTH / 2.0
    - (KEYBOARD_CENTER_X + KEYBOARD_TRAY_WIDTH / 2.0)
) < 1e-9
assert abs(KEYBOARD_TRAVEL_Y / PITCH_RADIUS - pi / 2.0) < 0.002
assert abs(ARM_RADIUS - hypot(WORK_VECTOR_X, WORK_VECTOR_Y)) < 1e-9
