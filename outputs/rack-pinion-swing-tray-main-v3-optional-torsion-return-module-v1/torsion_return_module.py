"""Optional, bottom-mounted coaxial torsion-return cartridge for V3."""

from importlib.util import module_from_spec, spec_from_file_location
from math import cos, pi, radians, sin
from pathlib import Path

from build123d import Align, Box, Color, Compound, Cylinder, Edge, Face, Plane, Pos, Rot, Solid, Wire

_V3_PATH = Path(__file__).resolve().parent.parent / "rack-pinion-swing-tray-main-v3" / "rack_pinion_swing_main_v3.py"
_SPEC = spec_from_file_location("rack_pinion_swing_main_v3_optional_source", _V3_PATH)
v3 = module_from_spec(_SPEC)
_SPEC.loader.exec_module(v3)

SHAFT_X = v3.v2.v1.SHAFT_X
SHAFT_Y = v3.v2.v1.SHAFT_Y
BASE_Z0 = v3.v2.v1.BASE_Z0
SWING_DEG = 90.0

CUP_OD = 34.0
CUP_INNER_DIAMETER = 23.0
CUP_Z0 = -11.0
CUP_BODY_HEIGHT = 12.0
FLANGE_OD = 38.0
FLANGE_Z0 = 1.0
FLANGE_HEIGHT = 2.0
MOUNT_BCD_RADIUS = 13.5
M3_CLEARANCE = 3.4
SPRING_WIRE = 1.2
SPRING_CENTERLINE_RADIUS = 9.5
SPRING_Z0 = -9.8
SPRING_HEIGHT = 8.0
SPRING_TURNS = 4.5
CLAMP_OD = 13.0
CLAMP_ID = 8.3
CLAMP_Z0 = -4.0
CLAMP_HEIGHT = 5.0
BOTTOM_COVER_Z0 = -13.0
BOTTOM_COVER_HEIGHT = 2.0
BOTTOM_COVER_SCREW_RADIUS = 14.2
BOTTOM_COVER_SCREW_DIAMETER = 2.7
PRELOAD_ANGLES_DEG = (-18.0, 0.0, 18.0)
SELECTED_PRELOAD_ANGLE_DEG = 0.0
PRELOAD_HOLE_WIDTH = 2.0
DRIVE_KEY_ANGLE_OFFSET_DEG = 90.0

FIXED_GRAY = Color(0.24, 0.28, 0.32)
SPRING_BLUE = Color(0.18, 0.64, 0.92)
ROTATING_GOLD = Color(0.95, 0.58, 0.08)
ANCHOR_GREEN = Color(0.22, 0.72, 0.36)
SHAFT_SILVER = Color(0.72, 0.76, 0.80)
FOOT_BLACK = Color(0.06, 0.07, 0.08)


def _label(shape, name, color):
    shape.label = name
    shape.color = color
    return shape


def z_cylinder(radius, height, x, y, z0):
    return Cylinder(radius, height, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Pos(x, y, z0))


def shaft_angle_deg(t):
    if not 0.0 <= t <= 1.0:
        raise ValueError("pose t must be within [0, 1]")
    return SWING_DEG * t


def _polar(radius, angle_deg, z):
    a = radians(angle_deg)
    return SHAFT_X + radius * cos(a), SHAFT_Y + radius * sin(a), z


def rotating_capture_center(t):
    return _polar(8.0, 180.0 + shaft_angle_deg(t), -0.8)


def _radial_block(angle_deg, r0, r1, tangential_width, height, z0):
    block = Box(
        r1 - r0,
        tangential_width,
        height,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos((r0 + r1) / 2.0, 0.0, z0))
    return block.moved(Rot(0, 0, angle_deg)).moved(Pos(SHAFT_X, SHAFT_Y, 0))


def spring_outer_diameter():
    return 2.0 * (SPRING_CENTERLINE_RADIUS + SPRING_WIRE / 2.0)


def spring_installation_access_tool():
    return z_cylinder(CUP_INNER_DIAMETER / 2.0 - 0.2, 3.0, SHAFT_X, SHAFT_Y, CUP_Z0 - 1.0)


def preload_hole_tool(angle_deg):
    return _radial_block(angle_deg, 9.5, 18.5, PRELOAD_HOLE_WIDTH, 2.0, SPRING_Z0 - 0.3)


def fixed_cup_uncut():
    body = z_cylinder(CUP_OD / 2, CUP_BODY_HEIGHT, SHAFT_X, SHAFT_Y, CUP_Z0)
    cavity = z_cylinder(CUP_INNER_DIAMETER / 2, CUP_BODY_HEIGHT + 2.0, SHAFT_X, SHAFT_Y, CUP_Z0 - 1.0)
    flange = z_cylinder(FLANGE_OD / 2, FLANGE_HEIGHT, SHAFT_X, SHAFT_Y, FLANGE_Z0)
    flange_bore = z_cylinder(5.0, FLANGE_HEIGHT + 2.0, SHAFT_X, SHAFT_Y, FLANGE_Z0 - 1.0)
    return body - cavity + (flange - flange_bore)


def _fixed_cup():
    part = fixed_cup_uncut()
    part = part - z_cylinder(5.0, 16.0, SHAFT_X, SHAFT_Y, CUP_Z0 - 1)
    for angle in (90.0, 210.0, 330.0):
        x, y, _ = _polar(MOUNT_BCD_RADIUS, angle, 0)
        part = part - z_cylinder(M3_CLEARANCE / 2, 16.0, x, y, CUP_Z0 - 1)
    for angle in PRELOAD_ANGLES_DEG:
        part = part - preload_hole_tool(angle)
    for angle in (30.0, 150.0, 270.0):
        x, y, _ = _polar(BOTTOM_COVER_SCREW_RADIUS, angle, 0)
        part = part - z_cylinder(BOTTOM_COVER_SCREW_DIAMETER / 2.0, 3.0, x, y, CUP_Z0 - 0.5)
    return _label(part, "torsion_fixed_mounting_cup", FIXED_GRAY)


def _removable_bottom_cover():
    cover = z_cylinder(CUP_OD / 2.0, BOTTOM_COVER_HEIGHT, SHAFT_X, SHAFT_Y, BOTTOM_COVER_Z0)
    cover = cover - z_cylinder(4.3, BOTTOM_COVER_HEIGHT + 2.0, SHAFT_X, SHAFT_Y, BOTTOM_COVER_Z0 - 1.0)
    for angle in (30.0, 150.0, 270.0):
        x, y, _ = _polar(BOTTOM_COVER_SCREW_RADIUS, angle, 0)
        cover = cover - z_cylinder(BOTTOM_COVER_SCREW_DIAMETER / 2.0, BOTTOM_COVER_HEIGHT + 2.0, x, y, BOTTOM_COVER_Z0 - 1.0)
    return _label(cover, "torsion_removable_bottom_cover", FIXED_GRAY)


def _helical_spring(t):
    turns = SPRING_TURNS + 0.25 * t
    path = Edge.make_helix(
        SPRING_HEIGHT / turns,
        SPRING_HEIGHT,
        SPRING_CENTERLINE_RADIUS,
        center=(SHAFT_X, SHAFT_Y, SPRING_Z0),
        normal=(0, 0, 1),
    )
    profile = Face(Wire.make_circle(SPRING_WIRE / 2, Plane(path.position_at(0), z_dir=path.tangent_at(0))))
    coil = Solid.sweep(profile, path, is_frenet=True)
    # Tangential-looking end legs make both load paths explicit.
    fixed_leg = v3.v2.v1._beam_between_xy(
        (SHAFT_X + 7.4, SHAFT_Y), (SHAFT_X + 10.2, SHAFT_Y), SPRING_WIRE, SPRING_WIRE, SPRING_Z0
    )
    angle = 180.0 + shaft_angle_deg(t)
    p1 = _polar(6.0, angle, 0)
    p2 = _polar(SPRING_CENTERLINE_RADIUS, angle, 0)
    moving_leg = v3.v2.v1._beam_between_xy(
        (p1[0], p1[1]), (p2[0], p2[1]), SPRING_WIRE, SPRING_WIRE, SPRING_Z0 + SPRING_HEIGHT - SPRING_WIRE
    )
    return _label(Compound(children=[coil, fixed_leg, moving_leg]), "torsion_helical_spring", SPRING_BLUE)


def rotating_clamp_uncut(t):
    ring = z_cylinder(CLAMP_OD / 2, CLAMP_HEIGHT, SHAFT_X, SHAFT_Y, CLAMP_Z0)
    bore = z_cylinder(CLAMP_ID / 2, CLAMP_HEIGHT + 2, SHAFT_X, SHAFT_Y, CLAMP_Z0 - 1)
    return ring - bore


def rotating_capture_slot_tool(t):
    return _radial_block(
        180.0 + shaft_angle_deg(t), 5.7, 7.2, 1.7, 1.5, SPRING_Z0 + SPRING_HEIGHT - SPRING_WIRE - 0.15
    )


def clamp_keyway_tool(t=0.5):
    return _radial_block(
        shaft_angle_deg(t) + DRIVE_KEY_ANGLE_OFFSET_DEG, 3.5, 6.8, 1.2, CLAMP_HEIGHT + 2.0, CLAMP_Z0 - 1.0
    )


def extended_shaft_keyway_tool(t=0.5):
    return _radial_block(
        shaft_angle_deg(t) + DRIVE_KEY_ANGLE_OFFSET_DEG, 3.5, 4.2, 1.2, 7.0, CLAMP_Z0 - 1.0
    )


def _rotating_clamp(t):
    # Capture notch is opposite the independent keyed shaft interface.
    part = rotating_clamp_uncut(t) - rotating_capture_slot_tool(t) - clamp_keyway_tool(t)
    return _label(part, "torsion_rotating_shaft_clamp", ROTATING_GOLD)


def _fixed_anchor_pin():
    shaft = _radial_block(SELECTED_PRELOAD_ANGLE_DEG, 9.4, 18.7, PRELOAD_HOLE_WIDTH, 1.8, SPRING_Z0 - 0.2)
    head = _radial_block(SELECTED_PRELOAD_ANGLE_DEG, 17.0, 19.2, 3.6, 2.8, SPRING_Z0 - 0.7)
    return _label(Compound(children=[shaft, head]), "torsion_fixed_end_anchor_pin", ANCHOR_GREEN)


def _shaft_drive_key(t):
    key = _radial_block(
        shaft_angle_deg(t) + DRIVE_KEY_ANGLE_OFFSET_DEG, 3.5, 6.25, 1.2, CLAMP_HEIGHT - 0.6, CLAMP_Z0 + 0.3
    )
    return _label(key, "torsion_shaft_drive_key", ANCHOR_GREEN)


def build_module(t=0.0):
    return Compound(label="optional_torsion_return_module", children=[
        _fixed_cup(), _removable_bottom_cover(), _helical_spring(t), _rotating_clamp(t), _fixed_anchor_pin(), _shaft_drive_key(t)
    ])


def _drilled_base(base):
    drilled = base - z_cylinder(5.0, 7.0, SHAFT_X, SHAFT_Y, 2.0)
    for angle in (90.0, 210.0, 330.0):
        x, y, _ = _polar(MOUNT_BCD_RADIUS, angle, 0)
        drilled = drilled - z_cylinder(M3_CLEARANCE / 2, 7.0, x, y, 2.0)
    return _label(drilled, "continuous_main_base_optional_return_drilled", v3.v2.v1.BASE_DARK)


def _extended_shaft(t):
    shaft = z_cylinder(4.0, 72.0, SHAFT_X, SHAFT_Y, -11.0) - extended_shaft_keyway_tool(t)
    return _label(shaft, "vertical_drive_shaft_optional_return_extended", SHAFT_SILVER)


def _clearance_feet():
    feet = []
    for x in (-130.0, 0.0, 130.0):
        for y in (-39.1, 220.9):
            feet.append(z_cylinder(8.0, 18.0, x, y, -15.0))
    return _label(Compound(children=feet), "optional_return_clearance_feet", FOOT_BLACK)


def build_v3_with_module(t):
    source = v3.build_pose(t, f"v3_optional_return_{t}")
    children = []
    for child in source.children:
        if child.label == "continuous_main_base":
            children.append(_drilled_base(child))
        elif child.label in ("vertical_drive_shaft", "replaceable_tpu_feet_6x"):
            continue
        else:
            children.append(child)
    children.extend([_extended_shaft(t), _clearance_feet()])
    children.extend(build_module(t).children)
    return Compound(label=f"v3_with_optional_torsion_return_{t}", children=children)


def build_exploded():
    children = list(build_module(0.5).children)
    offsets = {
        "torsion_helical_spring": 15.0,
        "torsion_rotating_shaft_clamp": 27.0,
        "torsion_shaft_drive_key": 27.0,
        "torsion_fixed_end_anchor_pin": 8.0,
        "torsion_removable_bottom_cover": -8.0,
    }
    moved = []
    for part in children:
        dz = offsets.get(part.label, 0.0)
        q = part.moved(Pos(0, 0, dz))
        q.label, q.color = part.label + ("_exploded" if dz else ""), part.color
        moved.append(q)
    return Compound(label="torsion_return_exploded", children=moved)


def gen_step():
    return build_module(0.0)
