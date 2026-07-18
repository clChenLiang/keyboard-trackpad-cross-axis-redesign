"""Physicalized V3 main assembly for the rack-pinion swing tray.

V1/V2 remain untouched. V3 preserves the approved V2 mechanism and adds only
explicit carriage-to-tray mounts, guide-to-base feet, a removable cylinder
cover, and a rigid rack-to-tray transition. Coordinates are millimetres.
"""

from importlib.util import module_from_spec, spec_from_file_location
from math import cos, hypot, radians, sin
from pathlib import Path

from build123d import Align, Box, Color, Compound, Cylinder, Pos


_V2_PATH = (
    Path(__file__).resolve().parent.parent
    / "rack-pinion-swing-tray-concept-v2-supported"
    / "rack_pinion_swing_supported_concept.py"
)
_V2_SPEC = spec_from_file_location("rack_pinion_swing_supported_concept_v2", _V2_PATH)
v2 = module_from_spec(_V2_SPEC)
_V2_SPEC.loader.exec_module(v2)

POSITIONS = ("left_front", "right_front", "left_rear", "right_rear")

M4_CLEARANCE_DIAMETER = 4.5
M3_CLEARANCE_DIAMETER = 3.4

CARRIAGE_EAR_WIDTH_X = 14.0
CARRIAGE_EAR_DEPTH_Y = 12.0
CARRIAGE_EAR_THICKNESS = 4.0
CARRIAGE_EAR_GROSS_VOLUME = (
    CARRIAGE_EAR_WIDTH_X * CARRIAGE_EAR_DEPTH_Y * CARRIAGE_EAR_THICKNESS
)

GUIDE_FOOT_WIDTH_X = 14.0
GUIDE_FOOT_DEPTH_Y = 12.0
GUIDE_FOOT_THICKNESS = 3.0
GUIDE_FOOT_GROSS_VOLUME = 2.0 * GUIDE_FOOT_WIDTH_X * GUIDE_FOOT_DEPTH_Y * GUIDE_FOOT_THICKNESS

COVER_THICKNESS = 3.0
COVER_FASTENER_RADIUS = 14.0
COVER_FASTENER_ANGLES_DEG = (75.0, 105.0)
COVER_EXPLODE_Z = 18.0
TRAY_BUSHING_OUTER_DIAMETER = 7.0
TRAY_BUSHING_CLEARANCE_DIAMETER = 7.2

EAR_TEAL = Color(0.10, 0.72, 0.62)
FOOT_BLUE = Color(0.16, 0.38, 0.72)
COVER_GRAY = Color(0.45, 0.49, 0.54)
INTERFACE_CYAN = Color(0.18, 0.78, 0.86)
TRANSITION_ORANGE = Color(0.92, 0.34, 0.06)


def _label(shape, name, color):
    shape.label = name
    shape.color = color
    return shape


def _z_cylinder(radius, height, x, y, z0):
    return Cylinder(radius, height, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Pos(x, y, z0)
    )


def _cover_fastener_centers():
    return tuple(
        (
            v2.v1.SHAFT_X + COVER_FASTENER_RADIUS * cos(radians(angle)),
            v2.v1.SHAFT_Y + COVER_FASTENER_RADIUS * sin(radians(angle)),
        )
        for angle in COVER_FASTENER_ANGLES_DEG
    )


def carriage_mount_ear_center(position, t):
    if position not in POSITIONS:
        raise ValueError(f"unknown carriage position: {position}")
    if not 0.0 <= t <= 1.0:
        raise ValueError("pose t must be within [0, 1]")
    side = -1.0 if position.startswith("left") else 1.0
    y0 = v2.FRONT_PIN_Y0 if position.endswith("front") else v2.REAR_PIN_Y0
    tray_z = v2.v1.KEYBOARD_TRAY_Z0 + v2.v1.KEYBOARD_TRAVEL_Z * t
    return (
        side * (v2.v1.KEYBOARD_TRAY_WIDTH / 2.0 - 8.0),
        y0 + v2.v1.KEYBOARD_TRAVEL_Y * t,
        tray_z - CARRIAGE_EAR_THICKNESS,
    )


def _carriage_mount_ear(position, t):
    x, y, z0 = carriage_mount_ear_center(position, t)
    pad = Box(
        CARRIAGE_EAR_WIDTH_X,
        CARRIAGE_EAR_DEPTH_Y,
        CARRIAGE_EAR_THICKNESS,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(x, y, z0))
    bore = _z_cylinder(
        M4_CLEARANCE_DIAMETER / 2.0,
        CARRIAGE_EAR_THICKNESS + 2.0,
        x,
        y,
        z0 - 1.0,
    )
    return _label(pad - bore, f"carriage_tray_mount_ear_{position}", EAR_TEAL)


def _guide_mount_centers(position):
    start = v2.carriage_pin_center(position, 0.0)
    end = v2.carriage_pin_center(position, 1.0)
    travel = hypot(v2.v1.KEYBOARD_TRAVEL_Y, v2.v1.KEYBOARD_TRAVEL_Z)
    unit_y = v2.v1.KEYBOARD_TRAVEL_Y / travel
    return (
        (start[0], start[1] - unit_y * v2.GUIDE_END_MARGIN),
        (end[0], end[1] + unit_y * v2.GUIDE_END_MARGIN),
    )


def _fixed_guide_mount_foot(position):
    pads = []
    for x, y in _guide_mount_centers(position):
        pad = Box(
            GUIDE_FOOT_WIDTH_X,
            GUIDE_FOOT_DEPTH_Y,
            GUIDE_FOOT_THICKNESS,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Pos(x, y, v2.v1.BASE_TOP_Z))
        bore = _z_cylinder(
            M4_CLEARANCE_DIAMETER / 2.0,
            GUIDE_FOOT_THICKNESS + 2.0,
            x,
            y,
            v2.v1.BASE_TOP_Z - 1.0,
        )
        pads.append(pad - bore)
    return _label(
        Compound(children=pads),
        f"fixed_guide_mount_foot_{position}",
        FOOT_BLUE,
    )


def guide_base_mount_hole_tools(position):
    """Return the two through-cut tools coaxial with one guide-foot group."""
    if position not in POSITIONS:
        raise ValueError(f"unknown guide position: {position}")
    holes = [
        _z_cylinder(
            M4_CLEARANCE_DIAMETER / 2.0,
            v2.v1.BASE_THICKNESS + 2.0,
            x,
            y,
            v2.v1.BASE_Z0 - 1.0,
        )
        for x, y in _guide_mount_centers(position)
    ]
    return Compound(children=holes)


def _base_with_guide_mount_holes(base):
    drilled = base
    for position in POSITIONS:
        drilled = drilled - guide_base_mount_hole_tools(position)
    return _label(drilled, "continuous_main_base", v2.v1.BASE_DARK)


def cylinder_top_service_aperture_probe():
    """Tool representing the unobstructed service path below a removed cover."""
    return _z_cylinder(
        v2.v1.CYLINDER_ID / 2.0,
        3.0,
        v2.v1.SHAFT_X,
        v2.v1.SHAFT_Y,
        v2.v1.CYLINDER_Z0 + v2.v1.CYLINDER_HEIGHT - 1.5,
    )


def _open_top_fixed_cylinder(fixed_cylinder):
    opened = fixed_cylinder - cylinder_top_service_aperture_probe()
    return _label(opened, "fixed_protective_cylinder", v2.v1.CYLINDER_GRAY)


def keyboard_tray_mount_hole_tool(position, t):
    """M4 coaxial through-path shared by one carriage ear and tray bushing."""
    x, y, _ = carriage_mount_ear_center(position, t)
    tray_z = v2.v1.KEYBOARD_TRAY_Z0 + v2.v1.KEYBOARD_TRAVEL_Z * t
    return _z_cylinder(
        M4_CLEARANCE_DIAMETER / 2.0,
        v2.v1.TRAY_THICKNESS + 2.0,
        x,
        y,
        tray_z - 1.0,
    )


def _keyboard_tray_bushing_clearance_tool(position, t):
    x, y, _ = carriage_mount_ear_center(position, t)
    tray_z = v2.v1.KEYBOARD_TRAY_Z0 + v2.v1.KEYBOARD_TRAVEL_Z * t
    return _z_cylinder(
        TRAY_BUSHING_CLEARANCE_DIAMETER / 2.0,
        v2.v1.TRAY_THICKNESS + 2.0,
        x,
        y,
        tray_z - 1.0,
    )


def _keyboard_tray_with_mount_holes(tray, t):
    drilled = tray
    for position in POSITIONS:
        drilled = drilled - _keyboard_tray_bushing_clearance_tool(position, t)
    return _label(drilled, "keyboard_tray_continuous", v2.v1.KEYBOARD_RED)


def _keyboard_tray_mount_bushing(position, t):
    x, y, _ = carriage_mount_ear_center(position, t)
    tray_z = v2.v1.KEYBOARD_TRAY_Z0 + v2.v1.KEYBOARD_TRAVEL_Z * t
    outer = _z_cylinder(
        TRAY_BUSHING_OUTER_DIAMETER / 2.0,
        v2.v1.TRAY_THICKNESS,
        x,
        y,
        tray_z,
    )
    bushing = outer - keyboard_tray_mount_hole_tool(position, t)
    return _label(
        bushing,
        f"keyboard_tray_mount_bushing_{position}",
        INTERFACE_CYAN,
    )


def _cylinder_top_cover(z_offset=0.0, exploded=False):
    z0 = v2.v1.CYLINDER_Z0 + v2.v1.CYLINDER_HEIGHT + z_offset
    cover = _z_cylinder(v2.v1.CYLINDER_OD / 2.0, COVER_THICKNESS, v2.v1.SHAFT_X, v2.v1.SHAFT_Y, z0)
    shaft_bore = _z_cylinder(
        v2.v1.SHAFT_DIAMETER / 2.0 + 0.35,
        COVER_THICKNESS + 2.0,
        v2.v1.SHAFT_X,
        v2.v1.SHAFT_Y,
        z0 - 1.0,
    )
    cover = cover - shaft_bore
    for x, y in _cover_fastener_centers():
        fastener_bore = _z_cylinder(
            M3_CLEARANCE_DIAMETER / 2.0,
            COVER_THICKNESS + 2.0,
            x,
            y,
            z0 - 1.0,
        )
        cover = cover - fastener_bore
    suffix = "_exploded" if exploded else ""
    return _label(cover, f"fixed_cylinder_removable_top_cover{suffix}", COVER_GRAY)


def _cover_fastener_interface(z_offset=0.0):
    z0 = v2.v1.CYLINDER_Z0 + v2.v1.CYLINDER_HEIGHT - 4.0 + z_offset
    sleeves = []
    for x, y in _cover_fastener_centers():
        outer = _z_cylinder(3.0, 4.0, x, y, z0)
        bore = _z_cylinder(
            M3_CLEARANCE_DIAMETER / 2.0,
            6.0,
            x,
            y,
            z0 - 1.0,
        )
        sleeves.append(outer - bore)
    return _label(Compound(children=sleeves), "cylinder_top_cover_fastener_interface", INTERFACE_CYAN)


def _rack_to_keyboard_transition(t):
    tray_z = v2.v1.KEYBOARD_TRAY_Z0 + v2.v1.KEYBOARD_TRAVEL_Z * t
    rack_spine_z0 = v2.v1.RACK_Z0 + v2.v1.KEYBOARD_TRAVEL_Z * t + 12.5
    tray_rear_y = v2.v1.KEYBOARD_TRAVEL_Y * t + v2.v1.KEYBOARD_TRAY_DEPTH / 2.0
    rack_front_y = 90.0 + v2.v1.KEYBOARD_TRAVEL_Y * t - v2.v1.RACK_LENGTH / 2.0
    transition_depth = 7.0
    transition = Box(
        5.5,
        transition_depth,
        tray_z - rack_spine_z0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Pos(
            v2.v1.RACK_BODY_MAX_X - 2.75,
            (tray_rear_y + rack_front_y) / 2.0,
            rack_spine_z0,
        )
    )
    return _label(transition, "rack_to_keyboard_rigid_transition", TRANSITION_ORANGE)


def build_pose(t, label):
    if not 0.0 <= t <= 1.0:
        raise ValueError("pose t must be within [0, 1]")
    base_assembly = v2.build_pose(t, label)
    children = [
        _base_with_guide_mount_holes(child)
        if child.label == "continuous_main_base"
        else _open_top_fixed_cylinder(child)
        if child.label == "fixed_protective_cylinder"
        else _keyboard_tray_with_mount_holes(child, t)
        if child.label == "keyboard_tray_continuous"
        else child
        for child in base_assembly.children
    ]
    children.extend(_carriage_mount_ear(position, t) for position in POSITIONS)
    children.extend(_keyboard_tray_mount_bushing(position, t) for position in POSITIONS)
    children.extend(_fixed_guide_mount_foot(position) for position in POSITIONS)
    children.extend(
        [
            _cylinder_top_cover(),
            _cover_fastener_interface(),
            _rack_to_keyboard_transition(t),
        ]
    )
    return Compound(label=label, children=children)


def build_section_exploded():
    base_assembly = v2.build_section_exploded()
    children = [
        _open_top_fixed_cylinder(child)
        if child.label == "fixed_protective_cylinder"
        else child
        for child in base_assembly.children
    ]
    children.extend(
        [
            _cylinder_top_cover(COVER_EXPLODE_Z, exploded=True),
            _cover_fastener_interface(),
        ]
    )
    return Compound(label="cylinder_section_exploded_v3", children=children)
