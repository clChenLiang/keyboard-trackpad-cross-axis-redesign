"""V3 main assembly with integrated torsion return and rack retention.

Coordinates are millimetres. X is left/right, +Y points toward the display,
and +Z points upward. Earlier V3 and optional-return sources remain untouched.
"""

from importlib.util import module_from_spec, spec_from_file_location
from math import atan2, degrees, hypot
from pathlib import Path

from build123d import Align, Box, Color, Compound, Cylinder, Pos, Rot


ROOT = Path(__file__).resolve().parent.parent
TORSION_PATH = (
    ROOT
    / "rack-pinion-swing-tray-main-v3-optional-torsion-return-module-v1"
    / "torsion_return_module.py"
)
TORSION_SPEC = spec_from_file_location(
    "rack_pinion_swing_integrated_torsion_source", TORSION_PATH
)
torsion = module_from_spec(TORSION_SPEC)
TORSION_SPEC.loader.exec_module(torsion)
v3 = torsion.v3

RETURN_TARGET_T = 0.0
MAX_BOOLEAN_NOISE_VOLUME = 0.05

ROLLER_RADIUS = 3.0
ROLLER_LENGTH = 24.0
ROLLER_BORE_RADIUS = 1.35
ROLLER_AXLE_RADIUS = 1.15
ROLLER_CENTER_X = v3.v2.v1.RACK_BODY_MAX_X + ROLLER_RADIUS
ROLLER_CENTER_Z = 30.0
ROLLER_CENTERS_Y = {"datum": 91.0, "preload": 108.0}
ROLLER_TILT_X_DEG = -degrees(
    atan2(-v3.v2.v1.KEYBOARD_TRAVEL_Z, v3.v2.v1.KEYBOARD_TRAVEL_Y)
)

BRACKET_FOOT_X = 146.0
BRACKET_FOOT_Y = 99.5
BRACKET_FOOT_WIDTH = 15.0
BRACKET_FOOT_DEPTH = 43.0
BRACKET_FOOT_HEIGHT = 3.0
POST_WIDTH_X = 3.0
POST_DEPTH_Y = 3.2
POST_OVERLAP_Z = 1.0
FLEXURE_LENGTH_Y = 8.0
FLEXURE_THICKNESS_X = 1.2
FLEXURE_HEIGHT_Z = 2.4

BRACKET_BLUE = Color(0.10, 0.42, 0.72)
ROLLER_PURPLE = Color(0.65, 0.30, 0.88)
PRELOAD_GREEN = Color(0.20, 0.72, 0.42)
SHAFT_SILVER = Color(0.72, 0.76, 0.80)
FOOT_BLACK = Color(0.06, 0.07, 0.08)


def _label(shape, name, color):
    shape.label = name
    shape.color = color
    return shape


def rack_travel_unit():
    length = hypot(v3.v2.v1.KEYBOARD_TRAVEL_Y, v3.v2.v1.KEYBOARD_TRAVEL_Z)
    return (
        0.0,
        v3.v2.v1.KEYBOARD_TRAVEL_Y / length,
        v3.v2.v1.KEYBOARD_TRAVEL_Z / length,
    )


def roller_axis_unit():
    length = hypot(v3.v2.v1.KEYBOARD_TRAVEL_Y, v3.v2.v1.KEYBOARD_TRAVEL_Z)
    return (
        0.0,
        -v3.v2.v1.KEYBOARD_TRAVEL_Z / length,
        v3.v2.v1.KEYBOARD_TRAVEL_Y / length,
    )


def _oriented_cylinder(radius, length, x, y, z):
    return (
        Cylinder(
            radius,
            length,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        )
        .moved(Rot(ROLLER_TILT_X_DEG, 0, 0))
        .moved(Pos(x, y, z))
    )


def _axis_endpoints(name, extra=0.0):
    axis = roller_axis_unit()
    half = ROLLER_LENGTH / 2.0 + extra
    y = ROLLER_CENTERS_Y[name]
    return (
        (
            ROLLER_CENTER_X,
            y - axis[1] * half,
            ROLLER_CENTER_Z - axis[2] * half,
        ),
        (
            ROLLER_CENTER_X,
            y + axis[1] * half,
            ROLLER_CENTER_Z + axis[2] * half,
        ),
    )


def _roller(name):
    outer = _oriented_cylinder(
        ROLLER_RADIUS,
        ROLLER_LENGTH,
        ROLLER_CENTER_X,
        ROLLER_CENTERS_Y[name],
        ROLLER_CENTER_Z,
    )
    bore = _oriented_cylinder(
        ROLLER_BORE_RADIUS,
        ROLLER_LENGTH + 2.0,
        ROLLER_CENTER_X,
        ROLLER_CENTERS_Y[name],
        ROLLER_CENTER_Z,
    )
    return _label(
        outer - bore,
        f"rack_back_pressure_{name}_roller",
        ROLLER_PURPLE,
    )


def _support_post(x, y, top_z):
    height = top_z - v3.v2.v1.BASE_TOP_Z + POST_OVERLAP_Z
    return Box(
        POST_WIDTH_X,
        POST_DEPTH_Y,
        height,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(x, y, v3.v2.v1.BASE_TOP_Z))


def _fixed_bracket():
    foot = Box(
        BRACKET_FOOT_WIDTH,
        BRACKET_FOOT_DEPTH,
        BRACKET_FOOT_HEIGHT,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(BRACKET_FOOT_X, BRACKET_FOOT_Y, v3.v2.v1.BASE_TOP_Z))

    datum_ends = _axis_endpoints("datum", extra=1.0)
    datum_posts = [
        _support_post(ROLLER_CENTER_X, y, z) for _, y, z in datum_ends
    ]
    datum_axle = _oriented_cylinder(
        ROLLER_AXLE_RADIUS,
        ROLLER_LENGTH + 3.0,
        ROLLER_CENTER_X,
        ROLLER_CENTERS_Y["datum"],
        ROLLER_CENTER_Z,
    )

    preload_ends = _axis_endpoints("preload", extra=1.0)
    preload_roots = [
        _support_post(ROLLER_CENTER_X, y + FLEXURE_LENGTH_Y, z)
        for _, y, z in preload_ends
    ]
    return _label(
        Compound(children=[foot, datum_axle, *datum_posts, *preload_roots]),
        "rack_retention_fixed_bracket",
        BRACKET_BLUE,
    )


def _preload_flexure():
    leaves = []
    for _, y, z in _axis_endpoints("preload"):
        leaves.append(
            Box(
                FLEXURE_THICKNESS_X,
                FLEXURE_LENGTH_Y,
                FLEXURE_HEIGHT_Z,
                align=(Align.CENTER, Align.MIN, Align.CENTER),
            ).moved(Pos(ROLLER_CENTER_X, y, z))
        )
    axle = _oriented_cylinder(
        ROLLER_AXLE_RADIUS,
        ROLLER_LENGTH + 1.0,
        ROLLER_CENTER_X,
        ROLLER_CENTERS_Y["preload"],
        ROLLER_CENTER_Z,
    )
    return _label(
        Compound(children=[*leaves, axle]),
        "rack_back_pressure_flexure",
        PRELOAD_GREEN,
    )


def build_retention_parts():
    return {
        "rack_retention_fixed_bracket": _fixed_bracket(),
        "rack_back_pressure_datum_roller": _roller("datum"),
        "rack_back_pressure_preload_roller": _roller("preload"),
        "rack_back_pressure_flexure": _preload_flexure(),
    }


def roller_rack_contact_point(name, t):
    if name not in ROLLER_CENTERS_Y:
        raise ValueError(f"unknown roller: {name}")
    if not 0.0 <= t <= 1.0:
        raise ValueError("pose t must be within [0, 1]")
    rack_body_center_z = (
        v3.v2.v1.RACK_Z0
        + v3.v2.v1.KEYBOARD_TRAVEL_Z * t
        + 12.5
        + 2.5
    )
    axis = roller_axis_unit()
    along = (rack_body_center_z - ROLLER_CENTER_Z) / axis[2]
    return (
        v3.v2.v1.RACK_BODY_MAX_X,
        ROLLER_CENTERS_Y[name] + along * axis[1],
        rack_body_center_z,
    )


def contact_point_is_on_roller(name, point):
    axis = roller_axis_unit()
    dy = point[1] - ROLLER_CENTERS_Y[name]
    dz = point[2] - ROLLER_CENTER_Z
    along = dy * axis[1] + dz * axis[2]
    radial_error = abs(
        (point[0] - ROLLER_CENTER_X) * (point[0] - ROLLER_CENTER_X)
        - ROLLER_RADIUS * ROLLER_RADIUS
    )
    return abs(along) <= ROLLER_LENGTH / 2.0 + 1e-6 and radial_error <= 1e-5


def contact_point_is_on_rack_body(t, point):
    rack_y = 90.0 + v3.v2.v1.KEYBOARD_TRAVEL_Y * t
    rack_min_y = rack_y - v3.v2.v1.RACK_LENGTH / 2.0
    rack_max_y = rack_y + v3.v2.v1.RACK_LENGTH / 2.0
    rack_spine_z0 = (
        v3.v2.v1.RACK_Z0
        + v3.v2.v1.KEYBOARD_TRAVEL_Z * t
        + 12.5
    )
    return (
        abs(point[0] - v3.v2.v1.RACK_BODY_MAX_X) <= 1e-6
        and rack_min_y <= point[1] <= rack_max_y
        and rack_spine_z0 <= point[2] <= rack_spine_z0 + 5.0
    )


def spring_wind_turns(t):
    if not 0.0 <= t <= 1.0:
        raise ValueError("pose t must be within [0, 1]")
    return torsion.SPRING_TURNS + 0.25 * t


def _integrated_base(base):
    part = torsion._drilled_base(base)
    return _label(
        part,
        "continuous_main_base_integrated_return_drilled",
        v3.v2.v1.BASE_DARK,
    )


def _integrated_shaft(t):
    part = torsion._extended_shaft(t)
    return _label(
        part,
        "vertical_drive_shaft_integrated_return",
        SHAFT_SILVER,
    )


def _integrated_clearance_feet():
    part = torsion._clearance_feet()
    return _label(part, "integrated_return_clearance_feet", FOOT_BLACK)


def build_pose(t):
    if not 0.0 <= t <= 1.0:
        raise ValueError("pose t must be within [0, 1]")
    source = v3.build_pose(t, f"v3_integrated_return_source_{t}")
    children = []
    for child in source.children:
        if child.label == "continuous_main_base":
            children.append(_integrated_base(child))
        elif child.label in ("vertical_drive_shaft", "replaceable_tpu_feet_6x"):
            continue
        else:
            children.append(child)
    children.extend([_integrated_shaft(t), _integrated_clearance_feet()])
    children.extend(torsion.build_module(t).children)
    children.extend(build_retention_parts().values())
    return Compound(label=f"v3_integrated_return_{t}", children=children)


def build_section_exploded():
    children = list(v3.build_section_exploded().children)
    children.extend(build_retention_parts().values())
    for part in torsion.build_exploded().children:
        children.append(part)
    return Compound(
        label="v3_integrated_return_cylinder_section_exploded",
        children=children,
    )


def gen_step():
    return build_pose(0.0)
