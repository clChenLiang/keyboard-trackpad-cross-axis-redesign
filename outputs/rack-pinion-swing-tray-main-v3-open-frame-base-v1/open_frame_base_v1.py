"""FDM open-frame base derivative of the integrated-return V3 assembly."""

from importlib.util import module_from_spec, spec_from_file_location
from math import cos, radians, sin
from pathlib import Path

from build123d import Align, Box, Color, Compound, Cylinder, Pos


ROOT = Path(__file__).resolve().parent.parent
SOURCE_PATH = (
    ROOT
    / "rack-pinion-swing-tray-main-v3-integrated-return-v1"
    / "integrated_return_v1.py"
)
SOURCE_SPEC = spec_from_file_location("v3_integrated_return_open_base_source", SOURCE_PATH)
integrated = module_from_spec(SOURCE_SPEC)
SOURCE_SPEC.loader.exec_module(integrated)
v3 = integrated.v3
torsion = integrated.torsion

MAX_BOOLEAN_NOISE_VOLUME = 0.05

BASE_WIDTH = 310.0
BASE_DEPTH = 225.0
BASE_CENTER_Y = 57.5
BASE_Z0 = 1.0
BASE_TOP_Z = 7.0
BASE_HEIGHT = BASE_TOP_Z - BASE_Z0
RAIL_WIDTH = 12.0
CROSS_MEMBER_WIDTH = 288.0
CROSS_MEMBER_DEPTH = 12.0
CROSS_MEMBER_Y = (-37.5, 30.0, 100.0)

GUIDE_ISLAND_WIDTH = 18.0
GUIDE_ISLAND_DEPTH = 48.0
GUIDE_ISLAND_Y = (-21.75, 38.25)
RIGHT_SPINE_X0 = 95.0
RIGHT_SPINE_WIDTH = 60.0
RIGHT_SPINE_CENTER_Y = 100.0
RIGHT_SPINE_DEPTH = 64.0

PRIMARY_FOOT_CENTERS = (
    (-143.0, -44.0),
    (143.0, -44.0),
    (-143.0, 159.0),
    (143.0, 159.0),
)
AUXILIARY_PAD_CENTERS = ((0.0, -44.0), (0.0, 159.0))
FOOT_RADIUS = 8.0
FOOT_ISLAND_RADIUS = 11.0
FOOT_POCKET_RADIUS = 8.2
FOOT_POCKET_Z0 = 0.5
FOOT_POCKET_HEIGHT = 2.5
PRIMARY_TABLE_Z = -15.0
AUXILIARY_TABLE_Z = -14.7
FOOT_TOP_Z = 3.0

FLANGE_RECESS_RADIUS = 19.2
FLANGE_RECESS_Z0 = 1.0
FLANGE_RECESS_DEPTH = 2.0

BASE_GRAPHITE = Color(0.16, 0.18, 0.21)
PRIMARY_FOOT_BLACK = Color(0.05, 0.06, 0.07)
AUXILIARY_FOOT_GRAY = Color(0.16, 0.18, 0.20)


def _label(shape, name, color):
    shape.label = name
    shape.color = color
    return shape


def _box(width, depth, z0=BASE_Z0, height=BASE_HEIGHT, x=0.0, y=BASE_CENTER_Y):
    return Box(
        width,
        depth,
        height,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(x, y, z0))


def _z_cylinder(radius, height, x, y, z0):
    return Cylinder(
        radius,
        height,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(x, y, z0))


def _cartridge_mount_centers():
    centers = []
    for angle in (90.0, 210.0, 330.0):
        a = radians(angle)
        centers.append(
            (
                torsion.SHAFT_X + torsion.MOUNT_BCD_RADIUS * cos(a),
                torsion.SHAFT_Y + torsion.MOUNT_BCD_RADIUS * sin(a),
            )
        )
    return tuple(centers)


def foot_pocket_tools():
    return tuple(
        _z_cylinder(
            FOOT_POCKET_RADIUS,
            FOOT_POCKET_HEIGHT,
            x,
            y,
            FOOT_POCKET_Z0,
        )
        for x, y in (*PRIMARY_FOOT_CENTERS, *AUXILIARY_PAD_CENTERS)
    )


def make_open_frame_base():
    left_rail = _box(RAIL_WIDTH, BASE_DEPTH, x=-BASE_WIDTH / 2.0 + RAIL_WIDTH / 2.0)
    right_rail = _box(RAIL_WIDTH, BASE_DEPTH, x=BASE_WIDTH / 2.0 - RAIL_WIDTH / 2.0)
    front_rail = _box(BASE_WIDTH, RAIL_WIDTH, y=-55.0 + RAIL_WIDTH / 2.0)
    rear_rail = _box(BASE_WIDTH, RAIL_WIDTH, y=170.0 - RAIL_WIDTH / 2.0)
    frame = left_rail + right_rail + front_rail + rear_rail

    for y in CROSS_MEMBER_Y:
        frame = frame + _box(CROSS_MEMBER_WIDTH, CROSS_MEMBER_DEPTH, y=y)

    for x in (-146.0, 146.0):
        for y in GUIDE_ISLAND_Y:
            frame = frame + _box(
                GUIDE_ISLAND_WIDTH,
                GUIDE_ISLAND_DEPTH,
                x=x,
                y=y,
            )

    frame = frame + _box(
        RIGHT_SPINE_WIDTH,
        RIGHT_SPINE_DEPTH,
        x=RIGHT_SPINE_X0 + RIGHT_SPINE_WIDTH / 2.0,
        y=RIGHT_SPINE_CENTER_Y,
    )
    frame = frame + _z_cylinder(
        23.0,
        BASE_HEIGHT,
        torsion.SHAFT_X,
        torsion.SHAFT_Y,
        BASE_Z0,
    )

    for x, y in (*PRIMARY_FOOT_CENTERS, *AUXILIARY_PAD_CENTERS):
        frame = frame + _z_cylinder(
            FOOT_ISLAND_RADIUS,
            BASE_HEIGHT,
            x,
            y,
            BASE_Z0,
        )

    frame = frame - _z_cylinder(
        FLANGE_RECESS_RADIUS,
        FLANGE_RECESS_DEPTH,
        torsion.SHAFT_X,
        torsion.SHAFT_Y,
        FLANGE_RECESS_Z0,
    )
    frame = frame - _z_cylinder(
        5.0,
        BASE_HEIGHT + 2.0,
        torsion.SHAFT_X,
        torsion.SHAFT_Y,
        BASE_Z0 - 1.0,
    )
    for x, y in _cartridge_mount_centers():
        frame = frame - _z_cylinder(
            torsion.M3_CLEARANCE / 2.0,
            BASE_HEIGHT + 2.0,
            x,
            y,
            BASE_Z0 - 1.0,
        )
    for position in v3.POSITIONS:
        frame = frame - v3.guide_base_mount_hole_tools(position)
    for pocket in foot_pocket_tools():
        frame = frame - pocket
    return _label(frame, "open_frame_base_310x225", BASE_GRAPHITE)


def _feet(centers, table_z, label, color):
    feet = [
        _z_cylinder(FOOT_RADIUS, FOOT_TOP_Z - table_z, x, y, table_z)
        for x, y in centers
    ]
    return _label(Compound(children=feet), label, color)


def make_primary_feet():
    return _feet(
        PRIMARY_FOOT_CENTERS,
        PRIMARY_TABLE_Z,
        "primary_tpu_feet_4x",
        PRIMARY_FOOT_BLACK,
    )


def make_auxiliary_pads():
    return _feet(
        AUXILIARY_PAD_CENTERS,
        AUXILIARY_TABLE_Z,
        "auxiliary_tpu_pads_2x",
        AUXILIARY_FOOT_GRAY,
    )


def support_probe_points():
    points = []
    for position in v3.POSITIONS:
        land_x_offset = 4.0 if position.startswith("left") else -4.0
        for index, (x, y) in enumerate(v3._guide_mount_centers(position), 1):
            points.append(
                (f"guide_land_{position}_{index}", x + land_x_offset, y)
            )
    points.extend(
        (
            ("cylinder_front_left", 102.0, 78.0),
            ("cylinder_rear_left", 102.0, 121.0),
            ("retention_root", 149.0, 100.0),
            ("right_spine_rear", 145.0, 126.0),
        )
    )
    for index, angle in enumerate((90.0, 210.0, 330.0), 1):
        a = radians(angle)
        points.append(
            (
                f"cartridge_land_{index}",
                torsion.SHAFT_X + 17.0 * cos(a),
                torsion.SHAFT_Y + 17.0 * sin(a),
            )
        )
    return tuple(points)


def build_pose(t):
    source = integrated.build_pose(t)
    children = []
    for child in source.children:
        if child.label == "continuous_main_base_integrated_return_drilled":
            children.append(make_open_frame_base())
        elif child.label == "integrated_return_clearance_feet":
            children.extend((make_primary_feet(), make_auxiliary_pads()))
        else:
            children.append(child)
    return Compound(label=f"v3_open_frame_base_{t}", children=children)


def build_base_and_return_interface():
    labels = {
        "open_frame_base_310x225",
        "primary_tpu_feet_4x",
        "auxiliary_tpu_pads_2x",
        "fixed_protective_cylinder",
        "torsion_fixed_mounting_cup",
        "torsion_removable_bottom_cover",
        "torsion_helical_spring",
        "torsion_rotating_shaft_clamp",
        "torsion_fixed_end_anchor_pin",
        "torsion_shaft_drive_key",
        "vertical_drive_shaft_integrated_return",
        "rack_retention_fixed_bracket",
    }
    labels.update(f"fixed_guide_mount_foot_{position}" for position in v3.POSITIONS)
    children = [child for child in build_pose(0.0).children if child.label in labels]
    return Compound(label="base_and_return_interface", children=children)


def gen_step():
    return build_pose(0.0)
