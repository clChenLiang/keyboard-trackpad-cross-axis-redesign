"""V2 open-base variant of the independent cross-axis slider concept."""

from pathlib import Path
import sys

from build123d import Align, Box, Color, Compound, Cylinder, Pos


V1_DIR = Path(__file__).resolve().parents[1] / "cross-axis-slider-concept-v1"
if str(V1_DIR) not in sys.path:
    sys.path.insert(0, str(V1_DIR))

import cross_axis_slider_concept as v1  # noqa: E402


WINDOW_WIDTH = 112.0
WINDOW_DEPTH = 112.0
WINDOW_CORNER_RADIUS = 10.0
WINDOW_CENTERS_X = (-66.5, 66.5)
WINDOW_CENTERS_Y = (-34.0, 104.0)


def _rounded_rect_cutter(width, depth, radius, height, center_x, center_y, center_z):
    """Rounded rectangular prism aligned with XY and extruded through Z."""
    cutter = Box(
        width - 2.0 * radius,
        depth,
        height,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    cutter = cutter + Box(
        width,
        depth - 2.0 * radius,
        height,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            corner = Cylinder(
                radius,
                height,
                align=(Align.CENTER, Align.CENTER, Align.CENTER),
            ).moved(Pos(
                sx * (width / 2.0 - radius),
                sy * (depth / 2.0 - radius),
                0.0,
            ))
            cutter = cutter + corner
    return cutter.moved(Pos(center_x, center_y, center_z))


def make_open_base():
    base = Box(
        v1.BASE_WIDTH,
        v1.BASE_DEPTH,
        v1.BASE_THICKNESS,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(0.0, v1.BASE_CENTER_Y, 0.0))

    cutter_height = v1.BASE_THICKNESS + 2.0
    cutter_z = v1.BASE_THICKNESS / 2.0
    for x in WINDOW_CENTERS_X:
        for y in WINDOW_CENTERS_Y:
            base = base - _rounded_rect_cutter(
                WINDOW_WIDTH,
                WINDOW_DEPTH,
                WINDOW_CORNER_RADIUS,
                cutter_height,
                x,
                y,
                cutter_z,
            )

    base.label = "base_310x300_four_window_openwork"
    base.color = Color(0.29, 0.31, 0.34)
    return base


def build_pose(travel=0.0, pose_name="keyboard_mode"):
    original = v1.build_pose(travel, pose_name)
    children = list(original.children)
    children[0] = make_open_base()
    assembly = Compound(
        label=f"cross_axis_slider_concept_v2_open_base_{pose_name}",
        children=children,
    )
    return assembly

