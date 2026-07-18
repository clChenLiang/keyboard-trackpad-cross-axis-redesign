"""H2D-sized print plate compounds and print-orientation transforms."""

from build123d import Axis, Compound, Pos

import parts


SIDE_PRINT_NAMES = {
    "side_frame_left",
    "side_frame_right",
    "keyboard_carriage_left",
    "keyboard_carriage_right",
    "trackpad_carriage_left",
    "trackpad_carriage_right",
}

TOP_FACE_DOWN_NAMES = {"keyboard_tray", "trackpad_tray"}


def _floor(shape):
    bbox = shape.bounding_box()
    return shape.moved(Pos(0.0, 0.0, -bbox.min.Z))


def print_oriented(name, shape):
    if name.startswith(("cross_eccentric_bushing", "guide_eccentric_bushing")):
        # Put the wide adjustment flange on the bed.
        oriented = shape.rotate(Axis.Y, -90.0)
    elif name in SIDE_PRINT_NAMES or name.startswith("right_float_washer"):
        oriented = shape.rotate(Axis.Y, 90.0)
    elif name in TOP_FACE_DOWN_NAMES:
        # Put the continuous input surface on the build plate so the
        # underside rib grid grows upward without broad bridges.
        oriented = shape.rotate(Axis.X, 180.0)
    else:
        oriented = shape
    return _floor(oriented)


def _place(name, shape, x, y):
    oriented = print_oriented(name, shape)
    bbox = oriented.bounding_box()
    center_x = (bbox.min.X + bbox.max.X) / 2.0
    center_y = (bbox.min.Y + bbox.max.Y) / 2.0
    placed = oriented.moved(Pos(x - center_x, y - center_y, 0.0))
    placed.label = name
    return placed


def base_plate():
    return Compound(label="v10_base_plate", children=[
        _place("base", parts.make_open_base(), 0.0, 0.0),
    ])


def trays_plate():
    return Compound(label="v10_trays_plate", children=[
        _place("keyboard_tray", parts.make_tray("keyboard"), 0.0, -72.0),
        _place("trackpad_tray", parts.make_tray("trackpad"), 0.0, 65.0),
    ])


def frames_plate():
    return Compound(label="v10_frames_plate", children=[
        _place("side_frame_left", parts.make_side_frame("left"), -55.0, 0.0),
        _place("side_frame_right", parts.make_side_frame("right"), 55.0, 0.0),
    ])


def carriages_plate():
    return Compound(label="v10_carriages_plate", children=[
        _place("trackpad_carriage_left", parts.make_carriage("trackpad", "left"), -88.0, 0.0),
        _place("trackpad_carriage_right", parts.make_carriage("trackpad", "right"), -24.0, 0.0),
        _place("keyboard_carriage_left", parts.make_carriage("keyboard", "left"), 48.0, -58.0),
        _place("keyboard_carriage_right", parts.make_carriage("keyboard", "right"), 48.0, 58.0),
    ])


def petg_small_parts_plate():
    children = [_place("calibration_coupon", parts.calibration_coupon(), 0.0, -45.0)]
    for index in range(2):
        children.append(_place(
            f"cross_eccentric_bushing_{index + 1}",
            parts.make_eccentric_bushing("cross"),
            -70.0 + index * 20.0,
            5.0,
        ))
    for index in range(4):
        children.append(_place(
            f"guide_eccentric_bushing_{index + 1}",
            parts.make_eccentric_bushing("guide"),
            -20.0 + index * 20.0,
            5.0,
        ))
    return Compound(label="v10_petg_small_parts_plate", children=children)


def tpu_parts_plate():
    children = []
    for index in range(4):
        children.append(_place(
            f"tpu_stop_collar_{index + 1}",
            parts.make_tpu_stop_collar(),
            -20.0 + index * 20.0,
            -70.0,
        ))
        children.append(_place(
            f"right_float_washer_{index + 1}",
            parts.make_right_float_washer(),
            -20.0 + index * 20.0,
            -50.0,
        ))
        children.append(_place(
            f"tpu_base_foot_{index + 1}",
            parts.make_tpu_base_foot(),
            -55.0 + index * 37.0,
            -15.0,
        ))
        children.append(_place(
            f"keyboard_tpu_pad_{index + 1}",
            parts.make_tpu_device_pad("keyboard"),
            -55.0 + index * 37.0,
            20.0,
        ))
        children.append(_place(
            f"trackpad_tpu_pad_{index + 1}",
            parts.make_tpu_device_pad("trackpad"),
            -55.0 + index * 37.0,
            45.0,
        ))
    return Compound(label="v10_tpu_parts_plate", children=children)


def small_parts_plate():
    """Legacy combined review plate; manufacturing uses two material plates."""
    return Compound(
        label="v10_small_parts_plate_review_only",
        children=list(petg_small_parts_plate().children) + list(tpu_parts_plate().children),
    )


def plate_facts():
    result = {}
    for name, factory in (
        ("base_plate", base_plate),
        ("trays_plate", trays_plate),
        ("frames_plate", frames_plate),
        ("carriages_plate", carriages_plate),
        ("petg_small_parts_plate", petg_small_parts_plate),
        ("tpu_parts_plate", tpu_parts_plate),
    ):
        bbox = factory().bounding_box().size
        result[name] = (bbox.X, bbox.Y, bbox.Z)
    return result
