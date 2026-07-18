"""Printable V10 part factories.

Every public factory returns one closed BREP solid in its part-local frame.
Purchased bearings, shafts, screws and springs are created in ``assembly.py``.
"""

from math import atan2, degrees, hypot
from typing import Dict, Tuple

from build123d import Align, Axis, Box, Cylinder, FontStyle, Plane, Pos, Text, extrude

import mechanism as mech
import parameters as p


Vec2 = Tuple[float, float]


def _x_cylinder(radius: float, length: float, x: float, y: float, z: float):
    return (
        Cylinder(radius, length, align=(Align.CENTER, Align.CENTER, Align.CENTER))
        .rotate(Axis.Y, 90.0)
        .moved(Pos(x, y, z))
    )


def _beam_between_yz(a: Vec2, b: Vec2, width: float, thickness_x: float, x: float = 0.0):
    dy = b[0] - a[0]
    dz = b[1] - a[1]
    length = hypot(dy, dz)
    if length <= 0.0:
        raise ValueError("beam endpoints must differ")
    beam = Box(
        thickness_x,
        length,
        width,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    return beam.rotate(Axis.X, degrees(atan2(dz, dy))).moved(
        Pos(x, (a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
    )


def _cylinder_between_yz(radius: float, a: Vec2, b: Vec2, x: float = 0.0):
    dy = b[0] - a[0]
    dz = b[1] - a[1]
    length = hypot(dy, dz)
    angle = degrees(atan2(-dy, dz))
    return (
        Cylinder(radius, length, align=(Align.CENTER, Align.CENTER, Align.CENTER))
        .rotate(Axis.X, angle)
        .moved(Pos(x, (a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0))
    )


def _ring_axis_yz(od: float, bore: float, length: float, center: Vec2, u: Vec2, x: float = 0.0):
    a = (center[0] - length * u[0] / 2.0, center[1] - length * u[1] / 2.0)
    b = (center[0] + length * u[0] / 2.0, center[1] + length * u[1] / 2.0)
    inner_a = (center[0] - (length + 2.0) * u[0] / 2.0, center[1] - (length + 2.0) * u[1] / 2.0)
    inner_b = (center[0] + (length + 2.0) * u[0] / 2.0, center[1] + (length + 2.0) * u[1] / 2.0)
    return (
        _cylinder_between_yz(od / 2.0, a, b, x)
        - _cylinder_between_yz(bore / 2.0, inner_a, inner_b, x)
    )


def _x_bridge(x1: float, x2: float, y: float, z: float, size_y: float, size_z: float):
    return Box(
        abs(x2 - x1), size_y, size_z,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Pos((x1 + x2) / 2.0, y, z))


def _capsule_yz(
    length: float,
    width: float,
    thickness_x: float,
    center_y: float,
    center_z: float,
    angle_deg: float,
    center_x: float = 0.0,
):
    if length < width:
        raise ValueError("capsule length must be at least its width")
    core = length - width
    angle_rad = angle_deg * 3.141592653589793 / 180.0
    uy = __import__("math").cos(angle_rad)
    uz = __import__("math").sin(angle_rad)
    a = (center_y - core * uy / 2.0, center_z - core * uz / 2.0)
    b = (center_y + core * uy / 2.0, center_z + core * uz / 2.0)
    shape = _beam_between_yz(a, b, width, thickness_x, center_x)
    shape = shape + _x_cylinder(width / 2.0, thickness_x, center_x, a[0], a[1])
    shape = shape + _x_cylinder(width / 2.0, thickness_x, center_x, b[0], b[1])
    return shape


def _slotted_plate(
    body_length: float,
    body_width: float,
    slot_length: float,
    slot_width: float,
    thickness_x: float,
    center_y: float,
    center_z: float,
    angle_deg: float,
    center_x: float = 0.0,
):
    body = _capsule_yz(
        body_length, body_width, thickness_x,
        center_y, center_z, angle_deg, center_x,
    )
    cutter = _capsule_yz(
        slot_length, slot_width, thickness_x + 2.0,
        center_y, center_z, angle_deg, center_x,
    )
    return body - cutter


def _guide_geometry(kind: str):
    if kind == "keyboard":
        start = (p.KEYBOARD_Y0 + 48.0, p.KEYBOARD_Z0 + 18.0)
        travel = (p.KEYBOARD_DY, p.KEYBOARD_DZ)
        spacing = p.KEYBOARD_GUIDE_ROLLER_SPACING
    elif kind == "trackpad":
        start = (p.TRACKPAD_Y0 - 25.0, p.TRACKPAD_Z0 + 12.0)
        travel = (p.TRACKPAD_DY, p.TRACKPAD_DZ)
        spacing = p.TRACKPAD_GUIDE_ROLLER_SPACING
    else:
        raise ValueError(kind)
    path = hypot(*travel)
    u = (travel[0] / path, travel[1] / path)
    n = (-u[1], u[0])
    center = (start[0] + travel[0] / 2.0, start[1] + travel[1] / 2.0)
    slot_length = path + spacing + p.BEARING_604ZZ.od + 10.0
    body_length = slot_length + 12.0
    angle = degrees(atan2(travel[1], travel[0]))
    return start, travel, u, n, center, slot_length, body_length, angle


def spring_seats():
    start, _, u, n, _, _, _, _ = _guide_geometry("keyboard")
    moving = (
        start[0] + 24.0 * u[0] + 25.0 * n[0],
        start[1] + 24.0 * u[1] + 25.0 * n[1],
    )
    fixed = (
        moving[0] + (p.SPRING_INSTALLED_LENGTH + 12.0) * u[0],
        moving[1] + (p.SPRING_INSTALLED_LENGTH + 12.0) * u[1],
    )
    return moving, fixed


def make_open_base():
    outer = Box(
        p.BASE_WIDTH, p.BASE_DEPTH, p.BASE_THICKNESS,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(0.0, p.BASE_CENTER_Y, 0.0))
    inner = Box(
        p.BASE_WIDTH - 2.0 * p.BASE_RAIL_WIDTH,
        p.BASE_DEPTH - 2.0 * p.BASE_RAIL_WIDTH,
        p.BASE_THICKNESS + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(0.0, p.BASE_CENTER_Y, -1.0))
    base = outer - inner
    rear_crossbar = Box(
        p.BASE_WIDTH - 2.0 * p.BASE_RAIL_WIDTH,
        16.0,
        p.BASE_THICKNESS,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(0.0, p.BASE_CENTER_Y + 88.0, 0.0))
    # Keep the front brace ahead of the descending keyboard rib envelope.
    # At the previous Y=-73 position the rib crossed it at 75-80% travel.
    front_crossbar = rear_crossbar.moved(Pos(0.0, -198.0, 0.0))
    base = base + rear_crossbar + front_crossbar
    for x in (-145.0, 145.0):
        for y in (-70.0, 116.0):
            cutter = Cylinder(
                p.M4_CLEARANCE_DIAMETER / 2.0,
                p.BASE_THICKNESS + 2.0,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            ).moved(Pos(x, y, -1.0))
            base = base - cutter
            counterbore = Cylinder(
                4.25,
                2.6,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            ).moved(Pos(x, y, 0.0))
            base = base - counterbore
    for x in (-137.0, 137.0):
        for y in (-101.0, 131.0):
            foot_pocket = Box(
                24.3, 18.3, 1.0,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            ).moved(Pos(x, y, 0.0))
            base = base - foot_pocket
    return base


def make_side_frame(side: str):
    if side not in ("left", "right"):
        raise ValueError(side)
    _, _, _, _, kc, kslot, kbody, kangle = _guide_geometry("keyboard")
    _, _, _, _, tc, tslot, tbody, tangle = _guide_geometry("trackpad")
    # The right moving carriages carry 0.6 mm axial float.  Shift only the
    # right rail layer by the same amount so both sides retain identical
    # shoulder lengths and bearing stacks.
    keyboard_x = 16.6 if side == "right" else 16.0
    trackpad_x = 17.1 if side == "right" else 16.5
    keyboard_rail = _slotted_plate(
        kbody, p.GUIDE_PLATE_WIDTH, kslot, p.GUIDE_SLOT_WIDTH,
        p.GUIDE_PLATE_THICKNESS, kc[0], kc[1], kangle, keyboard_x,
    )
    trackpad_rail = _slotted_plate(
        tbody, p.GUIDE_PLATE_WIDTH, tslot, p.GUIDE_SLOT_WIDTH,
        p.GUIDE_PLATE_THICKNESS, tc[0], tc[1], tangle, trackpad_x,
    )
    front_foot_y = -70.0
    rear_foot_y = 116.0
    outboard_x = 22.0
    front_foot = Box(
        21.0, 18.0, 8.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(3.5, front_foot_y, p.BASE_THICKNESS))
    rear_foot = Box(
        21.0, 18.0, 8.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(3.5, rear_foot_y, p.BASE_THICKNESS))

    def rail_tips(center, body_length, body_width, u):
        q = body_length / 2.0 - body_width / 2.0 + 1.0
        return (
            (center[0] - q * u[0], center[1] - q * u[1]),
            (center[0] + q * u[0], center[1] + q * u[1]),
        )

    ku = _guide_geometry("keyboard")[2]
    tu = _guide_geometry("trackpad")[2]
    ktips = rail_tips(kc, kbody, p.GUIDE_PLATE_WIDTH, ku)
    ttips = rail_tips(tc, tbody, p.GUIDE_PLATE_WIDTH, tu)
    support = front_foot + rear_foot
    support = support + _x_bridge(3.5, outboard_x, front_foot_y, 10.0, 18.0, 8.0)
    support = support + _x_bridge(3.5, outboard_x, rear_foot_y, 10.0, 18.0, 8.0)
    support = support + _beam_between_yz(
        (front_foot_y, 10.0), ktips[0], 9.0, 4.0, outboard_x
    )
    support = support + _beam_between_yz(
        (rear_foot_y, 10.0), ktips[1], 9.0, 4.0, outboard_x
    )
    support = support + _beam_between_yz(
        (front_foot_y, 10.0), ttips[1], 9.0, 4.0, outboard_x
    )
    support = support + _beam_between_yz(
        (rear_foot_y, 10.0), ttips[0], 9.0, 4.0, outboard_x
    )
    for rail_x, tip in (
        (keyboard_x, ktips[0]), (keyboard_x, ktips[1]),
        (trackpad_x, ttips[0]), (trackpad_x, ttips[1]),
    ):
        support = support + _x_bridge(rail_x, outboard_x, tip[0], tip[1], 9.0, 9.0)
    moving_seat, fixed_seat = spring_seats()
    spring_x = p.RIGHT_SIDE_AXIAL_FLOAT if side == "right" else 0.0
    spring_anchor = _capsule_yz(
        16.0, 12.0, 14.0, fixed_seat[0], fixed_seat[1],
        degrees(atan2(mech.KEYBOARD_U[1], mech.KEYBOARD_U[0])) + 90.0,
        spring_x,
    )
    spring_bridge = _x_bridge(spring_x, outboard_x, fixed_seat[0], fixed_seat[1], 10.0, 10.0)
    spring_brace = _beam_between_yz(
        (rear_foot_y, 10.0), fixed_seat, 9.0, 4.0, outboard_x
    )
    frame = keyboard_rail + trackpad_rail + support + spring_anchor + spring_bridge + spring_brace
    # Re-cut both complete guide corridors after all support unions so no
    # gusset or crossing rail can silently obstruct a roller path.
    # Cut through the complete outboard hardware zone, not only the nominal
    # rail plate.  The shoulder tip, washer and M3 retaining nut travel with the roller.
    hardware_corridor_x = 14.0
    keyboard_slot_cutter = _capsule_yz(
        kslot, p.GUIDE_SLOT_WIDTH, hardware_corridor_x,
        kc[0], kc[1], kangle, keyboard_x,
    )
    trackpad_slot_cutter = _capsule_yz(
        tslot, p.GUIDE_SLOT_WIDTH, hardware_corridor_x,
        tc[0], tc[1], tangle, trackpad_x,
    )
    frame = frame - keyboard_slot_cutter - trackpad_slot_cutter
    # Real blind socket for the fixed end of the 3 mm spring guide rod.
    rod_hole_a = (
        fixed_seat[0] - 8.0 * mech.KEYBOARD_U[0],
        fixed_seat[1] - 8.0 * mech.KEYBOARD_U[1],
    )
    rod_hole_b = (
        fixed_seat[0] + 3.0 * mech.KEYBOARD_U[0],
        fixed_seat[1] + 3.0 * mech.KEYBOARD_U[1],
    )
    frame = frame - _cylinder_between_yz(
        (p.SPRING_GUIDE_DIAMETER + 0.15) / 2.0,
        rod_hole_a,
        rod_hole_b,
        spring_x,
    )
    for y in (front_foot_y, rear_foot_y):
        insert_pocket = Cylinder(
            p.M4_HEAT_INSERT_OD / 2.0,
            p.M4_HEAT_INSERT_LENGTH,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Pos(-3.0, y, p.BASE_THICKNESS))
        frame = frame - insert_pocket
    return frame.mirror(Plane.YZ) if side == "left" else frame


def make_tray(kind: str):
    if kind not in ("keyboard", "trackpad"):
        raise ValueError(kind)
    skin = Box(
        p.TRAY_WIDTH, p.TRAY_DEPTH, p.TRAY_SKIN,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    rear_beam = Box(
        p.TRAY_WIDTH - 8.0, p.TRAY_RIB_THICKNESS, p.TRAY_RIB_DEPTH,
        align=(Align.CENTER, Align.CENTER, Align.MAX),
    ).moved(Pos(0.0, p.TRAY_DEPTH / 2.0 - 5.0, 0.0))
    front_beam = rear_beam.moved(Pos(0.0, -p.TRAY_DEPTH + 10.0, 0.0))
    side_beam = Box(
        p.TRAY_RIB_THICKNESS, p.TRAY_DEPTH - 8.0, p.TRAY_RIB_DEPTH,
        align=(Align.CENTER, Align.CENTER, Align.MAX),
    ).moved(Pos(p.TRAY_WIDTH / 2.0 - 5.0, 0.0, 0.0))
    side_beam_2 = side_beam.moved(Pos(-p.TRAY_WIDTH + 10.0, 0.0, 0.0))
    long_rib_1 = Box(
        p.TRAY_RIB_THICKNESS, p.TRAY_DEPTH - 12.0, p.TRAY_RIB_DEPTH,
        align=(Align.CENTER, Align.CENTER, Align.MAX),
    ).moved(Pos(-45.0, 0.0, 0.0))
    long_rib_2 = long_rib_1.moved(Pos(90.0, 0.0, 0.0))
    cross_rib_1 = Box(
        p.TRAY_WIDTH - 12.0, p.TRAY_RIB_THICKNESS, p.TRAY_RIB_DEPTH,
        align=(Align.CENTER, Align.CENTER, Align.MAX),
    ).moved(Pos(0.0, -30.0, 0.0))
    cross_rib_2 = cross_rib_1.moved(Pos(0.0, 60.0, 0.0))
    ear_y = 0.0 if kind == "keyboard" else -65.0
    ear_hole_z = 4.0 if kind == "keyboard" else 5.0
    left_ear = Box(
        6.0, 34.0, 8.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(-142.5, ear_y, 0.0))
    right_ear = left_ear.mirror(Plane.YZ)
    tray = (
        skin + rear_beam + front_beam + side_beam + side_beam_2
        + long_rib_1 + long_rib_2 + cross_rib_1 + cross_rib_2
        + left_ear + right_ear
    )
    # Rear-center charging access passes through the low lip/skin edge only.
    cable = Box(
        16.0, 8.0, p.TRAY_SKIN + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(0.0, p.TRAY_DEPTH / 2.0 - 1.0, -0.5))
    tray = tray - cable
    for sign in (-1.0, 1.0):
        for y in (ear_y - 10.0, ear_y + 10.0):
            tray = tray - _x_cylinder(
                p.M4_HEAT_INSERT_OD / 2.0,
                p.M4_HEAT_INSERT_LENGTH,
                sign * (145.5 - p.M4_HEAT_INSERT_LENGTH / 2.0),
                y,
                ear_hole_z,
            )

    if kind == "keyboard":
        pad_positions = [(-100.0, -45.0), (-100.0, 45.0), (100.0, -45.0), (100.0, 45.0)]
        pad_length = 32.0
        text_value = "by cl"
    else:
        pad_positions = [(20.0, -45.0), (20.0, 45.0), (100.0, -45.0), (100.0, 45.0)]
        pad_length = 26.0
        text_value = "created"
    for x, y in pad_positions:
        pocket = Box(
            pad_length + 0.3, 8.3, 0.7,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Pos(x, y, p.TRAY_SKIN - 0.6))
        tray = tray - pocket

    engraving = extrude(
        Text(
            text_value,
            8.0,
            font_style=FontStyle.BOLD,
            align=(Align.CENTER, Align.CENTER),
        ),
        0.25,
    )
    tray = tray - engraving
    return tray


def _bearing_centers(kind: str):
    start, _, u, n, _, _, _, _ = _guide_geometry(kind)
    spacing = (
        p.KEYBOARD_GUIDE_ROLLER_SPACING
        if kind == "keyboard" else p.TRACKPAD_GUIDE_ROLLER_SPACING
    )
    wall_offset = (p.GUIDE_SLOT_WIDTH - p.BEARING_604ZZ.od) / 2.0
    return (
        (
            start[0] - spacing * u[0] / 2.0 + wall_offset * n[0],
            start[1] - spacing * u[1] / 2.0 + wall_offset * n[1],
        ),
        (
            start[0] + spacing * u[0] / 2.0 - wall_offset * n[0],
            start[1] + spacing * u[1] / 2.0 - wall_offset * n[1],
        ),
    )


def make_carriage(kind: str, side: str):
    if side not in ("left", "right") or kind not in ("keyboard", "trackpad"):
        raise ValueError((kind, side))
    thickness = 5.0
    guide_centers = _bearing_centers(kind)
    if kind == "keyboard":
        tray_node = (p.KEYBOARD_Y0, p.KEYBOARD_Z0 + 4.0)
        yoke = mech.pose(0.0).cross_yoke_center
        body = _beam_between_yz(tray_node, guide_centers[0], 9.0, thickness)
        body = body + _beam_between_yz(tray_node, guide_centers[1], 9.0, thickness)
        body = body + _beam_between_yz(tray_node, yoke, 12.0, thickness)
        body = body + _capsule_yz(30.0, 14.0, thickness, yoke[0], yoke[1], degrees(atan2(mech.SLOT_U[1], mech.SLOT_U[0])))
        cross_centers = mech.pose(0.0).cross_roller_centers
        guide_n = _guide_geometry("keyboard")[3]
        holes = [
            (guide_centers[0], False, guide_n),
            (guide_centers[1], True, guide_n),
            (cross_centers[0], False, mech.SLOT_N),
            (cross_centers[1], True, mech.SLOT_N),
        ]
    else:
        tray_node = (p.TRACKPAD_Y0 - 65.0, p.TRACKPAD_Z0 + 5.0)
        slot_center = mech.pose(0.0).cross_slot_center
        slot_angle = degrees(atan2(mech.SLOT_U[1], mech.SLOT_U[0]))
        slot_x = 0.0
        slot_plate = _slotted_plate(
            p.CROSS_SLOT_LENGTH + 12.0,
            p.CROSS_SLOT_PLATE_WIDTH,
            p.CROSS_SLOT_LENGTH,
            p.CROSS_SLOT_WIDTH,
            p.CROSS_SLOT_PLATE_THICKNESS,
            slot_center[0], slot_center[1], slot_angle, slot_x,
        )
        body = _beam_between_yz(tray_node, guide_centers[0], 10.0, thickness)
        body = body + _beam_between_yz(tray_node, guide_centers[0], 13.0, thickness)
        body = body + _beam_between_yz(tray_node, guide_centers[1], 13.0, thickness)
        bridge_nodes = []
        for q in (-94.0, 94.0):
            node = (
                slot_center[0] + q * mech.SLOT_U[0],
                slot_center[1] + q * mech.SLOT_U[1],
            )
            bridge_nodes.append(node)
        body = body + slot_plate
        body = body + _beam_between_yz(tray_node, bridge_nodes[0], 10.0, thickness)
        body = body + _beam_between_yz(guide_centers[0], bridge_nodes[1], 10.0, thickness)
        guide_n = _guide_geometry("trackpad")[3]
        holes = [
            (guide_centers[0], False, guide_n),
            (guide_centers[1], True, guide_n),
        ]

    tray_pad = Box(
        thickness, 34.0, 9.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Pos(0.0, tray_node[0], tray_node[1]))
    body = body + tray_pad

    if kind == "trackpad":
        inward = 1.0 if side == "left" else -1.0
        lateral_mount = Box(
            11.5, 34.0, 9.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).moved(Pos(inward * 3.25, tray_node[0], tray_node[1]))
        body = body + lateral_mount

    if kind == "keyboard":
        moving_seat, _ = spring_seats()
        spring_seat = _capsule_yz(
            16.0, 12.0, thickness, moving_seat[0], moving_seat[1],
            degrees(atan2(mech.KEYBOARD_U[1], mech.KEYBOARD_U[0])) + 90.0,
        )
        spring_bridge_end = (
            moving_seat[0] + 4.0 * mech.KEYBOARD_U[0],
            moving_seat[1] + 4.0 * mech.KEYBOARD_U[1],
        )
        body = body + spring_seat + _beam_between_yz(
            guide_centers[1], spring_bridge_end, 12.0, thickness
        )
        body = body + _beam_between_yz(yoke, moving_seat, 10.0, thickness)
        # The fixed guide rod slides deeper through the moving carriage over
        # the 33.54 mm stroke, so the rod tunnel spans the full relative path.
        # Its diameter clears the Ø3 rod but is smaller than the TPU collar.
        hole_a = (
            moving_seat[0] - (mech.KEYBOARD_PATH + 6.0) * mech.KEYBOARD_U[0],
            moving_seat[1] - (mech.KEYBOARD_PATH + 6.0) * mech.KEYBOARD_U[1],
        )
        hole_b = (
            moving_seat[0] + 10.0 * mech.KEYBOARD_U[0],
            moving_seat[1] + 10.0 * mech.KEYBOARD_U[1],
        )
        body = body - _cylinder_between_yz(
            p.SPRING_GUIDE_DIAMETER / 2.0 + 0.25,
            hole_a,
            hole_b,
            0.0,
        )

    for center, is_eccentric, normal in holes:
        y, z = center
        if not is_eccentric:
            cutter_d = p.M4_CLEARANCE_DIAMETER
        else:
            cutter_d = 10.2  # eccentric bushing socket
            y += p.ECCENTRICITY * normal[0]
            z += p.ECCENTRICITY * normal[1]
        body = body - _x_cylinder(cutter_d / 2.0, thickness + 2.0, 0.0, y, z)
        if is_eccentric:
            inner_sign = 1.0 if side == "left" else -1.0
            recess_x = inner_sign * (
                thickness / 2.0 - p.ECCENTRIC_FLANGE_THICKNESS / 2.0
            )
            body = body - _x_cylinder(
                (p.ECCENTRIC_FLANGE_OD + 0.2) / 2.0,
                p.ECCENTRIC_FLANGE_THICKNESS + 0.2,
                recess_x,
                y,
                z,
            )
    for y_offset in (-10.0, 10.0):
        if kind == "trackpad":
            mount_length = 13.5
            mount_center_x = inward * 3.25
        else:
            mount_length = thickness + 2.0
            mount_center_x = 0.0
        body = body - _x_cylinder(
            p.M4_CLEARANCE_DIAMETER / 2.0,
            mount_length,
            mount_center_x,
            tray_node[0] + y_offset,
            tray_node[1],
        )
    if kind == "trackpad":
        slot_center = mech.pose(0.0).cross_slot_center
        slot_angle = degrees(atan2(mech.SLOT_U[1], mech.SLOT_U[0]))
        body = body - _capsule_yz(
            p.CROSS_SLOT_LENGTH,
            p.CROSS_SLOT_WIDTH,
            13.0,
            slot_center[0],
            slot_center[1],
            slot_angle,
            0.0,
        )
    else:
        body = body + _ring_axis_yz(
            10.0,
            3.8,
            2.0,
            spring_seats()[0],
            mech.KEYBOARD_U,
            0.0,
        )
    return body


def make_eccentric_bushing(kind: str):
    if kind not in ("cross", "guide"):
        raise ValueError(kind)
    outer_diameter = 10.0
    width = 4.8
    body = _x_cylinder(outer_diameter / 2.0, width, 0.0, 0.0, 0.0)
    flange_x = -(width + p.ECCENTRIC_FLANGE_THICKNESS) / 2.0
    flange = _x_cylinder(
        p.ECCENTRIC_FLANGE_OD / 2.0,
        p.ECCENTRIC_FLANGE_THICKNESS,
        flange_x,
        0.0,
        0.0,
    )
    # Two flats expose a real 10 mm spanner surface.  The flats rotate with
    # the eccentric and also show the preload direction in the assembly.
    flat_depth = (p.ECCENTRIC_FLANGE_OD - p.ECCENTRIC_FLANGE_AF) / 2.0 + 0.5
    for sign in (-1.0, 1.0):
        flange = flange - Box(
            p.ECCENTRIC_FLANGE_THICKNESS + 1.0,
            p.ECCENTRIC_FLANGE_OD + 2.0,
            flat_depth,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).moved(Pos(
            flange_x,
            0.0,
            sign * (p.ECCENTRIC_FLANGE_AF / 2.0 + flat_depth / 2.0),
        ))
    body = body + flange
    bore = _x_cylinder(
        (p.SHAFT_DIAMETER + 0.20) / 2.0,
        width + p.ECCENTRIC_FLANGE_THICKNESS + 2.0,
        -p.ECCENTRIC_FLANGE_THICKNESS / 2.0,
        p.ECCENTRICITY,
        0.0,
    )
    return body - bore


def make_tpu_stop(kind: str):
    if kind not in ("keyboard", "trackpad"):
        raise ValueError(kind)
    length = 14.0 if kind == "keyboard" else 18.0
    return Box(length, 8.0, 5.0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def make_tpu_stop_collar():
    collar = (
        Cylinder(3.0, 3.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
        - Cylinder(1.4, 4.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    )
    # Radial slit makes a real snap-on C collar; the 2.8 mm bore grips the
    # Ø3 guide rod after positioning instead of sliding freely along it.
    slit = Box(
        4.0,
        0.8,
        4.0,
        align=(Align.MIN, Align.CENTER, Align.CENTER),
    )
    return collar - slit


def make_right_float_washer():
    return (
        _x_cylinder(4.0, 0.6, 0.0, 0.0, 0.0)
        - _x_cylinder(p.M4_CLEARANCE_DIAMETER / 2.0, 1.0, 0.0, 0.0, 0.0)
    )


def make_tpu_device_pad(kind: str):
    if kind not in ("keyboard", "trackpad"):
        raise ValueError(kind)
    length = 32.0 if kind == "keyboard" else 26.0
    return Box(length, 8.0, 1.2, align=(Align.CENTER, Align.CENTER, Align.MIN))


def make_tpu_base_foot():
    return Box(24.0, 18.0, 2.0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def calibration_coupon():
    coupon = Box(92.0, 38.0, 7.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    holes = [
        (-35.0, 0.0, p.SHAFT_DIAMETER + delta)
        for delta in (0.10, 0.20, 0.30)
    ]
    for x, y, diameter in holes:
        cutter = Cylinder(
            diameter / 2.0, 9.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Pos(x, y, -1.0))
        coupon = coupon - cutter
    for x, diameter in ((5.0, 8.2), (20.0, 12.2), (37.0, 10.2)):
        pocket = Cylinder(
            diameter / 2.0, 4.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Pos(x, 0.0, 4.0))
        coupon = coupon - pocket
    return coupon


def printable_parts() -> Dict[str, object]:
    return {
        "base": make_open_base(),
        "side_frame_left": make_side_frame("left"),
        "side_frame_right": make_side_frame("right"),
        "keyboard_tray": make_tray("keyboard"),
        "trackpad_tray": make_tray("trackpad"),
        "keyboard_carriage_left": make_carriage("keyboard", "left"),
        "keyboard_carriage_right": make_carriage("keyboard", "right"),
        "trackpad_carriage_left": make_carriage("trackpad", "left"),
        "trackpad_carriage_right": make_carriage("trackpad", "right"),
        "cross_eccentric_bushing": make_eccentric_bushing("cross"),
        "guide_eccentric_bushing": make_eccentric_bushing("guide"),
        "tpu_stop_collar": make_tpu_stop_collar(),
        "right_float_washer": make_right_float_washer(),
        "keyboard_tpu_pad": make_tpu_device_pad("keyboard"),
        "trackpad_tpu_pad": make_tpu_device_pad("trackpad"),
        "tpu_base_foot": make_tpu_base_foot(),
        "calibration_coupon": calibration_coupon(),
    }
