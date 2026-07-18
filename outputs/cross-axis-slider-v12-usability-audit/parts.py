"""Printable V12 part factories.

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
        start[0] + 24.0 * u[0] + p.SPRING_NORMAL_OFFSET * n[0],
        start[1] + 24.0 * u[1] + p.SPRING_NORMAL_OFFSET * n[1],
    )
    fixed = (
        moving[0] + (p.SPRING_INSTALLED_LENGTH + 12.0) * u[0],
        moving[1] + (p.SPRING_INSTALLED_LENGTH + 12.0) * u[1],
    )
    return moving, fixed


def return_anchor_center():
    return (
        p.RETURN_SPOOL_CENTER[0]
        - p.RETURN_SPRING_INITIAL_DEFLECTION * mech.TRACKPAD_U[0],
        p.RETURN_SPOOL_CENTER[1]
        - p.RETURN_SPRING_INITIAL_DEFLECTION * mech.TRACKPAD_U[1],
    )


def _oriented_box_yz(
    x_size: float,
    along_size: float,
    normal_size: float,
    center: Vec2,
    center_x: float = 0.0,
):
    angle = degrees(atan2(mech.TRACKPAD_U[1], mech.TRACKPAD_U[0]))
    return (
        Box(
            x_size,
            along_size,
            normal_size,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        )
        .rotate(Axis.X, angle)
        .moved(Pos(center_x, center[0], center[1]))
    )


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
    # Each TPU foot sits on a complete printed island tied into the perimeter
    # rail.  Without these islands most of the 24 x 18 foot would bridge the
    # open center and could peel or rock under diagonal input load.
    for x, y in p.BASE_FOOT_POSITIONS:
        support_pad = Box(
            26.0,
            20.0,
            p.BASE_THICKNESS,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Pos(x, y, 0.0))
        base = base + support_pad
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
    for x, y in p.BASE_FOOT_POSITIONS:
        foot_pocket = Box(
            24.3, 18.3, 1.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Pos(x, y, 0.0))
        base = base - foot_pocket
    # The keyboard carriage reaches Z=5.5 mm at the pressed hard stop.  Two
    # shallow rail-top pockets preserve 1.5 mm clearance while retaining a
    # continuous 4 mm base rail below the moving member.
    for x in (-145.0, 145.0):
        base = base - Box(
            10.0,
            60.0,
            3.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Pos(x, -5.0, 4.0))
    # The descending keyboard side beam runs beside the inner base rails from
    # 60-100% travel.  Relieve 1.5 mm from those rail faces over the swept
    # zone; stop 2 mm before the front crossbar so the base stays connected.
    # The remaining 6.5 mm rail is continuous and the nominal gap rises from
    # 0.65 to 2.15 mm throughout the full tray sweep.
    for sign in (-1.0, 1.0):
        base = base - Box(
            3.0,
            145.0,
            p.BASE_THICKNESS + 1.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Pos(sign * 141.0, -12.5, -0.5))
    return base


def make_side_frame(side: str):
    if side not in ("left", "right"):
        raise ValueError(side)
    _, _, _, _, kc, kslot, kbody, kangle = _guide_geometry("keyboard")
    _, _, _, _, tc, tslot, tbody, tangle = _guide_geometry("trackpad")
    # Left and right X layers are exact mirrors.  The right side's real
    # anti-binding degree of freedom is in the wider Y-Z follower slots.
    keyboard_x = 15.5
    trackpad_x = 19.5
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
    spring_x = -p.SPRING_LATERAL_INSET + (
        p.RIGHT_SIDE_AXIAL_FLOAT if side == "right" else 0.0
    )
    spring_anchor = _capsule_yz(
        20.0, 12.0, 16.0, fixed_seat[0], fixed_seat[1],
        degrees(atan2(mech.KEYBOARD_U[1], mech.KEYBOARD_U[0])) + 90.0,
        spring_x,
    )
    fixed_sleeve_center = (
        fixed_seat[0] - (6.0 + p.SPRING_GUIDE_SLEEVE_LENGTH / 2.0) * mech.KEYBOARD_U[0],
        fixed_seat[1] - (6.0 + p.SPRING_GUIDE_SLEEVE_LENGTH / 2.0) * mech.KEYBOARD_U[1],
    )
    fixed_sleeve = _ring_axis_yz(
        p.SPRING_GUIDE_SLEEVE_OD,
        p.SPRING_GUIDE_SLEEVE_BORE,
        p.SPRING_GUIDE_SLEEVE_LENGTH,
        fixed_sleeve_center,
        mech.KEYBOARD_U,
        spring_x,
    )
    spring_bridge = _x_bridge(spring_x, outboard_x, fixed_seat[0], fixed_seat[1], 10.0, 10.0)
    spring_brace = _beam_between_yz(
        (rear_foot_y, 10.0), fixed_seat, 9.0, 4.0, outboard_x
    )
    return_anchor = return_anchor_center()
    return_n = (-mech.TRACKPAD_U[1], mech.TRACKPAD_U[0])
    return_pad_center = (
        return_anchor[0] + 3.1 * return_n[0],
        return_anchor[1] + 3.1 * return_n[1],
    )
    return_pad_x = p.RETURN_SPOOL_GLOBAL_X - 148.0
    return_pad = _oriented_box_yz(
        10.0, 18.0, 6.0, return_pad_center,
        return_pad_x,
    )
    return_pad_bridge = _x_bridge(
        return_pad_x, outboard_x,
        return_pad_center[0], return_pad_center[1], 12.0, 10.0,
    )
    return_wall_point = (
        return_anchor[0] + 4.0 * return_n[0],
        return_anchor[1] + 4.0 * return_n[1],
    )
    return_wall_bridge = _beam_between_yz(
        return_pad_center, return_wall_point, 10.0, 4.0, outboard_x
    )
    frame = (
        keyboard_rail + trackpad_rail + support + spring_anchor
        + fixed_sleeve + spring_bridge + spring_brace
        + return_pad + return_pad_bridge + return_wall_bridge
    )
    # Re-cut both complete guide corridors after all support unions so no
    # gusset or crossing rail can silently obstruct a roller path.
    # Cut through the complete outboard hardware zone, not only the nominal
    # rail plate.  The shoulder tip, washer and M3 retaining nut travel with the roller.
    # Cut through the complete inner bushing-to-outer-nut stack.  The earlier
    # 14 mm tunnel cleared only the bearing layer and could let a support
    # gusset enter the eccentric bushing when guide angles changed.
    hardware_corridor_x = 32.0
    follower_extra = p.RIGHT_FOLLOWER_SLOT_EXTRA_WIDTH if side == "right" else 0.0
    keyboard_slot_cutter = _capsule_yz(
        kslot, p.GUIDE_SLOT_WIDTH + follower_extra, hardware_corridor_x,
        kc[0], kc[1], kangle, keyboard_x,
    )
    trackpad_slot_cutter = _capsule_yz(
        tslot, p.GUIDE_SLOT_WIDTH + follower_extra, hardware_corridor_x,
        tc[0], tc[1], tangle, trackpad_x,
    )
    frame = frame - keyboard_slot_cutter - trackpad_slot_cutter
    # The sourceable 12 mm cross shoulder screws project farther than the
    # earlier custom-length shafts.  Their two centers travel with the keyboard
    # carriage and briefly cross a fixed diagonal brace near 10-20% travel.
    # Cut a true swept tool/shaft corridor through the fixed frame rather than
    # hiding the interference with a shorter non-standard shoulder.  Ø6.6
    # clears the selected Ø6 low head as well as the Ø4 shoulder.
    cross_corridor_width = 6.6
    # The M3 threaded tip reaches local X=14.1, so include the complete
    # shoulder/thread envelope plus 0.9 mm axial clearance.
    cross_corridor_x = 6.0
    cross_corridor_depth = 18.0
    cross_start = mech.pose(0.0).cross_roller_centers
    cross_end = mech.pose(1.0).cross_roller_centers
    for start_center, end_center in zip(cross_start, cross_end):
        dy = end_center[0] - start_center[0]
        dz = end_center[1] - start_center[1]
        path_length = hypot(dy, dz)
        frame = frame - _capsule_yz(
            path_length + cross_corridor_width,
            cross_corridor_width,
            cross_corridor_depth,
            (start_center[0] + end_center[0]) / 2.0,
            (start_center[1] + end_center[1]) / 2.0,
            degrees(atan2(dz, dy)),
            cross_corridor_x,
        )
    # Real blind socket for the fixed end of the 3 mm spring guide rod.
    rod_hole_a = (
        fixed_seat[0] - 16.0 * mech.KEYBOARD_U[0],
        fixed_seat[1] - 16.0 * mech.KEYBOARD_U[1],
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
            p.M4_HEAT_INSERT_HOLE_DIAMETER / 2.0,
            p.M4_HEAT_INSERT_LENGTH,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Pos(-3.0, y, p.BASE_THICKNESS))
        frame = frame - insert_pocket
    # Swept clearance for the trackpad carriage as it passes the inboard end
    # of the return-clamp bridge between 10% and 25% travel.
    return_bridge_relief = Box(
        6.0,
        17.0,
        8.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Pos(8.5, 5.5, 83.0))
    frame = frame - return_bridge_relief
    # One blind insert pocket receives the screw through the stock CFS0.2
    # accessory plate.  The plate itself is supplied with the spring; no
    # printed clamp or custom drum is required.
    insert_a = (
        return_anchor[0] + 1.0 * return_n[0],
        return_anchor[1] + 1.0 * return_n[1],
    )
    insert_b = (
        return_anchor[0] + 6.5 * return_n[0],
        return_anchor[1] + 6.5 * return_n[1],
    )
    frame = frame - _cylinder_between_yz(
        p.RETURN_CLAMP_INSERT_HOLE_DIAMETER / 2.0,
        insert_a,
        insert_b,
        return_pad_x,
    )
    # The keyboard tray side beam passes the fixed frame at mid-stroke.  The
    # former nominal axial gap was only 0.65 mm at Y=-73.85/Z=8.5; remove a
    # 1 mm inner-face strip locally to provide a printable 1.65 mm clearance
    # without moving the guide or base-mount datums.
    keyboard_tray_midstroke_relief = Box(
        2.0,
        50.0,
        16.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Pos(-7.0, -74.0, 13.0))
    frame = frame - keyboard_tray_midstroke_relief
    # The first cross-roller M3 retention nut sweeps past the outer fixed
    # brace with only 0.51-0.96 mm nominal clearance.  A shallow outer-face
    # relief follows the whole nut path and raises the axial gap above the
    # calibrated-print clearance floor without thinning the brace core.
    cross_nut_sweep_relief = Box(
        2.0,
        58.0,
        32.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Pos(13.0, 7.0, 50.0))
    frame = frame - cross_nut_sweep_relief
    # At the return endpoint the trackpad carriage previously passed only
    # 0.165 mm below a local frame rib.  The TPU rod collar is the designed
    # stop, so this rib must remain a non-contact clearance surface.
    trackpad_return_clearance = Box(
        26.0,
        36.0,
        10.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Pos(0.0, -18.0, 79.5))
    frame = frame - trackpad_return_clearance
    # A fixed frame sits on the base top; no brace, guide boss or legacy rod
    # support may penetrate the base or extend toward the desk below Z=6.
    frame = frame & Box(
        80.0,
        400.0,
        200.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(0.0, p.BASE_CENTER_Y, p.BASE_THICKNESS))
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
    left_ear_root = Box(
        6.0, 34.0, 8.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(-142.5, ear_y, 0.0))
    # Only the root overlaps the tray edge, and it stops at the skin top.
    # The full-height lug starts at X=140.3, leaving 0.45 mm beside the
    # right-aligned 160 mm trackpad while retaining the 5.2 mm insert length.
    left_ear_root = left_ear_root & Box(
        8.0, 36.0, p.TRAY_SKIN,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(-142.5, ear_y, 0.0))
    left_ear_lug = Box(
        p.M4_HEAT_INSERT_LENGTH,
        34.0,
        8.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(-(145.5 - p.M4_HEAT_INSERT_LENGTH / 2.0), ear_y, 0.0))
    left_ear = left_ear_root + left_ear_lug
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
                p.M4_HEAT_INSERT_HOLE_DIAMETER / 2.0,
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
        # V12 moves the trackpad carriage layer 1.5 mm outward to accept a
        # sourceable 12 mm cross shoulder screw.  Extend only the inward tray
        # mounting tongue so its mating face remains at the original tray ear;
        # this avoids a loose spacer and keeps the tray envelope unchanged.
        lateral_mount = Box(
            13.0, 34.0, 9.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).moved(Pos(inward * 4.0, tray_node[0], tray_node[1]))
        body = body + lateral_mount

    if kind == "keyboard":
        moving_seat, _ = spring_seats()
        spring_x = p.SPRING_LATERAL_INSET if side == "left" else -p.SPRING_LATERAL_INSET
        spring_seat = _capsule_yz(
            20.0, 12.0, 12.0, moving_seat[0], moving_seat[1],
            degrees(atan2(mech.KEYBOARD_U[1], mech.KEYBOARD_U[0])) + 90.0,
            spring_x,
        )
        moving_sleeve_center = (
            moving_seat[0] + (6.0 + p.SPRING_GUIDE_SLEEVE_LENGTH / 2.0) * mech.KEYBOARD_U[0],
            moving_seat[1] + (6.0 + p.SPRING_GUIDE_SLEEVE_LENGTH / 2.0) * mech.KEYBOARD_U[1],
        )
        moving_sleeve = _ring_axis_yz(
            p.SPRING_GUIDE_SLEEVE_OD,
            p.SPRING_GUIDE_SLEEVE_BORE,
            p.SPRING_GUIDE_SLEEVE_LENGTH,
            moving_sleeve_center,
            mech.KEYBOARD_U,
            spring_x,
        )
        spring_bridge_end = (
            moving_seat[0] + 4.0 * mech.KEYBOARD_U[0],
            moving_seat[1] + 4.0 * mech.KEYBOARD_U[1],
        )
        body = body + spring_seat + moving_sleeve + _beam_between_yz(
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
            spring_x,
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
            mount_length = 15.0
            mount_center_x = inward * 4.0
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
        cross_slot_width = p.CROSS_SLOT_WIDTH + (
            p.RIGHT_FOLLOWER_SLOT_EXTRA_WIDTH if side == "right" else 0.0
        )
        body = body - _capsule_yz(
            p.CROSS_SLOT_LENGTH,
            cross_slot_width,
            13.0,
            slot_center[0],
            slot_center[1],
            slot_angle,
            0.0,
        )
        # Integrated Ø7.8 post accepts the polypropylene drum supplied with
        # MISUMI CFS0.2.  It lies on the existing long-slot carriage and moves
        # rigidly with the trackpad tray.
        spool_y, spool_z = p.RETURN_SPOOL_CENTER
        sign = -1.0 if side == "left" else 1.0
        carriage_global_x = sign * (
            156.0 + (p.RIGHT_SIDE_AXIAL_FLOAT if side == "right" else 0.0)
        )
        boss_global_x = sign * (
            p.RETURN_SPOOL_GLOBAL_X + p.RETURN_SPOOL_TOTAL_WIDTH / 2.0 + 2.0
        )
        spool_support_x = boss_global_x - carriage_global_x
        spool_boss = _x_cylinder(7.0, 4.0, spool_support_x, spool_y, spool_z)
        spool_bridge = _beam_between_yz(
            tray_node, (spool_y, spool_z), 12.0, 4.0, spool_support_x
        )
        spool_x_bridge = _x_bridge(
            spool_support_x,
            0.0,
            spool_y,
            spool_z,
            14.0,
            14.0,
        )
        post_center_x = sign * p.RETURN_SPOOL_POST_CENTER_X - carriage_global_x
        spool_post = _x_cylinder(
            p.RETURN_SPOOL_POST_DIAMETER / 2.0,
            p.RETURN_SPOOL_POST_LENGTH,
            post_center_x,
            spool_y,
            spool_z,
        )
        body = body + spool_boss + spool_bridge + spool_x_bridge + spool_post
        # At the keyboard endpoint the fixed strip clamp occupies only the
        # outer skin of this carriage.  Preserve more than 8 mm of inner
        # structure while providing a real, symmetric clamp/tool relief.
        outboard = 1.0 if side == "right" else -1.0
        clamp_relief = Box(
            7.0,
            54.0,
            20.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).moved(Pos(outboard * 0.9, 9.0, 85.0))
        body = body - clamp_relief
        # The spool bridge is added after the tray mounting holes are first
        # cut.  Re-cut both holes through the completed union so the bridge
        # can never close either M4 screw path.
        for y_offset in (-10.0, 10.0):
            body = body - _x_cylinder(
                p.M4_CLEARANCE_DIAMETER / 2.0,
                15.0,
                inward * 4.0,
                tray_node[0] + y_offset,
                tray_node[1],
            )
        # Return-end clearance between the lower rear edge of this mounting
        # pad and the keyboard carriage yoke.  The notch stays 5 mm clear of
        # the nearest M4 mounting-hole center and crosses only the unloaded
        # corner of the pad.
        body = body - Box(
            30.0,
            8.0,
            4.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).moved(Pos(0.0, -1.0, 68.5))
    else:
        body = body + _ring_axis_yz(
            10.0,
            3.8,
            2.0,
            spring_seats()[0],
            mech.KEYBOARD_U,
            spring_x,
        )
        # The trackpad tray's side ear crosses this inner spring-seat face
        # during the first 40% of travel.  A shallow symmetric relief supplies
        # real axial clearance without weakening the guide or spring tunnel.
        inner_sign = 1.0 if side == "left" else -1.0
        tray_sweep_relief = Box(
            3.0,
            42.0,
            21.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).moved(Pos(inner_sign * 7.5, 52.0, 57.5))
        body = body - tray_sweep_relief
        # The trackpad tray mounting ear also passes the central keyboard
        # yoke during the first 10% of travel.  Remove the 2.0 mm inner-face
        # layer so calibrated print error does not consume the carriage gap.
        trackpad_ear_return_relief = Box(
            3.0,
            28.0,
            14.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).moved(Pos(inner_sign * 2.0, -8.0, 66.0))
        body = body - trackpad_ear_return_relief
    # Deep swept clearances can leave isolated slivers from formerly crossing
    # brace intersections.  They are not functional members and cannot be
    # printed as floating islands; retain the single connected load path.
    solids = list(body.solids())
    return max(solids, key=lambda solid: solid.volume) if len(solids) > 1 else body


def make_tpu_spool_retainer():
    return (
        _x_cylinder(7.0, 4.0, 0.0, 0.0, 0.0)
        - _x_cylinder(3.75, 5.0, 0.0, 0.0, 0.0)
    )


def make_eccentric_bushing(kind: str):
    if kind not in ("cross", "guide"):
        raise ValueError(kind)
    outer_diameter = 10.0
    # Fill the 5 mm carriage thickness so the shoulder-screw head bears on a
    # real face.  The flange includes a concentric head pocket: the adjustment
    # annulus remains exposed, while the screw head no longer intersects it.
    width = 5.0
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
    head_pocket = _x_cylinder(
        3.1,
        p.ECCENTRIC_FLANGE_THICKNESS + 0.05,
        flange_x,
        p.ECCENTRICITY,
        0.0,
    )
    body = body - head_pocket
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


def make_tpu_device_pad(kind: str):
    if kind not in ("keyboard", "trackpad"):
        raise ValueError(kind)
    length = 32.0 if kind == "keyboard" else 26.0
    return Box(length, 8.0, 1.2, align=(Align.CENTER, Align.CENTER, Align.MIN))


def make_tpu_base_foot():
    return Box(24.0, 18.0, 2.0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def calibration_coupon():
    coupon = Box(136.0, 42.0, 7.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # Three distinct Ø4 shoulder-clearance candidates; the former coupon put
    # all three at x=-35 and therefore produced only the largest hole.
    holes = [
        (-48.0, 10.0, 4.10),
        (-36.0, 10.0, 4.20),
        (-24.0, 10.0, 4.30),
        # Heat-set pilot candidates for the actual insert batch/material.
        (-48.0, -10.0, 5.20),
        (-34.0, -10.0, 5.40),
        (-20.0, -10.0, 5.60),
        (-6.0, -10.0, 3.80),
        (6.0, -10.0, 4.00),
        (18.0, -10.0, 4.20),
    ]
    for x, y, diameter in holes:
        cutter = Cylinder(
            diameter / 2.0, 9.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Pos(x, y, -1.0))
        coupon = coupon - cutter
    for x, diameter in ((28.0, 8.2), (44.0, 12.2), (60.0, 10.2)):
        pocket = Cylinder(
            diameter / 2.0, 4.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Pos(x, 8.0, 4.0))
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
        "tpu_spool_retainer": make_tpu_spool_retainer(),
        "keyboard_tpu_pad": make_tpu_device_pad("keyboard"),
        "trackpad_tpu_pad": make_tpu_device_pad("trackpad"),
        "tpu_base_foot": make_tpu_base_foot(),
        "calibration_coupon": calibration_coupon(),
    }
