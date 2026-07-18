"""Explicit V12 assembly placements for static travel poses."""

from dataclasses import dataclass
from math import atan2, degrees, hypot
from typing import Dict, List, Tuple

from build123d import (
    Align,
    Axis,
    Box,
    Color,
    Compound,
    Cylinder,
    Face,
    Helix,
    Plane,
    Pos,
    Wire,
    sweep,
)

import mechanism as mech
import parameters as p
import parts


SIDE_FRAME_X = 148.0
KEYBOARD_CARRIAGE_X = 148.0
TRACKPAD_CARRIAGE_X = 156.0

# Sourceable low-head precision shoulder screws use standard 20/16/12 mm
# shoulder lengths.  The X-layer datums below are set so the shoulder ends
# exactly at the far bearing inner-ring face on both sides.
KEYBOARD_GUIDE_SHOULDER_LENGTH = 20.0
TRACKPAD_GUIDE_SHOULDER_LENGTH = 16.0
CROSS_SHOULDER_LENGTH = 12.0
SHOULDER_THREAD_LENGTH = 4.0
SHOULDER_HEAD_DIAMETER = 6.0
# AMPG STR402M4 ultra-low smooth head: Ø6 x 1.30 mm, 2 mm hex.
SHOULDER_HEAD_THICKNESS = 1.3
CROSS_SLOT_X = 154.5

BASE_COLOR = Color(0.24, 0.26, 0.29)
FIXED_COLOR = Color(0.16, 0.18, 0.21)
KEYBOARD_COLOR = Color(0.88, 0.16, 0.13)
TRACKPAD_COLOR = Color(0.10, 0.38, 0.92)
BEARING_COLOR = Color(0.88, 0.68, 0.18)
HARDWARE_COLOR = Color(0.70, 0.73, 0.76)
SPRING_COLOR = Color(0.66, 0.57, 0.35)
TPU_COLOR = Color(0.12, 0.12, 0.13)


def _label(shape, name: str, color):
    shape.label = name
    shape.color = color
    return shape


def _x_cylinder(radius: float, length: float, x: float, y: float, z: float):
    return (
        Cylinder(radius, length, align=(Align.CENTER, Align.CENTER, Align.CENTER))
        .rotate(Axis.Y, 90.0)
        .moved(Pos(x, y, z))
    )


def _ring_x(od: float, bore: float, width: float, x: float, y: float, z: float):
    return _x_cylinder(od / 2.0, width, x, y, z) - _x_cylinder(
        bore / 2.0, width + 1.0, x, y, z
    )


def _ring_z(od: float, bore: float, width: float, x: float, y: float, z_min: float):
    outer = Cylinder(
        od / 2.0, width, align=(Align.CENTER, Align.CENTER, Align.MIN)
    ).moved(Pos(x, y, z_min))
    inner = Cylinder(
        bore / 2.0, width + 1.0, align=(Align.CENTER, Align.CENTER, Align.MIN)
    ).moved(Pos(x, y, z_min - 0.5))
    return outer - inner


def _base_mount_screw(x: float, y: float):
    shaft = Cylinder(
        2.0, 8.0, align=(Align.CENTER, Align.CENTER, Align.MIN)
    ).moved(Pos(x, y, 2.5))
    head = Cylinder(
        4.0, 2.5, align=(Align.CENTER, Align.CENTER, Align.MIN)
    ).moved(Pos(x, y, 0.0))
    return shaft + head


def _tray_mount_screw(side_sign: float, outer_face_x: float, length: float, y: float, z: float):
    inward = -side_sign
    shaft_center_x = outer_face_x + inward * length / 2.0
    shaft = _x_cylinder(2.0, length, shaft_center_x, y, z)
    head_thickness = 2.5
    head_center_x = outer_face_x - inward * head_thickness / 2.0
    head = _x_cylinder(4.0, head_thickness, head_center_x, y, z)
    return shaft + head


def _shoulder_fastener(
    carriage_x: float,
    bearing_x: float,
    carriage_thickness: float,
    bearing_width: float,
    y: float,
    z: float,
):
    direction = 1.0 if bearing_x > carriage_x else -1.0
    head_thickness = SHOULDER_HEAD_THICKNESS
    near_x = carriage_x - direction * carriage_thickness / 2.0
    head_center_x = near_x - direction * head_thickness / 2.0
    bearing_far_x = bearing_x + direction * bearing_width / 2.0
    # True ISO-7379 topology: the bearing runs only on the Ø4 shoulder;
    # a Ø3 threaded tip begins at the bearing's far inner-ring face.
    shoulder = _x_cylinder(
        p.SHAFT_DIAMETER / 2.0,
        abs(bearing_far_x - near_x),
        (bearing_far_x + near_x) / 2.0,
        y,
        z,
    )
    thread_far_x = bearing_far_x + direction * SHOULDER_THREAD_LENGTH
    threaded_tip = _x_cylinder(
        1.5,
        abs(thread_far_x - bearing_far_x),
        (thread_far_x + bearing_far_x) / 2.0,
        y,
        z,
    )
    head = _x_cylinder(SHOULDER_HEAD_DIAMETER / 2.0, head_thickness, head_center_x, y, z)
    return shoulder + threaded_tip + head


def _shoulder_length(carriage_x: float, bearing_x: float, carriage_thickness: float, bearing_width: float):
    direction = 1.0 if bearing_x > carriage_x else -1.0
    near_x = carriage_x - direction * carriage_thickness / 2.0
    bearing_far_x = bearing_x + direction * bearing_width / 2.0
    return abs(bearing_far_x - near_x)


def roller_stack_report() -> Dict[str, object]:
    """Exact commercial shoulder lengths derived from assembly datums."""
    values = {"keyboard_guide": [], "trackpad_guide": [], "cross": []}
    for side, sign in (("left", -1.0), ("right", 1.0)):
        float_offset = p.RIGHT_SIDE_AXIAL_FLOAT if side == "right" else 0.0
        keyboard_x = sign * KEYBOARD_CARRIAGE_X + float_offset
        trackpad_x = sign * TRACKPAD_CARRIAGE_X + float_offset
        keyboard_rail_x = sign * SIDE_FRAME_X + sign * 15.5
        trackpad_rail_x = sign * SIDE_FRAME_X + sign * 19.5
        values["keyboard_guide"].append(
            _shoulder_length(keyboard_x, keyboard_rail_x, 5.0, p.BEARING_604ZZ.width)
        )
        values["trackpad_guide"].append(
            _shoulder_length(trackpad_x, trackpad_rail_x, 5.0, p.BEARING_604ZZ.width)
        )
        values["cross"].append(
            _shoulder_length(keyboard_x, trackpad_x, 5.0, p.MR84ZZ.width)
        )
    return {
        "lengths_mm": values,
        "bilateral_mismatch_mm": {
            key: abs(items[0] - items[1]) for key, items in values.items()
        },
        "threaded_tip_mm": SHOULDER_THREAD_LENGTH,
        "head_diameter_mm": SHOULDER_HEAD_DIAMETER,
        "head_thickness_mm": SHOULDER_HEAD_THICKNESS,
        "washer_mm": 0.5,
        "nut_mm": 2.4,
    }


def _roller_retention(carriage_x: float, bearing_x: float, bearing_width: float, y: float, z: float):
    """Return the far-side washer and nut for a shoulder-bearing stack."""
    direction = 1.0 if bearing_x > carriage_x else -1.0
    washer_width = 0.5
    nut_width = 2.4
    bearing_far_face = bearing_x + direction * bearing_width / 2.0
    washer_x = bearing_far_face + direction * washer_width / 2.0
    nut_x = bearing_far_face + direction * (washer_width + nut_width / 2.0)
    washer = _ring_x(7.0, 3.2, washer_width, washer_x, y, z)
    # 6.4 mm cylindrical envelope conservatively contains a DIN 934 M3 hex
    # nut (5.5 mm AF) while keeping the threaded bore visible in Explorer.
    nut = _ring_x(6.4, 3.0, nut_width, nut_x, y, z)
    return washer, nut


def _eccentric_bushing_at(kind: str, side: str, x: float, desired_center, normal, translation):
    desired_y = desired_center[0] + translation[0]
    desired_z = desired_center[1] + translation[1]
    socket_y = desired_y + p.ECCENTRICITY * normal[0]
    socket_z = desired_z + p.ECCENTRICITY * normal[1]
    angle = degrees(atan2(-normal[1], -normal[0]))
    bushing = parts.make_eccentric_bushing(kind)
    if side == "left":
        bushing = bushing.mirror(Plane.YZ)
    return (
        bushing
        .rotate(Axis.X, angle)
        .moved(Pos(x, socket_y, socket_z))
    )


def _spring_local(length: float):
    turns = max(1, int(round(p.SPRING_ACTIVE_COILS)))
    pitch = length / turns
    centerline_radius = (p.SPRING_OD - p.SPRING_WIRE) / 2.0
    path = Helix(pitch, length, centerline_radius)
    profile_plane = Plane(origin=path.start_point(), z_dir=path.tangent_at(0.0))
    profile = Face(Wire.make_circle(p.SPRING_WIRE / 2.0, profile_plane))
    return sweep(profile, path, is_frenet=True)


def _spring_world(spring_x: float, travel: float, moving_seat):
    length = p.SPRING_INSTALLED_LENGTH - travel * mech.KEYBOARD_PATH
    angle = degrees(atan2(-mech.KEYBOARD_U[0], mech.KEYBOARD_U[1]))
    spring = _spring_local(length).rotate(Axis.X, angle)
    start_face = (
        moving_seat[0] + 6.0 * mech.KEYBOARD_U[0],
        moving_seat[1] + 6.0 * mech.KEYBOARD_U[1],
    )
    return spring.moved(Pos(spring_x, start_face[0], start_face[1]))


def _return_spring_world(side: str, travel: float):
    sign = -1.0 if side == "left" else 1.0
    spring_x = sign * p.RETURN_SPOOL_GLOBAL_X
    delta = mech.pose(travel).trackpad_delta
    spool_center = (
        p.RETURN_SPOOL_CENTER[0] + delta[0],
        p.RETURN_SPOOL_CENTER[1] + delta[1],
    )
    anchor = parts.return_anchor_center()
    # The supplied accessory plate is 18 mm long with its Ø3.2 hole 5 mm
    # from the fixed end.  Its far edge therefore lies 13 mm toward the drum.
    plate_center = (
        anchor[0] + 4.0 * mech.TRACKPAD_U[0],
        anchor[1] + 4.0 * mech.TRACKPAD_U[1],
    )
    tape_start = (
        anchor[0] + 12.5 * mech.TRACKPAD_U[0],
        anchor[1] + 12.5 * mech.TRACKPAD_U[1],
    )
    tape_end = (
        spool_center[0]
        - (p.RETURN_SPRING_MOUNTED_COIL_OD / 2.0 - 1.0) * mech.TRACKPAD_U[0],
        spool_center[1]
        - (p.RETURN_SPRING_MOUNTED_COIL_OD / 2.0 - 1.0) * mech.TRACKPAD_U[1],
    )
    tape = parts._beam_between_yz(
        tape_start,
        tape_end,
        p.RETURN_SPRING_STRIP_THICKNESS,
        p.RETURN_SPRING_STRIP_WIDTH,
        spring_x,
    )
    spring_body = _ring_x(
        p.RETURN_SPRING_MOUNTED_COIL_OD,
        p.RETURN_SPRING_DRUM_BORE,
        p.RETURN_SPRING_STRIP_WIDTH,
        spring_x,
        spool_center[0],
        spool_center[1],
    )
    accessory_plate = parts._oriented_box_yz(
        p.RETURN_SPRING_ACCESSORY_PLATE_WIDTH,
        p.RETURN_SPRING_ACCESSORY_PLATE_LENGTH,
        p.RETURN_SPRING_ACCESSORY_PLATE_THICKNESS,
        plate_center,
        spring_x,
    )
    return_n = (-mech.TRACKPAD_U[1], mech.TRACKPAD_U[0])
    hole_a = (
        anchor[0] - 1.0 * return_n[0],
        anchor[1] - 1.0 * return_n[1],
    )
    hole_b = (
        anchor[0] + 1.0 * return_n[0],
        anchor[1] + 1.0 * return_n[1],
    )
    accessory_plate = accessory_plate - parts._cylinder_between_yz(
        p.RETURN_SPRING_ACCESSORY_HOLE / 2.0,
        hole_a,
        hole_b,
        spring_x,
    )
    return tape + spring_body + accessory_plate, spool_center, anchor


def spring_collision_envelope(spring_x: float, travel: float, moving_seat):
    """Conservative, cheap collision proxy for the detailed helical spring.

    The solid cylinder encloses the complete spring OD over its compressed
    length.  Keeping this separate from the visual helix makes dense motion
    validation deterministic without weakening the clearance test.
    """
    length = p.SPRING_INSTALLED_LENGTH - travel * mech.KEYBOARD_PATH
    angle = degrees(atan2(-mech.KEYBOARD_U[0], mech.KEYBOARD_U[1]))
    envelope = Cylinder(
        p.SPRING_OD / 2.0,
        length,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).rotate(Axis.X, angle)
    start_face = (
        moving_seat[0] + 6.0 * mech.KEYBOARD_U[0],
        moving_seat[1] + 6.0 * mech.KEYBOARD_U[1],
    )
    return envelope.moved(Pos(spring_x, start_face[0], start_face[1]))


def _axis_oriented_part(shape, x: float, center):
    angle = degrees(atan2(-mech.KEYBOARD_U[0], mech.KEYBOARD_U[1]))
    return shape.rotate(Axis.X, angle).moved(Pos(x, center[0], center[1]))


def _rod_between_yz(radius: float, a, b, x: float):
    dy = b[0] - a[0]
    dz = b[1] - a[1]
    length = hypot(dy, dz)
    angle = degrees(atan2(-dy, dz))
    return (
        Cylinder(radius, length, align=(Align.CENTER, Align.CENTER, Align.CENTER))
        .rotate(Axis.X, angle)
        .moved(Pos(x, (a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0))
    )


def _overlap_volume(a, b) -> float:
    common = a & b
    return 0.0 if common is None else sum(s.volume for s in common.solids())


@dataclass
class PoseAssembly:
    compound: Compound
    bodies: Dict[str, object]
    metadata: Dict[str, object]

    def interface_report(self) -> Dict[str, object]:
        disconnected: List[dict] = []
        checks: List[dict] = []
        for side in ("left", "right"):
            pairs = [("base", f"fixed_side_frame_{side}")]
            pairs.extend([
                ("keyboard_tray", f"keyboard_carriage_{side}"),
                ("trackpad_tray", f"trackpad_carriage_{side}"),
            ])
            for a_name, b_name in pairs:
                distance = self.bodies[a_name].distance_to(self.bodies[b_name])
                overlap = _overlap_volume(self.bodies[a_name], self.bodies[b_name])
                item = {
                    "a": a_name,
                    "b": b_name,
                    "distance_mm": distance,
                    "overlap_mm3": overlap,
                }
                checks.append(item)
                if distance > 0.05:
                    disconnected.append(item)

            for prefix, count in (("keyboard_guide", 2), ("trackpad_guide", 2), ("cross", 2)):
                for index in range(1, count + 1):
                    shaft_name = f"{prefix}_shaft_{side}_{index}"
                    bearing_name = f"{prefix}_bearing_{side}_{index}"
                    distance = self.bodies[shaft_name].distance_to(self.bodies[bearing_name])
                    item = {"a": shaft_name, "b": bearing_name, "distance_mm": distance}
                    checks.append(item)
                    if distance > 0.05:
                        disconnected.append(item)
                    washer_name = f"{prefix}_retaining_washer_{side}_{index}"
                    nut_name = f"{prefix}_retaining_nut_{side}_{index}"
                    for first, second in ((bearing_name, washer_name), (washer_name, nut_name)):
                        distance = self.bodies[first].distance_to(self.bodies[second])
                        item = {"a": first, "b": second, "distance_mm": distance}
                        checks.append(item)
                        if distance > 0.05:
                            disconnected.append(item)
            spring_name = f"trackpad_return_spring_{side}"
            for mate_name in (
                f"trackpad_return_spool_retainer_{side}",
                f"fixed_return_plate_screw_{side}",
            ):
                distance = self.bodies[spring_name].distance_to(self.bodies[mate_name])
                item = {"a": spring_name, "b": mate_name, "distance_mm": distance}
                checks.append(item)
                if distance > 0.05:
                    disconnected.append(item)
        return {
            "checks": checks,
            "disconnected": disconnected,
            "minimum_cross_end_margin_mm": self.metadata["cross_end_margin_mm"],
            "minimum_stop_before_slot_end_mm": self.metadata["stop_before_slot_end_mm"],
        }

    def hardware_interference_report(self, volume_tolerance: float = 0.05) -> Dict[str, object]:
        """Check same-stack hardware conflicts omitted from group collision scans."""
        checks: List[dict] = []
        failures: List[dict] = []
        for side in ("left", "right"):
            for prefix in ("keyboard_guide", "trackpad_guide", "cross"):
                shaft_name = f"{prefix}_shaft_{side}_2"
                bushing_name = f"{prefix}_eccentric_bushing_{side}"
                overlap = _overlap_volume(self.bodies[shaft_name], self.bodies[bushing_name])
                item = {"a": shaft_name, "b": bushing_name, "overlap_mm3": overlap}
                checks.append(item)
                if overlap > volume_tolerance:
                    failures.append(item)
            for mount_index in (1, 2):
                screw_name = f"trackpad_tray_mount_screw_{side}_{mount_index}"
                carriage_name = f"trackpad_carriage_{side}"
                overlap = _overlap_volume(self.bodies[screw_name], self.bodies[carriage_name])
                item = {"a": screw_name, "b": carriage_name, "overlap_mm3": overlap}
                checks.append(item)
                if overlap > volume_tolerance:
                    failures.append(item)
        return {"checks": checks, "failures": failures}

    def preload_report(self) -> Dict[str, float]:
        guide_wall_offset = (p.GUIDE_SLOT_WIDTH - p.BEARING_604ZZ.od) / 2.0
        cross_wall_offset = (p.CROSS_SLOT_WIDTH - p.MR84ZZ.od) / 2.0
        guide_gap = p.GUIDE_SLOT_WIDTH - p.BEARING_604ZZ.od - 2.0 * guide_wall_offset
        cross_gap = p.CROSS_SLOT_WIDTH - p.MR84ZZ.od - 2.0 * cross_wall_offset
        right_nominal_clearance = p.RIGHT_FOLLOWER_SLOT_EXTRA_WIDTH / 2.0
        right_eccentric_min_clearance = max(
            0.0, right_nominal_clearance - p.ECCENTRICITY
        )
        return {
            "left_datum_maximum_adjusted_wall_gap_mm": max(abs(guide_gap), abs(cross_gap)),
            "minimum_adjustment_reserve_mm": min(
                p.ECCENTRICITY - guide_wall_offset,
                p.ECCENTRICITY - cross_wall_offset,
            ),
            "right_side_axial_float_mm": p.RIGHT_SIDE_AXIAL_FLOAT,
            "right_side_normal_float_mm": right_nominal_clearance,
            "right_fixed_roller_nominal_clearance_mm": right_nominal_clearance,
            "right_eccentric_minimum_clearance_mm": right_eccentric_min_clearance,
            "right_side_nominally_preloaded": False,
            "strategy": "left_datum_right_follower",
        }

    def stop_report(self) -> Dict[str, float]:
        travel = float(self.metadata["travel"])
        if travel <= 1e-9:
            active = "return"
        elif travel >= 1.0 - 1e-9:
            active = "pressed"
        else:
            active = "none"
        distances = []
        overlaps = []
        if active != "none":
            for side in ("left", "right"):
                collar = self.bodies[f"fixed_stop_collar_{side}_{active}"]
                carriage = self.bodies[f"keyboard_carriage_{side}"]
                distances.append(collar.distance_to(carriage))
                overlaps.append(_overlap_volume(collar, carriage))
        return {
            "active_stop": active,
            "active_stop_distance_mm": max(distances) if distances else None,
            "active_stop_overlap_mm3": max(overlaps) if overlaps else None,
            "bearing_end_margin_mm": self.metadata["stop_before_slot_end_mm"],
        }


def build_pose(
    travel: float,
    include_devices: bool = False,
    include_visual_springs: bool = True,
) -> PoseAssembly:
    state = mech.pose(travel)
    t = state.travel
    key_translation = state.keyboard_delta
    track_translation = state.trackpad_delta
    children = []
    bodies: Dict[str, object] = {}

    def add(name, shape, color):
        labeled = _label(shape, name, color)
        children.append(labeled)
        bodies[name] = labeled

    add("base", parts.make_open_base(), BASE_COLOR)
    for foot_index, (foot_x, foot_y) in enumerate(p.BASE_FOOT_POSITIONS, 1):
        add(
            f"fixed_tpu_base_foot_{foot_index}",
            parts.make_tpu_base_foot().moved(Pos(foot_x, foot_y, -1.0)),
            TPU_COLOR,
        )
    add(
        "keyboard_tray",
        parts.make_tray("keyboard").moved(
            Pos(0.0, state.keyboard_position[0], state.keyboard_position[1])
        ),
        KEYBOARD_COLOR,
    )
    add(
        "trackpad_tray",
        parts.make_tray("trackpad").moved(
            Pos(0.0, state.trackpad_position[0], state.trackpad_position[1])
        ),
        TRACKPAD_COLOR,
    )
    if include_devices:
        keyboard_device = Box(
            p.KEYBOARD.width,
            p.KEYBOARD.depth,
            p.KEYBOARD.height,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Pos(
            0.0,
            state.keyboard_position[0],
            state.keyboard_position[1] + p.TRAY_SKIN,
        ))
        trackpad_device = Box(
            p.TRACKPAD.width,
            p.TRACKPAD.depth,
            p.TRACKPAD.height,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Pos(
            (p.TRAY_WIDTH - p.TRACKPAD.width) / 2.0,
            state.trackpad_position[0],
            state.trackpad_position[1] + p.TRAY_SKIN,
        ))
        add("keyboard_device_envelope", keyboard_device, HARDWARE_COLOR)
        add("trackpad_device_envelope", trackpad_device, HARDWARE_COLOR)

    for kind, tray_position, pad_positions in (
        (
            "keyboard",
            state.keyboard_position,
            [(-100.0, -45.0), (-100.0, 45.0), (100.0, -45.0), (100.0, 45.0)],
        ),
        (
            "trackpad",
            state.trackpad_position,
            [(20.0, -45.0), (20.0, 45.0), (100.0, -45.0), (100.0, 45.0)],
        ),
    ):
        for index, (pad_x, pad_y) in enumerate(pad_positions, 1):
            pad = parts.make_tpu_device_pad(kind).moved(Pos(
                pad_x,
                tray_position[0] + pad_y,
                tray_position[1] + p.TRAY_SKIN - 0.6,
            ))
            add(f"{kind}_tpu_pad_{index}", pad, TPU_COLOR)

    motion_report = mech.validate_motion(401)
    for side, sign in (("left", -1.0), ("right", 1.0)):
        float_offset = p.RIGHT_SIDE_AXIAL_FLOAT if side == "right" else 0.0
        keyboard_carriage_x = sign * KEYBOARD_CARRIAGE_X + float_offset
        trackpad_carriage_x = sign * TRACKPAD_CARRIAGE_X + float_offset
        cross_slot_x = trackpad_carriage_x

        frame = parts.make_side_frame(side).moved(Pos(sign * SIDE_FRAME_X, 0.0, 0.0))
        add(f"fixed_side_frame_{side}", frame, FIXED_COLOR)

        for mount_index, mount_y in enumerate((-70.0, 116.0), 1):
            mount_x = sign * 145.0
            add(
                f"fixed_base_insert_{side}_{mount_index}",
                _ring_z(
                    p.M4_HEAT_INSERT_OD,
                    p.M4_HEAT_INSERT_BORE,
                    p.M4_HEAT_INSERT_LENGTH,
                    mount_x,
                    mount_y,
                    p.BASE_THICKNESS,
                ),
                HARDWARE_COLOR,
            )
            add(
                f"fixed_base_mount_screw_{side}_{mount_index}",
                _base_mount_screw(mount_x, mount_y),
                HARDWARE_COLOR,
            )

        keyboard_carriage = parts.make_carriage("keyboard", side).moved(
            Pos(keyboard_carriage_x, key_translation[0], key_translation[1])
        )
        trackpad_carriage = parts.make_carriage("trackpad", side).moved(
            Pos(trackpad_carriage_x, track_translation[0], track_translation[1])
        )
        add(f"keyboard_carriage_{side}", keyboard_carriage, KEYBOARD_COLOR)
        add(f"trackpad_carriage_{side}", trackpad_carriage, TRACKPAD_COLOR)

        # Guide bearings, eccentric bushings and shoulder fasteners.
        for kind, translation, carriage_x in (
            ("keyboard", key_translation, keyboard_carriage_x),
            ("trackpad", track_translation, trackpad_carriage_x),
        ):
            initial_centers = parts._bearing_centers(kind)
            normal = parts._guide_geometry(kind)[3]
            if kind == "keyboard":
                local_rail_x = sign * 15.5
            else:
                local_rail_x = sign * 19.5
            rail_x = sign * SIDE_FRAME_X + local_rail_x
            for index, initial_center in enumerate(initial_centers, 1):
                y = initial_center[0] + translation[0]
                z = initial_center[1] + translation[1]
                bearing = _ring_x(
                    p.BEARING_604ZZ.od,
                    p.BEARING_604ZZ.bore,
                    p.BEARING_604ZZ.width,
                    rail_x, y, z,
                )
                shaft = _shoulder_fastener(
                    carriage_x,
                    rail_x,
                    5.0,
                    p.BEARING_604ZZ.width,
                    y,
                    z,
                )
                washer, nut = _roller_retention(
                    carriage_x, rail_x, p.BEARING_604ZZ.width, y, z
                )
                add(f"{kind}_guide_bearing_{side}_{index}", bearing, BEARING_COLOR)
                add(f"{kind}_guide_shaft_{side}_{index}", shaft, HARDWARE_COLOR)
                add(f"{kind}_guide_retaining_washer_{side}_{index}", washer, HARDWARE_COLOR)
                add(f"{kind}_guide_retaining_nut_{side}_{index}", nut, HARDWARE_COLOR)
                if index == 2:
                    bushing = _eccentric_bushing_at(
                        "guide", side, carriage_x, initial_center, normal, translation
                    )
                    add(f"{kind}_guide_eccentric_bushing_{side}", bushing, HARDWARE_COLOR)

        # Two cross bearings contact opposite walls and reverse load without backlash.
        for index, center in enumerate(state.cross_roller_centers, 1):
            bearing_x = cross_slot_x
            bearing = _ring_x(
                p.MR84ZZ.od, p.MR84ZZ.bore, p.MR84ZZ.width,
                bearing_x, center[0], center[1],
            )
            shaft = _shoulder_fastener(
                keyboard_carriage_x,
                bearing_x,
                5.0,
                p.MR84ZZ.width,
                center[0],
                center[1],
            )
            washer, nut = _roller_retention(
                keyboard_carriage_x,
                bearing_x,
                p.MR84ZZ.width,
                center[0],
                center[1],
            )
            add(f"cross_bearing_{side}_{index}", bearing, BEARING_COLOR)
            add(f"cross_shaft_{side}_{index}", shaft, HARDWARE_COLOR)
            add(f"cross_retaining_washer_{side}_{index}", washer, HARDWARE_COLOR)
            add(f"cross_retaining_nut_{side}_{index}", nut, HARDWARE_COLOR)
            if index == 2:
                initial_center = mech.pose(0.0).cross_roller_centers[1]
                bushing = _eccentric_bushing_at(
                    "cross",
                    side,
                    keyboard_carriage_x,
                    initial_center,
                    mech.SLOT_N,
                    key_translation,
                )
                add(f"cross_eccentric_bushing_{side}", bushing, HARDWARE_COLOR)

        # The guide rod now carries only the independent TPU hard stops.  A
        # stock constant-force strip spring supplies return force on the
        # trackpad path, avoiding the compression spring's rising end load.
        moving_seat_initial, fixed_seat = parts.spring_seats()
        spring_x = keyboard_carriage_x - sign * p.SPRING_LATERAL_INSET
        moving_seat = (
            moving_seat_initial[0] + key_translation[0],
            moving_seat_initial[1] + key_translation[1],
        )
        return_spring, return_spool_center, return_anchor = _return_spring_world(side, t)
        add(f"trackpad_return_spring_{side}", return_spring, SPRING_COLOR)
        retainer_x = sign * (
            p.RETURN_SPOOL_GLOBAL_X - p.RETURN_SPOOL_TOTAL_WIDTH / 2.0 - 1.5
        )
        add(
            f"trackpad_return_spool_retainer_{side}",
            parts.make_tpu_spool_retainer().moved(
                Pos(retainer_x, return_spool_center[0], return_spool_center[1])
            ),
            TPU_COLOR,
        )
        return_n = (-mech.TRACKPAD_U[1], mech.TRACKPAD_U[0])
        screw_a = (
            return_anchor[0] - 2.5 * return_n[0],
            return_anchor[1] - 2.5 * return_n[1],
        )
        screw_b = (
            return_anchor[0] + 6.0 * return_n[0],
            return_anchor[1] + 6.0 * return_n[1],
        )
        screw_shaft = _rod_between_yz(
            1.5, screw_a, screw_b, sign * p.RETURN_SPOOL_GLOBAL_X
        )
        head_a = (
            return_anchor[0] - 3.0 * return_n[0],
            return_anchor[1] - 3.0 * return_n[1],
        )
        head_b = (
            return_anchor[0] - 0.5 * return_n[0],
            return_anchor[1] - 0.5 * return_n[1],
        )
        screw_head = _rod_between_yz(
            3.0, head_a, head_b, sign * p.RETURN_SPOOL_GLOBAL_X
        )
        add(
            f"fixed_return_plate_screw_{side}",
            screw_shaft + screw_head,
            HARDWARE_COLOR,
        )
        insert_a = (
            return_anchor[0] + 1.0 * return_n[0],
            return_anchor[1] + 1.0 * return_n[1],
        )
        insert_b = (
            return_anchor[0] + 6.5 * return_n[0],
            return_anchor[1] + 6.5 * return_n[1],
        )
        insert_outer = _rod_between_yz(
            p.RETURN_CLAMP_INSERT_OD / 2.0,
            insert_a,
            insert_b,
            sign * p.RETURN_SPOOL_GLOBAL_X,
        )
        insert_inner = _rod_between_yz(
            p.RETURN_CLAMP_INSERT_BORE / 2.0,
            insert_a,
            insert_b,
            sign * p.RETURN_SPOOL_GLOBAL_X,
        )
        add(
            f"fixed_return_plate_insert_{side}",
            insert_outer - insert_inner,
            HARDWARE_COLOR,
        )
        add(
            f"spring_guide_rod_{side}",
            _rod_between_yz(
                p.SPRING_GUIDE_DIAMETER / 2.0,
                (
                    moving_seat_initial[0] - 30.0 * mech.KEYBOARD_U[0],
                    moving_seat_initial[1] - 30.0 * mech.KEYBOARD_U[1],
                ),
                fixed_seat,
                spring_x,
            ),
            HARDWARE_COLOR,
        )

        # Measured end faces of the reinforced guide tunnel.  The tiny gaps
        # avoid rigid BREP penetration; TPU compliance supplies the real stop.
        return_stop_q = -20.90
        pressed_stop_q = 10.92
        return_stop_center = (
            moving_seat_initial[0] + return_stop_q * mech.KEYBOARD_U[0],
            moving_seat_initial[1] + return_stop_q * mech.KEYBOARD_U[1],
        )
        pressed_stop_center = (
            moving_seat_initial[0] + (mech.KEYBOARD_PATH + pressed_stop_q) * mech.KEYBOARD_U[0],
            moving_seat_initial[1] + (mech.KEYBOARD_PATH + pressed_stop_q) * mech.KEYBOARD_U[1],
        )
        for stop_name, stop_center in (
            ("return", return_stop_center),
            ("pressed", pressed_stop_center),
        ):
            add(
                f"fixed_stop_collar_{side}_{stop_name}",
                _axis_oriented_part(
                    parts.make_tpu_stop_collar(), spring_x, stop_center
                ),
                TPU_COLOR,
            )

        for kind, tray_position, ear_y, hole_z, carriage_x, screw_length in (
            ("keyboard", state.keyboard_position, 0.0, 4.0, keyboard_carriage_x, 10.0),
            ("trackpad", state.trackpad_position, -65.0, 5.0, trackpad_carriage_x, 16.0),
        ):
            outer_face_x = carriage_x + sign * 2.5
            for mount_index, y_offset in enumerate((-10.0, 10.0), 1):
                mount_y = tray_position[0] + ear_y + y_offset
                mount_z = tray_position[1] + hole_z
                insert_x = sign * (145.5 - p.M4_HEAT_INSERT_LENGTH / 2.0)
                add(
                    f"{kind}_tray_insert_{side}_{mount_index}",
                    _ring_x(
                        p.M4_HEAT_INSERT_OD,
                        p.M4_HEAT_INSERT_BORE,
                        p.M4_HEAT_INSERT_LENGTH,
                        insert_x,
                        mount_y,
                        mount_z,
                    ),
                    HARDWARE_COLOR,
                )
                add(
                    f"{kind}_tray_mount_screw_{side}_{mount_index}",
                    _tray_mount_screw(
                        sign, outer_face_x, screw_length, mount_y, mount_z
                    ),
                    HARDWARE_COLOR,
                )

    compound = Compound(
        label=f"cross_axis_slider_v12_travel_{t:.3f}",
        children=children,
    )
    return PoseAssembly(
        compound=compound,
        bodies=bodies,
        metadata={
            "travel": t,
            "cross_end_margin_mm": motion_report["min_cross_end_margin_mm"],
            "stop_before_slot_end_mm": 5.0,
            "include_devices": include_devices,
            "include_visual_springs": include_visual_springs,
        },
    )


def gen_step():
    return build_pose(0.0).compound
