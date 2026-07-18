"""Parametric kinematic CAD for the independent A-P validation coupon.

The belt is represented by pose-specific BREP segments.  It communicates the
required planar single-layer path; it is not a flexible-material simulation.
"""

from dataclasses import dataclass
from math import atan2, ceil, cos, degrees, floor, pi, radians, sin
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

from build123d import Align, Axis, Box, Compound, Cylinder, Location, Shape, export_step, import_step


# Kinematic contract (millimetres, degrees only at the public boundary).
PULLEY_TOOTH_COUNT = 41
SYNCHRONOUS_PROFILE = "2M-GT2"
FEED_TRAVEL = PULLEY_TOOTH_COUNT * 2.0 / 4.0
Z_FLOAT_TRAVEL = 15.0
BELT_WIDTH = 12.0
BELT_PITCH = 2.0
DRUM_PITCH_DIAMETER = PULLEY_TOOTH_COUNT * BELT_PITCH / pi
DRUM_PITCH_RADIUS = DRUM_PITCH_DIAMETER / 2.0
BELT_TOTAL_CENTERLINE = 60.0

# Coupon packaging.
HOUSING_OUTER_DIAMETER = 46.0
HOUSING_HEIGHT = 58.0
HOUSING_INNER_RADIUS = 20.0
BELT_Z = 29.0
DRUM_ROOT_RADIUS = 12.05
DRUM_OUTER_RADIUS = DRUM_PITCH_RADIUS - 0.25
DRUM_GROOVE_TANGENTIAL = 1.10
BELT_BACKING_INNER_RADIUS = 13.30
BELT_BACKING_THICKNESS = 1.2
TOOTH_RADIAL_DEPTH = 0.90
TOOTH_LENGTH = 0.78
BELT_CHORD_STEP_DEG = 5.0
OVERLAP_TOLERANCE = 1e-5


@dataclass(frozen=True)
class KinematicState:
    travel: float
    carrier_dy: float
    carrier_dz: float
    clamp_dy: float
    clamp_dz: float
    feed: float
    angle_rad: float
    angle_deg: float
    straight_length: float


@dataclass(frozen=True)
class InterferenceHit:
    travel: float
    moving_label: str
    fixed_label: str
    volume: float


@dataclass(frozen=True)
class InterferenceReport:
    samples: int
    checked_pairs: int
    unintended: List[InterferenceHit]


@dataclass(frozen=True)
class RoundtripResult:
    readable: bool
    solid_count: int


@dataclass(frozen=True)
class BREPEngagementReport:
    pulley_grooves: int
    meshed_teeth: int
    phase_misses: int
    pulley_interference_volume: float


@dataclass(frozen=True)
class StopContactReport:
    zero_stop_gap: float
    ninety_stop_gap: float
    penetration_volume: float


@dataclass(frozen=True)
class GuideContactReport:
    y_contact_gap: float
    penetration_volume: float
    z_capture_above: float
    z_capture_below: float


def _clamp01(value: float) -> float:
    if not 0.0 <= value <= 1.0:
        raise ValueError("travel must be between 0 and 1")
    return float(value)


def kinematics(travel: float) -> KinematicState:
    """Return the single real degree-of-freedom mapping for a posed coupon."""
    t = _clamp01(travel)
    feed = FEED_TRAVEL * t
    angle = feed / DRUM_PITCH_RADIUS
    return KinematicState(
        travel=t,
        carrier_dy=feed,
        carrier_dz=-Z_FLOAT_TRAVEL * t,
        clamp_dy=feed,
        clamp_dz=0.0,
        feed=feed,
        angle_rad=angle,
        angle_deg=degrees(angle),
        straight_length=BELT_TOTAL_CENTERLINE - feed,
    )


def posed_belt_length(travel: float) -> float:
    state = kinematics(travel)
    return state.straight_length + DRUM_PITCH_RADIUS * state.angle_rad


def engaged_tooth_count(travel: float) -> int:
    return brep_engagement_report(travel).meshed_teeth


def _box(size: Tuple[float, float, float], center: Tuple[float, float, float]) -> Shape:
    return Box(*size, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(Location(center))


def _cylinder(radius: float, height: float, z_min: float) -> Shape:
    return Cylinder(radius, height, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0.0, 0.0, z_min))
    )


def _polar_box(
    radial_center: float,
    angle_rad: float,
    radial_thickness: float,
    tangential_length: float,
    z_width: float,
    z_center: float,
) -> Shape:
    x = radial_center * cos(angle_rad)
    y = radial_center * sin(angle_rad)
    # Rotate at the local origin first. Rotating a translated box about world Z
    # doubles its polar angle and was the source of the apparent 180 degree wrap.
    part = _box((radial_thickness, tangential_length, z_width), (0.0, 0.0, z_center))
    part = part.rotate(axis=Axis.Z, angle=degrees(angle_rad) + 180.0)
    return part.moved(Location((x, y, 0.0)))


def _label(shape: Shape, label: str) -> Shape:
    shape.label = label
    return shape


def _compound(label: str, shapes: Sequence[Shape]) -> Compound:
    result = Compound(label=label, children=list(shapes))
    result.label = label
    return result


def _housing(sectioned: bool = False) -> Shape:
    outer = _cylinder(HOUSING_OUTER_DIAMETER / 2.0, HOUSING_HEIGHT, 0.0)
    inner = _cylinder(HOUSING_INNER_RADIUS, HOUSING_HEIGHT + 2.0, -1.0)
    shell = outer - inner
    # Tangential entry port. The separate tunnel encloses this opening.
    portal = _box((12.0, 30.0, 20.0), (-13.0, -15.0, BELT_Z))
    shell = shell - portal
    if sectioned:
        # Remove the +X/+Y quadrant for an unambiguous internal section view.
        shell = shell - _box((26.0, 26.0, 62.0), (13.0, 13.0, 29.0))
    return _label(shell, "protective_housing_section" if sectioned else "protective_housing")


def _entry_guide() -> Shape:
    outer = _box((10.0, 19.0, 18.0), (-13.0, -28.5, BELT_Z))
    tunnel = _box((4.8, 21.0, 14.0), (-13.0, -28.5, BELT_Z))
    guide = outer - tunnel
    # Two shoulders bridge the tunnel casing into the housing wall at the port.
    guide = guide + _box((3.0, 7.0, 18.0), (-18.5, -20.5, BELT_Z))
    guide = guide + _box((3.0, 7.0, 18.0), (-7.5, -20.5, BELT_Z))
    return _label(guide, "closed_entry_guide")


def _bearing_seat(label: str, z_min: float) -> Shape:
    seat = _cylinder(9.0, 5.0, z_min) - _cylinder(4.2, 7.0, z_min - 1.0)
    z_center = z_min + 2.5
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        seat = seat + _polar_box(14.6, radians(angle_deg), 11.6, 3.0, 5.0, z_center)
    return _label(seat, label)


def _thrust_interface() -> Shape:
    thrust = _cylinder(10.0, 2.0, 0.5) - _cylinder(4.3, 4.0, -0.5)
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        thrust = thrust + _polar_box(15.0, radians(angle_deg), 10.4, 3.0, 2.0, 1.5)
    return _label(thrust, "lower_thrust_interface")


def _fixed_stops() -> Compound:
    # The faces are placed on the non-travel side of the lug at each endpoint.
    # This avoids the corner sweep penetration caused by radial end blocks.
    zero = _box((5.0, 3.0, 4.0), (12.5, 3.0, 39.0))
    zero = zero + _box((8.0, 3.0, 4.0), (19.0, 3.0, 39.0))
    zero.label = "hard_stop_0deg"
    ninety = _box((3.0, 5.0, 4.0), (-3.0, -12.5, 39.0))
    ninety = ninety + _box((3.0, 8.0, 4.0), (-3.0, -19.0, 39.0))
    ninety.label = "hard_stop_90deg"
    return _compound("hard_stops_0_90", [zero, ninety])


def _spring_interface() -> Shape:
    plate = _box((5.0, 30.0, 12.0), (25.5, -6.0, 9.0))
    for z in (5.0, 9.0, 13.0):
        hole = Cylinder(1.6, 7.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
        hole = hole.rotate(axis=Axis.Y, angle=90.0).moved(Location((22.0, -6.0, z)))
        plate = plate - hole
    return _label(plate, "spring_anchor_interface_uncommitted")


def _straight_belt(state: KinematicState) -> Shape:
    y0 = -BELT_TOTAL_CENTERLINE + state.feed
    return _box(
        (BELT_BACKING_THICKNESS, state.straight_length, BELT_WIDTH),
        (-(BELT_BACKING_INNER_RADIUS + BELT_BACKING_THICKNESS / 2.0), y0 / 2.0, BELT_Z),
    )


def _arc_backing(state: KinematicState) -> List[Shape]:
    if state.angle_rad <= 1e-12:
        return []
    count = max(1, ceil(state.angle_deg / BELT_CHORD_STEP_DEG))
    delta = state.angle_rad / count
    radius = BELT_BACKING_INNER_RADIUS + BELT_BACKING_THICKNESS / 2.0
    length = radius * delta * 1.03
    result = []
    for index in range(count):
        a = pi - (index + 0.5) * delta
        result.append(_polar_box(radius, a, BELT_BACKING_THICKNESS, length, BELT_WIDTH, BELT_Z))
    return result


def _pulley_groove_spaces(state: KinematicState) -> List[Shape]:
    """Return 41 rounded 2M-GT2 nominal groove cutter BREP solids."""
    pitch_angle = BELT_PITCH / DRUM_PITCH_RADIUS
    result = []
    for index in range(PULLEY_TOOTH_COUNT):
        angle = pi - (index + 0.5) * pitch_angle - state.angle_rad
        root_center = DRUM_ROOT_RADIUS + 0.48
        x = root_center * cos(angle)
        y = root_center * sin(angle)
        rounded_root = Cylinder(0.55, 14.4, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((x, y, 21.8))
        )
        mouth_center = (root_center + DRUM_OUTER_RADIUS + 0.2) / 2.0
        mouth_depth = DRUM_OUTER_RADIUS + 0.2 - root_center
        mouth = _polar_box(mouth_center, angle, mouth_depth, 0.92, 14.4, 29.0)
        result.append(rounded_root + mouth)
    return result


def _toothed_drum(state: KinematicState) -> Shape:
    drum = _cylinder(DRUM_OUTER_RADIUS, 14.0, 22.0)
    for groove in _pulley_groove_spaces(state):
        drum = drum - groove
    return _label(drum, "reel_drum_41T_pitch_dia_26p10")


def _clockwise_from_entry(angle: float) -> float:
    return (pi - angle) % (2.0 * pi)


def _engaged_groove_angles(state: KinematicState) -> List[float]:
    if state.angle_rad <= 1e-12:
        return []
    angles = []
    pitch_angle = BELT_PITCH / DRUM_PITCH_RADIUS
    for index in range(PULLEY_TOOTH_COUNT):
        angle = pi - (index + 0.5) * pitch_angle - state.angle_rad
        normalized = atan2(sin(angle), cos(angle))
        distance = _clockwise_from_entry(normalized)
        if 1e-8 < distance < state.angle_rad - 1e-8:
            angles.append(normalized)
    return sorted(angles, key=_clockwise_from_entry)


def _arc_belt_teeth(state: KinematicState) -> List[Shape]:
    radial_center = BELT_BACKING_INNER_RADIUS - TOOTH_RADIAL_DEPTH / 2.0
    result = []
    for angle in _engaged_groove_angles(state):
        x = radial_center * cos(angle)
        y = radial_center * sin(angle)
        rounded_tooth = Cylinder(TOOTH_LENGTH / 2.0, BELT_WIDTH, align=(Align.CENTER, Align.CENTER, Align.MIN))
        rounded_tooth = rounded_tooth.moved(Location((x, y, BELT_Z - BELT_WIDTH / 2.0)))
        neck = _polar_box(BELT_BACKING_INNER_RADIUS - 0.10, angle, 0.36, 0.62, BELT_WIDTH, BELT_Z)
        result.append(rounded_tooth + neck)
    return result


def _belt_teeth(state: KinematicState) -> List[Shape]:
    result: List[Shape] = []
    radial_center = BELT_BACKING_INNER_RADIUS - TOOTH_RADIAL_DEPTH / 2.0
    # Straight-run teeth, with transverse tooth bars spanning the 12 mm width.
    tooth_count = max(1, floor(state.straight_length / BELT_PITCH))
    y0 = -BELT_TOTAL_CENTERLINE + state.feed
    for index in range(tooth_count):
        y = y0 + BELT_PITCH * (index + 0.5)
        if y < -0.25:
            rounded_tooth = Cylinder(
                TOOTH_LENGTH / 2.0, BELT_WIDTH, align=(Align.CENTER, Align.CENTER, Align.MIN)
            ).moved(Location((-radial_center, y, BELT_Z - BELT_WIDTH / 2.0)))
            neck = _box((0.36, 0.62, BELT_WIDTH), (-(BELT_BACKING_INNER_RADIUS - 0.10), y, BELT_Z))
            result.append(rounded_tooth + neck)
    # Engaged teeth are selected from the posed physical pulley groove phase.
    result.extend(_arc_belt_teeth(state))
    return result


def _floating_fork(state: KinematicState) -> Compound:
    clamp_y = -BELT_TOTAL_CENTERLINE - 4.0 + state.feed
    center_z = 36.5 + state.carrier_dz
    rails = [
        _label(_box((2.0, 4.0, 30.0), (-17.0, clamp_y, center_z)), "left_z_drive_key"),
        _label(_box((2.0, 4.0, 30.0), (-9.0, clamp_y, center_z)), "right_z_drive_key"),
        _box((12.0, 4.0, 3.0), (-13.0, clamp_y - 2.0, center_z + 15.0)),
        _box((2.0, 6.0, 3.0), (-17.0, clamp_y - 3.0, center_z + 15.0)),
        _box((2.0, 6.0, 3.0), (-9.0, clamp_y - 3.0, center_z + 15.0)),
    ]
    return _compound("floating_z_fork_15mm", rails)


def _input_carrier(state: KinematicState) -> Shape:
    clamp_y = -BELT_TOTAL_CENTERLINE - 4.0 + state.feed
    return _label(_box((46.0, 28.0, 4.0), (-13.0, clamp_y - 18.0, 52.0 + state.carrier_dz)), "horizontal_input_coupon")


def _clamp_slider(state: KinematicState) -> Shape:
    clamp_y = -BELT_TOTAL_CENTERLINE - 4.0 + state.feed
    slider = _box((14.0, 8.0, 8.0), (-13.0, clamp_y, BELT_Z))
    # Two vertical keyways have Y faces coincident with the drive keys: they
    # transmit feed force while leaving clearance in X and free stroke in Z.
    for x in (-17.0, -9.0):
        slot = _box((2.4, 4.0, 10.0), (x, clamp_y, BELT_Z))
        slider = slider - slot
    # Removable clamp screws are kept between the two guide keyways.
    for y_offset in (-2.6, 2.6):
        tool = Cylinder(1.0, 10.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
        tool = tool.moved(Location((-13.0, clamp_y + y_offset, BELT_Z - 5.0)))
        slider = slider - tool
    return _label(slider, "belt_end_clamp_slider")


def _rotating_parts(state: KinematicState) -> Tuple[Shape, Shape, Shape, Shape]:
    angle = -state.angle_deg
    drum = _toothed_drum(state)
    shaft = _label(_cylinder(4.0, 66.0, -2.0), "vertical_output_shaft")
    pointer = _box((20.0, 4.0, 3.0), (10.0, 0.0, 61.5)).rotate(axis=Axis.Z, angle=angle)
    pointer = _label(pointer, "short_output_pointer")
    # Radial tip is exactly R15: coincident with the stop inner face only at
    # 0 and 90 degrees, without positive-volume penetration.
    lug = _box((11.0, 3.0, 4.0), (9.5, 0.0, 39.0)).rotate(axis=Axis.Z, angle=angle)
    lug = _label(lug, "rotary_stop_lug")
    return drum, shaft, pointer, lug


def required_labels() -> set:
    return {
        "protective_housing",
        "closed_entry_guide",
        "upper_bearing_seat",
        "lower_bearing_seat",
        "lower_thrust_interface",
        "hard_stops_0_90",
        "spring_anchor_interface_uncommitted",
        "reel_drum_41T_pitch_dia_26p10",
        "vertical_output_shaft",
        "short_output_pointer",
        "rotary_stop_lug",
        "belt_backing_pose",
        "transverse_belt_teeth_pose",
        "belt_end_clamp_slider",
        "floating_z_fork_15mm",
        "horizontal_input_coupon",
    }


def build_pose(travel: float) -> Compound:
    state = kinematics(travel)
    backing = _compound("belt_backing_pose", [_straight_belt(state), *_arc_backing(state)])
    teeth = _compound("transverse_belt_teeth_pose", _belt_teeth(state))
    drum, shaft, pointer, lug = _rotating_parts(state)
    children = [
        _housing(),
        _entry_guide(),
        _bearing_seat("upper_bearing_seat", 49.0),
        _bearing_seat("lower_bearing_seat", 5.0),
        _thrust_interface(),
        _fixed_stops(),
        _spring_interface(),
        drum,
        shaft,
        pointer,
        lug,
        backing,
        teeth,
        _clamp_slider(state),
        _floating_fork(state),
        _input_carrier(state),
    ]
    assembly = Compound(label=f"ap_flat_reel_coupon_{state.angle_deg:.1f}deg", children=children)
    assembly.label = f"ap_flat_reel_coupon_{state.angle_deg:.1f}deg"
    return assembly


def _translated(shape: Shape, xyz: Tuple[float, float, float], label: str) -> Shape:
    moved = shape.moved(Location(xyz))
    moved.label = label
    return moved


def build_exploded_section() -> Compound:
    """Return a sectioned and exploded diagnostic assembly, not an assembly pose."""
    state = kinematics(0.5)
    drum, shaft, pointer, lug = _rotating_parts(state)
    backing = _compound("belt_backing_pose_exploded", [_straight_belt(state), *_arc_backing(state)])
    teeth = _compound("transverse_belt_teeth_pose_exploded", _belt_teeth(state))
    children = [
        _housing(sectioned=True),
        _translated(_entry_guide(), (-12.0, 0.0, 0.0), "closed_entry_guide_exploded"),
        _translated(_bearing_seat("upper_bearing_seat", 49.0), (0.0, 0.0, 18.0), "upper_bearing_seat_exploded"),
        _translated(_bearing_seat("lower_bearing_seat", 5.0), (0.0, 0.0, -12.0), "lower_bearing_seat_exploded"),
        _translated(_thrust_interface(), (0.0, 0.0, -20.0), "lower_thrust_interface_exploded"),
        _translated(drum, (32.0, 0.0, 0.0), "reel_drum_pitch_dia_26_exploded"),
        _translated(shaft, (44.0, 0.0, 0.0), "vertical_output_shaft_exploded"),
        _translated(pointer, (44.0, 0.0, 0.0), "short_output_pointer_exploded"),
        _translated(lug, (32.0, 0.0, 0.0), "rotary_stop_lug_exploded"),
        _translated(backing, (-12.0, -10.0, 0.0), "belt_backing_pose_exploded"),
        _translated(teeth, (-12.0, -10.0, 0.0), "transverse_belt_teeth_pose_exploded"),
        _translated(_clamp_slider(state), (-12.0, -10.0, 0.0), "belt_end_clamp_slider_exploded"),
        _translated(_floating_fork(state), (-28.0, -10.0, 0.0), "floating_z_fork_15mm_exploded"),
        _translated(_input_carrier(state), (-28.0, -10.0, 12.0), "horizontal_input_coupon_exploded"),
        _translated(_fixed_stops(), (0.0, 0.0, 10.0), "hard_stops_0_90_exploded"),
        _spring_interface(),
    ]
    result = Compound(label="ap_coupon_cylinder_section_exploded", children=children)
    result.label = "ap_coupon_cylinder_section_exploded"
    return result


def _label_map(assembly: Compound) -> Dict[str, Shape]:
    return {child.label: child for child in assembly.children}


def measured_wrap_angle_deg(travel: float) -> float:
    """Measure the posed backing sweep from BREP segment centroids."""
    state = kinematics(travel)
    segments = _arc_backing(state)
    if not segments:
        return 0.0
    center_angles = []
    for segment in segments:
        center = segment.bounding_box().center()
        center_angles.append(_clockwise_from_entry(atan2(center.Y, center.X)))
    if len(center_angles) == 1:
        return state.angle_deg
    center_angles.sort()
    segment_pitch = (center_angles[-1] - center_angles[0]) / (len(center_angles) - 1)
    return degrees(center_angles[-1] - center_angles[0] + segment_pitch)


def measure_step_wrap_angle_deg(path: Path) -> float:
    """Measure wrap from arc-segment BREP centers after STEP import."""
    imported = import_step(str(Path(path)))
    backing = next(child for child in imported.children if child.label == "belt_backing_pose")
    expected_radius = BELT_BACKING_INNER_RADIUS + BELT_BACKING_THICKNESS / 2.0
    center_angles = []
    for child in backing.children:
        center = child.bounding_box().center()
        radius = (center.X * center.X + center.Y * center.Y) ** 0.5
        if abs(radius - expected_radius) <= 1.0:
            center_angles.append(_clockwise_from_entry(atan2(center.Y, center.X)))
    if not center_angles:
        return 0.0
    center_angles.sort()
    if len(center_angles) == 1:
        return degrees(2.0 * atan2(BELT_CHORD_STEP_DEG, expected_radius))
    segment_pitch = (center_angles[-1] - center_angles[0]) / (len(center_angles) - 1)
    return degrees(center_angles[-1] - center_angles[0] + segment_pitch)


def brep_engagement_report(travel: float) -> BREPEngagementReport:
    state = kinematics(travel)
    teeth = _arc_belt_teeth(state)
    grooves = _pulley_groove_spaces(state)
    drum = _toothed_drum(state)
    misses = 0
    interference = 0.0
    for tooth in teeth:
        groove_overlap = max((tooth & groove).volume for groove in grooves)
        # The tooth neck remains outside the pulley; at least 60% of the full
        # rounded tooth BREP must occupy one matching groove cutter volume.
        if groove_overlap < tooth.volume * 0.60:
            misses += 1
        interference += (tooth & drum).volume
    return BREPEngagementReport(
        pulley_grooves=len(grooves),
        meshed_teeth=len(teeth) - misses,
        phase_misses=misses,
        pulley_interference_volume=interference,
    )


def stop_contact_report(travel: float) -> StopContactReport:
    state = kinematics(travel)
    lug = _rotating_parts(state)[3]
    zero_stop, ninety_stop = _fixed_stops().children
    return StopContactReport(
        zero_stop_gap=lug.distance_to(zero_stop),
        ninety_stop_gap=lug.distance_to(ninety_stop),
        penetration_volume=(lug & zero_stop).volume + (lug & ninety_stop).volume,
    )


def guide_contact_report(travel: float) -> GuideContactReport:
    state = kinematics(travel)
    fork = _floating_fork(state)
    slider = _clamp_slider(state)
    key_box = fork.children[0].bounding_box()
    slider_box = slider.bounding_box()
    return GuideContactReport(
        y_contact_gap=fork.distance_to(slider),
        penetration_volume=(fork & slider).volume,
        z_capture_above=key_box.max.Z - slider_box.max.Z,
        z_capture_below=slider_box.min.Z - key_box.min.Z,
    )


def fixed_connectivity_report() -> Dict[str, bool]:
    housing = _housing()
    features = {
        "closed_entry_guide": _entry_guide(),
        "upper_bearing_seat": _bearing_seat("upper_bearing_seat", 49.0),
        "lower_bearing_seat": _bearing_seat("lower_bearing_seat", 5.0),
        "lower_thrust_interface": _thrust_interface(),
        "hard_stops_0_90": _fixed_stops(),
    }
    return {label: len((housing + feature).solids()) == 1 for label, feature in features.items()}


def interference_report(samples: int = 11) -> InterferenceReport:
    if samples < 2:
        raise ValueError("at least two poses are required")
    # Every top-level pair is evaluated. Positive-volume intersections are only
    # accepted for explicit structural joins or rotating-part hub joins.
    intended_overlap_pairs = {
        frozenset(("protective_housing", "closed_entry_guide")),
        frozenset(("protective_housing", "upper_bearing_seat")),
        frozenset(("protective_housing", "lower_bearing_seat")),
        frozenset(("protective_housing", "lower_thrust_interface")),
        frozenset(("protective_housing", "hard_stops_0_90")),
        frozenset(("protective_housing", "spring_anchor_interface_uncommitted")),
        frozenset(("reel_drum_41T_pitch_dia_26p10", "vertical_output_shaft")),
        frozenset(("vertical_output_shaft", "short_output_pointer")),
        frozenset(("vertical_output_shaft", "rotary_stop_lug")),
        frozenset(("belt_backing_pose", "transverse_belt_teeth_pose")),
        frozenset(("belt_backing_pose", "belt_end_clamp_slider")),
        frozenset(("transverse_belt_teeth_pose", "belt_end_clamp_slider")),
        frozenset(("floating_z_fork_15mm", "horizontal_input_coupon")),
    }
    hits: List[InterferenceHit] = []
    checked_pairs = 0
    for index in range(samples):
        t = index / (samples - 1)
        parts = _label_map(build_pose(t))
        labels = list(parts)
        for first_index, moving_label in enumerate(labels):
            for fixed_label in labels[first_index + 1 :]:
                checked_pairs += 1
                overlap = parts[moving_label] & parts[fixed_label]
                volume = overlap.volume
                if volume <= OVERLAP_TOLERANCE:
                    continue
                if frozenset((moving_label, fixed_label)) in intended_overlap_pairs:
                    continue
                hits.append(InterferenceHit(t, moving_label, fixed_label, volume))
    return InterferenceReport(samples=samples, checked_pairs=checked_pairs, unintended=hits)


def roundtrip_step(assembly: Compound, path: Path) -> RoundtripResult:
    path = Path(path)
    export_step(assembly, str(path))
    readable = False
    solid_count = 0
    try:
        imported = import_step(str(path))
        solid_count = len(imported.solids())
        readable = imported.is_valid() and solid_count > 0
    except Exception:
        readable = False
    return RoundtripResult(readable=readable, solid_count=solid_count)


def gen_step() -> Compound:
    """Default source envelope: mid-travel diagnostic pose."""
    return build_pose(0.5)
