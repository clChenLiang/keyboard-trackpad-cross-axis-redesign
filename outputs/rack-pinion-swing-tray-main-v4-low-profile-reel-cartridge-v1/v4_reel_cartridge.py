"""Purchased open-ended 2 mm pitch belt and screwless V4 terminations.

The model is an interface envelope, not a flexible-material simulation.  Belt
backing and transverse teeth are deliberately separate closed BREP solids.
The reel axis is +Z and the straight run stays centred at ``BELT_Z``.
"""

from dataclasses import dataclass
from math import atan2, cos, degrees, pi, radians, sin
from typing import List, Sequence, Tuple

from build123d import (
    Align,
    AngularDirection,
    Axis,
    Box,
    Compound,
    Cylinder,
    Edge,
    Face,
    Location,
    Plane,
    Shape,
    Solid,
    Wire,
    extrude,
)

from v4_kinematics import (
    BELT_PITCH,
    BELT_TOTAL_CENTERLINE,
    BELT_Z,
    INITIAL_STRAIGHT_BELT_LENGTH,
    INITIAL_WRAP_TEETH,
    REEL_PITCH_RADIUS,
    REEL_TEETH,
    pose_state,
)


BELT_WIDTH = 12.0
BACKING_THICKNESS = 1.2
TOOTH_DEPTH = 0.9
TOOTH_TANGENTIAL_LENGTH = 0.80

WEDGE_CAPTURE_TEETH = 5
BACKING_INNER_RADIUS = REEL_PITCH_RADIUS - BACKING_THICKNESS / 2.0
BACKING_OUTER_RADIUS = REEL_PITCH_RADIUS + BACKING_THICKNESS / 2.0
DRUM_LAND_CLEARANCE = 0.05
DRUM_GROOVE_ROOT_CLEARANCE = 0.15
DRUM_GROOVE_AXIAL_CLEARANCE = 1.0
DRUM_OUTER_RADIUS = BACKING_INNER_RADIUS - DRUM_LAND_CLEARANCE
DRUM_ROOT_RADIUS = BACKING_INNER_RADIUS - TOOTH_DEPTH - DRUM_GROOVE_ROOT_CLEARANCE
POCKET_CLEARANCE = 0.06
CONTACT_TOLERANCE = 1e-6
CONTACT_WITNESS_TRAVEL = 0.02
REEL_WEDGE_BASE_RADIAL_DEPTH = 1.10
REEL_WEDGE_TAPER = 0.10
REEL_BACKING_SERVICE_CLEARANCE = 0.10
WEDGE_CELL_OVERLAP = 0.18
POCKET_FLOOR_THICKNESS = 0.80
REEL_LOAD_STOP_THICKNESS = 0.18
REEL_LOAD_STOP_RADIAL_DEPTH = 1.30
REEL_SERVICE_APPROACH = BELT_WIDTH + 2.0
SLIDER_WEDGE_X_SIZE = 1.70
SLIDER_WEDGE_RAIL_THICKNESS = 1.0
SLIDER_DATUM_THICKNESS = 0.80
SLIDER_SUPPORT_RAIL_THICKNESS = 1.0
SLIDER_APPROACH_ALLOWANCE = 3.0
SLIDER_LOAD_STOP_THICKNESS = 0.40
SLIDER_SUPPORT_Z_OVERHANG = 2.0

# Cartridge datums and honest above-pan stack (millimetres).
BASE_STRUCTURAL_UNDERSIDE_Z = 1.0
BASE_TOP_Z = 7.0
LOWER_THRUST_Z0 = BASE_TOP_Z
LOWER_THRUST_HEIGHT = 1.0
LOWER_RADIAL_Z0 = 9.0
LOWER_RADIAL_HEIGHT = 5.0
SPRING_Z0 = 15.0
SPRING_HEIGHT = 10.0
SPRING_WIRE_DIAMETER = 1.2
SPRING_CENTERLINE_RADIUS = 8.0
SPRING_TURNS = 4.0
REEL_Z0 = BELT_Z - BELT_WIDTH / 2.0
REEL_Z1 = BELT_Z + BELT_WIDTH / 2.0
UPPER_RADIAL_Z0 = 42.0
UPPER_RADIAL_HEIGHT = 4.0
TOP_CAP_Z0 = 46.0
TOP_CAP_HEIGHT = 3.0
OUTPUT_SHAFT_RADIUS = 3.0
BEARING_BORE_RADIUS = 3.1
HOUSING_OUTER_RADIUS = 21.0
HOUSING_INNER_RADIUS = 18.5
HOUSING_Z0 = BASE_TOP_Z
HOUSING_Z1 = TOP_CAP_Z0
STOP_Z0 = 26.2
STOP_HEIGHT = 2.0
STOP_LUG_RADIAL_MIN = 4.0
STOP_LUG_RADIAL_MAX = 9.0
STOP_LUG_TANGENTIAL_WIDTH = 2.0
DISTANCE_TOLERANCE = 1e-6
VOLUME_TOLERANCE = 1e-6


@dataclass(frozen=True)
class ReelEngagementReport:
    engaged_teeth: int
    occupied_grooves: int
    positive_penetration_volume: float


@dataclass(frozen=True)
class BeltRetentionReport:
    reel_wedge_captured_teeth: int
    slider_wedge_captured_teeth: int
    reel_wedge_at_datum: bool
    slider_wedge_at_datum: bool
    reel_working_direction_blocked: bool
    slider_working_direction_blocked: bool


@dataclass(frozen=True)
class BeltPathReport:
    centerline_length: float
    backing_entry_gap: float


@dataclass(frozen=True)
class SliderWedgeInsertionReport:
    insertion_direction: str
    samples: int
    max_positive_collision_volume: float
    final_at_datum: bool


@dataclass(frozen=True)
class TangentToothPitchReport:
    straight_nearest_center_offset: float
    wrapped_nearest_center_offset: float
    tangent_center_pitch: float
    total_tooth_solids: int


@dataclass(frozen=True)
class ReelWedgeInsertionReport:
    insertion_direction: str
    samples: int
    max_positive_collision_volume: float
    final_datum_distance: float


@dataclass(frozen=True)
class CartridgeStackReport:
    minimum_component_z: float
    minimum_above_base_top: float
    minimum_above_structural_underside: float
    lower_radial_clearance: float
    lower_radial_axial_overlap: float
    lower_axial_contact_distance: float
    lower_axial_positive_penetration_volume: float
    upper_radial_clearance: float
    upper_radial_axial_overlap: float
    spring_inner_anchor_distance: float
    spring_outer_anchor_distance: float
    spring_anchor_positive_penetration_volume: float
    top_cap_wedge_blocking_overlap_volume: float
    top_cap_removed_service_overlap_volume: float
    housing_outer_diameter: float


@dataclass(frozen=True)
class StopContactReport:
    travel: float
    keyboard_contact: bool
    trackpad_contact: bool
    keyboard_distance: float
    trackpad_distance: float
    keyboard_positive_penetration_volume: float
    trackpad_positive_penetration_volume: float


def _box(size: Tuple[float, float, float], center: Tuple[float, float, float]) -> Shape:
    return Box(*size, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(Location(center))


def _polar_box(
    radial_center: float,
    angle: float,
    radial_size: float,
    tangential_size: float,
    z_size: float = BELT_WIDTH,
    z_center: float = BELT_Z,
) -> Shape:
    shape = _box((radial_size, tangential_size, z_size), (0.0, 0.0, z_center))
    shape = shape.rotate(Axis.Z, degrees(angle))
    return shape.moved(Location((radial_center * cos(angle), radial_center * sin(angle), 0.0)))


def _label(shape: Shape, label: str) -> Shape:
    shape.label = label
    return shape


def _compound(label: str, children: Sequence[Shape]) -> Compound:
    result = Compound(label=label, children=list(children))
    result.label = label
    return result


def _phase(travel: float) -> float:
    """World-space tooth phase generated by the kinematic reel rotation."""
    return pose_state(travel).reel_angle_rad


def _clockwise_from_entry(angle: float) -> float:
    return (pi - angle) % (2.0 * pi)


def _wrap_angle(travel: float) -> float:
    return (INITIAL_WRAP_TEETH * BELT_PITCH + pose_state(travel).belt_feed) / REEL_PITCH_RADIUS


def _material_tooth_coordinates(travel: float) -> List[float]:
    """Signed pitch-line coordinates for every tooth in the purchased belt."""
    tooth_count = round(BELT_TOTAL_CENTERLINE / BELT_PITCH)
    first_center = -INITIAL_STRAIGHT_BELT_LENGTH + BELT_PITCH / 2.0
    feed = pose_state(travel).belt_feed
    return [first_center + index * BELT_PITCH + feed for index in range(tooth_count)]


def _arc_tooth_angles(travel: float) -> List[float]:
    # Half-open ownership is deterministic at the tangent: u == 0 belongs to
    # the reel side, while u < 0 remains on the straight segment.
    return [pi - u / REEL_PITCH_RADIUS for u in _material_tooth_coordinates(travel) if u >= 0.0]


def _arc_tooth(angle: float, clearance: float = 0.0) -> Shape:
    radial_center = BACKING_INNER_RADIUS - TOOTH_DEPTH / 2.0
    return _polar_box(
        radial_center,
        angle,
        TOOTH_DEPTH + 2.0 * clearance,
        TOOTH_TANGENTIAL_LENGTH + 2.0 * clearance,
        BELT_WIDTH + 2.0 * clearance,
    )


def _groove(angle: float) -> Shape:
    radial_center = (DRUM_ROOT_RADIUS + BACKING_INNER_RADIUS + POCKET_CLEARANCE) / 2.0
    return _polar_box(
        radial_center,
        angle,
        BACKING_INNER_RADIUS + POCKET_CLEARANCE - DRUM_ROOT_RADIUS,
        TOOTH_TANGENTIAL_LENGTH + 2.0 * POCKET_CLEARANCE,
        BELT_WIDTH + DRUM_GROOVE_AXIAL_CLEARANCE,
    )


def _all_groove_angles(travel: float) -> List[float]:
    pitch_angle = BELT_PITCH / REEL_PITCH_RADIUS
    phase = _phase(travel)
    return [pi - phase - (index + 0.5) * pitch_angle for index in range(REEL_TEETH)]


def _all_grooves(travel: float) -> List[Shape]:
    return [_groove(angle) for angle in _all_groove_angles(travel)]


def _reel_capture_angles(travel: float) -> List[float]:
    return _arc_tooth_angles(travel)[-WEDGE_CAPTURE_TEETH:]


def _reel_wedge_outer_cells(travel: float) -> List[Shape]:
    # Overlapping cells form a tapered/undercut strip around five tooth roots.
    cells = []
    angles = _reel_capture_angles(travel)
    for index, angle in enumerate(angles):
        taper = REEL_WEDGE_TAPER * index / max(1, len(angles) - 1)
        radial_depth = REEL_WEDGE_BASE_RADIAL_DEPTH + taper
        cells.append(
            _polar_box(
                BACKING_INNER_RADIUS - REEL_BACKING_SERVICE_CLEARANCE - radial_depth / 2.0,
                angle,
                radial_depth,
                BELT_PITCH + WEDGE_CELL_OVERLAP,
            )
        )
    return cells


def _reel_wedge_and_pockets(travel: float) -> Tuple[Shape, List[Shape]]:
    outer_cells = _reel_wedge_outer_cells(travel)
    wedge = outer_cells[0]
    for cell in outer_cells[1:]:
        wedge = wedge + cell
    pockets = [_arc_tooth(angle, POCKET_CLEARANCE) for angle in _reel_capture_angles(travel)]
    for pocket in pockets:
        wedge = wedge - pocket
    return _label(wedge, "reel_end_tooth_wedge"), pockets


def _reel_pocket_support(travel: float) -> Tuple[Shape, Shape]:
    angles = _reel_capture_angles(travel)
    mid_angle = sum(angles) / len(angles)
    angular_span = abs(angles[-1] - angles[0])
    tangent_length = REEL_PITCH_RADIUS * angular_span + BELT_PITCH + 2.0 * POCKET_CLEARANCE
    # The floor is connected radially to the drum and its top is the insertion datum.
    floor = _polar_box(
        (DRUM_ROOT_RADIUS + BACKING_INNER_RADIUS) / 2.0,
        mid_angle,
        BACKING_INNER_RADIUS - DRUM_ROOT_RADIUS + BACKING_THICKNESS / 2.0,
        tangent_length,
        POCKET_FLOOR_THICKNESS,
        BELT_Z - BELT_WIDTH / 2.0 - POCKET_FLOOR_THICKNESS / 2.0,
    )
    # A tangential shoulder at the loaded end makes belt tension self-tightening.
    stop_angle = angles[0]
    stop_thickness = REEL_LOAD_STOP_THICKNESS
    wedge_cell_length = BELT_PITCH + WEDGE_CELL_OVERLAP
    tangent_offset = (wedge_cell_length + stop_thickness) / 2.0
    radius = (
        BACKING_INNER_RADIUS
        - REEL_BACKING_SERVICE_CLEARANCE
        - REEL_WEDGE_BASE_RADIAL_DEPTH / 2.0
    )
    stop = _box((REEL_LOAD_STOP_RADIAL_DEPTH, stop_thickness, BELT_WIDTH), (0.0, 0.0, BELT_Z))
    stop = stop.rotate(Axis.Z, degrees(stop_angle))
    stop = stop.moved(
        Location(
            (
                radius * cos(stop_angle) - tangent_offset * sin(stop_angle),
                radius * sin(stop_angle) + tangent_offset * cos(stop_angle),
                0.0,
            )
        )
    )
    return floor, stop


def _reel_drum(travel: float) -> Shape:
    drum = Cylinder(
        DRUM_OUTER_RADIUS,
        BELT_WIDTH,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, BELT_Z - BELT_WIDTH / 2.0)))
    for groove in _all_grooves(travel):
        drum = drum - groove
    # A broad radial pocket replaces five ordinary lands at the service end.
    for cell in _reel_wedge_outer_cells(travel):
        drum = drum - cell
    shaft_bore = Cylinder(
        OUTPUT_SHAFT_RADIUS,
        BELT_WIDTH + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, REEL_Z0 - 1.0)))
    drum = drum - shaft_bore
    floor, stop = _reel_pocket_support(travel)
    drum = (drum + floor + stop) - shaft_bore
    return _label(drum, "reel_drum_41t")


def _pitch_line_edges(travel: float) -> Tuple[Edge, Edge]:
    z_min = BELT_Z - BELT_WIDTH / 2.0
    straight_length = pose_state(travel).free_belt_length
    end_angle = 180.0 - degrees(_wrap_angle(travel))
    straight = Edge.make_line(
        (-REEL_PITCH_RADIUS, -straight_length, z_min),
        (-REEL_PITCH_RADIUS, 0.0, z_min),
    )
    arc = Edge.make_circle(
        REEL_PITCH_RADIUS,
        Plane.XY.offset(z_min),
        180.0,
        end_angle,
        AngularDirection.CLOCKWISE,
    )
    return straight, arc


def _pitch_line_wire(travel: float) -> Wire:
    return Wire(list(_pitch_line_edges(travel)))


def _backing_solid(travel: float) -> Shape:
    z_min = BELT_Z - BELT_WIDTH / 2.0
    straight_length = pose_state(travel).free_belt_length
    end_angle = 180.0 - degrees(_wrap_angle(travel))

    def point(radius: float, angle_deg: float) -> Tuple[float, float, float]:
        angle = radians(angle_deg)
        return (radius * cos(angle), radius * sin(angle), z_min)

    edges = [
        Edge.make_line((-BACKING_OUTER_RADIUS, -straight_length, z_min), (-BACKING_OUTER_RADIUS, 0.0, z_min)),
        Edge.make_circle(
            BACKING_OUTER_RADIUS,
            Plane.XY.offset(z_min),
            180.0,
            end_angle,
            AngularDirection.CLOCKWISE,
        ),
        Edge.make_line(point(BACKING_OUTER_RADIUS, end_angle), point(BACKING_INNER_RADIUS, end_angle)),
        Edge.make_circle(
            BACKING_INNER_RADIUS,
            Plane.XY.offset(z_min),
            end_angle,
            180.0,
            AngularDirection.COUNTER_CLOCKWISE,
        ),
        Edge.make_line((-BACKING_INNER_RADIUS, 0.0, z_min), (-BACKING_INNER_RADIUS, -straight_length, z_min)),
        Edge.make_line(
            (-BACKING_INNER_RADIUS, -straight_length, z_min),
            (-BACKING_OUTER_RADIUS, -straight_length, z_min),
        ),
    ]
    backing = extrude(Face(Wire(edges)), amount=BELT_WIDTH, dir=(0.0, 0.0, 1.0))
    return _label(backing, "flexible_belt_backing")


def _straight_backing(travel: float) -> Shape:
    length = pose_state(travel).free_belt_length
    return _box(
        (BACKING_THICKNESS, length, BELT_WIDTH),
        (-REEL_PITCH_RADIUS, -length / 2.0, BELT_Z),
    )


def _straight_teeth(travel: float) -> List[Shape]:
    radial_center = -(BACKING_INNER_RADIUS - TOOTH_DEPTH / 2.0)
    return [
        _box((TOOTH_DEPTH, TOOTH_TANGENTIAL_LENGTH, BELT_WIDTH), (radial_center, u, BELT_Z))
        for u in _material_tooth_coordinates(travel)
        if u < 0.0
    ]


def _slider_capture_teeth(travel: float) -> List[Shape]:
    return _straight_teeth(travel)[-WEDGE_CAPTURE_TEETH:]


def _slider_wedge_and_pockets(travel: float) -> Tuple[Shape, List[Shape]]:
    teeth = _slider_capture_teeth(travel)
    centers = [tooth.bounding_box().center() for tooth in teeth]
    y_center = sum(center.Y for center in centers) / len(centers)
    y_span = max(center.Y for center in centers) - min(center.Y for center in centers) + BELT_PITCH
    # The wedge approaches from +X. Tooth slots break through its leading -X
    # face but stop before the +X back wall; integral Z rails remain above and
    # below the full-width belt teeth.
    wedge_x_min = -BACKING_INNER_RADIUS
    wedge_x_size = SLIDER_WEDGE_X_SIZE
    wedge = _box(
        (wedge_x_size, y_span, BELT_WIDTH + 2.0 * SLIDER_WEDGE_RAIL_THICKNESS),
        (wedge_x_min + wedge_x_size / 2.0, y_center, BELT_Z),
    )
    pockets = []
    for tooth in teeth:
        tooth_box = tooth.bounding_box()
        center = tooth_box.center()
        pocket_x_min = wedge_x_min - POCKET_CLEARANCE
        pocket_x_max = tooth_box.max.X + POCKET_CLEARANCE
        pocket = _box(
            (
                pocket_x_max - pocket_x_min,
                TOOTH_TANGENTIAL_LENGTH + 2 * POCKET_CLEARANCE,
                BELT_WIDTH + 2 * POCKET_CLEARANCE,
            ),
            ((pocket_x_min + pocket_x_max) / 2.0, center.Y, center.Z),
        )
        pockets.append(pocket)
        wedge = wedge - pocket
    return _label(wedge, "slider_end_tooth_wedge"), pockets


def _slider_support(travel: float) -> Tuple[Shape, Shape, Shape]:
    teeth = _slider_capture_teeth(travel)
    y_values = [tooth.bounding_box().center().Y for tooth in teeth]
    y_center = sum(y_values) / len(y_values)
    y_span = max(y_values) - min(y_values) + BELT_PITCH
    # Open +X approach corridor, left seating wall, upper/lower Z capture rails,
    # and a -Y load wall. No solid closes the transverse insertion path.
    wedge_x_min = -BACKING_INNER_RADIUS
    wedge_x_max = wedge_x_min + SLIDER_WEDGE_X_SIZE
    datum_wall = _box(
        (
            SLIDER_DATUM_THICKNESS,
            y_span + SLIDER_APPROACH_ALLOWANCE,
            BELT_WIDTH + 2.0 * SLIDER_SUPPORT_Z_OVERHANG,
        ),
        (wedge_x_min - SLIDER_DATUM_THICKNESS / 2.0, y_center, BELT_Z),
    )
    corridor_x_min = wedge_x_min - SLIDER_DATUM_THICKNESS
    corridor_x_max = wedge_x_max + SLIDER_APPROACH_ALLOWANCE
    corridor_x_center = (corridor_x_min + corridor_x_max) / 2.0
    corridor_x_size = corridor_x_max - corridor_x_min
    lower_rail = _box(
        (corridor_x_size, y_span + SLIDER_APPROACH_ALLOWANCE, SLIDER_SUPPORT_RAIL_THICKNESS),
        (
            corridor_x_center,
            y_center,
            BELT_Z
            - BELT_WIDTH / 2.0
            - SLIDER_WEDGE_RAIL_THICKNESS
            - SLIDER_SUPPORT_RAIL_THICKNESS / 2.0,
        ),
    )
    upper_rail = _box(
        (corridor_x_size, y_span + SLIDER_APPROACH_ALLOWANCE, SLIDER_SUPPORT_RAIL_THICKNESS),
        (
            corridor_x_center,
            y_center,
            BELT_Z
            + BELT_WIDTH / 2.0
            + SLIDER_WEDGE_RAIL_THICKNESS
            + SLIDER_SUPPORT_RAIL_THICKNESS / 2.0,
        ),
    )
    load_stop_y = y_center + y_span / 2.0 + SLIDER_LOAD_STOP_THICKNESS / 2.0
    load_stop = _box(
        (
            corridor_x_size,
            SLIDER_LOAD_STOP_THICKNESS,
            BELT_WIDTH + 2.0 * SLIDER_WEDGE_RAIL_THICKNESS,
        ),
        (corridor_x_center, load_stop_y, BELT_Z),
    )
    slider = datum_wall + lower_rail + upper_rail + load_stop
    return _label(slider, "screwless_belt_end_slider"), datum_wall, load_stop


def build_belt_system(travel: float) -> Compound:
    """Build the purchased-belt envelope and two removable tooth wedges."""
    pose_state(travel)  # public range validation and single source of truth
    backing = _backing_solid(travel)
    teeth = _compound(
        "flexible_belt_teeth",
        [*_straight_teeth(travel), *[_arc_tooth(angle) for angle in _arc_tooth_angles(travel)]],
    )
    reel_wedge, _ = _reel_wedge_and_pockets(travel)
    slider_wedge, _ = _slider_wedge_and_pockets(travel)
    children = [
        backing,
        teeth,
        _reel_drum(travel),
        reel_wedge,
        _slider_support(travel)[0],
        slider_wedge,
    ]
    return _compound(f"v4_serviceable_belt_system_t{travel:.3f}", children)


def reel_engagement_report(travel: float) -> ReelEngagementReport:
    """Count real tooth/groove BREP occupancy and positive land penetration."""
    pose_state(travel)
    capture_angles = set(_reel_capture_angles(travel))
    tooth_angles = [angle for angle in _arc_tooth_angles(travel) if angle not in capture_angles]
    grooves = _all_grooves(travel)
    drum = _reel_drum(travel)
    engaged = 0
    occupied = set()
    penetration = 0.0
    for angle in tooth_angles:
        tooth = _arc_tooth(angle)
        overlaps = [(tooth & groove).volume for groove in grooves]
        groove_index = max(range(len(overlaps)), key=overlaps.__getitem__)
        if overlaps[groove_index] >= tooth.volume * 0.95:
            engaged += 1
            occupied.add(groove_index)
        penetration += (tooth & drum).volume
    return ReelEngagementReport(engaged, len(occupied), penetration)


def belt_path_report(travel: float) -> BeltPathReport:
    """Measure the actual straight-plus-arc pitch-line BREP."""
    straight, arc = _pitch_line_edges(travel)
    wire = _pitch_line_wire(travel)
    return BeltPathReport(
        centerline_length=sum(edge.length for edge in wire.edges()),
        backing_entry_gap=straight.distance_to(arc),
    )


def tangent_tooth_pitch_report(travel: float) -> TangentToothPitchReport:
    """Measure tooth-center pitch across the fixed straight/arc tangent."""
    straight_teeth = _straight_teeth(travel)
    wrapped_teeth = [_arc_tooth(angle) for angle in _arc_tooth_angles(travel)]
    straight_offset = min(-tooth.bounding_box().center().Y for tooth in straight_teeth)
    wrapped_offset = min(
        REEL_PITCH_RADIUS
        * _clockwise_from_entry(
            atan2(tooth.bounding_box().center().Y, tooth.bounding_box().center().X)
        )
        for tooth in wrapped_teeth
    )
    return TangentToothPitchReport(
        straight_nearest_center_offset=straight_offset,
        wrapped_nearest_center_offset=wrapped_offset,
        tangent_center_pitch=straight_offset + wrapped_offset,
        total_tooth_solids=len(straight_teeth) + len(wrapped_teeth),
    )


def _reel_load_probes(angles: Sequence[float]) -> List[Shape]:
    probes = []
    probe_length = 0.20
    radial_center = BACKING_INNER_RADIUS - TOOTH_DEPTH / 2.0
    tangent_offset = TOOTH_TANGENTIAL_LENGTH / 2.0 + POCKET_CLEARANCE + probe_length / 2.0
    for angle in angles:
        probe = _box((TOOTH_DEPTH * 0.8, probe_length, BELT_WIDTH * 0.8), (0.0, 0.0, BELT_Z))
        probe = probe.rotate(Axis.Z, degrees(angle))
        probe = probe.moved(
            Location(
                (
                    radial_center * cos(angle) - tangent_offset * sin(angle),
                    radial_center * sin(angle) + tangent_offset * cos(angle),
                    0.0,
                )
            )
        )
        probes.append(probe)
    return probes


def _slider_load_probes(teeth: Sequence[Shape]) -> List[Shape]:
    probes = []
    probe_length = 0.20
    for tooth in teeth:
        bounds = tooth.bounding_box()
        probes.append(
            _box(
                (TOOTH_DEPTH * 0.8, probe_length, BELT_WIDTH * 0.8),
                (
                    bounds.center().X,
                    bounds.max.Y + POCKET_CLEARANCE + probe_length / 2.0,
                    BELT_Z,
                ),
            )
        )
    return probes


def _captured_tooth_count(
    teeth: Sequence[Shape],
    pockets: Sequence[Shape],
    wedge: Shape,
    load_probes: Sequence[Shape],
) -> int:
    """Count only occupied, collision-free cavities with real load-side lands."""
    captured = 0
    for tooth, pocket, probe in zip(teeth, pockets, load_probes):
        occupies_cut_cavity = (tooth & pocket).volume >= tooth.volume * 0.95
        no_wedge_penetration = (tooth & wedge).volume <= CONTACT_TOLERANCE
        load_land_volume = (probe & wedge).volume
        has_load_side_material = load_land_volume >= probe.volume * 0.60
        if occupies_cut_cavity and no_wedge_penetration and has_load_side_material:
            captured += 1
    return captured


def slider_wedge_insertion_report(samples: int = 7) -> SliderWedgeInsertionReport:
    """Sample the explicit +X to -X transverse service insertion."""
    if samples < 6:
        raise ValueError("at least six insertion samples are required")
    travel = 0.5
    wedge, _ = _slider_wedge_and_pockets(travel)
    teeth = _compound("slider_capture_teeth_check", _slider_capture_teeth(travel))
    backing = _straight_backing(travel)
    support, datum, _ = _slider_support(travel)
    approach_offset = SLIDER_APPROACH_ALLOWANCE
    max_collision = 0.0
    for index in range(samples):
        offset = approach_offset * (samples - 1 - index) / (samples - 1)
        posed_wedge = wedge.moved(Location((offset, 0.0, 0.0)))
        collision = (
            (posed_wedge & teeth).volume
            + (posed_wedge & backing).volume
            + (posed_wedge & support).volume
        )
        max_collision = max(max_collision, collision)
    return SliderWedgeInsertionReport(
        insertion_direction="+X toward -X",
        samples=samples,
        max_positive_collision_volume=max_collision,
        final_at_datum=_has_directional_contact(
            wedge, datum, (-CONTACT_WITNESS_TRAVEL, 0.0, 0.0)
        ),
    )


def reel_wedge_insertion_report(samples: int = 7) -> ReelWedgeInsertionReport:
    """Sample axial +Z-to-seated service insertion into the reel pocket."""
    if samples < 7:
        raise ValueError("at least seven insertion samples are required")
    travel = 0.5
    wedge, _ = _reel_wedge_and_pockets(travel)
    teeth = _compound(
        "reel_capture_teeth_check",
        [_arc_tooth(angle) for angle in _reel_capture_angles(travel)],
    )
    backing = _backing_solid(travel)
    drum = _reel_drum(travel)
    floor, load_stop = _reel_pocket_support(travel)
    max_collision = 0.0
    for index in range(samples):
        z_offset = REEL_SERVICE_APPROACH * (samples - 1 - index) / (samples - 1)
        posed_wedge = wedge.moved(Location((0.0, 0.0, z_offset)))
        collision = (
            (posed_wedge & teeth).volume
            + (posed_wedge & backing).volume
            + (posed_wedge & drum).volume
            + (posed_wedge & floor).volume
            + (posed_wedge & load_stop).volume
        )
        max_collision = max(max_collision, collision)
    return ReelWedgeInsertionReport(
        insertion_direction="+Z toward -Z",
        samples=samples,
        max_positive_collision_volume=max_collision,
        final_datum_distance=wedge.distance_to(floor),
    )


def _has_directional_contact(
    moving: Shape, fixed: Shape, witness_delta: Tuple[float, float, float]
) -> bool:
    """Require contact, no penetration, and material behind the contact face."""
    witness = moving.moved(Location(witness_delta))
    return (
        moving.distance_to(fixed) <= CONTACT_TOLERANCE
        and (moving & fixed).volume <= CONTACT_TOLERANCE
        and (witness & fixed).volume > CONTACT_TOLERANCE
    )


def belt_retention_report() -> BeltRetentionReport:
    """Measure nominal-pose tooth-pocket occupancy and physical seating faces."""
    travel = 0.5
    reel_teeth = [_arc_tooth(angle) for angle in _reel_capture_angles(travel)]
    reel_wedge, reel_pockets = _reel_wedge_and_pockets(travel)
    reel_floor, reel_stop = _reel_pocket_support(travel)
    slider_teeth = _slider_capture_teeth(travel)
    slider_wedge, slider_pockets = _slider_wedge_and_pockets(travel)
    _, slider_datum, slider_stop = _slider_support(travel)

    reel_stop_angle = _reel_capture_angles(travel)[0]
    reel_tangent_delta = (
        -CONTACT_WITNESS_TRAVEL * sin(reel_stop_angle),
        CONTACT_WITNESS_TRAVEL * cos(reel_stop_angle),
        0.0,
    )
    return BeltRetentionReport(
        reel_wedge_captured_teeth=_captured_tooth_count(
            reel_teeth,
            reel_pockets,
            reel_wedge,
            _reel_load_probes(_reel_capture_angles(travel)),
        ),
        slider_wedge_captured_teeth=_captured_tooth_count(
            slider_teeth,
            slider_pockets,
            slider_wedge,
            _slider_load_probes(slider_teeth),
        ),
        reel_wedge_at_datum=_has_directional_contact(
            reel_wedge, reel_floor, (0.0, 0.0, -CONTACT_WITNESS_TRAVEL)
        ),
        slider_wedge_at_datum=_has_directional_contact(
            slider_wedge, slider_datum, (-CONTACT_WITNESS_TRAVEL, 0.0, 0.0)
        ),
        reel_working_direction_blocked=_has_directional_contact(
            reel_wedge, reel_stop, reel_tangent_delta
        ),
        slider_working_direction_blocked=_has_directional_contact(
            slider_wedge, slider_stop, (0.0, CONTACT_WITNESS_TRAVEL, 0.0)
        ),
    )


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float, z0: float) -> Shape:
    outer = Cylinder(
        outer_radius, height, align=(Align.CENTER, Align.CENTER, Align.MIN)
    ).moved(Location((0.0, 0.0, z0)))
    bore = Cylinder(
        inner_radius, height + 2.0, align=(Align.CENTER, Align.CENTER, Align.MIN)
    ).moved(Location((0.0, 0.0, z0 - 1.0)))
    return outer - bore


def _lower_axial_thrust_interface() -> Shape:
    return _label(
        _annular_cylinder(8.0, BEARING_BORE_RADIUS, LOWER_THRUST_HEIGHT, LOWER_THRUST_Z0),
        "lower_axial_thrust_interface",
    )


def _lower_radial_bearing_seat() -> Shape:
    return _label(
        _annular_cylinder(8.0, BEARING_BORE_RADIUS, LOWER_RADIAL_HEIGHT, LOWER_RADIAL_Z0),
        "lower_radial_bearing_seat",
    )


def _upper_radial_bearing_seat() -> Shape:
    return _label(
        _annular_cylinder(9.0, BEARING_BORE_RADIUS, UPPER_RADIAL_HEIGHT, UPPER_RADIAL_Z0),
        "upper_radial_bearing_seat",
    )


def _vertical_output_shaft() -> Shape:
    shaft = Cylinder(
        OUTPUT_SHAFT_RADIUS,
        57.0 - (LOWER_THRUST_Z0 + LOWER_THRUST_HEIGHT),
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, LOWER_THRUST_Z0 + LOWER_THRUST_HEIGHT)))
    thrust_shoulder = Cylinder(
        5.0, 1.0, align=(Align.CENTER, Align.CENTER, Align.MIN)
    ).moved(Location((0.0, 0.0, LOWER_THRUST_Z0 + LOWER_THRUST_HEIGHT)))
    return _label(shaft + thrust_shoulder, "vertical_output_shaft")


def _return_spring() -> Shape:
    path = Edge.make_helix(
        SPRING_HEIGHT / SPRING_TURNS,
        SPRING_HEIGHT,
        SPRING_CENTERLINE_RADIUS,
        center=(0.0, 0.0, SPRING_Z0),
        normal=(0.0, 0.0, 1.0),
    )
    profile = Face(
        Wire.make_circle(
            SPRING_WIRE_DIAMETER / 2.0,
            Plane(path.position_at(0), z_dir=path.tangent_at(0)),
        )
    )
    coil = Solid.sweep(profile, path, is_frenet=True)
    inner_leg = _box(
        (4.4, SPRING_WIRE_DIAMETER, SPRING_WIRE_DIAMETER),
        (5.8, 0.0, SPRING_Z0),
    )
    outer_leg = _box(
        (10.0, SPRING_WIRE_DIAMETER, SPRING_WIRE_DIAMETER),
        (13.0, 0.0, SPRING_Z0 + SPRING_HEIGHT),
    )
    return _label(coil + inner_leg + outer_leg, "above_base_return_spring")


def _return_spring_inner_anchor() -> Shape:
    return _label(
        _box((1.2, 1.2, 1.2), (3.0, 0.0, SPRING_Z0)),
        "return_spring_inner_anchor",
    )


def _return_spring_outer_anchor() -> Shape:
    return _label(
        _box((1.5, 1.2, 1.2), (18.75, 0.0, SPRING_Z0 + SPRING_HEIGHT)),
        "return_spring_outer_anchor",
    )


def _fixed_cartridge_housing(sectioned: bool = False) -> Shape:
    housing = _annular_cylinder(
        HOUSING_OUTER_RADIUS,
        HOUSING_INNER_RADIUS,
        HOUSING_Z1 - HOUSING_Z0,
        HOUSING_Z0,
    )
    belt_entry = _box((5.0, 24.0, 16.0), (-13.0, -12.0, BELT_Z))
    housing = housing - belt_entry
    if sectioned:
        section_tool = _box(
            (
                HOUSING_OUTER_RADIUS + 2.0,
                2.0 * HOUSING_OUTER_RADIUS + 4.0,
                HOUSING_Z1 - HOUSING_Z0 + 2.0,
            ),
            ((HOUSING_OUTER_RADIUS + 2.0) / 2.0, 0.0, (HOUSING_Z0 + HOUSING_Z1) / 2.0),
        )
        housing = housing - section_tool
    return _label(housing, "fixed_cartridge_housing")


def _fixed_planar_belt_entry_guide() -> Shape:
    side = _box((0.7, 23.0, 14.0), (-14.15, -12.5, BELT_Z))
    lower = _box((2.8, 23.0, 0.5), (-12.75, -12.5, REEL_Z0 - 0.25))
    upper = _box((2.8, 23.0, 0.5), (-12.75, -12.5, REEL_Z1 + 0.25))
    return _label(side + lower + upper, "fixed_planar_belt_entry_guide")


def _removable_cartridge_top_cap() -> Shape:
    return _label(
        _annular_cylinder(HOUSING_OUTER_RADIUS, BEARING_BORE_RADIUS, TOP_CAP_HEIGHT, TOP_CAP_Z0),
        "removable_cartridge_top_cap",
    )


def _radial_stop_bar(angle_deg: float, label: str, tangent_offset: float = 0.0) -> Shape:
    radial_size = STOP_LUG_RADIAL_MAX - STOP_LUG_RADIAL_MIN
    bar = _box(
        (radial_size, STOP_LUG_TANGENTIAL_WIDTH, STOP_HEIGHT),
        (
            (STOP_LUG_RADIAL_MIN + STOP_LUG_RADIAL_MAX) / 2.0,
            tangent_offset,
            STOP_Z0 + STOP_HEIGHT / 2.0,
        ),
    ).rotate(Axis.Z, angle_deg)
    return _label(bar, label)


def _keyboard_hard_stop() -> Shape:
    return _radial_stop_bar(0.0, "independent_hard_stop_keyboard", -STOP_LUG_TANGENTIAL_WIDTH)


def _trackpad_hard_stop() -> Shape:
    return _radial_stop_bar(90.0, "independent_hard_stop_trackpad", STOP_LUG_TANGENTIAL_WIDTH)


def _shaft_rotating_stop_lug(travel: float) -> Shape:
    return _radial_stop_bar(pose_state(travel).reel_angle_deg, "shaft_rotating_stop_lug")


def _stack_parts(travel: float, sectioned: bool) -> List[Shape]:
    return [
        _lower_axial_thrust_interface(),
        _lower_radial_bearing_seat(),
        _return_spring(),
        _return_spring_inner_anchor(),
        _return_spring_outer_anchor(),
        _reel_drum(travel),
        _upper_radial_bearing_seat(),
        _vertical_output_shaft(),
        _fixed_cartridge_housing(sectioned),
        _fixed_planar_belt_entry_guide(),
        _removable_cartridge_top_cap(),
        _keyboard_hard_stop(),
        _trackpad_hard_stop(),
        _shaft_rotating_stop_lug(travel),
    ]


def build_cartridge(
    travel: float, sectioned: bool = False, include_belt: bool = True
) -> Compound:
    """Build a flat, separately selectable cartridge and optional Task 2 belt."""
    pose_state(travel)
    stack = _stack_parts(travel, sectioned)
    if include_belt:
        belt_children = [
            child for child in build_belt_system(travel).children if child.label != "reel_drum_41t"
        ]
        stack.extend(belt_children)
    return _compound(f"v4_reel_cartridge_t{travel:.3f}", stack)


def build_cartridge_exploded(travel: float = 0.5) -> Compound:
    """Explode the shared sectioned build while preserving its coaxial witness axis."""
    offsets = {
        "lower_axial_thrust_interface": -8.0,
        "lower_radial_bearing_seat": -4.0,
        "above_base_return_spring": 5.0,
        "return_spring_inner_anchor": 5.0,
        "return_spring_outer_anchor": 5.0,
        "reel_drum_41t": 10.0,
        "upper_radial_bearing_seat": 18.0,
        "removable_cartridge_top_cap": 26.0,
    }
    children = []
    for child in build_cartridge(travel, sectioned=True).children:
        dz = offsets.get(child.label, 0.0)
        children.append(child.moved(Location((0.0, 0.0, dz))) if dz else child)
    return _compound(f"v4_reel_cartridge_section_exploded_t{travel:.3f}", children)


def _axial_overlap(a: Shape, b: Shape) -> float:
    a_bounds = a.bounding_box()
    b_bounds = b.bounding_box()
    return max(0.0, min(a_bounds.max.Z, b_bounds.max.Z) - max(a_bounds.min.Z, b_bounds.min.Z))


def cartridge_stack_report() -> CartridgeStackReport:
    """Measure support, anchor, service, and envelope facts from nominal BREPs."""
    travel = 0.5
    parts = {part.label: part for part in _stack_parts(travel, False)}
    shaft = parts["vertical_output_shaft"]
    lower_radial = parts["lower_radial_bearing_seat"]
    upper_radial = parts["upper_radial_bearing_seat"]
    thrust = parts["lower_axial_thrust_interface"]
    spring = parts["above_base_return_spring"]
    inner_anchor = parts["return_spring_inner_anchor"]
    outer_anchor = parts["return_spring_outer_anchor"]
    cap = parts["removable_cartridge_top_cap"]
    housing = parts["fixed_cartridge_housing"]
    wedge, _ = _reel_wedge_and_pockets(travel)

    lower_probe = Cylinder(
        OUTPUT_SHAFT_RADIUS, LOWER_RADIAL_HEIGHT, align=(Align.CENTER, Align.CENTER, Align.MIN)
    ).moved(Location((0.0, 0.0, LOWER_RADIAL_Z0)))
    upper_probe = Cylinder(
        OUTPUT_SHAFT_RADIUS, UPPER_RADIAL_HEIGHT, align=(Align.CENTER, Align.CENTER, Align.MIN)
    ).moved(Location((0.0, 0.0, UPPER_RADIAL_Z0)))
    blocking_lift = TOP_CAP_Z0 - wedge.bounding_box().max.Z + CONTACT_WITNESS_TRAVEL
    blocking_wedge = wedge.moved(Location((0.0, 0.0, blocking_lift)))
    service_wedge = wedge.moved(Location((0.0, 0.0, REEL_SERVICE_APPROACH)))
    service_fixed = _compound("service_fixed_witness", [housing, upper_radial])
    housing_bounds = housing.bounding_box()
    minimum_component_z = min(part.bounding_box().min.Z for part in parts.values())

    return CartridgeStackReport(
        minimum_component_z=minimum_component_z,
        minimum_above_base_top=minimum_component_z - BASE_TOP_Z,
        minimum_above_structural_underside=minimum_component_z
        - BASE_STRUCTURAL_UNDERSIDE_Z,
        lower_radial_clearance=lower_probe.distance_to(lower_radial),
        lower_radial_axial_overlap=_axial_overlap(lower_probe, lower_radial),
        lower_axial_contact_distance=shaft.distance_to(thrust),
        lower_axial_positive_penetration_volume=(shaft & thrust).volume,
        upper_radial_clearance=upper_probe.distance_to(upper_radial),
        upper_radial_axial_overlap=_axial_overlap(upper_probe, upper_radial),
        spring_inner_anchor_distance=spring.distance_to(inner_anchor),
        spring_outer_anchor_distance=spring.distance_to(outer_anchor),
        spring_anchor_positive_penetration_volume=(spring & inner_anchor).volume
        + (spring & outer_anchor).volume,
        top_cap_wedge_blocking_overlap_volume=(blocking_wedge & cap).volume,
        top_cap_removed_service_overlap_volume=(service_wedge & service_fixed).volume,
        housing_outer_diameter=housing_bounds.max.X - housing_bounds.min.X,
    )


def stop_contact_report(travel: float) -> StopContactReport:
    """Measure exclusive 0/90 degree stop state without using the spring."""
    lug = _shaft_rotating_stop_lug(travel)
    keyboard = _keyboard_hard_stop()
    trackpad = _trackpad_hard_stop()
    keyboard_distance = lug.distance_to(keyboard)
    trackpad_distance = lug.distance_to(trackpad)
    keyboard_penetration = (lug & keyboard).volume
    trackpad_penetration = (lug & trackpad).volume
    return StopContactReport(
        travel=float(travel),
        keyboard_contact=keyboard_distance <= DISTANCE_TOLERANCE
        and keyboard_penetration <= VOLUME_TOLERANCE,
        trackpad_contact=trackpad_distance <= DISTANCE_TOLERANCE
        and trackpad_penetration <= VOLUME_TOLERANCE,
        keyboard_distance=keyboard_distance,
        trackpad_distance=trackpad_distance,
        keyboard_positive_penetration_volume=keyboard_penetration,
        trackpad_positive_penetration_volume=trackpad_penetration,
    )


def gen_step() -> Compound:
    """CAD-skill source envelope; Task 2 intentionally does not export STEP."""
    return build_belt_system(0.5)
