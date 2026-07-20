"""Rounded V4 open-frame base with transverse screwless guide supports.

Coordinates are millimetres.  The mechanism datum is the flat Z=7 plane; the
structural frame occupies Z=1..7 and the four primary pads define Z=-1.5.
"""

from dataclasses import dataclass
from build123d import (
    Align,
    Axis,
    Box,
    Color,
    Compound,
    Cone,
    Cylinder,
    Plane,
    Polygon,
    Pos,
    RectangleRounded,
    Shape,
    extrude,
)


BASE_WIDTH = 310.0
BASE_DEPTH = 225.0
BASE_CENTER_Y = 57.5
BASE_STRUCTURAL_UNDERSIDE_Z = 1.0
BASE_TOP_Z = 7.0
PRIMARY_TABLE_Z = -1.5
OUTER_CORNER_RADIUS = 10.0
MAIN_OPENING_RADIUS = 8.0
CARTRIDGE_CENTER = (120.0, 99.5)
CARTRIDGE_POD_RADIUS = 23.0

POSITIONS = ("left_front", "right_front", "left_rear", "right_rear")
RECEIVER_EXTERNAL_NAME = {
    "left_front": "front_left",
    "right_front": "front_right",
    "left_rear": "rear_left",
    "right_rear": "rear_right",
}
GUIDE_X = {"left": -146.0, "right": 146.0}
GUIDE_CENTER_Y = {"front": -21.79, "rear": 38.21}
GUIDE_LANDING_DEPTH = 44.0

PAD_CENTERS = {
    "front_left": (-142.0, -44.0),
    "front_right": (142.0, -44.0),
    "rear_left": (-142.0, 159.0),
    "rear_right": (142.0, 159.0),
}
PAD_RADIUS = 7.0
PAD_POCKET_RADIUS = 7.25
PAD_TOP_Z = 2.0

RECEIVER_Z0 = 4.6
RECEIVER_TOP_Z = BASE_TOP_Z
RECEIVER_LENGTH = 18.0
RECEIVER_HALF_WIDTH = 9.0
END_WALL_THICKNESS = 1.5
MALE_LENGTH = 15.0
MALE_Z0 = 4.75
MALE_TOP_Z = 6.55
MALE_BOTTOM_HALF_WIDTH = 7.0
MALE_TOP_HALF_WIDTH = 5.0
SLIDE_CLEARANCE = 0.2
TAB_OFFSET_Y = 6.0
SNAP_WINDOW_LENGTH = 8.0
SNAP_WINDOW_DEPTH = 4.5
SNAP_TAB_LENGTH = 7.0
SNAP_TAB_DEPTH = 2.0
OPERATING_Y_WITNESS_MOVE = 0.45
DETAIL_DISENGAGEMENT_CLEARANCE = 1.0
DETAIL_WITHDRAWAL = RECEIVER_LENGTH + DETAIL_DISENGAGEMENT_CLEARANCE
SWEEP_WITHDRAWALS = (DETAIL_WITHDRAWAL, 16.0, 12.0, 8.0, 4.0, 2.0, 1.0, 0.0)

BASE_GRAPHITE = Color(0.17, 0.19, 0.22)
RECEIVER_BLUE = Color(0.12, 0.40, 0.66)
SUPPORT_TEAL = Color(0.08, 0.66, 0.59)
PAD_BLACK = Color(0.04, 0.05, 0.06)
WITNESS_ORANGE = Color(0.96, 0.35, 0.08)


def _label(shape: Shape, name: str, color: Color) -> Shape:
    shape.label = name
    shape.color = color
    return shape


def _box(width, depth, height, x, y, z0):
    return Box(width, depth, height, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Pos(x, y, z0)
    )


def _rounded_prism(width, depth, radius, height, x, y, z0):
    face = RectangleRounded(width, depth, radius)
    return extrude(face, height).moved(Pos(x, y, z0))


def _receiver_parts(position: str):
    """Return load flanks, explicit snap ramp, end wall, and release window."""
    side, row = position.split("_")
    direction = 1.0 if side == "left" else -1.0
    outer_x = -155.0 if side == "left" else 155.0
    inner_x = outer_x + direction * RECEIVER_LENGTH
    y = GUIDE_CENTER_Y[row]

    # Female dovetail flanks: the inward-facing surfaces narrow toward Z=7.
    left_section = Plane.YZ * Polygon(
        (0.0, 0.0),
        (RECEIVER_HALF_WIDTH - MALE_BOTTOM_HALF_WIDTH - SLIDE_CLEARANCE, 0.0),
        (RECEIVER_HALF_WIDTH - MALE_TOP_HALF_WIDTH - SLIDE_CLEARANCE, RECEIVER_TOP_Z - RECEIVER_Z0),
        (0.0, RECEIVER_TOP_Z - RECEIVER_Z0),
        align=(Align.MIN, Align.MIN),
    )
    right_section = Plane.YZ * Polygon(
        (MALE_BOTTOM_HALF_WIDTH - MALE_TOP_HALF_WIDTH, 0.0),
        (RECEIVER_HALF_WIDTH - MALE_TOP_HALF_WIDTH - SLIDE_CLEARANCE, 0.0),
        (RECEIVER_HALF_WIDTH - MALE_TOP_HALF_WIDTH - SLIDE_CLEARANCE, RECEIVER_TOP_Z - RECEIVER_Z0),
        (0.0, RECEIVER_TOP_Z - RECEIVER_Z0),
        align=(Align.MIN, Align.MIN),
    )
    x0 = min(outer_x, inner_x)
    left_flank = extrude(left_section, RECEIVER_LENGTH, dir=(1.0, 0.0, 0.0)).moved(
        Pos(x0, y - RECEIVER_HALF_WIDTH, RECEIVER_Z0)
    )
    right_flank = extrude(right_section, RECEIVER_LENGTH, dir=(1.0, 0.0, 0.0)).moved(
        Pos(x0, y + MALE_TOP_HALF_WIDTH + SLIDE_CLEARANCE, RECEIVER_Z0)
    )

    end_center_x = inner_x - direction * END_WALL_THICKNESS / 2.0
    end_wall = _box(
        END_WALL_THICKNESS,
        2.0 * RECEIVER_HALF_WIDTH,
        RECEIVER_TOP_Z - RECEIVER_Z0,
        end_center_x,
        y,
        RECEIVER_Z0,
    )

    # The access window is a real top notch through the positive-Y capture
    # flank, aligned with the support's offset cantilever tab.
    window_x = outer_x + direction * 5.0
    window = _box(
        SNAP_WINDOW_LENGTH,
        SNAP_WINDOW_DEPTH,
        1.6,
        window_x,
        y + TAB_OFFSET_Y,
        5.9,
    )
    snap_ramp = right_flank - window
    return left_flank, snap_ramp, end_wall, window


def _receiver_geometry(position: str):
    """Return receiver, its solid end datum, and release-window witness."""
    left_flank, snap_ramp, end_wall, window = _receiver_parts(position)
    external_name = RECEIVER_EXTERNAL_NAME[position]
    receiver = left_flank + snap_ramp + end_wall
    return (
        _label(receiver, f"female_slide_receiver_{external_name}", RECEIVER_BLUE),
        _label(end_wall, f"receiver_end_wall_{position}", RECEIVER_BLUE),
        _label(window, f"receiver_release_window_{position}", WITNESS_ORANGE),
    )


def _receiver_channel_tool(position: str):
    side, row = position.split("_")
    direction = 1.0 if side == "left" else -1.0
    outer_x = -155.5 if side == "left" else 155.5
    inner_x = outer_x + direction * (RECEIVER_LENGTH - END_WALL_THICKNESS + 0.5)
    return _box(
        abs(inner_x - outer_x),
        16.2,
        BASE_TOP_Z - RECEIVER_Z0 + 0.1,
        (outer_x + inner_x) / 2.0,
        GUIDE_CENTER_Y[row] + 0.4,
        RECEIVER_Z0,
    )


def _main_base():
    frame = _rounded_prism(
        BASE_WIDTH,
        BASE_DEPTH,
        OUTER_CORNER_RADIUS,
        BASE_TOP_Z - BASE_STRUCTURAL_UNDERSIDE_Z,
        0.0,
        BASE_CENTER_Y,
        BASE_STRUCTURAL_UNDERSIDE_Z,
    )
    openings = (
        _rounded_prism(250.0, 40.0, MAIN_OPENING_RADIUS, 8.0, 0.0, -20.0, 0.5),
        _rounded_prism(250.0, 42.0, MAIN_OPENING_RADIUS, 8.0, 0.0, 40.0, 0.5),
        _rounded_prism(210.0, 60.0, MAIN_OPENING_RADIUS, 8.0, -20.0, 115.0, 0.5),
    )
    for opening in openings:
        frame = frame - opening

    # The pod guarantees an unbroken OD42 cartridge landing and blends into
    # the retained right-side load path without changing the housing datum.
    pod = Cylinder(
        CARTRIDGE_POD_RADIUS,
        BASE_TOP_Z - BASE_STRUCTURAL_UNDERSIDE_Z,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(*CARTRIDGE_CENTER, BASE_STRUCTURAL_UNDERSIDE_Z))
    frame = frame + pod

    for position in POSITIONS:
        frame = frame - _receiver_channel_tool(position)
    for x, y in PAD_CENTERS.values():
        pocket = Cylinder(
            PAD_POCKET_RADIUS,
            1.15,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Pos(x, y, BASE_STRUCTURAL_UNDERSIDE_Z))
        frame = frame - pocket
    return _label(frame, "rounded_low_profile_base", BASE_GRAPHITE)


def _primary_pad(name: str, center):
    x, y = center
    pad = Cylinder(
        PAD_RADIUS,
        PAD_TOP_Z - PRIMARY_TABLE_Z,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(x, y, PRIMARY_TABLE_Z))
    return _label(pad, f"primary_tpu_pad_{name}", PAD_BLACK)


def _support_components(position: str):
    """Return complete support, rigid load body/root, flexible blade, and root.

    Keeping these actual BREP components available lets reports distinguish
    rigid collision from the intentional elastic snap-ramp contact.
    """
    side, row = position.split("_")
    direction = 1.0 if side == "left" else -1.0
    outer_x = -155.0 if side == "left" else 155.0
    inner_face = outer_x + direction * (RECEIVER_LENGTH - END_WALL_THICKNESS)
    if side == "left":
        x0 = inner_face - MALE_LENGTH
    else:
        x0 = inner_face

    y = GUIDE_CENTER_Y[row]
    section = Plane.YZ * Polygon(
        (0.0, 0.0),
        (2.0 * MALE_BOTTOM_HALF_WIDTH, 0.0),
        (MALE_BOTTOM_HALF_WIDTH + MALE_TOP_HALF_WIDTH, MALE_TOP_Z - MALE_Z0),
        (MALE_BOTTOM_HALF_WIDTH - MALE_TOP_HALF_WIDTH, MALE_TOP_Z - MALE_Z0),
        align=(Align.MIN, Align.MIN),
    )
    male = extrude(section, MALE_LENGTH, dir=(1.0, 0.0, 0.0)).moved(
        Pos(x0, y - MALE_BOTTOM_HALF_WIDTH, MALE_Z0)
    )
    guide_x = GUIDE_X[side]
    stem = _box(10.0, 8.0, 0.65, guide_x, y, MALE_TOP_Z)
    landing = _box(14.0, GUIDE_LANDING_DEPTH, 3.0, guide_x, y, BASE_TOP_Z)

    # The root stays within the male-foot clearance envelope and therefore is
    # rigid.  The thin blade overlaps it and flexes only while crossing the
    # explicit positive-Y snap ramp; at full insertion it drops into the
    # release window without positive-volume interference.
    tab_center_x = outer_x + direction * 5.0
    root_center_x = tab_center_x + direction * 2.5
    tab_root = _box(3.0, 1.0, 0.45, root_center_x, y + 4.7, 6.05)
    tab_blade = _box(
        SNAP_TAB_LENGTH,
        SNAP_TAB_DEPTH,
        0.55,
        tab_center_x,
        y + TAB_OFFSET_Y,
        6.1,
    )
    release_access = _box(
        SNAP_WINDOW_LENGTH + 0.2,
        SNAP_WINDOW_DEPTH + 0.2,
        3.5,
        tab_center_x,
        y + TAB_OFFSET_Y,
        6.8,
    )
    rigid_body_and_root = male + stem + (landing - release_access) + tab_root
    support = rigid_body_and_root + tab_blade
    return (
        _label(support, f"removable_guide_support_{position}", SUPPORT_TEAL),
        rigid_body_and_root,
        tab_blade,
        tab_root,
        male,
    )


def _installed_support(position: str):
    return _support_components(position)[0]


def build_low_profile_base() -> Compound:
    """Return the structural base, four pads, and four selectable receivers."""
    children = [_main_base()]
    children.extend(_primary_pad(name, center) for name, center in PAD_CENTERS.items())
    children.extend(_receiver_geometry(position)[0] for position in POSITIONS)
    return Compound(label="v4_rounded_low_profile_base", children=children)


def build_supports(installed: bool = True) -> Compound:
    """Return four one-piece supports, installed or staged just outside entry."""
    supports = []
    for position in POSITIONS:
        support = _installed_support(position)
        if not installed:
            side = position.split("_")[0]
            support = support.moved(
                Pos(
                    -DETAIL_WITHDRAWAL if side == "left" else DETAIL_WITHDRAWAL,
                    0.0,
                    0.0,
                )
            )
            support.label = f"removable_guide_support_{position}"
            support.color = SUPPORT_TEAL
        supports.append(support)
    return Compound(label="v4_screwless_guide_supports", children=supports)


def build_mount_assembly(installed: bool = True) -> Compound:
    base = build_low_profile_base()
    supports = build_supports(installed=installed)
    return Compound(
        label="v4_low_profile_screwless_mount_assembly",
        children=[*base.children, *supports.children],
    )


@dataclass(frozen=True)
class MountReport:
    receiver_count: int
    all_at_end_datum: bool
    all_vertically_captured: bool
    all_laterally_captured: bool
    snap_tabs_clear_primary_load_path: bool
    left_insertion_direction: str
    right_insertion_direction: str
    end_datum_gaps: tuple
    vertical_witness_overlap_volumes: tuple
    lateral_witness_overlap_volumes: tuple
    operating_y_rigid_witness_overlap_volumes: tuple
    operating_y_snap_overlap_volumes: tuple
    installed_support_receiver_overlap_volumes: tuple


def mount_report() -> MountReport:
    base_parts = {part.label: part for part in build_low_profile_base().children}
    end_gaps = []
    vertical_overlaps = []
    lateral_overlaps = []
    operating_y_rigid_overlaps = []
    operating_y_snap_overlaps = []
    installed_overlaps = []
    directions = {}
    for position in POSITIONS:
        receiver, end_wall, _ = _receiver_geometry(position)
        support, rigid_body_and_root, tab_blade, tab_root, male = _support_components(position)
        end_gaps.append(support.distance_to(end_wall))
        installed_overlaps.append((support & receiver).volume)
        vertical_overlaps.append(
            (rigid_body_and_root.moved(Pos(0, 0, 0.45)) & receiver).volume
        )
        lateral_overlaps.append(
            max(
                (rigid_body_and_root.moved(Pos(0, -OPERATING_Y_WITNESS_MOVE, 0)) & receiver).volume,
                (rigid_body_and_root.moved(Pos(0, OPERATING_Y_WITNESS_MOVE, 0)) & receiver).volume,
            )
        )
        snap_with_root = tab_blade + tab_root
        rigid_y_contact = max(
            (male.moved(Pos(0, -OPERATING_Y_WITNESS_MOVE, 0)) & receiver).volume,
            (male.moved(Pos(0, OPERATING_Y_WITNESS_MOVE, 0)) & receiver).volume,
        )
        snap_y_contact = max(
            (snap_with_root.moved(Pos(0, -OPERATING_Y_WITNESS_MOVE, 0)) & receiver).volume,
            (snap_with_root.moved(Pos(0, OPERATING_Y_WITNESS_MOVE, 0)) & receiver).volume,
        )
        operating_y_rigid_overlaps.append(rigid_y_contact)
        operating_y_snap_overlaps.append(snap_y_contact)
        side = position.split("_")[0]
        entry_x = receiver.bounding_box().min.X if side == "left" else receiver.bounding_box().max.X
        datum_x = end_wall.bounding_box().min.X if side == "left" else end_wall.bounding_box().max.X
        directions[side] = "+X" if datum_x > entry_x else "-X"

    receiver_count = sum(
        label.startswith("female_slide_receiver_") for label in base_parts
    )
    return MountReport(
        receiver_count=receiver_count,
        all_at_end_datum=all(gap < 0.02 for gap in end_gaps),
        all_vertically_captured=all(volume > 0.01 for volume in vertical_overlaps),
        all_laterally_captured=all(volume > 0.01 for volume in lateral_overlaps),
        snap_tabs_clear_primary_load_path=(
            all(volume > 0.01 for volume in operating_y_rigid_overlaps)
            and all(volume < 1e-7 for volume in operating_y_snap_overlaps)
        ),
        left_insertion_direction=directions["left"],
        right_insertion_direction=directions["right"],
        end_datum_gaps=tuple(end_gaps),
        vertical_witness_overlap_volumes=tuple(vertical_overlaps),
        lateral_witness_overlap_volumes=tuple(lateral_overlaps),
        operating_y_rigid_witness_overlap_volumes=tuple(operating_y_rigid_overlaps),
        operating_y_snap_overlap_volumes=tuple(operating_y_snap_overlaps),
        installed_support_receiver_overlap_volumes=tuple(installed_overlaps),
    )


@dataclass(frozen=True)
class InsertionSweepReport:
    positions: tuple
    all_approaches_clear: bool
    all_finish_at_end_wall: bool
    maximum_unintended_overlap_volumes: tuple
    sample_withdrawals: tuple
    maximum_rigid_interference_volumes: tuple
    maximum_designed_elastic_tab_contact_volumes: tuple
    installed_total_overlap_volumes: tuple
    fully_clear_start_separations: tuple


def insertion_sweep_report() -> InsertionSweepReport:
    frame = _main_base()
    rigid_maxima = []
    elastic_maxima = []
    installed_overlaps = []
    start_separations = []
    finishes = []
    for position in POSITIONS:
        side = position.split("_")[0]
        direction = 1.0 if side == "left" else -1.0
        support, rigid_body_and_root, tab_blade, _, _ = _support_components(position)
        receiver, end_wall, _ = _receiver_geometry(position)
        _, snap_ramp, _, _ = _receiver_parts(position)
        rigid_overlaps = []
        elastic_contacts = []
        for withdrawal in SWEEP_WITHDRAWALS:
            move = Pos(-direction * withdrawal, 0.0, 0.0)
            staged_rigid = rigid_body_and_root.moved(move)
            staged_tab = tab_blade.moved(move)
            rigid_overlaps.append(
                (staged_rigid & frame).volume + (staged_rigid & receiver).volume
            )
            elastic_contacts.append((staged_tab & snap_ramp).volume)
        rigid_maxima.append(max(rigid_overlaps))
        elastic_maxima.append(max(elastic_contacts))
        installed_overlaps.append((support & receiver).volume)
        staged_start = support.moved(Pos(-direction * SWEEP_WITHDRAWALS[0], 0.0, 0.0))
        start_separations.append(staged_start.distance_to(receiver))
        finishes.append(support.distance_to(end_wall) < 0.02)
    return InsertionSweepReport(
        positions=POSITIONS,
        all_approaches_clear=(
            all(volume < 1e-6 for volume in rigid_maxima)
            and all(volume < 1e-6 for volume in installed_overlaps)
            and all(distance >= DETAIL_DISENGAGEMENT_CLEARANCE for distance in start_separations)
        ),
        all_finish_at_end_wall=all(finishes),
        maximum_unintended_overlap_volumes=tuple(rigid_maxima),
        sample_withdrawals=SWEEP_WITHDRAWALS,
        maximum_rigid_interference_volumes=tuple(rigid_maxima),
        maximum_designed_elastic_tab_contact_volumes=tuple(elastic_maxima),
        installed_total_overlap_volumes=tuple(installed_overlaps),
        fully_clear_start_separations=tuple(start_separations),
    )


def build_mount_detail() -> Compound:
    """Return a derived left-front coupon/detail; no source geometry is copied."""
    position = "left_front"
    base = _main_base()
    receiver, end_wall, window = _receiver_geometry(position)
    coupon_tool = _box(38.0, 60.0, 12.0, -142.0, GUIDE_CENTER_Y["front"], 0.0)
    coupon = _label(base & coupon_tool, "receiver_base_coupon", BASE_GRAPHITE)
    installed = _installed_support(position)
    installed.label = "installed_guide_support_foot"
    exploded = _installed_support(position).moved(Pos(-DETAIL_WITHDRAWAL, 0.0, 0.0))
    exploded.label = "exploded_guide_support_foot"
    exploded.color = SUPPORT_TEAL
    arrow_shaft = Cylinder(
        1.0, 18.0, align=(Align.CENTER, Align.CENTER, Align.MIN)
    ).rotate(Axis.Y, 90.0).moved(Pos(-169.0, GUIDE_CENTER_Y["front"] - 13.0, 9.0))
    arrow_head = Cone(2.5, 0.0, 5.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).rotate(
        Axis.Y, 90.0
    ).moved(Pos(-151.0, GUIDE_CENTER_Y["front"] - 13.0, 9.0))
    arrow = _label(arrow_shaft + arrow_head, "insertion_direction_witness_plus_x", WITNESS_ORANGE)
    end_wall.label = "solid_inward_end_wall_datum"
    window.label = "top_access_release_window_witness"
    return Compound(
        label="v4_screwless_guide_mount_detail",
        children=[coupon, receiver, installed, exploded, arrow, end_wall, window],
    )
