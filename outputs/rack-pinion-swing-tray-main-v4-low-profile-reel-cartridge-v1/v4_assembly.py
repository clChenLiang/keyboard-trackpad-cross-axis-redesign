"""Complete V4 keyboard/trackpad mechanism composed from approved subsystems.

The V3 moving trays and carriage hardware stay in world coordinates.  Task 2
and Task 3 geometry is authored around the reel origin and is translated to
``CARTRIDGE_CENTER`` exactly once while it is promoted to top-level children.
"""

from dataclasses import dataclass
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from typing import Iterable, Tuple

from build123d import Align, Box, Color, Compound, Location, Pos, Shape

from v4_kinematics import pose_state
from v4_reel_cartridge import (
    BeltEndSliderDatumReport,
    belt_end_slider_datum_report,
    build_belt_system,
    build_cartridge,
)
from v4_screwless_base import (
    POSITIONS,
    RECEIVER_BASE_INTEGRATION_ALLOWLIST,
    RECEIVER_EXTERNAL_NAME,
    V3_GUIDE_LABELS_TO_FILTER,
    build_low_profile_base,
    build_removable_guide_assembly,
)


CARTRIDGE_CENTER = (120.0, 99.5, 0.0)
VOLUME_TOLERANCE = 1e-6
GUIDE_BOOLEAN_NOISE_VOLUME = 0.05

_V3_PATH = (
    Path(__file__).resolve().parent.parent
    / "rack-pinion-swing-tray-main-v3"
    / "rack_pinion_swing_main_v3.py"
)
_V3_SPEC = spec_from_file_location("v4_assembly_v3_source", _V3_PATH)
v3 = module_from_spec(_V3_SPEC)
_V3_SPEC.loader.exec_module(v3)

_V3_MOVING_LABELS = {
    "keyboard_tray_continuous",
    "eccentric_horizontal_swing_arm",
    "magic_trackpad_small_tray",
    *(f"keyboard_carriage_{position}" for position in POSITIONS),
    *(f"carriage_tray_mount_ear_{position}" for position in POSITIONS),
    *(f"keyboard_tray_mount_bushing_{position}" for position in POSITIONS),
}

_BASE_LABELS = {
    "rounded_low_profile_base",
    *(f"primary_tpu_pad_{name}" for name in RECEIVER_EXTERNAL_NAME.values()),
    *(f"female_slide_receiver_{name}" for name in RECEIVER_EXTERNAL_NAME.values()),
}
_GUIDE_LABELS = {f"removable_keyboard_guide_{position}" for position in POSITIONS}
_BELT_LABELS = {
    "flexible_belt_backing",
    "flexible_belt_teeth",
    "reel_drum_41t",
    "reel_end_tooth_wedge",
    "screwless_belt_end_slider",
    "slider_end_tooth_wedge",
}
_CARTRIDGE_LABELS = {
    "lower_axial_thrust_interface",
    "lower_radial_bearing_seat",
    "above_base_return_spring",
    "return_spring_inner_anchor",
    "return_spring_outer_anchor",
    "removable_reel_axial_retainer",
    "upper_radial_bearing_seat",
    "vertical_output_shaft",
    "fixed_cartridge_housing",
    "fixed_planar_belt_entry_guide",
    "removable_cartridge_top_cap",
    "independent_hard_stop_keyboard",
    "independent_hard_stop_trackpad",
    "shaft_rotating_stop_lug",
}
_DRIVE_LABELS = {
    "keyboard_short_drive_tongue",
    "floating_z_drive_fork",
    "belt_slider_drive_adapter",
}

REQUIRED_LABELS = frozenset(
    _BASE_LABELS
    | _GUIDE_LABELS
    | _BELT_LABELS
    | _CARTRIDGE_LABELS
    | _V3_MOVING_LABELS
    | _DRIVE_LABELS
)

REMOVED_LABELS = frozenset(
    {
        "continuous_main_base",
        "replaceable_tpu_feet_6x",
        "keyboard_right_straight_rack",
        "module_1_26t_pinion",
        "rack_to_keyboard_rigid_transition",
        "fixed_protective_cylinder",
        "fixed_cylinder_removable_top_cover",
        "cylinder_top_cover_fastener_interface",
        "lower_axial_thrust_interface_v3",
        "lower_radial_bearing_seat_v3",
        "upper_radial_bearing_seat_v3",
        "vertical_drive_shaft",
        "vertical_drive_shaft_integrated_return",
        "shaft_rotating_stop_lug_v3",
        "independent_hard_stop_keyboard_v3",
        "independent_hard_stop_trackpad_v3",
        *V3_GUIDE_LABELS_TO_FILTER,
        *(f"replaceable_tpu_foot_{index}" for index in range(1, 7)),
    }
)

# Every suppression is one named physical seam.  There are no category-wide
# exclusions.  The three overlaps below are deliberate fused-load interfaces;
# receiver seams and base/receiver seams are designed zero-gap contacts.
INTENDED_CONTACT_ALLOWLIST = frozenset(
    {
        ("keyboard_tray_continuous", "keyboard_short_drive_tongue"),
        ("keyboard_short_drive_tongue", "floating_z_drive_fork"),
        ("screwless_belt_end_slider", "belt_slider_drive_adapter"),
        ("flexible_belt_backing", "fixed_planar_belt_entry_guide"),
        ("flexible_belt_teeth", "fixed_planar_belt_entry_guide"),
        *RECEIVER_BASE_INTEGRATION_ALLOWLIST,
        *(
            ("rounded_low_profile_base", f"removable_keyboard_guide_{position}")
            for position in POSITIONS
        ),
        *(
            (
                f"female_slide_receiver_{RECEIVER_EXTERNAL_NAME[position]}",
                f"removable_keyboard_guide_{position}",
            )
            for position in POSITIONS
        ),
    }
)

TONGUE_RED = Color(0.86, 0.25, 0.12)
FORK_GOLD = Color(0.96, 0.63, 0.08)
ADAPTER_CYAN = Color(0.08, 0.72, 0.78)


def _label(shape: Shape, label: str, color: Color) -> Shape:
    shape.label = label
    shape.color = color
    return shape


def _box(width: float, depth: float, height: float, x: float, y: float, z0: float) -> Shape:
    return Box(
        width,
        depth,
        height,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(x, y, z0))


def _world_children(group: Compound) -> list[Shape]:
    location = Location(CARTRIDGE_CENTER)
    children = []
    for source in group.children:
        child = source.moved(location)
        child.label = source.label
        child.color = source.color
        children.append(child)
    return children


def _keyboard_rear_y(travel: float) -> float:
    state = pose_state(travel)
    return v3.v2.v1.KEYBOARD_TRAY_DEPTH / 2.0 + state.keyboard_dy


def _keyboard_z0(travel: float) -> float:
    return v3.v2.v1.KEYBOARD_TRAY_Z0 + pose_state(travel).keyboard_dz


def _drive_centers(
    travel: float, datum: BeltEndSliderDatumReport
) -> tuple[float, float, float]:
    adapter_x = CARTRIDGE_CENTER[0] + datum.slider_center_x - 4.70
    adapter_y = CARTRIDGE_CENTER[1] + datum.slider_center_y
    fork_y = _keyboard_rear_y(travel) + 0.65
    return adapter_x, adapter_y, fork_y


def _keyboard_short_drive_tongue(travel: float) -> Shape:
    rear_y = _keyboard_rear_y(travel)
    tongue = _box(12.0, 0.90, 3.50, 108.0, rear_y - 0.25, _keyboard_z0(travel))
    return _label(tongue, "keyboard_short_drive_tongue", TONGUE_RED)


def _belt_slider_drive_adapter(
    travel: float, datum: BeltEndSliderDatumReport
) -> Shape:
    adapter_x, adapter_y, _ = _drive_centers(travel, datum)
    # The bridge enters the existing slider by 0.75 mm.  Its narrow key stays
    # at constant Z and is the only surface captured by the floating fork.
    bridge = _box(3.50, 2.8, 6.0, adapter_x + 0.95, adapter_y, 32.0)
    key = _box(1.60, 3.0, 12.0, adapter_x, adapter_y, 29.0)
    return _label(bridge + key, "belt_slider_drive_adapter", ADAPTER_CYAN)


def _floating_z_drive_fork(
    travel: float, datum: BeltEndSliderDatumReport
) -> Shape:
    adapter_x, _, fork_y = _drive_centers(travel, datum)
    z0 = _keyboard_z0(travel) - 19.0
    height = 22.0
    key_half_depth = 1.50
    y_clearance = 0.10
    rail_depth = 1.0
    rail_offset = key_half_depth + y_clearance + rail_depth / 2.0
    front_rail = _box(2.60, rail_depth, height, adapter_x, fork_y - rail_offset, z0)
    rear_rail = _box(2.60, rail_depth, height, adapter_x, fork_y + rail_offset, z0)
    web = _box(2.0, 2.0 * (rail_offset + rail_depth / 2.0), height, adapter_x - 2.30, fork_y, z0)
    # The full-height web overlaps the tongue directly at its top edge; no
    # cross-key bridge is needed, so the adapter remains free to slide in Z.
    fork = front_rail + rear_rail + web
    return _label(fork, "floating_z_drive_fork", FORK_GOLD)


def _drive_parts(
    travel: float, datum: BeltEndSliderDatumReport
) -> tuple[Shape, Shape, Shape]:
    return (
        _keyboard_short_drive_tongue(travel),
        _floating_z_drive_fork(travel, datum),
        _belt_slider_drive_adapter(travel, datum),
    )


def _v3_moving_parts(travel: float) -> list[Shape]:
    """Build only approved V3 moving parts, never the filtered legacy solids."""
    tray = v3._keyboard_tray_with_mount_holes(v3.v2.v1._keyboard_tray(travel), travel)
    children = [
        v3.v2.v1._swing_arm(travel),
        tray,
        v3.v2.v1._trackpad_tray(travel),
    ]
    children.extend(v3.v2._moving_carriage(position, travel) for position in POSITIONS)
    children.extend(v3._carriage_mount_ear(position, travel) for position in POSITIONS)
    children.extend(v3._keyboard_tray_mount_bushing(position, travel) for position in POSITIONS)
    return children


def build_pose(travel: float) -> Compound:
    """Return one deterministic, flat, selectable V4 assembly."""
    state = pose_state(travel)
    # Intentionally expensive BREP query: cache exactly once for this pose.
    datum = belt_end_slider_datum_report(state.travel)

    v3_children = _v3_moving_parts(state.travel)
    base_children = list(build_low_profile_base().children)
    guides = [build_removable_guide_assembly(position) for position in POSITIONS]
    belt_children = _world_children(build_belt_system(state.travel))
    cartridge_children = _world_children(build_cartridge(state.travel))
    drive_children = list(_drive_parts(state.travel, datum))
    children = [
        *base_children,
        *guides,
        *v3_children,
        *belt_children,
        *cartridge_children,
        *drive_children,
    ]
    labels = [child.label for child in children]
    if len(labels) != len(set(labels)):
        raise AssertionError("V4 top-level labels must be unique")
    if not REQUIRED_LABELS <= set(labels):
        raise AssertionError("V4 assembly is missing required labels")
    if REMOVED_LABELS & set(labels):
        raise AssertionError("V4 assembly retained a removed V3 label")
    return Compound(label=f"v4_keyboard_trackpad_pose_t{state.travel:.3f}", children=children)


@dataclass(frozen=True)
class DriveInterfaceReport:
    travel: float
    slider_dy: float
    tongue_dy: float
    slider_dz: float
    tongue_dz: float
    y_contact_travel_positive: float
    y_contact_travel_negative: float
    positive_y_contact_proven: bool
    negative_y_contact_proven: bool
    z_capture_overlap: float
    nominal_positive_penetration_volume: float


def _z_overlap(first: Shape, second: Shape) -> float:
    a = first.bounding_box()
    b = second.bounding_box()
    return max(0.0, min(a.max.Z, b.max.Z) - max(a.min.Z, b.min.Z))


def _contact_after_y_travel(moving: Shape, fixed: Shape, dy: float) -> bool:
    witnessed = moving.moved(Location((0.0, dy, 0.0)))
    return witnessed.distance_to(fixed) <= 1e-6 and (witnessed & fixed).volume <= VOLUME_TOLERANCE


def drive_interface_report(travel: float) -> DriveInterfaceReport:
    state = pose_state(travel)
    datum = belt_end_slider_datum_report(state.travel)
    _, fork, adapter = _drive_parts(state.travel, datum)
    adapter_box = adapter.bounding_box()
    fork_y = _keyboard_rear_y(state.travel) + 0.65
    key_half_depth = 1.50
    inner_negative_y = fork_y - 1.60
    inner_positive_y = fork_y + 1.60
    negative_travel = adapter_box.min.Y - inner_negative_y
    positive_travel = inner_positive_y - adapter_box.max.Y
    nominal_penetration = (adapter & fork).volume
    return DriveInterfaceReport(
        travel=state.travel,
        slider_dy=state.belt_feed,
        tongue_dy=state.keyboard_dy,
        slider_dz=datum.slider_center_z - 35.0,
        tongue_dz=state.keyboard_dz,
        y_contact_travel_positive=positive_travel,
        y_contact_travel_negative=negative_travel,
        positive_y_contact_proven=_contact_after_y_travel(adapter, fork, positive_travel),
        negative_y_contact_proven=_contact_after_y_travel(adapter, fork, -negative_travel),
        z_capture_overlap=_z_overlap(adapter, fork),
        nominal_positive_penetration_volume=nominal_penetration,
    )


@dataclass(frozen=True)
class GuideAlignmentReport:
    poses: tuple[float, ...]
    guide_labels: tuple[str, ...]
    all_guides_one_valid_solid: bool
    all_carriages_captured: bool
    carriage_guide_distances: tuple[float, ...]
    carriage_guide_penetration_volumes: tuple[float, ...]


def guide_alignment_report(travels: Iterable[float]) -> GuideAlignmentReport:
    poses = tuple(pose_state(travel).travel for travel in travels)
    guide_labels = tuple(f"removable_keyboard_guide_{position}" for position in POSITIONS)
    distances = []
    penetrations = []
    one_solid = []
    for travel in poses:
        parts = {child.label: child for child in build_pose(travel).children}
        for position, guide_label in zip(POSITIONS, guide_labels):
            guide = parts[guide_label]
            carriage = parts[f"keyboard_carriage_{position}"]
            distances.append(carriage.distance_to(guide))
            penetrations.append((carriage & guide).volume)
            one_solid.append(guide.is_valid() and len(guide.solids()) == 1)
    return GuideAlignmentReport(
        poses=poses,
        guide_labels=guide_labels,
        all_guides_one_valid_solid=all(one_solid),
        all_carriages_captured=all(distance <= 0.61 for distance in distances),
        carriage_guide_distances=tuple(distances),
        carriage_guide_penetration_volumes=tuple(penetrations),
    )


@dataclass(frozen=True)
class CollisionPair:
    travel: float
    first_label: str
    second_label: str
    positive_volume: float
    distance: float


@dataclass(frozen=True)
class InterferenceReport:
    poses: tuple[float, ...]
    checked_pairs: tuple[CollisionPair, ...]
    intended_contact_pairs: tuple[Tuple[str, str], ...]
    unintended_positive_volume_pairs: tuple[CollisionPair, ...]


def _canonical_pair(first: str, second: str) -> tuple[str, str]:
    direct = (first, second)
    reverse = (second, first)
    if direct in INTENDED_CONTACT_ALLOWLIST:
        return direct
    if reverse in INTENDED_CONTACT_ALLOWLIST:
        return reverse
    return direct


def _changed_pair_names() -> tuple[tuple[str, str], ...]:
    pairs = []
    base = "rounded_low_profile_base"
    pairs.extend((base, f"removable_keyboard_guide_{position}") for position in POSITIONS)
    pairs.append((base, "keyboard_tray_continuous"))
    pairs.extend((base, f"keyboard_carriage_{position}") for position in POSITIONS)
    housing = "fixed_cartridge_housing"
    pairs.extend(
        (housing, moving)
        for moving in (
            "keyboard_tray_continuous",
            "keyboard_short_drive_tongue",
            "floating_z_drive_fork",
            "eccentric_horizontal_swing_arm",
        )
    )
    for belt in ("flexible_belt_backing", "flexible_belt_teeth"):
        pairs.extend(
            (belt, fixed)
            for fixed in (
                "fixed_cartridge_housing",
                "fixed_planar_belt_entry_guide",
                "floating_z_drive_fork",
            )
        )
    pairs.extend(
        ("eccentric_horizontal_swing_arm", fixed)
        for fixed in ("removable_cartridge_top_cap", "fixed_cartridge_housing")
    )
    pairs.extend(
        (
            f"female_slide_receiver_{RECEIVER_EXTERNAL_NAME[position]}",
            f"removable_keyboard_guide_{position}",
        )
        for position in POSITIONS
    )
    pairs.extend(
        (
            "rounded_low_profile_base",
            f"female_slide_receiver_{external_name}",
        )
        for external_name in RECEIVER_EXTERNAL_NAME.values()
    )
    pairs.extend(
        (
            ("belt_slider_drive_adapter", "floating_z_drive_fork"),
            ("belt_slider_drive_adapter", "screwless_belt_end_slider"),
            ("floating_z_drive_fork", "screwless_belt_end_slider"),
            ("keyboard_tray_continuous", "keyboard_short_drive_tongue"),
            ("keyboard_short_drive_tongue", "floating_z_drive_fork"),
        )
    )
    return tuple(pairs)


def interference_report(travels: Iterable[float] = (0.0, 0.5, 1.0)) -> InterferenceReport:
    poses = tuple(pose_state(travel).travel for travel in travels)
    pair_names = _changed_pair_names()
    checked = []
    intended = set()
    unintended = []
    for travel in poses:
        parts = {child.label: child for child in build_pose(travel).children}
        for first_label, second_label in pair_names:
            first = parts[first_label]
            second = parts[second_label]
            overlap = (first & second).volume
            pair = CollisionPair(
                travel,
                first_label,
                second_label,
                overlap,
                first.distance_to(second),
            )
            checked.append(pair)
            canonical = _canonical_pair(first_label, second_label)
            if canonical in INTENDED_CONTACT_ALLOWLIST and pair.distance <= 1e-5:
                intended.add(canonical)
            elif overlap > VOLUME_TOLERANCE:
                unintended.append(pair)
    return InterferenceReport(
        poses=poses,
        checked_pairs=tuple(checked),
        intended_contact_pairs=tuple(sorted(intended)),
        unintended_positive_volume_pairs=tuple(unintended),
    )
