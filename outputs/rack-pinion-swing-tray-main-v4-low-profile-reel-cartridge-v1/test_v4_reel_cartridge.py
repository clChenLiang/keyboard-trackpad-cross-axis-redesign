from dataclasses import FrozenInstanceError

import pytest

import v4_kinematics
import v4_reel_cartridge
import reel_cartridge_section_exploded
from build123d import Location
from v4_kinematics import BELT_PITCH, BELT_TOTAL_CENTERLINE, BELT_Z, FEED_TRAVEL, REEL_TEETH
from v4_reel_cartridge import (
    BASE_STRUCTURAL_UNDERSIDE_Z,
    BASE_TOP_Z,
    BACKING_THICKNESS,
    BELT_WIDTH,
    TOOTH_DEPTH,
    belt_path_report,
    belt_retention_report,
    build_belt_system,
    build_cartridge,
    cartridge_stack_report,
    reel_engagement_report,
    reel_wedge_insertion_report,
    slider_wedge_insertion_report,
    stop_contact_report,
    tangent_tooth_pitch_report,
)


CARTRIDGE_LABELS = {
    "lower_axial_thrust_interface",
    "lower_radial_bearing_seat",
    "above_base_return_spring",
    "return_spring_inner_anchor",
    "return_spring_outer_anchor",
    "reel_drum_41t",
    "upper_radial_bearing_seat",
    "vertical_output_shaft",
    "fixed_cartridge_housing",
    "fixed_planar_belt_entry_guide",
    "removable_cartridge_top_cap",
    "independent_hard_stop_keyboard",
    "independent_hard_stop_trackpad",
    "shaft_rotating_stop_lug",
}


def test_cartridge_stack_entities_are_flat_unique_valid_positive_volume_parts():
    cartridge = build_cartridge(0.5)
    labels = [child.label for child in cartridge.children]

    assert CARTRIDGE_LABELS <= set(labels)
    assert len(labels) == len(set(labels))
    for child in cartridge.children:
        assert child.is_valid(), child.label
        assert child.volume > 0.0, child.label
        if child.label in CARTRIDGE_LABELS:
            assert len(child.solids()) == 1, child.label


def test_cartridge_can_omit_belt_without_losing_the_reused_reel():
    labels = {child.label for child in build_cartridge(0.5, include_belt=False).children}

    assert labels == CARTRIDGE_LABELS


def test_stack_report_is_immutable_and_measures_above_base_supports_and_housing():
    report = cartridge_stack_report()

    assert BASE_TOP_Z == pytest.approx(7.0)
    assert BASE_STRUCTURAL_UNDERSIDE_Z == pytest.approx(1.0)
    assert report.minimum_component_z >= BASE_TOP_Z - 1e-6
    assert report.minimum_component_z > BASE_STRUCTURAL_UNDERSIDE_Z
    assert report.minimum_above_base_top >= -1e-6
    assert report.minimum_above_structural_underside >= 6.0 - 1e-6
    assert report.lower_radial_clearance <= 0.15
    assert report.lower_radial_axial_overlap > 0.0
    assert report.lower_axial_contact_distance <= 1e-6
    assert report.lower_axial_positive_penetration_volume < 1e-6
    assert report.upper_radial_clearance <= 0.15
    assert report.upper_radial_axial_overlap > 0.0
    assert report.spring_inner_anchor_distance <= 1e-6
    assert report.spring_outer_anchor_distance <= 1e-6
    assert report.spring_anchor_positive_penetration_volume < 1e-6
    assert report.top_cap_wedge_blocking_overlap_volume > 1e-3
    assert report.top_cap_removed_service_overlap_volume < 1e-6
    assert 38.0 <= report.housing_outer_diameter <= 42.0

    with pytest.raises(FrozenInstanceError):
        report.housing_outer_diameter = 0.0


@pytest.mark.parametrize(
    ("travel", "keyboard", "trackpad"),
    [(0.0, True, False), (0.5, False, False), (1.0, False, True)],
)
def test_independent_hard_stops_contact_only_at_their_intended_end_pose(
    travel, keyboard, trackpad
):
    report = stop_contact_report(travel)

    assert report.keyboard_contact is keyboard
    assert report.trackpad_contact is trackpad
    assert report.keyboard_positive_penetration_volume < 1e-6
    assert report.trackpad_positive_penetration_volume < 1e-6


def test_sectioned_cartridge_preserves_labels_and_exposes_a_valid_housing():
    cartridge = build_cartridge(0.5, sectioned=True)
    children = {child.label: child for child in cartridge.children}

    assert CARTRIDGE_LABELS <= children.keys()
    assert children["fixed_cartridge_housing"].is_valid()
    assert 0.0 < children["fixed_cartridge_housing"].volume < build_cartridge(0.5).children[
        [child.label for child in build_cartridge(0.5).children].index("fixed_cartridge_housing")
    ].volume


def test_section_exploded_entrypoint_reuses_the_labeled_cartridge_builder():
    exploded = reel_cartridge_section_exploded.gen_step()

    assert exploded.label.startswith("v4_reel_cartridge_section_exploded")
    assert CARTRIDGE_LABELS <= {child.label for child in exploded.children}


@pytest.mark.parametrize("travel", [0.0, 0.5, 1.0])
def test_reel_engagement_is_geometric_and_clear(travel):
    report = reel_engagement_report(travel)

    assert report.engaged_teeth >= 5
    assert report.occupied_grooves == report.engaged_teeth
    assert report.positive_penetration_volume < 1e-3


def test_serviceable_wedges_capture_teeth_and_seat_against_load():
    report = belt_retention_report()

    assert report.reel_wedge_captured_teeth >= 4
    assert report.slider_wedge_captured_teeth >= 4
    assert report.reel_wedge_at_datum
    assert report.slider_wedge_at_datum
    assert report.reel_working_direction_blocked
    assert report.slider_working_direction_blocked

    with pytest.raises(FrozenInstanceError):
        report.reel_wedge_captured_teeth = 0


def test_retention_count_requires_material_in_the_actual_wedge():
    travel = 0.5
    teeth = v4_reel_cartridge._slider_capture_teeth(travel)
    wedge, pockets = v4_reel_cartridge._slider_wedge_and_pockets(travel)
    probes = v4_reel_cartridge._slider_load_probes(teeth)

    assert v4_reel_cartridge._captured_tooth_count(teeth, pockets, wedge, probes) == 5
    displaced = wedge.moved(Location((0.0, 3.0, 0.0)))
    assert v4_reel_cartridge._captured_tooth_count(teeth, pockets, displaced, probes) == 0


def test_slider_wedge_has_a_collision_free_transverse_insertion_path():
    report = slider_wedge_insertion_report(samples=7)

    assert report.insertion_direction == "+X toward -X"
    assert report.samples == 7
    assert report.max_positive_collision_volume < 1e-6
    assert report.final_at_datum


def test_belt_system_has_commercial_belt_and_service_parts_as_labeled_solids():
    assembly = build_belt_system(0.5)
    children = {child.label: child for child in assembly.children}

    assert {
        "flexible_belt_backing",
        "flexible_belt_teeth",
        "reel_end_tooth_wedge",
        "slider_end_tooth_wedge",
        "reel_drum_41t",
        "screwless_belt_end_slider",
    } <= children.keys()
    for label in (
        "flexible_belt_backing",
        "flexible_belt_teeth",
        "reel_end_tooth_wedge",
        "slider_end_tooth_wedge",
        "reel_drum_41t",
        "screwless_belt_end_slider",
    ):
        assert children[label].is_valid()
        assert children[label].volume > 0.0


def test_belt_envelope_uses_v4_pitch_and_fixed_world_z():
    assert BELT_PITCH == pytest.approx(2.0)
    assert BELT_WIDTH == pytest.approx(12.0)
    assert BACKING_THICKNESS == pytest.approx(1.2)
    assert TOOTH_DEPTH == pytest.approx(0.9)
    assert REEL_TEETH == 41

    for travel in (0.0, 0.5, 1.0):
        children = {child.label: child for child in build_belt_system(travel).children}
        for label in ("flexible_belt_backing", "flexible_belt_teeth"):
            bounds = children[label].bounding_box()
            assert (bounds.min.Z + bounds.max.Z) / 2.0 == pytest.approx(BELT_Z)


@pytest.mark.parametrize("travel", [0.0, 0.5, 1.0])
def test_fixed_entry_backing_is_continuous_and_keeps_68mm_centerline(travel):
    report = belt_path_report(travel)

    assert report.backing_entry_gap <= 0.05
    assert report.centerline_length == pytest.approx(BELT_TOTAL_CENTERLINE, abs=1e-6)


def test_reel_cartridge_imports_the_upstream_initial_wrap_contract():
    assert v4_reel_cartridge.INITIAL_WRAP_TEETH == v4_kinematics.INITIAL_WRAP_TEETH


@pytest.mark.parametrize(
    ("travel", "straight_offset", "wrapped_offset"),
    [(0.0, 1.00, 1.00), (0.5, 0.75, 1.25), (1.0, 0.50, 1.50)],
)
def test_tangent_tooth_pitch_and_total_tooth_count_are_conserved(
    travel, straight_offset, wrapped_offset
):
    report = tangent_tooth_pitch_report(travel)

    assert report.straight_nearest_center_offset == pytest.approx(straight_offset, abs=1e-6)
    assert report.wrapped_nearest_center_offset == pytest.approx(wrapped_offset, abs=1e-6)
    assert report.tangent_center_pitch == pytest.approx(BELT_PITCH, abs=1e-6)
    assert report.total_tooth_solids == 34


def test_every_tangent_transfer_boundary_has_half_open_tooth_ownership():
    for crossing_feed in range(1, 20, 2):
        for delta in (-1e-7, 0.0, 1e-7):
            report = tangent_tooth_pitch_report((crossing_feed + delta) / FEED_TRAVEL)
            assert report.total_tooth_solids == int(BELT_TOTAL_CENTERLINE / BELT_PITCH)
            assert report.tangent_center_pitch == pytest.approx(BELT_PITCH, abs=1e-6)


def test_tooth_count_and_tangent_pitch_survive_a_21_pose_sweep():
    for index in range(21):
        report = tangent_tooth_pitch_report(index / 20.0)
        assert report.total_tooth_solids == int(BELT_TOTAL_CENTERLINE / BELT_PITCH)
        assert report.tangent_center_pitch == pytest.approx(BELT_PITCH, abs=1e-6)


@pytest.mark.parametrize("travel", [0.0, 0.5, 1.0, 1.0 / FEED_TRAVEL])
def test_backing_is_one_exact_solid_with_conserved_material_volume(travel):
    children = {child.label: child for child in build_belt_system(travel).children}
    backing = children["flexible_belt_backing"]
    report = belt_path_report(travel)

    assert backing.is_valid()
    assert len(backing.solids()) == 1
    assert backing.volume == pytest.approx(
        BACKING_THICKNESS * BELT_WIDTH * BELT_TOTAL_CENTERLINE, rel=1e-7
    )
    assert backing.volume / BELT_WIDTH == pytest.approx(
        BACKING_THICKNESS * report.centerline_length, rel=1e-7
    )


@pytest.mark.parametrize("travel", [0.0, 0.5, 1.0])
def test_path_report_measures_the_explicit_pitch_line_wire(travel):
    wire = v4_reel_cartridge._pitch_line_wire(travel)
    report = belt_path_report(travel)

    assert report.centerline_length == pytest.approx(sum(edge.length for edge in wire.edges()))
    assert report.backing_entry_gap == pytest.approx(0.0, abs=1e-9)


def test_reel_wedge_has_collision_free_axial_service_insertion():
    report = reel_wedge_insertion_report(samples=9)

    assert report.insertion_direction == "+Z toward -Z"
    assert report.samples == 9
    assert report.max_positive_collision_volume < 1e-6
    assert report.final_datum_distance == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize("travel", [-0.01, 1.01])
def test_constructor_rejects_out_of_range_travel(travel):
    with pytest.raises(ValueError):
        build_belt_system(travel)
