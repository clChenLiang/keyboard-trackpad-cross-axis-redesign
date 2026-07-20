"""Focused integration contract for the complete three-pose V4 mechanism."""

from dataclasses import FrozenInstanceError
from importlib import import_module

import pytest

from v4_assembly import (
    CARTRIDGE_CENTER,
    INTENDED_CONTACT_ALLOWLIST,
    INTENDED_CONTACT_LIMITS,
    REMOVED_LABELS,
    REQUIRED_LABELS,
    _shape_is_valid,
    build_pose,
    drive_interface_report,
    guide_alignment_report,
    interference_report,
)


POSES = (0.0, 0.5, 1.0)


def _parts(pose):
    return {child.label: child for child in pose.children}


@pytest.mark.parametrize("travel", POSES)
def test_pose_has_exact_unique_required_labels_and_no_removed_parts(travel):
    assembly = build_pose(travel)
    labels = [child.label for child in assembly.children]
    assert assembly.label == f"v4_keyboard_trackpad_pose_t{travel:.3f}"
    assert len(labels) == len(set(labels))
    assert REQUIRED_LABELS <= set(labels)
    assert REMOVED_LABELS.isdisjoint(labels)
    assert not any("back_pressure_" in label for label in labels)
    assert not any("rack_retention" in label for label in labels)


def test_base_and_cartridge_share_world_datums_without_penetration():
    parts = _parts(build_pose(0.5))
    base = parts["rounded_low_profile_base"]
    housing = parts["fixed_cartridge_housing"]
    shaft = parts["vertical_output_shaft"]
    assert base.bounding_box().size.X == pytest.approx(310.0, abs=0.1)
    assert base.bounding_box().size.Y == pytest.approx(225.0, abs=0.1)
    assert base.bounding_box().max.Z == pytest.approx(7.0)
    assert base.bounding_box().min.Z == pytest.approx(1.0)
    assert shaft.bounding_box().center().X == pytest.approx(CARTRIDGE_CENTER[0], abs=0.02)
    assert shaft.bounding_box().center().Y == pytest.approx(CARTRIDGE_CENTER[1], abs=0.02)
    assert housing.distance_to(base) == pytest.approx(0.0, abs=1e-6)
    assert (housing & base).volume < 1e-6


def test_trackpad_finishes_right_aligned_without_keyboard_z_overlap():
    final = _parts(build_pose(1.0))
    keyboard = final["keyboard_tray_continuous"].bounding_box()
    trackpad = final["magic_trackpad_small_tray"].bounding_box()
    assert keyboard.size.X == pytest.approx(279.7, abs=0.1)
    assert keyboard.size.Y == pytest.approx(115.7, abs=0.1)
    assert trackpad.size.X == pytest.approx(160.8, abs=0.1)
    assert trackpad.size.Y == pytest.approx(115.7, abs=0.1)
    assert trackpad.max.X == pytest.approx(139.85, abs=0.1)
    assert keyboard.max.X == pytest.approx(139.85, abs=0.1)
    for travel in POSES:
        parts = _parts(build_pose(travel))
        assert parts["keyboard_tray_continuous"].bounding_box().max.Z < (
            parts["magic_trackpad_small_tray"].bounding_box().min.Z
        )


def test_real_floating_z_interface_carries_y_and_accommodates_full_stroke():
    reports = tuple(drive_interface_report(travel) for travel in POSES)
    assert reports[-1].slider_dy == pytest.approx(20.5)
    assert reports[-1].tongue_dy == pytest.approx(20.42)
    assert reports[-1].tongue_dz == pytest.approx(-15.0)
    assert reports[-1].slider_dz == pytest.approx(0.0)
    assert min(report.z_capture_overlap for report in reports) >= 5.9
    assert max(report.nominal_positive_penetration_volume for report in reports) < 1e-6
    assert all(report.positive_y_contact_proven for report in reports)
    assert all(report.negative_y_contact_proven for report in reports)
    assert max(report.y_contact_travel_positive for report in reports) <= 0.19
    assert max(report.y_contact_travel_negative for report in reports) <= 0.19
    with pytest.raises(FrozenInstanceError):
        reports[0].z_capture_overlap = 0.0


def test_changed_interface_collision_report_has_only_named_clean_pairs():
    report = interference_report(POSES)
    assert report.poses == POSES
    assert report.checked_pairs
    assert report.unintended_positive_volume_pairs == ()
    assert report.intended_contact_pairs
    assert set(report.intended_contact_pairs) <= INTENDED_CONTACT_ALLOWLIST
    for travel in POSES:
        pose_pairs = [
            frozenset((pair.first_label, pair.second_label))
            for pair in report.checked_pairs
            if pair.travel == travel
        ]
        assert len(pose_pairs) == len(set(pose_pairs))
    for intended_pair in report.intended_contact_pairs:
        allowance = INTENDED_CONTACT_LIMITS[intended_pair]
        observed = [
            pair.positive_volume
            for pair in report.checked_pairs
            if frozenset((pair.first_label, pair.second_label)) == frozenset(intended_pair)
        ]
        assert observed
        assert max(observed) <= allowance.max_overlap
    assert all(pair.first_label != pair.second_label for pair in report.checked_pairs)
    with pytest.raises(FrozenInstanceError):
        report.poses = ()


def test_collision_report_covers_every_changed_belt_drum_and_guide_pair():
    report = interference_report(POSES)
    required_pairs = {
        frozenset(("floating_z_drive_fork", "fixed_planar_belt_entry_guide")),
        frozenset(("flexible_belt_backing", "reel_drum_41t")),
        frozenset(("flexible_belt_teeth", "reel_drum_41t")),
        frozenset(("flexible_belt_backing", "fixed_cartridge_housing")),
        frozenset(("flexible_belt_backing", "fixed_planar_belt_entry_guide")),
        frozenset(("flexible_belt_teeth", "fixed_cartridge_housing")),
        frozenset(("flexible_belt_teeth", "fixed_planar_belt_entry_guide")),
    }
    checked_pairs = {
        frozenset((pair.first_label, pair.second_label))
        for pair in report.checked_pairs
    }
    assert required_pairs <= checked_pairs

    by_pair = {}
    for pair in report.checked_pairs:
        by_pair.setdefault(frozenset((pair.first_label, pair.second_label)), []).append(pair)
    fork_guide = by_pair[
        frozenset(("floating_z_drive_fork", "fixed_planar_belt_entry_guide"))
    ]
    backing_drum = by_pair[frozenset(("flexible_belt_backing", "reel_drum_41t"))]
    teeth_drum = by_pair[frozenset(("flexible_belt_teeth", "reel_drum_41t"))]
    assert max(pair.positive_volume for pair in fork_guide) < 1e-6
    assert 0.0 < min(pair.positive_volume for pair in backing_drum)
    assert max(pair.positive_volume for pair in backing_drum) < 0.13
    assert max(pair.positive_volume for pair in teeth_drum) < 1.10
    for belt_entry_pair in (
        ("flexible_belt_backing", "fixed_planar_belt_entry_guide"),
        ("flexible_belt_teeth", "fixed_planar_belt_entry_guide"),
    ):
        allowance = INTENDED_CONTACT_LIMITS[belt_entry_pair]
        observed = by_pair[frozenset(belt_entry_pair)]
        assert allowance.max_overlap == pytest.approx(1e-6)
        assert max(pair.positive_volume for pair in observed) <= allowance.max_overlap
    assert {
        ("flexible_belt_backing", "reel_drum_41t"),
        ("flexible_belt_teeth", "reel_drum_41t"),
    } <= set(report.intended_contact_pairs)


def test_four_fused_guides_follow_retained_carriage_datums():
    report = guide_alignment_report(POSES)
    assert report.guide_labels == tuple(
        f"removable_keyboard_guide_{position}"
        for position in ("left_front", "right_front", "left_rear", "right_rear")
    )
    assert report.all_guides_one_valid_solid
    assert report.all_carriages_captured
    assert max(report.carriage_guide_penetration_volumes) < 0.05
    for travel in POSES:
        labels = [child.label for child in build_pose(travel).children]
        assert not any(label.startswith("fixed_keyboard_guide_") for label in labels)


@pytest.mark.parametrize("travel", POSES)
def test_every_selectable_top_level_child_is_valid_and_positive(travel):
    for child in build_pose(travel).children:
        assert _shape_is_valid(child), child.label
        assert child.volume > 0.0, child.label


@pytest.mark.parametrize(
    ("module_name", "travel"),
    (("keyboard_mode", 0.0), ("mid_mode", 0.5), ("trackpad_mode", 1.0)),
)
def test_entry_scripts_only_delegate_to_shared_pose_builder(module_name, travel):
    module = import_module(module_name)
    assert set(name for name in vars(module) if not name.startswith("__")) == {
        "Compound",
        "build_pose",
        "gen_step",
    }
    assert module.gen_step().label == f"v4_keyboard_trackpad_pose_t{travel:.3f}"
