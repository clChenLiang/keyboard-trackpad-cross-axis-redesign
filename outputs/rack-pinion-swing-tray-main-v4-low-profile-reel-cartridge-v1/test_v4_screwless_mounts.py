"""Contract tests for the V4 rounded base and screwless guide supports."""

from dataclasses import FrozenInstanceError

import pytest

from v4_screwless_base import (
    BASE_DEPTH,
    BASE_TOP_Z,
    BASE_WIDTH,
    CARTRIDGE_CENTER,
    MAIN_OPENING_RADIUS,
    OUTER_CORNER_RADIUS,
    POSITIONS,
    PRIMARY_TABLE_Z,
    build_low_profile_base,
    build_mount_assembly,
    build_supports,
    insertion_sweep_report,
    mount_report,
)


def _parts(compound):
    return {child.label: child for child in compound.children}


def test_low_base_keeps_approved_envelope_and_height_datums():
    assembly = build_low_profile_base()
    box = assembly.bounding_box()
    assert box.size.X == pytest.approx(BASE_WIDTH, abs=0.1)
    assert box.size.Y == pytest.approx(BASE_DEPTH, abs=0.1)
    assert (box.min.Y + box.max.Y) / 2.0 == pytest.approx(57.5, abs=0.05)
    assert box.max.Z == pytest.approx(BASE_TOP_Z, abs=0.05)
    assert box.min.Z == pytest.approx(PRIMARY_TABLE_Z, abs=0.05)


def test_structural_base_is_one_valid_load_path_above_z_one():
    parts = _parts(build_low_profile_base())
    base = parts["rounded_low_profile_base"]
    assert base.is_valid
    assert len(base.solids()) == 1
    assert base.bounding_box().min.Z >= 1.0 - 1e-6
    assert base.bounding_box().max.Z == pytest.approx(7.0)

    # Cartridge pod must carry the future OD42 housing at its V1 world datum.
    from build123d import Align, Cylinder, Pos

    x, y = CARTRIDGE_CENTER
    probe = Cylinder(20.5, 0.5, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Pos(x, y, 6.5)
    )
    assert (base & probe).volume > 500.0


def test_restrained_rounded_contract_and_three_real_openings():
    assert 8.0 <= OUTER_CORNER_RADIUS <= 12.0
    assert 6.0 <= MAIN_OPENING_RADIUS <= 10.0
    base = _parts(build_low_profile_base())["rounded_low_profile_base"]
    # These points are in the three named primary openings, not an uncut slab.
    for x, y in ((0.0, -20.0), (0.0, 40.0), (0.0, 115.0)):
        from build123d import Align, Cylinder, Pos

        void_probe = Cylinder(2.0, 6.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Pos(x, y, 1.0)
        )
        assert (base & void_probe).volume < 1e-5


def test_primary_pads_are_separate_and_define_table_plane():
    parts = _parts(build_low_profile_base())
    expected = {
        "primary_tpu_pad_front_left",
        "primary_tpu_pad_front_right",
        "primary_tpu_pad_rear_left",
        "primary_tpu_pad_rear_right",
    }
    assert expected <= parts.keys()
    for label in expected:
        pad = parts[label]
        assert pad.is_valid and pad.volume > 0.0
        assert pad.bounding_box().min.Z == pytest.approx(-1.5)


def test_required_receiver_and_support_labels_are_unique():
    base = build_low_profile_base()
    supports = build_supports(installed=True)
    labels = [part.label for part in (*base.children, *supports.children)]
    assert len(labels) == len(set(labels))
    for position in POSITIONS:
        assert f"female_slide_receiver_{position}" in labels
        assert f"removable_guide_support_{position}" in labels
        support = _parts(supports)[f"removable_guide_support_{position}"]
        assert support.is_valid and len(support.solids()) == 1


def test_four_supports_are_captured_by_load_bearing_geometry():
    report = mount_report()
    assert report.receiver_count == 4
    assert report.all_at_end_datum
    assert report.all_vertically_captured
    assert report.all_laterally_captured
    assert report.snap_tabs_clear_primary_load_path
    assert report.left_insertion_direction == "+X"
    assert report.right_insertion_direction == "-X"
    with pytest.raises(FrozenInstanceError):
        report.receiver_count = 3


def test_insertion_sweeps_are_clear_before_intended_final_contact():
    sweep = insertion_sweep_report()
    assert sweep.positions == POSITIONS
    assert sweep.all_approaches_clear
    assert sweep.all_finish_at_end_wall
    assert max(sweep.maximum_unintended_overlap_volumes) < 1e-4


def test_combined_builder_is_stable_and_has_no_screw_holes():
    assembly = build_mount_assembly(installed=True)
    labels = {part.label for part in assembly.children}
    assert "rounded_low_profile_base" in labels
    assert not any("screw" in label or "hole" in label for label in labels)
