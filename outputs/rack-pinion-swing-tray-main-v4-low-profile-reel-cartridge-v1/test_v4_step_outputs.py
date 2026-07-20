"""Reload and labeling contracts for the five final V4 STEP deliverables."""

from pathlib import Path

import pytest
from build123d import import_step

from v4_assembly import REQUIRED_LABELS


OUTPUT_DIR = Path(__file__).resolve().parent
POSE_STEMS = ("keyboard_mode", "mid_mode", "trackpad_mode")
DETAIL_LABELS = {
    "reel_cartridge_section_exploded": {
        "lower_axial_thrust_interface",
        "lower_radial_bearing_seat",
        "above_base_return_spring",
        "reel_drum_41t",
        "removable_reel_axial_retainer",
        "upper_radial_bearing_seat",
        "vertical_output_shaft",
        "fixed_cartridge_housing",
        "fixed_planar_belt_entry_guide",
        "removable_cartridge_top_cap",
        "flexible_belt_backing",
        "flexible_belt_teeth",
        "reel_end_tooth_wedge",
        "screwless_belt_end_slider",
        "slider_end_tooth_wedge",
    },
    "screwless_guide_mount_detail": {
        "receiver_base_coupon",
        "female_slide_receiver_front_left",
        "installed_guide_support_foot",
        "exploded_guide_support_foot",
        "released_depressed_tab_witness_left_front",
        "insertion_direction_witness_plus_x",
        "solid_inward_end_wall_datum",
        "top_access_release_window_witness",
    },
}
ALL_STEMS = (*POSE_STEMS, *DETAIL_LABELS)
STEP_PATHS = {stem: OUTPUT_DIR / f"{stem}.step" for stem in ALL_STEMS}
ANY_STEP_EXISTS = any(path.exists() for path in STEP_PATHS.values())


def _recursive_labels(shape):
    labels = {shape.label} if shape.label else set()
    for child in getattr(shape, "children", ()):
        labels.update(_recursive_labels(child))
    return labels


@pytest.mark.skipif(not ANY_STEP_EXISTS, reason="STEP generation has not started")
@pytest.mark.parametrize("stem", ALL_STEMS)
def test_step_reloads_with_nonzero_solids_valid_bbox_and_required_labels(stem):
    path = STEP_PATHS[stem]
    assert path.exists(), f"missing STEP after generation started: {path.name}"
    assert path.stat().st_size > 0

    model = import_step(path)
    solids = model.solids()
    assert solids, stem
    assert all(solid.volume > 0.0 for solid in solids), stem

    bbox = model.bounding_box()
    assert bbox.size.X > 0.0, stem
    assert bbox.size.Y > 0.0, stem
    assert bbox.size.Z > 0.0, stem

    required = REQUIRED_LABELS if stem in POSE_STEMS else DETAIL_LABELS[stem]
    assert required <= _recursive_labels(model), stem


@pytest.mark.skipif(not ANY_STEP_EXISTS, reason="STEP generation has not started")
@pytest.mark.parametrize("stem", POSE_STEMS)
def test_pose_step_keeps_low_profile_base_envelope(stem):
    model = import_step(STEP_PATHS[stem])
    base = next(child for child in model.children if child.label == "rounded_low_profile_base")
    bbox = base.bounding_box()
    assert bbox.size.X == pytest.approx(310.0, abs=0.1)
    assert bbox.size.Y == pytest.approx(225.0, abs=0.1)
    assert bbox.min.Z == pytest.approx(1.0, abs=0.01)
    assert bbox.max.Z == pytest.approx(7.0, abs=0.01)
