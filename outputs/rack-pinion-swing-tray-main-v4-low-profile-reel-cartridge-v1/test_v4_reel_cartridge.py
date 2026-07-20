from dataclasses import FrozenInstanceError

import pytest

from v4_kinematics import BELT_PITCH, BELT_Z, REEL_TEETH
from v4_reel_cartridge import (
    BACKING_THICKNESS,
    BELT_WIDTH,
    TOOTH_DEPTH,
    belt_retention_report,
    build_belt_system,
    reel_engagement_report,
)


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


@pytest.mark.parametrize("travel", [-0.01, 1.01])
def test_constructor_rejects_out_of_range_travel(travel):
    with pytest.raises(ValueError):
        build_belt_system(travel)
