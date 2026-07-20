from dataclasses import FrozenInstanceError
from math import isclose, pi

import pytest

from v4_kinematics import (
    BELT_PITCH,
    BELT_Z,
    FEED_TRAVEL,
    FREE_BELT_CENTERLINE,
    KEYBOARD_DY,
    KEYBOARD_DZ,
    REEL_PITCH_RADIUS,
    REEL_TEETH,
    V4PoseState,
    belt_centerline_length,
    pose_state,
)


@pytest.mark.parametrize(
    ("travel", "belt_feed", "reel_angle_deg", "keyboard_dy", "keyboard_dz"),
    (
        (0.0, 0.0, 0.0, 0.0, 0.0),
        (0.5, 10.25, 45.0, 10.21, -7.5),
        (1.0, 20.50, 90.0, 20.42, -15.0),
    ),
)
def test_exported_pose_mapping(
    travel, belt_feed, reel_angle_deg, keyboard_dy, keyboard_dz
):
    state = pose_state(travel)

    assert state.travel == travel
    assert isclose(state.belt_feed, belt_feed, abs_tol=1e-12)
    assert isclose(state.reel_angle_deg, reel_angle_deg, abs_tol=1e-12)
    assert isclose(state.keyboard_dy, keyboard_dy, abs_tol=1e-12)
    assert isclose(state.keyboard_dz, keyboard_dz, abs_tol=1e-12)
    assert state.belt_z == BELT_Z


@pytest.mark.parametrize("travel", (0.0, 0.5, 1.0))
def test_belt_centerline_length_is_constant_at_exported_poses(travel):
    state = pose_state(travel)

    assert isclose(
        state.free_belt_length + REEL_PITCH_RADIUS * state.reel_angle_rad,
        FREE_BELT_CENTERLINE,
        abs_tol=1e-12,
    )
    assert isclose(
        belt_centerline_length(travel), FREE_BELT_CENTERLINE, abs_tol=1e-12
    )


@pytest.mark.parametrize("travel", (-0.001, 1.001))
def test_pose_state_rejects_travel_outside_unit_interval(travel):
    with pytest.raises(ValueError, match=r"\[0, 1\]"):
        pose_state(travel)


def test_reel_angle_is_derived_from_pitch_radius():
    state = pose_state(1.0)

    assert REEL_PITCH_RADIUS == REEL_TEETH * BELT_PITCH / (2.0 * pi)
    assert state.reel_angle_rad == pytest.approx(FEED_TRAVEL / REEL_PITCH_RADIUS)


def test_pose_state_is_frozen():
    state = pose_state(0.5)

    assert isinstance(state, V4PoseState)
    with pytest.raises(FrozenInstanceError):
        state.travel = 0.75


def test_public_travel_constants_match_v4_contract():
    assert FEED_TRAVEL == 20.50
    assert KEYBOARD_DY == 20.42
    assert KEYBOARD_DZ == -15.0
    assert BELT_Z == 35.0
    assert FREE_BELT_CENTERLINE == 46.0
