"""Pure kinematics for the V4 constant-Z belt and planar reel interface.

All distances are millimetres. Angles are stored in radians for calculation
and exposed in degrees as a convenience for pose exporters.
"""

from dataclasses import dataclass
from math import degrees, pi


BELT_PITCH = 2.0
REEL_TEETH = 41
REEL_PITCH_RADIUS = REEL_TEETH * BELT_PITCH / (2.0 * pi)
FEED_TRAVEL = 20.50
KEYBOARD_DY = 20.42
KEYBOARD_DZ = -15.0
BELT_Z = 35.0
FREE_BELT_CENTERLINE = 46.0


@dataclass(frozen=True)
class V4PoseState:
    """One normalized pose of the decoupled keyboard and belt interface."""

    travel: float
    belt_feed: float
    reel_angle_rad: float
    reel_angle_deg: float
    keyboard_dy: float
    keyboard_dz: float
    belt_z: float
    free_belt_length: float


def pose_state(travel: float) -> V4PoseState:
    """Return the V4 pose at normalized ``travel`` in the closed interval [0, 1]."""
    if not 0.0 <= travel <= 1.0:
        raise ValueError("travel must be within [0, 1]")

    normalized_travel = float(travel)
    belt_feed = FEED_TRAVEL * normalized_travel
    reel_angle_rad = belt_feed / REEL_PITCH_RADIUS
    return V4PoseState(
        travel=normalized_travel,
        belt_feed=belt_feed,
        reel_angle_rad=reel_angle_rad,
        reel_angle_deg=degrees(reel_angle_rad),
        keyboard_dy=KEYBOARD_DY * normalized_travel,
        keyboard_dz=KEYBOARD_DZ * normalized_travel,
        belt_z=BELT_Z,
        free_belt_length=FREE_BELT_CENTERLINE - belt_feed,
    )


def belt_centerline_length(travel: float) -> float:
    """Return free plus reeled belt centerline length at a normalized pose."""
    state = pose_state(travel)
    reeled_length = REEL_PITCH_RADIUS * state.reel_angle_rad
    return state.free_belt_length + reeled_length
