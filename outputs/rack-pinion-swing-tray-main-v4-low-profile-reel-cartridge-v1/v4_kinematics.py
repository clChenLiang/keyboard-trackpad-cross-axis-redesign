"""Pure kinematics for the V4 constant-Z belt and planar reel interface.

All distances are millimetres. Angles are calculated in radians and exposed
in degrees as a convenience for pose exporters.
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

    def __post_init__(self) -> None:
        normalized_travel = float(self.travel)
        if not 0.0 <= normalized_travel <= 1.0:
            raise ValueError("travel must be within [0, 1]")
        object.__setattr__(self, "travel", normalized_travel)

    @property
    def belt_feed(self) -> float:
        return FEED_TRAVEL * self.travel

    @property
    def reel_angle_rad(self) -> float:
        return self.belt_feed / REEL_PITCH_RADIUS

    @property
    def reel_angle_deg(self) -> float:
        return degrees(self.reel_angle_rad)

    @property
    def keyboard_dy(self) -> float:
        return KEYBOARD_DY * self.travel

    @property
    def keyboard_dz(self) -> float:
        return KEYBOARD_DZ * self.travel

    @property
    def belt_z(self) -> float:
        return BELT_Z

    @property
    def free_belt_length(self) -> float:
        return FREE_BELT_CENTERLINE - self.belt_feed

    @property
    def belt_centerline_length(self) -> float:
        reeled_length = REEL_PITCH_RADIUS * self.reel_angle_rad
        return self.free_belt_length + reeled_length


def pose_state(travel: float) -> V4PoseState:
    """Return the V4 pose at normalized ``travel`` in the closed interval [0, 1]."""
    return V4PoseState(travel)


def belt_centerline_length(travel: float) -> float:
    """Return free plus reeled belt centerline length at a normalized pose."""
    return pose_state(travel).belt_centerline_length
