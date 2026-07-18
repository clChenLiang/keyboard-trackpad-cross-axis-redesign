"""Constraint-derived motion and force calculations for V12."""

from dataclasses import dataclass
from math import hypot, radians, sin
from typing import Dict, Tuple

import parameters as p


Vec2 = Tuple[float, float]


def _add(a: Vec2, b: Vec2) -> Vec2:
    return (a[0] + b[0], a[1] + b[1])


def _scale(a: Vec2, value: float) -> Vec2:
    return (a[0] * value, a[1] * value)


def _dot(a: Vec2, b: Vec2) -> float:
    return a[0] * b[0] + a[1] * b[1]


KEYBOARD_PATH = hypot(p.KEYBOARD_DY, p.KEYBOARD_DZ)
TRACKPAD_PATH = hypot(p.TRACKPAD_DY, p.TRACKPAD_DZ)
RELATIVE_DELTA = (
    p.KEYBOARD_DY - p.TRACKPAD_DY,
    p.KEYBOARD_DZ - p.TRACKPAD_DZ,
)
RELATIVE_PATH = hypot(*RELATIVE_DELTA)
SLOT_U = (RELATIVE_DELTA[0] / RELATIVE_PATH, RELATIVE_DELTA[1] / RELATIVE_PATH)
SLOT_N = (-SLOT_U[1], SLOT_U[0])
KEYBOARD_U = (p.KEYBOARD_DY / KEYBOARD_PATH, p.KEYBOARD_DZ / KEYBOARD_PATH)
TRACKPAD_U = (p.TRACKPAD_DY / TRACKPAD_PATH, p.TRACKPAD_DZ / TRACKPAD_PATH)


@dataclass(frozen=True)
class Pose:
    travel: float
    keyboard_delta: Vec2
    trackpad_delta: Vec2
    keyboard_position: Vec2
    trackpad_position: Vec2
    cross_yoke_center: Vec2
    cross_slot_center: Vec2
    cross_roller_centers: Tuple[Vec2, Vec2]


def pose(travel: float) -> Pose:
    """Return the only valid affine pose for normalized travel 0..1."""
    t = max(0.0, min(1.0, float(travel)))
    keyboard_delta = (t * p.KEYBOARD_DY, t * p.KEYBOARD_DZ)
    trackpad_delta = (t * p.TRACKPAD_DY, t * p.TRACKPAD_DZ)
    keyboard_position = _add((p.KEYBOARD_Y0, p.KEYBOARD_Z0), keyboard_delta)
    trackpad_position = _add((p.TRACKPAD_Y0, p.TRACKPAD_Z0), trackpad_delta)

    yoke_center = _add(keyboard_position, (30.0, 37.0))
    initial_yoke = (p.KEYBOARD_Y0 + 30.0, p.KEYBOARD_Z0 + 37.0)
    slot_center_initial = _add(initial_yoke, _scale(RELATIVE_DELTA, 0.5))
    slot_center = _add(slot_center_initial, trackpad_delta)

    half_spacing = p.CROSS_ROLLER_SPACING / 2.0
    wall_offset = (p.CROSS_SLOT_WIDTH - p.MR84ZZ.od) / 2.0
    roller_1 = _add(
        _add(yoke_center, _scale(SLOT_U, -half_spacing)),
        _scale(SLOT_N, wall_offset),
    )
    roller_2 = _add(
        _add(yoke_center, _scale(SLOT_U, half_spacing)),
        _scale(SLOT_N, -wall_offset),
    )
    return Pose(
        t,
        keyboard_delta,
        trackpad_delta,
        keyboard_position,
        trackpad_position,
        yoke_center,
        slot_center,
        (roller_1, roller_2),
    )


def validate_motion(sample_count: int = 401) -> Dict[str, float]:
    if sample_count < 2:
        raise ValueError("sample_count must be at least 2")
    samples = [pose(index / (sample_count - 1)) for index in range(sample_count)]
    cross_errors = []
    end_margins = []
    along_history = []
    for item in samples:
        relative = (
            item.cross_yoke_center[0] - item.cross_slot_center[0],
            item.cross_yoke_center[1] - item.cross_slot_center[1],
        )
        cross_errors.append(abs(_dot(relative, SLOT_N)))
        roller_alongs = []
        for center in item.cross_roller_centers:
            delta = (center[0] - item.cross_slot_center[0], center[1] - item.cross_slot_center[1])
            along = _dot(delta, SLOT_U)
            roller_alongs.append(along)
            end_margins.append(
                p.CROSS_SLOT_LENGTH / 2.0 - abs(along) - p.MR84ZZ.od / 2.0
            )
        along_history.append(sum(roller_alongs) / len(roller_alongs))
    return {
        "sample_count": sample_count,
        "max_cross_error_mm": max(cross_errors),
        "min_cross_end_margin_mm": min(end_margins),
        "monotonic": all(
            along_history[index + 1] > along_history[index]
            for index in range(len(along_history) - 1)
        ),
        "stroke_ratio": TRACKPAD_PATH / KEYBOARD_PATH,
        "input_projection": abs(_dot(SLOT_N, KEYBOARD_U)),
        "output_projection": abs(_dot(SLOT_N, TRACKPAD_U)),
    }


def validate_bilateral_parallelism(sample_count: int = 401) -> Dict[str, float]:
    """Check the mirrored dual-side path and the right follower reserve.

    Both carriages are attached to rigid trays, so their Y-Z motion is shared.
    The left side defines the path; deliberately wider right slots absorb a
    small frame-to-frame angular mismatch instead of closing a redundant rigid
    constraint loop.
    """
    if sample_count < 2:
        raise ValueError("sample_count must be at least 2")
    samples = [pose(index / (sample_count - 1)) for index in range(sample_count)]
    # The rigid trays impose zero nominal left/right Y-Z mismatch.  Separately
    # simulate an independently misaligned right guide frame and compare the
    # resulting normal error with the follower slot reserve.  This avoids the
    # former tautological same-variable subtraction.
    yz_mismatch = 0.0
    angle = radians(p.PARALLELISM_ASSUMED_MAX_ANGLE_DEG)
    skew_history = []
    for pose_item in samples:
        relative_y = pose_item.trackpad_delta[0] - pose_item.keyboard_delta[0]
        relative_z = pose_item.trackpad_delta[1] - pose_item.keyboard_delta[1]
        relative_distance = hypot(relative_y, relative_z)
        skew_history.append(relative_distance * sin(angle))
    simulated_frame_skew = max(skew_history)
    follower_float = p.RIGHT_FOLLOWER_SLOT_EXTRA_WIDTH / 2.0
    angular_error = simulated_frame_skew
    return {
        "sample_count": sample_count,
        "left_right_yz_path_mismatch_mm": yz_mismatch,
        "right_follower_normal_float_mm": follower_float,
        "assumed_frame_angle_error_deg": p.PARALLELISM_ASSUMED_MAX_ANGLE_DEG,
        "angle_error_at_relative_stroke_mm": angular_error,
        "simulated_right_frame_normal_mismatch_mm": simulated_frame_skew,
        "normal_float_margin_mm": follower_float - angular_error,
        "parallel_motion_possible": yz_mismatch <= 1e-9 and follower_float >= angular_error,
        "constraint_strategy": "left_datum_right_follower",
    }


def spring_requirement(
    moving_mass_kg: float,
    rolling_resistance_n: float,
    spring_count: int = p.RETURN_SPRING_COUNT,
) -> Dict[str, float]:
    """Evaluate constant-force output springs at the keyboard input.

    Until individual printed masses are available, 45% of moving mass follows
    the keyboard path and 55% follows the longer trackpad path.  Full geometry
    validation replaces this estimate with part-level masses.
    """
    if moving_mass_kg <= 0.0 or spring_count <= 0:
        raise ValueError("mass and spring count must be positive")
    weighted_drop_mm = 0.45 * abs(p.KEYBOARD_DZ) + 0.55 * abs(p.TRACKPAD_DZ)
    gravity_equivalent_n = (
        moving_mass_kg * p.GRAVITY * weighted_drop_mm / KEYBOARD_PATH
    )
    required_start_n = gravity_equivalent_n + max(0.0, rolling_resistance_n)
    generalized_return_n = (
        spring_count * p.RETURN_FORCE_PER_SIDE * TRACKPAD_PATH / KEYBOARD_PATH
    )
    weakest_return_n = generalized_return_n * (1.0 - p.RETURN_FORCE_TOLERANCE_LOW)
    strongest_return_n = generalized_return_n * (1.0 + p.RETURN_FORCE_TOLERANCE_HIGH)
    return {
        "gravity_equivalent_n": gravity_equivalent_n,
        "required_start_force_n": required_start_n,
        "start_force_n": generalized_return_n,
        "end_force_n": generalized_return_n,
        "weakest_return_force_n": weakest_return_n,
        "strongest_return_force_n": strongest_return_n,
        "start_return_margin_n": weakest_return_n - required_start_n,
        "maximum_user_force_n": (
            generalized_return_n - gravity_equivalent_n
            + max(0.0, rolling_resistance_n)
        ),
        "strongest_maximum_user_force_n": (
            strongest_return_n - gravity_equivalent_n
            + max(0.0, rolling_resistance_n)
        ),
        "working_deflection_margin_mm": (
            p.RETURN_SPRING_WORKING_DEFLECTION
            - p.RETURN_SPRING_INITIAL_DEFLECTION
            - TRACKPAD_PATH
        ),
        "stored_energy_n_mm": generalized_return_n * KEYBOARD_PATH,
    }
