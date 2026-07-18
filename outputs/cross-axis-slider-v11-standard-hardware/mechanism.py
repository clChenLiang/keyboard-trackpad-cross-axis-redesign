"""Constraint-derived motion and force calculations for V11."""

from dataclasses import dataclass
from math import hypot
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


def spring_requirement(
    moving_mass_kg: float,
    rolling_resistance_n: float,
    spring_count: int = p.SPRING_COUNT,
) -> Dict[str, float]:
    """Evaluate the exact baseline spring against generalized input force.

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
    start_force_n = spring_count * p.SPRING_PRELOAD_PER_SPRING
    end_force_n = start_force_n + (
        spring_count * p.SPRING_RATE_PER_SPRING * KEYBOARD_PATH
    )
    compressed_length = p.SPRING_INSTALLED_LENGTH - KEYBOARD_PATH
    return {
        "gravity_equivalent_n": gravity_equivalent_n,
        "required_start_force_n": required_start_n,
        "start_force_n": start_force_n,
        "end_force_n": end_force_n,
        "start_return_margin_n": start_force_n - required_start_n,
        "maximum_user_force_n": end_force_n - gravity_equivalent_n + max(0.0, rolling_resistance_n),
        "coil_bind_margin_mm": compressed_length - p.SPRING_SOLID_LENGTH,
        "stored_energy_n_mm": (
            start_force_n * KEYBOARD_PATH
            + 0.5 * spring_count * p.SPRING_RATE_PER_SPRING * KEYBOARD_PATH ** 2
        ),
    }
