"""Regression tests for the concept's two user-reported physical failures."""

import unittest

import rack_pinion_swing_concept as concept


POSES = (0.0, 0.5, 1.0)
MAX_BOOLEAN_NOISE_VOLUME = 0.05


def labeled_parts(t):
    assembly = concept.build_pose(t, f"physical_test_{t}")
    return {child.label: child for child in assembly.children}


class PhysicalInterfaceTests(unittest.TestCase):
    def test_keyboard_and_trackpad_trays_do_not_intersect_fixed_cylinder(self):
        for t in POSES:
            with self.subTest(t=t):
                parts = labeled_parts(t)
                for tray_label in (
                    "keyboard_tray_continuous",
                    "magic_trackpad_small_tray",
                ):
                    intersection = (
                        parts[tray_label] & parts["fixed_protective_cylinder"]
                    )
                    self.assertLessEqual(
                        intersection.volume,
                        MAX_BOOLEAN_NOISE_VOLUME,
                        f"{tray_label} intersects cylinder at t={t}",
                    )

    def test_rack_and_pinion_mesh_without_solid_penetration(self):
        for t in POSES:
            with self.subTest(t=t):
                parts = labeled_parts(t)
                intersection = (
                    parts["keyboard_right_straight_rack"]
                    & parts["module_1_26t_pinion"]
                )
                self.assertLessEqual(
                    intersection.volume,
                    MAX_BOOLEAN_NOISE_VOLUME,
                    f"rack and pinion penetrate at t={t}",
                )
                surface_gap = parts[
                    "keyboard_right_straight_rack"
                ].distance_to(parts["module_1_26t_pinion"])
                self.assertGreaterEqual(
                    surface_gap,
                    0.01,
                    f"rack/pinion backlash closes at t={t}",
                )
                self.assertLessEqual(
                    surface_gap,
                    0.15,
                    f"rack/pinion are too far apart at t={t}",
                )

    def test_moving_rack_clears_fixed_cylinder_and_mount(self):
        for t in POSES:
            with self.subTest(t=t):
                parts = labeled_parts(t)
                intersection = (
                    parts["keyboard_right_straight_rack"]
                    & parts["fixed_protective_cylinder"]
                )
                self.assertLessEqual(
                    intersection.volume,
                    MAX_BOOLEAN_NOISE_VOLUME,
                    f"moving rack intersects fixed housing at t={t}",
                )


if __name__ == "__main__":
    unittest.main()
