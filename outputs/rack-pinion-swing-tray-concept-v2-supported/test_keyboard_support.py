"""Regression contract for the supported V2 keyboard tray."""

import unittest

try:
    import rack_pinion_swing_supported_concept as concept
except ImportError:
    concept = None


POSES = tuple(index / 10.0 for index in range(11))
POSITIONS = ("left_front", "right_front", "left_rear", "right_rear")
MAX_BOOLEAN_NOISE_VOLUME = 0.05


class KeyboardSupportTests(unittest.TestCase):
    def test_supported_concept_module_exists(self):
        self.assertIsNotNone(concept)

    def test_four_fixed_guides_and_four_moving_carriages_are_selectable(self):
        self.assertIsNotNone(concept)
        for t in POSES:
            parts = {
                child.label: child
                for child in concept.build_pose(t, f"support_test_{t}").children
            }
            for position in POSITIONS:
                self.assertIn(f"fixed_keyboard_guide_{position}", parts)
                self.assertIn(f"keyboard_carriage_{position}", parts)
                self.assertGreater(parts[f"fixed_keyboard_guide_{position}"].volume, 1.0)
                self.assertGreater(parts[f"keyboard_carriage_{position}"].volume, 1.0)

    def test_each_carriage_follows_the_keyboard_translation_exactly(self):
        self.assertIsNotNone(concept)
        for position in POSITIONS:
            x0, y0, z0 = concept.carriage_pin_center(position, 0.0)
            x1, y1, z1 = concept.carriage_pin_center(position, 1.0)
            self.assertAlmostEqual(x1 - x0, 0.0, places=6)
            self.assertAlmostEqual(y1 - y0, concept.v1.KEYBOARD_TRAVEL_Y, places=6)
            self.assertAlmostEqual(z1 - z0, concept.v1.KEYBOARD_TRAVEL_Z, places=6)

    def test_guides_are_grounded_and_carriages_run_without_solid_penetration(self):
        self.assertIsNotNone(concept)
        for t in POSES:
            parts = {
                child.label: child
                for child in concept.build_pose(t, f"support_test_{t}").children
            }
            for position in POSITIONS:
                guide = parts[f"fixed_keyboard_guide_{position}"]
                carriage = parts[f"keyboard_carriage_{position}"]
                self.assertAlmostEqual(guide.bounding_box().min.Z, concept.v1.BASE_TOP_Z, places=5)
                self.assertLessEqual((guide & carriage).volume, MAX_BOOLEAN_NOISE_VOLUME)

    def test_supports_do_not_intersect_drive_housing_rack_or_trackpad(self):
        self.assertIsNotNone(concept)
        for t in POSES:
            parts = {
                child.label: child
                for child in concept.build_pose(t, f"support_test_{t}").children
            }
            protected = (
                "fixed_protective_cylinder",
                "keyboard_right_straight_rack",
                "magic_trackpad_small_tray",
            )
            for position in POSITIONS:
                for support_label in (
                    f"fixed_keyboard_guide_{position}",
                    f"keyboard_carriage_{position}",
                ):
                    for protected_label in protected:
                        self.assertLessEqual(
                            (parts[support_label] & parts[protected_label]).volume,
                            MAX_BOOLEAN_NOISE_VOLUME,
                            f"{support_label} intersects {protected_label} at t={t}",
                        )


if __name__ == "__main__":
    unittest.main()
