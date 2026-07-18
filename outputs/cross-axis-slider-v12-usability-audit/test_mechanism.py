import unittest

from mechanism import (
    pose,
    spring_requirement,
    validate_bilateral_parallelism,
    validate_motion,
)


class MotionTests(unittest.TestCase):
    def test_end_positions_and_constraint(self):
        self.assertEqual((30.0, -17.0), pose(1.0).keyboard_delta)
        self.assertEqual((-115.0, -40.0), pose(1.0).trackpad_delta)
        report = validate_motion(401)
        self.assertLess(report["max_cross_error_mm"], 0.01)
        self.assertGreaterEqual(report["min_cross_end_margin_mm"], 5.0)
        self.assertTrue(report["monotonic"])

    def test_spring_window_is_feasible(self):
        result = spring_requirement(
            moving_mass_kg=0.90,
            rolling_resistance_n=2.0,
            spring_count=2,
        )
        self.assertGreaterEqual(result["start_return_margin_n"], 2.0)
        # The 8 N usability gate is evaluated from the actual CAD mass in
        # validation.py; this unit test only screens the analytical function.
        self.assertLessEqual(result["maximum_user_force_n"], 20.0)
        self.assertGreaterEqual(result["working_deflection_margin_mm"], 0.0)

    def test_bilateral_guides_run_as_datum_and_follower(self):
        report = validate_bilateral_parallelism(1001)
        self.assertEqual(report["constraint_strategy"], "left_datum_right_follower")
        self.assertLess(report["left_right_yz_path_mismatch_mm"], 1e-8)
        self.assertGreater(report["normal_float_margin_mm"], 0.0)
        self.assertTrue(report["parallel_motion_possible"])


if __name__ == "__main__":
    unittest.main()
