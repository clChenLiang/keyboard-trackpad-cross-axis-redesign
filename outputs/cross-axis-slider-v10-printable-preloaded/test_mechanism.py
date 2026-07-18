import unittest

from mechanism import pose, spring_requirement, validate_motion


class MotionTests(unittest.TestCase):
    def test_end_positions_and_constraint(self):
        self.assertEqual((30.0, -15.0), pose(1.0).keyboard_delta)
        self.assertEqual((-115.0, -43.0), pose(1.0).trackpad_delta)
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
        self.assertLessEqual(result["maximum_user_force_n"], 20.0)
        self.assertGreaterEqual(result["coil_bind_margin_mm"], 2.0)


if __name__ == "__main__":
    unittest.main()
