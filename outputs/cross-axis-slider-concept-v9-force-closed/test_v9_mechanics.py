import unittest

import cross_axis_slider_concept_v9 as v9


class V9MechanicsTests(unittest.TestCase):
    def test_exact_kinematics_and_virtual_work(self):
        facts = v9.validate_kinematics(201)
        self.assertLess(facts["max_cross_error"], 1e-8)
        self.assertGreaterEqual(facts["minimum_output_projection"], 0.16)
        self.assertAlmostEqual(facts["stroke_ratio"], 3.6604796286, places=8)
        self.assertTrue(facts["roller_monotonic"])

    def test_required_force_path_is_connected(self):
        for travel in (0.0, 0.5, 1.0):
            report = v9.validate_force_closure(travel)
            self.assertEqual([], report["failures"], report)

    def test_no_forbidden_collisions(self):
        report = v9.validate_collisions(21)
        self.assertEqual([], report["collisions"], report)


if __name__ == "__main__":
    unittest.main()
