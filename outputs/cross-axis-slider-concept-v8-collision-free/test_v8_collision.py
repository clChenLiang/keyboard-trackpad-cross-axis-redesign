import unittest

import cross_axis_slider_concept_v8 as v8


class V8MotionTests(unittest.TestCase):
    def test_exact_slot_kinematics(self):
        facts = v8.validate_kinematics(201)
        self.assertLess(facts["max_cross_error"], 1e-8)
        self.assertGreater(facts["minimum_roller_center_margin"], 0.0)
        self.assertTrue(facts["roller_monotonic"])
        self.assertTrue(facts["spring_monotonic"])

    def test_no_forbidden_rigid_body_collisions(self):
        report = v8.validate_collisions(21)
        self.assertEqual([], report["collisions"])


if __name__ == "__main__":
    unittest.main()
