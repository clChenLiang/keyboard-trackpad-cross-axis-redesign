import unittest

from parts import calibration_coupon, make_eccentric_bushing, printable_parts
from assembly import build_pose, roller_stack_report
from roller_detail import build_roller_detail
from validation import validate_spring


class GeometryTests(unittest.TestCase):
    def test_every_printable_is_one_valid_solid(self):
        for name, part in printable_parts().items():
            self.assertEqual(1, len(part.solids()), name)
            self.assertTrue(part.is_valid(), name)
            self.assertGreater(part.volume, 0.0, name)

    def test_coupon_contains_required_interfaces(self):
        coupon = calibration_coupon()
        self.assertEqual(1, len(coupon.solids()))
        self.assertTrue(coupon.is_valid())
        self.assertGreater(coupon.volume, 0.0)

    def test_eccentric_bushing_has_exposed_adjustment_flange(self):
        for kind in ("guide", "cross"):
            bushing = make_eccentric_bushing(kind)
            bbox = bushing.bounding_box()
            self.assertEqual(1, len(bushing.solids()))
            self.assertTrue(bushing.is_valid())
            self.assertGreaterEqual(bbox.size.X, 6.0)
            self.assertGreaterEqual(max(bbox.size.Y, bbox.size.Z), 11.5)

    def test_required_interfaces_touch_or_overlap(self):
        for travel in (0.0, 0.5, 1.0):
            assembly = build_pose(travel)
            report = assembly.interface_report()
            self.assertEqual([], report["disconnected"], report)
            self.assertGreaterEqual(report["minimum_cross_end_margin_mm"], 5.0)
            self.assertGreaterEqual(report["minimum_stop_before_slot_end_mm"], 2.0)

    def test_opposed_rollers_contact_opposite_walls(self):
        for travel in (0.0, 0.5, 1.0):
            report = build_pose(travel).preload_report()
            self.assertLessEqual(report["maximum_adjusted_wall_gap_mm"], 0.05)
            self.assertGreaterEqual(report["minimum_adjustment_reserve_mm"], 0.10)

    def test_hard_stops_contact_before_slot_ends(self):
        for travel, active in ((0.0, "return"), (1.0, "pressed")):
            report = build_pose(travel).stop_report()
            self.assertEqual(active, report["active_stop"])
            self.assertLessEqual(report["active_stop_distance_mm"], 0.05)
            self.assertLessEqual(report["active_stop_overlap_mm3"], 0.05)
            self.assertGreaterEqual(report["bearing_end_margin_mm"], 2.0)

    def test_hard_stops_clear_the_carriage_between_endpoints(self):
        for travel in (0.1, 0.25, 0.5, 0.75, 0.9):
            pose = build_pose(travel)
            for side in ("left", "right"):
                carriage = pose.bodies[f"keyboard_carriage_{side}"]
                for stop in ("return", "pressed"):
                    collar = pose.bodies[f"fixed_stop_collar_{side}_{stop}"]
                    self.assertLessEqual((carriage & collar).volume, 0.01)
                    self.assertGreater(carriage.distance_to(collar), 0.05)

    def test_each_roller_stack_has_axial_retention(self):
        pose = build_pose(0.5)
        for side in ("left", "right"):
            for prefix, count in (("keyboard_guide", 2), ("trackpad_guide", 2), ("cross", 2)):
                for index in range(1, count + 1):
                    for suffix in ("bearing", "shaft", "retaining_washer", "retaining_nut"):
                        self.assertIn(f"{prefix}_{suffix}_{side}_{index}", pose.bodies)

    def test_shoulder_lengths_are_exact_and_bilateral(self):
        report = roller_stack_report()
        expected = {
            "keyboard_guide": 20.0,
            "trackpad_guide": 16.0,
            "cross": 12.0,
        }
        for kind, length in expected.items():
            for actual in report["lengths_mm"][kind]:
                self.assertAlmostEqual(length, actual, places=6)
            self.assertLessEqual(report["bilateral_mismatch_mm"][kind], 1e-6)
        self.assertEqual(4.0, report["threaded_tip_mm"])
        self.assertEqual(6.0, report["head_diameter_mm"])
        self.assertEqual(2.0, report["head_thickness_mm"])

    def test_spring_guide_rods_have_real_clearance_holes(self):
        for travel in (0.0, 0.5, 1.0):
            pose = build_pose(travel)
            for side in ("left", "right"):
                rod = pose.bodies[f"spring_guide_rod_{side}"]
                frame = pose.bodies[f"fixed_side_frame_{side}"]
                carriage = pose.bodies[f"keyboard_carriage_{side}"]
                self.assertLessEqual((rod & frame).volume, 0.01)
                self.assertLessEqual((rod & carriage).volume, 0.01)

    def test_spring_force_stress_and_guide_are_consistent(self):
        report = validate_spring()
        self.assertTrue(report["returns_with_10_percent_mismatch"], report)
        self.assertLessEqual(report["maximum_user_force_n"], 20.0, report)
        self.assertLessEqual(report["strongest_10_percent_maximum_user_force_n"], 23.0, report)
        self.assertGreaterEqual(report["coil_bind_margin_mm"], 2.0, report)
        self.assertGreaterEqual(report["spring_shear_safety_factor"], 1.25, report)
        self.assertLessEqual(report["rate_error_fraction"], 0.05, report)
        self.assertGreaterEqual(report["guide_sleeve_diametral_clearance_mm"], 0.4, report)
        self.assertAlmostEqual(122.0, report["guide_rod_length_mm"], places=6)

    def test_roller_detail_has_selectable_complete_stacks(self):
        for exploded in (False, True):
            detail = build_roller_detail(exploded=exploded)
            self.assertGreaterEqual(len(detail.children), 18)
            labels = {child.label for child in detail.children}
            for stack in ("guide_fixed", "guide_eccentric", "cross_eccentric"):
                for suffix in ("bearing", "shaft", "retaining_washer", "retaining_nut"):
                    self.assertIn(f"{stack}_{suffix}", labels)


if __name__ == "__main__":
    unittest.main()
