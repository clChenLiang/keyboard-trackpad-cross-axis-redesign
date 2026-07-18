import math
import tempfile
import unittest
from pathlib import Path

import ap_coupon
from ap_coupon import (
    BELT_PITCH,
    DRUM_PITCH_RADIUS,
    FEED_TRAVEL,
    Z_FLOAT_TRAVEL,
    build_pose,
    engaged_tooth_count,
    interference_report,
    kinematics,
    posed_belt_length,
    required_labels,
    roundtrip_step,
)


class APCouponRegressionTests(unittest.TestCase):
    def test_polar_box_keeps_requested_polar_center_after_local_rotation(self):
        box = ap_coupon._polar_box(10.0, math.pi / 2.0, 1.0, 2.0, 3.0, 7.0)
        center = box.bounding_box().center()
        self.assertAlmostEqual(center.X, 0.0, places=7)
        self.assertAlmostEqual(center.Y, 10.0, places=7)
        self.assertAlmostEqual(center.Z, 7.0, places=7)

    def test_stored_brep_wrap_measures_a_quarter_turn_not_a_half_turn(self):
        self.assertTrue(hasattr(ap_coupon, "measured_wrap_angle_deg"))
        self.assertAlmostEqual(ap_coupon.measured_wrap_angle_deg(1.0), 90.0, delta=0.5)

    def test_stored_step_roundtrip_measures_a_quarter_turn(self):
        self.assertTrue(hasattr(ap_coupon, "measure_step_wrap_angle_deg"))
        with tempfile.TemporaryDirectory() as tmp:
            step_path = Path(tmp) / "stored.step"
            result = roundtrip_step(build_pose(1.0), step_path)
            self.assertTrue(result.readable)
            self.assertAlmostEqual(ap_coupon.measure_step_wrap_angle_deg(step_path), 90.0, delta=0.5)

    def test_2mm_synchronous_pulley_uses_41_integer_teeth(self):
        self.assertEqual(getattr(ap_coupon, "SYNCHRONOUS_PROFILE", None), "2M-GT2")
        self.assertEqual(getattr(ap_coupon, "PULLEY_TOOTH_COUNT", None), 41)
        expected_pitch_diameter = 41 * BELT_PITCH / math.pi
        self.assertAlmostEqual(ap_coupon.DRUM_PITCH_DIAMETER, expected_pitch_diameter, places=9)
        self.assertAlmostEqual(ap_coupon.DRUM_PITCH_DIAMETER, 26.10, delta=0.01)
        self.assertAlmostEqual(2.0 * ap_coupon.DRUM_OUTER_RADIUS, expected_pitch_diameter - 0.50, delta=0.02)

    def test_stored_engagement_is_proven_by_brep_groove_phase(self):
        self.assertTrue(hasattr(ap_coupon, "brep_engagement_report"))
        report = ap_coupon.brep_engagement_report(1.0)
        self.assertEqual(report.pulley_grooves, 41)
        self.assertGreaterEqual(report.meshed_teeth, 5)
        self.assertEqual(report.phase_misses, 0)
        self.assertLessEqual(report.pulley_interference_volume, 1e-5)

    def test_stops_touch_only_at_their_respective_endpoints(self):
        self.assertTrue(hasattr(ap_coupon, "stop_contact_report"))
        for index in range(11):
            travel = index / 10.0
            report = ap_coupon.stop_contact_report(travel)
            self.assertLessEqual(report.penetration_volume, 1e-5)
            if index == 0:
                self.assertLessEqual(report.zero_stop_gap, 1e-5)
                self.assertGreater(report.ninety_stop_gap, 0.1)
            elif index == 10:
                self.assertLessEqual(report.ninety_stop_gap, 1e-5)
                self.assertGreater(report.zero_stop_gap, 0.1)
            else:
                self.assertGreater(report.zero_stop_gap, 0.1)
                self.assertGreater(report.ninety_stop_gap, 0.1)

    def test_floating_keys_transmit_y_while_remaining_free_in_z(self):
        self.assertTrue(hasattr(ap_coupon, "guide_contact_report"))
        for index in range(11):
            report = ap_coupon.guide_contact_report(index / 10.0)
            self.assertLessEqual(report.y_contact_gap, 1e-5)
            self.assertLessEqual(report.penetration_volume, 1e-5)
            self.assertGreaterEqual(report.z_capture_above, 1.0)
            self.assertGreaterEqual(report.z_capture_below, 1.0)

    def test_fixed_features_are_brep_connected_to_the_housing(self):
        self.assertTrue(hasattr(ap_coupon, "fixed_connectivity_report"))
        report = ap_coupon.fixed_connectivity_report()
        self.assertEqual(
            set(report),
            {
                "closed_entry_guide",
                "upper_bearing_seat",
                "lower_bearing_seat",
                "lower_thrust_interface",
                "hard_stops_0_90",
            },
        )
        self.assertTrue(all(report.values()), report)

    def test_endpoint_motion_isolated_from_belt_feed(self):
        for travel, expected_angle in [(0.0, 0.0), (0.5, 45.0), (1.0, 90.0)]:
            with self.subTest(travel=travel):
                state = kinematics(travel)
                self.assertAlmostEqual(state.carrier_dy, FEED_TRAVEL * travel, places=9)
                self.assertAlmostEqual(state.carrier_dz, -Z_FLOAT_TRAVEL * travel, places=9)
                self.assertAlmostEqual(state.clamp_dy, FEED_TRAVEL * travel, places=9)
                self.assertAlmostEqual(state.clamp_dz, 0.0, places=9)
                self.assertAlmostEqual(state.angle_deg, expected_angle, delta=0.01)

    def test_41t_pitch_radius_converts_20_5_mm_to_quarter_turn(self):
        exact_quarter_turn_feed = math.pi * DRUM_PITCH_RADIUS / 2.0
        self.assertAlmostEqual(FEED_TRAVEL, exact_quarter_turn_feed, delta=0.001)

    def test_posed_belt_centerline_length_does_not_change(self):
        lengths = [posed_belt_length(i / 10) for i in range(11)]
        self.assertLess(max(lengths) - min(lengths), 1e-8)

    def test_at_least_five_transverse_teeth_are_engaged_at_stored_pose(self):
        self.assertAlmostEqual(BELT_PITCH, 2.0)
        self.assertGreaterEqual(engaged_tooth_count(1.0), 5)

    def test_pose_is_a_labelled_positive_volume_assembly(self):
        for travel in (0.0, 0.5, 1.0):
            with self.subTest(travel=travel):
                assembly = build_pose(travel)
                labels = {child.label for child in assembly.children}
                self.assertTrue(required_labels() <= labels)
                self.assertTrue(all(child.is_valid() for child in assembly.children))
                self.assertTrue(all(child.volume > 0.0 for child in assembly.children))

    def test_eleven_pose_interference_sweep_has_no_unintended_overlap(self):
        report = interference_report(samples=11)
        self.assertEqual(report.samples, 11)
        self.assertGreaterEqual(report.checked_pairs, 800)
        self.assertEqual(report.unintended, [])

    def test_step_roundtrip_preserves_readable_solids(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = roundtrip_step(build_pose(0.5), Path(tmp) / "roundtrip.step")
        self.assertTrue(result.readable)
        self.assertGreaterEqual(result.solid_count, len(required_labels()))


if __name__ == "__main__":
    unittest.main()
