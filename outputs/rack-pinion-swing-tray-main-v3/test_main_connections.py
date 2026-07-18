"""Regression contract for V3 physical connections and service access."""

import gc
import unittest

try:
    import rack_pinion_swing_main_v3 as concept
except ImportError:
    concept = None


POSES = tuple(index / 10.0 for index in range(11))
POSITIONS = ("left_front", "right_front", "left_rear", "right_rear")
MAX_BOOLEAN_NOISE_VOLUME = 0.05


class MainConnectionTests(unittest.TestCase):
    def tearDown(self):
        # OpenCascade shapes can retain large native allocations until an
        # explicit collection between pose-heavy regression cases.
        gc.collect()

    def test_v3_module_exists(self):
        self.assertIsNotNone(concept)

    def test_preserves_v2_mechanism_labels_at_eleven_poses(self):
        required = {
            "keyboard_tray_continuous",
            "magic_trackpad_small_tray",
            "keyboard_right_straight_rack",
            "module_1_26t_pinion",
            "vertical_drive_shaft",
            "eccentric_horizontal_swing_arm",
            "fixed_protective_cylinder",
        }
        for t in POSES:
            labels = {child.label for child in concept.build_pose(t, f"v3_{t}").children}
            self.assertTrue(required.issubset(labels))
            for position in POSITIONS:
                self.assertIn(f"fixed_keyboard_guide_{position}", labels)
                self.assertIn(f"keyboard_carriage_{position}", labels)
            gc.collect()

    def test_four_selectable_carriage_mount_ears_have_real_fastener_bores(self):
        for t in POSES:
            parts = {child.label: child for child in concept.build_pose(t, f"ears_{t}").children}
            for position in POSITIONS:
                ear = parts[f"carriage_tray_mount_ear_{position}"]
                self.assertGreater(ear.volume, 20.0)
                self.assertLess(ear.volume, concept.CARRIAGE_EAR_GROSS_VOLUME)
            del parts
            gc.collect()

    def test_mount_ears_follow_keyboard_translation_exactly(self):
        for position in POSITIONS:
            p0 = concept.carriage_mount_ear_center(position, 0.0)
            p1 = concept.carriage_mount_ear_center(position, 1.0)
            self.assertAlmostEqual(p1[0] - p0[0], 0.0, places=6)
            self.assertAlmostEqual(p1[1] - p0[1], concept.v2.v1.KEYBOARD_TRAVEL_Y, places=6)
            self.assertAlmostEqual(p1[2] - p0[2], concept.v2.v1.KEYBOARD_TRAVEL_Z, places=6)

    def test_four_selectable_guide_mount_feet_are_grounded_and_drilled(self):
        parts = {child.label: child for child in concept.build_pose(0.5, "feet").children}
        for position in POSITIONS:
            foot = parts[f"fixed_guide_mount_foot_{position}"]
            self.assertAlmostEqual(foot.bounding_box().min.Z, concept.v2.v1.BASE_TOP_Z, places=5)
            self.assertGreater(foot.volume, 50.0)
            self.assertLess(foot.volume, concept.GUIDE_FOOT_GROSS_VOLUME)

    def test_base_has_eight_matching_through_holes_for_guide_feet(self):
        parts = {child.label: child for child in concept.build_pose(0.5, "base_holes").children}
        base = parts["continuous_main_base"]
        for position in POSITIONS:
            hole_tools = concept.guide_base_mount_hole_tools(position)
            self.assertLessEqual((base & hole_tools).volume, MAX_BOOLEAN_NOISE_VOLUME)

    def test_cover_and_rack_transition_are_selectable_in_all_poses(self):
        for t in POSES:
            parts = {child.label: child for child in concept.build_pose(t, f"service_{t}").children}
            for label in ("fixed_cylinder_removable_top_cover", "rack_to_keyboard_rigid_transition"):
                self.assertIn(label, parts)
                self.assertGreater(parts[label].volume, 10.0)
            del parts
            gc.collect()

    def test_removing_cover_exposes_real_top_service_aperture(self):
        self.assertTrue(hasattr(concept, "cylinder_top_service_aperture_probe"))
        parts = {child.label: child for child in concept.build_pose(0.5, "open_top").children}
        aperture_probe = concept.cylinder_top_service_aperture_probe()
        self.assertLessEqual(
            (parts["fixed_protective_cylinder"] & aperture_probe).volume,
            MAX_BOOLEAN_NOISE_VOLUME,
        )

    def test_cover_fastener_interface_clears_swing_arm_at_eleven_poses(self):
        for t in POSES:
            parts = {child.label: child for child in concept.build_pose(t, f"cover_clearance_{t}").children}
            self.assertLessEqual(
                (
                    parts["cylinder_top_cover_fastener_interface"]
                    & parts["eccentric_horizontal_swing_arm"]
                ).volume,
                MAX_BOOLEAN_NOISE_VOLUME,
            )
            del parts
            gc.collect()

    def test_keyboard_tray_has_four_matching_holes_and_selectable_bushings(self):
        self.assertTrue(hasattr(concept, "keyboard_tray_mount_hole_tool"))
        for t in POSES:
            parts = {child.label: child for child in concept.build_pose(t, f"tray_interfaces_{t}").children}
            tray = parts["keyboard_tray_continuous"]
            for position in POSITIONS:
                hole = concept.keyboard_tray_mount_hole_tool(position, t)
                self.assertLessEqual((tray & hole).volume, MAX_BOOLEAN_NOISE_VOLUME)
                bushing = parts[f"keyboard_tray_mount_bushing_{position}"]
                self.assertGreater(bushing.volume, 10.0)
                self.assertLessEqual((bushing & hole).volume, MAX_BOOLEAN_NOISE_VOLUME)
            del parts
            gc.collect()

    def test_new_parts_preserve_key_no_penetration_relationships(self):
        for t in POSES:
            parts = {child.label: child for child in concept.build_pose(t, f"clearance_{t}").children}
            for position in POSITIONS:
                ear = parts[f"carriage_tray_mount_ear_{position}"]
                guide = parts[f"fixed_keyboard_guide_{position}"]
                self.assertLessEqual((ear & guide).volume, MAX_BOOLEAN_NOISE_VOLUME)
            transition = parts["rack_to_keyboard_rigid_transition"]
            for fixed_label in ("fixed_protective_cylinder", "module_1_26t_pinion"):
                self.assertLessEqual((transition & parts[fixed_label]).volume, MAX_BOOLEAN_NOISE_VOLUME)
            del parts
            gc.collect()

    def test_exploded_view_includes_service_parts(self):
        labels = {child.label for child in concept.build_section_exploded().children}
        self.assertIn("fixed_cylinder_removable_top_cover_exploded", labels)
        self.assertIn("cylinder_top_cover_fastener_interface", labels)


if __name__ == "__main__":
    unittest.main()
