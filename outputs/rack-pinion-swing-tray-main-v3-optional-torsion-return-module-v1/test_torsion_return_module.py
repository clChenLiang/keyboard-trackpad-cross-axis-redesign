import unittest

try:
    import torsion_return_module as concept
except ImportError:
    concept = None


class TorsionReturnModuleTests(unittest.TestCase):
    def test_module_exists(self):
        self.assertIsNotNone(concept)

    def test_selectable_real_parts_exist(self):
        labels = {part.label for part in concept.build_module(0.0).children}
        self.assertEqual(labels, {
            "torsion_fixed_mounting_cup",
            "torsion_removable_bottom_cover",
            "torsion_helical_spring",
            "torsion_rotating_shaft_clamp",
            "torsion_fixed_end_anchor_pin",
            "torsion_shaft_drive_key",
        })
        for part in concept.build_module(0.0).children:
            self.assertGreater(part.volume, 0.1)

    def test_rotating_parts_follow_shaft_angle(self):
        p0 = concept.rotating_capture_center(0.0)
        p1 = concept.rotating_capture_center(1.0)
        self.assertAlmostEqual(concept.shaft_angle_deg(0.0), 0.0, places=6)
        self.assertAlmostEqual(concept.shaft_angle_deg(1.0), 90.0, places=6)
        self.assertAlmostEqual(p0[2], p1[2], places=6)
        self.assertGreater(((p0[0]-p1[0])**2 + (p0[1]-p1[1])**2)**0.5, 5.0)

    def test_module_stays_below_v3_base_and_clears_shaft(self):
        for t in (0.0, 0.5, 1.0):
            parts = {p.label: p for p in concept.build_module(t).children}
            self.assertLessEqual(parts["torsion_fixed_mounting_cup"].bounding_box().max.Z, concept.v3.v2.v1.BASE_Z0)
            shaft_tool = concept.z_cylinder(4.0, 30.0, concept.SHAFT_X, concept.SHAFT_Y, -15.0)
            for label in ("torsion_fixed_mounting_cup", "torsion_helical_spring"):
                self.assertLessEqual((parts[label] & shaft_tool).volume, 0.05)

    def test_derived_v3_has_module_labels_and_required_interface_parts(self):
        required = {
            "continuous_main_base_optional_return_drilled",
            "vertical_drive_shaft_optional_return_extended",
            "optional_return_clearance_feet",
            "torsion_fixed_mounting_cup",
            "torsion_helical_spring",
            "torsion_rotating_shaft_clamp",
        }
        for t in (0.0, 1.0):
            labels = {p.label for p in concept.build_v3_with_module(t).children}
            self.assertTrue(required.issubset(labels))
            self.assertNotIn("vertical_drive_shaft", labels)

    def test_fixed_module_does_not_penetrate_v3_mechanism(self):
        for t in (0.0, 0.5, 1.0):
            parts = {p.label: p for p in concept.build_v3_with_module(t).children}
            cup = parts["torsion_fixed_mounting_cup"]
            for label in ("module_1_26t_pinion", "lower_radial_bearing_seat", "upper_radial_bearing_seat", "lower_axial_thrust_interface", "shaft_rotating_stop_lug", "eccentric_horizontal_swing_arm"):
                self.assertLessEqual((cup & parts[label]).volume, 0.05)

    def test_split_cup_has_removable_bottom_cover_and_real_access(self):
        parts = {p.label: p for p in concept.build_module(0.0).children}
        self.assertIn("torsion_removable_bottom_cover", parts)
        self.assertGreater(parts["torsion_removable_bottom_cover"].volume, 1.0)
        access = concept.spring_installation_access_tool()
        self.assertLessEqual((parts["torsion_fixed_mounting_cup"] & access).volume, 0.05)
        self.assertGreater(concept.CUP_INNER_DIAMETER, concept.spring_outer_diameter())
        self.assertGreater(concept.CUP_INNER_DIAMETER, concept.CLAMP_OD)

    def test_three_preload_holes_cut_real_cup_wall_and_anchor_is_seated(self):
        parts = {p.label: p for p in concept.build_module(0.0).children}
        cup = parts["torsion_fixed_mounting_cup"]
        pin = parts["torsion_fixed_end_anchor_pin"]
        self.assertEqual(len(concept.PRELOAD_ANGLES_DEG), 3)
        for angle in concept.PRELOAD_ANGLES_DEG:
            tool = concept.preload_hole_tool(angle)
            self.assertGreater((concept.fixed_cup_uncut() & tool).volume, 0.5)
            self.assertLessEqual((cup & tool).volume, 0.05)
        self.assertLessEqual(pin.distance_to(cup), 0.05)
        self.assertGreater((pin & concept.preload_hole_tool(concept.SELECTED_PRELOAD_ANGLE_DEG)).volume, 0.5)

    def test_spring_clears_bottom_cover(self):
        for t in (0.0, 0.5, 1.0):
            parts = {p.label: p for p in concept.build_module(t).children}
            self.assertLessEqual(
                (parts["torsion_helical_spring"] & parts["torsion_removable_bottom_cover"]).volume,
                0.05,
            )
            self.assertGreaterEqual(
                parts["torsion_helical_spring"].bounding_box().min.Z
                - parts["torsion_removable_bottom_cover"].bounding_box().max.Z,
                0.25,
            )

    def test_rotating_capture_slot_really_contains_moving_spring_leg(self):
        for t in (0.0, 0.5, 1.0):
            parts = {p.label: p for p in concept.build_module(t).children}
            slot = concept.rotating_capture_slot_tool(t)
            self.assertGreater((concept.rotating_clamp_uncut(t) & slot).volume, 0.2)
            self.assertLessEqual((parts["torsion_rotating_shaft_clamp"] & slot).volume, 0.05)
            self.assertGreater((parts["torsion_helical_spring"] & slot).volume, 0.01)

    def test_shaft_and_clamp_have_a_real_drive_key(self):
        module_parts = {p.label: p for p in concept.build_module(0.5).children}
        self.assertIn("torsion_shaft_drive_key", module_parts)
        installed = {p.label: p for p in concept.build_v3_with_module(0.5).children}
        key = installed["torsion_shaft_drive_key"]
        self.assertGreater((key & concept.extended_shaft_keyway_tool()).volume, 0.2)
        self.assertGreater((key & concept.clamp_keyway_tool(0.5)).volume, 0.2)
        self.assertLessEqual((key & installed["vertical_drive_shaft_optional_return_extended"]).volume, 0.05)
        self.assertLessEqual((key & installed["torsion_rotating_shaft_clamp"]).volume, 0.05)
        self.assertLessEqual(key.distance_to(installed["vertical_drive_shaft_optional_return_extended"]), 0.01)
        self.assertLessEqual(key.distance_to(installed["torsion_rotating_shaft_clamp"]), 0.01)

    def test_latest_v3_combined_exports_keep_all_four_tray_bushings(self):
        required = {f"keyboard_tray_mount_bushing_{p}" for p in concept.v3.POSITIONS}
        for t in (0.0, 1.0):
            labels = {p.label for p in concept.build_v3_with_module(t).children}
            self.assertTrue(required.issubset(labels))

    def test_replacement_feet_leave_two_mm_table_clearance(self):
        parts = {p.label: p for p in concept.build_v3_with_module(0.0).children}
        table_z = parts["optional_return_clearance_feet"].bounding_box().min.Z
        module_min_z = min(
            p.bounding_box().min.Z
            for label, p in parts.items()
            if label.startswith("torsion_")
        )
        self.assertGreaterEqual(module_min_z - table_z, 2.0 - 1e-6)


if __name__ == "__main__":
    unittest.main()
