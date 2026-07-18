import math
import sys
import unittest
from pathlib import Path


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

try:
    import integrated_return_v1 as concept
except ImportError:
    concept = None


REQUIRED_LABELS = {
    "rack_retention_fixed_bracket",
    "rack_back_pressure_datum_roller",
    "rack_back_pressure_preload_roller",
    "rack_back_pressure_flexure",
    "torsion_fixed_mounting_cup",
    "torsion_removable_bottom_cover",
    "torsion_helical_spring",
    "torsion_rotating_shaft_clamp",
    "torsion_fixed_end_anchor_pin",
    "torsion_shaft_drive_key",
    "vertical_drive_shaft_integrated_return",
    "integrated_return_clearance_feet",
}


class IntegratedReturnTests(unittest.TestCase):
    def test_module_exists(self):
        self.assertIsNotNone(concept)

    def test_new_entities_are_selectable_in_three_review_poses(self):
        for t in (0.0, 0.5, 1.0):
            parts = {child.label: child for child in concept.build_pose(t).children}
            self.assertTrue(REQUIRED_LABELS <= set(parts))
            for label in REQUIRED_LABELS:
                self.assertGreater(parts[label].volume, 0.05, label)

    def test_roller_axis_is_perpendicular_to_diagonal_rack_travel(self):
        travel = concept.rack_travel_unit()
        axis = concept.roller_axis_unit()
        dot = sum(a * b for a, b in zip(travel, axis))
        self.assertAlmostEqual(dot, 0.0, places=7)
        self.assertAlmostEqual(math.sqrt(sum(v * v for v in axis)), 1.0, places=7)

    def test_both_rollers_are_tangent_and_cover_the_moving_rack_back(self):
        for name in ("datum", "preload"):
            roller = concept.build_retention_parts()[
                f"rack_back_pressure_{name}_roller"
            ]
            self.assertAlmostEqual(
                roller.bounding_box().min.X,
                concept.v3.v2.v1.RACK_BODY_MAX_X,
                places=5,
            )
            for t in (0.0, 0.5, 1.0):
                point = concept.roller_rack_contact_point(name, t)
                self.assertTrue(concept.contact_point_is_on_roller(name, point))
                self.assertTrue(concept.contact_point_is_on_rack_body(t, point))

    def test_retention_hardware_does_not_interfere_with_mesh_or_shell(self):
        retention_labels = {
            "rack_retention_fixed_bracket",
            "rack_back_pressure_datum_roller",
            "rack_back_pressure_preload_roller",
            "rack_back_pressure_flexure",
        }
        protected_labels = {
            "keyboard_right_straight_rack",
            "module_1_26t_pinion",
            "fixed_protective_cylinder",
            "rack_to_keyboard_rigid_transition",
            "keyboard_tray_continuous",
        }
        for t in (0.0, 0.5, 1.0):
            parts = {child.label: child for child in concept.build_pose(t).children}
            for retention_label in retention_labels:
                for protected_label in protected_labels:
                    overlap = (
                        parts[retention_label] & parts[protected_label]
                    ).volume
                    self.assertLessEqual(
                        overlap,
                        concept.MAX_BOOLEAN_NOISE_VOLUME,
                        f"{retention_label} vs {protected_label} at t={t}",
                    )

    def test_integrated_return_keeps_physical_cartridge_interfaces(self):
        for t in (0.0, 0.5, 1.0):
            parts = {child.label: child for child in concept.build_pose(t).children}
            self.assertLessEqual(
                parts["torsion_helical_spring"].distance_to(
                    parts["torsion_removable_bottom_cover"]
                ),
                20.0,
            )
            self.assertLessEqual(
                parts["torsion_shaft_drive_key"].distance_to(
                    parts["vertical_drive_shaft_integrated_return"]
                ),
                0.01,
            )
            self.assertLessEqual(
                parts["torsion_shaft_drive_key"].distance_to(
                    parts["torsion_rotating_shaft_clamp"]
                ),
                0.01,
            )
        keyboard = {p.label: p for p in concept.build_pose(0.0).children}
        table_z = keyboard["integrated_return_clearance_feet"].bounding_box().min.Z
        self.assertGreaterEqual(
            keyboard["torsion_removable_bottom_cover"].bounding_box().min.Z
            - table_z,
            1.99,
        )

    def test_return_direction_biases_trackpad_pose_toward_keyboard_pose(self):
        self.assertEqual(concept.RETURN_TARGET_T, 0.0)
        self.assertGreater(
            concept.spring_wind_turns(1.0), concept.spring_wind_turns(0.0)
        )


if __name__ == "__main__":
    unittest.main()
