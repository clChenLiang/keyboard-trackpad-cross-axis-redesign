import sys
import unittest
from pathlib import Path

from build123d import Align, Box, Pos


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

try:
    import open_frame_base_v1 as concept
except ImportError:
    concept = None


class OpenFrameBaseTests(unittest.TestCase):
    def test_module_exists(self):
        self.assertIsNotNone(concept)

    def test_base_has_compact_envelope_and_unchanged_top_datum(self):
        base = concept.make_open_frame_base()
        box = base.bounding_box()
        self.assertAlmostEqual(box.size.X, 310.0, places=5)
        self.assertAlmostEqual(box.size.Y, 225.0, places=5)
        self.assertAlmostEqual(box.min.Y, -55.0, places=5)
        self.assertAlmostEqual(box.max.Y, 170.0, places=5)
        self.assertAlmostEqual(box.min.Z, 1.0, places=5)
        self.assertAlmostEqual(box.max.Z, 7.0, places=5)

    def test_front_crossmember_does_not_leave_an_unprintable_narrow_slot(self):
        front_rail_inner_y = -55.0 + concept.RAIL_WIDTH
        front_crossmember_min_y = (
            concept.CROSS_MEMBER_Y[0] - concept.CROSS_MEMBER_DEPTH / 2.0
        )
        self.assertLessEqual(front_crossmember_min_y, front_rail_inner_y)

    def test_mounting_probes_land_on_positive_base_material(self):
        base = concept.make_open_frame_base()
        for name, x, y in concept.support_probe_points():
            probe = Box(
                1.0,
                1.0,
                1.0,
                align=(Align.CENTER, Align.CENTER, Align.CENTER),
            ).moved(Pos(x, y, 5.5))
            self.assertGreater((base & probe).volume, 0.8, name)

    def test_new_base_and_feet_are_selectable_in_three_poses(self):
        required = {
            "open_frame_base_310x225",
            "primary_tpu_feet_4x",
            "auxiliary_tpu_pads_2x",
        }
        for t in (0.0, 0.5, 1.0):
            parts = {child.label: child for child in concept.build_pose(t).children}
            self.assertTrue(required <= set(parts))
            for label in required:
                self.assertGreater(parts[label].volume, 1.0, label)

    def test_flange_recess_and_foot_pockets_remove_solid_overlap(self):
        parts = {child.label: child for child in concept.build_pose(0.0).children}
        base = parts["open_frame_base_310x225"]
        for label in (
            "torsion_fixed_mounting_cup",
            "primary_tpu_feet_4x",
            "auxiliary_tpu_pads_2x",
        ):
            self.assertLessEqual(
                (base & parts[label]).volume,
                concept.MAX_BOOLEAN_NOISE_VOLUME,
                label,
            )
            self.assertLessEqual(base.distance_to(parts[label]), 0.01, label)

    def test_primary_and_auxiliary_feet_have_the_intended_table_planes(self):
        parts = {child.label: child for child in concept.build_pose(0.0).children}
        primary = parts["primary_tpu_feet_4x"].bounding_box()
        auxiliary = parts["auxiliary_tpu_pads_2x"].bounding_box()
        cover = parts["torsion_removable_bottom_cover"].bounding_box()
        self.assertAlmostEqual(primary.min.Z, -15.0, places=5)
        self.assertAlmostEqual(primary.max.Z, 3.0, places=5)
        self.assertAlmostEqual(auxiliary.min.Z, -14.7, places=5)
        self.assertAlmostEqual(auxiliary.max.Z, 3.0, places=5)
        self.assertGreaterEqual(cover.min.Z - primary.min.Z, 2.0 - 1e-6)

    def test_base_does_not_interfere_with_preserved_mechanism(self):
        protected = {
            "keyboard_tray_continuous",
            "magic_trackpad_small_tray",
            "keyboard_right_straight_rack",
            "module_1_26t_pinion",
            "eccentric_horizontal_swing_arm",
            "vertical_drive_shaft_integrated_return",
            "fixed_protective_cylinder",
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
        }
        for t in (0.0, 0.5, 1.0):
            parts = {child.label: child for child in concept.build_pose(t).children}
            base = parts["open_frame_base_310x225"]
            for label in protected:
                overlap = (base & parts[label]).volume
                self.assertLessEqual(
                    overlap,
                    concept.MAX_BOOLEAN_NOISE_VOLUME,
                    f"base vs {label} at t={t}",
                )


if __name__ == "__main__":
    unittest.main()
