import unittest
import json
from pathlib import Path
from zipfile import is_zipfile

from parameters import BOM
from print_layouts import petg_small_parts_plate, tpu_parts_plate
from validation import (
    validate_device_clearance,
    validate_plate_layouts,
    validate_strength,
)


class FullValidationTests(unittest.TestCase):
    def test_full_validation(self):
        report = json.loads(
            (Path(__file__).parent / "validation_report.json").read_text(encoding="utf-8")
        )
        self.assertEqual([], report["errors"], report)
        self.assertGreaterEqual(report["motion"]["sample_count"], 401)
        self.assertGreaterEqual(report["collision"]["sample_count"], 101)
        self.assertEqual([], report["collision"]["collisions"])
        self.assertTrue(
            report["collision"]["passes_1mm_worst_case_noncontact_clearance"],
            report["collision"],
        )
        self.assertGreaterEqual(report["stability"]["minimum_margin_mm"], 15.0)
        self.assertGreaterEqual(report["clearance"]["bearing_to_slot_end_mm"], 2.0)
        self.assertTrue(report["spring"]["returns_with_10_percent_mismatch"])
        self.assertLessEqual(report["spring"]["maximum_user_force_n"], 8.0)
        self.assertTrue(report["parallelism"]["parallel_motion_possible"])

    def test_bom_is_exact(self):
        self.assertEqual(4, BOM["MR84ZZ"]["quantity"])
        self.assertEqual(8, BOM["604ZZ"]["quantity"])
        self.assertEqual(2, BOM["constant_force_spring"]["quantity"])
        for key, item in BOM.items():
            self.assertNotIn("range", item, key)
            self.assertIsNotNone(item.get("spec"), key)

    def test_static_strength_screen(self):
        report = validate_strength(hand_load_n=20.0)
        self.assertGreaterEqual(report["tray_bending_safety_factor"], 2.0)
        self.assertLessEqual(report["tray_center_deflection_mm"], 1.0)
        self.assertGreaterEqual(report["side_carriage_safety_factor"], 2.0)

    def test_h2d_plate_layouts_have_spacing(self):
        report = validate_plate_layouts(clearance_mm=2.0)
        self.assertEqual([], report["failures"], report)

    def test_real_device_envelopes_clear_opposing_ribs(self):
        report = validate_device_clearance(401)
        self.assertTrue(report["passes_1mm_nominal_clearance"], report)
        self.assertTrue(report["passes_1mm_worst_case_clearance"], report)

    def test_small_axial_parts_are_not_printed_on_edge(self):
        children = list(petg_small_parts_plate().children) + list(tpu_parts_plate().children)
        for child in children:
            bbox = child.bounding_box()
            self.assertAlmostEqual(0.0, bbox.min.Z, places=6, msg=child.label)
            if child.label.startswith(("cross_eccentric_bushing", "guide_eccentric_bushing")):
                self.assertLessEqual(bbox.size.Z, 6.3, child.label)

    def test_required_deliverables_exist(self):
        root = Path(__file__).parent
        required = [
            root / "keyboard_mode.step",
            root / "mid_mode.step",
            root / "trackpad_mode.step",
            root / "master_assembly.step",
            root / "device_clearance_reference.step",
            root / "printables" / "base.step",
            root / "printables" / "keyboard_tray.stl",
            root / "printables" / "trackpad_tray.stl",
            root / "printables" / "base_plate.3mf",
            root / "printables" / "trays_plate.3mf",
            root / "printables" / "frames_plate.3mf",
            root / "printables" / "carriages_plate.3mf",
            root / "printables" / "petg_small_parts_plate.3mf",
            root / "printables" / "tpu_parts_plate.3mf",
            root / "validation_report.json",
            root / "BOM.md",
            root / "ASSEMBLY.md",
        ]
        self.assertEqual([], [str(path) for path in required if not path.exists()])
        for path in required:
            if path.suffix == ".3mf":
                self.assertTrue(is_zipfile(path), path)


if __name__ == "__main__":
    unittest.main()
