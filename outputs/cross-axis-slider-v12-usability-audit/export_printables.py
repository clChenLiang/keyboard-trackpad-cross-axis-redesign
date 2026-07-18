"""Validate and export the complete V12 STEP/STL/3MF handoff package."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
from xml.etree import ElementTree
from zipfile import ZipFile, is_zipfile

from build123d import export_step, export_stl

import assembly
import parts
import print_layouts
import roller_detail
from validation import validate_all


ROOT = Path(__file__).resolve().parent
PRINTABLES = ROOT / "printables"
CAD_STEP_LAUNCHER = Path("/Users/bytedance/.agents/skills/cad/scripts/step")


def _export_shape(shape, step_path: Path, stl_path: Path = None):
    step_path.parent.mkdir(parents=True, exist_ok=True)
    if not export_step(shape, step_path):
        raise RuntimeError(f"STEP export failed: {step_path}")
    if stl_path is not None:
        # Mesh the persisted STEP through the same OCCT pipeline used for the
        # verified 3MF plates.  Direct build123d STL export left an open seam
        # on the right frame even though its B-rep was valid.
        completed = subprocess.run(
            [
                sys.executable,
                str(CAD_STEP_LAUNCHER),
                "--kind",
                "part",
                str(step_path),
                "--stl",
                stl_path.name,
                "--mesh-tolerance",
                "0.05",
                "--mesh-angular-tolerance",
                "0.10",
            ],
            cwd=str(stl_path.parent),
            text=True,
            capture_output=True,
        )
        if completed.returncode != 0 or not stl_path.exists():
            raise RuntimeError(
                f"STL export failed: {stl_path}\n"
                f"{completed.stdout}\n{completed.stderr}"
            )


def _export_3mf_from_step(step_path: Path, output_path: Path):
    completed = subprocess.run(
        [
            sys.executable,
            str(CAD_STEP_LAUNCHER),
            "--kind",
            "assembly",
            str(step_path),
            "--3mf",
            output_path.name,
            "--mesh-tolerance",
            "0.05",
            "--mesh-angular-tolerance",
            "0.10",
        ],
        cwd=str(ROOT),
        text=True,
        capture_output=True,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"3MF export failed for {step_path.name}:\n"
            f"{completed.stdout}\n{completed.stderr}"
        )
    if not output_path.exists() or output_path.stat().st_size == 0:
        raise RuntimeError(f"3MF exporter returned success but did not create {output_path}")


def _inspect_3mf(path: Path, root_label: str, expected_component_count: int):
    if not is_zipfile(path):
        raise RuntimeError(f"invalid 3MF ZIP container: {path}")
    namespace = {"m": "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"}
    with ZipFile(path) as archive:
        model = ElementTree.fromstring(archive.read("3D/3dmodel.model"))
    objects = model.findall(".//m:resources/m:object", namespace)
    build_items = model.findall(".//m:build/m:item", namespace)
    roots = [item for item in objects if item.attrib.get("name") == root_label]
    component_count = 0
    if roots:
        component_count = len(roots[0].findall("./m:components/m:component", namespace))
    if not roots or component_count != expected_component_count or not build_items:
        raise RuntimeError(
            f"3MF structure check failed for {path.name}: "
            f"root={root_label!r}, components={component_count}/"
            f"{expected_component_count}, build_items={len(build_items)}"
        )
    return {
        "object_count": len(objects),
        "build_item_count": len(build_items),
        "root_label": root_label,
        "component_count": component_count,
    }


def export_package(report):
    PRINTABLES.mkdir(parents=True, exist_ok=True)
    manifest = {"poses": [], "parts": [], "plates": []}

    for name, travel in (("keyboard_mode", 0.0), ("mid_mode", 0.5), ("trackpad_mode", 1.0)):
        target = ROOT / f"{name}.step"
        _export_shape(assembly.build_pose(travel).compound, target)
        manifest["poses"].append(target.name)
    master = ROOT / "master_assembly.step"
    _export_shape(assembly.build_pose(0.0).compound, master)
    manifest["poses"].append(master.name)
    device_reference = ROOT / "device_clearance_reference.step"
    _export_shape(
        assembly.build_pose(1.0, include_devices=True).compound,
        device_reference,
    )
    manifest["poses"].append(device_reference.name)

    for name, exploded in (("roller_detail_assembled", False), ("roller_detail_exploded", True)):
        target = ROOT / f"{name}.step"
        _export_shape(roller_detail.build_roller_detail(exploded), target)
        manifest["poses"].append(target.name)

    for name, shape in parts.printable_parts().items():
        step_path = PRINTABLES / f"{name}.step"
        stl_path = PRINTABLES / f"{name}.stl"
        _export_shape(shape, step_path, stl_path)
        manifest["parts"].append({
            "name": name,
            "step": step_path.name,
            "stl": stl_path.name,
        })

    plate_factories = (
        ("base_plate", print_layouts.base_plate),
        ("trays_plate", print_layouts.trays_plate),
        ("frames_plate", print_layouts.frames_plate),
        ("carriages_plate", print_layouts.carriages_plate),
        ("petg_small_parts_plate", print_layouts.petg_small_parts_plate),
        ("tpu_parts_plate", print_layouts.tpu_parts_plate),
    )
    for name, factory in plate_factories:
        plate = factory()
        step_path = PRINTABLES / f"{name}.step"
        three_mf_path = PRINTABLES / f"{name}.3mf"
        _export_shape(plate, step_path)
        _export_3mf_from_step(step_path, three_mf_path)
        structure = _inspect_3mf(
            three_mf_path,
            plate.label,
            len(plate.children),
        )
        manifest["plates"].append({
            "name": name,
            "step": step_path.name,
            "3mf": three_mf_path.name,
            "structure": structure,
        })

    manifest["generated_utc"] = datetime.now(timezone.utc).isoformat()
    (PRINTABLES / "manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    report["export_manifest"] = manifest
    return manifest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--verify-only", action="store_true")
    parser.add_argument("--export-only", action="store_true")
    parser.add_argument("--motion-samples", type=int, default=401)
    parser.add_argument("--collision-samples", type=int, default=101)
    args = parser.parse_args()

    if args.verify_only and args.export_only:
        parser.error("--verify-only and --export-only are mutually exclusive")

    report_path = ROOT / "validation_report.json"
    if args.export_only:
        if not report_path.exists():
            raise SystemExit("validation_report.json is missing; refusing export-only")
        report = json.loads(report_path.read_text(encoding="utf-8"))
        if report.get("errors") or report.get("collision", {}).get("sample_count", 0) < 101:
            raise SystemExit("cached validation is not a passing 101-point report")
        export_package(report)
        report_path.write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        print(json.dumps({"ok": True, "export_only": True}, indent=2))
        return

    report = validate_all(args.motion_samples, args.collision_samples)
    if report["errors"]:
        print(json.dumps(report, indent=2, ensure_ascii=False))
        raise SystemExit("validation failed; refusing to export")
    # Persist the authoritative validation before secondary export work so a
    # packaging failure can never erase a completed long-running audit.
    report_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    if not args.verify_only:
        export_package(report)
    report_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({
        "ok": True,
        "verify_only": args.verify_only,
        "motion_samples": args.motion_samples,
        "collision_samples": args.collision_samples,
        "collision_count": len(report["collision"]["collisions"]),
        "minimum_stability_margin_mm": report["stability"]["minimum_margin_mm"],
        "maximum_user_force_n": report["spring"]["maximum_user_force_n"],
        "return_spring_working_margin_mm": report["spring"]["working_deflection_margin_mm"],
    }, indent=2))


if __name__ == "__main__":
    main()
