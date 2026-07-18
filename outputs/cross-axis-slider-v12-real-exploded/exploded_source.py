"""Truthful exploded view derived directly from the validated V12 assembly.

Only rigid display translations are applied.  Every visible body comes from
``assembly.build_pose(0.0)``; no replacement or illustrative geometry is
introduced here.
"""

from build123d import Compound, Pos

from assembly import build_pose


def _side(name: str) -> float:
    if "_left" in name:
        return -1.0
    if "_right" in name:
        return 1.0
    return 0.0


def _display_offset(name: str):
    side = _side(name)

    if name == "base":
        return (0.0, 0.0, 0.0)
    if name.startswith("fixed_tpu_base_foot_"):
        return (0.0, 0.0, -16.0)

    # Fixed frames stay close to the base, but separate laterally so their
    # guide slots and four base fasteners remain readable.
    if name.startswith("fixed_side_frame_"):
        return (side * 28.0, 0.0, 24.0)
    if name.startswith("fixed_base_insert_"):
        return (side * 28.0, 0.0, 10.0)
    if name.startswith("fixed_base_mount_screw_"):
        return (side * 28.0, 0.0, -10.0)

    # The two complete trays are deliberately kept intact and separate.
    if name == "keyboard_tray":
        return (0.0, 0.0, 132.0)
    if name.startswith("keyboard_tpu_pad_"):
        return (0.0, 0.0, 142.0)
    if name == "trackpad_tray":
        return (0.0, 0.0, 214.0)
    if name.startswith("trackpad_tpu_pad_"):
        return (0.0, 0.0, 224.0)

    # Moving side modules sit between the frames and trays.  Their long-slot
    # and cross-yoke topology is unchanged.
    if name.startswith("keyboard_carriage_"):
        return (side * 40.0, 0.0, 92.0)
    if name.startswith("trackpad_carriage_"):
        return (side * 58.0, 0.0, 158.0)

    # Tray attachment hardware follows its tray, with a small axial explode.
    if name.startswith("keyboard_tray_insert_"):
        return (side * 8.0, 0.0, 132.0)
    if name.startswith("keyboard_tray_mount_screw_"):
        return (side * 18.0, 0.0, 132.0)
    if name.startswith("trackpad_tray_insert_"):
        return (side * 10.0, 0.0, 214.0)
    if name.startswith("trackpad_tray_mount_screw_"):
        return (side * 22.0, 0.0, 214.0)

    # Guide hardware remains aligned with its true X-axis stack.
    if name.startswith("keyboard_guide_"):
        axial = 54.0
        if "bearing" in name:
            axial = 46.0
        elif "washer" in name:
            axial = 62.0
        elif "nut" in name:
            axial = 70.0
        elif "eccentric_bushing" in name:
            axial = 38.0
        return (side * axial, 0.0, 92.0)
    if name.startswith("trackpad_guide_"):
        axial = 72.0
        if "bearing" in name:
            axial = 64.0
        elif "washer" in name:
            axial = 80.0
        elif "nut" in name:
            axial = 88.0
        elif "eccentric_bushing" in name:
            axial = 56.0
        return (side * axial, 0.0, 158.0)

    # Crosspoint stacks are shown between the red yoke and blue long slot.
    if name.startswith("cross_"):
        axial = 52.0
        if "bearing" in name:
            axial = 48.0
        elif "washer" in name:
            axial = 58.0
        elif "nut" in name:
            axial = 64.0
        elif "eccentric_bushing" in name:
            axial = 43.0
        return (side * axial, 0.0, 124.0)

    # Return and hard-stop parts remain grouped per side, not converted into
    # the fictitious full-width shaft seen in the rejected illustration.
    if name.startswith("trackpad_return_"):
        return (side * 48.0, 0.0, 58.0)
    if name.startswith("fixed_return_"):
        return (side * 34.0, 0.0, 42.0)
    if name.startswith("spring_guide_rod_"):
        return (side * 42.0, 0.0, 48.0)
    if name.startswith("fixed_stop_collar_"):
        return (side * 50.0, 0.0, 48.0)

    return (0.0, 0.0, 0.0)


def gen_step():
    source = build_pose(0.0)
    children = []
    for name, body in source.bodies.items():
        dx, dy, dz = _display_offset(name)
        displayed = body.moved(Pos(dx, dy, dz))
        displayed.label = name
        displayed.color = body.color
        children.append(displayed)
    return Compound(label="cross_axis_slider_v12_truthful_exploded", children=children)

