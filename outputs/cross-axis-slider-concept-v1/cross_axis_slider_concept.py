"""Parametric phase-1 concept for a keyboard / trackpad cross-axis slider stand.

This is an independent concept.  It intentionally contains no V7 geometry.
Coordinate system: X left/right, +Y toward display, +Z up.  Units are mm.
"""

from math import atan2, degrees, hypot

from build123d import Align, Box, Color, Compound, Cylinder, Pos, Rot


# Primary envelope parameters
BASE_WIDTH = 310.0
BASE_DEPTH = 300.0
BASE_THICKNESS = 4.0
BASE_CENTER_Y = 35.0

TRAY_WIDTH = 279.7
TRAY_DEPTH = 115.7
TRAY_THICKNESS = 3.5

# Confirmed endpoint travels
KEYBOARD_DY = 30.0
KEYBOARD_DZ = -15.0
TRACKPAD_DY = -115.0
TRACKPAD_DZ = -18.0

# Keyboard-mode tray origins (bottom faces)
KEYBOARD_Y0 = -35.0
KEYBOARD_Z0 = 22.0
TRACKPAD_Y0 = 45.0
TRACKPAD_Z0 = 68.0

# Side mechanism X layers, all inside the 310 mm width envelope
TRAY_EAR_X = 142.8
INPUT_ARM_X = 142.0
OUTPUT_LINK_X = 146.5
GUIDE_RAIL_X = 151.5

# Guide and coupling sizes
GUIDE_BODY_WIDTH = 10.0
GUIDE_SLOT_WIDTH = 5.4
GUIDE_THICKNESS = 4.0
GUIDE_PIN_DIAMETER = 4.0
GUIDE_PIN_SPACING = 28.0

COUPLING_BODY_LENGTH = 170.0
COUPLING_BODY_WIDTH = 13.0
COUPLING_SLOT_LENGTH = 158.0
COUPLING_SLOT_WIDTH = 5.4
COUPLING_THICKNESS = 4.0
COUPLING_PIN_DIAMETER = 4.0


BASE_COLOR = Color(0.29, 0.31, 0.34)
GUIDE_COLOR = Color(0.18, 0.20, 0.23)
KEYBOARD_COLOR = Color(0.88, 0.16, 0.13)
TRACKPAD_COLOR = Color(0.10, 0.38, 0.92)
PIN_COLOR = Color(0.96, 0.65, 0.12)
ANCHOR_COLOR = Color(0.32, 0.34, 0.37)


def _x_cylinder(radius, length, x, y, z):
    """Cylinder whose axis is global X, centered at x/y/z."""
    shape = Cylinder(
        radius,
        length,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    return shape.moved(Rot(0, 90, 0)).moved(Pos(x, y, z))


def _capsule_yz(length, width, thickness_x, center_y, center_z, angle_deg, center_x):
    """Closed rounded beam, long axis in the YZ plane."""
    straight = max(length - width, 0.1)
    body = Box(
        thickness_x,
        straight,
        width,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    end_offset = straight / 2.0
    body = body + _x_cylinder(width / 2.0, thickness_x, 0, end_offset, 0)
    body = body + _x_cylinder(width / 2.0, thickness_x, 0, -end_offset, 0)
    return body.moved(Rot(angle_deg, 0, 0)).moved(Pos(center_x, center_y, center_z))


def _beam_between_yz(p1, p2, width, thickness_x, center_x):
    y1, z1 = p1
    y2, z2 = p2
    length = hypot(y2 - y1, z2 - z1)
    angle = degrees(atan2(z2 - z1, y2 - y1))
    return _capsule_yz(
        length,
        width,
        thickness_x,
        (y1 + y2) / 2.0,
        (z1 + z2) / 2.0,
        angle,
        center_x,
    )


def _slotted_beam(length, body_width, slot_length, slot_width, thickness_x,
                  center_y, center_z, angle_deg, center_x):
    body = _capsule_yz(
        length, body_width, thickness_x, center_y, center_z, angle_deg, center_x
    )
    cutter = _capsule_yz(
        slot_length,
        slot_width,
        thickness_x + 2.0,
        center_y,
        center_z,
        angle_deg,
        center_x,
    )
    return body - cutter


def _tray(label, y, z, color):
    tray = Box(
        TRAY_WIDTH,
        TRAY_DEPTH,
        TRAY_THICKNESS,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(0, y, z))
    tray.label = label
    tray.color = color
    return tray


def _tray_ears(prefix, y, z, color):
    ears = []
    for side_name, sign in (("left", -1), ("right", 1)):
        ear = Box(
            7.0,
            46.0,
            8.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).moved(Pos(sign * TRAY_EAR_X, y, z - 2.2))
        ear.label = f"{prefix}_side_carriage_{side_name}"
        ear.color = color
        ears.append(ear)
    return ears


def _guide_geometry(prefix, c0, travel, pin_center, pose_t, color):
    """Fixed slotted guide plus two carriage rollers on both sides."""
    dy, dz = travel
    travel_len = hypot(dy, dz)
    uy, uz = dy / travel_len, dz / travel_len
    rail_center_y = pin_center[0] + dy / 2.0
    rail_center_z = pin_center[1] + dz / 2.0
    rail_angle = degrees(atan2(dz, dy))
    slot_length = travel_len + GUIDE_PIN_SPACING + GUIDE_PIN_DIAMETER + 3.0
    body_length = slot_length + 8.0

    children = []
    for side_name, sign in (("left", -1), ("right", 1)):
        x = sign * GUIDE_RAIL_X
        rail = _slotted_beam(
            body_length,
            GUIDE_BODY_WIDTH,
            slot_length,
            GUIDE_SLOT_WIDTH,
            GUIDE_THICKNESS,
            rail_center_y,
            rail_center_z,
            rail_angle,
            x,
        )
        rail.label = f"{prefix}_fixed_guide_{side_name}"
        rail.color = GUIDE_COLOR
        children.append(rail)

        # Two rollers in one straight guide prevent tray pitch while allowing translation.
        cy = pin_center[0] + pose_t * dy
        cz = pin_center[1] + pose_t * dz
        rollers = []
        for roller_index, offset in enumerate((-GUIDE_PIN_SPACING / 2.0, GUIDE_PIN_SPACING / 2.0), 1):
            py = cy + offset * uy
            pz = cz + offset * uz
            roller = _x_cylinder(GUIDE_PIN_DIAMETER / 2.0, 7.0, sign * 148.5, py, pz)
            roller.label = f"{prefix}_guide_roller_{side_name}_{roller_index}"
            roller.color = color
            rollers.append(roller)
        children.extend(rollers)

        # Slim open supports keep the guide visibly grounded without a large side shield.
        for support_index, support_y in enumerate((rail_center_y - body_length * 0.30,
                                                   rail_center_y + body_length * 0.30), 1):
            axis_z = rail_center_z + (support_y - rail_center_y) * (dz / dy if abs(dy) > 1e-9 else 0.0)
            height = max(axis_z - BASE_THICKNESS, 4.0)
            support = Box(
                6.0,
                7.0,
                height,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            ).moved(Pos(x, support_y, BASE_THICKNESS))
            support.label = f"{prefix}_guide_support_{side_name}_{support_index}"
            support.color = GUIDE_COLOR
            children.append(support)
    return children


def _cross_coupler(key_y, key_z, track_y, track_z):
    """Red fixed-pin input arms crossing real blue slotted trackpad links."""
    children = []

    # Pin is rigidly attached to the keyboard carriage.
    pin_y = key_y + 30.0
    pin_z = key_z + 21.0
    input_anchor = (key_y - 45.0, key_z + 2.0)

    # Slot center is derived so the pin travels symmetrically by the relative
    # displacement (+145,+3) across the full stroke.
    rel_dy = KEYBOARD_DY - TRACKPAD_DY
    rel_dz = KEYBOARD_DZ - TRACKPAD_DZ
    # In keyboard mode the pin starts at the negative half of slot travel.
    # From there the complete blue member follows the trackpad carriage as a
    # single rigid part; only the red pin changes position inside its slot.
    slot_center_y0 = (KEYBOARD_Y0 + 30.0) + rel_dy / 2.0
    slot_center_z0 = (KEYBOARD_Z0 + 21.0) + rel_dz / 2.0
    slot_center_y = slot_center_y0 + (track_y - TRACKPAD_Y0)
    slot_center_z = slot_center_z0 + (track_z - TRACKPAD_Z0)
    slot_angle = degrees(atan2(rel_dz, rel_dy))

    for side_name, sign in (("left", -1), ("right", 1)):
        input_arm = _beam_between_yz(
            input_anchor,
            (pin_y, pin_z),
            11.0,
            4.0,
            sign * INPUT_ARM_X,
        )
        input_arm.label = f"keyboard_drive_arm_{side_name}"
        input_arm.color = KEYBOARD_COLOR
        children.append(input_arm)

        slotted_link = _slotted_beam(
            COUPLING_BODY_LENGTH,
            COUPLING_BODY_WIDTH,
            COUPLING_SLOT_LENGTH,
            COUPLING_SLOT_WIDTH,
            COUPLING_THICKNESS,
            slot_center_y,
            slot_center_z,
            slot_angle,
            sign * OUTPUT_LINK_X,
        )

        # A short integral neck connects the blue slotted member to the trackpad carriage.
        link_attach_y = slot_center_y + 24.0
        link_attach_z = slot_center_z + 24.0 * (rel_dz / rel_dy)
        tray_anchor = (track_y + 30.0, track_z - 2.0)
        neck = _beam_between_yz(
            (link_attach_y, link_attach_z),
            tray_anchor,
            11.0,
            COUPLING_THICKNESS,
            sign * OUTPUT_LINK_X,
        )
        slotted_link = slotted_link + neck
        slotted_link.label = f"trackpad_slotted_link_{side_name}"
        slotted_link.color = TRACKPAD_COLOR
        children.append(slotted_link)

        coupling_pin = _x_cylinder(
            COUPLING_PIN_DIAMETER / 2.0,
            10.0,
            sign * 144.4,
            pin_y,
            pin_z,
        )
        coupling_pin.label = f"cross_axis_roller_pin_{side_name}"
        coupling_pin.color = PIN_COLOR
        children.append(coupling_pin)
    return children


def _return_anchor_tabs():
    children = []
    for side_name, sign in (("left", -1), ("right", 1)):
        x = sign * 146.5
        tab = Box(
            5.0,
            24.0,
            14.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Pos(x, 137.0, BASE_THICKNESS))
        for hole_y in (132.0, 137.0, 142.0):
            tab = tab - _x_cylinder(1.8, 7.0, x, hole_y, BASE_THICKNESS + 8.0)
        tab.label = f"return_anchor_3x5mm_{side_name}"
        tab.color = ANCHOR_COLOR
        children.append(tab)
    return children


def build_pose(travel=0.0, pose_name="keyboard_mode"):
    """Build one static assembly pose for 0 <= travel <= 1."""
    t = max(0.0, min(1.0, float(travel)))
    key_y = KEYBOARD_Y0 + t * KEYBOARD_DY
    key_z = KEYBOARD_Z0 + t * KEYBOARD_DZ
    track_y = TRACKPAD_Y0 + t * TRACKPAD_DY
    track_z = TRACKPAD_Z0 + t * TRACKPAD_DZ

    base = Box(
        BASE_WIDTH,
        BASE_DEPTH,
        BASE_THICKNESS,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Pos(0, BASE_CENTER_Y, 0))
    base.label = "base_310x300"
    base.color = BASE_COLOR

    children = [
        base,
        _tray("keyboard_tray_continuous", key_y, key_z, KEYBOARD_COLOR),
        _tray("trackpad_tray_continuous_right_aligned_reference", track_y, track_z, TRACKPAD_COLOR),
    ]
    children.extend(_tray_ears("keyboard", key_y, key_z, KEYBOARD_COLOR))
    children.extend(_tray_ears("trackpad", track_y, track_z, TRACKPAD_COLOR))

    children.extend(_guide_geometry(
        "keyboard",
        (KEYBOARD_Y0, KEYBOARD_Z0),
        (KEYBOARD_DY, KEYBOARD_DZ),
        (KEYBOARD_Y0, KEYBOARD_Z0 + 8.0),
        t,
        KEYBOARD_COLOR,
    ))
    children.extend(_guide_geometry(
        "trackpad",
        (TRACKPAD_Y0, TRACKPAD_Z0),
        (TRACKPAD_DY, TRACKPAD_DZ),
        (TRACKPAD_Y0, TRACKPAD_Z0 - 8.0),
        t,
        TRACKPAD_COLOR,
    ))
    children.extend(_cross_coupler(key_y, key_z, track_y, track_z))
    children.extend(_return_anchor_tabs())

    assembly = Compound(label=f"cross_axis_slider_concept_v1_{pose_name}", children=children)
    return assembly
