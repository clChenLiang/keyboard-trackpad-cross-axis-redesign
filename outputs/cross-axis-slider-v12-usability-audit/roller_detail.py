"""Selectable assembled/exploded details for the three V12 roller interfaces."""

from build123d import Align, Box, Color, Compound, Pos

import assembly
import parameters as p
import parts


PLASTIC = Color(0.20, 0.23, 0.27)
SLOT = Color(0.10, 0.38, 0.92)
BEARING = assembly.BEARING_COLOR
HARDWARE = assembly.HARDWARE_COLOR
BUSHING = Color(0.88, 0.16, 0.13)


def _add(children, name, shape, color, dx=0.0):
    shape = shape.moved(Pos(dx, 0.0, 0.0))
    shape.label = name
    shape.color = color
    children.append(shape)


def _carriage_coupon(y: float, eccentric: bool):
    coupon = Box(5.0, 30.0, 24.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
        Pos(0.0, y, 0.0)
    )
    socket_y = y - p.ECCENTRICITY if eccentric else y
    diameter = 10.2 if eccentric else p.M4_CLEARANCE_DIAMETER
    coupon = coupon - assembly._x_cylinder(diameter / 2.0, 7.0, 0.0, socket_y, 0.0)
    if eccentric:
        coupon = coupon - assembly._x_cylinder(
            (p.ECCENTRIC_FLANGE_OD + 0.2) / 2.0,
            p.ECCENTRIC_FLANGE_THICKNESS + 0.2,
            -(2.5 - p.ECCENTRIC_FLANGE_THICKNESS / 2.0),
            socket_y,
            0.0,
        )
    return coupon


def _slot_coupon(y: float, slot_width: float):
    body = Box(5.2, 48.0, 22.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
        Pos(16.0, y, 0.0)
    )
    corridor = Box(7.2, 36.0, slot_width, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
        Pos(16.0, y, 0.0)
    )
    return body - corridor


def _stack(children, name: str, y: float, spec, slot_width: float, eccentric: bool, exploded: bool):
    carriage_x = 0.0
    bearing_x = 16.0
    bearing = assembly._ring_x(spec.od, spec.bore, spec.width, bearing_x, y, 0.0)
    shaft = assembly._shoulder_fastener(carriage_x, bearing_x, 5.0, spec.width, y, 0.0)
    washer, nut = assembly._roller_retention(carriage_x, bearing_x, spec.width, y, 0.0)

    offsets = {
        "shaft": -24.0,
        "carriage": -10.0,
        "bushing": -3.0,
        "slot": 6.0,
        "bearing": 16.0,
        "retaining_washer": 25.0,
        "retaining_nut": 33.0,
    } if exploded else {key: 0.0 for key in (
        "shaft", "carriage", "bushing", "slot", "bearing", "retaining_washer", "retaining_nut"
    )}

    _add(children, f"{name}_carriage_coupon", _carriage_coupon(y, eccentric), PLASTIC, offsets["carriage"])
    _add(children, f"{name}_slot_coupon", _slot_coupon(y, slot_width), SLOT, offsets["slot"])
    _add(children, f"{name}_shaft", shaft, HARDWARE, offsets["shaft"])
    _add(children, f"{name}_bearing", bearing, BEARING, offsets["bearing"])
    _add(children, f"{name}_retaining_washer", washer, HARDWARE, offsets["retaining_washer"])
    _add(children, f"{name}_retaining_nut", nut, HARDWARE, offsets["retaining_nut"])
    if eccentric:
        bushing = parts.make_eccentric_bushing("cross" if spec == p.MR84ZZ else "guide")
        bushing = bushing.moved(Pos(0.0, y - p.ECCENTRICITY, 0.0))
        _add(children, f"{name}_eccentric_bushing", bushing, BUSHING, offsets["bushing"])


def build_roller_detail(exploded: bool = True):
    children = []
    _stack(children, "guide_fixed", 52.0, p.BEARING_604ZZ, p.GUIDE_SLOT_WIDTH, False, exploded)
    _stack(children, "guide_eccentric", 0.0, p.BEARING_604ZZ, p.GUIDE_SLOT_WIDTH, True, exploded)
    _stack(children, "cross_eccentric", -52.0, p.MR84ZZ, p.CROSS_SLOT_WIDTH, True, exploded)
    _stack(
        children,
        "right_follower",
        -104.0,
        p.BEARING_604ZZ,
        p.GUIDE_SLOT_WIDTH + p.RIGHT_FOLLOWER_SLOT_EXTRA_WIDTH,
        True,
        exploded,
    )
    return Compound(
        label="v12_roller_detail_exploded" if exploded else "v12_roller_detail_assembled",
        children=children,
    )


def gen_step():
    return build_roller_detail(exploded=True)
