"""Named engineering parameters for the V10 printable mechanism.

Coordinates are millimetres. X is left/right, +Y points toward the display,
and +Z points upward.  Values in this module are independent inputs; derived
motion values live in :mod:`mechanism`.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class DeviceEnvelope:
    width: float
    depth: float
    height: float
    mass_kg: float


@dataclass(frozen=True)
class BearingSpec:
    bore: float
    od: float
    width: float


KEYBOARD = DeviceEnvelope(278.9, 114.9, 10.9, 0.239)
TRACKPAD = DeviceEnvelope(160.0, 114.9, 10.9, 0.230)

TRAY_WIDTH = 279.7
TRAY_DEPTH = 115.7
TRAY_SKIN = 3.5
TRAY_RIB_DEPTH = 6.0
TRAY_RIB_THICKNESS = 2.4

KEYBOARD_Y0 = -35.0
KEYBOARD_Z0 = 23.0
KEYBOARD_DY = 30.0
KEYBOARD_DZ = -15.0
TRACKPAD_Y0 = 45.0
TRACKPAD_Z0 = 68.0
TRACKPAD_DY = -115.0
TRACKPAD_DZ = -43.0

BASE_WIDTH = 298.0
BASE_DEPTH = 250.0
BASE_CENTER_Y = 15.0
BASE_THICKNESS = 6.0
BASE_RAIL_WIDTH = 8.0
BASE_FOOT_INSET_X = 12.0
BASE_FOOT_INSET_Y = 9.0
STABILITY_MARGIN = 15.0
H2D_SINGLE_NOZZLE_VOLUME = (325.0, 320.0, 325.0)
H2D_DUAL_NOZZLE_X = 300.0

MR84ZZ = BearingSpec(4.0, 8.0, 3.0)
BEARING_604ZZ = BearingSpec(4.0, 12.0, 4.0)
PRINT_HOLE_COMPENSATION = 0.15
PRINT_SLOT_COMPENSATION = 0.10
CROSS_SLOT_WIDTH = MR84ZZ.od + 0.50 + PRINT_SLOT_COMPENSATION
CROSS_SLOT_LENGTH = 184.0
CROSS_SLOT_PLATE_WIDTH = 18.0
CROSS_SLOT_PLATE_THICKNESS = 4.8
CROSS_ROLLER_SPACING = 18.0
GUIDE_SLOT_WIDTH = BEARING_604ZZ.od + 0.50 + PRINT_SLOT_COMPENSATION
GUIDE_PLATE_WIDTH = 22.0
GUIDE_PLATE_THICKNESS = 5.2
KEYBOARD_GUIDE_ROLLER_SPACING = 28.0
TRACKPAD_GUIDE_ROLLER_SPACING = 36.0
ECCENTRICITY = 0.40
ECCENTRIC_FLANGE_OD = 12.0
ECCENTRIC_FLANGE_THICKNESS = 1.2
ECCENTRIC_FLANGE_AF = 10.0
RIGHT_SIDE_AXIAL_FLOAT = 0.60

SHAFT_DIAMETER = 4.0
M4_CLEARANCE_DIAMETER = SHAFT_DIAMETER + 0.35 + PRINT_HOLE_COMPENSATION
M4_HEAT_INSERT_OD = 6.2
M4_HEAT_INSERT_BORE = 4.2
M4_HEAT_INSERT_LENGTH = 5.2
MIN_LOAD_WALL = 1.2

SPRING_COUNT = 2
SPRING_FREE_LENGTH = 82.0
SPRING_INSTALLED_LENGTH = 49.0
SPRING_SOLID_LENGTH = 8.0
SPRING_OD = 8.0
SPRING_WIRE = 0.55
SPRING_RATE_PER_SPRING = 0.19  # N/mm
SPRING_PRELOAD_PER_SPRING = (
    SPRING_FREE_LENGTH - SPRING_INSTALLED_LENGTH
) * SPRING_RATE_PER_SPRING
SPRING_GUIDE_DIAMETER = 3.0
SPRING_COIL_BIND_SAFETY = 2.0

PETG_DENSITY_KG_PER_MM3 = 1.27e-6
TPU_DENSITY_KG_PER_MM3 = 1.21e-6
GRAVITY = 9.80665

# Exact baseline; printed-part quantities are populated by the exporter.
BOM = {
    "MR84ZZ": {"quantity": 4, "spec": "4 x 8 x 3 mm shielded bearing"},
    "604ZZ": {"quantity": 8, "spec": "4 x 12 x 4 mm shielded bearing"},
    "compression_spring": {
        "quantity": 2,
        "spec": "OD 8 x free 82 mm, wire 0.55 mm, about 11 active coils, rate 0.19 N/mm",
    },
    "spring_guide_rod": {
        "quantity": 2,
        "spec": "3.0 mm stainless precision rod x 91.0 mm, cut ends deburred",
    },
    "M4_shoulder_fastener_keyboard_guide": {
        "quantity": 4,
        "spec": "ISO 7379 configurable: 4 mm smooth shoulder x 20.5 mm, M3 x 3.4 mm threaded tip",
    },
    "M4_shoulder_fastener_trackpad_guide": {
        "quantity": 4,
        "spec": "ISO 7379 configurable: 4 mm smooth shoulder x 14.5 mm, M3 x 3.4 mm threaded tip",
    },
    "M4_shoulder_fastener_cross": {
        "quantity": 4,
        "spec": "ISO 7379 configurable: 4 mm smooth shoulder x 10.5 mm, M3 x 3.4 mm threaded tip",
    },
    "M3_retaining_nut": {
        "quantity": 12,
        "spec": "DIN 934 M3 hex nut, 2.4 mm thick; secure with removable medium-strength threadlocker",
    },
    "M3_inner_race_washer": {
        "quantity": 12,
        "spec": "3.2 mm ID x 7 mm OD x 0.5 mm flat washer bearing on the inner ring",
    },
    "M4x8_socket_screw": {
        "quantity": 4,
        "spec": "M4 x 8 mm socket-head for recessed base mounts",
    },
    "M4x10_socket_screw": {
        "quantity": 4,
        "spec": "M4 x 10 mm socket-head for keyboard tray mounts",
    },
    "M4x16_socket_screw": {
        "quantity": 4,
        "spec": "M4 x 16 mm socket-head for trackpad tray mounts",
    },
    "M4_heat_set_insert": {
        "quantity": 12,
        "spec": "M4, OD 6.2 x 5.2 mm heat-set insert",
    },
}
