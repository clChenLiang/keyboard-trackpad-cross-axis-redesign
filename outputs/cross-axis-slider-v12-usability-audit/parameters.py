"""Named engineering parameters for the V12 printable mechanism.

Coordinates are millimetres. X is left/right, +Y points toward the display,
and +Z points upward.  Values in this module are independent inputs; derived
motion values live in :mod:`mechanism`.
"""

from dataclasses import dataclass


GRAVITY = 9.80665


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
# Conservative 20 N beam screen requires 11 mm deep-plan ribs to keep the
# 279.7 mm tray below 1.0 mm center deflection without increasing hand height.
TRAY_RIB_THICKNESS = 11.0

KEYBOARD_Y0 = -35.0
KEYBOARD_Z0 = 23.0
KEYBOARD_DY = 30.0
KEYBOARD_DZ = -17.0
TRACKPAD_Y0 = 45.0
# Lower storage position and shallower output drop keep the long cross slot
# below the 85 mm envelope while preserving a physically usable work height.
TRACKPAD_Z0 = 68.0
TRACKPAD_DY = -115.0
TRACKPAD_DZ = -40.0

BASE_WIDTH = 298.0
BASE_DEPTH = 250.0
BASE_CENTER_Y = 15.0
BASE_THICKNESS = 6.0
BASE_RAIL_WIDTH = 8.0
BASE_FOOT_INSET_X = 13.0
BASE_FOOT_INSET_Y = 10.0
BASE_FOOT_POSITIONS = (
    (-136.0, -100.0),
    (-136.0, 130.0),
    (0.0, -95.0),
    (0.0, 103.0),
    (136.0, -100.0),
    (136.0, 130.0),
)
STABILITY_MARGIN = 15.0
H2D_SINGLE_NOZZLE_VOLUME = (325.0, 320.0, 325.0)
H2D_DUAL_NOZZLE_X = 300.0

MR84ZZ = BearingSpec(4.0, 8.0, 3.0)
BEARING_604ZZ = BearingSpec(4.0, 12.0, 4.0)
PRINT_HOLE_COMPENSATION = 0.15
PRINT_SLOT_COMPENSATION = 0.10
# Total worst-direction closing allowance for both printed tray systems after
# selecting the calibrated H2D profile with the supplied coupon.
CALIBRATED_PRINT_Z_CLOSING_TOLERANCE_TOTAL = 0.50
# In-plane/normal closing allowance for two calibrated printed mechanism
# parts.  Eccentric datum adjustment and the right follower float are handled
# separately; this value is reserved for residual print/profile error.
CALIBRATED_PRINT_NONCONTACT_CLOSING_TOLERANCE_TOTAL = 0.25
CROSS_SLOT_WIDTH = MR84ZZ.od + 0.50 + PRINT_SLOT_COMPENSATION
CROSS_SLOT_LENGTH = 184.0
CROSS_SLOT_PLATE_WIDTH = 11.0
CROSS_SLOT_PLATE_THICKNESS = 4.8
CROSS_ROLLER_SPACING = 18.0
GUIDE_SLOT_WIDTH = BEARING_604ZZ.od + 0.50 + PRINT_SLOT_COMPENSATION
GUIDE_PLATE_WIDTH = 15.0
GUIDE_PLATE_THICKNESS = 5.2
KEYBOARD_GUIDE_ROLLER_SPACING = 28.0
TRACKPAD_GUIDE_ROLLER_SPACING = 36.0
ECCENTRICITY = 0.40
ECCENTRIC_FLANGE_OD = 12.0
ECCENTRIC_FLANGE_THICKNESS = 1.2
ECCENTRIC_FLANGE_AF = 10.0
# Bilateral X stacks are true mirrors.  Anti-binding compliance is provided
# by the wider right Y-Z follower slots, not by a clamped TPU axial washer.
RIGHT_SIDE_AXIAL_FLOAT = 0.0
# The left slots are the kinematic datum.  The right slots are deliberately
# wider so the rigid trays do not close two nominally identical, zero-clearance
# guide loops.  The extra 1.00 mm gives +/-0.50 mm in-plane follower float;
# the right eccentric rollers are then adjusted only to remove perceptible
# rattle, not to create a second hard datum.
RIGHT_FOLLOWER_SLOT_EXTRA_WIDTH = 1.00
PARALLELISM_ASSUMED_MAX_ANGLE_DEG = 0.10

SHAFT_DIAMETER = 4.0
M4_CLEARANCE_DIAMETER = SHAFT_DIAMETER + 0.35 + PRINT_HOLE_COMPENSATION
M4_HEAT_INSERT_OD = 6.2
M4_HEAT_INSERT_BORE = 4.2
M4_HEAT_INSERT_LENGTH = 5.2
# Printed pilot is deliberately smaller than the knurled brass envelope.
# The calibration coupon also carries 5.2/5.4/5.6 variants so the first
# article can be tuned for the actual PETG-HF profile before full printing.
M4_HEAT_INSERT_HOLE_DIAMETER = 5.4
MIN_LOAD_WALL = 1.2

SPRING_COUNT = 2
SPRING_FREE_LENGTH = 111.0
SPRING_INSTALLED_LENGTH = 80.0
SPRING_SOLID_LENGTH = 38.0
SPRING_OD = 14.0
SPRING_WIRE = 1.10
SPRING_ACTIVE_COILS = 32.84
SPRING_RATE_PER_SPRING = 0.200  # N/mm
SPRING_SHEAR_MODULUS_MPA = 77000.0  # SUS304-WPB nominal screen value
SPRING_SHEAR_ALLOWABLE_MPA = 600.0  # conservative first-order screen
SPRING_LATERAL_INSET = 2.0
SPRING_NORMAL_OFFSET = 29.0
SPRING_GUIDE_SLEEVE_OD = 11.2
SPRING_GUIDE_SLEEVE_LENGTH = 10.0
SPRING_GUIDE_SLEEVE_BORE = 6.4  # passes the Ø6 TPU stop collar
SPRING_PRELOAD_PER_SPRING = (
    SPRING_FREE_LENGTH - SPRING_INSTALLED_LENGTH
) * SPRING_RATE_PER_SPRING
SPRING_GUIDE_DIAMETER = 3.0
SPRING_COIL_BIND_SAFETY = 2.0

# Stock constant-force return system.  MISUMI rates this family in kgf;
# CFS0.2 is 0.2 kgf, not 0.2 N.  The supplied polypropylene drum travels with
# the trackpad carriage and its supplied holed end plate fixes to the frame.
RETURN_SPRING_PART = "MISUMI CFS0.2"
RETURN_SPRING_COUNT = 2
RETURN_FORCE_PER_SIDE = 0.2 * GRAVITY  # 0.2 kgf = 1.96133 N
RETURN_FORCE_TOLERANCE_LOW = 0.0
RETURN_FORCE_TOLERANCE_HIGH = 0.15
RETURN_SPRING_LIFE_CYCLES = 35000
RETURN_SPRING_INITIAL_DEFLECTION = 45.0  # > approximately half a 26 mm turn
RETURN_SPRING_WORKING_DEFLECTION = 500.0
RETURN_SPRING_STRIP_WIDTH = 10.0
RETURN_SPRING_STRIP_THICKNESS = 0.13
RETURN_SPRING_DRUM_BORE = 8.2
RETURN_SPRING_DRUM_OD = 26.0
RETURN_SPRING_MOUNTED_COIL_OD = 26.0
RETURN_SPRING_ACCESSORY_PLATE_LENGTH = 18.0
RETURN_SPRING_ACCESSORY_PLATE_WIDTH = 10.0
RETURN_SPRING_ACCESSORY_PLATE_THICKNESS = 1.0
RETURN_SPRING_ACCESSORY_HOLE = 3.2
RETURN_SPOOL_TOTAL_WIDTH = 10.0
RETURN_SPOOL_CENTER = (-55.0, 71.6)  # ahead of the trackpad tray mounting tongue
RETURN_SPOOL_GLOBAL_X = 146.0
RETURN_SPOOL_POST_DIAMETER = 7.8
RETURN_SPOOL_POST_CENTER_X = 148.5
RETURN_SPOOL_POST_LENGTH = 23.0
RETURN_CLAMP_INSERT_OD = 4.6
RETURN_CLAMP_INSERT_BORE = 3.2
RETURN_CLAMP_INSERT_LENGTH = 5.0
RETURN_CLAMP_INSERT_HOLE_DIAMETER = 4.0

PETG_DENSITY_KG_PER_MM3 = 1.27e-6
TPU_DENSITY_KG_PER_MM3 = 1.21e-6

# Exact baseline; printed-part quantities are populated by the exporter.
BOM = {
    "MR84ZZ": {"quantity": 4, "spec": "4 x 8 x 3 mm shielded bearing"},
    "604ZZ": {"quantity": 8, "spec": "4 x 12 x 4 mm shielded bearing"},
    "constant_force_spring": {
        "quantity": 2,
        "spec": "MISUMI CFS0.2: 0.2 kgf (1.961 N), 500 mm max stroke, 35,000 cycles, supplied drum and holed accessory plate",
    },
    "TPU_return_spool_retainer": {
        "quantity": 2,
        "spec": "TPU 95A push-on cap for the integrated 7.8 mm printed spool post",
    },
    "M3x8_return_plate_screw": {
        "quantity": 2,
        "spec": "M3 x 8 mm socket-head screw; one per supplied spring end plate",
    },
    "M3_return_plate_insert": {
        "quantity": 2,
        "spec": "M3 heat-set insert, OD 4.6 x 5.0 mm; one per fixed spring anchor",
    },
    "spring_guide_rod": {
        "quantity": 2,
        "spec": "3.0 mm stainless precision rod x 122.0 mm, cut ends deburred",
    },
    "M4_shoulder_fastener_keyboard_guide": {
        "quantity": 4,
        "spec": "AMPG STR402M4X20HUL: 4 mm shoulder x 20 mm, M3 x 4 mm thread, head 6 x 1.3 mm",
    },
    "M4_shoulder_fastener_trackpad_guide": {
        "quantity": 4,
        "spec": "AMPG STR402M4X16HUL: 4 mm shoulder x 16 mm, M3 x 4 mm thread, head 6 x 1.3 mm",
    },
    "M4_shoulder_fastener_cross": {
        "quantity": 4,
        "spec": "AMPG STR402M4X12HUL: 4 mm shoulder x 12 mm, M3 x 4 mm thread, head 6 x 1.3 mm",
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
    "TPU_base_foot": {
        "quantity": 6,
        "spec": "TPU 95A, 24 x 18 x 2 mm, replaceable",
    },
}
