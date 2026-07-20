"""Sectioned/exploded review entry point for the V4 reel cartridge."""

from build123d import Compound

from v4_reel_cartridge import build_cartridge_exploded


def gen_step() -> Compound:
    """Return shared sectioned cartridge geometry without exporting a STEP file."""
    return build_cartridge_exploded(0.5)
