"""Terminal pose with conservative keyboard and trackpad device envelopes."""

from assembly import build_pose


def gen_step():
    return build_pose(1.0, include_devices=True).compound
