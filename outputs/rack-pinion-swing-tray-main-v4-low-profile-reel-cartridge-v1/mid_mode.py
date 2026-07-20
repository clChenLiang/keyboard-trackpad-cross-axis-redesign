from build123d import Compound

from v4_assembly import build_pose


def gen_step() -> Compound:
    return build_pose(0.5)
