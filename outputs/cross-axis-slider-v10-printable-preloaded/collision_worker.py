"""One-pose OpenCascade collision worker used to isolate kernel failures."""

import json
import sys

from validation import validate_collision_pose


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: collision_worker.py TRAVEL VOLUME_TOLERANCE")
    report = validate_collision_pose(float(sys.argv[1]), float(sys.argv[2]))
    print(json.dumps(report, sort_keys=True))


if __name__ == "__main__":
    main()
