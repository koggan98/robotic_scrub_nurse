#!/usr/bin/env python3

"""Convert tracked frame axes into a top-down gripper quaternion.

Convention used by this helper:
- RViz axis colors are assumed to be red=x, green=y, blue=z.
- TCP z-axis is the approach axis.
- TCP x-axis is the gripper/tool axis.
- TCP z = -world z.
- TCP x = frame z projected into the world-horizontal plane.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

from grasp_orientation_utils import compute_gripper_quaternion, normalize_vector


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compute a gripper quaternion from tracked frame axes."
    )
    parser.add_argument(
        "--frame-x",
        nargs=3,
        type=float,
        required=True,
        metavar=("X", "Y", "Z"),
        help="Frame x-axis vector.",
    )
    parser.add_argument(
        "--frame-y",
        nargs=3,
        type=float,
        required=True,
        metavar=("X", "Y", "Z"),
        help="Frame y-axis vector.",
    )
    parser.add_argument(
        "--frame-z",
        nargs=3,
        type=float,
        required=True,
        metavar=("X", "Y", "Z"),
        help="Frame z-axis vector.",
    )
    return parser.parse_args()

def main() -> int:
    args = parse_args()

    try:
        frame_x = normalize_vector(np.array(args.frame_x, dtype=np.float64), "frame_x")
        frame_y = normalize_vector(np.array(args.frame_y, dtype=np.float64), "frame_y")
        frame_z = normalize_vector(np.array(args.frame_z, dtype=np.float64), "frame_z")
        quaternion, warning = compute_gripper_quaternion(frame_x, frame_y, frame_z)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    if warning is not None:
        print(f"WARNING: {warning}", file=sys.stderr)

    print(
        "quat_xyzw=[{:.6f}, {:.6f}, {:.6f}, {:.6f}]".format(
            quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
