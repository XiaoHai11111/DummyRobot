#!/usr/bin/env python3
"""Compare deterministic MJCF wrist holding torque with the F4 C++ model.

This is an optional offline check. The standard firmware test runner does not
invoke it because MuJoCo and NumPy are not host-build dependencies.
"""

from __future__ import annotations

import os
import random
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
GRAVITY_SOURCE = ROOT / "2.Firmware/Core-STM32F4-fw/Robot/algorithms/gravity_compensation"
MJCF = ROOT / "3.Software/DummyMujoco/dummy_description/mjcf/scene.xml"
MAPPING = ROOT / "3.Software/DummyWeb/config.json"
CALIBRATION = ROOT / "3.Software/GravityCalibration"
POSE_COUNT = 65
TORQUE_TOLERANCE_NM = 1e-5


def deterministic_poses() -> list[list[float]]:
    sys.path.insert(0, str(CALIBRATION))
    from calibration import WRIST_STRICT_LIMITS_DEG

    generator = random.Random(0x4A354D4A)
    poses = [[0.0, 0.0, 90.0, 0.0, 0.0, 0.0]]
    for _ in range(POSE_COUNT - 1):
        poses.append([
            generator.uniform(float(low), float(high))
            for low, high in WRIST_STRICT_LIMITS_DEG
        ])
    return poses


def compile_probe(build_dir: Path) -> Path:
    probe_source = build_dir / "wrist_probe.cpp"
    probe_source.write_text(
        """#include "gravity_compensation.h"
#include <cstdio>
int main() {
    GravityCompensation::Model model;
    float q[6];
    while (std::scanf("%f%f%f%f%f%f",
                      &q[0], &q[1], &q[2], &q[3], &q[4], &q[5]) == 6) {
        float torque[7]{};
        model.CalculateGravityTorqueNm(q, torque);
        std::printf("%.9g %.9g\\n", torque[4], torque[5]);
    }
}
""",
        encoding="utf-8",
    )
    probe = build_dir / "wrist_probe"
    compiler = os.environ.get("CXX", "g++")
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-O2",
            f"-I{GRAVITY_SOURCE}",
            str(probe_source),
            str(GRAVITY_SOURCE / "gravity_compensation.cpp"),
            "-o",
            str(probe),
        ],
        check=True,
    )
    return probe


def run_cpp_probe(probe: Path, poses: list[list[float]]) -> list[tuple[float, float]]:
    stdin = "".join(" ".join(f"{value:.17g}" for value in pose) + "\n" for pose in poses)
    result = subprocess.run(
        [str(probe)],
        input=stdin,
        text=True,
        capture_output=True,
        check=True,
    )
    values = [tuple(float(value) for value in line.split())
              for line in result.stdout.splitlines() if line.strip()]
    if len(values) != len(poses) or any(len(value) != 2 for value in values):
        raise RuntimeError(f"Unexpected C++ probe output: {result.stdout!r}")
    return values


def main() -> int:
    try:
        import mujoco  # noqa: F401
        import numpy as np
    except ImportError as error:
        print(f"SKIP wrist gravity MJCF check: optional dependency missing ({error.name})")
        return 0

    poses = deterministic_poses()
    sys.path.insert(0, str(CALIBRATION))
    from gravity_preview import GravityTorqueEvaluator

    evaluator = GravityTorqueEvaluator(MAPPING, MJCF, gripper_rad=0.0)
    with tempfile.TemporaryDirectory(prefix="dummy-wrist-gravity.") as temp:
        actual = run_cpp_probe(compile_probe(Path(temp)), poses)

    errors = []
    for pose, measured in zip(poses, actual):
        expected = evaluator.joint_torque(np.asarray(pose))
        errors.append((abs(measured[0] - expected[3]),
                       abs(measured[1] - expected[4])))
    worst = max(
        ((error[0], index, "J4") for index, error in enumerate(errors)),
        default=(0.0, 0, "J4"),
    )
    j5_worst = max(
        ((error[1], index, "J5") for index, error in enumerate(errors)),
        default=(0.0, 0, "J5"),
    )
    if j5_worst[0] > worst[0]:
        worst = j5_worst
    print(
        f"{POSE_COUNT}-pose MJCF/C++ wrist torque max error: "
        f"{worst[0]:.9g} Nm ({worst[2]} pose {worst[1]})"
    )
    if worst[0] >= TORQUE_TOLERANCE_NM:
        print(f"FAIL: tolerance is {TORQUE_TOLERANCE_NM:g} Nm", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
