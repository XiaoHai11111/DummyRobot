"""Generate SYNTHETIC traces to exercise fitting; these are not robot parameters."""
import argparse
import csv
from pathlib import Path

import numpy as np

from calibration import FIELDS, MOTOR_SIGNS, Q_FIELDS


def generate(path, seed, speed=2.0):
    rng = np.random.default_rng(seed)
    dt, clock = 0.05, 0.0
    with Path(path).open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=FIELDS)
        writer.writeheader()
        for cycle in range(1, 5):
            for direction, phase in [(1, "forward"), (-1, "reverse")]:
                sweep = np.arange(-35, 30, speed * dt)
                if direction < 0:
                    sweep = sweep[::-1]
                for q in sweep:
                    v = direction * speed
                    amps = (0.3 * np.sin(np.deg2rad(q)) + 0.6 * np.cos(np.deg2rad(q)) + 0.05
                            + 0.12 * direction + 0.01 * v + rng.normal(0, 0.004))
                    tick = round(clock * 1000)
                    row = dict(t_s=clock, cycle=cycle, phase=phase, joint=2,
                               pose_id="SYNTHETIC_P0", current_source="command",
                               position_tick_ms=tick, velocity_tick_ms=tick, current_tick_ms=tick,
                               skew_ms=0, roundtrip_ms=3, dq_deg_s=v,
                               i_cmd_motor_a=amps * MOTOR_SIGNS[1], driver_state=2, fault_flags=0,
                               status_supported=1, enabled=1, robot_fault_mask=0)
                    row.update(zip(Q_FIELDS, [0, q, 90, 0, 0, 0]))
                    writer.writerow(row)
                    clock += dt


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("data/synthetic"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    generate(args.output_dir / "train.csv", 10)
    generate(args.output_dir / "validation.csv", 20)
    print(f"Synthetic data written to {args.output_dir}; do not deploy fitted values.")

