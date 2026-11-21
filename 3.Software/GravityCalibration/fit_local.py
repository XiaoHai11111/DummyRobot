"""Fit one fixed-pose gravity-current curve; never enables compensation."""
import argparse
import json
from pathlib import Path

import numpy as np

from calibration import make_pairs, fit_local, sine_design


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train", nargs="+", type=Path, required=True)
    parser.add_argument("--validate", nargs="+", type=Path, required=True,
                        help="Separate complete runs, never random rows from training")
    parser.add_argument("--joint", type=int, choices=range(1, 7), required=True)
    parser.add_argument("--current-limit-a", type=float, required=True,
                        help="Actual configured F1 current limit; not a new limit to send")
    parser.add_argument("--bin-deg", type=float, default=1.0)
    parser.add_argument("--max-accel-deg-s2", type=float, default=2.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if {p.resolve() for p in args.train} & {p.resolve() for p in args.validate}:
        parser.error("Training and validation files must be disjoint")
    # Real collector output has sidecar metadata. Refuse failed runs and mixed
    # loads/firmware. Imported CSV without a sidecar remains usable but is labelled.
    metadata_keys, missing_metadata = set(), []
    for path in [*args.train, *args.validate]:
        sidecar = path.with_suffix(".meta.json")
        if not sidecar.exists():
            missing_metadata.append(str(path))
            continue
        record = json.loads(sidecar.read_text())
        if not record.get("completed") or not record.get("stop_acknowledged"):
            raise ValueError(f"{path}: incomplete collection or unacknowledged stop")
        config = record["config"]
        if config["joint"] != args.joint or not np.isclose(config["current_limit_a"], args.current_limit_a):
            raise ValueError(f"{path}: joint/current limit differs from collection metadata")
        metadata_keys.add((config["payload_description"], config["firmware_revision"]))
    if len(metadata_keys) > 1:
        raise ValueError("Do not combine experiments with different loads or firmware")
    options = dict(joint=args.joint, current_limit_a=args.current_limit_a,
                   bin_deg=args.bin_deg, max_accel_deg_s2=args.max_accel_deg_s2)
    train, info = make_pairs(args.train, **options)
    validation, validation_info = make_pairs(args.validate, **options)
    held = np.arange(6) != args.joint - 1
    if (info["pose_id"] != validation_info["pose_id"] or
            np.any(abs(np.array(info["fixed_pose_deg"])[held] -
                       np.array(validation_info["fixed_pose_deg"])[held]) > 0.5)):
        raise ValueError("Validation must use the same held-joint pose")
    coefficients, metrics = fit_local(train)
    prediction = sine_design(np.deg2rad(validation[:, 0])) @ coefficients
    residual = prediction - validation[:, 1]
    metrics.update(validation_rmse_a=float(np.sqrt(np.mean(residual**2))),
                   validation_max_abs_error_a=float(np.max(abs(residual))),
                   validation_bias_a=float(np.mean(residual)))
    low, high = float(train[:, 0].min()), float(train[:, 0].max())
    # Slight bin-centre differences across independent runs are expected; no extrapolation
    # is permitted when evaluating the exported model.
    if validation[:, 0].min() < low - args.bin_deg or validation[:, 0].max() > high + args.bin_deg:
        raise ValueError("Validation exceeds the training range")
    low = max(low, float(validation[:, 0].min()))
    high = min(high, float(validation[:, 0].max()))
    if high <= low:
        raise ValueError("No shared training/validation range")
    model = dict(schema_version=1, model="fixed_pose_sine_current", joint=args.joint,
                 current_source="command", current_direction="robot_joint_positive",
                 coefficients_a=coefficients.tolist(), coefficient_order=["sin(q_rad)", "cos(q_rad)", "1"],
                 angle_range_deg=[low, high], pose_tolerance_deg=0.5,
                 current_limit_a=args.current_limit_a, enabled=False,
                 deployment_review_required=True, metrics=metrics,
                 training_files=[str(p.resolve()) for p in args.train],
                 validation_files=[str(p.resolve()) for p in args.validate],
                 files_without_metadata=missing_metadata,
                 validation_counts=validation_info, **info)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(model, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    for label, pairs in [("train", train), ("validation", validation)]:
        np.savetxt(args.output.with_suffix(f".{label}.csv"), pairs, delimiter=",",
                   header="q_deg,gravity_current_a,directional_loss_a,speed_deg_s", comments="")
    print(json.dumps(metrics, indent=2))
    print(f"Saved {args.output}; fixed-pose model only, compensation remains disabled.")


if __name__ == "__main__":
    main()
