"""Units and offline fitting for a fixed-pose, single-joint experiment.

All fitted currents use the positive ROBOT JOINT direction; raw telemetry uses
the F1 MOTOR direction. No routine in this module communicates with hardware.
"""
from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

MOTOR_SIGNS = np.array([1, -1, -1, 1, -1, -1])
REDUCTIONS = np.array([50, 30, 30, 30, 30, 50])
JOINT_LIMITS_DEG = np.array([[-170, 170], [-75, 90], [35, 180],
                             [-180, 180], [-120, 120], [-720, 720]])
Q_FIELDS = [f"q{j}_deg" for j in range(1, 7)]
FIELDS = ["t_s", "cycle", "phase", "joint", "pose_id", "current_source",
          "position_tick_ms", "velocity_tick_ms", "current_tick_ms", "skew_ms",
          "roundtrip_ms", "dq_deg_s", "i_cmd_motor_a", "driver_state", "fault_flags",
          "status_supported", "enabled", "robot_fault_mask", *Q_FIELDS]


def tick_skew_ms(ticks):
    """Span of nearby uint32 receive times, including the 49.7 day wrap."""
    first = int(ticks[0])
    offsets = [((int(t) - first + 2**31) % 2**32) - 2**31 for t in ticks]
    return max(offsets) - min(offsets)


def sine_design(q_rad):
    q = np.asarray(q_rad)
    return np.column_stack([np.sin(q), np.cos(q), np.ones_like(q)])


def load_rows(path):
    with Path(path).open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        if not set(FIELDS).issubset(reader.fieldnames or []):
            raise ValueError(f"{path}: CSV schema mismatch")
        rows = list(reader)
    if len(rows) < 10:
        raise ValueError(f"{path}: too few samples")
    if any(row["current_source"] != "command" for row in rows):
        raise ValueError("This fitter expects command current, not measured ADC current")
    return rows


def make_pairs(paths, joint, current_limit_a, *, bin_deg=1.0,
               min_speed_deg_s=0.3, max_accel_deg_s2=2.0,
               max_skew_ms=10.0, pose_tolerance_deg=0.5):
    """Pair equal-position/equal-speed forward and reverse runs within each cycle.

    Averages remove only approximately odd friction. Returned gravity current
    can still contain sensor/drive bias, cables, and asymmetric transmission loss.
    """
    if joint not in range(1, 7) or not 0 < current_limit_a <= 3.3:
        raise ValueError("Invalid joint or verified current limit")
    if not bin_deg > 0 or not pose_tolerance_deg > 0:
        raise ValueError("Bin width and pose tolerance must be positive")
    pairs, poses, total, retained = [], [], 0, 0
    pose_ids = set()
    for path in paths:
        rows = load_rows(path)
        pose_ids.update(row["pose_id"] for row in rows)
        if any(int(r["joint"]) != joint for r in rows):
            raise ValueError(f"{path}: wrong active joint")
        keys = ["t_s", "dq_deg_s", "i_cmd_motor_a", "skew_ms", "roundtrip_ms",
                "driver_state", "fault_flags", "status_supported", "enabled",
                "robot_fault_mask", "cycle", *Q_FIELDS]
        values = np.array([[float(row[k]) for k in keys] for row in rows])
        if not np.isfinite(values).all():
            raise ValueError(f"{path}: non-finite sample")
        t, v, current = values[:, 0], values[:, 1], values[:, 2] * MOTOR_SIGNS[joint - 1]
        if np.any(np.diff(t) <= 0):
            raise ValueError(f"{path}: time must increase strictly")
        # Velocity is the F1 encoder estimate, not the motion setpoint.
        smoothed = np.convolve(np.pad(v, (2, 2), mode="edge"), np.ones(5) / 5, mode="valid")
        acceleration = np.gradient(smoothed, t)
        q_all = values[:, -6:]
        poses.append(q_all)
        good = ((abs(v) >= min_speed_deg_s) & (abs(acceleration) <= max_accel_deg_s2)
                & (abs(current) < 0.9 * current_limit_a)
                & (values[:, 3] <= max_skew_ms) & (values[:, 3] >= 0)
                & (values[:, 4] <= 100) & (values[:, 4] >= 0)
                & np.isin(values[:, 5], [1, 2]) & (values[:, 6] == 0)
                & (values[:, 7] == 1) & (values[:, 8] == 1) & (values[:, 9] == 0))
        good[:3] = False
        good[-3:] = False
        # Exclude boundaries/reversals/gaps with a real-time guard on both sides.
        changes = np.flatnonzero((np.sign(v[1:]) != np.sign(v[:-1])) |
                                 (np.diff(t) > 0.2)) + 1
        for edge in changes:
            good[abs(t - t[edge]) < 0.4] = False
        buckets = {}
        for k, row in enumerate(rows):
            if not good[k] or row["phase"] not in {"forward", "reverse"}:
                continue
            # 0.5 deg/s speed bins; require matched median speed below as well.
            key = (int(row["cycle"]), int(np.floor(q_all[k, joint - 1] / bin_deg)),
                   int(np.floor(abs(v[k]) / 0.5 + 0.5)))
            buckets.setdefault(key, [[], []])[0 if v[k] > 0 else 1].append(k)
            retained += 1
        total += len(rows)
        for forward, reverse in buckets.values():
            if min(len(forward), len(reverse)) < 3:
                continue
            p, n = np.asarray(forward), np.asarray(reverse)
            vp, vn = np.median(abs(v[p])), np.median(abs(v[n]))
            qp, qn = np.mean(q_all[p, joint - 1]), np.mean(q_all[n, joint - 1])
            if abs(vp - vn) > max(0.1, 0.1 * max(vp, vn)) or abs(qp - qn) > bin_deg / 2:
                continue
            ip, im = np.median(current[p]), np.median(current[n])
            pairs.append([(qp + qn) / 2, (ip + im) / 2, (ip - im) / 2, (vp + vn) / 2])
    if len(pose_ids) != 1:
        raise ValueError("A local model must have exactly one pose_id")
    all_q = np.concatenate(poses)
    held = np.arange(6) != joint - 1
    if np.any(np.ptp(all_q[:, held], axis=0) > pose_tolerance_deg):
        raise ValueError("Other joints moved: use a separate local fit for each fixed pose")
    if len(pairs) < 12:
        raise ValueError("Need >=12 matched angle/speed bins; check reversals, faults and saturation")
    return np.asarray(pairs), {
        "raw_samples": total, "retained_samples": retained, "paired_bins": len(pairs),
        "pose_id": next(iter(pose_ids)), "fixed_pose_deg": np.median(all_q, axis=0).tolist(),
    }


def fit_local(pairs):
    design = sine_design(np.deg2rad(pairs[:, 0]))
    coeff, _, rank, singular = np.linalg.lstsq(design, pairs[:, 1], rcond=None)
    condition = float(singular[0] / singular[-1]) if singular[-1] else float("inf")
    if rank < 3 or condition > 1e4:
        raise ValueError("Gravity fit is ill-conditioned: increase safe angle coverage")
    residual = pairs[:, 1] - design @ coeff
    return coeff, {"rank": int(rank), "condition_number": condition,
                   "train_rmse_a": float(np.sqrt(np.mean(residual**2)))}


def predict_local(model, q_deg):
    """Joint-positive current in A. Refuse extrapolation or a different held pose."""
    q = np.asarray(q_deg, dtype=float)
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError("Expected six finite joint angles")
    j = model["joint"] - 1
    held = np.arange(6) != j
    if np.any(abs(q[held] - np.asarray(model["fixed_pose_deg"])[held]) > model["pose_tolerance_deg"]):
        raise ValueError("Outside the fitted fixed pose")
    low, high = model["angle_range_deg"]
    if not low <= q[j] <= high:
        raise ValueError("Outside the fitted angle range")
    return float(sine_design(np.deg2rad([q[j]]))[0] @ model["coefficients_a"])


def limited_feedforward(target_motor_a, previous_motor_a, dt_s, *, alpha,
                        limit_a, slew_a_s):
    """Offline reference for F1 implementation; does not include feedback current.

    F1 must ALSO limit feedback+feedforward and include feedforward in anti-windup.
    Fault/STOP handling must clear the FF state separately, without this ramp.
    """
    vals = [target_motor_a, previous_motor_a, dt_s, alpha, limit_a, slew_a_s]
    if not np.isfinite(vals).all() or not 0 <= alpha <= 1 or min(dt_s, limit_a, slew_a_s) <= 0:
        raise ValueError("Invalid feedforward limiter parameters")
    target = np.clip(alpha * target_motor_a, -limit_a, limit_a)
    ramped = np.clip(target, previous_motor_a - slew_a_s * dt_s,
                     previous_motor_a + slew_a_s * dt_s)
    return float(np.clip(ramped, -limit_a, limit_a))

