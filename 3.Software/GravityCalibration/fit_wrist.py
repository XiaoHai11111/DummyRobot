"""Reproducible offline J4/J5 holding-current and odd-friction fitting.

The fitter discovers only CSV files whose sibling metadata explicitly records
both completed=true and stop_acknowledged=true. It never imports or opens a
serial port. Outputs are written only to a dedicated, manifest-managed
candidate directory.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import tempfile
from typing import Any

import numpy as np

from calibration import (JOINT_LIMITS_DEG, MOTOR_SIGNS,
                         WRIST_HISTORICAL_LIMITS_DEG, WRIST_STRICT_LIMITS_DEG,
                         make_pairs)
from gravity_preview import GravityTorqueEvaluator

HERE = Path(__file__).resolve().parent
DEFAULT_MAPPING = HERE.parent / "DummyWeb" / "config.json"
DEFAULT_MJCF = HERE.parent / "DummyMujoco" / "dummy_description" / "mjcf" / "scene.xml"
DEFAULT_OUTPUT = HERE / "results" / "j45_offline_fit_candidates_v2"
OUTPUT_MANIFEST = "manifest.json"
MANAGED_BY = "fit_wrist.py"
HARMONIC_ORDER = 30.0
OFFLINE_JOINT_LIMIT_TOLERANCE_DEG = 0.05
LOCAL_COMP_HARMONIC_LIMIT_MA = 500
INDEPENDENT_RETEST_ROUND = "independent_retest"

E_NAMES = ["gravity_scale", "bias", "harmonic_sin", "harmonic_cos"]
O_NAMES = ["odd_constant", "harmonic_sin", "harmonic_cos"]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def normalize_payload_description(description: str) -> str:
    """Remove pose annotations while retaining the physical payload wording."""
    normalized = re.sub(
        r"(?i)(?:,\s*)J[4-6]\s*=\s*[+-]?(?:\d+(?:\.\d*)?|\.\d+)\s*",
        ",",
        description,
    )
    normalized = re.sub(
        r"(?i)(?:,\s*)combined\s+pose\s+[A-Za-z0-9_+-]+\s*",
        ",",
        normalized,
    )
    return ",".join(part.strip() for part in normalized.split(",") if part.strip())


def grouped_payload_description(config: dict[str, Any]) -> str:
    """Keep explicit hardware wiring identity in a separate payload group."""
    payload = normalize_payload_description(str(config.get("payload_description", "")))
    identity = config.get("payload_identity")
    if identity is None:
        return payload
    if not isinstance(identity, dict):
        raise ValueError("payload_identity must be an object")
    canonical = json.dumps(identity, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return f"{payload}|hardware:{canonical}"


def grouped_firmware_identity(config: dict[str, Any]) -> str:
    revision = str(config.get("firmware_revision", "unspecified"))
    firmware_hash = config.get("firmware_hash")
    if firmware_hash in (None, ""):
        return revision
    return f"{revision}@sha256:{str(firmware_hash).lower()}"


def slug(text: str, limit: int = 64) -> str:
    value = re.sub(r"[^a-z0-9]+", "_", text.lower()).strip("_")
    return (value or "unlabelled")[:limit].rstrip("_")


def harmonic_features(q_deg: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    phase = np.deg2rad(HARMONIC_ORDER * np.asarray(q_deg, dtype=float))
    return np.sin(phase), np.cos(phase)


def round_half_away_from_zero(value: float) -> int:
    if not math.isfinite(value):
        raise ValueError("Cannot round a non-finite motor parameter")
    return (math.floor(value + 0.5) if value >= 0
            else math.ceil(value - 0.5))


def even_design(observations: list[dict[str, Any]]) -> tuple[np.ndarray, list[str]]:
    q = np.asarray([row["q_deg"] for row in observations])
    torque = np.asarray([row["gravity_torque_nm"] for row in observations])
    sine, cosine = harmonic_features(q)
    design = np.column_stack([torque, np.ones_like(q), sine, cosine])
    return design, E_NAMES.copy()


def odd_design(observations: list[dict[str, Any]], include_speed: bool
               ) -> tuple[np.ndarray, list[str]]:
    q = np.asarray([row["q_deg"] for row in observations])
    speed = np.asarray([row["speed_magnitude_deg_s"] for row in observations])
    sine, cosine = harmonic_features(q)
    columns = [np.ones_like(q), sine, cosine]
    names = O_NAMES.copy()
    if include_speed:
        columns.append(speed)
        names.append("odd_speed")
    return np.column_stack(columns), names


def equal_run_weights(observations: list[dict[str, Any]]) -> np.ndarray:
    counts = Counter(row["run_id"] for row in observations)
    weights = np.asarray([1.0 / counts[row["run_id"]] for row in observations])
    return weights / float(np.mean(weights))


def weighted_linear_fit(observations: list[dict[str, Any]], design: np.ndarray,
                        response: np.ndarray) -> dict[str, Any]:
    """Fit with equal total weight per independent run."""
    x = np.asarray(design, dtype=float)
    y = np.asarray(response, dtype=float)
    if x.ndim != 2 or len(x) != len(y) or not np.isfinite(x).all() or not np.isfinite(y).all():
        raise ValueError("Invalid regression matrices")
    weights = equal_run_weights(observations)
    root_weights = np.sqrt(weights)
    weighted_x = x * root_weights[:, None]
    weighted_y = y * root_weights
    coefficients, _, rank, singular = np.linalg.lstsq(weighted_x, weighted_y, rcond=None)
    rank = int(rank)
    if rank != x.shape[1]:
        raise ValueError(f"Design rank {rank} is below parameter count {x.shape[1]}")
    residual = y - x @ coefficients
    weighted_sse = float(np.sum(weights * residual**2))
    degrees_freedom = max(len(y) - rank, 1)
    covariance = (weighted_sse / degrees_freedom) * np.linalg.pinv(weighted_x.T @ weighted_x)
    standard_error = np.sqrt(np.maximum(np.diag(covariance), 0.0))
    weighted_mean = float(np.average(y, weights=weights))
    centered = float(np.sum(weights * (y - weighted_mean) ** 2))
    r_squared = 1.0 - weighted_sse / centered if centered > 0 else float("nan")
    condition = float(singular[0] / singular[-1]) if singular[-1] > 0 else float("inf")
    return {
        "coefficients": coefficients,
        "standard_error": standard_error,
        "covariance": covariance,
        "rank": rank,
        "condition_number": condition,
        "weighted_sse": weighted_sse,
        "r_squared": r_squared,
        "residual": residual,
        "weights": weights,
    }


def prediction_metrics(observations: list[dict[str, Any]], prediction: np.ndarray,
                       actual: np.ndarray) -> dict[str, Any]:
    residual = np.asarray(prediction) - np.asarray(actual)
    by_run = []
    for run_id in sorted({row["run_id"] for row in observations}):
        indexes = np.asarray([index for index, row in enumerate(observations)
                              if row["run_id"] == run_id])
        run_residual = residual[indexes]
        by_run.append({
            "run_id": run_id,
            "pairs": int(len(indexes)),
            "rmse_a": float(np.sqrt(np.mean(run_residual**2))),
            "mae_a": float(np.mean(abs(run_residual))),
            "bias_prediction_minus_actual_a": float(np.mean(run_residual)),
        })
    return {
        "pooled_rmse_a": float(np.sqrt(np.mean(residual**2))),
        "pooled_mae_a": float(np.mean(abs(residual))),
        "pooled_bias_prediction_minus_actual_a": float(np.mean(residual)),
        "run_macro_rmse_a": float(np.mean([row["rmse_a"] for row in by_run])),
        "run_macro_mae_a": float(np.mean([row["mae_a"] for row in by_run])),
        "run_macro_bias_prediction_minus_actual_a":
            float(np.mean([row["bias_prediction_minus_actual_a"] for row in by_run])),
        "runs": by_run,
    }


def evaluate_design(observations: list[dict[str, Any]], design: np.ndarray,
                    response: np.ndarray) -> dict[str, Any]:
    fit = weighted_linear_fit(observations, design, response)
    prediction = design @ fit["coefficients"]
    return {
        "fit": fit,
        "metrics": prediction_metrics(observations, prediction, response),
    }


def assess_speed_term(observations: list[dict[str, Any]]) -> dict[str, Any]:
    measured_speeds = np.asarray(
        [row["speed_magnitude_deg_s"] for row in observations], dtype=float)
    commanded_speeds = np.asarray(
        [row["commanded_speed_deg_s"] for row in observations], dtype=float)
    unique_commanded = np.unique(np.round(commanded_speeds, 3))
    response = np.asarray([row["odd_current_a"] for row in observations])
    baseline_design, _ = odd_design(observations, include_speed=False)
    baseline = evaluate_design(observations, baseline_design, response)
    candidate_design, _ = odd_design(observations, include_speed=True)
    try:
        candidate = evaluate_design(observations, candidate_design, response)
    except ValueError as error:
        return {
            "included": False,
            "reason": str(error),
            "commanded_speed_levels_deg_s": [float(value) for value in unique_commanded],
            "unique_commanded_speed_levels": int(len(unique_commanded)),
            "measured_speed_range_deg_s":
                [float(measured_speeds.min()), float(measured_speeds.max())],
            "candidate_coefficient_a_per_deg_s": None,
            "candidate_standard_error_a_per_deg_s": None,
        }
    coefficient = float(candidate["fit"]["coefficients"][-1])
    standard_error = float(candidate["fit"]["standard_error"][-1])
    n = len(response)
    baseline_bic = n * math.log(max(baseline["fit"]["weighted_sse"], 1e-24) / n) + 3 * math.log(n)
    candidate_bic = n * math.log(max(candidate["fit"]["weighted_sse"], 1e-24) / n) + 4 * math.log(n)
    significant = coefficient > 1.96 * standard_error
    improved = candidate_bic + 2.0 < baseline_bic
    enough_levels = len(unique_commanded) >= 2
    commanded_span = (float(unique_commanded.max() - unique_commanded.min())
                      if len(unique_commanded) else 0.0)
    meaningful_span = commanded_span >= max(
        0.2, 0.1 * float(unique_commanded.max() if len(unique_commanded) else 0.0))
    included = bool(enough_levels and meaningful_span and significant and improved)
    reasons = []
    if not enough_levels:
        reasons.append("fewer_than_two_speed_levels")
    if not meaningful_span:
        reasons.append("commanded_speed_span_too_small")
    if not significant:
        if coefficient <= 0:
            reasons.append("non_positive_viscous_term_not_deployable")
        else:
            reasons.append("coefficient_not_95pct_significant")
    if not improved:
        reasons.append("bic_did_not_improve_by_2")
    return {
        "included": included,
        "reason": "significant_speed_term" if included else ",".join(reasons),
        "commanded_speed_levels_deg_s": [float(value) for value in unique_commanded],
        "unique_commanded_speed_levels": int(len(unique_commanded)),
        "commanded_speed_span_deg_s": commanded_span,
        "measured_speed_range_deg_s":
            [float(measured_speeds.min()), float(measured_speeds.max())],
        "candidate_coefficient_a_per_deg_s": coefficient,
        "candidate_standard_error_a_per_deg_s": standard_error,
        "non_negative_physical_sign_required": True,
        "candidate_bic": float(candidate_bic),
        "constant_model_bic": float(baseline_bic),
    }


def discover_runs(data_root: Path, joints: set[int], evaluator: GravityTorqueEvaluator,
                  bin_deg: float, max_accel_deg_s2: float,
                  time_source: str = "host"
                  ) -> tuple[list[dict[str, Any]], dict[str, int], list[dict[str, str]]]:
    runs = []
    exclusions: Counter[str] = Counter()
    failures = []
    metadata_paths = sorted(data_root.rglob("*.meta.json"))
    for metadata_path in metadata_paths:
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            exclusions["invalid_metadata"] += 1
            continue
        if metadata.get("completed") is not True:
            exclusions["not_completed"] += 1
            continue
        if metadata.get("stop_acknowledged") is not True:
            exclusions["stop_not_acknowledged"] += 1
            continue
        config = metadata.get("config")
        if not isinstance(config, dict) or config.get("joint") not in joints:
            exclusions["not_requested_joint"] += 1
            continue
        csv_path = metadata_path.with_suffix("").with_suffix(".csv")
        if not csv_path.is_file():
            exclusions["csv_missing"] += 1
            continue
        joint = int(config["joint"])
        try:
            current_limit = float(config["current_limit_a"])
            gripper_rad = float(config.get("gripper_rad", 0.0))
            if not math.isfinite(gripper_rad) or not 0 <= gripper_rad <= 0.78:
                raise ValueError("gripper_rad is outside 0..0.78 rad")
            pairs, info = make_pairs(
                [csv_path],
                joint,
                current_limit,
                bin_deg=bin_deg,
                max_accel_deg_s2=max_accel_deg_s2,
                time_source=time_source,
            )
            if pairs.shape[1] != 4:
                raise ValueError("make_pairs did not return the traditional four-column contract")
            observations = []
            fixed_pose = np.asarray(info["fixed_pose_deg"], dtype=float)
            for pair in pairs:
                q_all = fixed_pose.copy()
                q_all[joint - 1] = float(pair[0])
                if np.any(q_all < WRIST_STRICT_LIMITS_DEG[:, 0]
                          - OFFLINE_JOINT_LIMIT_TOLERANCE_DEG) or np.any(
                              q_all > WRIST_STRICT_LIMITS_DEG[:, 1]
                              + OFFLINE_JOINT_LIMIT_TOLERANCE_DEG):
                    raise ValueError("Data lies outside the wrist strict joint limits")
                torque = evaluator.joint_torque(q_all, gripper_rad=gripper_rad)
                observations.append({
                    "run_id": csv_path.relative_to(data_root).as_posix(),
                    "source": csv_path.relative_to(data_root).as_posix(),
                    "metadata": metadata_path.relative_to(data_root).as_posix(),
                    "pose_id": str(info["pose_id"]),
                    "commanded_speed_deg_s": float(config["speed_deg_s"]),
                    "q_deg": float(pair[0]),
                    "q_all_deg": q_all.tolist(),
                    "even_current_a": float(pair[1]),
                    "odd_current_a": float(pair[2]),
                    "speed_magnitude_deg_s": float(pair[3]),
                    "gravity_torque_nm": float(torque[joint - 1]),
                })
        except Exception as error:
            failures.append({
                "source": csv_path.relative_to(data_root).as_posix(),
                "error": f"{type(error).__name__}: {error}",
            })
            exclusions["pairing_or_domain_failure"] += 1
            continue
        runs.append({
            "csv_path": csv_path,
            "metadata_path": metadata_path,
            "source": csv_path.relative_to(data_root).as_posix(),
            "metadata": metadata_path.relative_to(data_root).as_posix(),
            "metadata_record": metadata,
            "config": config,
            "gripper_rad": gripper_rad,
            "pairs": pairs,
            "info": info,
            "observations": observations,
            "csv_sha256": sha256_file(csv_path),
            "metadata_sha256": sha256_file(metadata_path),
        })
    exclusions["qualified_runs"] = len(runs)
    return runs, dict(sorted(exclusions.items())), failures


def group_runs(runs: list[dict[str, Any]], joints: set[int]
               ) -> dict[tuple[int, str, str, float], list[dict[str, Any]]]:
    groups: dict[tuple[int, str, str, float], list[dict[str, Any]]] = defaultdict(list)
    for run in runs:
        config = run["config"]
        joint = int(config["joint"])
        firmware = grouped_firmware_identity(config)
        payload = grouped_payload_description(config)
        groups[(joint, firmware, payload, float(run["gripper_rad"]))].append(run)
    return {key: groups[key] for key in sorted(groups) if key[0] in joints}


def is_independent_retest(run: dict[str, Any]) -> bool:
    config_label = run.get("config", {}).get("measurement_round")
    metadata_label = run.get("metadata_record", {}).get("measurement_round")
    return INDEPENDENT_RETEST_ROUND in (config_label, metadata_label)


def partition_training_and_retest_runs(runs: list[dict[str, Any]]
                                       ) -> tuple[list[dict[str, Any]],
                                                  list[dict[str, Any]]]:
    retest = [run for run in runs if is_independent_retest(run)]
    retest_ids = {id(run) for run in retest}
    training = [run for run in runs if id(run) not in retest_ids]
    return training, retest


def source_file_records(runs: list[dict[str, Any]], default_gripper_rad: float
                        ) -> list[dict[str, Any]]:
    records = []
    for run in runs:
        records.append({
            "csv": run["source"],
            "metadata": run["metadata"],
            "csv_sha256": run["csv_sha256"],
            "metadata_sha256": run["metadata_sha256"],
            "pose_id": run["info"]["pose_id"],
            "speed_deg_s": float(run["config"]["speed_deg_s"]),
            "measurement_round": run["config"].get("measurement_round"),
            "gripper_rad": float(run.get("gripper_rad", default_gripper_rad)),
            "pairs": len(run["pairs"]),
            "raw_samples": int(run["info"]["raw_samples"]),
            "retained_samples": int(run["info"]["retained_samples"]),
            "time_source": str(run["info"].get("time_source", "host")),
            "position_tick_wrap_count":
                int(run["info"].get("position_tick_wrap_count", 0)),
        })
    return records


def evaluate_frozen_retest(candidate: dict[str, Any],
                           observations: list[dict[str, Any]]) -> dict[str, Any]:
    if not observations:
        return {
            "present": False,
            "excluded_from_training": True,
            "runs": 0,
            "pairs": 0,
            "even": None,
            "odd": None,
        }
    even_model = candidate["model"]["joint_positive"]["even_current_a"]
    odd_model = candidate["model"]["joint_positive"]["odd_current_a"]
    even_x, _ = even_design(observations)
    odd_x, _ = odd_design(
        observations,
        include_speed=bool(candidate["speed_term_identifiability"]["included"]),
    )
    even_coefficients = np.asarray([
        even_model["gravity_scale_a_per_nm"],
        even_model["bias_a"],
        even_model["harmonic_sin_a"],
        even_model["harmonic_cos_a"],
    ])
    odd_coefficients = np.asarray([
        odd_model["constant_a"],
        odd_model["harmonic_sin_a"],
        odd_model["harmonic_cos_a"],
    ])
    if odd_x.shape[1] == 4:
        odd_coefficients = np.asarray([
            odd_model["constant_a"],
            odd_model["harmonic_sin_a"],
            odd_model["harmonic_cos_a"],
            odd_model["speed_coefficient_a_per_deg_s"],
        ])
    even_actual = np.asarray([row["even_current_a"] for row in observations])
    odd_actual = np.asarray([row["odd_current_a"] for row in observations])
    return {
        "present": True,
        "excluded_from_training": True,
        "coefficient_source": "candidate frozen after training-only fit",
        "runs": len({row["run_id"] for row in observations}),
        "pairs": len(observations),
        "even": prediction_metrics(
            observations, even_x @ even_coefficients, even_actual),
        "odd": prediction_metrics(
            observations, odd_x @ odd_coefficients, odd_actual),
    }


def coefficient_records(names: list[str], fit: dict[str, Any], motor_sign: int
                        ) -> list[dict[str, Any]]:
    records = []
    for name, value, standard_error in zip(
            names, fit["coefficients"], fit["standard_error"]):
        half_width = 1.96 * float(standard_error)
        records.append({
            "name": name,
            "value": float(value),
            "standard_error": float(standard_error),
            "ci95": [float(value - half_width), float(value + half_width)],
            "motor_value": float(motor_sign * value),
            "motor_ci95": [
                float(motor_sign * (value - half_width)),
                float(motor_sign * (value + half_width)),
            ],
        })
    return records


def rounded_motor_parameter(value: float, unit: str) -> dict[str, Any]:
    rounded = round_half_away_from_zero(value)
    return {
        "unscaled_value": float(value),
        "rounded_value": int(rounded),
        "unit": unit,
        "rounding": "nearest integer; exact halves away from zero",
    }


def motor_phase_parameters(joint: int, motor_sign: int,
                           even_values: dict[str, dict[str, Any]],
                           odd_values: dict[str, dict[str, Any]],
                           speed_assessment: dict[str, Any]
                           ) -> tuple[dict[str, Any], list[str]]:
    """Convert joint-angle coefficients to F1's physical motor-angle model."""
    reduction = int(30)
    e_sin_ma = 1000.0 * float(even_values["harmonic_sin"]["value"])
    e_cos_ma = 1000.0 * motor_sign * float(even_values["harmonic_cos"]["value"])
    o_sin_ma = 1000.0 * motor_sign * float(odd_values["harmonic_sin"]["value"])
    o_cos_ma = 1000.0 * float(odd_values["harmonic_cos"]["value"])
    coulomb_ma = 1000.0 * float(odd_values["odd_constant"]["value"])
    viscous_joint = 1000.0 * float(
        odd_values.get("odd_speed", {}).get("value", 0.0))
    viscous_motor = viscous_joint * 360.0 / reduction

    harmonics = {
        "e_sin": rounded_motor_parameter(e_sin_ma, "mA"),
        "e_cos": rounded_motor_parameter(e_cos_ma, "mA"),
        "o_sin": rounded_motor_parameter(o_sin_ma, "mA"),
        "o_cos": rounded_motor_parameter(o_cos_ma, "mA"),
    }
    harmonic_values_ok = all(
        abs(record["rounded_value"]) <= LOCAL_COMP_HARMONIC_LIMIT_MA
        for record in harmonics.values()
    )
    speed_term_included = bool(speed_assessment["included"])
    blockers = []
    if not harmonic_values_ok:
        blockers.append(
            f"rounded harmonic exceeds the firmware ±{LOCAL_COMP_HARMONIC_LIMIT_MA} mA limit")
    if not speed_term_included:
        blockers.append("viscous term is omitted because speed identifiability failed")
        if (joint == 5
                and int(speed_assessment.get("unique_commanded_speed_levels", 0)) < 2):
            blockers.append("J5 single-speed data cannot deploy a viscous term")
    if viscous_motor < 0:
        blockers.append("negative viscous coefficient is not deployable")

    parameter = {
        "schema_version": 1,
        "phase_expression":
            "psi_motor_rad = motor_sign_J * 30 * radians(q_robot_deg)",
        "motor_sign_J": int(motor_sign),
        "motor_phase_offset_rad": 0.0,
        "phase_offset_status":
            "F1 phase is encoder-home referenced; zero offset is a model convention pending hardware phase verification",
        "motion_direction":
            "dir_motor = sign(motor angular velocity); dir_motor = motor_sign_J * dir_joint",
        "gain_reference": {
            "gain_percent": 100,
            "coefficients": "unscaled; apply F1 gain exactly once outside these rounded mA parameters",
        },
        "signed_gravity_and_bias_motor_current_a": {
            "gravity_scale_a_per_nm":
                motor_sign * float(even_values["gravity_scale"]["value"]),
            "bias_a": motor_sign * float(even_values["bias"]["value"]),
        },
        "four_harmonics_mA": harmonics,
        "coulomb_mA": rounded_motor_parameter(coulomb_ma, "mA"),
        "viscous_mA_per_motor_rev_s":
            rounded_motor_parameter(viscous_motor, "mA/(motor rev/s)"),
        "speed_conversion": {
            "joint_deg_s_to_motor_rev_s": reduction / 360.0,
            "motor_rev_s_to_joint_deg_s": 360.0 / reduction,
            "reduction": reduction,
            "speed_input": "abs(motor rev/s)",
        },
        "signed_motor_phase_equation": (
            "i_motor = motor_sign*J*(gravity_scale*tau_hold+bias) "
            "+ (e_sin*sin(psi)+e_cos*cos(psi))/1000 "
            "+ dir_motor*(coulomb + viscous*abs(motor_rev_s) "
            "+ (o_sin*sin(psi)+o_cos*cos(psi))/1000) A"
        ),
        "deployment": {
            "harmonic_parameters_deployable": harmonic_values_ok,
            "viscous_parameter_deployable": speed_term_included and viscous_motor >= 0,
            "blockers": blockers,
            "candidate_enabled": False,
        },
    }
    if joint == 5:
        parameter["j5_sign_note"] = (
            "For motor_sign=-1, e_sin and o_cos keep their joint-fit signs; "
            "e_cos and o_sin reverse sign because psi=-phi.")
    return parameter, blockers


def evaluate_joint_phase_motor_current(candidate: dict[str, Any], q_deg: float,
                                       direction_joint: int, torque_nm: float,
                                       speed_deg_s: float) -> float:
    if direction_joint not in (-1, 1):
        raise ValueError("direction_joint must be -1 or +1")
    even = candidate["model"]["joint_positive"]["even_current_a"]
    odd = candidate["model"]["joint_positive"]["odd_current_a"]
    phase = math.radians(HARMONIC_ORDER * q_deg)
    sign = int(candidate["motor_phase"]["motor_sign_J"])
    even_current = (
        even["gravity_scale_a_per_nm"] * torque_nm
        + even["bias_a"]
        + even["harmonic_sin_a"] * math.sin(phase)
        + even["harmonic_cos_a"] * math.cos(phase)
    )
    odd_current = (
        odd["constant_a"]
        + odd["speed_coefficient_a_per_deg_s"] * abs(speed_deg_s)
        + odd["harmonic_sin_a"] * math.sin(phase)
        + odd["harmonic_cos_a"] * math.cos(phase)
    )
    return sign * (even_current + direction_joint * odd_current)


def evaluate_motor_phase_current(candidate: dict[str, Any], q_deg: float,
                                 direction_motor: int, torque_nm: float,
                                 motor_speed_abs_rev_s: float) -> float:
    if direction_motor not in (-1, 1):
        raise ValueError("direction_motor must be -1 or +1")
    motor_phase = candidate["motor_phase"]
    sign = int(motor_phase["motor_sign_J"])
    phase = sign * math.radians(HARMONIC_ORDER * q_deg)
    harmonics = motor_phase["four_harmonics_mA"]
    even_harmonic = (
        harmonics["e_sin"]["unscaled_value"] * math.sin(phase)
        + harmonics["e_cos"]["unscaled_value"] * math.cos(phase)
    ) / 1000.0
    odd_harmonic = (
        harmonics["o_sin"]["unscaled_value"] * math.sin(phase)
        + harmonics["o_cos"]["unscaled_value"] * math.cos(phase)
    ) / 1000.0
    gravity = motor_phase["signed_gravity_and_bias_motor_current_a"]
    coulomb = motor_phase["coulomb_mA"]["unscaled_value"] / 1000.0
    viscous = (
        motor_phase["viscous_mA_per_motor_rev_s"]["unscaled_value"]
        / 1000.0
    )
    return (
        gravity["gravity_scale_a_per_nm"] * torque_nm
        + gravity["bias_a"]
        + even_harmonic
        + direction_motor * (
            coulomb
            + viscous * abs(motor_speed_abs_rev_s)
            + odd_harmonic
        )
    )
def assert_fold_fit(observations: list[dict[str, Any]]
                    ) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    even_x, _ = even_design(observations)
    even_y = np.asarray([row["even_current_a"] for row in observations])
    speed_selection = assess_speed_term(observations)
    include_speed = bool(speed_selection["included"])
    odd_x, _ = odd_design(observations, include_speed)
    odd_y = np.asarray([row["odd_current_a"] for row in observations])
    return (
        weighted_linear_fit(observations, even_x, even_y),
        weighted_linear_fit(observations, odd_x, odd_y),
        speed_selection,
    )


def holdout_evaluation(observations: list[dict[str, Any]]
                       ) -> tuple[dict[str, Any], dict[str, Any]]:
    poses = sorted({row["pose_id"] for row in observations})
    runs = sorted({row["run_id"] for row in observations})
    pose_folds = []
    run_folds = []
    pose_errors = []
    run_errors = []

    for held_pose in poses:
        train = [row for row in observations if row["pose_id"] != held_pose]
        test = [row for row in observations if row["pose_id"] == held_pose]
        try:
            even_fit, odd_fit, speed_selection = assert_fold_fit(train)
            include_speed = bool(speed_selection["included"])
            even_x, _ = even_design(test)
            odd_x, _ = odd_design(test, include_speed)
            even_actual = np.asarray([row["even_current_a"] for row in test])
            odd_actual = np.asarray([row["odd_current_a"] for row in test])
            pose_folds.append({
                "held_pose_id": held_pose,
                "train_runs": len({row["run_id"] for row in train}),
                "test_runs": len({row["run_id"] for row in test}),
                "test_pairs": len(test),
                "speed_term_selected_on_training": speed_selection,
                "even": prediction_metrics(test, even_x @ even_fit["coefficients"], even_actual),
                "odd": prediction_metrics(test, odd_x @ odd_fit["coefficients"], odd_actual),
            })
        except Exception as error:
            pose_errors.append(f"held pose {held_pose}: {type(error).__name__}: {error}")

    for held_run in runs:
        train = [row for row in observations if row["run_id"] != held_run]
        test = [row for row in observations if row["run_id"] == held_run]
        try:
            even_fit, odd_fit, speed_selection = assert_fold_fit(train)
            include_speed = bool(speed_selection["included"])
            even_x, _ = even_design(test)
            odd_x, _ = odd_design(test, include_speed)
            even_actual = np.asarray([row["even_current_a"] for row in test])
            odd_actual = np.asarray([row["odd_current_a"] for row in test])
            run_folds.append({
                "held_run_id": held_run,
                "held_pose_id": test[0]["pose_id"],
                "test_pairs": len(test),
                "speed_term_selected_on_training": speed_selection,
                "even": prediction_metrics(test, even_x @ even_fit["coefficients"], even_actual),
                "odd": prediction_metrics(test, odd_x @ odd_fit["coefficients"], odd_actual),
            })
        except Exception as error:
            run_errors.append(f"held run {held_run}: {type(error).__name__}: {error}")

    def aggregate(folds, component):
        values = [fold[component]["run_macro_rmse_a"] for fold in folds]
        return {
            "fold_count": len(folds),
            "macro_rmse_a": float(np.mean(values)) if values else None,
            "median_fold_rmse_a": float(np.median(values)) if values else None,
            "worst_fold_rmse_a": float(np.max(values)) if values else None,
        }

    pose_result = {
        "folds": pose_folds,
        "even_summary": aggregate(pose_folds, "even"),
        "odd_summary": aggregate(pose_folds, "odd"),
        "errors": pose_errors,
    }
    run_result = {
        "folds": run_folds,
        "even_summary": aggregate(run_folds, "even"),
        "odd_summary": aggregate(run_folds, "odd"),
        "errors": run_errors,
    }
    return pose_result, run_result


def coverage_summary(observations: list[dict[str, Any]]) -> dict[str, Any]:
    poses: dict[str, dict[str, Any]] = {}
    for pose_id in sorted({row["pose_id"] for row in observations}):
        rows = [row for row in observations if row["pose_id"] == pose_id]
        speeds = sorted({round(row["commanded_speed_deg_s"], 6) for row in rows})
        q = np.asarray([row["q_deg"] for row in rows])
        torque = np.asarray([row["gravity_torque_nm"] for row in rows])
        poses[pose_id] = {
            "runs": len({row["run_id"] for row in rows}),
            "pairs": len(rows),
            "commanded_speeds_deg_s": speeds,
            "angle_range_deg": [float(q.min()), float(q.max())],
            "gravity_torque_range_nm": [float(torque.min()), float(torque.max())],
        }
    q = np.asarray([row["q_deg"] for row in observations])
    all_q = np.asarray([row["q_all_deg"] for row in observations])
    torque = np.asarray([row["gravity_torque_nm"] for row in observations])
    speed = np.asarray([row["speed_magnitude_deg_s"] for row in observations])
    excess = np.maximum(0.0, np.maximum(
        WRIST_STRICT_LIMITS_DEG[:, 0] - all_q.min(axis=0),
        all_q.max(axis=0) - WRIST_STRICT_LIMITS_DEG[:, 1],
    ))
    return {
        "poses": poses,
        "pose_count": len(poses),
        "run_count": len({row["run_id"] for row in observations}),
        "paired_bins": len(observations),
        "angle_range_deg": [float(q.min()), float(q.max())],
        "gravity_torque_range_nm": [float(torque.min()), float(torque.max())],
        "speed_magnitude_range_deg_s": [float(speed.min()), float(speed.max())],
        "all_joint_ranges_deg": [
            [float(low), float(high)] for low, high in zip(all_q.min(axis=0), all_q.max(axis=0))
        ],
        "legacy_joint_limits_deg": JOINT_LIMITS_DEG.tolist(),
        "wrist_historical_limits_deg": WRIST_HISTORICAL_LIMITS_DEG.tolist(),
        "wrist_strict_limits_deg": WRIST_STRICT_LIMITS_DEG.tolist(),
        "offline_limit_tolerance_deg": OFFLINE_JOINT_LIMIT_TOLERANCE_DEG,
        "maximum_nominal_limit_excess_deg": round(float(np.max(excess)), 6),
    }


def common_limitations(firmware: str, speed_assessment: dict[str, Any],
                       time_source: str = "host"
                       ) -> list[str]:
    limitations = [
        "The observed signal is filtered command current, not calibrated motor torque or measured torque.",
        "E still includes direction-even transmission, cable, sensor and model-error terms; k is a model-current scale, not a mass estimate.",
        "O is the forward-minus-reverse half difference. The speed term is a local magnitude approximation and does not identify Stribeck hysteresis.",
        "The MJCF masses, centers of mass, closed-gripper geometry and robot/model mappings require hardware validation.",
        "The joint-angle harmonic phase and F1 encoder-home motor phase differ by the motor sign and a phase offset that is not independently identified here.",
        "No range extension is allowed outside the exported angle, gravity-torque, speed and pose domains.",
        "The model does not validate F1 feedforward direction, current limiting, position tracking, vibration or hand feel.",
        "Payload grouping removes only pose annotations such as J4=<angle>; exact source descriptions remain in the input manifest.",
    ]
    if time_source == "f4-position":
        limitations.append(
            "Motion filtering uses unwrapped F4 position ticks, so host receive latency is excluded. "
            "Missing or asynchronous F4 feedback remains unsampled dynamics.")
    else:
        limitations.append(
            "Motion filtering uses host receive time. Host delays remain in the timeline and "
            "make_pairs can remove gross gaps but cannot reconstruct unsampled dynamics.")
    if not speed_assessment["included"]:
        limitations.append(
            "The odd-friction speed term is omitted because it failed the exported identifiability test: "
            + str(speed_assessment["reason"]))
    if firmware == "current":
        limitations.append(
            "The firmware group is labelled only 'current'; exact F4/F1 build identifiers were not recorded.")
    return limitations


def fit_group(joint: int, firmware: str, payload: str, runs: list[dict[str, Any]],
              minimum_poses: int,
              gripper_rad: float = 0.0) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    run_grippers = [float(run.get("gripper_rad", gripper_rad)) for run in runs]
    if any(not math.isclose(value, gripper_rad, rel_tol=0.0, abs_tol=1e-12)
           for value in run_grippers):
        raise ValueError(
            f"J{joint}: runs with different calibrated gripper_rad values cannot share a model")
    time_sources = {
        str(run.get("info", {}).get("time_source", "host"))
        for run in runs
    }
    if len(time_sources) > 1:
        raise ValueError(
            f"J{joint}: runs with different time sources cannot share a model: "
            f"{sorted(time_sources)}")
    time_source = next(iter(time_sources), "host")
    training_runs, retest_runs = partition_training_and_retest_runs(runs)
    observations = [row for run in training_runs for row in run["observations"]]
    retest_observations = [row for run in retest_runs for row in run["observations"]]
    poses = sorted({row["pose_id"] for row in observations})
    group_id = f"J{joint}_{slug(firmware, 20)}_{slug(payload, 64)}"
    if not math.isclose(gripper_rad, 0.0):
        group_id += f"_g{gripper_rad:.3f}".replace(".", "p")
    if "|hardware:" in payload or "@sha256:" in firmware:
        identity_digest = hashlib.sha256(
            f"{joint}\0{firmware}\0{payload}\0{gripper_rad:.12g}".encode("utf-8")
        ).hexdigest()[:12]
        group_id += f"_{identity_digest}"
    summary = {
        "group_id": group_id,
        "joint": joint,
        "time_source": time_source,
        "firmware_revision": firmware,
        "payload_group": payload,
        "payload_descriptions": sorted({
            str(run["config"].get("payload_description", "")) for run in runs
        }),
        "firmware_hash": sorted({
            str(run["config"]["firmware_hash"])
            for run in runs if run["config"].get("firmware_hash")
        }),
        "payload_identity": sorted(
            json.dumps(run["config"]["payload_identity"], sort_keys=True,
                       ensure_ascii=False, separators=(",", ":"))
            for run in runs if isinstance(run["config"].get("payload_identity"), dict)
        ),
        "gripper_rad": float(gripper_rad),
        "poses": poses,
        "runs": len(training_runs),
        "pairs": len(observations),
        "independent_retest_runs": len(retest_runs),
        "independent_retest_pairs": len(retest_observations),
        "candidate": False,
        "reason": None,
    }
    if not training_runs:
        summary["reason"] = "only labelled independent_retest runs were found"
        return None, summary
    if len(poses) < minimum_poses:
        summary["reason"] = (
            f"requires at least {minimum_poses} whole poses for leave-pose-out; found {len(poses)}")
        return None, summary

    speed_assessment = assess_speed_term(observations)
    include_speed = bool(speed_assessment["included"])
    try:
        even_x, even_names = even_design(observations)
        odd_x, odd_names = odd_design(observations, include_speed)
        even_result = evaluate_design(
            observations, even_x, np.asarray([row["even_current_a"] for row in observations]))
        odd_result = evaluate_design(
            observations, odd_x, np.asarray([row["odd_current_a"] for row in observations]))
        pose_holdout, run_holdout = holdout_evaluation(observations)
    except Exception as error:
        summary["reason"] = f"full fit/holdout failed: {type(error).__name__}: {error}"
        return None, summary
    holdout_errors = pose_holdout["errors"] + run_holdout["errors"]
    if holdout_errors:
        summary["reason"] = "one or more holdouts failed: " + "; ".join(holdout_errors[:3])
        return None, summary

    sign = int(MOTOR_SIGNS[joint - 1])
    even_records = coefficient_records(even_names, even_result["fit"], sign)
    odd_records = coefficient_records(odd_names, odd_result["fit"], sign)
    even_values = {record["name"]: record for record in even_records}
    odd_values = {record["name"]: record for record in odd_records}
    odd_speed = float(odd_values.get("odd_speed", {}).get("value", 0.0))
    motor_phase, deployment_blockers = motor_phase_parameters(
        joint, sign, even_values, odd_values, speed_assessment)
    source_files = source_file_records(training_runs, gripper_rad)
    coverage = coverage_summary(observations)
    coverage["input_current_limits_a"] = sorted(
        {float(run["config"]["current_limit_a"]) for run in runs})
    limitations = common_limitations(firmware, speed_assessment, time_source)
    if coverage["maximum_nominal_limit_excess_deg"] > 0:
        limitations.append(
            "Offline data includes a measured pose up to "
            f"{coverage['maximum_nominal_limit_excess_deg']:.3f} deg outside a nominal limit; "
            f"the exported limit tolerance is {OFFLINE_JOINT_LIMIT_TOLERANCE_DEG:.2f} deg. "
            "Hardware entry still rejects this excess.")
    candidate = {
        "schema_version": 1,
        "model_type": "wrist_holding_current_harmonic_odd_friction",
        "candidate_id": group_id,
        "joint": joint,
        "time_source": time_source,
        "enabled": False,
        "deployment_review_required": True,
        "firmware_revision": firmware,
        "payload_group": payload,
        "payload_descriptions": sorted({
            str(run["config"].get("payload_description", "")) for run in runs
        }),
        "firmware_hash": sorted({
            str(run["config"]["firmware_hash"])
            for run in runs if run["config"].get("firmware_hash")
        }),
        "payload_identity": sorted(
            json.dumps(run["config"]["payload_identity"], sort_keys=True,
                       ensure_ascii=False, separators=(",", ":"))
            for run in runs if isinstance(run["config"].get("payload_identity"), dict)
        ),
        "gravity_torque_source": {
            "mjcf": str(DEFAULT_MJCF.resolve()),
            "mapping": str(DEFAULT_MAPPING.resolve()),
            "gripper_rad": float(gripper_rad),
            "joint_velocity_rad_s": 0.0,
            "expression": "robot-joint-positive qfrc_bias with qvel=0",
        },
        "angle_reference": {
            "source": "F4 #GETJPOS q_deg; initPose is already included",
            "joint_positive_direction": "A positive joint command is increasing q_deg",
            "initial_pose_offset_added_deg": 0.0,
            "harmonic_order": HARMONIC_ORDER,
            "harmonic_phase_expression": "phi = 30 * q_robot_rad",
            "motor_phase_expression":
                "psi_motor = motor_sign_J * phi; F1 encoder-home offset is not identified by this joint fit",
        },
        "direction_and_units": {
            "E_a": "even_current_a = (i_forward + i_reverse) / 2 in joint-positive A",
            "O_a": "odd_current_a = (i_forward - i_reverse) / 2 in joint-positive A",
            "current_equation": "i_joint_positive = E + direction * O",
            "direction_values": {"joint_positive_motion": 1, "joint_negative_motion": -1},
            "gravity_torque_unit": "N*m",
            "joint_positive_to_motor_sign": sign,
            "joint_phase_motor_equation":
                "i_motor_joint_phase = sign_J * (E + dir_joint * O)",
            "f1_motor_phase_equation":
                "i_motor = sign_J*(k*tau+bias) + E_harm(psi) "
                "+ dir_motor*(C+V*abs(motor_rev_s)+O_harm(psi))",
            "do_not_deploy_model_motor_joint_phase_coefficients_directly":
                "Use motor_phase.four_harmonics_mA; reversing phase changes only cosine-even and sine-odd terms",
        },
        "model": {
            "joint_positive": {
                "even_current_a": {
                    "gravity_scale_a_per_nm": even_values["gravity_scale"]["value"],
                    "bias_a": even_values["bias"]["value"],
                    "harmonic_sin_a": even_values["harmonic_sin"]["value"],
                    "harmonic_cos_a": even_values["harmonic_cos"]["value"],
                },
                "odd_current_a": {
                    "constant_a": odd_values["odd_constant"]["value"],
                    "speed_coefficient_a_per_deg_s": odd_speed,
                    "speed_input": "speed_magnitude_deg_s",
                    "harmonic_sin_a": odd_values["harmonic_sin"]["value"],
                    "harmonic_cos_a": odd_values["harmonic_cos"]["value"],
                },
            },
            "motor": {
                "phase_reference": "joint angle phi, not F1 motor encoder angle psi",
                "motion_direction": "dir_joint, not dir_motor",
                "even_current_a": {
                    "gravity_scale_a_per_nm":
                        float(sign * even_values["gravity_scale"]["value"]),
                    "bias_a": float(sign * even_values["bias"]["value"]),
                    "harmonic_sin_a": float(sign * even_values["harmonic_sin"]["value"]),
                    "harmonic_cos_a": float(sign * even_values["harmonic_cos"]["value"]),
                },
                "odd_current_a": {
                    "constant_a": float(sign * odd_values["odd_constant"]["value"]),
                    "speed_coefficient_a_per_deg_s": float(sign * odd_speed),
                    "speed_input": "speed_magnitude_deg_s",
                    "harmonic_sin_a": float(sign * odd_values["harmonic_sin"]["value"]),
                    "harmonic_cos_a": float(sign * odd_values["harmonic_cos"]["value"]),
                },
            },
            "motor_phase": motor_phase,
        },
        "motor_phase": motor_phase,
        "deployment_blockers": deployment_blockers,
        "coefficients": {
            "even_joint_positive": even_records,
            "odd_joint_positive": odd_records,
            "confidence_level": 0.95,
            "standard_error_method": "weighted least squares covariance; equal total weight per run",
        },
        "speed_term_identifiability": speed_assessment,
        "training_metrics": {
            "even": even_result["metrics"],
            "odd": odd_result["metrics"],
            "even_condition_number": even_result["fit"]["condition_number"],
            "odd_condition_number": odd_result["fit"]["condition_number"],
            "even_r_squared": even_result["fit"]["r_squared"],
            "odd_r_squared": odd_result["fit"]["r_squared"],
        },
        "holdout": {
            "whole_pose": pose_holdout,
            "independent_run": run_holdout,
        },
        "coverage": coverage,
        "limitations": limitations,
        "source_files": source_files,
    }
    retest_evaluation = evaluate_frozen_retest(candidate, retest_observations)
    retest_evaluation["source_files"] = source_file_records(retest_runs, gripper_rad)
    candidate["independent_retest"] = retest_evaluation
    summary.update({
        "candidate": True,
        "reason": "fitted with complete leave-pose-out and leave-run-out evidence",
        "output": f"{group_id}.model.json",
        "even_rmse_a": even_result["metrics"]["run_macro_rmse_a"],
        "odd_rmse_a": odd_result["metrics"]["run_macro_rmse_a"],
        "gravity_scale_a_per_nm": even_values["gravity_scale"]["value"],
        "gravity_scale_ci95_a_per_nm": even_values["gravity_scale"]["ci95"],
        "speed_term_included": include_speed,
        "leave_pose_even_rmse_a": pose_holdout["even_summary"]["macro_rmse_a"],
        "leave_pose_odd_rmse_a": pose_holdout["odd_summary"]["macro_rmse_a"],
        "leave_run_even_rmse_a": run_holdout["even_summary"]["macro_rmse_a"],
        "leave_run_odd_rmse_a": run_holdout["odd_summary"]["macro_rmse_a"],
        "independent_retest": retest_evaluation,
        "observations": observations,
    })
    return candidate, summary


def pair_csv(candidate: dict[str, Any], observations: list[dict[str, Any]]) -> str:
    even_model = candidate["model"]["joint_positive"]["even_current_a"]
    odd_model = candidate["model"]["joint_positive"]["odd_current_a"]
    buffer = io.StringIO()
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerow(
        "source,pose_id,commanded_speed_deg_s,q_deg,gravity_torque_nm,"
        "even_actual_a,even_predicted_a,odd_actual_a,odd_predicted_a,"
        "speed_magnitude_deg_s".split(","))
    for row in observations:
        phase = math.radians(HARMONIC_ORDER * row["q_deg"])
        even_prediction = (
            even_model["gravity_scale_a_per_nm"] * row["gravity_torque_nm"]
            + even_model["bias_a"]
            + even_model["harmonic_sin_a"] * math.sin(phase)
            + even_model["harmonic_cos_a"] * math.cos(phase)
        )
        odd_prediction = (
            odd_model["constant_a"]
            + odd_model["speed_coefficient_a_per_deg_s"] * row["speed_magnitude_deg_s"]
            + odd_model["harmonic_sin_a"] * math.sin(phase)
            + odd_model["harmonic_cos_a"] * math.cos(phase)
        )
        writer.writerow([
            row["source"],
            row["pose_id"],
            f"{row['commanded_speed_deg_s']:.9g}",
            f"{row['q_deg']:.12g}",
            f"{row['gravity_torque_nm']:.12g}",
            f"{row['even_current_a']:.12g}",
            f"{even_prediction:.12g}",
            f"{row['odd_current_a']:.12g}",
            f"{odd_prediction:.12g}",
            f"{row['speed_magnitude_deg_s']:.12g}",
        ])
    return buffer.getvalue()


def input_evidence(runs: list[dict[str, Any]],
                   time_source: str | None = None) -> dict[str, Any]:
    files = []
    for run in runs:
        files.append({
            "csv": run["source"],
            "metadata": run["metadata"],
            "csv_sha256": run["csv_sha256"],
            "metadata_sha256": run["metadata_sha256"],
            "joint": int(run["config"]["joint"]),
            "pose_id": run["info"]["pose_id"],
            "firmware_revision": str(run["config"].get("firmware_revision", "")),
            "payload_description": str(run["config"].get("payload_description", "")),
            "gripper_rad": float(run["gripper_rad"]),
            "speed_deg_s": float(run["config"]["speed_deg_s"]),
            "measurement_round": run["config"].get("measurement_round"),
            "current_limit_a": float(run["config"]["current_limit_a"]),
            "raw_samples": int(run["info"]["raw_samples"]),
            "retained_samples": int(run["info"]["retained_samples"]),
            "time_source": str(run["info"].get("time_source", "host")),
            "position_tick_wrap_count":
                int(run["info"].get("position_tick_wrap_count", 0)),
            "paired_bins": int(len(run["pairs"])),
            "completed": True,
            "stop_acknowledged": True,
        })
    actual_sources = {record["time_source"] for record in files}
    if time_source is None:
        if len(actual_sources) > 1:
            raise ValueError(f"Mixed time sources in input evidence: {sorted(actual_sources)}")
        time_source = next(iter(actual_sources), "host")
    if actual_sources and actual_sources != {time_source}:
        raise ValueError(
            f"Requested time source {time_source!r} does not match inputs: "
            f"{sorted(actual_sources)}")
    return {"schema_version": 1, "time_source": time_source, "files": files}


def implementation_hashes() -> dict[str, str]:
    return {
        name: sha256_file(HERE / name)
        for name in ("calibration.py", "fit_wrist.py", "gravity_preview.py")
    }


def write_managed_outputs(output_dir: Path, contents: dict[str, str]) -> None:
    manifest_path = output_dir / OUTPUT_MANIFEST
    expected = set(contents)
    managed = set()
    if output_dir.exists():
        existing = {path.name for path in output_dir.iterdir()}
        if manifest_path.exists():
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            if manifest.get("managed_by") != MANAGED_BY:
                raise ValueError(f"Refusing to overwrite unmanaged output directory: {output_dir}")
            managed = set(manifest.get("files", []))
            unexpected = existing - managed - {OUTPUT_MANIFEST}
            if unexpected:
                raise ValueError(
                    f"Refusing to ignore unmanaged files in {output_dir}: {sorted(unexpected)}")
        elif existing:
            raise ValueError(
                f"Refusing to overwrite non-managed files in {output_dir}: {sorted(existing)}")
    else:
        output_dir.mkdir(parents=True)

    for name, text in sorted(contents.items()):
        if Path(name).name != name:
            raise ValueError("Output names must not contain directories")
        file_descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{name}.", dir=output_dir, text=True)
        try:
            with os.fdopen(file_descriptor, "w", encoding="utf-8") as file:
                file.write(text)
                file.flush()
                os.fsync(file.fileno())
            os.replace(temporary_name, output_dir / name)
        finally:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
    retained_managed = {
        name for name in managed
        if Path(name).name == name and (output_dir / name).is_file()
    }
    manifest = {
        "schema_version": 1,
        "managed_by": MANAGED_BY,
        "files": sorted(expected | retained_managed),
    }
    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{OUTPUT_MANIFEST}.", dir=output_dir, text=True)
    try:
        with os.fdopen(file_descriptor, "w", encoding="utf-8") as file:
            json.dump(manifest, file, indent=2, ensure_ascii=False)
            file.write("\n")
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary_name, manifest_path)
    finally:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, default=HERE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--mapping", type=Path, default=DEFAULT_MAPPING)
    parser.add_argument("--mjcf", type=Path, default=DEFAULT_MJCF)
    parser.add_argument("--joints", type=int, nargs="+", choices=(4, 5), default=(4, 5))
    parser.add_argument("--minimum-poses", type=int, default=3)
    parser.add_argument("--bin-deg", type=float, default=1.0)
    parser.add_argument("--max-accel-deg-s2", type=float, default=2.0)
    parser.add_argument(
        "--time-source",
        choices=("host", "f4-position"),
        default="host",
        help="time axis for motion filtering (default: host)",
    )
    parser.add_argument("--dry-run", action="store_true",
                        help="Fit and print the summary without writing candidate files")
    args = parser.parse_args()
    if args.minimum_poses < 3:
        parser.error("--minimum-poses must be at least 3 for whole-pose holdout")
    data_root = args.data_root.resolve()
    if not data_root.is_dir():
        parser.error(f"--data-root is not a directory: {data_root}")
    joints = set(args.joints)
    evaluator = GravityTorqueEvaluator(args.mapping, args.mjcf, gripper_rad=0.0)
    runs, exclusions, failures = discover_runs(
        data_root, joints, evaluator, args.bin_deg, args.max_accel_deg_s2,
        time_source=args.time_source)
    groups = group_runs(runs, joints)
    measurement_round_counts = Counter(
        str(run["config"].get("measurement_round") or "unlabelled") for run in runs)

    candidates = {}
    group_summaries = []
    for (joint, firmware, payload, gripper_rad), group in groups.items():
        candidate, summary = fit_group(
            joint, firmware, payload, group, args.minimum_poses,
            gripper_rad=gripper_rad)
        group_summaries.append({key: value for key, value in summary.items()
                                if key != "observations"})
        if candidate is not None:
            if summary["group_id"] in candidates:
                raise ValueError(f"Candidate group id collision: {summary['group_id']}")
            candidates[summary["group_id"]] = (candidate, summary["observations"])

    summary = {
        "schema_version": 1,
        "tool": MANAGED_BY,
        "mode": "offline_only",
        "time_source": args.time_source,
        "hardware_connection": None,
        "qualified_metadata": {
            "completed": True,
            "stop_acknowledged": True,
            "joints": sorted(joints),
        },
        "inputs": {
            "data_root": data_root.relative_to(HERE).as_posix()
            if data_root.is_relative_to(HERE) else str(data_root),
            "mapping": str(args.mapping.resolve()),
            "mjcf": str(args.mjcf.resolve()),
            "time_source": args.time_source,
            "implementation_sha256": implementation_hashes(),
            "scan_counts": exclusions,
            "measurement_round_counts": dict(sorted(measurement_round_counts.items())),
            "pairing_failures": failures,
        },
        "joint_limits_deg": WRIST_STRICT_LIMITS_DEG.tolist(),
        "legacy_joint_limits_deg": JOINT_LIMITS_DEG.tolist(),
        "wrist_historical_limits_deg": WRIST_HISTORICAL_LIMITS_DEG.tolist(),
        "groups": group_summaries,
        "candidate_count": len(candidates),
    }
    print(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False))
    if args.dry_run:
        return

    contents = {
        "summary.json": json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        "inputs.sha256.json":
            json.dumps(input_evidence(runs, args.time_source),
                       indent=2, ensure_ascii=False) + "\n",
    }
    for group_id, (candidate, observations) in candidates.items():
        contents[f"{group_id}.model.json"] = (
            json.dumps(candidate, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
        contents[f"{group_id}.pairs.csv"] = pair_csv(candidate, observations)
    write_managed_outputs(args.output_dir, contents)
    print(f"Wrote {len(candidates)} offline candidate models to {args.output_dir}")


if __name__ == "__main__":
    main()
