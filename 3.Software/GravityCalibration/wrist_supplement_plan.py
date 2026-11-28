"""Generate the six-route J4/J5 supplement plan without hardware access.

The default route definition is built into this file. Every invocation checks
those routes against the current MJCF before writing configs. A historical
route-verification JSON can be supplied only for an explicit evidence check.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
from typing import Any

from collect import validate_config
from pose_sweep_planner import (COLLISION_TOLERANCE_M, DEFAULT_MAPPING,
                                DEFAULT_MJCF, PoseEvaluator,
                                REFERENCE_FEASIBLE_POSE_DEG,
                                transition_waypoints, verify_path)

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DEFAULT_OUTPUT = HERE / "results" / "pose_configs_j45_supplement_20261127"
SWEEP_LOW_DEG = -40.0
SWEEP_HIGH_DEG = 40.0
EXPECTED_ROUTES = (
    (4, -60.0),
    (4, 0.0),
    (4, 60.0),
    (5, -120.0),
    (5, 0.0),
    (5, 120.0),
)
PRIMARY_SPEEDS_DEG_S = (0.5, 1.0, 2.0)
RETEST_SPEEDS_DEG_S = (1.0,)
DEFAULT_CYCLES = 3
DEFAULT_SAMPLE_HZ = 50
FIRMWARE_REVISION_PLACEHOLDER = "REPLACE_WITH_F1_F4_BUILD_REVISION"
FIRMWARE_HASH_PLACEHOLDER = "REPLACE_WITH_SHA256_OF_ACTUAL_F1_F4_BUILD"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def project_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(ROOT).as_posix()
    except ValueError:
        return path.name


def expected_fixed_pose(active_joint: int, fixed_wrist_deg: float) -> list[float]:
    if active_joint not in (4, 5):
        raise ValueError(f"Supplement routes require J4 or J5, found J{active_joint}")
    fixed_pose = [float(value) for value in REFERENCE_FEASIBLE_POSE_DEG]
    paired_joint = 5 if active_joint == 4 else 4
    fixed_pose[paired_joint - 1] = float(fixed_wrist_deg)
    fixed_pose[active_joint - 1] = 0.0
    return fixed_pose


def scan_waypoints(fixed_pose: list[float], active: int, cycles: int) -> list[list[float]]:
    waypoints = []
    current = fixed_pose.copy()
    for _cycle in range(cycles):
        current[active] = SWEEP_LOW_DEG
        waypoints.append(current.copy())
        current[active] = SWEEP_HIGH_DEG
        waypoints.append(current.copy())
        current[active] = SWEEP_LOW_DEG
        waypoints.append(current.copy())
    return waypoints


def build_firmware_metadata(f1_hex: Path | None, f4_hex: Path | None
                            ) -> tuple[dict[str, Any], dict[str, Any]]:
    if (f1_hex is None) != (f4_hex is None):
        raise ValueError("Supply both F1 and F4 HEX files together")
    if f1_hex is None:
        config_fields = {
            "firmware_revision": FIRMWARE_REVISION_PLACEHOLDER,
            "firmware_hash": FIRMWARE_HASH_PLACEHOLDER,
            "firmware_artifacts": {},
            "firmware_pair_sha256": None,
        }
        summary_fields = {
            "firmware_artifacts": {},
            "firmware_pair_sha256": None,
        }
        return config_fields, summary_fields

    for path in (f1_hex, f4_hex):
        if not path.is_file():
            raise ValueError(f"Firmware HEX file does not exist: {path}")
    f1_sha256 = sha256_file(f1_hex)
    f4_sha256 = sha256_file(f4_hex)
    pair_payload = json.dumps(
        {"F1_HEX": f1_sha256, "F4_HEX": f4_sha256},
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    pair_sha256 = hashlib.sha256(pair_payload).hexdigest()
    artifacts = {
        "F1_HEX": {"filename": f1_hex.name, "sha256": f1_sha256},
        "F4_HEX": {"filename": f4_hex.name, "sha256": f4_sha256},
    }
    config_fields = {
        "firmware_revision": FIRMWARE_REVISION_PLACEHOLDER,
        "firmware_hash": pair_sha256,
        "firmware_artifacts": artifacts,
        "firmware_pair_sha256": pair_sha256,
    }
    summary_fields = {
        "firmware_artifacts": artifacts,
        "firmware_pair_sha256": pair_sha256,
        "firmware_pair_algorithm":
            "SHA-256 of canonical JSON {F1_HEX:<sha256>,F4_HEX:<sha256>}",
    }
    return config_fields, summary_fields


def evidence_fixed_pose(route: dict[str, Any]) -> list[float]:
    stops = route.get("stops")
    if not isinstance(stops, list) or len(stops) < 3:
        raise ValueError(f"Evidence route {route_key(route)} has no complete stop list")
    fixed_pose = [float(value) for value in stops[1]]
    if len(fixed_pose) != 6:
        raise ValueError(f"Evidence route {route_key(route)} has no six-axis fixed pose")
    if [float(value) for value in stops[0]] != REFERENCE_FEASIBLE_POSE_DEG:
        raise ValueError(f"Evidence route {route_key(route)} has a different shared start")
    if [float(value) for value in stops[-1]] != REFERENCE_FEASIBLE_POSE_DEG:
        raise ValueError(f"Evidence route {route_key(route)} does not return to the shared start")
    sweep = [float(value) for value in route.get("sweep_deg", [])]
    if sweep != [SWEEP_LOW_DEG, SWEEP_HIGH_DEG]:
        raise ValueError(
            f"Evidence route {route_key(route)} sweep is {sweep}, expected "
            f"{[SWEEP_LOW_DEG, SWEEP_HIGH_DEG]}")
    return fixed_pose


def route_key(route: dict[str, Any]) -> tuple[int, float]:
    return int(route["active_joint"]), float(route["fixed_wrist_deg"])


def verified_model_files(evidence: dict[str, Any]) -> dict[Path, str]:
    by_suffix = {
        "DummyMujoco/dummy_description/mjcf/scene.xml": DEFAULT_MJCF,
        "DummyMujoco/dummy_description/mjcf/dummy.xml": DEFAULT_MJCF.with_name("dummy.xml"),
        "DummyWeb/config.json": DEFAULT_MAPPING,
    }
    resolved = {}
    for evidence_name, expected_hash in evidence.get("model_sha256", {}).items():
        normalized = str(evidence_name).replace("\\", "/")
        for suffix, path in by_suffix.items():
            if normalized.endswith(suffix):
                resolved[path] = str(expected_hash)
                break
    if set(resolved) != set(by_suffix.values()):
        raise ValueError("Route evidence does not identify all three current model files")
    return resolved


def compare_verified_routes(path: Path, evaluator: PoseEvaluator) -> dict[str, Any]:
    evidence = json.loads(path.read_text(encoding="utf-8"))
    if evidence.get("offline") is not True or evidence.get("hardware_commands") != 0:
        raise ValueError("Route evidence must be offline and contain zero hardware commands")
    if evidence.get("reference_start_deg") != REFERENCE_FEASIBLE_POSE_DEG:
        raise ValueError("Route evidence uses a different shared start pose")
    if not math.isclose(float(evidence.get("contact_tol_m", -1.0)),
                        COLLISION_TOLERANCE_M, rel_tol=0.0, abs_tol=1e-15):
        raise ValueError("Route evidence uses a different collision tolerance")
    if 0.0 not in [float(value) for value in evidence.get("gripper_sampled_rad", [])]:
        raise ValueError("Route evidence did not verify the closed gripper")
    if evidence.get("operator_reported_payload") != {
            "gripper": "closed",
            "gripped_object": None,
            "camera": "mounted",
            "camera_data_cable": "connected",
            "new_measurements": False,
    }:
        raise ValueError("Route evidence payload does not match the supplement setup")

    model_hashes = verified_model_files(evidence)
    for model_path, expected_hash in model_hashes.items():
        if sha256_file(model_path) != expected_hash:
            raise ValueError(f"Current model file differs from route evidence: {model_path}")

    routes = {route_key(route): route for route in evidence.get("roundtrips", [])}
    if set(routes) != set(EXPECTED_ROUTES):
        raise ValueError(
            f"Route evidence keys differ: expected {EXPECTED_ROUTES}, found {sorted(routes)}")
    checks = []
    for active_joint, fixed_wrist in EXPECTED_ROUTES:
        route = routes[(active_joint, fixed_wrist)]
        fixed_pose = evidence_fixed_pose(route)
        expected_pose = expected_fixed_pose(active_joint, fixed_wrist)
        if any(not math.isclose(actual, expected, abs_tol=1e-9)
               for actual, expected in zip(fixed_pose, expected_pose)):
            raise ValueError(
                f"Evidence route {active_joint, fixed_wrist} differs from EXPECTED_ROUTES")
        checks.append({
            "active_joint": active_joint,
            "fixed_wrist_deg": fixed_wrist,
            "current_mjcf_recheck": verify_path(
                evaluator,
                REFERENCE_FEASIBLE_POSE_DEG,
                [[float(value) for value in stop] for stop in route["stops"][1:]],
                f"evidence-route:J{active_joint}:fixed-{fixed_wrist:g}",
            ),
        })
    return {
        "file": path.name,
        "sha256": sha256_file(path),
        "route_checks": checks,
    }


def build_configs(firmware_fields: dict[str, Any], current_limit_a: float,
                  cycles: int, sample_hz: int,
                  evidence_reference: dict[str, str] | None
                  ) -> list[tuple[str, dict[str, Any]]]:
    payload_description = (
        "gripper closed; camera mounted; camera data cable connected; "
        "no grasped object"
    )
    payload_identity = {
        "gripper": "closed",
        "gripped_object": None,
        "camera": "mounted",
        "camera_data_cable": "connected",
        "tool_mount": "unchanged",
    }
    planned = []
    sequence = 1
    rounds = (
        ("primary", PRIMARY_SPEEDS_DEG_S),
        ("independent_retest", RETEST_SPEEDS_DEG_S),
    )
    for round_name, speeds in rounds:
        for route_index, (active_joint, fixed_wrist) in enumerate(
                EXPECTED_ROUTES, start=1):
            fixed_pose = expected_fixed_pose(active_joint, fixed_wrist)
            pose_token = (
                "0" if fixed_wrist == 0
                else ("m" if fixed_wrist < 0 else "p") + f"{abs(fixed_wrist):g}"
            )
            pose_id = f"J{active_joint}_J{5 if active_joint == 4 else 4}{pose_token}"
            for speed_index, speed in enumerate(speeds):
                speed_token = f"{speed:g}".replace(".", "p")
                filename = (
                    f"j{active_joint}_fj{5 if active_joint == 4 else 4}{pose_token}"
                    f"_v{speed_token}_{round_name}.json"
                )
                config = {
                    "pose_id": pose_id,
                    "joint": active_joint,
                    "fixed_pose_deg": fixed_pose,
                    "sweep_min_deg": SWEEP_LOW_DEG,
                    "sweep_max_deg": SWEEP_HIGH_DEG,
                    "speed_deg_s": float(speed),
                    "cycles": cycles,
                    "sample_hz": sample_hz,
                    "endpoint_hold_s": 1.0,
                    "full_pose_poll_hz": 2.0,
                    "current_limit_a": current_limit_a,
                    "current_limit_source":
                        "Operator must confirm this against the active F1 current limit",
                    "payload_description": payload_description,
                    "payload_identity": payload_identity,
                    "gripper_rad": 0.0,
                    "safety_profile": "wrist_strict_v1",
                    "temperature_note": "Record measured ambient and motor temperatures",
                    **firmware_fields,
                    "operator_reviewed_workspace": False,
                    "source_pose_deg": REFERENCE_FEASIBLE_POSE_DEG,
                    "route_source": "EXPECTED_ROUTES",
                    "source_route_index": route_index,
                    "measurement_round": round_name,
                    "batch_sequence_index": sequence,
                    "speed_index": speed_index,
                    "mjcf_checked": True,
                }
                if evidence_reference is not None:
                    config["verified_route_evidence"] = evidence_reference
                validate_config(config, execute=False)
                planned.append((filename, config))
                sequence += 1
    return planned


def execution_sequence_preflight(planned: list[tuple[str, dict[str, Any]]],
                                 evaluator: PoseEvaluator
                                 ) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Mirror batch order and verify each distinct transition and scan once."""
    current = list(REFERENCE_FEASIBLE_POSE_DEG)
    transition_cache = {}
    scan_cache = {}
    checks = {}
    sequence = []
    for filename, config in planned:
        target = [float(value) for value in config["fixed_pose_deg"]]
        transition_key = (tuple(current), tuple(target))
        if transition_key not in transition_cache:
            label = f"transition-{len(transition_cache) + 1}"
            checks[label] = verify_path(
                evaluator,
                current,
                transition_waypoints(current, target),
                label,
            )
            transition_cache[transition_key] = label

        active = int(config["joint"]) - 1
        scan_key = (tuple(target), active, int(config["cycles"]))
        if scan_key not in scan_cache:
            label = f"scan-{len(scan_cache) + 1}"
            waypoints = scan_waypoints(target, active, int(config["cycles"]))
            checks[label] = verify_path(evaluator, target, waypoints, label)
            scan_cache[scan_key] = (label, waypoints[-1])

        scan_label, end_pose = scan_cache[scan_key]
        sequence.append({
            "config": filename,
            "transition_check": transition_cache[transition_key],
            "scan_check": scan_label,
        })
        current = list(end_pose)
    return sequence, checks


def process_markdown() -> str:
    return f"""# J4/J5 verified supplement acquisition

This directory was generated from the built-in `EXPECTED_ROUTES`, the shared
start pose and a fresh offline check of the current MJCF. It contains 18
primary configs at 0.5, 1 and 2 deg/s and six independent retest configs at
1 deg/s.

Before hardware execution:

1. Confirm gripper closed, no object, camera mounted and camera data cable connected.
2. Confirm the shared arm pose is `[-90, -16, 110, 0, 0, 0]` deg and the
   workspace has an operator-approved clear route and independent emergency stop.
3. Read the active F1 current limit and replace `current_limit_a` and
   `current_limit_source` in every config.
4. Confirm `firmware_revision`. If the configs are still pending, generate a new
   directory with both actual HEX files:
   `python wrist_supplement_plan.py --current-limit-a <A> --output-dir <new-dir> --f1-hex <F1.hex> --f4-hex <F4.hex>`.
   This records each SHA-256 and a deterministic combined `firmware_pair_sha256`.
5. Set `operator_reviewed_workspace=true` only after on-site review.

A historical route evidence file can be compared explicitly with
`--verified-routes <file>`. It is not required for generation.

Offline preview:

```bash
python wrist_supplement_plan.py --current-limit-a 1.0 --output-dir /tmp/j45_supplement_check
python batch_collect.py --config-dir .
```

Actual acquisition, only after all review items above are resolved:

```bash
python batch_collect.py --config-dir . --output-dir data/j45_supplement_20261127 --resume --execute --port /dev/ttyACM0
```

`--resume` skips only outputs whose metadata says `completed=true` and
`stop_acknowledged=true`.
"""


def write_outputs_atomic(output_dir: Path, files: dict[str, str]) -> None:
    if output_dir.exists():
        raise FileExistsError(f"Refusing to overwrite existing output directory: {output_dir}")
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{output_dir.name}.", dir=output_dir.parent))
    try:
        for name, text in sorted(files.items()):
            path = staging / name
            with path.open("x", encoding="utf-8") as file:
                file.write(text)
                file.flush()
                os.fsync(file.fileno())
        os.replace(staging, output_dir)
    finally:
        if staging.exists():
            shutil.rmtree(staging)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--verified-routes",
        type=Path,
        help="Optional historical route verifier JSON for explicit evidence comparison",
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--current-limit-a", type=float, required=True)
    parser.add_argument("--cycles", type=int, default=DEFAULT_CYCLES)
    parser.add_argument("--sample-hz", type=int, default=DEFAULT_SAMPLE_HZ)
    parser.add_argument("--f1-hex", type=Path)
    parser.add_argument("--f4-hex", type=Path)
    args = parser.parse_args()
    if not 0 < args.current_limit_a <= 3.3:
        parser.error("--current-limit-a must be in 0..3.3 A")
    if not 1 <= args.cycles <= 20:
        parser.error("--cycles must be in 1..20")
    if not 5 <= args.sample_hz <= 50:
        parser.error("--sample-hz must be in 5..50")
    if (args.f1_hex is None) != (args.f4_hex is None):
        parser.error("--f1-hex and --f4-hex must be supplied together")

    try:
        firmware_fields, firmware_summary = build_firmware_metadata(
            args.f1_hex, args.f4_hex)
    except ValueError as error:
        parser.error(str(error))

    evaluator = PoseEvaluator(DEFAULT_MAPPING, DEFAULT_MJCF, 0.0, include_floor=True)
    evidence_summary = None
    evidence_reference = None
    if args.verified_routes is not None:
        evidence_path = args.verified_routes.resolve()
        try:
            evidence_summary = compare_verified_routes(evidence_path, evaluator)
        except (OSError, ValueError, RuntimeError) as error:
            parser.error(f"Route evidence comparison failed: {error}")
        evidence_reference = {
            "file": evidence_summary["file"],
            "sha256": evidence_summary["sha256"],
        }

    planned = build_configs(
        firmware_fields,
        args.current_limit_a,
        args.cycles,
        args.sample_hz,
        evidence_reference,
    )
    execution_sequence, preflight_checks = execution_sequence_preflight(
        planned, evaluator)

    files = {
        filename: json.dumps(config, indent=2, ensure_ascii=False) + "\n"
        for filename, config in planned
    }
    model_paths = (DEFAULT_MJCF, DEFAULT_MJCF.with_name("dummy.xml"), DEFAULT_MAPPING)
    placeholders = {
        "current_limit_a": "verify against active F1 setup",
        "current_limit_source": "replace descriptive text with readback evidence",
        "firmware_revision": "fill from actual F1/F4 build labels",
        "operator_reviewed_workspace": "operator must set true only after on-site review",
    }
    if args.f1_hex is None:
        placeholders["firmware_hash"] = "supply both actual HEX files and regenerate"
    summary = {
        "schema_version": 2,
        "mode": "offline_only",
        "hardware_connection": None,
        "generator": Path(__file__).name,
        "generator_sha256": sha256_file(Path(__file__).resolve()),
        "route_source": "EXPECTED_ROUTES",
        "route_definition": [
            {"active_joint": active_joint, "fixed_wrist_deg": fixed_wrist}
            for active_joint, fixed_wrist in EXPECTED_ROUTES
        ],
        "shared_start_deg": REFERENCE_FEASIBLE_POSE_DEG,
        "model_sha256": {
            project_path(path): sha256_file(path) for path in model_paths
        },
        "verified_route_evidence": evidence_summary,
        "gripper_rad": 0.0,
        "payload_identity": planned[0][1]["payload_identity"],
        "current_limit_a": args.current_limit_a,
        "cycles": args.cycles,
        "sample_hz": args.sample_hz,
        "primary_speeds_deg_s": list(PRIMARY_SPEEDS_DEG_S),
        "retest_speeds_deg_s": list(RETEST_SPEEDS_DEG_S),
        "config_count": len(planned),
        "configs": [filename for filename, _ in planned],
        "offline_preflight": {
            "route_checks": preflight_checks,
            "execution_sequence": execution_sequence,
        },
        **firmware_summary,
        "operator_placeholders": placeholders,
    }
    files["plan_summary.json"] = json.dumps(
        summary, indent=2, ensure_ascii=False) + "\n"
    files["README_采集流程.md"] = process_markdown()
    write_outputs_atomic(args.output_dir, files)
    print(
        f"Wrote {len(planned)} offline-previewed configs to {args.output_dir}; "
        "no serial port opened")


if __name__ == "__main__":
    main()
