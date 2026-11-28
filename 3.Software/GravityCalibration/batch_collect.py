"""Run generated wrist calibration configs in order, one CSV per config.

Execute mode reads fresh six-axis telemetry and verifies every transition and
sweep segment before the first enable. It then rechecks each segment from the
measured position, monitors joint-space deviation, and always attempts STOP.
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

from collect import (Link, acquisition_limits, read_fresh_pose,
                     require_fresh_pose_close, uses_strict_wrist_safety,
                     validate_config)
from pose_sweep_planner import (DEFAULT_MAPPING, DEFAULT_MJCF, PoseEvaluator,
                                TRANSITION_AXIS_ORDER, transition_waypoints,
                                verify_path)

HERE = Path(__file__).resolve().parent
START_PLAN_TOLERANCE_DEG = 0.1
SETTLE_MIN_DURATION_S = 0.5
SETTLE_MIN_SAMPLES = 5
SETTLE_MAX_GAP_S = 0.35
SETTLE_DURATION_EPSILON_S = 1e-6


def read_status(link: Link) -> list[int]:
    reply = link.request("#GETSTATUS", "oks ", retries=1)
    if len(reply) != 10:
        raise RuntimeError(f"Bad #GETSTATUS reply: {reply}")
    return [int(value) for value in reply[1:]]


def require_ready_status(link: Link, *, require_disabled: bool = False) -> list[int]:
    status = read_status(link)
    enabled, mode, fault_mask = status[0], status[1], status[2]
    if fault_mask or any(status[3:]):
        raise RuntimeError(
            f"Robot or axis fault mask present {status}; resolve before positioning")
    if mode != 2:
        raise RuntimeError(
            f"Robot must already be in position mode 2 for batch positioning; "
            f"reported mode {mode}")
    if require_disabled and enabled:
        raise RuntimeError("Robot is already enabled before the reviewed batch preflight")
    return status


def scan_waypoints(config: dict) -> list[list[float]]:
    fixed_pose = [float(value) for value in config["fixed_pose_deg"]]
    joint = int(config["joint"])
    low = float(config["sweep_min_deg"])
    high = float(config["sweep_max_deg"])
    if not low <= fixed_pose[joint - 1] <= high:
        raise RuntimeError(
            f"{config['pose_id']}: fixed active-axis angle {fixed_pose[joint - 1]} "
            f"is outside the reviewed sweep interval {low}..{high}")
    waypoints = []
    current = fixed_pose.copy()
    for _cycle in range(int(config["cycles"])):
        current[joint - 1] = low
        waypoints.append(current.copy())
        current[joint - 1] = high
        waypoints.append(current.copy())
        current[joint - 1] = low
        waypoints.append(current.copy())
    return waypoints


def path_deviation_deg(pose: list[float], start: list[float],
                       target: list[float]) -> float:
    start_array = np.asarray(start, dtype=float)
    target_array = np.asarray(target, dtype=float)
    pose_array = np.asarray(pose, dtype=float)
    direction = target_array - start_array
    squared_length = float(direction @ direction)
    if squared_length <= 1e-18:
        return float(np.linalg.norm(pose_array - start_array))
    ratio = float((pose_array - start_array) @ direction / squared_length)
    ratio = min(1.0, max(0.0, ratio))
    projected = start_array + ratio * direction
    return float(np.linalg.norm(pose_array - projected))


def _settle_peak_to_peak(window: list[tuple[float, list[float]]]) -> float:
    return float(np.ptp([pose for _now, pose in window], axis=0).max())


def _raise_stop_failure(link: Link, context: str,
                        operation_error: BaseException | None,
                        stop_error: Exception) -> None:
    link.stop_failed = True
    link.last_stop_acknowledged = False
    if operation_error is not None:
        raise RuntimeError(
            f"{context} failed ({operation_error}); final !STOP also failed ({stop_error})"
        ) from operation_error
    raise stop_error


def move_to(link: Link, target: list[float], speed: float, tolerance: float,
            evaluator: PoseEvaluator,
            expected_start: list[float] | None = None) -> None:
    if getattr(link, "stop_failed", False):
        raise RuntimeError(
            "Refusing motor-enable because the previous !STOP was not acknowledged")
    require_ready_status(link, require_disabled=True)
    start = read_fresh_pose(link)
    if expected_start is not None:
        require_fresh_pose_close(
            start, expected_start, START_PLAN_TOLERANCE_DEG,
            "Pre-enable pose differs from the reviewed route")
    verify_path(
        evaluator,
        start,
        [target],
        "batch-segment-before-enable",
    )
    operation_error = None
    try:
        link.request("!START", "Started ok")
        queue_start = read_fresh_pose(link)
        require_fresh_pose_close(
            queue_start, start, START_PLAN_TOLERANCE_DEG,
            "Pose changed after motor-enable")
        if expected_start is not None:
            require_fresh_pose_close(
                queue_start, expected_start, START_PLAN_TOLERANCE_DEG,
                "Queue start differs from the reviewed route")
        queue_status = require_ready_status(link)
        if queue_status[0] != 1:
            raise RuntimeError(f"Robot did not enable before queue: {queue_status}")
        queue_target = queue_start.copy()
        planned_reference = expected_start if expected_start is not None else start
        changed_axes = [
            index for index, (before, after) in enumerate(zip(planned_reference, target))
            if abs(before - after) > 1e-12
        ]
        for index in changed_axes:
            queue_target[index] = target[index]
        verify_path(
            evaluator,
            queue_start,
            [queue_target],
            "batch-segment-immediately-before-queue",
        )
        delta = max(abs(a - b) for a, b in zip(start, queue_target))
        command = ">" + ",".join(f"{value:.5f}" for value in [*queue_target, speed])
        link.request(command, "queue")
        deadline = time.monotonic() + max(20.0, delta / speed * 2.0 + 20.0)
        settle_window = []
        settle_limit = min(0.1, float(tolerance))
        while time.monotonic() < deadline:
            time.sleep(0.1)
            pose = read_fresh_pose(link)
            now = time.monotonic()
            status = require_ready_status(link)
            if not status[0]:
                raise RuntimeError(f"Robot disabled while positioning: {status}")
            ok, tcp, collisions = evaluator.evaluate(pose, limit_margin_deg=0.0)
            if not ok:
                raise RuntimeError(
                    f"Online transition check failed: pose={pose}, "
                    f"tcp_mm={tcp.round(2).tolist()}, collisions={collisions[:3]}")
            deviation = path_deviation_deg(pose, start, queue_target)
            if deviation > max(float(tolerance), 0.25):
                raise RuntimeError(
                    f"Positioning deviated from the reviewed joint-space path by "
                    f"{deviation:.3f} deg: pose={pose}, target={queue_target}")
            if not all(abs(a - b) <= tolerance for a, b in zip(pose, queue_target)):
                settle_window = []
                continue
            if settle_window and now - settle_window[-1][0] > SETTLE_MAX_GAP_S:
                settle_window = []
            settle_window.append((now, pose))
            peak_to_peak = _settle_peak_to_peak(settle_window)
            if peak_to_peak > settle_limit:
                settle_window = [(now, pose)]
                continue
            if (now - settle_window[0][0]
                    >= SETTLE_MIN_DURATION_S - SETTLE_DURATION_EPSILON_S
                    and len(settle_window) >= SETTLE_MIN_SAMPLES):
                return
        raise TimeoutError(
            f"Positioning timeout: target={queue_target}, current={read_fresh_pose(link)}")
    except BaseException as error:
        operation_error = error
        raise
    finally:
        try:
            link.request("!STOP", "Stopped ok")
            link.last_stop_acknowledged = True
        except Exception as stop_error:
            _raise_stop_failure(link, "positioning", operation_error, stop_error)


def preflight_for_config(link: Link, config: dict,
                         evaluator: PoseEvaluator) -> tuple[list[float], list[list[float]]]:
    """Verify the full route and scan before any motor-enable command."""
    require_ready_status(link, require_disabled=True)
    current = read_fresh_pose(link)
    ok, tcp, collisions = evaluator.evaluate(current, limit_margin_deg=0.0)
    if not ok:
        raise RuntimeError(
            "Refusing to move or back off from an unsafe actual start: "
            f"q={[round(value, 4) for value in current]}, "
            f"tcp_mm={tcp.round(2).tolist()}, collisions={collisions[:3]}")

    target = [float(value) for value in config["fixed_pose_deg"]]
    transitions = transition_waypoints(current, target)
    scans = scan_waypoints(config)
    transition_evidence = verify_path(
        evaluator,
        current,
        transitions,
        f"{config['pose_id']}:full-transition",
    )
    scan_evidence = verify_path(
        evaluator,
        target,
        scans,
        f"{config['pose_id']}:full-scan",
    )
    combined = {
        "pose_id": config["pose_id"],
        "actual_start_deg": current,
        "transition": transition_evidence,
        "scan": scan_evidence,
        "enabled_before_preflight": False,
    }
    print("MJCF_BATCH_PREFLIGHT " + json.dumps(combined, ensure_ascii=False), flush=True)
    return current, transitions


def position_for_config(link: Link, config: dict, evaluator: PoseEvaluator,
                        speed: float, tolerance: float) -> None:
    target = [float(value) for value in config["fixed_pose_deg"]]
    current, reviewed_waypoints = preflight_for_config(link, config, evaluator)
    if len(reviewed_waypoints) != len(TRANSITION_AXIS_ORDER):
        raise RuntimeError("Internal transition preflight mismatch")

    for axis, reviewed_target in zip(TRANSITION_AXIS_ORDER, reviewed_waypoints):
        intermediate = current.copy()
        intermediate[axis] = reviewed_target[axis]
        verify_path(
            evaluator,
            current,
            [intermediate],
            f"{config['pose_id']}:remeasured-transition-to-J{axis + 1}",
        )
        if any(abs(a - b) > 1e-9 for a, b in zip(current, intermediate)):
            move_to(
                link,
                intermediate,
                speed,
                tolerance,
                evaluator,
                expected_start=current,
            )
        current = read_fresh_pose(link)
        if any(abs(a - b) > tolerance + 1e-6 for a, b in zip(current, intermediate)):
            raise RuntimeError(
                f"J{axis + 1}: transition did not reach its verified waypoint; "
                f"target={intermediate}, actual={current}")
        ok, tcp, collisions = evaluator.evaluate(current, limit_margin_deg=0.0)
        if not ok:
            raise RuntimeError(
                f"J{axis + 1}: actual transition endpoint is unsafe; q={current}, "
                f"tcp_mm={tcp.round(2).tolist()}, collisions={collisions[:3]}")
    verify_path(evaluator, current, [target], f"{config['pose_id']}:remeasured-final-pose")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config-dir", type=Path,
                        default=HERE / "results" / "pose_configs_j45_50hz")
    parser.add_argument("--output-dir", type=Path,
                        default=HERE / "data" / "j45_50hz")
    parser.add_argument("--execute", action="store_true", help="Enable and move the real robot")
    parser.add_argument("--port")
    parser.add_argument("--joint", type=int, choices=range(1, 7), action="append")
    parser.add_argument("--speed", type=float, action="append")
    parser.add_argument("--start", help="Start at the first config whose name contains this text")
    parser.add_argument("--limit", type=int, help="Only run the first N matching configs")
    parser.add_argument("--resume", action="store_true",
                        help="Skip configs whose output metadata is already completed")
    parser.add_argument("--position-speed", type=float, default=10.0)
    parser.add_argument("--position-tolerance", type=float, default=0.5)
    parser.add_argument("--debug-comm", action="store_true")
    args = parser.parse_args()

    if args.execute and not args.port:
        parser.error("--execute requires --port")
    if args.position_speed <= 0 or args.position_tolerance <= 0:
        parser.error("--position-speed and --position-tolerance must be positive")
    if args.limit is not None and args.limit < 1:
        parser.error("--limit must be positive")

    configs = []
    for path in sorted(args.config_dir.glob("*.json")):
        config = json.loads(path.read_text(encoding="utf-8"))
        required_config_keys = {
            "joint", "fixed_pose_deg", "sweep_min_deg", "sweep_max_deg",
            "speed_deg_s", "cycles",
        }
        if not required_config_keys.issubset(config):
            continue
        if args.joint and config["joint"] not in args.joint:
            continue
        if args.speed and not any(math.isclose(config["speed_deg_s"], speed, abs_tol=1e-9)
                                  for speed in args.speed):
            continue
        if args.start and args.start not in path.name:
            continue
        configs.append((path, config))
    configs.sort(key=lambda item: (
        item[1].get("batch_sequence_index", 10**9),
        item[1].get("speed_index", 10**9),
        item[0].name,
    ))
    configs = [path for path, _config in configs]
    if args.limit is not None:
        configs = configs[:args.limit]
    if not configs:
        parser.error("No configs matched the filters")
    for path in configs:
        config = json.loads(path.read_text(encoding="utf-8"))
        validate_config(config, execute=args.execute)
        if args.execute and not uses_strict_wrist_safety(config):
            raise ValueError(
                f"{path}: batch execution requires safety_profile=wrist_strict_v1")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    jobs = []
    for path in configs:
        config = json.loads(path.read_text(encoding="utf-8"))
        output = args.output_dir / f"{path.stem}.csv"
        metadata = output.with_suffix(".meta.json")
        if args.resume and metadata.exists():
            try:
                record = json.loads(metadata.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                record = {}
            if record.get("completed") is True and record.get("stop_acknowledged") is True:
                print(f"Skip completed and stop-acknowledged {path.name}", flush=True)
                continue
        if args.execute:
            occupied = [
                candidate for candidate in (
                    output,
                    metadata,
                    output.with_suffix(".comm.log") if args.debug_comm else None,
                )
                if candidate is not None and candidate.exists()
            ]
            if occupied:
                raise FileExistsError(
                    "Refusing to overwrite existing acquisition output: "
                    + ", ".join(str(path) for path in occupied))
        jobs.append((path, config, output))
    if not jobs:
        print("No incomplete configs remain", flush=True)
        return

    for path, config, output in jobs:
        if args.execute:
            import serial

            limits = acquisition_limits(config)
            evaluator = PoseEvaluator(
                DEFAULT_MAPPING,
                DEFAULT_MJCF,
                float(config["gripper_rad"]),
                include_floor=True,
                limits_deg=limits,
            )
            with serial.Serial(args.port, 115200, timeout=0.05, write_timeout=0.5) as port:
                port.reset_input_buffer()
                link = Link(port)
                print(f"Preflighting and positioning for {path.name}", flush=True)
                position_for_config(link, config, evaluator,
                                    args.position_speed, args.position_tolerance)
        command = [sys.executable, str(HERE / "collect.py"), "--config", str(path)]
        if args.execute:
            command += ["--execute", "--port", args.port,
                        "--output", str(output)]
            if args.debug_comm:
                command.append("--debug-comm")
        print(" ".join(command), flush=True)
        subprocess.run(command, check=True)


if __name__ == "__main__":
    main()
