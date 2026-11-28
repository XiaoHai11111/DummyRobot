"""Generate J4/J5 sweep configs checked against the current MJCF model.

The planner uses MuJoCo forward kinematics and self-collision detection, then
writes collect.py-compatible configs. It never opens a serial port or moves the
robot.
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from calibration import (JOINT_LIMITS_DEG, WRIST_HISTORICAL_LIMITS_DEG,
                         WRIST_STRICT_LIMITS_DEG, WRIST_STRICT_PROFILE)
from collect import validate_config

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent

DEFAULT_MAPPING = ROOT / "DummyWeb" / "config.json"
DEFAULT_MJCF = ROOT / "DummyMujoco" / "dummy_description" / "mjcf" / "scene.xml"
DEFAULT_OUTPUT = HERE / "results" / "pose_configs_j45_50hz"
DEFAULT_POSES = ROOT / "GravityCalibration" / "results" / "pose_combos.json"

MARGIN_DEG = 3.0
PATH_STEP_DEG = 2.0
VERIFY_STEP_DEG = 1.0
MIN_SWEEP_SPAN_DEG = 40.0
MAX_SWEEP_SPAN_DEG = 120.0
COLLISION_TOLERANCE_M = 1e-9
TCP_MIN_X_MM = -200.0
TCP_MIN_Z_MM = 5.0
REFERENCE_FEASIBLE_POSE_DEG = [-90.0, -16.0, 110.0, 0.0, 0.0, 0.0]
TRANSITION_AXIS_ORDER = (0, 1, 2, 5, 3, 4)

BASE_GRIDS = {
    1: (-150, -90, -75, 0, 75, 90, 150),
    2: (-70, -35, -16, 0, 35, 55),
    3: (40, 70, 95, 110, 120, 145, 165),
    4: (-170, -110, -90, -55, 0, 55, 90, 110, 170),
    5: (-100, -50, 0, 50, 100),
    6: (-360, -180, 0, 180, 360),
}
PAIRED_GRIDS = {
    4: (-170, -110, -90, -55, 0, 55, 90, 110, 170),
    5: (-100, -50, 0, 50, 100),
}


class PoseEvaluator:
    def __init__(self, mapping_path: Path, mjcf_path: Path, gripper_rad: float,
                 include_floor: bool = True,
                 limits_deg: np.ndarray = WRIST_STRICT_LIMITS_DEG):
        import mujoco

        if not include_floor:
            raise ValueError("Floor collision checks are mandatory")
        if not math.isfinite(gripper_rad) or not 0 <= gripper_rad <= 0.78:
            raise ValueError("Configured gripper opening must be in 0..0.78 rad")
        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        self.data = mujoco.MjData(self.model)
        robot = json.loads(mapping_path.read_text(encoding="utf-8"))["robot"]
        self.mapping = robot["joints"]
        limits_deg = np.asarray(limits_deg, dtype=float)
        if limits_deg.shape != (6, 2):
            raise ValueError("Joint safety limits must have shape 6x2")
        self.qpos_ids = []
        self.joint_ranges = []
        for axis, mapping in enumerate(self.mapping):
            joint = self.model.joint(mapping["model_joint"])
            self.qpos_ids.append(int(self.model.jnt_qposadr[joint.id]))
            low = max(float(mapping["min_deg"]), float(limits_deg[axis, 0]))
            high = min(float(mapping["max_deg"]), float(limits_deg[axis, 1]))
            if low >= high:
                raise ValueError(f"J{axis + 1}: no intersection of mechanism and qualified limits")
            self.joint_ranges.append((low, high))
            if not 0 < len(mapping["model_joint"]) < 64:
                raise ValueError("Invalid joint mapping name")
        self.finger_ids = [
            int(self.model.jnt_qposadr[self.model.joint(name).id])
            for name in robot["gripper"]["model_joints"]
        ]
        self.site_id = int(self.model.site("grasp_tcp").id)
        self.gripper_rad = gripper_rad
        self.include_floor = True
        self.collision_tolerance_m = COLLISION_TOLERANCE_M

    def _geom_name(self, geom_id: int) -> str:
        name = self.model.geom(int(geom_id)).name
        return name if name else f"geom{int(geom_id)}"

    def _adjacent_bodies(self, body1: int, body2: int) -> bool:
        return (int(self.model.body_parentid[body1]) == body2
                or int(self.model.body_parentid[body2]) == body1)

    def evaluate(self, q_deg: list[float], limit_margin_deg: float = MARGIN_DEG
                 ) -> tuple[bool, np.ndarray, list[tuple[str, str, float]]]:
        if len(q_deg) != 6 or not all(math.isfinite(x) for x in q_deg):
            return False, np.full(3, np.nan), []
        if not math.isfinite(limit_margin_deg) or limit_margin_deg < 0:
            raise ValueError("Joint limit margin must be finite and non-negative")
        for q, (low, high), mapping in zip(q_deg, self.joint_ranges, self.mapping):
            if not low + limit_margin_deg <= q <= high - limit_margin_deg:
                return False, np.full(3, np.nan), []
            value = math.radians(mapping["model_sign"] * q + mapping["model_offset_deg"])
            joint_id = self.model.joint(mapping["model_joint"]).id
            if self.model.jnt_limited[joint_id] and not (
                    self.model.jnt_range[joint_id, 0] <= value <= self.model.jnt_range[joint_id, 1]):
                return False, np.full(3, np.nan), []

        for qpos_id, mapping, q in zip(self.qpos_ids, self.mapping, q_deg):
            self.data.qpos[qpos_id] = math.radians(
                mapping["model_sign"] * q + mapping["model_offset_deg"])
        for qpos_id in self.finger_ids:
            self.data.qpos[qpos_id] = self.gripper_rad
        self.data.qvel[:] = 0
        self.mujoco.mj_forward(self.model, self.data)

        tcp_mm = np.asarray(self.data.site_xpos[self.site_id], dtype=float) * 1000.0
        if tcp_mm[0] <= TCP_MIN_X_MM or tcp_mm[2] <= TCP_MIN_Z_MM:
            return False, tcp_mm, []

        collisions = []
        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            if contact.dist >= -self.collision_tolerance_m:
                continue
            name1 = self._geom_name(contact.geom1)
            name2 = self._geom_name(contact.geom2)
            body1 = int(self.model.geom_bodyid[contact.geom1])
            body2 = int(self.model.geom_bodyid[contact.geom2])
            if "floor" not in (name1, name2) and self._adjacent_bodies(body1, body2):
                continue
            collisions.append((name1, name2, float(contact.dist)))
        return not collisions, tcp_mm, collisions


def verify_path(evaluator: PoseEvaluator, start: list[float], targets: list[list[float]],
                label: str, step_deg: float = VERIFY_STEP_DEG) -> dict[str, Any]:
    """Check every joint-space segment at a bounded angular resolution."""
    if not math.isfinite(step_deg) or step_deg <= 0:
        raise ValueError("Path verification step must be positive")
    start = [float(value) for value in start]
    targets = [[float(value) for value in target] for target in targets]
    if len(start) != 6 or any(len(target) != 6 for target in targets):
        raise ValueError("Path verification needs six-joint waypoints")
    tcp_samples = []
    checked = 0

    def evaluate_pose(q: list[float], sample_label: str) -> None:
        nonlocal checked
        ok, tcp, collisions = evaluator.evaluate(q, limit_margin_deg=0.0)
        checked += 1
        if not ok:
            raise RuntimeError(
                f"{label} {sample_label}: MJCF path blocked, q={[round(x, 4) for x in q]}, "
                f"tcp_mm={np.round(tcp, 2).tolist()}, collisions={collisions[:3]}")
        tcp_samples.append(tcp)

    evaluate_pose(start, "start")
    previous = start
    for target_index, target in enumerate(targets, start=1):
        delta = max(abs(a - b) for a, b in zip(previous, target))
        sample_count = max(1, int(math.ceil(delta / step_deg)))
        for sample_index in range(1, sample_count + 1):
            ratio = sample_index / sample_count
            q = [a + (b - a) * ratio for a, b in zip(previous, target)]
            evaluate_pose(q, f"target{target_index}:{ratio:.3f}")
        previous = target
    tcp = np.asarray(tcp_samples)
    return {
        "label": label,
        "waypoints": len(targets) + 1,
        "checked_poses": checked,
        "step_deg": step_deg,
        "tcp_x_min_mm": float(np.min(tcp[:, 0])),
        "tcp_y_min_mm": float(np.min(tcp[:, 1])),
        "tcp_z_min_mm": float(np.min(tcp[:, 2])),
        "collision_tolerance_m": evaluator.collision_tolerance_m,
    }


def transition_waypoints(start: list[float], target: list[float]) -> list[list[float]]:
    """Return one-axis-at-a-time waypoints in the reviewed transition order."""
    waypoints = []
    current = [float(value) for value in start]
    for axis in TRANSITION_AXIS_ORDER:
        current = current.copy()
        current[axis] = float(target[axis])
        waypoints.append(current.copy())
    return waypoints


def order_safe_sequence(evaluator: PoseEvaluator, records: list[dict[str, Any]],
                        start: list[float]) -> list[dict[str, Any]]:
    """Find a safe transition order, beginning with a required source record."""
    if not records:
        return []
    edge_cache: dict[tuple[tuple[float, ...], tuple[float, ...]], dict[str, Any] | None] = {}

    def edge(before: list[float], after: list[float]) -> dict[str, Any] | None:
        key = (tuple(before), tuple(after))
        if key not in edge_cache:
            try:
                edge_cache[key] = verify_path(
                    evaluator,
                    before,
                    transition_waypoints(before, after),
                    "sequence-preflight",
                )
            except RuntimeError:
                edge_cache[key] = None
        return edge_cache[key]

    def search(path: list[int], used: set[int]) -> list[int] | None:
        if len(path) == len(records):
            return path
        before = start if not path else records[path[-1]]["fixed_pose_deg"]
        for index, record in enumerate(records):
            if index in used or edge(before, record["fixed_pose_deg"]) is None:
                continue
            result = search([*path, index], used | {index})
            if result is not None:
                return result
        return None

    starts = [
        index for index, record in enumerate(records)
        if str(record["source"]).startswith("required:")
    ]
    starts.extend(index for index in range(len(records)) if index not in starts)
    for start_index in starts:
        record = records[start_index]
        if edge(start, record["fixed_pose_deg"]) is None:
            continue
        result = search([start_index], {start_index})
        if result is not None:
            return [records[index] for index in result]
    raise ValueError(
        "No complete collision-free ordering exists from the required reference pose")


def candidate_bases(joint: int, sample_count: int, seed: int,
                    existing_poses_path: Path) -> list[tuple[list[float], str]]:
    rng = np.random.default_rng(seed)
    other_joints = [axis for axis in range(1, 7) if axis != joint]
    grids = [BASE_GRIDS[axis] for axis in other_joints]
    if joint in (4, 5):
        paired_axis = 5 if joint == 4 else 4
        grids[other_joints.index(paired_axis)] = PAIRED_GRIDS[paired_axis]

    reference = list(REFERENCE_FEASIBLE_POSE_DEG)
    reference[joint - 1] = 0.0
    bases: list[tuple[list[float], str]] = [
        (reference, "required:positive-feasible"),
    ]
    if existing_poses_path.exists():
        for pose in json.loads(existing_poses_path.read_text(encoding="utf-8")):
            q = list(map(float, pose["fixed_pose_deg"]))
            q[joint - 1] = 0.0
            bases.append((q, f"existing:{pose['pose_id']}"))

    combinations = list(itertools.product(*grids))
    order = rng.permutation(len(combinations))
    for index in order[:sample_count]:
        values = [float(x) for x in combinations[int(index)]]
        q = []
        cursor = 0
        for axis in range(1, 7):
            if axis == joint:
                q.append(0.0)
            else:
                q.append(float(values[cursor]))
                cursor += 1
        bases.append((q, f"grid:{index:04d}"))
    return bases


def plan_sweep(evaluator: PoseEvaluator, base: list[float], joint: int) -> dict[str, Any] | None:
    low_limit, high_limit = evaluator.joint_ranges[joint - 1]
    low_limit += MARGIN_DEG
    high_limit -= MARGIN_DEG
    points = np.arange(low_limit, high_limit + 0.001, PATH_STEP_DEG, dtype=float)
    ok_flags = []
    for angle in points:
        q = list(base)
        q[joint - 1] = float(angle)
        ok, _, _ = evaluator.evaluate(q)
        ok_flags.append(ok)

    runs = []
    start = None
    for index, ok in enumerate(ok_flags + [False]):
        if ok and start is None:
            start = index
        elif not ok and start is not None:
            runs.append((start, index - 1))
            start = None
    if not runs:
        return None
    run_start, run_end = max(runs, key=lambda run: run[1] - run[0])
    low = math.ceil(float(points[run_start]))
    high = math.floor(float(points[run_end]))
    if high - low > MAX_SWEEP_SPAN_DEG:
        center = (low + high) / 2.0
        low = max(low, math.ceil(center - MAX_SWEEP_SPAN_DEG / 2.0))
        high = min(high, math.floor(center + MAX_SWEEP_SPAN_DEG / 2.0))
    if high - low < MIN_SWEEP_SPAN_DEG:
        return None

    verify_angles = np.arange(low, high + 0.001, VERIFY_STEP_DEG, dtype=float)
    verified = []
    for angle in verify_angles:
        q = list(base)
        q[joint - 1] = float(angle)
        ok, tcp, collisions = evaluator.evaluate(q)
        if not ok:
            return None
        verified.append(tcp)
    verified_array = np.asarray(verified)
    fixed_pose = list(base)
    fixed_pose[joint - 1] = float(low)
    return {
        "joint": joint,
        "fixed_pose_deg": [round(float(x), 3) for x in fixed_pose],
        "sweep_min_deg": float(low),
        "sweep_max_deg": float(high),
        "sweep_span_deg": float(high - low),
        "tcp_x_min_mm": float(np.min(verified_array[:, 0])),
        "tcp_z_min_mm": float(np.min(verified_array[:, 2])),
        "collision_path_step_deg": VERIFY_STEP_DEG,
        "collision_tolerance_m": evaluator.collision_tolerance_m,
        "floor_collisions_required": True,
    }


def diverse_select(records: list[dict[str, Any]], limit: int) -> list[dict[str, Any]]:
    selected: list[dict[str, Any]] = []
    for record in records:
        if str(record["source"]).startswith("required:") and len(selected) < limit:
            selected.append(record)
    for record in records:
        if str(record["source"]).startswith("existing:") and len(selected) < limit:
            if record not in selected:
                selected.append(record)
    ranges = np.asarray([high - low for low, high in JOINT_LIMITS_DEG], dtype=float)
    while len(selected) < min(limit, len(records)):
        best = None
        best_score = -1.0
        for record in records:
            if record in selected:
                continue
            q = np.asarray(record["fixed_pose_deg"], dtype=float)
            if selected:
                distances = []
                for other in selected:
                    other_q = np.asarray(other["fixed_pose_deg"], dtype=float)
                    distances.append(float(np.linalg.norm((q - other_q) / ranges)))
                score = min(distances)
            else:
                score = float(record["sweep_span_deg"])
            if score > best_score:
                best_score = score
                best = record
        if best is None:
            break
        selected.append(best)
    return selected


def write_configs(records: list[dict[str, Any]], output_dir: Path, speeds: list[float],
                  cycles: int, sample_hz: int, current_limit_a: float,
                  payload_description: str, gripper_rad: float) -> list[str]:
    output_dir.mkdir(parents=True, exist_ok=True)
    planned = []
    for record in records:
        for speed in speeds:
            token = f"{speed:g}".replace(".", "p")
            joint = record["joint"]
            index = record["pose_index"]
            name = f"j{joint}_s{index:02d}_v{token}.json"
            config = {
                "pose_id": f"J{joint}_S{index:02d}",
                "joint": joint,
                "fixed_pose_deg": record["fixed_pose_deg"],
                "sweep_min_deg": record["sweep_min_deg"],
                "sweep_max_deg": record["sweep_max_deg"],
                "speed_deg_s": float(speed),
                "cycles": cycles,
                "sample_hz": sample_hz,
                "endpoint_hold_s": 1.0,
                "full_pose_poll_hz": 2.0,
                "current_limit_a": current_limit_a,
                "current_limit_source":
                    "Operator must confirm this against the active F1 current limit",
                "payload_description": payload_description,
                "gripper_rad": float(gripper_rad),
                "safety_profile": WRIST_STRICT_PROFILE,
                "temperature_note": "Record measured temperature before execution",
                "firmware_revision": "REPLACE_WITH_F1_F4_BUILD_REVISION",
                "firmware_hash": "REPLACE_WITH_SHA256_OF_ACTUAL_F1_F4_BUILD",
                "operator_reviewed_workspace": False,
                "source_pose": record["source"],
                "batch_sequence_index": record["safe_sequence_index"],
                "speed_index": speeds.index(speed),
                "mjcf_checked": True,
            }
            validate_config(config, execute=False)
            path = output_dir / name
            planned.append((path, config))
    existing = [str(path) for path, _ in planned if path.exists()]
    if existing:
        raise FileExistsError(
            "Refusing to overwrite existing sweep configs: " + ", ".join(existing[:5]))
    paths = []
    for path, config in planned:
        with path.open("x", encoding="utf-8") as file:
            file.write(json.dumps(config, indent=2, ensure_ascii=False) + "\n")
        paths.append(str(path))
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mapping", type=Path, default=DEFAULT_MAPPING)
    parser.add_argument("--mjcf", type=Path, default=DEFAULT_MJCF)
    parser.add_argument("--existing-poses", type=Path, default=DEFAULT_POSES)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--joints", type=int, nargs="+", choices=range(1, 7), default=(4, 5))
    parser.add_argument("--candidates", type=int, default=240,
                        help="Random base poses evaluated per joint")
    parser.add_argument("--max-poses", type=int, default=8,
                        help="Selected fixed poses per joint")
    parser.add_argument("--speeds", type=float, nargs="+", default=(0.5, 1.0, 2.0))
    parser.add_argument("--cycles", type=int, default=5)
    parser.add_argument("--sample-hz", type=int, default=50)
    parser.add_argument("--current-limit-a", type=float, default=3.0)
    parser.add_argument("--gripper-rad", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=20261126)
    parser.add_argument("--payload-description",
                        default="gripper, camera mounted; J4/J5 domestic reducer study")
    args = parser.parse_args()

    if not 5 <= args.sample_hz <= 50:
        parser.error("--sample-hz must be in 5..50")
    if not all(0.3 <= speed <= 5 for speed in args.speeds):
        parser.error("speeds must be in 0.3..5 deg/s")
    if not 1 <= args.cycles <= 20:
        parser.error("--cycles must be in 1..20")
    if args.candidates < 1 or args.max_poses < 1:
        parser.error("--candidates and --max-poses must be positive")
    if not 0 < args.current_limit_a <= 3.3:
        parser.error("--current-limit-a must be in 0..3.3 A")

    evaluator = PoseEvaluator(args.mapping, args.mjcf, args.gripper_rad)
    summary_path = args.output_dir / "pose_summary.json"
    if summary_path.exists():
        parser.error(f"Refusing to overwrite existing summary: {summary_path}")
    summary = {
        "mjcf": str(args.mjcf.resolve()),
        "mapping": str(args.mapping.resolve()),
        "joint_limits_deg": JOINT_LIMITS_DEG.tolist(),
        "wrist_historical_limits_deg": WRIST_HISTORICAL_LIMITS_DEG.tolist(),
        "wrist_strict_limits_deg": WRIST_STRICT_LIMITS_DEG.tolist(),
        "planning_limit_margin_deg": MARGIN_DEG,
        "gripper_rad": args.gripper_rad,
        "safety_profile": WRIST_STRICT_PROFILE,
        "operator_reviewed_workspace": False,
        "constraints": {
            "tcp_x_min_mm": TCP_MIN_X_MM,
            "tcp_z_min_mm": TCP_MIN_Z_MM,
            "self_collision_tolerance_m": evaluator.collision_tolerance_m,
            "ignored_non_ground_contacts": "adjacent MJCF bodies only",
            "floor_collisions_required": True,
            "path_step_deg": VERIFY_STEP_DEG,
        },
        "speeds_deg_s": args.speeds,
        "sample_hz": args.sample_hz,
        "cycles": args.cycles,
        "records": [],
        "configs": [],
    }

    all_records = []
    for joint in args.joints:
        bases = candidate_bases(joint, args.candidates, args.seed + joint, args.existing_poses)
        candidates = []
        transitions_rejected = 0
        for base, source in bases:
            record = plan_sweep(evaluator, base, joint)
            if record is None:
                continue
            try:
                evidence = verify_path(
                    evaluator,
                    REFERENCE_FEASIBLE_POSE_DEG,
                    transition_waypoints(
                        REFERENCE_FEASIBLE_POSE_DEG, record["fixed_pose_deg"]),
                    f"{source}:source-to-fixed",
                )
            except RuntimeError:
                transitions_rejected += 1
                continue
            record["source"] = source
            record["source_pose_deg"] = REFERENCE_FEASIBLE_POSE_DEG.copy()
            record["source_transition_checked_poses"] = evidence["checked_poses"]
            record["source_transition_tcp_x_min_mm"] = evidence["tcp_x_min_mm"]
            record["source_transition_tcp_z_min_mm"] = evidence["tcp_z_min_mm"]
            candidates.append(record)
        selected = diverse_select(candidates, args.max_poses)
        if len(selected) < args.max_poses:
            print(f"J{joint}: only {len(selected)} feasible poses found")
        all_records.extend(selected)
        print(
            f"J{joint}: selected {len(selected)} poses from {len(candidates)} "
            f"scan-and-source-feasible candidates; rejected {transitions_rejected} "
            "source transitions")

    all_records = order_safe_sequence(
        evaluator, all_records, REFERENCE_FEASIBLE_POSE_DEG)
    pose_counts = {joint: 0 for joint in args.joints}
    for sequence_index, record in enumerate(all_records, start=1):
        record["safe_sequence_index"] = sequence_index
        joint = int(record["joint"])
        pose_counts[joint] += 1
        record["pose_index"] = pose_counts[joint]
        record["pose_id"] = f"J{joint}_S{pose_counts[joint]:02d}"
    summary["reference_source_pose_deg"] = REFERENCE_FEASIBLE_POSE_DEG
    summary["records"] = all_records
    summary["configs"] = write_configs(
        all_records, args.output_dir, args.speeds, args.cycles, args.sample_hz,
        args.current_limit_a, args.payload_description, args.gripper_rad)
    with summary_path.open("x", encoding="utf-8") as file:
        file.write(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    print(f"Wrote {len(summary['configs'])} configs and {summary_path}")


if __name__ == "__main__":
    main()
