"""Record a bounded J4/J5 mode-5 hand-drag acceptance window.

Default is an offline preview that never opens a serial port. ``--execute``
opens the robot serial port exclusively, confirms a stopped position-mode-2
start with read-only queries, enables, switches to explicit DRAG mode 5, and
records the wrist while the operator pushes and then releases it. The script
never sends joint targets and never writes firmware/EEPROM parameters; the
only motion-state commands are ``!START``, ``!STOP``, and ``#CMDMODE 5/2``.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field
from datetime import datetime, timezone
import itertools
import json
import math
from pathlib import Path
import sys
import threading
import time
from typing import Any, Callable

import numpy as np

from calibration import WRIST_STRICT_LIMITS_DEG, tick_delta_ms
from collect import (FRESH_TELEMETRY_LIMIT_MS, Link, _TelemetrySample,
                     acquisition_limits, read_fresh_pose,
                     require_fresh_pose_close, status_fields)
from pose_sweep_planner import (COLLISION_TOLERANCE_M, DEFAULT_MAPPING,
                                DEFAULT_MJCF, PoseEvaluator, TCP_MIN_X_MM,
                                TCP_MIN_Z_MM)

MODE = 5
POST_RELEASE_S = 10.0
MAX_TRIAL_DURATION_S = 600.0
TELEMETRY_SKEW_LIMIT_MS = 10.0
UINT32_HALF_RANGE = 1 << 31


@dataclass(frozen=True)
class TrialSettings:
    reference_deg: tuple[float, float, float, float, float, float]
    j4_interval_deg: tuple[float, float]
    j5_interval_deg: tuple[float, float]
    other_deviation_deg: float
    duration_s: float
    sample_hz: float
    max_telemetry_gap_ms: float
    start_tolerance_deg: float
    gripper_rad: float
    corridor_step_deg: float
    joint: int = 4
    hands_off: bool = False
    j3_deviation_deg: float | None = None

    def held_bound(self, axis: int) -> float:
        if axis == 2 and self.j3_deviation_deg is not None:
            return self.j3_deviation_deg
        return self.other_deviation_deg


@dataclass
class CapturedSample:
    sample_index: int
    t_s: float
    phase: str
    release_marker: bool
    q_deg: list[float]
    samples: list[_TelemetrySample]
    status: list[int]
    snapshot_age_ms: float
    other_max_deviation_deg: float
    j4_excursion_deg: float
    j5_excursion_deg: float
    active_velocity_deg_s: float = 0.0
    wrist_readbacks: dict[str, Any] = field(default_factory=dict)


CSV_FIELDS = [
    "t_s",
    "sample_index",
    "phase",
    "release_marker",
    "enabled",
    "mode",
    "robot_fault_mask",
    "snapshot_age_ms",
    "max_roundtrip_ms",
    "fault_flags_mask",
    "status_supported_all",
    "other_max_deviation_deg",
    "j4_excursion_deg",
    "j5_excursion_deg",
    "j4_q_deg",
    "j5_q_deg",
    "j4_i_cmd_motor_a",
    "j5_i_cmd_motor_a",
    "j4_position_tick_ms",
    "j5_position_tick_ms",
    "j1_q_deg",
    "j2_q_deg",
    "j3_q_deg",
    "j6_q_deg",
    "active_axis_velocity_deg_s",
    "j4_wrist_f",
    "j4_wrist_dir",
    "j4_wrist_current_ma",
    "j4_wrist_query_start_s",
    "j4_wrist_query_end_s",
    "j5_wrist_f",
    "j5_wrist_dir",
    "j5_wrist_current_ma",
    "j5_wrist_query_start_s",
    "j5_wrist_query_end_s",
]


def _finite(value: Any) -> bool:
    return (not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(value))


def validate_settings(settings: TrialSettings) -> None:
    if settings.joint not in (4, 5):
        raise ValueError("active wrist joint must be 4 or 5")
    if (len(settings.reference_deg) != 6
            or not all(_finite(value) for value in settings.reference_deg)):
        raise ValueError("reference pose must contain six finite angles")
    for name, interval in (("J4", settings.j4_interval_deg),
                           ("J5", settings.j5_interval_deg)):
        if (len(interval) != 2 or not all(_finite(v) for v in interval)
                or interval[0] >= interval[1]):
            raise ValueError(f"{name} interval must be finite with low < high")
    for axis, interval in ((3, settings.j4_interval_deg),
                           (4, settings.j5_interval_deg)):
        if not interval[0] <= settings.reference_deg[axis] <= interval[1]:
            raise ValueError(
                f"reference must lie inside the J{axis + 1} interval")
    for name, value, low, high in [
        ("other deviation", settings.other_deviation_deg, 0.01, 20.0),
        ("J3 deviation", settings.held_bound(2), 0.01, 20.0),
        ("duration", settings.duration_s, 0.1, MAX_TRIAL_DURATION_S),
        ("sample rate", settings.sample_hz, 2.0, 20.0),
        ("max telemetry gap", settings.max_telemetry_gap_ms, 50.0, 5000.0),
        ("start tolerance", settings.start_tolerance_deg, 0.01, 1.0),
        ("gripper opening", settings.gripper_rad, 0.0, 0.78),
        ("corridor step", settings.corridor_step_deg, 0.25, 5.0),
    ]:
        if not _finite(value) or not low <= value <= high:
            raise ValueError(f"{name} must be in {low:g}..{high:g}")
    if settings.max_telemetry_gap_ms < 1250.0 / settings.sample_hz:
        raise ValueError(
            "max telemetry gap must be at least 1.25 sample periods")
    if settings.hands_off and (
            settings.duration_s < POST_RELEASE_S + 1.0 / settings.sample_hz):
        raise ValueError(
            "--hands-off requires the full 10 second rest window plus "
            "one sample inside --duration")

    limits = acquisition_limits({"safety_profile": "wrist_strict_v1"})
    for axis, value in enumerate(settings.reference_deg):
        if not limits[axis, 0] <= value <= limits[axis, 1]:
            raise ValueError(f"reference J{axis + 1} is outside strict limits")
    for axis, interval in ((3, settings.j4_interval_deg),
                           (4, settings.j5_interval_deg)):
        if (interval[0] < limits[axis, 0]
                or interval[1] > limits[axis, 1]):
            raise ValueError(f"J{axis + 1} interval is outside strict limits")
    for axis in (0, 1, 2, 5):
        if (settings.reference_deg[axis] - settings.held_bound(axis)
                < limits[axis, 0]
                or settings.reference_deg[axis] + settings.held_bound(axis)
                > limits[axis, 1]):
            raise ValueError(
                f"other-axis deviation bound for J{axis + 1} exceeds strict limits")


def _grid_interval(low: float, high: float, step: float) -> list[float]:
    count = max(1, int(math.ceil((high - low) / step)))
    return [low + (high - low) * index / count
            for index in range(count + 1)]


def _wrist_pose(base, j4: float, j5: float) -> list[float]:
    pose = list(base)
    pose[3] = j4
    pose[4] = j5
    return pose


def preflight_wrist_corridor(evaluator: PoseEvaluator, settings: TrialSettings
                             ) -> dict[str, Any]:
    validate_settings(settings)
    active_axis = settings.joint
    active_interval = (settings.j4_interval_deg if active_axis == 4
                       else settings.j5_interval_deg)
    non_active_axis = 5 if active_axis == 4 else 4
    non_active_index = non_active_axis - 1
    peer_interval = (settings.j5_interval_deg if active_axis == 4
                     else settings.j4_interval_deg)
    held_indices = [0, 1, 2, 5, non_active_index]
    poses = []
    active_values = _grid_interval(
        *active_interval, settings.corridor_step_deg)
    for active_value in active_values:
        pose = list(settings.reference_deg)
        pose[active_axis - 1] = active_value
        poses.append(tuple(pose))
    for signs in itertools.product((-1.0, 1.0), repeat=len(held_indices)):
        held = list(settings.reference_deg)
        for axis, sign in zip(held_indices, signs):
            held[axis] += sign * settings.held_bound(axis)
        held[non_active_index] = min(
            peer_interval[1], max(peer_interval[0], held[non_active_index]))
        for active_value in active_values:
            pose = list(held)
            pose[active_axis - 1] = active_value
            poses.append(tuple(pose))

    tcp_points = []
    for pose in dict.fromkeys(poses):
        q = list(pose)
        ok, tcp, collisions = evaluator.evaluate(q, limit_margin_deg=0.0)
        if not ok:
            raise RuntimeError(
                f"Wrist corridor preflight failed at "
                f"q={[round(value, 4) for value in q]}, "
                f"tcp_mm={np.round(tcp, 2).tolist()}, "
                f"collisions={collisions[:3]}")
        tcp_points.append(np.asarray(tcp, dtype=float))
    tcp = np.asarray(tcp_points)
    evidence = {
        "checked_poses": len(tcp_points),
        "wrist_grid_step_deg": settings.corridor_step_deg,
        "other_axes_bound_deg": settings.other_deviation_deg,
        "j3_bound_deg": settings.held_bound(2),
        "tcp_min_mm": np.min(tcp, axis=0).round(6).tolist(),
        "tcp_max_mm": np.max(tcp, axis=0).round(6).tolist(),
        "tcp_x_min_mm": TCP_MIN_X_MM,
        "tcp_z_min_mm": TCP_MIN_Z_MM,
        "collision_tolerance_m": COLLISION_TOLERANCE_M,
        "floor_collisions_required": True,
        "safety_profile": "wrist_strict_v1",
    }
    print("WRIST_CORRIDOR_PREFLIGHT "
          + json.dumps(evidence, ensure_ascii=False), flush=True)
    return evidence


def _prefixed_value(field: str, prefix: str, cast: Callable[[str], Any]
                    ) -> Any:
    if not isinstance(field, str) or not field.startswith(prefix):
        raise ValueError(f"Bad {prefix} field: {field!r}")
    try:
        value = cast(field[len(prefix):])
    except ValueError as error:
        raise ValueError(f"Bad {prefix} field: {field!r}") from error
    if (isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)):
        raise ValueError(f"Non-finite {prefix} field: {field!r}")
    return value


def parse_wrist_readback(reply: list[str]) -> dict[str, Any]:
    if len(reply) != 12 or reply[0] != "okw":
        raise ValueError(f"Bad #GETWRIST reply: {reply}")
    try:
        joint = int(reply[1])
    except ValueError as error:
        raise ValueError(f"Bad #GETWRIST joint: {reply}") from error
    if joint not in (4, 5):
        raise ValueError(f"Bad #GETWRIST joint: {reply}")
    return {
        "raw": list(reply),
        "joint": joint,
        "friction_enabled_scale": _prefixed_value(reply[4], "f", float),
        "direction": _prefixed_value(reply[8], "dir", int),
        "actual_assist_ma": _prefixed_value(reply[9], "current", int),
    }


def parse_drag_readback(reply: list[str]) -> dict[str, Any]:
    if len(reply) != 8 or reply[0] != "okg":
        raise ValueError(f"Bad #GETDRAG reply: {reply}")
    try:
        mode = int(reply[1])
        feedforward_raw = int(reply[2])
        if feedforward_raw not in (0, 1):
            raise ValueError(f"Bad #GETDRAG feedforward: {reply[2]!r}")
        feedforward = feedforward_raw == 1
        damping = float(reply[3])
        friction_alpha = float(reply[4])
        current_limit_a = float(reply[5])
    except ValueError as error:
        raise ValueError(f"Bad #GETDRAG reply: {reply}") from error
    if not all(math.isfinite(v) for v in (damping, friction_alpha, current_limit_a)):
        raise ValueError(f"Non-finite #GETDRAG reply: {reply}")
    return {
        "mode": mode,
        "gravity_feedforward_enabled": bool(feedforward),
        "damping": damping,
        "friction_alpha": friction_alpha,
        "current_limit_a": current_limit_a,
    }


def query_configured_readbacks(link: Link) -> dict[str, Any]:
    wrist = {}
    for joint in (4, 5):
        parsed = parse_wrist_readback(
            link.request(f"#GETWRIST {joint}", "okw "))
        if parsed["joint"] != joint:
            raise RuntimeError(
                f"#GETWRIST axis mismatch: requested J{joint}, "
                f"received J{parsed['joint']}")
        wrist[f"J{joint}"] = parsed
    drag = parse_drag_readback(link.request("#GETDRAG", "okg "))
    return {"wrist": wrist, "drag": drag}


def _query_wrist_with_timing(link: Link, joint: int) -> dict[str, Any]:
    if joint not in (4, 5):
        raise ValueError("wrist readback joint must be 4 or 5")
    query_start = time.monotonic()
    command = f"#GETWRIST {joint}"
    for attempt in range(3):
        try:
            reply = link.request(command, "okw ")
            break
        except RuntimeError as error:
            # F4 consumed the inquiry and returned a complete CAN-side error.
            # A silent USB timeout or malformed reply cannot be retried here.
            if (str(error) != "error wrist inquiry timeout (requires new F1 protocol)"
                    or attempt == 2):
                raise
            _validate_status(status_fields(link, retries=0), MODE, require_enabled=True)
            link.retries += 1
            time.sleep(0.003)
    parsed = parse_wrist_readback(reply)
    query_end = time.monotonic()
    if parsed["joint"] != joint:
        raise RuntimeError(
            f"#GETWRIST axis mismatch: requested J{joint}, "
            f"received J{parsed['joint']}")
    parsed["query_start"] = query_start
    parsed["query_end"] = query_end
    return parsed


def _validate_status(status: list[int], expected_mode: int,
                     require_enabled: bool | None = None) -> None:
    if len(status) != 9:
        raise RuntimeError(f"Bad #GETSTATUS fields: {status}")
    enabled, mode, robot_fault = status[0], status[1], status[2]
    if robot_fault or any(status[3:]):
        raise RuntimeError(
            f"Robot or axis fault present: status={status}; "
            "the trial never clears faults")
    if mode != expected_mode:
        raise RuntimeError(
            f"Expected command mode {expected_mode}, received {mode}")
    if require_enabled is True and enabled != 1:
        raise RuntimeError("Robot disabled during the enabled trial")
    if require_enabled is False and enabled != 0:
        raise RuntimeError("Robot remained enabled when stopped state was required")


def _snapshot_age_ms(samples: list[_TelemetrySample]) -> float:
    starts = [sample.query_start for sample in samples
              if sample.query_start is not None]
    ends = []
    for sample in samples:
        if sample.status_reply is not None:
            ends.append(sample.status_reply)
        elif sample.query_reply is not None:
            ends.append(sample.query_reply)
    if len(starts) != 6 or len(ends) != 6:
        raise RuntimeError("Fresh telemetry did not provide complete timing")
    return (max(ends) - min(starts)) * 1000.0


def _row(sample: CapturedSample) -> dict[str, Any]:
    max_roundtrip = max(item.roundtrip_ms for item in sample.samples)
    fault_mask = sum(int(item["fault_flags"]) for item in sample.samples)
    all_supported = all(bool(item["status_supported"]) for item in sample.samples)
    return {
        "t_s": f"{sample.t_s:.6f}",
        "sample_index": sample.sample_index,
        "phase": sample.phase,
        "release_marker": int(sample.release_marker),
        "enabled": sample.status[0],
        "mode": sample.status[1],
        "robot_fault_mask": sample.status[2],
        "snapshot_age_ms": f"{sample.snapshot_age_ms:.6f}",
        "max_roundtrip_ms": f"{max_roundtrip:.6f}",
        "fault_flags_mask": fault_mask,
        "status_supported_all": int(all_supported),
        "other_max_deviation_deg":
            f"{sample.other_max_deviation_deg:.6f}",
        "j4_excursion_deg": f"{sample.j4_excursion_deg:.6f}",
        "j5_excursion_deg": f"{sample.j5_excursion_deg:.6f}",
        "j4_q_deg": f"{sample.q_deg[3]:.8f}",
        "j5_q_deg": f"{sample.q_deg[4]:.8f}",
        "j4_i_cmd_motor_a":
            f"{sample.samples[3]['i_cmd_motor_a']:.8f}",
        "j5_i_cmd_motor_a":
            f"{sample.samples[4]['i_cmd_motor_a']:.8f}",
        "j4_position_tick_ms": int(sample.samples[3]["position_tick_ms"]),
        "j5_position_tick_ms": int(sample.samples[4]["position_tick_ms"]),
        "j1_q_deg": f"{sample.q_deg[0]:.8f}",
        "j2_q_deg": f"{sample.q_deg[1]:.8f}",
        "j3_q_deg": f"{sample.q_deg[2]:.8f}",
        "j6_q_deg": f"{sample.q_deg[5]:.8f}",
        "active_axis_velocity_deg_s":
            f"{sample.active_velocity_deg_s:.8f}",
        "j4_wrist_f":
            f"{sample.wrist_readbacks['J4']['friction_enabled_scale']:.8f}",
        "j4_wrist_dir": int(sample.wrist_readbacks["J4"]["direction"]),
        "j4_wrist_current_ma":
            int(sample.wrist_readbacks["J4"]["actual_assist_ma"]),
        "j4_wrist_query_start_s":
            f"{sample.wrist_readbacks['J4']['query_start']:.6f}",
        "j4_wrist_query_end_s":
            f"{sample.wrist_readbacks['J4']['query_end']:.6f}",
        "j5_wrist_f":
            f"{sample.wrist_readbacks['J5']['friction_enabled_scale']:.8f}",
        "j5_wrist_dir": int(sample.wrist_readbacks["J5"]["direction"]),
        "j5_wrist_current_ma":
            int(sample.wrist_readbacks["J5"]["actual_assist_ma"]),
        "j5_wrist_query_start_s":
            f"{sample.wrist_readbacks['J5']['query_start']:.6f}",
        "j5_wrist_query_end_s":
            f"{sample.wrist_readbacks['J5']['query_end']:.6f}",
    }


def _unverified_drift(reason: str, *, sample_count: int = 0,
                      observed_tick_span_ms: int | None = None
                      ) -> dict[str, Any]:
    return {
        "status": "unverified",
        "reason": reason,
        "sample_count": sample_count,
        "observed_tick_span_ms": observed_tick_span_ms,
        "net_drift_deg": None,
        "peak_to_peak_deg": None,
        "max_tick_gap_ms": None,
    }


def summarize_release_drift(
        samples: list[CapturedSample], release_index: int | None, axis: int,
        max_gap_ms: float) -> dict[str, Any]:
    if release_index is None:
        return _unverified_drift("operator release marker not observed")
    if axis not in (4, 5):
        raise ValueError("drift summary axis must be 4 or 5")
    if not 0 <= release_index < len(samples):
        return _unverified_drift("release marker sample is unavailable")

    window = samples[release_index:]
    start_tick = int(window[0].samples[axis - 1]["position_tick_ms"])
    target_span = int(POST_RELEASE_S * 1000)
    end_index = None
    for index, sample in enumerate(window):
        tick = int(sample.samples[axis - 1]["position_tick_ms"])
        if tick_delta_ms(start_tick, tick) >= target_span:
            end_index = index
            break
    if end_index is None:
        last_tick = int(window[-1].samples[axis - 1]["position_tick_ms"])
        return _unverified_drift(
            "10 second post-release window is not covered",
            sample_count=len(window),
            observed_tick_span_ms=tick_delta_ms(start_tick, last_tick),
        )

    checked = window[:end_index + 1]
    max_gap = 0
    for before, after in zip(checked, checked[1:]):
        previous_tick = int(before.samples[axis - 1]["position_tick_ms"])
        current_tick = int(after.samples[axis - 1]["position_tick_ms"])
        gap = tick_delta_ms(previous_tick, current_tick)
        max_gap = max(max_gap, gap)
        if gap <= 0 or gap >= UINT32_HALF_RANGE or gap > max_gap_ms:
            return _unverified_drift(
                f"post-release telemetry gap {gap} ms exceeds "
                f"{max_gap_ms:g} ms or is not forward",
                sample_count=len(checked),
                observed_tick_span_ms=tick_delta_ms(
                    start_tick, current_tick),
            )

    positions = [float(sample.q_deg[axis - 1]) for sample in checked]
    end_tick = int(checked[-1].samples[axis - 1]["position_tick_ms"])
    end_span = tick_delta_ms(start_tick, end_tick)
    if end_span == target_span:
        end_position = positions[-1]
    else:
        previous = checked[-2]
        previous_span = tick_delta_ms(
            start_tick,
            int(previous.samples[axis - 1]["position_tick_ms"]),
        )
        ratio = (target_span - previous_span) / (end_span - previous_span)
        end_position = positions[-2] + ratio * (positions[-1] - positions[-2])
    peak_positions = [*positions[:-1], end_position]
    return {
        "status": "verified",
        "reason": None,
        "sample_count": len(checked),
        "observed_tick_span_ms": target_span,
        "net_drift_deg": end_position - positions[0],
        "peak_to_peak_deg": max(peak_positions) - min(peak_positions),
        "max_tick_gap_ms": max_gap,
    }


def _require_pose_safe(evaluator: PoseEvaluator, q_deg: list[float],
                       context: str) -> None:
    ok, tcp, collisions = evaluator.evaluate(q_deg, limit_margin_deg=0.0)
    if not ok:
        raise RuntimeError(
            f"{context}: q={[round(value, 4) for value in q_deg]}, "
            f"tcp_mm={np.round(tcp, 2).tolist()}, "
            f"collisions={collisions[:3]}")


def _validate_start_pose(q_deg: list[float], settings: TrialSettings,
                         evaluator: PoseEvaluator) -> None:
    if len(q_deg) != 6 or not all(math.isfinite(value) for value in q_deg):
        raise RuntimeError("Stopped actual start is not a finite six-axis pose")
    active_axis = settings.joint
    active_interval = (settings.j4_interval_deg if active_axis == 4
                       else settings.j5_interval_deg)
    held_axes = [axis for axis in range(6) if axis != active_axis - 1]
    held_deviation = max(
        abs(q_deg[axis] - settings.reference_deg[axis])
        for axis in held_axes
    )
    if held_deviation > settings.start_tolerance_deg:
        axis = max(held_axes, key=lambda a: abs(
            q_deg[a] - settings.reference_deg[a]))
        raise RuntimeError(
            f"Stopped held J{axis + 1} differs from reference by "
            f"{held_deviation:.4f} deg (limit "
            f"{settings.start_tolerance_deg:.4f})")
    if not active_interval[0] <= q_deg[active_axis - 1] <= active_interval[1]:
        raise RuntimeError(
            f"Stopped active J{active_axis} is outside the reviewed interval "
            f"{active_interval}")
    _require_pose_safe(evaluator, q_deg, "Stopped actual start")


def _validate_capture(
        q_deg: list[float], samples: list[_TelemetrySample], status: list[int],
        settings: TrialSettings, evaluator: PoseEvaluator,
        previous_ticks: list[int] | None, drag_current_limit_a: float,
        ) -> tuple[float, float, float, float]:
    if len(q_deg) != 6 or len(samples) != 6:
        raise RuntimeError("Fresh six-axis snapshot is incomplete")
    if not all(math.isfinite(value) for value in q_deg):
        raise RuntimeError("Fresh pose contains a non-finite angle")
    for axis, sample in enumerate(samples, start=1):
        if sample["fault_flags"] or not sample["status_supported"]:
            raise RuntimeError(f"J{axis} telemetry fault or unsupported status")
        if sample["skew_ms"] > TELEMETRY_SKEW_LIMIT_MS:
            raise RuntimeError(f"J{axis} telemetry ticks are skewed")
        if (sample.roundtrip_ms < 0
                or sample.roundtrip_ms > FRESH_TELEMETRY_LIMIT_MS):
            raise RuntimeError(f"J{axis} telemetry is stale")

    ticks = [int(item["position_tick_ms"]) for item in samples]
    if previous_ticks is not None:
        for axis, (previous, current) in enumerate(
                zip(previous_ticks, ticks), start=1):
            gap = tick_delta_ms(previous, current)
            if (gap <= 0 or gap >= UINT32_HALF_RANGE
                    or gap > settings.max_telemetry_gap_ms):
                raise RuntimeError(
                    f"J{axis} F4 position tick gap {gap} ms is outside "
                    f"1..{settings.max_telemetry_gap_ms:g} ms")

    ok, tcp, collisions = evaluator.evaluate(q_deg, limit_margin_deg=0.0)
    if not ok:
        raise RuntimeError(
            f"Online wrist safety check failed: q={q_deg}, "
            f"tcp_mm={np.round(tcp, 2).tolist()}, "
            f"collisions={collisions[:3]}")
    active_axis = settings.joint
    held_axes = [axis for axis in range(6) if axis != active_axis - 1]
    other_deviation = max(
        abs(q_deg[axis] - settings.reference_deg[axis])
        for axis in held_axes
    )
    for axis in held_axes:
        deviation = abs(q_deg[axis] - settings.reference_deg[axis])
        if deviation > settings.held_bound(axis):
            raise RuntimeError(
                f"Held J{axis + 1} deviated by {deviation:.4f} deg, above "
                f"{settings.held_bound(axis):.4f} deg; q={q_deg}")
    j4_excursion = max(0.0, settings.j4_interval_deg[0] - q_deg[3],
                       q_deg[3] - settings.j4_interval_deg[1])
    j5_excursion = max(0.0, settings.j5_interval_deg[0] - q_deg[4],
                       q_deg[4] - settings.j5_interval_deg[1])
    if j4_excursion or j5_excursion:
        raise RuntimeError(
            f"Wrist left its active interval: J4 excess={j4_excursion:.4f}, "
            f"J5 excess={j5_excursion:.4f} deg")
    for axis in (4, 5):
        current = abs(float(samples[axis - 1]["i_cmd_motor_a"]))
        if current >= 0.9 * drag_current_limit_a:
            raise RuntimeError(
                f"J{axis} commanded current {current:.4f} A reached 90% of "
                f"the reviewed {drag_current_limit_a:.4f} A drag limit")
    if status[0] != 1 or status[1] != MODE:
        raise RuntimeError(
            f"Enabled/mode changed during trial: expected 1/{MODE}, "
            f"received {status[0]}/{status[1]}")
    return _snapshot_age_ms(samples), other_deviation, j4_excursion, j5_excursion


def run_trial(
        link: Link, settings: TrialSettings, evaluator: PoseEvaluator,
        writer: csv.DictWriter, file, *,
        release_requested: Callable[[float], bool] | None,
        drag_current_limit_a: float,
        metadata_callback: Callable[[], None] | None = None,
        release_source: str | None = None,
        ) -> tuple[list[CapturedSample], int | None, dict[str, Any]]:
    validate_settings(settings)
    if not _finite(drag_current_limit_a) or not (
            0.1 <= drag_current_limit_a <= 1.5):
        raise ValueError("drag current limit readback must be in 0.1..1.5 A")
    active_axis = settings.joint
    session_start = time.monotonic()
    next_sample_at = session_start + 1.0 / settings.sample_hz
    previous_ticks = None
    captured: list[CapturedSample] = []
    release_index = None
    release_info: dict[str, Any] = {
        "source": (
            release_source or
            ("hands_off" if settings.hands_off else "runtime callback")
        ),
        "hands_off": settings.hands_off,
        "sample_index": None,
        "elapsed_s": None,
        "wall_time_utc": None,
    }

    while True:
        elapsed = time.monotonic() - session_start
        marker = (not settings.hands_off
                  and release_index is None
                  and release_requested is not None
                  and bool(release_requested(elapsed)))
        q_deg, samples = read_fresh_pose(
            link, return_samples=True, expected_mode=MODE)
        status = status_fields(link, retries=1)
        _validate_status(status, MODE, require_enabled=True)
        snapshot_age, other_deviation, j4_excursion, j5_excursion = (
            _validate_capture(
                q_deg, samples, status, settings, evaluator,
                previous_ticks, drag_current_limit_a,
            )
        )
        wrist_readbacks = {
            f"J{joint}": _query_wrist_with_timing(link, joint)
            for joint in (4, 5)
        }
        previous_ticks = [int(item["position_tick_ms"]) for item in samples]
        active_velocity = float(samples[active_axis - 1]["dq_deg_s"])

        if marker:
            release_index = len(captured)
            release_info["sample_index"] = release_index
            release_info["elapsed_s"] = elapsed
            release_info["wall_time_utc"] = (
                datetime.now(timezone.utc).isoformat())

        phase = "drag"
        if settings.hands_off:
            phase = "hands_off"
        elif release_index is not None:
            phase = "post_release"

        captured_sample = CapturedSample(
            sample_index=len(captured),
            t_s=time.monotonic() - session_start,
            phase=phase,
            release_marker=marker,
            q_deg=list(q_deg),
            samples=samples,
            status=status,
            snapshot_age_ms=snapshot_age,
            other_max_deviation_deg=other_deviation,
            j4_excursion_deg=j4_excursion,
            j5_excursion_deg=j5_excursion,
            active_velocity_deg_s=active_velocity,
            wrist_readbacks=wrist_readbacks,
        )
        captured.append(captured_sample)
        writer.writerow(_row(captured_sample))
        file.flush()
        if metadata_callback is not None:
            metadata_callback()

        if settings.hands_off:
            release_info["drift"] = {
                "J4": summarize_release_drift(
                    captured, 0, 4, settings.max_telemetry_gap_ms),
                "J5": summarize_release_drift(
                    captured, 0, 5, settings.max_telemetry_gap_ms),
            }
            if all(item["status"] == "verified"
                   for item in release_info["drift"].values()):
                return captured, None, release_info

        if release_index is not None:
            release_info["drift"] = {
                "J4": summarize_release_drift(
                    captured, release_index, 4, settings.max_telemetry_gap_ms),
                "J5": summarize_release_drift(
                    captured, release_index, 5, settings.max_telemetry_gap_ms),
            }
            if all(item["status"] == "verified"
                   for item in release_info["drift"].values()):
                return captured, release_index, release_info

        now = time.monotonic()
        if now - session_start >= settings.duration_s:
            if release_index is None and not settings.hands_off:
                release_info["drift"] = {
                    "J4": _unverified_drift("release marker not observed"),
                    "J5": _unverified_drift("release marker not observed"),
                }
            elif settings.hands_off and "drift" not in release_info:
                release_info["drift"] = {
                    "J4": _unverified_drift("hands-off window not covered"),
                    "J5": _unverified_drift("hands-off window not covered"),
                }
            release_info["timed_out"] = True
            return captured, release_index, release_info
        sleep_until = next_sample_at
        next_sample_at += 1.0 / settings.sample_hz
        time.sleep(max(0.0, sleep_until - time.monotonic()))


def _after_stop_readback_ok(readback: dict[str, Any]) -> bool:
    drag = readback.get("drag", {})
    wrist = readback.get("wrist", {})
    j4 = wrist.get("J4", {})
    j5 = wrist.get("J5", {})
    return (
        drag.get("mode") == 2
        and j4.get("friction_enabled_scale") == 0
        and j5.get("friction_enabled_scale") == 0
        and j4.get("actual_assist_ma") == 0
        and j5.get("actual_assist_ma") == 0
    )


def stop_and_restore_mode_2(link: Link) -> dict[str, Any]:
    result: dict[str, Any] = {
        "stop_acknowledged": False,
        "stop_confirmed": False,
        "mode_2_restored": False,
        "verified": False,
        "status_after_stop": None,
        "status_after_restore": None,
    }
    try:
        link.request("!STOP", "Stopped ok")
        result["stop_acknowledged"] = True
    except Exception as error:
        result["stop_error"] = str(error) or type(error).__name__
        result["reason"] = "!STOP was not acknowledged"
        return result

    if getattr(link, "_desynchronized", False):
        result["reason"] = (
            "serial link is desynchronized; only !STOP was issued and "
            "disabled/mode-2 state remains unverified")
        return result

    try:
        after_stop = status_fields(link, retries=1)
    except Exception as error:
        result["reason"] = (
            "stop state could not be read: "
            + (str(error) or type(error).__name__))
        return result
    result["status_after_stop"] = after_stop
    if after_stop[0] != 0:
        result["reason"] = "robot remained enabled after !STOP"
        return result
    result["stop_confirmed"] = True

    if after_stop[1] != 2:
        try:
            link.request("#CMDMODE 2", "Set command mode to [2]")
        except Exception as error:
            result["mode_restore_error"] = (
                str(error) or type(error).__name__)
            return result
        if getattr(link, "_desynchronized", False):
            result["reason"] = (
                "serial link desynchronized during mode restoration")
            return result
        try:
            after_restore = status_fields(link, retries=1)
        except Exception as error:
            result["reason"] = (
                "mode-2 state could not be read: "
                + (str(error) or type(error).__name__))
            return result
        result["status_after_restore"] = after_restore
        if after_restore[0] != 0 or after_restore[1] != 2:
            result["reason"] = (
                "robot is not confirmed stopped in mode 2 after restoration")
            return result

    result["mode_2_restored"] = True
    result["verified"] = True
    return result


def execute_trial(
        link: Link, settings: TrialSettings, evaluator: PoseEvaluator,
        writer: csv.DictWriter, file, outcome: dict[str, Any], *,
        release_requested: Callable[[float], bool] | None,
        metadata_callback: Callable[[], None],
        release_source: str | None = None,
        ) -> None:
    initial_status = status_fields(link, retries=1)
    _validate_status(initial_status, 2, require_enabled=False)
    initial_pose = read_fresh_pose(link, expected_mode=2)
    _validate_start_pose(initial_pose, settings, evaluator)
    outcome["stopped_reference_pose_deg"] = initial_pose
    outcome["corridor_preflight"] = preflight_wrist_corridor(
        evaluator, settings)

    before = query_configured_readbacks(link)
    if before["drag"]["mode"] != 2:
        raise RuntimeError(
            "#GETDRAG did not report command mode 2 before enable")
    outcome["before_enable"] = before

    ready_pose = read_fresh_pose(link, expected_mode=2)
    require_fresh_pose_close(
        ready_pose,
        initial_pose,
        settings.start_tolerance_deg,
        "Stopped pose changed before enable",
    )
    _validate_start_pose(ready_pose, settings, evaluator)
    pre_enable_status = status_fields(link, retries=1)
    _validate_status(pre_enable_status, 2, require_enabled=False)

    try:
        link.request("!START", "Started ok")
        mode2_pose = read_fresh_pose(link, expected_mode=2)
        require_fresh_pose_close(
            mode2_pose,
            ready_pose,
            settings.start_tolerance_deg,
            "Pose changed while entering enabled mode 2",
        )
        _require_pose_safe(evaluator, mode2_pose, "Enabled mode-2 start")
        mode2_status = status_fields(link, retries=1)
        _validate_status(mode2_status, 2, require_enabled=True)

        link.request("#CMDMODE 5", "Set command mode to [5]")
        mode5_pose = read_fresh_pose(link, expected_mode=5)
        require_fresh_pose_close(
            mode5_pose,
            mode2_pose,
            settings.start_tolerance_deg,
            "Pose changed while entering DRAG mode 5",
        )
        _require_pose_safe(evaluator, mode5_pose, "Enabled DRAG mode-5 start")
        mode5_status = status_fields(link, retries=1)
        _validate_status(mode5_status, 5, require_enabled=True)

        active = query_configured_readbacks(link)
        if active["drag"]["mode"] != 5:
            raise RuntimeError(
                "#GETDRAG did not report command mode 5 after switching")
        outcome["active_readback"] = active
        outcome["enabled_start"] = {
            "pose_deg": mode5_pose,
            "status": mode5_status,
            "command_mode": MODE,
        }

        captured, release_index, release_info = run_trial(
            link,
            settings,
            evaluator,
            writer,
            file,
            release_requested=release_requested,
            drag_current_limit_a=active["drag"]["current_limit_a"],
            metadata_callback=metadata_callback,
            release_source=release_source,
        )
        outcome["samples"] = len(captured)
        outcome["release"] = release_info
        outcome["drift"] = release_info.get("drift", {})
        outcome["window_verified"] = all(
            item.get("status") == "verified"
            for item in outcome["drift"].values()
        )
        outcome["run_finished"] = bool(outcome["window_verified"])
    except BaseException as error:
        if "drift" not in outcome:
            reason = f"trial aborted: {error}"
            outcome["drift"] = {
                "J4": _unverified_drift(reason),
                "J5": _unverified_drift(reason),
            }
        outcome["run_error"] = str(error) or type(error).__name__
        raise
    finally:
        outcome["shutdown"] = stop_and_restore_mode_2(link)
        outcome["after_stop_verified"] = False
        if (outcome["shutdown"].get("verified")
                and not getattr(link, "_desynchronized", False)):
            try:
                after_stop = query_configured_readbacks(link)
                outcome["after_stop"] = after_stop
                outcome["after_stop_verified"] = (
                    _after_stop_readback_ok(after_stop))
            except Exception as error:
                outcome["after_stop_error"] = (
                    str(error) or type(error).__name__)
        outcome["completed"] = bool(
            outcome.get("run_finished")
            and outcome.get("window_verified")
            and outcome["shutdown"].get("verified")
            and outcome.get("after_stop_verified")
        )
        metadata_callback()


class KeyboardRelease:
    def __init__(self) -> None:
        self.event = threading.Event()
        self.eof = threading.Event()
        self.thread = threading.Thread(target=self._wait, daemon=True)

    def _wait(self) -> None:
        if sys.stdin.readline():
            self.event.set()
        else:
            self.eof.set()

    def start(self) -> None:
        print("Press Enter when the operator releases the wrist.", flush=True)
        self.thread.start()

    def __call__(self, _elapsed_s: float) -> bool:
        if self.eof.is_set():
            raise RuntimeError(
                "stdin closed before the operator release marker")
        return self.event.is_set()


def _settings_from_args(args) -> TrialSettings:
    settings = TrialSettings(
        reference_deg=tuple(float(value) for value in args.reference),
        j3_deviation_deg=args.j3_deviation,
        j4_interval_deg=(float(args.j4_min), float(args.j4_max)),
        j5_interval_deg=(float(args.j5_min), float(args.j5_max)),
        other_deviation_deg=float(args.other_deviation),
        duration_s=float(args.duration),
        sample_hz=float(args.sample_hz),
        max_telemetry_gap_ms=float(args.max_telemetry_gap_ms),
        start_tolerance_deg=float(args.start_tolerance),
        gripper_rad=float(args.gripper_rad),
        corridor_step_deg=float(args.corridor_step),
        joint=args.joint,
        hands_off=args.hands_off,
    )
    validate_settings(settings)
    return settings


def _build_release_trigger(args) -> tuple[
        Callable[[float], bool] | None, str | None]:
    choices = sum([
        args.hands_off,
        args.release_marker_file is not None,
        args.keyboard_release,
    ])
    if not args.execute and choices == 0:
        return None, None
    if choices != 1:
        raise ValueError(
            "drag trial requires exactly one of --hands-off, "
            "--release-marker-file, or --keyboard-release")
    if args.hands_off:
        return None, "hands_off"
    if args.release_marker_file is not None:
        marker = args.release_marker_file
        if marker.exists():
            raise ValueError(
                f"Release marker file already exists: {marker}")
        return lambda _elapsed_s: marker.exists(), "release_marker_file"
    return KeyboardRelease(), "keyboard"


def _write_metadata(file, outcome: dict[str, Any]) -> None:
    file.seek(0)
    file.truncate()
    file.write(json.dumps(
        outcome, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    file.flush()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", nargs=6, type=float, required=True,
                        metavar=("J1", "J2", "J3", "J4", "J5", "J6"))
    parser.add_argument("--joint", type=int, choices=(4, 5), required=True,
                        help="Active wrist axis; the other wrist axis is held")
    parser.add_argument("--j4-min", type=float, required=True)
    parser.add_argument("--j4-max", type=float, required=True)
    parser.add_argument("--j5-min", type=float, required=True)
    parser.add_argument("--j5-max", type=float, required=True)
    parser.add_argument("--other-deviation", type=float, required=True,
                        help="Maximum |q-reference| on all held axes")
    parser.add_argument("--j3-deviation", type=float,
                        help="Separate reviewed J3 excursion for coupled hand motion")
    parser.add_argument("--duration", type=float, default=60.0,
                        help="Hard trial duration limit in seconds")
    parser.add_argument("--sample-hz", type=float, default=10.0)
    parser.add_argument("--max-telemetry-gap-ms", type=float, default=350.0)
    parser.add_argument("--start-tolerance", type=float, default=0.15)
    parser.add_argument("--gripper-rad", type=float, required=True)
    parser.add_argument("--corridor-step", type=float, default=1.0)
    parser.add_argument("--hands-off", action="store_true",
                        help="Record a standalone 10 s hands-off rest; never "
                             "treat this as an operator release")
    parser.add_argument("--release-marker-file", type=Path)
    parser.add_argument("--keyboard-release", action="store_true")
    parser.add_argument("--execute", action="store_true",
                        help="Open the robot serial port and enable the trial")
    parser.add_argument("--port")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--debug-comm", action="store_true")
    args = parser.parse_args(argv)

    try:
        settings = _settings_from_args(args)
        release_requested, release_source = _build_release_trigger(args)
    except ValueError as error:
        parser.error(str(error))
    if args.execute and (not args.port or args.output is None):
        parser.error("--execute requires --port and --output")

    evaluator = PoseEvaluator(
        DEFAULT_MAPPING,
        DEFAULT_MJCF,
        settings.gripper_rad,
        include_floor=True,
        limits_deg=WRIST_STRICT_LIMITS_DEG,
    )
    corridor = preflight_wrist_corridor(evaluator, settings)
    if not args.execute:
        print(json.dumps({
            "mode": MODE,
            "duration_s": settings.duration_s,
            "serial_opened": False,
            "corridor_preflight": corridor,
        }, indent=2, ensure_ascii=False), flush=True)
        return 0

    if isinstance(release_requested, KeyboardRelease):
        release_requested.start()

    metadata_path = args.output.with_suffix(".meta.json")
    comm_path = args.output.with_suffix(".comm.log")
    reserved = [args.output, metadata_path]
    if args.debug_comm:
        reserved.append(comm_path)
    occupied = [path for path in reserved if path.exists()]
    if occupied:
        parser.error(
            "Refusing to overwrite trial output: "
            + ", ".join(str(path) for path in occupied))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    outcome: dict[str, Any] = {
        "schema_version": 1,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "settings": {
            "mode": MODE,
            "joint": settings.joint,
            "hands_off": settings.hands_off,
            "reference_deg": list(settings.reference_deg),
            "j4_interval_deg": list(settings.j4_interval_deg),
            "j5_interval_deg": list(settings.j5_interval_deg),
            "other_deviation_deg": settings.other_deviation_deg,
            "j3_deviation_deg": settings.held_bound(2),
            "duration_s": settings.duration_s,
            "sample_hz": settings.sample_hz,
            "max_telemetry_gap_ms": settings.max_telemetry_gap_ms,
            "start_tolerance_deg": settings.start_tolerance_deg,
            "gripper_rad": settings.gripper_rad,
            "corridor_step_deg": settings.corridor_step_deg,
            "post_release_window_s": POST_RELEASE_S,
        },
        "hands_off": settings.hands_off,
        "release_source": release_source,
        "source": {
            "pose": "F4 #GETTELEM paired with #GETSTATUS",
            "current": "F1 commanded current, motor coordinates, not measured torque",
            "force": "not measured and not estimated",
            "gravity_friction_configuration": "#GETWRIST J4/J5 readback",
        },
        "corridor_preflight": corridor,
        "completed": False,
        "stop_acknowledged": False,
        "mode_2_restored": False,
    }
    metadata_file = metadata_path.open("x", encoding="utf-8")
    comm_log = comm_path.open("x", encoding="utf-8") if args.debug_comm else None

    def write_metadata() -> None:
        _write_metadata(metadata_file, outcome)

    write_metadata()
    operation_error: BaseException | None = None
    try:
        with args.output.open("x", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=CSV_FIELDS)
            writer.writeheader()
            file.flush()
            import serial

            try:
                with serial.Serial(
                        args.port, 115200, timeout=0.05,
                        write_timeout=0.5, exclusive=True) as port:
                    port.reset_input_buffer()
                    link = Link(port, debug=comm_log)
                    try:
                        execute_trial(
                            link,
                            settings,
                            evaluator,
                            writer,
                            file,
                            outcome,
                            release_requested=release_requested,
                            metadata_callback=write_metadata,
                            release_source=release_source,
                        )
                    finally:
                        outcome["comm_retries"] = link.retries
            except KeyboardInterrupt as error:
                operation_error = error
                outcome["interrupted"] = True
            except BaseException as error:
                operation_error = error
                outcome["error"] = str(error) or type(error).__name__
    except BaseException as error:
        operation_error = error
        outcome["error"] = str(error) or type(error).__name__
    finally:
        shutdown = outcome.get("shutdown", {})
        outcome["stop_acknowledged"] = bool(
            shutdown.get("stop_acknowledged"))
        outcome["stop_confirmed"] = bool(shutdown.get("stop_confirmed"))
        outcome["mode_2_restored"] = bool(
            shutdown.get("mode_2_restored"))
        outcome["completed"] = bool(
            outcome.get("completed")
            and shutdown.get("verified")
            and not operation_error
        )
        outcome["finished_utc"] = datetime.now(timezone.utc).isoformat()
        write_metadata()
        metadata_file.close()
        if comm_log is not None:
            comm_log.close()

    print(
        f"Saved {args.output}; completed={outcome['completed']}; "
        f"stop_acknowledged={outcome['stop_acknowledged']}; "
        f"mode_2_restored={outcome['mode_2_restored']}",
        flush=True,
    )
    if operation_error is not None:
        if isinstance(operation_error, KeyboardInterrupt):
            return 130
        print(
            f"Trial failed: {operation_error}; "
            "the independent hardware stop remains authoritative.",
            file=sys.stderr,
            flush=True,
        )
        return 1
    return 0 if outcome["completed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
