"""Preview or execute one slow joint sweep and record F4 telemetry.

Default is offline preview. --execute explicitly enables motors and sends motion.
Only one client may own the USB serial port. No current/acceleration gains or
EEPROM parameters are changed. !STOP is attempted on every hardware-run exit.
"""
from __future__ import annotations

import argparse
import csv
from collections import deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import sys
import time

from calibration import (FIELDS, JOINT_LIMITS_DEG, Q_FIELDS,
                         WRIST_STRICT_LIMITS_DEG, WRIST_STRICT_PROFILE,
                         tick_delta_ms, tick_skew_ms)


FRESH_TELEMETRY_LIMIT_MS = 100.0
FRESH_POSE_READ_ATTEMPTS = 3
CAN_TELEMETRY_TIMEOUTS = frozenset({
    "error F1 telemetry timeout",
    "error F1 telemetry timeout (requires extended 0x21/0x22/0x23)",
})


class _TelemetryTransientError(RuntimeError):
    """A complete telemetry transaction reported a retryable CAN timeout."""


class _TelemetrySample(dict):
    """CSV telemetry plus timing that must not be overwritten by later queries."""

    __slots__ = ("query_start", "query_reply", "status_reply", "roundtrip_ms")

    def __init__(self, values, *, query_start, query_reply, roundtrip_ms,
                 status_reply=None):
        super().__init__(values)
        self.query_start = query_start
        self.query_reply = query_reply
        self.status_reply = status_reply
        self.roundtrip_ms = roundtrip_ms


def _validate_status_frame(wire):
    if len(wire) != 10 or wire[0] != "oks":
        raise RuntimeError("Bad #GETSTATUS reply")
    try:
        values = [int(value) for value in wire[1:]]
    except ValueError as error:
        raise RuntimeError("Bad #GETSTATUS reply") from error
    if values[0] not in (0, 1):
        raise RuntimeError("Bad #GETSTATUS enabled field")
    return values


def _require_safe_status(status, context, *, require_enabled=False,
                         expected_mode=2):
    if expected_mode not in (2, 5):
        raise ValueError("expected_mode must be 2 or 5")
    if status[1] != expected_mode or status[2] or any(status[3:]):
        raise RuntimeError(
            f"{context}: expected mode {expected_mode}, "
            f"enabled/mode/faults={status[:3]}, joint_faults={status[3:]}")
    if require_enabled and status[0] != 1:
        raise RuntimeError(
            f"{context}: robot disabled during online acquisition")


def _is_can_telemetry_timeout(text):
    return text in CAN_TELEMETRY_TIMEOUTS


class Link:
    def __init__(self, port, debug=None):
        self.port = port
        self.debug = debug
        self._rx = bytearray()
        self._pending_motion_acks = 0
        self._strict_transaction_used = False
        self._desynchronized = False
        self._desync_error = None
        self._motion_enabled = False
        self.retries = 0
        self.last_attempt_start = None
        self.last_reply_time = None
        self.last_roundtrip_ms = 0.0

    def _log(self, message):
        if self.debug is not None:
            self.debug.write(f"{time.monotonic():.6f} {message}\n")
            self.debug.flush()

    def _read_line(self, deadline):
        while True:
            index = self._rx.find(b"\n")
            if index >= 0:
                line = bytes(self._rx[:index])
                del self._rx[:index + 1]
                return line
            if time.monotonic() >= deadline:
                return None
            chunk = self.port.read_until(b"\n")
            if chunk:
                self._rx.extend(chunk)
                if len(self._rx) > 512:
                    raise RuntimeError("Overlong serial reply")

    def request(self, command, prefix, timeout=0.5, retries=0, joint=None):
        if command != "!STOP":
            self._require_query_allowed()
            if self._strict_transaction_used:
                try:
                    self._consume_buffered_transaction_frames()
                except BaseException as error:
                    self._mark_desynchronized(error)
                    raise
        # Only read-only queries may be retried. Motion/enable commands keep
        # retries=0 so a lost reply can never cause a duplicate side effect.
        for attempt in range(retries + 1):
            attempt_start = time.monotonic()
            self._log(f"TX {command}")
            self.port.write((command + "\n").encode("ascii"))
            deadline = attempt_start + timeout
            while True:
                line = self._read_line(deadline)
                if line is None:
                    break
                text = line.decode("ascii", errors="strict").strip()
                if not text:
                    continue
                self._log(f"RX {text}")
                if text.startswith("error"):
                    # A timed-out #GETTELEM reply can arrive after the next query
                    # was sent. Do not let that stale line fail status/pose reads.
                    if _is_can_telemetry_timeout(text) and prefix in ("okj ", "oks "):
                        self._log("DISCARD stale telemetry error")
                        continue
                    raise RuntimeError(text)
                if text == "ok" and self._pending_motion_acks:
                    self._pending_motion_acks -= 1
                    self._log("RX queued-motion acknowledgement")
                    continue
                # Motion emits an immediate queue count and a later 'ok'; neither
                # should be mistaken for a telemetry response. USB CDC can deliver
                # several complete lines in one read, so later lines stay buffered.
                if (prefix == "queue" and text.isdigit()) or text.startswith(prefix):
                    if command.startswith("#GETTELEM ") and joint is not None:
                        fields = text.split()
                        if len(fields) < 2 or fields[1] != str(joint):
                            self._log(f"DISCARD wrong joint reply: {text}")
                            continue
                    self._log(f"MATCH {prefix}")
                    if prefix == "queue" and text.isdigit():
                        self._pending_motion_acks += 1
                    if command == "!START":
                        self._motion_enabled = True
                    elif command in ("!STOP", "!DISABLE"):
                        self._motion_enabled = False
                    self.last_attempt_start = attempt_start
                    self.last_reply_time = time.monotonic()
                    self.last_roundtrip_ms = (self.last_reply_time - attempt_start) * 1000
                    return text.split()
            self._log(f"TIMEOUT {command}")
            if attempt < retries:
                self.retries += 1
                print(f"[comm] {command}: no reply, retry {attempt + 1}/{retries}",
                      file=sys.stderr, flush=True)
        raise TimeoutError(
            f"No {prefix} response to {command} after {retries + 1} attempts; "
            "aborting, no automatic retry")

    def _mark_desynchronized(self, error):
        if not self._desynchronized:
            self._desynchronized = True
            self._desync_error = str(error) or type(error).__name__

    def _require_query_allowed(self):
        if self._desynchronized:
            raise RuntimeError(
                f"Serial link is desynchronized; only !STOP is allowed: "
                f"{self._desync_error}")

    def _capture_waiting(self):
        waiting = getattr(self.port, "in_waiting", 0)
        if waiting is None:
            return
        if isinstance(waiting, bool) or not isinstance(waiting, int) or waiting < 0:
            raise RuntimeError(f"Invalid serial in_waiting value: {waiting!r}")
        if not waiting:
            return
        reader = getattr(self.port, "read", None)
        if reader is None:
            raise RuntimeError("Serial input is waiting but cannot be read")
        chunk = reader(waiting)
        if not chunk:
            raise RuntimeError("Serial input remained unreadable")
        self._rx.extend(chunk)
        if len(self._rx) > 512:
            raise RuntimeError("Overlong serial reply")

    def _consume_buffered_transaction_frames(self):
        self._capture_waiting()
        while True:
            index = self._rx.find(b"\n")
            if index < 0:
                break
            line = bytes(self._rx[:index])
            del self._rx[:index + 1]
            text = line.decode("ascii", errors="strict").strip()
            if not text:
                continue
            if text == "ok" and self._pending_motion_acks:
                self._pending_motion_acks -= 1
                self._log("RX queued-motion acknowledgement")
                continue
            raise RuntimeError(
                f"Extra serial frame outside telemetry transaction: {text!r}")
        if self._rx:
            raise RuntimeError("Trailing partial serial frame outside telemetry transaction")

    def _read_transaction_line(self, deadline, joint):
        while True:
            line = self._read_line(deadline)
            if line is None:
                raise TimeoutError(
                    f"Incomplete telemetry transaction for J{joint}; "
                    "both #GETTELEM and #GETSTATUS must finish by one deadline")
            received_at = time.monotonic()
            if received_at > deadline:
                raise TimeoutError(
                    f"Telemetry transaction for J{joint} completed after its deadline")
            text = line.decode("ascii", errors="strict").strip()
            if not text:
                continue
            self._log(f"RX transaction {text}")
            if text == "ok" and self._pending_motion_acks:
                self._pending_motion_acks -= 1
                self._log("RX queued-motion acknowledgement")
                continue
            return text, received_at

    def telemetry_transaction(self, joint, timeout=1.0, *, expected_mode=2):
        """Send one telemetry/status pair and consume both replies atomically."""
        if isinstance(joint, bool) or joint not in range(1, 7):
            raise ValueError("joint must be 1..6")
        if expected_mode not in (2, 5):
            raise ValueError("expected_mode must be 2 or 5")
        self._require_query_allowed()
        self._strict_transaction_used = True
        try:
            # Any frame already waiting belongs to an earlier exchange.
            self._consume_buffered_transaction_frames()
            attempt_start = time.monotonic()
            deadline = attempt_start + timeout
            payload = f"#GETTELEM {joint}\n#GETSTATUS\n".encode("ascii")
            self._log(f"TX transaction #GETTELEM {joint} + #GETSTATUS")
            written = self.port.write(payload)
            if written is not None and written != len(payload):
                raise RuntimeError(
                    f"Partial telemetry transaction write: "
                    f"{written}/{len(payload)} bytes")

            transient = False
            telemetry = None
            telemetry_reply = None
            while True:
                text, received_at = self._read_transaction_line(deadline, joint)
                fields = text.split()
                if transient:
                    if not text.startswith("oks "):
                        raise RuntimeError(
                            "Transient telemetry error was not followed by status: "
                            f"{text!r}")
                    status = _validate_status_frame(fields)
                    _require_safe_status(
                        status,
                        f"J{joint} CAN telemetry timeout",
                        require_enabled=self._motion_enabled,
                        expected_mode=expected_mode,
                    )
                    self._consume_buffered_transaction_frames()
                    raise _TelemetryTransientError(
                        f"J{joint} F1 telemetry timeout; "
                        "complete transaction retry allowed")

                if telemetry is None:
                    if text.startswith("okm "):
                        if len(fields) != 11 or fields[1] != str(joint):
                            raise RuntimeError(
                                f"Unexpected telemetry frame in J{joint} "
                                f"transaction: {text!r}")
                        telemetry = fields
                        telemetry_reply = received_at
                        continue
                    if _is_can_telemetry_timeout(text):
                        transient = True
                        continue
                    if text.startswith("oks "):
                        raise RuntimeError(
                            f"Wrong reply order in J{joint} transaction: "
                            "status before telemetry")
                    if text.startswith("error"):
                        raise RuntimeError(text)
                    raise RuntimeError(
                        f"Unexpected frame in J{joint} telemetry transaction: "
                        f"{text!r}")

                if not text.startswith("oks "):
                    raise RuntimeError(
                        f"Telemetry was not followed by one status frame: {text!r}")
                status = _validate_status_frame(fields)
                _require_safe_status(
                    status,
                    f"J{joint} paired telemetry status",
                    require_enabled=self._motion_enabled,
                    expected_mode=expected_mode,
                )
                status_reply = received_at
                break

            self._consume_buffered_transaction_frames()
            self.last_attempt_start = attempt_start
            self.last_reply_time = telemetry_reply
            self.last_roundtrip_ms = (telemetry_reply - attempt_start) * 1000.0
            return (
                telemetry,
                attempt_start,
                telemetry_reply,
                status_reply,
            )
        except _TelemetryTransientError:
            raise
        except BaseException as error:
            self._mark_desynchronized(error)
            raise


def parse_telemetry(data, joint):
    if len(data) != 11 or data[0] != "okm" or int(data[1]) != joint:
        raise ValueError("Bad #GETTELEM reply")
    ticks = list(map(int, data[2:5]))
    q, velocity, current = map(float, data[5:8])
    state, faults, supported = map(int, data[8:11])
    if (not all(math.isfinite(v) for v in (q, velocity, current)) or
            any(t < 0 or t >= 2**32 for t in ticks) or state not in range(7) or
            faults not in range(256) or supported not in (0, 1)):
        raise ValueError("Invalid telemetry values")
    return dict(position_tick_ms=ticks[0], velocity_tick_ms=ticks[1], current_tick_ms=ticks[2],
                skew_ms=tick_skew_ms(ticks), dq_deg_s=velocity, i_cmd_motor_a=current,
                driver_state=state, fault_flags=faults, status_supported=supported), q


def request_telemetry(link, joint, retries=4, *, expected_mode=None):
    """Retry only F4's transient per-node CAN telemetry timeout."""
    if expected_mode is None:
        timing_state = getattr(link, "_collect_snapshot_timing", None)
        expected_mode = (
            timing_state.get("expected_mode", 2)
            if isinstance(timing_state, dict)
            else 2
        )
    if expected_mode not in (2, 5):
        raise ValueError("expected_mode must be 2 or 5")
    for attempt in range(retries + 1):
        transaction = getattr(link, "telemetry_transaction", None)
        if transaction is not None:
            try:
                if expected_mode == 5:
                    wire, query_start, query_reply, status_reply = transaction(
                        joint, timeout=1.0, expected_mode=5)
                else:
                    wire, query_start, query_reply, status_reply = transaction(
                        joint, timeout=1.0)
                sample, q_deg = parse_telemetry(wire, joint)
                roundtrip_ms = (query_reply - query_start) * 1000.0
                return _TelemetrySample(
                    sample,
                    query_start=query_start,
                    query_reply=query_reply,
                    status_reply=status_reply,
                    roundtrip_ms=roundtrip_ms,
                ), q_deg
            except _TelemetryTransientError as error:
                if attempt == retries:
                    raise
                if hasattr(link, "retries"):
                    link.retries += 1
                else:
                    link.retries = 1
                print(
                    f"[comm] {error}; retry {attempt + 1}/{retries}",
                    file=sys.stderr,
                    flush=True,
                )
                time.sleep(0.02)
                continue

        # Keep simulation/custom Link callers compatible. Production Link uses
        # the paired transaction above so silent and wrong-order replies abort.
        snapshot_timing = getattr(link, "_collect_snapshot_timing", None)
        if snapshot_timing is None:
            query_start = time.monotonic()
        else:
            query_start = snapshot_timing.get("query_start")
            if query_start is None:
                query_start = time.monotonic()
        try:
            wire = link.request(
                f"#GETTELEM {joint}",
                "okm ",
                timeout=1.0,
                retries=2,
                joint=joint,
            )
        except RuntimeError as error:
            if not _is_can_telemetry_timeout(str(error)) or attempt == retries:
                raise
            if hasattr(link, "retries"):
                link.retries += 1
            else:
                link.retries = 1
            time.sleep(0.02)
            continue
        query_reply = time.monotonic()
        sample, q_deg = parse_telemetry(wire, joint)
        link_start = getattr(link, "last_attempt_start", None)
        link_reply = getattr(link, "last_reply_time", None)
        link_roundtrip = getattr(link, "last_roundtrip_ms", None)
        provided_timing = {}
        for name, value in [
            ("last_attempt_start", link_start),
            ("last_reply_time", link_reply),
            ("last_roundtrip_ms", link_roundtrip),
        ]:
            if value is None:
                continue
            if (isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(value)):
                raise RuntimeError(f"Invalid fake Link {name}")
            provided_timing[name] = float(value)
        if "last_attempt_start" in provided_timing:
            query_start = provided_timing["last_attempt_start"]
        if "last_reply_time" in provided_timing:
            query_reply = provided_timing["last_reply_time"]
        if query_start > query_reply:
            raise RuntimeError("Invalid fake Link telemetry timing order")
        if "last_roundtrip_ms" in provided_timing:
            roundtrip_ms = provided_timing["last_roundtrip_ms"]
            if roundtrip_ms < 0:
                raise RuntimeError("Invalid fake Link last_roundtrip_ms")
        else:
            roundtrip_ms = (query_reply - query_start) * 1000.0
        if snapshot_timing is not None:
            snapshot_timing["query_start"] = query_reply
        return _TelemetrySample(
            sample,
            query_start=query_start,
            query_reply=query_reply,
            roundtrip_ms=roundtrip_ms,
        ), q_deg
    raise RuntimeError("Unreachable telemetry retry state")


def read_fresh_pose(link, return_samples=False, *, expected_mode=2):
    """Read one bounded six-axis snapshot without mixing stale joints."""
    last_stale = None
    for _snapshot_attempt in range(FRESH_POSE_READ_ATTEMPTS):
        pose = []
        samples = []
        stale_reasons = []
        timing_state = {
            "query_start": None,
            "expected_mode": expected_mode,
        }
        previous_state = getattr(
            link, "_collect_snapshot_timing", timing_state)
        attached_timing = False
        try:
            setattr(link, "_collect_snapshot_timing", timing_state)
            attached_timing = True
        except (AttributeError, TypeError):
            pass
        try:
            for axis in range(1, 7):
                sample, q_deg = request_telemetry(link, axis)
                if (sample["fault_flags"] or not sample["status_supported"]
                        or sample["skew_ms"] > 10):
                    raise RuntimeError(f"J{axis}: invalid fresh telemetry")
                q_deg = float(q_deg)
                if not math.isfinite(q_deg):
                    raise RuntimeError(
                        "Fresh telemetry returned a non-finite joint angle")
                roundtrip_ms = getattr(sample, "roundtrip_ms", None)
                if (roundtrip_ms is not None
                        and roundtrip_ms > FRESH_TELEMETRY_LIMIT_MS):
                    stale_reasons.append(
                        f"J{axis} request latency={roundtrip_ms:.1f} ms")
                pose.append(q_deg)
                samples.append(sample)
        finally:
            if attached_timing:
                if previous_state is timing_state:
                    delattr(link, "_collect_snapshot_timing")
                else:
                    setattr(
                        link, "_collect_snapshot_timing", previous_state)

        start_times = [
            getattr(sample, "query_start", None)
            for sample in samples
            if getattr(sample, "query_start", None) is not None
        ]
        completion_times = [
            (getattr(sample, "status_reply", None)
             if getattr(sample, "status_reply", None) is not None
             else getattr(sample, "query_reply", None))
            for sample in samples
            if (getattr(sample, "status_reply", None) is not None
                or getattr(sample, "query_reply", None) is not None)
        ]
        if len(start_times) == 6 and len(completion_times) == 6:
            snapshot_age_ms = (
                max(completion_times) - min(start_times)) * 1000.0
            if snapshot_age_ms > FRESH_TELEMETRY_LIMIT_MS:
                stale_reasons.append(
                    f"snapshot age={snapshot_age_ms:.1f} ms")

        if stale_reasons:
            last_stale = ", ".join(stale_reasons)
            continue
        if return_samples:
            return pose, samples
        return pose
    raise RuntimeError(
        "Fresh six-axis telemetry remained stale after "
        f"{FRESH_POSE_READ_ATTEMPTS} complete rereads: {last_stale}")


def require_fresh_pose_close(actual, planned, tolerance_deg, context):
    if len(actual) != 6 or len(planned) != 6:
        raise RuntimeError(f"{context}: expected two six-axis poses")
    deviations = [abs(a - b) for a, b in zip(actual, planned)]
    if max(deviations) > tolerance_deg:
        axis = deviations.index(max(deviations)) + 1
        raise RuntimeError(
            f"{context}: J{axis} differs from the reviewed plan by "
            f"{max(deviations):.4f} deg (limit {tolerance_deg:.4f}); "
            f"actual={actual}, planned={planned}")


def status_fields(link, *, retries=1):
    status = link.request("#GETSTATUS", "oks ", retries=retries)
    return _validate_status_frame(status)


def uses_strict_wrist_safety(config):
    return config.get("safety_profile") == WRIST_STRICT_PROFILE


def acquisition_limits(config):
    return WRIST_STRICT_LIMITS_DEG if uses_strict_wrist_safety(config) else JOINT_LIMITS_DEG


def _valid_sha256(value):
    return isinstance(value, str) and re.fullmatch(r"[0-9a-fA-F]{64}", value) is not None


def validate_config(config, execute=False):
    j = config["joint"]
    if isinstance(j, bool) or j not in range(1, 7):
        raise ValueError("joint must be 1..6")
    pose = config["fixed_pose_deg"]
    if len(pose) != 6 or not all(math.isfinite(x) for x in pose):
        raise ValueError("fixed_pose_deg needs six finite angles")
    limits = acquisition_limits(config)
    for q, (low, high) in zip(pose, limits):
        if not low <= q <= high:
            raise ValueError("Pose is outside the qualified software limits")
    low, high = config["sweep_min_deg"], config["sweep_max_deg"]
    axis_limits = limits[j - 1]
    if not axis_limits[0] <= low < high <= axis_limits[1]:
        raise ValueError("Invalid sweep range outside the qualified software limits")
    if uses_strict_wrist_safety(config):
        gripper_rad = config.get("gripper_rad")
        if (isinstance(gripper_rad, bool) or not isinstance(gripper_rad, (int, float))
                or not math.isfinite(gripper_rad) or not 0 <= gripper_rad <= 0.78):
            raise ValueError("Strict wrist collection requires gripper_rad in 0..0.78 rad")
    elif execute and j in (4, 5):
        raise ValueError(
            "J4/J5 execution requires safety_profile=wrist_strict_v1 with "
            "historical angle limits, current MJCF, floor collision and gripper opening checks; "
            "legacy offline preview remains available")
    for key, minimum, maximum in [("speed_deg_s", 0.3, 5), ("sample_hz", 5, 50),
                                   ("cycles", 1, 20), ("endpoint_hold_s", 0.5, 10)]:
        if not minimum <= config[key] <= maximum:
            raise ValueError(f"Invalid {key}")
    if not isinstance(config["cycles"], int):
        raise ValueError("cycles must be an integer")
    # Optional endpoint stability parameters. A quiet position window is used
    # instead of noisy instantaneous F1 velocity for completion detection.
    for key, default, minimum, maximum in [
        ("endpoint_position_tolerance_deg", 0.3, 0.01, 0.5),
        ("endpoint_position_span_deg", 0.1, 0.005, 0.5),
    ]:
        value = config.get(key, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not minimum <= value <= maximum:
            raise ValueError(f"Invalid {key}: expected {minimum}..{maximum} deg")
    poll_hz = config.get("full_pose_poll_hz", 2.0)
    if (isinstance(poll_hz, bool) or not isinstance(poll_hz, (int, float))
            or not math.isfinite(poll_hz) or not 0.5 <= poll_hz <= 10):
        raise ValueError("Invalid full_pose_poll_hz: expected 0.5..10 Hz")
    current_limit = config.get("current_limit_a")
    if execute and (
            config.get("operator_reviewed_workspace") is not True
            or isinstance(current_limit, bool)
            or not isinstance(current_limit, (int, float))
            or not math.isfinite(current_limit)
            or not 0 < current_limit <= 3.3):
        raise ValueError("Review the physical workspace and fill in the verified F1 current limit")
    if execute and uses_strict_wrist_safety(config) and not _valid_sha256(
            config.get("firmware_hash")):
        raise ValueError(
            "Strict wrist execution requires firmware_hash as the 64-hex SHA-256 "
            "read from the actual F1/F4 build")
    if execute and uses_strict_wrist_safety(config):
        firmware_revision = config.get("firmware_revision")
        if (not isinstance(firmware_revision, str)
                or not firmware_revision.strip()
                or "REPLACE" in firmware_revision.upper()):
            raise ValueError(
                "Strict wrist execution requires firmware_revision from the "
                "actual F1/F4 build")
        current_limit_source = config.get("current_limit_source")
        if (not isinstance(current_limit_source, str)
                or not current_limit_source.strip()
                or "REPLACE" in current_limit_source.upper()
                or "MUST CONFIRM" in current_limit_source.upper()):
            raise ValueError(
                "Strict wrist execution requires current_limit_source with "
                "verified readback evidence")
    return j


def run(link, config, writer, file, safety_evaluator=None, safety_evidence=None):
    """Run acquisition and attempt a fail-closed !STOP on every exception."""
    try:
        return _run(
            link,
            config,
            writer,
            file,
            safety_evaluator=safety_evaluator,
            safety_evidence=safety_evidence,
        )
    except BaseException as error:
        try:
            link.request("!STOP", "Stopped ok")
        except Exception as stop_error:
            raise RuntimeError(
                f"Acquisition failed ({error}); fail-closed !STOP also failed "
                f"({stop_error})"
            ) from error
        raise


def _run(link, config, writer, file, safety_evaluator=None, safety_evidence=None):
    j = config["joint"]
    limits = acquisition_limits(config)
    if uses_strict_wrist_safety(config) and safety_evaluator is None:
        raise RuntimeError("Strict wrist collection requires an MJCF safety evaluator")
    link.request("!STOP", "Stopped ok")
    status = status_fields(link)
    if status[0] != 0:
        raise RuntimeError("Robot remained enabled after !STOP; refusing acquisition")
    if status[2] != 0 or any(value != 0 for value in status[3:]):
        raise RuntimeError("Robot fault: resolve it before collecting; this script never clears faults")
    if status[1] != 2:
        raise RuntimeError("Select position mode 2 before acquisition; switching mode changes acceleration")
    q0 = read_fresh_pose(link)
    for axis in range(6):
        if not limits[axis, 0] <= q0[axis] <= limits[axis, 1]:
            raise RuntimeError(
                f"Actual start J{axis + 1}={q0[axis]:.4f} is outside the qualified limits "
                f"{limits[axis].tolist()}; refusing to move or back off")
    for axis in range(6):
        if axis != j - 1 and abs(q0[axis] - config["fixed_pose_deg"][axis]) > 0.5:
            raise RuntimeError("Other joints are not in the reviewed fixed pose; position them first")
    if not config["sweep_min_deg"] <= q0[j - 1] <= config["sweep_max_deg"]:
        raise RuntimeError("Active joint must start inside the reviewed sweep interval")
    if safety_evaluator is not None:
        ok, tcp, collisions = safety_evaluator.evaluate(q0, limit_margin_deg=0.0)
        if not ok:
            raise RuntimeError(
                f"Unsafe actual start: tcp_mm={tcp.round(2).tolist()}, "
                f"collisions={collisions[:3]}")
        from pose_sweep_planner import verify_path

        waypoints = []
        waypoint = q0.copy()
        waypoint[j - 1] = config["sweep_min_deg"]
        waypoints.append(waypoint.copy())
        for _cycle in range(config["cycles"]):
            waypoint[j - 1] = config["sweep_max_deg"]
            waypoints.append(waypoint.copy())
            waypoint[j - 1] = config["sweep_min_deg"]
            waypoints.append(waypoint.copy())
        path_evidence = verify_path(
            safety_evaluator, q0, waypoints, f"J{j}:reviewed-round-trip")
        print("MJCF_SCAN_PREFLIGHT " + json.dumps(path_evidence), flush=True)
        if safety_evidence is not None:
            safety_evidence.update(path_evidence)
            safety_evidence["segment_preflights"] = []

    # The robot may have changed while MJCF and status checks were running.
    # Re-read all six axes before enabling and refuse a stale start.
    ready_pose = read_fresh_pose(link)
    require_fresh_pose_close(ready_pose, q0, 0.1, "Start pose changed before enable")
    link.request("!START", "Started ok")
    started = time.monotonic()
    segments = [(0, "approach", config["sweep_min_deg"])]
    for cycle in range(1, config["cycles"] + 1):
        segments.extend([(cycle, "forward", config["sweep_max_deg"]),
                         (cycle, "reverse", config["sweep_min_deg"])])
    previous_target = q0[j - 1]
    count = 0
    poll_interval = 1.0 / float(config.get("full_pose_poll_hz", 2.0))
    next_full_pose_poll = started
    queue_start_tolerance = max(
        0.1,
        float(config.get("endpoint_position_tolerance_deg", 0.3)) + 0.05,
    )
    for cycle, phase, target in segments:
        queue_start = read_fresh_pose(link)
        expected_start = q0.copy()
        expected_start[j - 1] = previous_target
        require_fresh_pose_close(
            queue_start, expected_start, queue_start_tolerance, "Segment start pose")
        queue_status = status_fields(link)
        if (queue_status[0] != 1 or queue_status[1] != 2
                or queue_status[2] != 0 or any(queue_status[3:])):
            raise RuntimeError(
                f"Robot is not ready for queue: enabled/mode/faults={queue_status[:3]}")
        if safety_evaluator is not None:
            from pose_sweep_planner import verify_path

            segment_command = queue_start.copy()
            segment_command[j - 1] = target
            try:
                evidence = verify_path(
                    safety_evaluator,
                    queue_start,
                    [segment_command],
                    f"J{j}:{phase}:remeasured-segment",
                )
            except RuntimeError:
                link.request("!STOP", "Stopped ok")
                raise
            if safety_evidence is not None:
                safety_evidence["segment_preflights"].append(evidence)
        q_command = queue_start.copy()
        q_command[j - 1] = target
        command = ">" + ",".join(f"{x:.5f}" for x in [*q_command, config["speed_deg_s"]])
        link.request(command, "queue")
        deadline = (time.monotonic()
                    + abs(target - previous_target) / config["speed_deg_s"] * 2
                    + config["endpoint_hold_s"] + 15)
        tolerance = config.get("endpoint_position_tolerance_deg", 0.3)
        max_span = config.get("endpoint_position_span_deg", 0.1)
        # Endpoint hold uses F4/CAN receive ticks, not host arrival time: a
        # serial retry must not invalidate an already stable endpoint.
        max_tick_gap_ms = 5000
        hold_ms = config["endpoint_hold_s"] * 1000
        position_window = deque()  # (F4 position_tick_ms, joint position)
        arrived_at = None
        last_tick_ms = None
        while True:
            loop_start = time.monotonic()
            now = time.monotonic()
            if now >= next_full_pose_poll:
                q, fresh_samples = read_fresh_pose(link, return_samples=True)
                sample = fresh_samples[j - 1]
                q_active = q[j - 1]
                next_full_pose_poll = now + poll_interval
            else:
                sample, q_active = request_telemetry(link, j)
                all_q = link.request("#GETJPOS", "okj ", retries=1)
                if len(all_q) != 7:
                    raise RuntimeError("Bad position reply")
                q = list(map(float, all_q[1:]))
                q[j - 1] = q_active
            query_start = getattr(sample, "query_start", None)
            query_reply = getattr(sample, "query_reply", None)
            if query_start is None or query_reply is None:
                query_start = loop_start
                query_reply = time.monotonic()
            roundtrip_ms = getattr(
                sample, "roundtrip_ms",
                (query_reply - query_start) * 1000.0,
            )
            status = status_fields(link)
            enabled, mode, robot_fault = status[0], status[1], status[2]
            if not all(math.isfinite(v) for v in q):
                raise RuntimeError("Non-finite position")
            if safety_evaluator is not None:
                ok, tcp, collisions = safety_evaluator.evaluate(
                    q, limit_margin_deg=0.0)
                if not ok:
                    raise RuntimeError(
                        f"Online MJCF check failed: q={[round(value, 4) for value in q]}, "
                        f"tcp_mm={tcp.round(2).tolist()}, collisions={collisions[:3]}")
            sample.update(t_s=(query_start + query_reply) / 2 - started, cycle=cycle,
                          phase="settle" if arrived_at is not None else phase,
                          joint=j, pose_id=config["pose_id"], current_source="command",
                          roundtrip_ms=roundtrip_ms,
                          enabled=enabled, robot_fault_mask=robot_fault)
            sample.update(zip(Q_FIELDS, q))
            writer.writerow(sample)
            file.flush()
            count += 1
            if (enabled != 1 or mode != 2 or robot_fault or sample["fault_flags"] or
                    not sample["status_supported"] or sample["skew_ms"] > 10 or
                    sample["roundtrip_ms"] > FRESH_TELEMETRY_LIMIT_MS):
                raise RuntimeError("Fault, stop, stale or skewed telemetry; partial CSV preserved")
            if abs(sample["i_cmd_motor_a"]) >= 0.9 * config["current_limit_a"]:
                raise RuntimeError("Current command reached 90% of configured limit; reduce load/range")
            if any(abs(q[axis] - q0[axis]) > 0.5 for axis in range(6) if axis != j - 1):
                raise RuntimeError("A held joint moved more than 0.5 degrees")
            if not config["sweep_min_deg"] - 0.5 <= q_active <= config["sweep_max_deg"] + 0.5:
                raise RuntimeError("Active joint left the reviewed interval")
            # Decide endpoint stability using measured *position* and F4/CAN
            # receive ticks. Host-side retries delay arrival but do not mean
            # the robot left the endpoint, so they must not reset the hold.
            tick_ms = int(sample["position_tick_ms"])
            if last_tick_ms is not None:
                tick_gap = tick_delta_ms(last_tick_ms, tick_ms)
                if tick_gap == 0 or tick_gap > max_tick_gap_ms:
                    position_window.clear()  # never bridge missing feedback
                    arrived_at = None
            last_tick_ms = tick_ms

            if abs(q_active - target) >= tolerance:
                position_window.clear()
                arrived_at = None
            else:
                position_window.append((tick_ms, q_active))
                # Drop oldest positions until the full window is quiet.
                while position_window and (max(p for _, p in position_window)
                                           - min(p for _, p in position_window)) >= max_span:
                    position_window.popleft()
                # Require at least two samples so one isolated reading cannot
                # satisfy the hold; use F4 ticks so retries are transparent.
                arrived_at = position_window[0][0] if len(position_window) >= 2 else None
                if (arrived_at is not None
                        and tick_delta_ms(arrived_at, tick_ms) >= hold_ms):
                    held_s = tick_delta_ms(arrived_at, tick_ms) / 1000.0
                    print(f"J{j} {phase}: target={target:.3f} deg, "
                          f"actual={q_active:.4f} deg; stable for "
                          f"{held_s:.2f} s", flush=True)
                    break

            now = time.monotonic()
            if now > deadline:
                span = (max(p for _, p in position_window)
                        - min(p for _, p in position_window)) if position_window else float("nan")
                raise TimeoutError(
                    f"Sweep segment did not finish: J{j} {phase}, "
                    f"target={target:.3f} deg, measured={q_active:.4f} deg, "
                    f"error={target-q_active:+.4f} deg, "
                    f"F1_velocity={sample['dq_deg_s']:+.3f} deg/s, "
                    f"window_span={span:.4f} deg, "
                    f"current_command={sample['i_cmd_motor_a']:+.3f} A"
                )
            time.sleep(max(
                0,
                1 / config["sample_hz"] - (time.monotonic() - loop_start),
            ))
        previous_target = target
        print(f"cycle={cycle} {phase} finished, samples={count}", flush=True)
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--execute", action="store_true", help="Enable and move the real robot")
    parser.add_argument("--port")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--debug-comm", action="store_true",
                        help="Log raw serial lines to <output>.comm.log for diagnosis")
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    j = validate_config(config, args.execute)
    print(f"J{j}: {config['sweep_min_deg']}..{config['sweep_max_deg']} deg, "
          f"{config['speed_deg_s']} deg/s, {config['cycles']} cycles; other joints held.")
    if not args.execute:
        print("OFFLINE PREVIEW: no serial port opened. Review config before --execute.")
        return
    if not args.port or not args.output:
        parser.error("--execute requires --port and --output")
    import serial
    from pose_sweep_planner import (
        DEFAULT_MAPPING,
        DEFAULT_MJCF,
        COLLISION_TOLERANCE_M,
        PoseEvaluator,
        TCP_MIN_X_MM,
        TCP_MIN_Z_MM,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    metadata = args.output.with_suffix(".meta.json")
    comm_path = args.output.with_suffix(".comm.log")
    reserved = [args.output, metadata] + ([comm_path] if args.debug_comm else [])
    occupied = [path for path in reserved if path.exists()]
    if occupied:
        raise FileExistsError(
            "Refusing to overwrite acquisition output: "
            + ", ".join(str(path) for path in occupied))
    outcome = {
        "config": config,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "completed": False,
        "source": "F1 commanded current; asynchronous F4 receive times",
        "safety_model": {
            "mapping": str(DEFAULT_MAPPING.resolve()),
            "mjcf": str(DEFAULT_MJCF.resolve()),
            "mujoco_collision_tolerance_m": COLLISION_TOLERANCE_M,
            "ignored_non_ground_contacts": "adjacent MJCF bodies only",
            "floor_collisions_required": True,
            "tcp_x_min_mm": TCP_MIN_X_MM,
            "tcp_z_min_mm": TCP_MIN_Z_MM,
            "path_step_deg": 1.0,
            "actual_start_and_online_checks": True,
            "full_pose_poll_hz": float(config.get("full_pose_poll_hz", 2.0)),
            "gripper_rad": config.get("gripper_rad"),
            "safety_profile": config.get("safety_profile"),
        },
    }
    safety_evaluator = None
    if uses_strict_wrist_safety(config):
        safety_evaluator = PoseEvaluator(
            DEFAULT_MAPPING, DEFAULT_MJCF, float(config["gripper_rad"]),
            include_floor=True)
    comm_log = comm_path.open("x", encoding="utf-8") if args.debug_comm else None
    metadata_file = metadata.open("x", encoding="utf-8")

    def write_metadata():
        metadata_file.seek(0)
        metadata_file.truncate()
        metadata_file.write(json.dumps(outcome, indent=2, ensure_ascii=False) + "\n")
        metadata_file.flush()

    try:
        with args.output.open("x", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=FIELDS)
            writer.writeheader()
            write_metadata()
            try:
                with serial.Serial(args.port, 115200, timeout=0.05, write_timeout=0.5) as port:
                    link = Link(port, debug=comm_log)
                    acquisition_error = None
                    try:
                        port.reset_input_buffer()
                        safety = {}
                        outcome["scan_preflight"] = safety
                        outcome["samples"] = run(
                            link,
                            config,
                            writer,
                            file,
                            safety_evaluator=safety_evaluator,
                            safety_evidence=safety,
                        )
                        outcome["acquisition_finished"] = True
                    except BaseException as error:
                        acquisition_error = error
                        raise
                    finally:
                        outcome["comm_retries"] = link.retries
                        try:
                            link.request("!STOP", "Stopped ok")
                            outcome["stop_acknowledged"] = True
                            outcome["completed"] = bool(
                                outcome.get("acquisition_finished"))
                        except Exception as error:
                            outcome["stop_acknowledged"] = False
                            outcome["stop_error"] = str(error) or type(error).__name__
                            print(f"STOP not acknowledged: {error}; use the independent stop control.", file=sys.stderr)
                            if acquisition_error is not None:
                                raise RuntimeError(
                                    f"Acquisition failed ({acquisition_error}); "
                                    f"final !STOP also failed ({error})"
                                ) from acquisition_error
                            raise
            except BaseException as error:
                outcome["error"] = str(error) or type(error).__name__
                raise
            finally:
                write_metadata()
    finally:
        metadata_file.close()
        if comm_log is not None:
            comm_log.close()
    if not outcome.get("stop_acknowledged"):
        raise RuntimeError(f"Saved {args.output}, but final !STOP was not acknowledged")
    print(f"Saved {args.output}; F4 acknowledged the !STOP hold request.")


if __name__ == "__main__":
    main()
