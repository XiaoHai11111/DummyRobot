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
import sys
import time

from calibration import (FIELDS, JOINT_LIMITS_DEG, Q_FIELDS, tick_delta_ms,
                         tick_skew_ms)


class Link:
    def __init__(self, port, debug=None):
        self.port = port
        self.debug = debug
        self._rx = bytearray()
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
                    raise RuntimeError(text)
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


def validate_config(config, execute=False):
    j = config["joint"]
    if isinstance(j, bool) or j not in range(1, 7):
        raise ValueError("joint must be 1..6")
    pose = config["fixed_pose_deg"]
    if len(pose) != 6 or not all(math.isfinite(x) for x in pose):
        raise ValueError("fixed_pose_deg needs six finite angles")
    for q, (low, high) in zip(pose, JOINT_LIMITS_DEG):
        if not low + 2 <= q <= high - 2:
            raise ValueError("Pose must retain at least a 2 degree software-limit margin")
    low, high = config["sweep_min_deg"], config["sweep_max_deg"]
    limits = JOINT_LIMITS_DEG[j - 1]
    if not limits[0] + 2 <= low < high <= limits[1] - 2:
        raise ValueError("Invalid sweep range / insufficient limit margin")
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
    if execute and (config.get("operator_reviewed_workspace") is not True or
                    config.get("current_limit_a") is None or
                    not 0 < config["current_limit_a"] <= 3.3):
        raise ValueError("Review the physical workspace and fill in the verified F1 current limit")
    return j


def run(link, config, writer, file):
    j = config["joint"]
    link.request("!STOP", "Stopped ok")
    status = link.request("#GETSTATUS", "oks ", retries=1)
    if len(status) != 10 or int(status[3]) != 0:
        raise RuntimeError("Robot fault: resolve it before collecting; this script never clears faults")
    if int(status[2]) != 2:
        raise RuntimeError("Select position mode 2 before acquisition; switching mode changes acceleration")
    q0 = []
    for axis in range(1, 7):
        sample, q = parse_telemetry(
            link.request(f"#GETTELEM {axis}", "okm ", timeout=1.0, retries=2, joint=axis), axis)
        if sample["fault_flags"] or not sample["status_supported"] or sample["skew_ms"] > 10:
            raise RuntimeError(f"J{axis}: invalid initial telemetry")
        q0.append(q)
    for axis in range(6):
        if axis != j - 1 and abs(q0[axis] - config["fixed_pose_deg"][axis]) > 0.5:
            raise RuntimeError("Other joints are not in the reviewed fixed pose; position them first")
    if not config["sweep_min_deg"] <= q0[j - 1] <= config["sweep_max_deg"]:
        raise RuntimeError("Active joint must start inside the reviewed sweep interval")
    link.request("!START", "Started ok")
    started = time.monotonic()
    segments = [(0, "approach", config["sweep_min_deg"])]
    for cycle in range(1, config["cycles"] + 1):
        segments.extend([(cycle, "forward", config["sweep_max_deg"]),
                         (cycle, "reverse", config["sweep_min_deg"])])
    previous_target = q0[j - 1]
    count = 0
    for cycle, phase, target in segments:
        q_command = q0.copy()
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
            before = time.monotonic()
            wire = link.request(f"#GETTELEM {j}", "okm ", timeout=1.0, retries=2, joint=j)
            after = getattr(link, "last_reply_time", None) or time.monotonic()
            if getattr(link, "last_attempt_start", None) is not None:
                before = link.last_attempt_start
            roundtrip_ms = getattr(link, "last_roundtrip_ms", (after - before) * 1000)
            sample, q_active = parse_telemetry(wire, j)
            all_q = link.request("#GETJPOS", "okj ", retries=1)
            status = link.request("#GETSTATUS", "oks ", retries=1)
            if len(all_q) != 7 or len(status) != 10:
                raise RuntimeError("Bad position/status reply")
            q = list(map(float, all_q[1:]))
            q[j - 1] = q_active
            enabled, robot_fault = int(status[1]), int(status[3])
            if not all(math.isfinite(v) for v in q):
                raise RuntimeError("Non-finite position")
            sample.update(t_s=(before + after) / 2 - started, cycle=cycle,
                          phase="settle" if arrived_at is not None else phase,
                          joint=j, pose_id=config["pose_id"], current_source="command",
                          roundtrip_ms=roundtrip_ms,
                          enabled=enabled, robot_fault_mask=robot_fault)
            sample.update(zip(Q_FIELDS, q))
            writer.writerow(sample)
            file.flush()
            count += 1
            if (not enabled or robot_fault or sample["fault_flags"] or
                    not sample["status_supported"] or sample["skew_ms"] > 10 or
                    sample["roundtrip_ms"] > 100):
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
            time.sleep(max(0, 1 / config["sample_hz"] - (time.monotonic() - before)))
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

    args.output.parent.mkdir(parents=True, exist_ok=True)
    metadata = args.output.with_suffix(".meta.json")
    if metadata.exists():
        raise FileExistsError(metadata)
    outcome = {"config": config, "started_utc": datetime.now(timezone.utc).isoformat(),
               "completed": False, "source": "F1 commanded current; asynchronous F4 receive times"}
    comm_log = (args.output.with_suffix(".comm.log").open("w", encoding="utf-8")
                if args.debug_comm else None)
    with args.output.open("x", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=FIELDS)
        writer.writeheader()
        metadata.write_text(json.dumps(outcome, indent=2, ensure_ascii=False) + "\n")
        try:
            with serial.Serial(args.port, 115200, timeout=0.05, write_timeout=0.5) as port:
                link = Link(port, debug=comm_log)
                port.reset_input_buffer()
                try:
                    outcome["samples"] = run(link, config, writer, file)
                    outcome["completed"] = True
                finally:
                    outcome["comm_retries"] = link.retries
                    try:
                        link.request("!STOP", "Stopped ok")
                        outcome["stop_acknowledged"] = True
                    except Exception as error:
                        outcome["stop_acknowledged"] = False
                        print(f"STOP not acknowledged: {error}; use the independent stop control.", file=sys.stderr)
        except BaseException as error:
            outcome["error"] = str(error) or type(error).__name__
            raise
        finally:
            metadata.write_text(json.dumps(outcome, indent=2, ensure_ascii=False) + "\n")
    if comm_log is not None:
        comm_log.close()
    if not outcome.get("stop_acknowledged"):
        raise RuntimeError(f"Saved {args.output}, but final !STOP was not acknowledged")
    print(f"Saved {args.output}; F4 acknowledged the !STOP hold request.")


if __name__ == "__main__":
    main()