import csv
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from calibration import FIELDS
import collect
from collect import (
    CAN_TELEMETRY_TIMEOUTS,
    FRESH_POSE_READ_ATTEMPTS,
    Link,
    _TelemetrySample,
    parse_telemetry,
    read_fresh_pose,
    request_telemetry,
    run,
)


def telemetry_frame(joint=2, tick=100, q_deg=0.0, fault_flags=0,
                    status_supported=1):
    return (
        f"okm {joint} {tick} {tick + 1} {tick + 2} "
        f"{q_deg} 0.0 -0.2 2 {fault_flags} {status_supported}\r\n"
    ).encode("ascii")


def status_frame(enabled=1, mode=2, robot_fault=0, joint_faults=None):
    joint_faults = joint_faults or [0] * 6
    fields = [enabled, mode, robot_fault, *joint_faults]
    return ("oks " + " ".join(map(str, fields)) + "\r\n").encode("ascii")


class ScriptedPort:
    def __init__(self, chunks=(), waiting=b""):
        self.chunks = list(chunks)
        self.waiting = bytearray(waiting)
        self.writes = []

    @property
    def in_waiting(self):
        return len(self.waiting)

    def read(self, size):
        chunk = bytes(self.waiting[:size])
        del self.waiting[:size]
        return chunk

    def read_until(self, delimiter):
        return self.chunks.pop(0) if self.chunks else b""

    def write(self, data):
        self.writes.append(data)
        return len(data)


class AdvancingClock:
    def __init__(self, step=0.2):
        self.now = 0.0
        self.step = step

    def monotonic(self):
        self.now += self.step
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def make_sample(axis, query_start, query_reply, roundtrip_ms=10.0,
                status_reply=None, q_deg=None, tick=None, fault_flags=0):
    tick = 100 * axis if tick is None else tick
    q_deg = float(axis) if q_deg is None else q_deg
    wire = telemetry_frame(
        axis, tick, q_deg, fault_flags).decode("ascii").strip().split()
    values, parsed_q = parse_telemetry(wire, axis)
    return _TelemetrySample(
        values,
        query_start=query_start,
        query_reply=query_reply,
        status_reply=status_reply,
        roundtrip_ms=roundtrip_ms,
    ), parsed_q


class TelemetryFramingTests(unittest.TestCase):
    def test_exact_64_byte_telemetry_is_flushed_by_following_status(self):
        prefix = "okm 2 100 101 102 "
        suffix = " 2.0 -0.5 2 0 1\r\n"
        q_prefix = "0."
        q_zeros = "0" * (64 - len(prefix) - len(q_prefix) - len(suffix))
        telemetry = (prefix + q_prefix + q_zeros + suffix).encode("ascii")
        self.assertEqual(len(telemetry), 64)

        class PairReleasingPort(ScriptedPort):
            def __init__(self):
                super().__init__()
                self.blocked_telemetry = None
                self.released = False

            def write(self, data):
                self.writes.append(data)
                for command in data.decode("ascii").splitlines():
                    if command == "#GETTELEM 2":
                        self.blocked_telemetry = telemetry
                    elif command == "#GETSTATUS" and self.blocked_telemetry:
                        self.chunks.extend([
                            self.blocked_telemetry,
                            status_frame(),
                        ])
                        self.blocked_telemetry = None
                        self.released = True
                return len(data)

        port = PairReleasingPort()
        link = Link(port)

        sample, q_deg = request_telemetry(link, 2)

        self.assertEqual(port.writes, [b"#GETTELEM 2\n#GETSTATUS\n"])
        self.assertTrue(port.released)
        self.assertEqual(port.chunks, [])
        self.assertEqual(q_deg, 0.0)
        self.assertEqual(sample["position_tick_ms"], 100)
        self.assertEqual(sample["velocity_tick_ms"], 101)
        self.assertEqual(sample["current_tick_ms"], 102)
        self.assertEqual(link.retries, 0)

    def test_silent_transaction_locks_link_except_for_stop(self):
        port = ScriptedPort()
        link = Link(port)
        clock = AdvancingClock()
        with patch.object(collect.time, "monotonic", clock.monotonic):
            with self.assertRaisesRegex(TimeoutError, "one deadline"):
                request_telemetry(link, 2)

        self.assertEqual(port.writes, [b"#GETTELEM 2\n#GETSTATUS\n"])
        with self.assertRaisesRegex(RuntimeError, "only !STOP"):
            link.request("#GETSTATUS", "oks ")
        self.assertEqual(len(port.writes), 1)

        port.chunks.append(b"Stopped ok\r\n")
        with patch.object(collect.time, "monotonic", lambda: 0.0):
            self.assertEqual(
                link.request("!STOP", "Stopped ok", timeout=0.01),
                ["Stopped", "ok"],
            )

    def test_blocking_read_after_transaction_deadline_is_rejected(self):
        class LatePort(ScriptedPort):
            def read_until(self, delimiter):
                clock.now = 2.0
                return status_frame() + status_frame()

        clock = AdvancingClock(step=0.0)
        port = LatePort()
        link = Link(port)
        with patch.object(collect.time, "monotonic", clock.monotonic):
            with self.assertRaisesRegex(TimeoutError, "after its deadline"):
                link.telemetry_transaction(2, timeout=1.0)

        self.assertTrue(link._desynchronized)

    def test_wrong_order_wrong_axis_and_extra_frames_abort(self):
        cases = [
            (
                "status first",
                status_frame() + telemetry_frame(),
                "status before telemetry",
            ),
            (
                "wrong axis",
                telemetry_frame(joint=1) + status_frame(),
                "Unexpected telemetry frame",
            ),
            (
                "two telemetry frames",
                telemetry_frame() + telemetry_frame() + status_frame(),
                "not followed by one status",
            ),
            (
                "extra status",
                telemetry_frame() + status_frame() + status_frame(),
                "Extra serial frame",
            ),
        ]
        for name, response, message in cases:
            with self.subTest(name=name):
                link = Link(ScriptedPort([response]))
                with self.assertRaisesRegex(RuntimeError, message):
                    request_telemetry(link, 2)
                self.assertTrue(link._desynchronized)

    def test_waiting_extra_status_or_partial_frame_aborts_transaction(self):
        class WaitingAfterStatusPort(ScriptedPort):
            def __init__(self, waiting):
                super().__init__([telemetry_frame()])
                self.extra = waiting
                self.read_count = 0

            def read_until(self, delimiter):
                self.read_count += 1
                if self.read_count == 1:
                    return telemetry_frame()
                self.waiting.extend(self.extra)
                return status_frame()

        for waiting, message in [
            (status_frame(), "Extra serial frame"),
            (b"ok", "Trailing partial serial frame"),
        ]:
            with self.subTest(waiting=waiting):
                link = Link(WaitingAfterStatusPort(waiting))
                with self.assertRaisesRegex(RuntimeError, message):
                    request_telemetry(link, 2)
                self.assertTrue(link._desynchronized)

    def test_extra_frame_arriving_after_pair_blocks_next_query_write(self):
        for extra, message in [
            (status_frame(), "Extra serial frame"),
            (b"oks 1 2", "Trailing partial serial frame"),
        ]:
            with self.subTest(extra=extra):
                port = ScriptedPort([
                    telemetry_frame() + status_frame(),
                ])
                link = Link(port)
                request_telemetry(link, 2)
                self.assertEqual(len(port.writes), 1)
                port.waiting.extend(extra)

                with self.assertRaisesRegex(RuntimeError, message):
                    link.request("#GETSTATUS", "oks ")
                self.assertEqual(len(port.writes), 1)
                self.assertTrue(link._desynchronized)

    def test_can_timeout_retries_only_after_complete_pair(self):
        timeout = next(iter(CAN_TELEMETRY_TIMEOUTS))
        port = ScriptedPort([
            timeout.encode("ascii") + b"\r\n" + status_frame(),
            telemetry_frame(tick=700) + status_frame(),
        ])
        link = Link(port)

        sample, q_deg = request_telemetry(link, 2)

        self.assertEqual(q_deg, 0.0)
        self.assertEqual(sample["position_tick_ms"], 700)
        self.assertEqual(
            port.writes,
            [
                b"#GETTELEM 2\n#GETSTATUS\n",
                b"#GETTELEM 2\n#GETSTATUS\n",
            ],
        )
        self.assertEqual(link.retries, 1)

    def test_incomplete_or_inexact_can_timeout_does_not_retry(self):
        timeout = next(iter(CAN_TELEMETRY_TIMEOUTS))
        clock = AdvancingClock()
        port = ScriptedPort([timeout.encode("ascii") + b"\r\n"])
        link = Link(port)
        with patch.object(collect.time, "monotonic", clock.monotonic):
            with self.assertRaises(TimeoutError):
                request_telemetry(link, 2)
        self.assertEqual(len(port.writes), 1)
        self.assertEqual(link.retries, 0)

        port = ScriptedPort([
            b"error F1 telemetry timeoutXYZ\r\n" + status_frame(),
            telemetry_frame() + status_frame(),
        ])
        link = Link(port)
        with self.assertRaisesRegex(RuntimeError, "timeoutXYZ"):
            request_telemetry(link, 2)
        self.assertEqual(len(port.writes), 1)
        self.assertEqual(link.retries, 0)

    def test_queued_motion_ok_is_allowed_but_rejection_is_not_swallowed(self):
        port = ScriptedPort([
            telemetry_frame(),
            b"ok\r\n" + status_frame(),
        ])
        link = Link(port)
        link._pending_motion_acks = 1
        request_telemetry(link, 2)
        self.assertEqual(link._pending_motion_acks, 0)

        port = ScriptedPort([
            b"error motion rejected or interrupted\r\n" + status_frame(),
        ])
        link = Link(port)
        link._pending_motion_acks = 1
        with self.assertRaisesRegex(RuntimeError, "motion rejected or interrupted"):
            request_telemetry(link, 2)
        self.assertTrue(link._desynchronized)
        self.assertEqual(link._pending_motion_acks, 1)

    def test_fault_and_unsafe_status_abort_fresh_pose(self):
        port = ScriptedPort([
            telemetry_frame(joint=1, fault_flags=1) + status_frame(),
        ])
        with self.assertRaisesRegex(RuntimeError, "invalid fresh telemetry"):
            read_fresh_pose(Link(port))

        port = ScriptedPort([
            telemetry_frame(joint=1) + status_frame(robot_fault=1),
        ])
        with self.assertRaisesRegex(RuntimeError, "paired telemetry status"):
            read_fresh_pose(Link(port))

    def test_paired_status_allows_preflight_but_not_unsafe_online_reply(self):
        preflight_port = ScriptedPort([
            telemetry_frame() + status_frame(enabled=0),
        ])
        request_telemetry(Link(preflight_port), 2)

        start_port = ScriptedPort([b"Started ok\r\n"])
        start_link = Link(start_port)
        start_link.request("!START", "Started ok")
        self.assertTrue(start_link._motion_enabled)

        cases = [
            (
                "disabled",
                status_frame(enabled=0),
                "disabled during online",
                False,
            ),
            (
                "wrong mode",
                status_frame(enabled=1, mode=1),
                "enabled/mode/faults",
                False,
            ),
            (
                "robot fault",
                status_frame(enabled=1, robot_fault=1),
                "enabled/mode/faults",
                False,
            ),
            (
                "joint fault",
                status_frame(enabled=1, joint_faults=[0, 0, 1, 0, 0, 0]),
                "joint_faults",
                False,
            ),
            (
                "faulted transient",
                status_frame(enabled=1, robot_fault=1),
                "enabled/mode/faults",
                True,
            ),
        ]
        for name, status, message, transient in cases:
            with self.subTest(name=name):
                timeout = next(iter(CAN_TELEMETRY_TIMEOUTS))
                first = (
                    timeout.encode("ascii") + b"\r\n" + status
                    if transient
                    else telemetry_frame() + status
                )
                port = ScriptedPort([
                    first,
                    telemetry_frame() + status_frame(),
                ])
                link = Link(port)
                link._motion_enabled = True
                with self.assertRaisesRegex(RuntimeError, message):
                    request_telemetry(link, 2)
                self.assertEqual(len(port.writes), 1)
                self.assertTrue(link._desynchronized)

    def test_snapshot_age_includes_first_start_and_last_status(self):
        first = {}
        for axis in range(1, 7):
            start = (axis - 1) * 0.01
            reply = start + 0.02
            status_reply = reply + 0.01
            if axis == 6:
                start = 0.08
                reply = 0.09
                status_reply = 0.25
            first[axis] = make_sample(
                axis, start, reply, roundtrip_ms=20.0,
                status_reply=status_reply,
            )[0]
        second = {
            axis: make_sample(
                axis, 1.0 + axis * 0.01, 1.02 + axis * 0.01,
                roundtrip_ms=20.0,
                status_reply=1.03 + axis * 0.01,
            )[0]
            for axis in range(1, 7)
        }
        calls = 0

        def request_axis(_link, axis):
            nonlocal calls
            calls += 1
            snapshot = first if calls <= 6 else second
            return snapshot[axis], float(axis)

        with patch.object(collect, "request_telemetry", side_effect=request_axis):
            pose = read_fresh_pose(object())

        self.assertEqual(calls, 12)
        self.assertEqual(pose, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

    def test_single_request_latency_forces_complete_snapshot_reread(self):
        first = {
            axis: make_sample(
                axis, 0.0 + axis * 0.001, 0.02 + axis * 0.001,
                roundtrip_ms=101.0 if axis == 2 else 20.0,
                status_reply=0.03 + axis * 0.001,
            )[0]
            for axis in range(1, 7)
        }
        second = {
            axis: make_sample(
                axis, 1.0 + axis * 0.001, 1.02 + axis * 0.001,
                roundtrip_ms=20.0,
                status_reply=1.03 + axis * 0.001,
            )[0]
            for axis in range(1, 7)
        }
        calls = 0

        def request_axis(_link, axis):
            nonlocal calls
            calls += 1
            snapshot = first if calls <= 6 else second
            return snapshot[axis], float(axis)

        with patch.object(collect, "request_telemetry", side_effect=request_axis):
            pose = read_fresh_pose(object())

        self.assertEqual(calls, 12)
        self.assertEqual(pose, [float(axis) for axis in range(1, 7)])

    def test_stale_group_never_passes(self):
        stale = {
            axis: make_sample(
                axis, axis * 0.001, axis * 0.001 + 0.001,
                roundtrip_ms=20.0,
                status_reply=0.3 + axis * 0.001,
            )[0]
            for axis in range(1, 7)
        }
        calls = 0

        def request_axis(_link, axis):
            nonlocal calls
            calls += 1
            return stale[axis], float(axis)

        with patch.object(collect, "request_telemetry", side_effect=request_axis):
            with self.assertRaisesRegex(RuntimeError, "remained stale"):
                read_fresh_pose(object())
        self.assertEqual(calls, 6 * FRESH_POSE_READ_ATTEMPTS)

    def test_fallback_link_timing_fields_keep_100ms_validation(self):
        class TimedLink:
            def __init__(self):
                self.calls = 0
                self.last_attempt_start = None
                self.last_reply_time = None
                self.last_roundtrip_ms = None

            def request(self, command, prefix, **_kwargs):
                self.calls += 1
                axis = int(command.split()[1])
                self.last_attempt_start = 10.0 + self.calls * 0.001
                self.last_reply_time = self.last_attempt_start + 0.11
                self.last_roundtrip_ms = 110.0
                return telemetry_frame(axis).decode("ascii").strip().split()

        link = TimedLink()
        with self.assertRaisesRegex(RuntimeError, "remained stale"):
            read_fresh_pose(link)
        self.assertEqual(link.calls, 6 * FRESH_POSE_READ_ATTEMPTS)

    def test_fallback_roundtrip_only_is_validated(self):
        class RoundtripOnlyLink:
            def __init__(self):
                self.calls = 0
                self.last_roundtrip_ms = 150.0

            def request(self, command, prefix, **_kwargs):
                del prefix
                self.calls += 1
                axis = int(command.split()[1])
                return telemetry_frame(
                    axis, q_deg=float(axis)).decode("ascii").strip().split()

        link = RoundtripOnlyLink()
        with self.assertRaisesRegex(RuntimeError, "request latency=150.0"):
            read_fresh_pose(link)
        self.assertEqual(link.calls, 6 * FRESH_POSE_READ_ATTEMPTS)

    def test_fallback_without_timing_uses_one_clock_sequence(self):
        class UntimedLink:
            def __init__(self):
                self.calls = 0

            def request(self, command, prefix, **_kwargs):
                del prefix
                self.calls += 1
                axis = int(command.split()[1])
                return telemetry_frame(
                    axis, q_deg=float(axis)).decode("ascii").strip().split()

        link = UntimedLink()
        clock = AdvancingClock(step=0.01)
        with patch.object(collect.time, "monotonic", clock.monotonic):
            pose = read_fresh_pose(link)
        self.assertEqual(link.calls, 6)
        self.assertEqual(pose, [float(axis) for axis in range(1, 7)])

        link = UntimedLink()
        clock = AdvancingClock(step=0.03)
        with patch.object(collect.time, "monotonic", clock.monotonic):
            with self.assertRaisesRegex(RuntimeError, "snapshot age"):
                read_fresh_pose(link)
        self.assertEqual(link.calls, 6 * FRESH_POSE_READ_ATTEMPTS)

    def test_active_axis_csv_time_is_not_overwritten_by_getjpos(self):
        class Clock:
            def __init__(self):
                self.now = 0.0

            def monotonic(self):
                self.now += 0.01
                return self.now

            def sleep(self, seconds):
                self.now += seconds

        class Robot:
            def __init__(self, clock):
                self.clock = clock
                self.enabled = 0
                self.commands = []
                self.q = [0.0, -1.0, 90.0, 0.0, 0.0, 0.0]
                self.start_command_at = None

            def request(self, command, prefix, timeout=0.5, retries=0,
                        joint=None):
                del prefix, timeout, retries, joint
                self.commands.append(command)
                if command == "!STOP":
                    self.enabled = 0
                    return ["Stopped", "ok"]
                if command == "!START":
                    self.enabled = 1
                    self.start_command_at = self.clock.now
                    return ["Started", "ok"]
                if command == "#GETSTATUS":
                    return status_frame(
                        enabled=self.enabled).decode("ascii").strip().split()
                if command.startswith(">"):
                    return ["15"]
                if command == "#GETJPOS":
                    self.last_attempt_start = 1000.0
                    self.last_reply_time = 1000.2
                    self.last_roundtrip_ms = 200.0
                    return ["okj", *map(str, self.q)]
                raise AssertionError(command)

        clock = Clock()
        robot = Robot(clock)
        start_pose = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
        full_samples = [
            make_sample(
                axis, 5.0 + axis * 0.01, 5.02 + axis * 0.01,
                roundtrip_ms=30.0 + axis * 7.0,
                status_reply=5.03 + axis * 0.01,
                q_deg=start_pose[axis - 1],
                tick=100,
            )[0]
            for axis in range(1, 7)
        ]
        active_samples = []

        def fresh_pose(_link, return_samples=False):
            if return_samples:
                return start_pose.copy(), full_samples
            return start_pose.copy()

        def active_sample(_link, axis):
            tick = 700 + len(active_samples) * 600
            sample, q_deg = make_sample(
                axis, 10.0 + len(active_samples) * 0.1,
                10.04 + len(active_samples) * 0.1,
                roundtrip_ms=40.0,
                status_reply=10.05 + len(active_samples) * 0.1,
                q_deg=-1.0,
                tick=tick,
            )
            active_samples.append(sample)
            return sample, q_deg

        config = {
            "joint": 2,
            "fixed_pose_deg": start_pose,
            "sweep_min_deg": -1.0,
            "sweep_max_deg": 1.0,
            "speed_deg_s": 5.0,
            "sample_hz": 50,
            "cycles": 0,
            "endpoint_hold_s": 0.5,
            "endpoint_position_tolerance_deg": 0.3,
            "endpoint_position_span_deg": 0.1,
            "full_pose_poll_hz": 0.5,
            "current_limit_a": 2.0,
            "pose_id": "timing-test",
        }
        file = io.StringIO()
        writer = csv.DictWriter(file, fieldnames=FIELDS)
        writer.writeheader()

        with patch.object(collect.time, "monotonic", clock.monotonic), \
                patch.object(collect.time, "sleep", clock.sleep), \
                patch.object(collect, "read_fresh_pose", side_effect=fresh_pose), \
                patch.object(collect, "request_telemetry", side_effect=active_sample):
            count = run(robot, config, writer, file)

        self.assertGreaterEqual(count, 3)
        self.assertGreaterEqual(
            robot.commands.count("#GETJPOS"), 1)
        rows = list(csv.DictReader(io.StringIO(file.getvalue())))
        first_row = rows[0]
        last_row = rows[-1]
        last_sample = active_samples[-1]
        j2_sample = full_samples[1]
        j6_sample = full_samples[5]
        started = robot.start_command_at + 0.01
        j2_midpoint = (j2_sample.query_start + j2_sample.query_reply) / 2
        j6_midpoint = (j6_sample.query_start + j6_sample.query_reply) / 2
        self.assertEqual(float(first_row["roundtrip_ms"]), 44.0)
        self.assertAlmostEqual(
            float(first_row["t_s"]), j2_midpoint - started, places=2)
        self.assertNotAlmostEqual(
            float(first_row["t_s"]), j6_midpoint - started, places=2)
        self.assertEqual(float(last_row["roundtrip_ms"]), 40.0)
        self.assertLess(abs(float(last_row["t_s"])), 100.0)
        self.assertNotAlmostEqual(float(last_row["t_s"]), 990.0, places=3)
        self.assertEqual(int(last_row["position_tick_ms"]), 1300)
        self.assertEqual(int(last_row["velocity_tick_ms"]), 1301)
        self.assertEqual(int(last_row["current_tick_ms"]), 1302)
        self.assertAlmostEqual(
            float(last_row["t_s"]),
            (last_sample.query_start + last_sample.query_reply) / 2
            - started,
            places=2,
        )

    def test_sampling_loop_uses_latest_status_after_full_pose(self):
        class Robot:
            def __init__(self):
                self.commands = []

            def request(self, command, prefix, **_kwargs):
                del prefix
                self.commands.append(command)
                if command == "!STOP":
                    return ["Stopped", "ok"]
                if command == "!START":
                    return ["Started", "ok"]
                if command.startswith(">"):
                    return ["15"]
                raise AssertionError(command)

        start_pose = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
        samples = [
            make_sample(
                axis, 5.0 + axis * 0.01, 5.02 + axis * 0.01,
                roundtrip_ms=20.0,
                status_reply=5.03 + axis * 0.01,
                q_deg=start_pose[axis - 1],
            )[0]
            for axis in range(1, 7)
        ]

        def fresh_pose(_link, return_samples=False):
            if return_samples:
                return start_pose.copy(), samples
            return start_pose.copy()

        statuses = iter([
            [0, 2, 0, 0, 0, 0, 0, 0, 0],
            [1, 2, 0, 0, 0, 0, 0, 0, 0],
            [0, 2, 0, 0, 0, 0, 0, 0, 0],
        ])
        config = {
            "joint": 2,
            "fixed_pose_deg": start_pose,
            "sweep_min_deg": -1.0,
            "sweep_max_deg": 1.0,
            "speed_deg_s": 5.0,
            "sample_hz": 50,
            "cycles": 0,
            "endpoint_hold_s": 0.5,
            "endpoint_position_tolerance_deg": 0.3,
            "endpoint_position_span_deg": 0.1,
            "full_pose_poll_hz": 0.5,
            "current_limit_a": 2.0,
            "pose_id": "status-test",
        }
        file = io.StringIO()
        writer = csv.DictWriter(file, fieldnames=FIELDS)
        writer.writeheader()
        robot = Robot()
        with patch.object(collect, "read_fresh_pose", side_effect=fresh_pose), \
                patch.object(collect, "status_fields", side_effect=statuses), \
                patch.object(
                    collect, "request_telemetry",
                    side_effect=AssertionError("status must be checked first")):
            with self.assertRaisesRegex(RuntimeError, "Fault, stop, stale"):
                run(robot, config, writer, file)

        rows = list(csv.DictReader(io.StringIO(file.getvalue())))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["enabled"], "0")
        self.assertEqual(robot.commands[-1], "!STOP")

    def test_run_attempts_stop_when_query_rejected(self):
        class StopTrackingLink:
            def __init__(self):
                self.commands = []

            def request(self, command, prefix, **_kwargs):
                self.commands.append(command)
                if command == "!STOP":
                    return ["Stopped", "ok"]
                if command == "#GETSTATUS":
                    raise RuntimeError("status rejected")
                raise AssertionError(command)

        link = StopTrackingLink()
        with self.assertRaisesRegex(RuntimeError, "status rejected"):
            run(link, {"joint": 2}, None, None)
        self.assertEqual(link.commands.count("!STOP"), 2)
        self.assertEqual(link.commands[-1], "!STOP")


if __name__ == "__main__":
    unittest.main()
