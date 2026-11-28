import csv
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import collect
import wrist_drag_trial
from collect import Link, _TelemetrySample, read_fresh_pose, request_telemetry
from wrist_drag_trial import (
    CSV_FIELDS,
    CapturedSample,
    TrialSettings,
    execute_trial,
    parse_drag_readback,
    parse_wrist_readback,
    preflight_wrist_corridor,
    query_configured_readbacks,
    run_trial,
    stop_and_restore_mode_2,
    summarize_release_drift,
)


def status_fields(enabled=1, mode=5, robot_fault=0, joint_faults=None):
    joint_faults = joint_faults or [0] * 6
    return [enabled, mode, robot_fault, *joint_faults]


def telemetry_frame(joint=2, tick=100, q_deg=0.0):
    return (
        f"okm {joint} {tick} {tick + 1} {tick + 2} "
        f"{q_deg} 0.0 -0.2 2 0 1\r\n"
    ).encode("ascii")


def status_frame(enabled=1, mode=5, joint_faults=None):
    fields = [enabled, mode, 0, *(joint_faults or [0] * 6)]
    return ("oks " + " ".join(map(str, fields)) + "\r\n").encode("ascii")


class ScriptedPort:
    def __init__(self, chunks=()):
        self.chunks = list(chunks)
        self.writes = []

    @property
    def in_waiting(self):
        return 0

    def read_until(self, _delimiter):
        return self.chunks.pop(0) if self.chunks else b""

    def write(self, data):
        self.writes.append(data)
        return len(data)


def make_settings(**overrides):
    values = {
        "reference_deg": (-90.0, -16.0, 110.0, 0.0, 0.0, 0.0),
        "j4_interval_deg": (-10.0, 10.0),
        "j5_interval_deg": (-10.0, 10.0),
        "other_deviation_deg": 1.0,
        "duration_s": 30.0,
        "sample_hz": 10.0,
        "max_telemetry_gap_ms": 350.0,
        "start_tolerance_deg": 0.15,
        "gripper_rad": 0.0,
        "corridor_step_deg": 2.0,
    }
    values.update(overrides)
    return TrialSettings(**values)


def make_telemetry(tick, q_deg):
    samples = []
    for axis in range(1, 7):
        values = {
            "position_tick_ms": tick,
            "velocity_tick_ms": tick + 1,
            "current_tick_ms": tick + 2,
            "skew_ms": 2,
            "dq_deg_s": 0.0,
            "i_cmd_motor_a": 0.1,
            "driver_state": 2,
            "fault_flags": 0,
            "status_supported": 1,
        }
        samples.append(_TelemetrySample(
            values,
            query_start=tick / 1000.0,
            query_reply=tick / 1000.0 + 0.01,
            roundtrip_ms=10.0,
            status_reply=tick / 1000.0 + 0.02,
        ))
    return samples


class SafeEvaluator:
    collision_tolerance_m = 1e-9

    @staticmethod
    def evaluate(_q, limit_margin_deg=0.0):
        del limit_margin_deg
        return True, np.array([0.0, 0.0, 100.0]), []


class ModeRobotLink:
    _desynchronized = False
    _motion_enabled = False

    def __init__(self):
        self.mode = 2
        self.enabled = 0
        self.commands = []

    def request(self, command, _prefix, **_kwargs):
        self.commands.append(command)
        if command == "!STOP":
            self.enabled = 0
            return ["Stopped", "ok"]
        if command == "!START":
            self.enabled = 1
            return ["Started", "ok"]
        if command == "#CMDMODE 5":
            self.mode = 5
            return ["Set", "command", "mode", "to", "[5]"]
        if command == "#CMDMODE 2":
            self.mode = 2
            return ["Set", "command", "mode", "to", "[2]"]
        if command == "#GETSTATUS":
            return ["oks"] + [
                int(value) for value in status_fields(self.enabled, self.mode)]
        raise AssertionError(command)


class WristDragTrialTests(unittest.TestCase):
    def test_wrist_can_timeout_retries_only_with_healthy_enabled_status(self):
        error = RuntimeError("error wrist inquiry timeout (requires new F1 protocol)")
        wrist = ("okw 5 g0 gm1,0,1 f0 fs0 s1 flags3 "
                 "dir0 current0 seq42 harm0,0,0,0").split()
        link = Mock(retries=0)
        link.request.side_effect = [error, ["oks", *status_fields()], wrist]
        self.assertEqual(wrist_drag_trial._query_wrist_with_timing(link, 5)["joint"], 5)
        self.assertEqual(link.retries, 1)
        for status in (status_fields(enabled=0), status_fields(robot_fault=1)):
            link.request.side_effect = [error, ["oks", *status]]
            with self.assertRaises(RuntimeError):
                wrist_drag_trial._query_wrist_with_timing(link, 5)
        link.request.side_effect = TimeoutError("silent USB transaction")
        with self.assertRaises(TimeoutError):
            wrist_drag_trial._query_wrist_with_timing(link, 5)

    def test_mode_5_must_be_explicit_and_still_requires_safe_status(self):
        port = ScriptedPort([
            telemetry_frame(joint=2, tick=100, q_deg=0.0)
            + status_frame(enabled=1, mode=5),
        ])
        link = Link(port)
        link._motion_enabled = True
        sample, q_deg = request_telemetry(link, 2, expected_mode=5)
        self.assertEqual(q_deg, 0.0)
        self.assertEqual(sample["i_cmd_motor_a"], -0.2)
        self.assertGreater(sample.roundtrip_ms, 0)

        port = ScriptedPort([
            telemetry_frame(joint=2, tick=100, q_deg=0.0)
            + status_frame(enabled=1, mode=5),
        ])
        link = Link(port)
        link._motion_enabled = True
        with self.assertRaisesRegex(RuntimeError, "expected mode 2"):
            request_telemetry(link, 2)
        self.assertTrue(link._desynchronized)

        for enabled, joint_faults, message in [
            (0, None, "disabled"),
            (1, [0, 0, 0, 0, 1, 0], "joint_faults"),
        ]:
            port = ScriptedPort([
                telemetry_frame(joint=2, tick=100, q_deg=0.0)
                + status_frame(
                    enabled=enabled, mode=5, joint_faults=joint_faults),
            ])
            link = Link(port)
            link._motion_enabled = True
            with self.subTest(message=message):
                with self.assertRaisesRegex(RuntimeError, message):
                    request_telemetry(link, 2, expected_mode=5)

    def test_read_fresh_pose_uses_real_link_and_roundtrip_attribute(self):
        chunks = []
        for joint in range(1, 7):
            chunks.append(
                telemetry_frame(joint=joint, tick=100 + joint, q_deg=0.0)
                + status_frame(enabled=1, mode=5))
        link = Link(ScriptedPort(chunks))
        link._motion_enabled = True
        pose, samples = read_fresh_pose(
            link, return_samples=True, expected_mode=5)
        self.assertEqual(len(pose), 6)
        self.assertEqual(len(samples), 6)
        for sample in samples:
            self.assertIsInstance(sample, _TelemetrySample)
            self.assertGreater(sample.roundtrip_ms, 0)
            with self.assertRaises(KeyError):
                _ = sample["roundtrip_ms"]

    def test_wrist_readback_records_friction_direction_current_and_axis(self):
        raw = "okw 4 g0.250 gm0.934275,0.010061,0.500 " \
            "f0.100 fs0.250 s1 flags3 dir-1 current125 seq42 " \
            "harm46,-84,-70,-33"
        parsed = parse_wrist_readback(raw.split())
        self.assertEqual(parsed["joint"], 4)
        self.assertEqual(parsed["friction_enabled_scale"], 0.1)
        self.assertEqual(parsed["direction"], -1)
        self.assertEqual(parsed["actual_assist_ma"], 125)
        self.assertEqual(parsed["raw"], raw.split())
        self.assertIn("g0.250", parsed["raw"])
        self.assertIn("gm0.934275,0.010061,0.500", parsed["raw"])
        self.assertIn("fs0.250", parsed["raw"])
        self.assertIn("harm46,-84,-70,-33", parsed["raw"])

        drag = parse_drag_readback(
            "okg 5 1 0.000 0.000 1.000 11 12".split())
        self.assertEqual(drag["mode"], 5)
        self.assertTrue(drag["gravity_feedforward_enabled"])
        self.assertEqual(drag["current_limit_a"], 1.0)
        self.assertEqual(drag["damping"], 0.0)
        self.assertEqual(drag["friction_alpha"], 0.0)

        drag_off = parse_drag_readback(
            "okg 5 0 0.250 0.125 0.800 11 12".split())
        self.assertFalse(drag_off["gravity_feedforward_enabled"])
        self.assertEqual(drag_off["damping"], 0.25)
        self.assertEqual(drag_off["friction_alpha"], 0.125)
        with self.assertRaises(ValueError):
            parse_drag_readback(
                "okg 5 2 0.250 0.125 0.800 11 12".split())
        with self.assertRaises(ValueError):
            parse_wrist_readback(
                "okw 6 g0.1 gm1,2,3 f0.1 fs0.2 s1 flags1 "
                "dir1 current10 seq1 harm1,2,3,4".split())

    def test_query_configured_readbacks_verifies_axis_number(self):
        wrist4 = (
            "okw 4 g0.1 gm1,2,3 f0.1 fs0.2 s1 flags1 "
            "dir1 current10 seq1 harm1,2,3,4").split()
        wrist5 = (
            "okw 5 g0.1 gm1,2,3 f0.1 fs0.2 s1 flags1 "
            "dir1 current10 seq1 harm1,2,3,4").split()
        drag = "okg 5 1 0.0 0.0 1.0 1 1".split()

        class GoodLink:
            def request(self, command, _prefix, **_kwargs):
                if command == "#GETWRIST 4":
                    return wrist4
                if command == "#GETWRIST 5":
                    return wrist5
                if command == "#GETDRAG":
                    return drag
                raise AssertionError(command)

        result = query_configured_readbacks(GoodLink())
        self.assertEqual(result["wrist"]["J4"]["joint"], 4)
        self.assertEqual(result["wrist"]["J5"]["joint"], 5)
        self.assertEqual(result["wrist"]["J4"]["friction_enabled_scale"], 0.1)
        self.assertEqual(result["wrist"]["J5"]["direction"], 1)
        self.assertEqual(result["wrist"]["J5"]["actual_assist_ma"], 10)

        class MismatchedLink(GoodLink):
            def request(self, command, _prefix, **_kwargs):
                if command == "#GETWRIST 4":
                    return wrist5
                return super().request(command, _prefix)

        with self.assertRaisesRegex(RuntimeError, "axis mismatch"):
            query_configured_readbacks(MismatchedLink())

    def test_j3_coupling_bound_is_checked_offline_and_during_capture(self):
        settings = make_settings(joint=5, j3_deviation_deg=2.5)

        class CoupledCollision(SafeEvaluator):
            @staticmethod
            def evaluate(q, limit_margin_deg=0.0):
                if q[2] < 108.0 and q[4] == 0.0:
                    return False, np.zeros(3), [("elbow", "wrist")]
                return SafeEvaluator.evaluate(q)

        with self.assertRaisesRegex(RuntimeError, "corridor preflight failed"):
            preflight_wrist_corridor(CoupledCollision(), settings)
        q = list(settings.reference_deg)
        q[2] -= 2.0
        wrist_drag_trial._validate_capture(q, make_telemetry(100, q), status_fields(),
                                           settings, SafeEvaluator(), None, 1.0)
        for axis, value in [(2, 107.4), (0, -91.1)]:
            changed = q.copy()
            changed[axis] = value
            with self.assertRaisesRegex(RuntimeError, f"Held J{axis + 1} deviated"):
                wrist_drag_trial._validate_capture(
                    changed, make_telemetry(100, changed), status_fields(),
                    settings, SafeEvaluator(), None, 1.0)

    def test_corridor_preflight_rejects_unsafe_pose_before_enable(self):
        class UnsafeEvaluator:
            collision_tolerance_m = 1e-9

            @staticmethod
            def evaluate(q, limit_margin_deg=0.0):
                del limit_margin_deg
                if q[3] >= 6:
                    return False, np.array([0.0, 0.0, 4.0]), [
                        ("wrist_link", "floor", -0.001)]
                return True, np.array([0.0, 0.0, 100.0]), []

        settings = make_settings(
            j4_interval_deg=(0.0, 10.0),
            j5_interval_deg=(0.0, 2.0),
            corridor_step_deg=2.0,
        )
        with self.assertRaisesRegex(RuntimeError, "corridor preflight failed"):
            preflight_wrist_corridor(UnsafeEvaluator(), settings)

    def test_release_drift_uses_exact_tick_interpolation_and_marks_gaps(self):
        def captured(ticks, q4, q5):
            result = []
            for index, tick in enumerate(ticks):
                samples = [
                    {"position_tick_ms": tick}
                    for _axis in range(6)
                ]
                result.append(CapturedSample(
                    sample_index=index,
                    t_s=index / 10.0,
                    phase="post_release",
                    release_marker=index == 0,
                    q_deg=[-90, -16, 110, q4[index], q5[index], 0],
                    samples=samples,
                    status=[1, 5, 0, 0, 0, 0, 0, 0, 0],
                    snapshot_age_ms=50.0,
                    other_max_deviation_deg=0.0,
                    j4_excursion_deg=0.0,
                    j5_excursion_deg=0.0,
                ))
            return result

        samples = captured(
            [1000, 5000, 9000, 13000],
            [0.0, 4.0, 8.0, 12.0],
            [1.0, 1.5, 1.0, 0.5],
        )
        j4 = summarize_release_drift(samples, 0, 4, 4500)
        j5 = summarize_release_drift(samples, 0, 5, 4500)
        self.assertEqual(j4["status"], "verified")
        self.assertAlmostEqual(j4["net_drift_deg"], 10.0)
        self.assertAlmostEqual(j4["peak_to_peak_deg"], 10.0)
        self.assertAlmostEqual(j5["net_drift_deg"], -0.25)
        self.assertAlmostEqual(j5["peak_to_peak_deg"], 0.75)

        gapped = summarize_release_drift(samples, 0, 4, 1000)
        self.assertEqual(gapped["status"], "unverified")
        self.assertIn("gap", gapped["reason"])

        short = summarize_release_drift(samples[:2], 0, 4, 5000)
        self.assertEqual(short["status"], "unverified")
        self.assertIn("not covered", short["reason"])

    def test_run_trial_records_release_marker_current_ticks_and_drift(self):
        class Clock:
            now = 0.0

            @classmethod
            def monotonic(cls):
                return cls.now

            @classmethod
            def sleep(cls, seconds):
                cls.now += seconds

        Clock.now = 0.0
        calls = 0

        def fresh_pose(_link, return_samples=False, *, expected_mode=2):
            nonlocal calls
            self.assertTrue(return_samples)
            self.assertEqual(expected_mode, 5)
            tick = 1000 + calls * 100
            post = calls >= 20
            q4 = 4.0 if post else min(4.0, calls * 0.2)
            q5 = -2.0 + (0.005 * (calls - 20) if post else 0.0)
            q = [-90.0, -16.0, 110.0, q4, q5, 0.0]
            samples = make_telemetry(tick, q)
            calls += 1
            return q, samples

        class WristLink:
            def __init__(self):
                self.commands = []

            def request(self, command, _prefix, **_kwargs):
                self.commands.append(command)
                if command == "#GETWRIST 4":
                    return (
                        "okw 4 g0.1 gm1,2,3 f0.1 fs0.2 s1 flags1 "
                        "dir1 current10 seq1 harm1,2,3,4"
                    ).split()
                if command == "#GETWRIST 5":
                    return (
                        "okw 5 g0.1 gm1,2,3 f0.1 fs0.2 s1 flags1 "
                        "dir-1 current20 seq1 harm1,2,3,4"
                    ).split()
                raise AssertionError(command)

        file = io.StringIO()
        writer = csv.DictWriter(file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        settings = make_settings(reference_deg=(-90, -16, 110, 4, -2, 0))
        wrist_link = WristLink()
        with patch.object(
                wrist_drag_trial, "read_fresh_pose", side_effect=fresh_pose), \
                patch.object(
                    wrist_drag_trial,
                    "status_fields",
                    return_value=status_fields(enabled=1, mode=5)), \
                patch.object(wrist_drag_trial.time, "monotonic",
                             Clock.monotonic), \
                patch.object(wrist_drag_trial.time, "sleep", Clock.sleep):
            captured, release_index, release = run_trial(
                wrist_link,
                settings,
                SafeEvaluator(),
                writer,
                file,
                release_requested=lambda elapsed: elapsed >= 2.0,
                drag_current_limit_a=1.0,
            )

        self.assertEqual(release_index, 20)
        self.assertEqual(calls, 121)
        self.assertEqual(release["drift"]["J4"]["status"], "verified")
        self.assertEqual(release["drift"]["J5"]["status"], "verified")
        self.assertEqual(len(captured), 121)
        rows = list(csv.DictReader(io.StringIO(file.getvalue())))
        self.assertEqual(len(rows), 121)
        self.assertEqual(
            sum(int(row["release_marker"]) for row in rows), 1)
        self.assertEqual(rows[0]["phase"], "drag")
        self.assertEqual(rows[-1]["phase"], "post_release")
        self.assertEqual(rows[0]["j4_position_tick_ms"], "1000")
        self.assertEqual(rows[-1]["j5_position_tick_ms"], "13000")
        self.assertEqual(rows[0]["j4_i_cmd_motor_a"], "0.10000000")
        self.assertEqual(rows[-1]["j4_q_deg"], "4.00000000")
        self.assertEqual(rows[0]["j4_wrist_current_ma"], "10")
        self.assertEqual(rows[0]["j5_wrist_current_ma"], "20")
        self.assertEqual(rows[-1]["active_axis_velocity_deg_s"], "0.00000000")
        self.assertEqual(
            [command for command in wrist_link.commands if command.startswith("#GETWRIST")],
            ["#GETWRIST 4", "#GETWRIST 5"] * len(rows),
        )

    def test_execute_mode_5_has_no_target_commands_and_restores_mode_2(self):
        reference = [-90.0, -16.0, 110.0, 0.0, 0.0, 0.0]
        release = {
            "source": "test",
            "sample_index": 0,
            "elapsed_s": 1.0,
            "wall_time_utc": "test",
            "drift": {
                "J4": {"status": "verified"},
                "J5": {"status": "verified"},
            },
        }
        file = io.StringIO()
        writer = csv.DictWriter(file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        outcome = {}
        robot = ModeRobotLink()

        def readbacks(_link):
            stopped = robot.enabled == 0
            return {
                "wrist": {
                    "J4": {"friction_enabled_scale": 0.0 if stopped else 0.1,
                           "direction": 1,
                           "actual_assist_ma": 0 if stopped else 10},
                    "J5": {"friction_enabled_scale": 0.0 if stopped else 0.1,
                           "direction": 1,
                           "actual_assist_ma": 0 if stopped else 10},
                },
                "drag": {"current_limit_a": 1.0, "mode": robot.mode},
            }

        with patch.object(
                wrist_drag_trial,
                "read_fresh_pose",
                return_value=reference), \
                patch.object(
                    wrist_drag_trial,
                    "preflight_wrist_corridor",
                    return_value={"checked_poses": 1}), \
                patch.object(
                    wrist_drag_trial,
                    "query_configured_readbacks",
                    side_effect=readbacks), \
                patch.object(
                    wrist_drag_trial,
                    "run_trial",
                    return_value=([], 0, release)):
            execute_trial(
                robot,
                make_settings(reference_deg=tuple(reference)),
                SafeEvaluator(),
                writer,
                file,
                outcome,
                release_requested=None,
                metadata_callback=lambda: None,
            )

        self.assertTrue(outcome["completed"])
        self.assertTrue(outcome["shutdown"]["verified"])
        self.assertEqual(robot.mode, 2)
        self.assertEqual(robot.enabled, 0)
        self.assertNotIn(
            "#CMDMODE 2", robot.commands[:robot.commands.index("!START")])
        self.assertIn("#CMDMODE 5", robot.commands)
        self.assertGreater(
            robot.commands.index("!STOP"), robot.commands.index("!START"))
        self.assertFalse(any(
            command.startswith(">")
            or command.startswith("@")
            or command.startswith("#DRAG_GAIN")
            or command.startswith("#WRIST_") and not command.startswith("#GET")
            or command.startswith("#LOCALCOMP")
            for command in robot.commands
        ))

    def test_keyboard_interrupt_is_finally_stopped_and_mode_2_restored(self):
        reference = [-90.0, -16.0, 110.0, 0.0, 0.0, 0.0]
        robot = ModeRobotLink()

        def readbacks(_link):
            return {
                "wrist": {
                    "J4": {"friction_enabled_scale": 0.1, "direction": 1,
                           "actual_assist_ma": 10},
                    "J5": {"friction_enabled_scale": 0.1, "direction": 1,
                           "actual_assist_ma": 10},
                },
                "drag": {"current_limit_a": 1.0, "mode": robot.mode},
            }

        file = io.StringIO()
        writer = csv.DictWriter(file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        outcome = {}
        with patch.object(
                wrist_drag_trial,
                "read_fresh_pose",
                return_value=reference), \
                patch.object(
                    wrist_drag_trial,
                    "preflight_wrist_corridor",
                    return_value={"checked_poses": 1}), \
                patch.object(
                    wrist_drag_trial,
                    "query_configured_readbacks",
                    side_effect=readbacks), \
                patch.object(
                    wrist_drag_trial,
                    "run_trial",
                    side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                execute_trial(
                    robot,
                    make_settings(reference_deg=tuple(reference)),
                    SafeEvaluator(),
                    writer,
                    file,
                    outcome,
                    release_requested=None,
                    metadata_callback=lambda: None,
                )

        self.assertFalse(outcome["completed"])
        self.assertTrue(outcome["shutdown"]["verified"])
        self.assertEqual(robot.mode, 2)
        self.assertEqual(robot.enabled, 0)
        self.assertEqual(robot.commands.count("!STOP"), 1)
        self.assertGreater(
            robot.commands.index("!STOP"), robot.commands.index("!START"))
        self.assertEqual(outcome["drift"]["J4"]["status"], "unverified")
        self.assertEqual(outcome["drift"]["J5"]["status"], "unverified")

    def test_startup_refusal_is_read_only_and_sends_no_stop(self):
        reference = [-90.0, -16.0, 110.0, 0.0, 0.0, 0.0]
        for initial_mode, initial_enabled in ((5, 0), (2, 1)):
            robot = ModeRobotLink()
            robot.mode = initial_mode
            robot.enabled = initial_enabled
            file = io.StringIO()
            writer = csv.DictWriter(file, fieldnames=CSV_FIELDS)
            writer.writeheader()
            outcome = {}
            with self.subTest(mode=initial_mode, enabled=initial_enabled):
                with patch.object(
                        wrist_drag_trial,
                        "read_fresh_pose",
                        return_value=reference), \
                        patch.object(
                            wrist_drag_trial,
                            "preflight_wrist_corridor",
                            return_value={"checked_poses": 1}):
                    with self.assertRaises(RuntimeError):
                        execute_trial(
                            robot,
                            make_settings(reference_deg=tuple(reference)),
                            SafeEvaluator(),
                            writer,
                            file,
                            outcome,
                            release_requested=None,
                            metadata_callback=lambda: None,
                        )
            self.assertNotIn("!STOP", robot.commands)
            self.assertNotIn("!START", robot.commands)

    def test_offline_main_does_not_import_serial(self):
        original_import = __import__

        def guarded_import(name, *args, **kwargs):
            if name == "serial":
                raise AssertionError("offline preview imported serial")
            return original_import(name, *args, **kwargs)

        with patch.object(
                wrist_drag_trial, "PoseEvaluator", return_value=SafeEvaluator()), \
                patch.object(
                    wrist_drag_trial,
                    "preflight_wrist_corridor",
                    return_value={"checked_poses": 1}), \
                patch("builtins.__import__", side_effect=guarded_import):
            result = wrist_drag_trial.main([
                "--reference", "-90", "-16", "110", "0", "0", "0",
                "--joint", "4",
                "--j4-min", "-5", "--j4-max", "5",
                "--j5-min", "-5", "--j5-max", "5",
                "--other-deviation", "1",
                "--duration", "12",
                "--gripper-rad", "0",
            ])
        self.assertEqual(result, 0)

    def test_desynchronized_shutdown_sends_only_stop(self):
        class DesynchronizedLink:
            _desynchronized = True

            def __init__(self):
                self.commands = []

            def request(self, command, _prefix, **_kwargs):
                self.commands.append(command)
                if command == "!STOP":
                    return ["Stopped", "ok"]
                raise AssertionError(command)

        link = DesynchronizedLink()
        result = stop_and_restore_mode_2(link)
        self.assertFalse(result["verified"])
        self.assertTrue(result["stop_acknowledged"])
        self.assertEqual(link.commands, ["!STOP"])

    def test_run_trial_captures_each_row_wrist_readback_not_reused(self):
        class Clock:
            now = 0.0

            @classmethod
            def monotonic(cls):
                return cls.now

            @classmethod
            def sleep(cls, seconds):
                cls.now += seconds

        Clock.now = 0.0
        calls = 0

        def fresh_pose(_link, return_samples=False, *, expected_mode=2):
            nonlocal calls
            self.assertTrue(return_samples)
            self.assertEqual(expected_mode, 5)
            tick = 1000 + calls * 100
            q = [-90.0, -16.0, 110.0, 4.0, -2.0, 0.0]
            samples = make_telemetry(tick, q)
            calls += 1
            return q, samples

        class VaryingWristLink:
            def __init__(self):
                self.calls = 0
                self.commands = []

            def request(self, command, _prefix, **_kwargs):
                self.commands.append(command)
                self.calls += 1
                if command == "#GETWRIST 4":
                    return (
                        f"okw 4 g0.1 gm1,2,3 f0.1 fs0.2 s1 flags1 "
                        f"dir1 current{10 + self.calls} seq1 "
                        f"harm1,2,3,4"
                    ).split()
                if command == "#GETWRIST 5":
                    return (
                        f"okw 5 g0.1 gm1,2,3 f0.1 fs0.2 s1 flags1 "
                        f"dir-1 current{20 + self.calls} seq1 "
                        f"harm1,2,3,4"
                    ).split()
                raise AssertionError(command)

        file = io.StringIO()
        writer = csv.DictWriter(file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        settings = make_settings(reference_deg=(-90, -16, 110, 4, -2, 0))
        link = VaryingWristLink()
        with patch.object(
                wrist_drag_trial, "read_fresh_pose", side_effect=fresh_pose), \
                patch.object(
                    wrist_drag_trial,
                    "status_fields",
                    return_value=status_fields(enabled=1, mode=5)), \
                patch.object(wrist_drag_trial.time, "monotonic",
                             Clock.monotonic), \
                patch.object(wrist_drag_trial.time, "sleep", Clock.sleep):
            _captured, release_index, release = run_trial(
                link,
                settings,
                SafeEvaluator(),
                writer,
                file,
                release_requested=lambda elapsed: elapsed >= 2.0,
                drag_current_limit_a=1.0,
            )

        self.assertEqual(release_index, 20)
        self.assertEqual(release["drift"]["J4"]["status"], "verified")
        rows = list(csv.DictReader(io.StringIO(file.getvalue())))
        j4_currents = [int(row["j4_wrist_current_ma"]) for row in rows]
        self.assertEqual(
            j4_currents,
            [11 + 2 * index for index in range(len(rows))],
        )
        self.assertEqual(len(set(j4_currents)), len(j4_currents))

    def test_hands_off_returns_rest_result_not_release(self):
        class Clock:
            now = 0.0

            @classmethod
            def monotonic(cls):
                return cls.now

            @classmethod
            def sleep(cls, seconds):
                cls.now += seconds

        Clock.now = 0.0
        calls = 0

        def fresh_pose(_link, return_samples=False, *, expected_mode=2):
            nonlocal calls
            tick = 1000 + calls * 100
            q = [-90.0, -16.0, 110.0, 4.0, -2.0, 0.0]
            samples = make_telemetry(tick, q)
            calls += 1
            return q, samples

        class FixedWristLink:
            def request(self, command, _prefix, **_kwargs):
                if command == "#GETWRIST 4":
                    return (
                        "okw 4 g0.1 gm1,2,3 f0.1 fs0.2 s1 flags1 "
                        "dir1 current10 seq1 harm1,2,3,4"
                    ).split()
                if command == "#GETWRIST 5":
                    return (
                        "okw 5 g0.1 gm1,2,3 f0.1 fs0.2 s1 flags1 "
                        "dir-1 current20 seq1 harm1,2,3,4"
                    ).split()
                raise AssertionError(command)

        file = io.StringIO()
        writer = csv.DictWriter(file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        settings = make_settings(
            reference_deg=(-90, -16, 110, 4, -2, 0),
            hands_off=True,
            duration_s=12.0,
        )
        with patch.object(
                wrist_drag_trial, "read_fresh_pose", side_effect=fresh_pose), \
                patch.object(
                    wrist_drag_trial,
                    "status_fields",
                    return_value=status_fields(enabled=1, mode=5)), \
                patch.object(wrist_drag_trial.time, "monotonic",
                             Clock.monotonic), \
                patch.object(wrist_drag_trial.time, "sleep", Clock.sleep):
            _captured, release_index, release = run_trial(
                FixedWristLink(),
                settings,
                SafeEvaluator(),
                writer,
                file,
                release_requested=lambda elapsed: True,
                drag_current_limit_a=1.0,
                release_source="hands_off",
            )

        self.assertIsNone(release_index)
        self.assertTrue(release["hands_off"])
        self.assertEqual(release["source"], "hands_off")
        self.assertEqual(release["drift"]["J4"]["status"], "verified")
        rows = list(csv.DictReader(io.StringIO(file.getvalue())))
        self.assertTrue(rows)
        self.assertEqual({row["phase"] for row in rows}, {"hands_off"})
        self.assertEqual(sum(int(row["release_marker"]) for row in rows), 0)

    def test_after_stop_residual_assist_rejects_completed(self):
        reference = [-90.0, -16.0, 110.0, 0.0, 0.0, 0.0]
        release = {
            "source": "test",
            "sample_index": 0,
            "elapsed_s": 1.0,
            "wall_time_utc": "test",
            "drift": {
                "J4": {"status": "verified"},
                "J5": {"status": "verified"},
            },
        }
        file = io.StringIO()
        writer = csv.DictWriter(file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        outcome = {}
        robot = ModeRobotLink()

        def readbacks(_link):
            return {
                "wrist": {
                    "J4": {"friction_enabled_scale": 0.1, "direction": 1,
                           "actual_assist_ma": 10},
                    "J5": {"friction_enabled_scale": 0.1, "direction": 1,
                           "actual_assist_ma": 10},
                },
                "drag": {"current_limit_a": 1.0, "mode": robot.mode},
            }

        with patch.object(
                wrist_drag_trial,
                "read_fresh_pose",
                return_value=reference), \
                patch.object(
                    wrist_drag_trial,
                    "preflight_wrist_corridor",
                    return_value={"checked_poses": 1}), \
                patch.object(
                    wrist_drag_trial,
                    "query_configured_readbacks",
                    side_effect=readbacks), \
                patch.object(
                    wrist_drag_trial,
                    "run_trial",
                    return_value=([], 0, release)):
            execute_trial(
                robot,
                make_settings(reference_deg=tuple(reference)),
                SafeEvaluator(),
                writer,
                file,
                outcome,
                release_requested=None,
                metadata_callback=lambda: None,
            )

        self.assertFalse(outcome["completed"])
        self.assertIn("after_stop", outcome)
        self.assertFalse(outcome["after_stop_verified"])
        self.assertEqual(outcome["after_stop"]["wrist"]["J4"]["actual_assist_ma"], 10)

    def test_old_telemetry_transaction_signature_remains_supported(self):
        class OldSignatureLink:
            def __init__(self):
                self.calls = []

            def telemetry_transaction(self, joint, timeout=1.0):
                self.calls.append((joint, timeout))
                wire = telemetry_frame(
                    joint=joint).decode("ascii").strip().split()
                return wire, 10.0, 10.01, 10.02

        link = OldSignatureLink()
        sample, q_deg = request_telemetry(link, 2)
        self.assertEqual(q_deg, 0.0)
        self.assertEqual(sample["position_tick_ms"], 100)
        self.assertEqual(link.calls, [(2, 1.0)])

        with self.assertRaises(TypeError):
            request_telemetry(link, 2, expected_mode=5)


if __name__ == "__main__":
    unittest.main()
