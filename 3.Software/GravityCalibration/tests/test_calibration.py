import csv
import json
import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from calibration import (FIELDS, fit_local, make_pairs, predict_local,
                         limited_feedforward, tick_delta_ms, tick_skew_ms)
from collect import Link, parse_telemetry, validate_config, run
from demo_data import generate


class CalibrationTests(unittest.TestCase):
    def test_recover_gravity_and_directional_friction(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "test.csv"
            generate(path, 32)
            pairs, info = make_pairs([path], 2, 3.0)
            coefficients, metrics = fit_local(pairs)
            np.testing.assert_allclose(coefficients, [0.3, 0.6, 0.05], atol=0.015)
            self.assertLess(metrics["train_rmse_a"], 0.003)
            self.assertAlmostEqual(float(np.median(pairs[:, 2])), 0.14, places=2)
            self.assertGreater(info["paired_bins"], 50)
            # A single corrupted held-joint trace must not silently become a global model.
            with path.open() as file:
                rows = list(csv.DictReader(file))
            rows[200]["q3_deg"] = 100
            with path.open("w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=FIELDS)
                writer.writeheader()
                writer.writerows(rows)
            with self.assertRaisesRegex(ValueError, "Other joints moved"):
                make_pairs([path], 2, 3.0)

    def test_invalid_data_and_saturation_fail_closed(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "test.csv"
            generate(path, 32)
            with self.assertRaisesRegex(ValueError, "matched"):
                make_pairs([path], 2, 0.01)
            with path.open() as file:
                rows = list(csv.DictReader(file))
            rows[10]["i_cmd_motor_a"] = "nan"
            with path.open("w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=FIELDS)
                writer.writeheader()
                writer.writerows(rows)
            with self.assertRaisesRegex(ValueError, "non-finite"):
                make_pairs([path], 2, 3.0)

    def test_identifiability_and_model_domain(self):
        with self.assertRaisesRegex(ValueError, "ill-conditioned"):
            fit_local(np.ones((15, 4)))
        model = dict(joint=2, fixed_pose_deg=[0, 0, 90, 0, 0, 0], pose_tolerance_deg=0.5,
                     angle_range_deg=[-30, 30], coefficients_a=[0.3, 0.6, 0.05])
        self.assertAlmostEqual(predict_local(model, [0, 0, 90, 0, 0, 0]), 0.65)
        with self.assertRaisesRegex(ValueError, "fixed pose"):
            predict_local(model, [0, 0, 95, 0, 0, 0])
        with self.assertRaisesRegex(ValueError, "angle range"):
            predict_local(model, [0, 40, 90, 0, 0, 0])

    def test_wire_units_and_tick_wrap(self):
        self.assertEqual(tick_skew_ms([2**32 - 2, 0, 1]), 3)
        self.assertEqual(tick_delta_ms(2**32 - 2, 1), 3)
        row, q = parse_telemetry("okm 2 100 101 102 -30.0 2.0 -0.5 2 0 1".split(), 2)
        self.assertEqual((q, row["dq_deg_s"], row["i_cmd_motor_a"], row["skew_ms"]),
                         (-30.0, 2.0, -0.5, 2))
        with self.assertRaises(ValueError):
            parse_telemetry("okm 2 100 101 102 nan 2.0 -0.5 2 0 1".split(), 2)

    def test_serial_partial_reply_and_motion_ack(self):
        class Port:
            chunks = [b"ok\r\n", b"okm 2 100 101 ", b"102 -30 2 -0.5 2 0 1\r\n"]
            def write(self, data):
                self.written = data
            def read_until(self, delimiter):
                return self.chunks.pop(0)
        port = Port()
        data = Link(port).request("#GETTELEM 2", "okm ")
        self.assertEqual(port.written, b"#GETTELEM 2\n")
        self.assertEqual(parse_telemetry(data, 2)[1], -30)

    def test_serial_coalesced_replies_are_not_lost(self):
        class Port:
            def __init__(self):
                self.chunks = [b"15\nokm 2 100 101 102 -30 2 -0.5 2 0 1\n"]

            def write(self, data):
                self.written = data

            def read_until(self, delimiter):
                return self.chunks.pop(0) if self.chunks else b""

        link = Link(Port())
        self.assertEqual(link.request(">", "queue"), ["15"])
        data = link.request("#GETTELEM 2", "okm ")
        self.assertEqual(parse_telemetry(data, 2)[1], -30)

    def test_stale_other_joint_reply_is_discarded(self):
        class Port:
            def __init__(self):
                self.chunks = [b"okm 1 100 101 102 -30 2 -0.5 2 0 1\r\n",
                               b"okm 2 100 101 102 0 2 -0.5 2 0 1\r\n"]

            def write(self, data):
                pass

            def read_until(self, delimiter):
                return self.chunks.pop(0) if self.chunks else b""

        link = Link(Port())
        data = link.request("#GETTELEM 2", "okm ", timeout=0.01,
                            retries=1, joint=2)
        self.assertEqual(data[1], "2")
        self.assertEqual(link.retries, 0)

    def test_read_only_query_retry_on_silent_drop(self):
        class Port:
            def __init__(self):
                self.writes = 0

            def write(self, data):
                self.writes += 1
                self.written = data

            def read_until(self, delimiter):
                if self.writes < 2:
                    return b""
                return b"okm 2 100 101 102 -30 2 -0.5 2 0 1\r\n"

        link = Link(Port())
        data = link.request("#GETTELEM 2", "okm ", timeout=0.01, retries=1)
        self.assertEqual(parse_telemetry(data, 2)[1], -30)
        self.assertEqual(link.retries, 1)
        self.assertLess(link.last_roundtrip_ms, 100)

    def test_preview_and_feedforward_limits(self):
        config = json.loads((Path(__file__).resolve().parents[1] / "config.example.json").read_text())
        config.update(joint=2, sweep_min_deg=-30, sweep_max_deg=30)
        self.assertEqual(validate_config(config), 2)
        config["operator_reviewed_workspace"] = False
        with self.assertRaises(ValueError):
            validate_config(config, execute=True)
        self.assertAlmostEqual(limited_feedforward(2, 0, 0.01, alpha=0.2,
                                                   limit_a=0.3, slew_a_s=1), 0.01)
        self.assertEqual(limited_feedforward(2, 0.3, 1, alpha=1, limit_a=0.3, slew_a_s=1), 0.3)
        with self.assertRaises(ValueError):
            limited_feedforward(float("nan"), 0, 1, alpha=1, limit_a=1, slew_a_s=1)

    def test_sweep_sequence_and_held_joint_interlock(self):
        class Clock:
            now = 0.0
            def monotonic(self):
                self.now += 0.01
                return self.now
            def sleep(self, delay):
                self.now += delay

        class Robot:
            def __init__(self):
                self.q = [0, 0, 90, 0, 0, 0]
                self.target = 0.0
                self.enabled = 0
                self.commands = []
                self.tick = 100
            def request(self, command, prefix, timeout=0.5, retries=0, joint=None):
                self.commands.append(command)
                if command == "!STOP":
                    self.enabled = 0
                    return ["Stopped", "ok"]
                if command == "!START":
                    self.enabled = 1
                    return ["Started", "ok"]
                if command.startswith("#CMDMODE"):
                    return ["Set", "command", "mode"]
                if command == "#GETSTATUS":
                    return f"oks {self.enabled} 2 0 0 0 0 0 0 0".split()
                if command.startswith(">"):
                    target = list(map(float, command[1:].split(",")))
                    self.target = target[1]
                    self.assert_held = [target[k] for k in [0, 2, 3, 4, 5]]
                    return ["15"]
                if command.startswith("#GETTELEM"):
                    axis = int(command.split()[1])
                    velocity = 0.0
                    if axis == 2 and self.enabled and abs(self.target - self.q[1]) > 1e-5:
                        velocity = float(np.sign(self.target - self.q[1]))
                        self.q[1] += float(np.clip(self.target - self.q[1], -0.05, 0.05))
                    self.tick += 50
                    return f"okm {axis} {self.tick} {self.tick-1} {self.tick-2} {self.q[axis-1]} {velocity} -0.2 2 0 1".split()
                if command == "#GETJPOS":
                    return ["okj", *map(str, self.q)]
                raise AssertionError(command)

        config = json.loads((Path(__file__).resolve().parents[1] / "config.example.json").read_text())
        config.update(joint=2, cycles=1, sweep_min_deg=-1, sweep_max_deg=1,
                      endpoint_hold_s=0.5, current_limit_a=2,
                      operator_reviewed_workspace=True)
        robot, clock = Robot(), Clock()
        file = io.StringIO()
        writer = csv.DictWriter(file, fieldnames=FIELDS)
        writer.writeheader()
        with patch("collect.time.monotonic", clock.monotonic), patch("collect.time.sleep", clock.sleep):
            count = run(robot, config, writer, file)
        motions = [c for c in robot.commands if c.startswith(">")]
        self.assertEqual(len(motions), 3)
        self.assertEqual([float(c[1:].split(",")[1]) for c in motions], [-1, 1, -1])
        self.assertEqual(robot.assert_held, [0, 90, 0, 0, 0])
        self.assertFalse(any(c.startswith("#CMDMODE") for c in robot.commands))
        self.assertGreater(count, 80)
        robot = Robot()
        robot.q[2] = 92
        with self.assertRaisesRegex(RuntimeError, "fixed pose"):
            run(robot, config, writer, file)
        self.assertNotIn("!START", robot.commands)


if __name__ == "__main__":
    unittest.main()
