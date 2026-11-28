import sys
import unittest
from contextlib import contextmanager
from itertools import cycle
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import batch_collect
from batch_collect import move_to


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeEvaluator:
    def evaluate(self, pose, limit_margin_deg=0.0):
        return True, np.zeros(3), []


class FakeLink:
    def __init__(self):
        self.stop_failed = False
        self.last_stop_acknowledged = False
        self.requests = []

    def request(self, command, prefix, timeout=0.5, retries=0, joint=None):
        self.requests.append((command, prefix))
        if command == "!START":
            return ["Started", "ok"]
        if command == "!STOP":
            return ["Stopped", "ok"]
        if prefix == "queue":
            return ["queue"]
        raise AssertionError(f"unexpected request {command!r} {prefix!r}")


def pose_sequence(initial, loop_poses):
    state = {"index": 0}

    def read(link):
        index = state["index"]
        state["index"] += 1
        if index < 2:
            return list(initial)
        return list(loop_poses[index - 2])

    return read, state


def constant_pose(initial, pose):
    state = {"index": 0}

    def read(link):
        index = state["index"]
        state["index"] += 1
        if index < 2:
            return list(initial)
        return list(pose)

    return read, state


def cycling_pose_sequence(initial, loop_poses):
    state = {"index": 0}
    poses = cycle(loop_poses)

    def read(link):
        index = state["index"]
        state["index"] += 1
        if index < 2:
            return list(initial)
        return list(next(poses))

    return read, state


@contextmanager
def move_to_patches(clock, read_side_effect):
    with patch.object(batch_collect.time, "monotonic", clock.monotonic), \
            patch.object(batch_collect.time, "sleep", clock.sleep), \
            patch.object(batch_collect, "read_fresh_pose",
                         side_effect=read_side_effect), \
            patch.object(batch_collect, "require_ready_status",
                         return_value=[1, 2, 0, 0, 0, 0, 0, 0, 0]), \
            patch.object(batch_collect, "verify_path"), \
            patch.object(batch_collect, "require_fresh_pose_close"), \
            patch.object(batch_collect, "path_deviation_deg",
                         return_value=0.0):
        yield


def run_move_to(read_side_effect, target, tolerance, speed=10.0,
                expected_start=None):
    link = FakeLink()
    clock = FakeClock()
    evaluator = FakeEvaluator()
    with move_to_patches(clock, read_side_effect):
        move_to(link, target, speed, tolerance, evaluator,
                expected_start=expected_start)
    return link, clock


class MoveToSettleTests(unittest.TestCase):
    def test_crossing_window_then_leaving_does_not_stop_early(self):
        start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        target = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        at_target = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        loop_poses = [
            at_target,
            [10.6, 0.0, 0.0, 0.0, 0.0, 0.0],
            at_target,
            at_target,
            at_target,
            at_target,
            at_target,
            at_target,
        ]
        read_side_effect, state = pose_sequence(start, loop_poses)
        link, _clock = run_move_to(read_side_effect, target, 0.5)

        self.assertEqual(state["index"], 2 + len(loop_poses))
        self.assertEqual(link.requests[-1], ("!STOP", "Stopped ok"))
        self.assertTrue(link.last_stop_acknowledged)
        self.assertEqual(link.requests.count(("!STOP", "Stopped ok")), 1)

    def test_converging_after_tolerance_entry_succeeds(self):
        start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        target = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        loop_poses = [
            [9.84, 0.0, 0.0, 0.0, 0.0, 0.0],
            [9.97, 0.0, 0.0, 0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        read_side_effect, state = pose_sequence(start, loop_poses)
        link, _clock = run_move_to(read_side_effect, target, 0.5)

        self.assertEqual(state["index"], 2 + len(loop_poses))
        self.assertEqual(link.requests[-1], ("!STOP", "Stopped ok"))
        self.assertTrue(link.last_stop_acknowledged)

    def test_completes_only_after_stable_window(self):
        start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        target = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        at_target = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        loop_poses = [at_target] * 6
        read_side_effect, state = pose_sequence(start, loop_poses)
        link, _clock = run_move_to(read_side_effect, target, 0.5)

        self.assertEqual(state["index"], 2 + len(loop_poses))
        self.assertEqual(link.requests[-1], ("!STOP", "Stopped ok"))
        self.assertTrue(link.last_stop_acknowledged)

    def test_timeout_stops_and_reports_current_pose(self):
        start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        target = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        never_settled = [10.6, 0.0, 0.0, 0.0, 0.0, 0.0]
        read_side_effect, state = constant_pose(start, never_settled)
        link = FakeLink()
        clock = FakeClock()
        evaluator = FakeEvaluator()
        with move_to_patches(clock, read_side_effect):
            with self.assertRaises(TimeoutError) as caught:
                move_to(link, target, 10.0, 0.5, evaluator)

        self.assertIn("Positioning timeout", str(caught.exception))
        self.assertEqual(link.requests[-1], ("!STOP", "Stopped ok"))
        self.assertTrue(link.last_stop_acknowledged)
        self.assertGreater(state["index"], 2)

    def test_sustained_oscillation_times_out_and_stops(self):
        start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        target = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        loop_poses = [
            [9.89, 0.0, 0.0, 0.0, 0.0, 0.0],
            [10.11, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        read_side_effect, state = cycling_pose_sequence(start, loop_poses)
        link = FakeLink()
        clock = FakeClock()
        evaluator = FakeEvaluator()
        with move_to_patches(clock, read_side_effect):
            with self.assertRaisesRegex(TimeoutError, "Positioning timeout"):
                move_to(link, target, 10.0, 0.5, evaluator)

        self.assertEqual(link.requests[-1], ("!STOP", "Stopped ok"))
        self.assertTrue(link.last_stop_acknowledged)
        self.assertGreater(state["index"], 2)


if __name__ == "__main__":
    unittest.main()
