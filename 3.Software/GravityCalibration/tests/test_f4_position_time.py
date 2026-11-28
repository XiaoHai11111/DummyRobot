import csv
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from calibration import FIELDS, make_pairs
from demo_data import generate
from fit_wrist import fit_group, input_evidence


def read_rows(path):
    with path.open(newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def write_rows(path, rows):
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


class PositionTimeSourceTests(unittest.TestCase):
    def test_empty_runs_default_to_host_and_preserve_legacy_return(self):
        candidate, summary = fit_group(
            2, "synthetic", "payload", [], minimum_poses=3)
        self.assertIsNone(candidate)
        self.assertEqual(summary["time_source"], "host")
        self.assertEqual(summary["reason"],
                         "only labelled independent_retest runs were found")

    def test_f4_wrap_ignores_late_host_timestamps(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "wrap.csv"
            generate(path, 32)
            rows = read_rows(path)
            reference_path = Path(folder) / "reference.csv"
            write_rows(reference_path, rows)
            reference_pairs, _ = make_pairs(
                [reference_path], 2, 3.0, time_source="f4-position")

            tick_start = 2**32 - 12_000
            for row in rows:
                source_tick = round(float(row["t_s"]) * 1000)
                row["position_tick_ms"] = str((tick_start + source_tick) % 2**32)

            with self.assertRaisesRegex(
                    ValueError, "f4-position.*velocity_source='position'"):
                make_pairs(
                    [reference_path],
                    2,
                    3.0,
                    velocity_source="f1",
                    time_source="f4-position",
                )

            # A late host reply makes the next receive timestamp go backward.
            # F4 position time remains regular and must ignore that host defect.
            rows[100]["t_s"] = f"{float(rows[99]['t_s']) + 0.5:.9f}"
            write_rows(path, rows)

            with self.assertRaisesRegex(ValueError, "time must increase strictly"):
                make_pairs([path], 2, 3.0)

            pairs, info = make_pairs(
                [path], 2, 3.0, time_source="f4-position")
            self.assertEqual(pairs.shape[1], 4)
            np.testing.assert_allclose(pairs, reference_pairs)
            self.assertEqual(info["time_source"], "f4-position")
            self.assertEqual(info["position_tick_wrap_count"], 1)
            self.assertGreaterEqual(info["paired_bins"], 12)

    def test_f4_rejects_invalid_backward_and_duplicate_ticks(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "bad-tick.csv"
            generate(path, 31)
            baseline = read_rows(path)

            cases = [
                ("outside uint32", {100: "4294967296"}, "outside uint32"),
                ("underscore integer", {100: "1_000"}, "raw unsigned decimal"),
                ("fraction", {100: "1.5"}, "raw unsigned decimal"),
                ("duplicate", {100: baseline[99]["position_tick_ms"]},
                 "duplicate"),
                ("backward",
                 {100: str(int(baseline[99]["position_tick_ms"]) - 1)},
                 "backward or ambiguous"),
                ("cross-half backward",
                 {99: "2147483648", 100: "2147483647"},
                 "backward or ambiguous"),
                ("exact half range",
                 {100: str(int(baseline[99]["position_tick_ms"]) + 2**31)},
                 "backward or ambiguous"),
            ]
            for label, changes, message in cases:
                with self.subTest(label=label):
                    rows = [row.copy() for row in baseline]
                    for row_index, value in changes.items():
                        rows[row_index]["position_tick_ms"] = value
                    write_rows(path, rows)
                    with self.assertRaisesRegex(ValueError, message):
                        make_pairs(
                            [path], 2, 3.0, time_source="f4-position")

    def test_time_source_is_recorded_in_evidence_and_model(self):
        runs = []
        for pose_index in range(3):
            for speed in (0.5, 1.0, 2.0):
                observations = []
                for q_deg in range(-30, 31, 5):
                    torque = 0.04 * (pose_index - 1) + 0.0002 * q_deg
                    phase = np.deg2rad(30 * q_deg)
                    even = (
                        0.8 * torque
                        + 0.02 * np.sin(phase)
                        - 0.01 * np.cos(phase)
                    )
                    odd = (
                        0.2
                        + 0.006 * speed
                        + 0.03 * np.sin(phase)
                        + 0.01 * np.cos(phase)
                    )
                    observations.append({
                        "run_id": f"P{pose_index}-v{speed}.csv",
                        "source": f"P{pose_index}-v{speed}.csv",
                        "pose_id": f"P{pose_index}",
                        "q_deg": float(q_deg),
                        "q_all_deg": [0, q_deg, 90, 0, 0, 0],
                        "gravity_torque_nm": float(torque),
                        "speed_magnitude_deg_s": float(speed),
                        "even_current_a": float(even),
                        "odd_current_a": float(odd),
                        "commanded_speed_deg_s": float(speed),
                    })
                runs.append({
                    "source": f"P{pose_index}-v{speed}.csv",
                    "metadata": f"P{pose_index}-v{speed}.meta.json",
                    "csv_sha256": "csv-hash",
                    "metadata_sha256": "meta-hash",
                    "gripper_rad": 0.0,
                    "config": {
                        "joint": 2,
                        "speed_deg_s": speed,
                        "current_limit_a": 3.0,
                        "payload_description": "synthetic payload",
                    },
                    "info": {
                        "pose_id": f"P{pose_index}",
                        "raw_samples": len(observations),
                        "retained_samples": len(observations),
                        "time_source": "f4-position",
                        "position_tick_wrap_count": 1,
                    },
                    "pairs": np.zeros((len(observations), 4)),
                    "observations": observations,
                })

        candidate, summary = fit_group(
            2, "synthetic", "payload", runs, minimum_poses=3)
        self.assertIsNotNone(candidate)
        self.assertEqual(candidate["time_source"], "f4-position")
        self.assertEqual(summary["time_source"], "f4-position")
        self.assertTrue(all(
            record["time_source"] == "f4-position"
            for record in candidate["source_files"]
        ))
        evidence = input_evidence(runs)
        self.assertEqual(evidence["time_source"], "f4-position")
        self.assertTrue(all(
            record["time_source"] == "f4-position"
            for record in evidence["files"]
        ))


if __name__ == "__main__":
    unittest.main()
