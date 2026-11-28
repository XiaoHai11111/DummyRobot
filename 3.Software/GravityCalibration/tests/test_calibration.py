import csv
import contextlib
import hashlib
import json
import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from calibration import (FIELDS, JOINT_LIMITS_DEG, WRIST_HISTORICAL_LIMITS_DEG,
                         WRIST_STRICT_LIMITS_DEG, WRIST_STRICT_PROFILE,
                         fit_local, make_pairs,
                         predict_local, limited_feedforward, tick_delta_ms, tick_skew_ms)
import batch_collect
import wrist_supplement_plan
from collect import Link, parse_telemetry, validate_config, run
from demo_data import generate
from fit_wrist import (assess_speed_term, discover_runs, even_design,
                       evaluate_design, evaluate_joint_phase_motor_current,
                       evaluate_motor_phase_current, fit_group, group_runs,
                       motor_phase_parameters, normalize_payload_description,
                       odd_design, pair_csv, round_half_away_from_zero,
                       MANAGED_BY, OUTPUT_MANIFEST, write_managed_outputs)
from gravity_preview import GravityTorqueEvaluator
from pose_sweep_planner import (BASE_GRIDS, COLLISION_TOLERANCE_M,
                                DEFAULT_MAPPING, DEFAULT_MJCF,
                                PoseEvaluator, REFERENCE_FEASIBLE_POSE_DEG,
                                candidate_bases, order_safe_sequence,
                                transition_waypoints, verify_path)


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
            self.assertEqual(pairs.shape[1], 4)
            self.assertTrue(np.allclose(pairs[:, 3], 2.0, atol=0.01))
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

    def test_noisy_f1_velocity_does_not_break_pairing(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "test.csv"
            generate(path, 32)
            with path.open() as file:
                rows = list(csv.DictReader(file))
            for row in rows:
                row["dq_deg_s"] = "0"
            with path.open("w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=FIELDS)
                writer.writeheader()
                writer.writerows(rows)
            pairs, _ = make_pairs([path], 2, 3.0)
            coefficients, metrics = fit_local(pairs)
            np.testing.assert_allclose(coefficients, [0.3, 0.6, 0.05], atol=0.015)
            self.assertLess(metrics["train_rmse_a"], 0.003)
            with self.assertRaisesRegex(ValueError, "matched"):
                make_pairs([path], 2, 3.0, velocity_source="f1")

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

    def test_legacy_limits_and_wrist_strict_limits_are_separate(self):
        np.testing.assert_array_equal(JOINT_LIMITS_DEG, [
            [-170, 170], [-75, 90], [35, 180],
            [-180, 180], [-120, 120], [-720, 720],
        ])
        np.testing.assert_array_equal(WRIST_STRICT_LIMITS_DEG, [
            [-155, 155], [-70, 55], [40, 165],
            [-170, 170], [-100, 100], [-360, 360],
        ])
        self.assertEqual(WRIST_HISTORICAL_LIMITS_DEG[3].tolist(), [-170, 170])

    def test_current_scene_holding_torque_is_finite_and_observable(self):
        evaluator = GravityTorqueEvaluator(DEFAULT_MAPPING, DEFAULT_MJCF, gripper_rad=0.0)
        torque = evaluator.joint_torque(
            [-45.956, -1.406, 72.192, -60.0, -62.5, -329.467])
        self.assertEqual(torque.shape, (6,))
        self.assertTrue(np.isfinite(torque).all())
        self.assertGreater(abs(torque[3]), 0.01)
        self.assertEqual(COLLISION_TOLERANCE_M, 1e-9)
        pose_evaluator = PoseEvaluator(DEFAULT_MAPPING, DEFAULT_MJCF, 0.0)
        ok, tcp, collisions = pose_evaluator.evaluate(
            REFERENCE_FEASIBLE_POSE_DEG,
            limit_margin_deg=0.0)
        self.assertTrue(ok, collisions)
        self.assertGreater(tcp[0], -200)
        self.assertGreater(tcp[2], 5)
        closed_finger_contacts = []
        for index in range(pose_evaluator.data.ncon):
            contact = pose_evaluator.data.contact[index]
            bodies = {
                pose_evaluator.model.body(
                    int(pose_evaluator.model.geom_bodyid[contact.geom1])).name,
                pose_evaluator.model.body(
                    int(pose_evaluator.model.geom_bodyid[contact.geom2])).name,
            }
            if bodies == {"finger_left", "finger_right"}:
                closed_finger_contacts.append(float(contact.dist))
        self.assertTrue(closed_finger_contacts)
        self.assertGreaterEqual(min(closed_finger_contacts), -COLLISION_TOLERANCE_M)
        self.assertLess(max(closed_finger_contacts), 0.0)
        ok, _, collisions = pose_evaluator.evaluate(
            [-45.956, -1.406, 72.192, -60.0, -62.5, -329.467],
            limit_margin_deg=0.0)
        self.assertFalse(ok)
        self.assertTrue(any(
            frozenset(pair[:2]) == frozenset(("geom2", "geom6"))
            and pair[2] < -0.0008 for pair in collisions
        ))
        with self.assertRaisesRegex(ValueError, "Floor collision checks are mandatory"):
            PoseEvaluator(DEFAULT_MAPPING, DEFAULT_MJCF, 0.0, include_floor=False)
        self.assertIn(-90.0, BASE_GRIDS[1])
        self.assertIn(90.0, BASE_GRIDS[1])
        self.assertIn(-16.0, BASE_GRIDS[2])
        self.assertIn(110.0, BASE_GRIDS[3])
        candidates = candidate_bases(4, 1, 7, Path("/nonexistent"))
        self.assertEqual(candidates[0][0], [-90, -16, 110, 0, 0, 0])

    def test_safe_pose_order_keeps_required_source_first_and_checks_transitions(self):
        evaluator = PoseEvaluator(DEFAULT_MAPPING, DEFAULT_MJCF, 0.0)
        records = [
            {
                "joint": 5,
                "fixed_pose_deg": [-90, -16, 110, 0, -60, 0],
                "source": "grid:test",
            },
            {
                "joint": 4,
                "fixed_pose_deg": [-90, -16, 110, -60, 0, 0],
                "source": "required:positive-feasible",
            },
        ]
        ordered = order_safe_sequence(
            evaluator, records, REFERENCE_FEASIBLE_POSE_DEG)
        self.assertEqual(ordered[0]["source"], "required:positive-feasible")
        before = REFERENCE_FEASIBLE_POSE_DEG
        for record in ordered:
            verify_path(
                evaluator,
                before,
                transition_waypoints(before, record["fixed_pose_deg"]),
                "safe-order-test",
            )
            before = record["fixed_pose_deg"]

    def test_wrist_fit_recovers_weighted_terms_and_rejects_fake_speed_term(self):
        observations = []
        for pose_index in range(3):
            for speed in (0.5, 1.0, 2.0):
                for q in range(-50, 51, 5):
                    torque = 0.03 * (pose_index - 1) + 0.0003 * q
                    phase = np.deg2rad(30 * q)
                    even = 0.9 * torque + 0.01 + 0.02 * np.sin(phase) - 0.03 * np.cos(phase)
                    odd = 0.2 + 0.01 * speed + 0.04 * np.sin(phase) + 0.015 * np.cos(phase)
                    observations.append({
                        "run_id": f"pose{pose_index}-v{speed}",
                        "pose_id": f"P{pose_index}",
                        "q_deg": float(q),
                        "gravity_torque_nm": float(torque),
                        "commanded_speed_deg_s": float(speed),
                        "speed_magnitude_deg_s": float(speed),
                        "even_current_a": float(even),
                        "odd_current_a": float(odd),
                    })
        even_matrix, _ = even_design(observations)
        even_result = evaluate_design(
            observations, even_matrix,
            np.asarray([row["even_current_a"] for row in observations]))
        np.testing.assert_allclose(even_result["fit"]["coefficients"],
                                   [0.9, 0.01, 0.02, -0.03], atol=1e-12)
        self.assertLess(even_result["metrics"]["run_macro_rmse_a"], 1e-12)
        speed = assess_speed_term(observations)
        self.assertTrue(speed["included"])
        negative = [row.copy() for row in observations]
        for row in negative:
            phase = np.deg2rad(30 * row["q_deg"])
            row["odd_current_a"] = (
                0.2 - 0.01 * row["commanded_speed_deg_s"]
                + 0.04 * np.sin(phase) + 0.015 * np.cos(phase))
        negative_assessment = assess_speed_term(negative)
        self.assertFalse(negative_assessment["included"])
        self.assertIn("non_positive_viscous_term_not_deployable",
                      negative_assessment["reason"])
        for row in observations:
            row["commanded_speed_deg_s"] = 1.0
            row["speed_magnitude_deg_s"] = 1.0 + 0.0002 * (row["q_deg"] % 2)
        speed = assess_speed_term(observations)
        self.assertFalse(speed["included"])
        self.assertEqual(speed["unique_commanded_speed_levels"], 1)

    def test_motor_phase_export_is_numerically_equivalent_in_both_directions(self):
        self.assertEqual(round_half_away_from_zero(12.5), 13)
        self.assertEqual(round_half_away_from_zero(-12.5), -13)
        even_values = {
            "gravity_scale": {"value": 0.91},
            "bias": {"value": 0.004},
            "harmonic_sin": {"value": 0.023},
            "harmonic_cos": {"value": -0.017},
        }
        odd_values = {
            "odd_constant": {"value": 0.173},
            "odd_speed": {"value": 0.0025},
            "harmonic_sin": {"value": -0.031},
            "harmonic_cos": {"value": 0.019},
        }
        speed = {
            "included": True,
            "unique_commanded_speed_levels": 3,
        }
        motor_phase, blockers = motor_phase_parameters(
            5, -1, even_values, odd_values, speed)
        self.assertEqual(blockers, [])
        self.assertAlmostEqual(
            motor_phase["four_harmonics_mA"]["e_sin"]["unscaled_value"], 23.0)
        self.assertAlmostEqual(
            motor_phase["four_harmonics_mA"]["e_cos"]["unscaled_value"], 17.0)
        self.assertAlmostEqual(
            motor_phase["four_harmonics_mA"]["o_sin"]["unscaled_value"], 31.0)
        self.assertAlmostEqual(
            motor_phase["four_harmonics_mA"]["o_cos"]["unscaled_value"], 19.0)
        self.assertAlmostEqual(
            motor_phase["viscous_mA_per_motor_rev_s"]["unscaled_value"], 30.0)

        candidate = {
            "motor_phase": motor_phase,
            "model": {
                "joint_positive": {
                    "even_current_a": {
                        "gravity_scale_a_per_nm": 0.91,
                        "bias_a": 0.004,
                        "harmonic_sin_a": 0.023,
                        "harmonic_cos_a": -0.017,
                    },
                    "odd_current_a": {
                        "constant_a": 0.173,
                        "speed_coefficient_a_per_deg_s": 0.0025,
                        "harmonic_sin_a": -0.031,
                        "harmonic_cos_a": 0.019,
                    },
                },
            },
        }
        for q_deg in (-35.0, -7.5, 0.0, 12.5, 31.0):
            for direction_joint in (-1, 1):
                torque = 0.02 * q_deg - 0.001
                speed_joint = 1.25
                joint_phase = evaluate_joint_phase_motor_current(
                    candidate, q_deg, direction_joint, torque, speed_joint)
                motor_phase_current = evaluate_motor_phase_current(
                    candidate, q_deg, -direction_joint, torque,
                    speed_joint * 30.0 / 360.0)
                self.assertAlmostEqual(joint_phase, motor_phase_current, places=14)

        single_speed = {
            "included": False,
            "unique_commanded_speed_levels": 1,
        }
        j5_single, blockers = motor_phase_parameters(
            5, -1, even_values, odd_values, single_speed)
        self.assertFalse(j5_single["deployment"]["viscous_parameter_deployable"])
        self.assertIn("J5 single-speed data cannot deploy a viscous term", blockers)

    def test_payload_normalization_and_metadata_qualification(self):
        self.assertEqual(
            normalize_payload_description(
                "gripper, camera mounted, camera data cable unplugged, J4=-30"),
            "gripper,camera mounted,camera data cable unplugged")
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            good = root / "good.csv"
            bad = root / "bad.csv"
            generate(good, 1, speed=2.0)
            generate(bad, 2, speed=2.0)

            def sidecar(csv_path, acknowledged):
                record = {
                    "config": {
                        "joint": 2,
                        "pose_id": "P0",
                        "current_limit_a": 3.0,
                        "speed_deg_s": 2.0,
                        "firmware_revision": "test",
                        "payload_description": "gripper, camera",
                        "gripper_rad": 0.39,
                        "payload_identity": {
                            "camera_data_cable": "connected",
                        },
                    },
                    "completed": True,
                    "stop_acknowledged": acknowledged,
                }
                csv_path.with_suffix(".meta.json").write_text(json.dumps(record))

            sidecar(good, True)
            sidecar(bad, False)

            grippers = []

            class Torque:
                @staticmethod
                def joint_torque(q, gripper_rad=None):
                    grippers.append(gripper_rad)
                    return np.zeros(6)

            runs, exclusions, failures = discover_runs(
                root, {2}, Torque(), bin_deg=1.0, max_accel_deg_s2=2.0)
            self.assertEqual([run["csv_path"].name for run in runs], ["good.csv"])
            self.assertEqual(exclusions["stop_not_acknowledged"], 1)
            self.assertEqual(failures, [])
            self.assertTrue(grippers)
            self.assertTrue(all(value == 0.39 for value in grippers))

    def test_payload_identity_and_gripper_keep_fit_groups_separate(self):
        base_config = {
            "joint": 5,
            "firmware_revision": "build",
            "firmware_hash": "a" * 64,
            "payload_description": "gripper; camera mounted",
        }
        connected = {
            "config": {
                **base_config,
                "payload_identity": {"camera_data_cable": "connected"},
            },
            "gripper_rad": 0.0,
        }
        unplugged = {
            "config": {
                **base_config,
                "payload_identity": {"camera_data_cable": "unplugged"},
            },
            "gripper_rad": 0.0,
        }
        open_gripper = {
            "config": {
                **base_config,
                "payload_identity": {"camera_data_cable": "connected"},
            },
            "gripper_rad": 0.39,
        }
        groups = group_runs([connected, unplugged, open_gripper], {5})
        self.assertEqual(len(groups), 3)

    def test_managed_outputs_do_not_delete_paths_from_old_manifest(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            output = root / "output"
            output.mkdir()
            source = root / "source.csv"
            source.write_text("input\n")
            stale = output / "old.json"
            stale.write_text("old\n")
            (output / OUTPUT_MANIFEST).write_text(json.dumps({
                "managed_by": MANAGED_BY,
                "files": ["../source.csv", "old.json"],
            }))
            write_managed_outputs(output, {"new.json": "{}\n"})
            self.assertTrue(source.is_file())
            self.assertTrue(stale.is_file())
            self.assertTrue((output / "new.json").is_file())

    def test_candidate_group_and_pair_export(self):
        runs = []
        for pose_index in range(3):
            for speed in (0.5, 1.0, 2.0):
                observations = []
                for q in range(-30, 31, 5):
                    torque = 0.04 * (pose_index - 1) + 0.0002 * q
                    phase = np.deg2rad(30 * q)
                    even = 0.8 * torque + 0.02 * np.sin(phase) - 0.01 * np.cos(phase)
                    odd = 0.2 + 0.006 * speed + 0.03 * np.sin(phase) + 0.01 * np.cos(phase)
                    observations.append({
                        "run_id": f"P{pose_index}-v{speed}.csv",
                        "source": f"P{pose_index}-v{speed}.csv",
                        "pose_id": f"P{pose_index}",
                        "q_deg": float(q),
                        "q_all_deg": [0, q, 90, 0, 0, 0],
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
                    },
                    "pairs": np.zeros((len(observations), 4)),
                    "observations": observations,
                })
        candidate, summary = fit_group(2, "synthetic", "payload", runs, minimum_poses=3)
        self.assertIsNotNone(candidate)
        self.assertTrue(summary["candidate"])
        self.assertEqual(len(candidate["source_files"]), 9)
        self.assertTrue(candidate["speed_term_identifiability"]["included"])
        self.assertIn("motor_phase", candidate)
        self.assertIn("motor_phase", candidate["model"])
        self.assertEqual(candidate["motor_phase"]["gain_reference"]["gain_percent"], 100)
        self.assertIn("four_harmonics_mA", candidate["motor_phase"])
        exported = pair_csv(candidate, summary["observations"])
        self.assertTrue(exported.startswith("source,pose_id,commanded_speed_deg_s"))

    def test_independent_retest_is_evaluated_only_after_training_is_frozen(self):
        def make_runs(retest_offset=0.0):
            runs = []
            for pose_index in range(3):
                for speed in (0.5, 1.0, 2.0):
                    observations = []
                    measurement_round = (
                        "independent_retest"
                        if pose_index == 0 and speed == 0.5 else None)
                    for q in range(-30, 31, 5):
                        torque = 0.04 * (pose_index - 1) + 0.0002 * q
                        phase = np.deg2rad(30 * q)
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
                        if measurement_round:
                            even += retest_offset
                            odd -= 2.0 * retest_offset
                        observations.append({
                            "run_id": f"P{pose_index}-v{speed}.csv",
                            "source": f"P{pose_index}-v{speed}.csv",
                            "pose_id": f"P{pose_index}",
                            "q_deg": float(q),
                            "q_all_deg": [0, q, 90, 0, 0, 0],
                            "gravity_torque_nm": float(torque),
                            "speed_magnitude_deg_s": float(speed),
                            "even_current_a": float(even),
                            "odd_current_a": float(odd),
                            "commanded_speed_deg_s": float(speed),
                        })
                    config = {
                        "joint": 2,
                        "speed_deg_s": speed,
                        "current_limit_a": 3.0,
                        "payload_description": "synthetic payload",
                    }
                    if measurement_round:
                        config["measurement_round"] = measurement_round
                    runs.append({
                        "source": f"P{pose_index}-v{speed}.csv",
                        "metadata": f"P{pose_index}-v{speed}.meta.json",
                        "csv_sha256": "csv-hash",
                        "metadata_sha256": "meta-hash",
                        "config": config,
                        "info": {
                            "pose_id": f"P{pose_index}",
                            "raw_samples": len(observations),
                            "retained_samples": len(observations),
                        },
                        "pairs": np.zeros((len(observations), 4)),
                        "observations": observations,
                    })
            return runs

        baseline, baseline_summary = fit_group(
            2, "build", "synthetic payload", make_runs(), minimum_poses=3)
        changed, changed_summary = fit_group(
            2, "build", "synthetic payload", make_runs(0.1), minimum_poses=3)
        self.assertIsNotNone(baseline)
        self.assertIsNotNone(changed)
        self.assertEqual(baseline["coefficients"], changed["coefficients"])
        self.assertEqual(baseline["holdout"], changed["holdout"])
        self.assertEqual(baseline["source_files"], changed["source_files"])
        self.assertEqual(baseline["independent_retest"]["runs"], 1)
        self.assertEqual(baseline_summary["independent_retest"]["runs"], 1)
        self.assertFalse(any(
            record["measurement_round"] == "independent_retest"
            for record in baseline["source_files"]
        ))
        self.assertNotEqual(
            baseline["independent_retest"]["even"]["pooled_rmse_a"],
            changed["independent_retest"]["even"]["pooled_rmse_a"],
        )

    def test_wrist_supplement_defaults_to_current_mjcf_and_hashes_two_hex_files(self):
        self.assertFalse(hasattr(wrist_supplement_plan, "DEFAULT_ROUTES"))
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            f1_hex = root / "f1.hex"
            f4_hex = root / "f4.hex"
            output_dir = root / "plan"
            f1_hex.write_text(":01000000AA55\n")
            f4_hex.write_text(":020000040000FA\n")
            with patch.object(sys, "argv", [
                    "wrist_supplement_plan.py",
                    "--current-limit-a", "1.0",
                    "--output-dir", str(output_dir),
                    "--f1-hex", str(f1_hex),
                    "--f4-hex", str(f4_hex),
            ]), contextlib.redirect_stdout(io.StringIO()):
                wrist_supplement_plan.main()

            summary = json.loads((output_dir / "plan_summary.json").read_text())
            config_paths = sorted(
                path for path in output_dir.glob("*.json")
                if path.name != "plan_summary.json")
            self.assertEqual(len(config_paths), 24)
            self.assertEqual(summary["route_source"], "EXPECTED_ROUTES")
            self.assertIsNone(summary["verified_route_evidence"])
            self.assertTrue(all(
                not Path(name).is_absolute()
                for name in summary["model_sha256"]
            ))
            f1_sha256 = wrist_supplement_plan.sha256_file(f1_hex)
            f4_sha256 = wrist_supplement_plan.sha256_file(f4_hex)
            canonical_pair = json.dumps(
                {"F1_HEX": f1_sha256, "F4_HEX": f4_sha256},
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
            expected_pair = hashlib.sha256(canonical_pair).hexdigest()
            self.assertEqual(summary["firmware_pair_sha256"], expected_pair)
            for path in config_paths:
                config = json.loads(path.read_text())
                validate_config(config, execute=False)
                self.assertEqual(config["firmware_hash"], expected_pair)
                self.assertEqual(
                    config["firmware_artifacts"]["F1_HEX"]["sha256"], f1_sha256)
                self.assertNotIn("source_route_verification", config)
            generated_text = "\n".join(
                path.read_text() for path in output_dir.iterdir() if path.is_file())
            self.assertNotIn("/home/xiaohai", generated_text)

    def test_wrist_supplement_without_hex_generates_pending_review_configs(self):
        config_fields, summary_fields = (
            wrist_supplement_plan.build_firmware_metadata(None, None))
        planned = wrist_supplement_plan.build_configs(
            config_fields, current_limit_a=1.0, cycles=3, sample_hz=50,
            evidence_reference=None)
        self.assertEqual(len(planned), 24)
        self.assertEqual(summary_fields["firmware_artifacts"], {})
        self.assertTrue(all(
            config["firmware_hash"] ==
            wrist_supplement_plan.FIRMWARE_HASH_PLACEHOLDER
            for _filename, config in planned
        ))

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
        example_path = Path(__file__).resolve().parents[1] / "config.example.json"
        example = json.loads(example_path.read_text())
        self.assertEqual(validate_config(example), 3)
        outside_example = json.loads(example_path.read_text())
        outside_example["sweep_min_deg"] = 34
        with self.assertRaisesRegex(ValueError, "outside"):
            validate_config(outside_example)
        ordinary_j4 = json.loads(example_path.read_text())
        ordinary_j4.update(
            joint=4,
            fixed_pose_deg=[-90, -16, 110, -175, 0, 0],
            sweep_min_deg=-175,
            sweep_max_deg=0,
        )
        self.assertEqual(validate_config(ordinary_j4), 4)
        strict_j4 = json.loads(example_path.read_text())
        strict_j4.update(
            joint=4,
            fixed_pose_deg=[-90, -16, 110, -170, 0, 0],
            sweep_min_deg=-170,
            sweep_max_deg=170,
            gripper_rad=0.0,
            safety_profile=WRIST_STRICT_PROFILE,
        )
        self.assertEqual(validate_config(strict_j4), 4)
        strict_j4["sweep_min_deg"] = -171
        strict_j4["fixed_pose_deg"][3] = -171
        with self.assertRaisesRegex(ValueError, "outside"):
            validate_config(strict_j4)
        strict_j4 = json.loads(example_path.read_text())
        strict_j4.update(
            joint=4,
            fixed_pose_deg=[-90, -16, 110, -170, 0, 0],
            sweep_min_deg=-170,
            sweep_max_deg=170,
            safety_profile=WRIST_STRICT_PROFILE,
        )
        strict_j4.pop("gripper_rad")
        with self.assertRaisesRegex(ValueError, "gripper_rad"):
            validate_config(strict_j4)
        config = json.loads(example_path.read_text())
        self.assertEqual(validate_config(config, execute=True), 3)
        legacy_wrist = json.loads(example_path.read_text())
        legacy_wrist.update(
            joint=4,
            fixed_pose_deg=[-90, -16, 110, -60, 0, 0],
            sweep_min_deg=-60,
            sweep_max_deg=40,
        )
        self.assertEqual(validate_config(legacy_wrist), 4)
        with self.assertRaisesRegex(ValueError, "requires safety_profile"):
            validate_config(legacy_wrist, execute=True)
        strict_execute = json.loads(example_path.read_text())
        strict_execute.update(
            joint=4,
            fixed_pose_deg=[-90, -16, 110, 0, -60, 0],
            sweep_min_deg=-40,
            sweep_max_deg=40,
            gripper_rad=0.0,
            safety_profile=WRIST_STRICT_PROFILE,
        )
        with self.assertRaisesRegex(ValueError, "firmware_hash"):
            validate_config(strict_execute, execute=True)
        strict_execute["firmware_hash"] = "a" * 64
        with self.assertRaisesRegex(ValueError, "current_limit_source"):
            validate_config(strict_execute, execute=True)
        strict_execute["current_limit_source"] = "DummyWeb #GETLIMIT readback"
        strict_execute["firmware_revision"] = "REPLACE_WITH_F1_F4_BUILD_REVISION"
        with self.assertRaisesRegex(ValueError, "firmware_revision"):
            validate_config(strict_execute, execute=True)
        strict_execute["firmware_revision"] = "F1/F4 2026-11-27 build"
        self.assertEqual(validate_config(strict_execute, execute=True), 4)
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

    def test_path_verification_checks_every_segment(self):
        class Evaluator:
            collision_tolerance_m = 0.001

            @staticmethod
            def evaluate(q, limit_margin_deg=0.0):
                if q[3] >= 10:
                    return False, np.array([0.0, 0.0, 100.0]), [
                        ("wrist1_link", "wrist2_link", -0.002)]
                return True, np.array([0.0, 0.0, 100.0]), []

        start = [0, 0, 90, 0, 0, 0]
        with self.assertRaisesRegex(RuntimeError, "MJCF path blocked"):
            verify_path(Evaluator(), start, [[0, 0, 90, 20, 0, 0]], "test", step_deg=1)

    def test_batch_scan_accepts_any_in_interval_start_and_preflights_approach(self):
        config = {
            "pose_id": "TEST",
            "joint": 4,
            "fixed_pose_deg": [-90, -16, 110, 20, -60, 0],
            "sweep_min_deg": -40,
            "sweep_max_deg": 40,
            "cycles": 2,
        }
        waypoints = batch_collect.scan_waypoints(config)
        self.assertEqual(
            [point[3] for point in waypoints],
            [-40, 40, -40, -40, 40, -40],
        )
        config["fixed_pose_deg"][3] = 50
        with self.assertRaisesRegex(RuntimeError, "outside"):
            batch_collect.scan_waypoints(config)

    def test_batch_refuses_plan_mismatch_and_failed_previous_stop_before_enable(self):
        class RobotLink:
            def __init__(self):
                self.commands = []

            def request(self, command, prefix, timeout=0.5, retries=1, joint=None):
                self.commands.append(command)
                if command == "#GETSTATUS":
                    return "oks 0 2 0 0 0 0 0 0 0".split()
                raise AssertionError(command)

        class SafeEvaluator:
            collision_tolerance_m = COLLISION_TOLERANCE_M

            @staticmethod
            def evaluate(q, limit_margin_deg=0.0):
                return True, np.array([0.0, 0.0, 100.0]), []

        planned = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
        robot = RobotLink()
        with patch.object(
                batch_collect,
                "read_fresh_pose",
                return_value=[0.2, 0.0, 90.0, 0.0, 0.0, 0.0]):
            with self.assertRaisesRegex(RuntimeError, "reviewed route"):
                batch_collect.move_to(
                    robot,
                    [1.0, 0.0, 90.0, 0.0, 0.0, 0.0],
                    5.0,
                    0.5,
                    SafeEvaluator(),
                    expected_start=planned,
                )
        self.assertNotIn("!START", robot.commands)

        robot = RobotLink()
        robot.stop_failed = True
        with self.assertRaisesRegex(RuntimeError, "previous !STOP"):
            batch_collect.move_to(
                robot,
                [1.0, 0.0, 90.0, 0.0, 0.0, 0.0],
                5.0,
                0.5,
                SafeEvaluator(),
                expected_start=planned,
            )
        self.assertNotIn("!START", robot.commands)

    def test_batch_rejects_unsafe_start_before_enable(self):
        class RobotLink:
            def __init__(self):
                self.commands = []

            def request(self, command, prefix, timeout=0.5, retries=1, joint=None):
                self.commands.append(command)
                if command == "#GETSTATUS":
                    return "oks 0 2 0 0 0 0 0 0 0".split()
                raise AssertionError(command)

        class UnsafeEvaluator:
            collision_tolerance_m = COLLISION_TOLERANCE_M

            @staticmethod
            def evaluate(q, limit_margin_deg=0.0):
                return False, np.array([0.0, 0.0, 4.0]), [
                    ("base_link", "upper_link", -0.00095)]

        robot = RobotLink()
        config = {
            "pose_id": "TEST",
            "joint": 4,
            "fixed_pose_deg": [-90, -16, 110, -60, 0, 0],
            "sweep_min_deg": -60,
            "sweep_max_deg": 60,
            "cycles": 1,
        }
        with patch.object(
                batch_collect, "read_fresh_pose",
                return_value=[-90, -16, 110, 0, 0, 0]):
            with self.assertRaisesRegex(RuntimeError, "unsafe actual start"):
                batch_collect.position_for_config(
                    robot, config, UnsafeEvaluator(), speed=5.0, tolerance=0.5)
        self.assertNotIn("!START", robot.commands)

    def test_batch_preflights_scan_before_enable_and_moves_one_axis_at_a_time(self):
        class RobotLink:
            def __init__(self):
                self.commands = []

            def request(self, command, prefix, timeout=0.5, retries=1, joint=None):
                self.commands.append(command)
                if command == "#GETSTATUS":
                    return "oks 0 2 0 0 0 0 0 0 0".split()
                raise AssertionError(command)

        class SafeEvaluator:
            collision_tolerance_m = COLLISION_TOLERANCE_M

            @staticmethod
            def evaluate(q, limit_margin_deg=0.0):
                if q[3] >= 30:
                    return False, np.array([0.0, 0.0, 100.0]), [
                        ("wrist1_link", "wrist2_link", -0.002)]
                return True, np.array([0.0, 0.0, 100.0]), []

        robot = RobotLink()
        start = [-90, -16, 110, 0, 0, 0]
        invalid_scan_config = {
            "pose_id": "TEST",
            "joint": 4,
            "fixed_pose_deg": [-90, -16, 110, 0, 0, 0],
            "sweep_min_deg": 0,
            "sweep_max_deg": 60,
            "cycles": 1,
        }
        with patch.object(batch_collect, "read_fresh_pose", return_value=start):
            with self.assertRaisesRegex(RuntimeError, "full-scan"):
                batch_collect.position_for_config(
                    robot, invalid_scan_config, SafeEvaluator(),
                    speed=5.0, tolerance=0.5)
        self.assertNotIn("!START", robot.commands)

        class AlwaysSafeEvaluator(SafeEvaluator):
            @staticmethod
            def evaluate(q, limit_margin_deg=0.0):
                return True, np.array([0.0, 0.0, 100.0]), []

        current = start.copy()
        target_config = {
            "pose_id": "TEST",
            "joint": 4,
            "fixed_pose_deg": [5, -10, 100, 20, 30, 40],
            "sweep_min_deg": 20,
            "sweep_max_deg": 30,
            "cycles": 1,
        }
        moved_targets = []

        def fake_move(_link, target, _speed, _tolerance, _evaluator,
                      expected_start=None):
            moved_targets.append(target.copy())
            current[:] = target

        with patch.object(batch_collect, "read_fresh_pose",
                          side_effect=lambda _link: current.copy()), \
                patch.object(batch_collect, "move_to", side_effect=fake_move):
            batch_collect.position_for_config(
                RobotLink(), target_config, AlwaysSafeEvaluator(),
                speed=5.0, tolerance=0.5)
            for index, (before, after) in enumerate(
                    zip([start, *moved_targets[:-1]], moved_targets)):
                changed = [axis for axis, (a, b) in enumerate(zip(before, after)) if a != b]
                self.assertEqual(len(changed), 1, (index, before, after))
                expected_axis = batch_collect.TRANSITION_AXIS_ORDER[index]
                self.assertEqual(changed, [expected_axis])

    def test_batch_mode_mismatch_rejects_before_enable(self):
        class RobotLink:
            def __init__(self):
                self.commands = []

            def request(self, command, prefix, timeout=0.5, retries=1, joint=None):
                self.commands.append(command)
                if command == "#GETSTATUS":
                    return "oks 0 1 0 0 0 0 0 0 0".split()
                raise AssertionError(command)

        robot = RobotLink()
        with self.assertRaisesRegex(RuntimeError, "position mode 2"):
            batch_collect.preflight_for_config(robot, {}, object())
        self.assertNotIn("!START", robot.commands)

    def test_batch_axis_fault_rejects_before_enable(self):
        class RobotLink:
            def __init__(self):
                self.commands = []

            def request(self, command, prefix, timeout=0.5, retries=1, joint=None):
                self.commands.append(command)
                if command == "#GETSTATUS":
                    return "oks 0 2 0 0 0 1 0 0 0".split()
                raise AssertionError(command)

        robot = RobotLink()
        with self.assertRaisesRegex(RuntimeError, "axis fault"):
            batch_collect.preflight_for_config(robot, {}, object())
        self.assertNotIn("!START", robot.commands)

    def test_batch_deviation_and_stop_failure_are_raised(self):
        class RobotLink:
            def __init__(self, fail_stop=False, disable_after_start=False):
                self.commands = []
                self.fail_stop = fail_stop
                self.disable_after_start = disable_after_start

            def request(self, command, prefix, timeout=0.5, retries=1, joint=None):
                self.commands.append(command)
                if command == "#GETSTATUS":
                    enabled = (
                        1 if self.commands.count("!START")
                        and not self.disable_after_start else 0)
                    return f"oks {enabled} 2 0 0 0 0 0 0 0".split()
                if command in ("!START", "!STOP") or command.startswith(">"):
                    if command == "!STOP" and self.fail_stop:
                        raise RuntimeError("STOP lost")
                    return ["ok"]
                raise AssertionError(command)

        class SafeEvaluator:
            collision_tolerance_m = COLLISION_TOLERANCE_M

            @staticmethod
            def evaluate(q, limit_margin_deg=0.0):
                return True, np.array([0.0, 0.0, 100.0]), []

        start = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
        target = [1.0, 0.0, 90.0, 0.0, 0.0, 0.0]
        robot = RobotLink()
        poses = iter([
            start,
            start,
            [0.2, 1.0, 90.0, 0.0, 0.0, 0.0],
        ])
        with patch.object(batch_collect, "read_fresh_pose",
                          side_effect=lambda _link: next(poses).copy()), \
                patch.object(batch_collect.time, "sleep", return_value=None):
            with self.assertRaisesRegex(RuntimeError, "deviated from"):
                batch_collect.move_to(robot, target, 5.0, 0.5, SafeEvaluator())
        self.assertEqual(robot.commands[-1], "!STOP")

        robot = RobotLink(fail_stop=True)
        poses = iter([start, start, target])
        with patch.object(batch_collect, "read_fresh_pose",
                          side_effect=lambda _link: next(poses).copy()), \
                patch.object(batch_collect.time, "sleep", return_value=None):
            with self.assertRaisesRegex(RuntimeError, "STOP lost"):
                batch_collect.move_to(robot, target, 5.0, 0.5, SafeEvaluator())

        robot = RobotLink(fail_stop=True, disable_after_start=True)
        poses = iter([start, start, target])
        with patch.object(batch_collect, "read_fresh_pose",
                          side_effect=lambda _link: next(poses).copy()), \
                patch.object(batch_collect.time, "sleep", return_value=None):
            with self.assertRaisesRegex(RuntimeError, "final !STOP also failed"):
                batch_collect.move_to(robot, target, 5.0, 0.5, SafeEvaluator())

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
        config.update(joint=2, fixed_pose_deg=[0, 0, 90, 0, 0, 0],
                      cycles=1, sweep_min_deg=-1, sweep_max_deg=1,
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
        for axis in range(1, 7):
            self.assertGreaterEqual(
                robot.commands.count(f"#GETTELEM {axis}"), 2,
                f"J{axis} was not repeatedly polled with fresh telemetry")

        class StillEnabledRobot(Robot):
            def request(self, command, prefix, timeout=0.5, retries=0, joint=None):
                if command == "#GETSTATUS":
                    self.commands.append(command)
                    return "oks 1 2 0 0 0 0 0 0 0".split()
                return super().request(command, prefix, timeout, retries, joint)

        robot = StillEnabledRobot()
        with self.assertRaisesRegex(RuntimeError, "remained enabled"):
            run(robot, config, writer, file)
        self.assertNotIn("!START", robot.commands)

        robot = Robot()
        robot.q[2] = 92
        with self.assertRaisesRegex(RuntimeError, "fixed pose"):
            run(robot, config, writer, file)
        self.assertNotIn("!START", robot.commands)

        robot = Robot()
        robot.q[1] = 91
        with self.assertRaisesRegex(RuntimeError, "outside the qualified limits"):
            run(robot, config, writer, file)
        self.assertNotIn("!START", robot.commands)

        class UnsafeEvaluator:
            @staticmethod
            def evaluate(q, limit_margin_deg=0.0):
                return False, np.array([0.0, 0.0, 4.0]), [
                    ("upper_link", "forearm_link", -0.002)]

        robot = Robot()
        with self.assertRaisesRegex(RuntimeError, "Unsafe actual start"):
            run(robot, config, writer, file, safety_evaluator=UnsafeEvaluator())
        self.assertNotIn("!START", robot.commands)


if __name__ == "__main__":
    unittest.main()
