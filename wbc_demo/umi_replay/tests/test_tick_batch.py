import copy
import csv
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from umi_replay.batch import frozen_config, jobs_for, run_batch
from umi_replay.kinematics import validate_backend
from umi_replay.tick_analysis import decompose_tick, audit_hold_transitions
from umi_replay.batch_evidence import export_batch


class TickBatchTests(unittest.TestCase):
    def test_hold_transition_cannot_hide_nominal_acceleration_jump(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            replay = {
                "active_joints": ["joint"],
                "planned_acceleration_limits_rad_s2": [3],
                "commands": 2,
                "deadline_misses": 1,
            }
            (root / "report.json").write_text(json.dumps({"replay": replay}))
            with (root / "realtime.csv").open("w") as stream:
                writer = csv.writer(stream)
                writer.writerow(["tick", "source_time_s", "published"])
                writer.writerows([[0, 0, True], [1, 1 / 30, True], [2, 2 / 30, False]])
            with (root / "command_joints.csv").open("w") as stream:
                writer = csv.writer(stream)
                writer.writerow(["source_time_s", "joint_velocity"])
                writer.writerows([[0, 0.1], [1 / 30, 0.2]])
            result = audit_hold_transitions(root)
            self.assertEqual(result["status"], "failed_nominal_acceleration")
            self.assertAlmostEqual(result["max_nominal_acceleration_excess_rad_s2"], 3)
            self.assertEqual(result["hold_ticks"], [2])

    def test_evidence_keeps_failed_and_not_run_rows_without_claiming_success(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            jobs = jobs_for([0, 1], ["urdf"], 1)
            jobs[0]["status"] = "failed"
            jobs[0]["error"] = "injected <guard>"
            (root / "summary.json").write_text(json.dumps({"status": "stopped_after_failure", "jobs": jobs}))
            result = export_batch(root, root / "evidence")
            self.assertEqual(len(result["rows"]), 2)
            page = (root / "evidence/index.html").read_text()
            self.assertIn("not_run", page)
            self.assertIn("&lt;guard&gt;", page)
            self.assertIn("<td>0</td>", page)

    def test_noncollinear_position_vectors_decompose_but_norms_do_not_add(self):
        def pose(position):
            return {"position": position, "quaternion_xyzw": [0, 0, 0, 1]}

        record = {
            "eef_target": [pose([1, 1, 0])] * 2,
            "eef_measured": [pose([0, 0, 0])] * 2,
            "eef_last_command": [pose([1, 0, 0])] * 2,
        }
        total, task, following = decompose_tick(record)
        np.testing.assert_allclose(total, task + following)
        self.assertNotAlmostEqual(np.linalg.norm(total[0]), np.linalg.norm(task[0]) + np.linalg.norm(following[0]))

    def test_nonfinite_pose_is_rejected(self):
        pose = {"position": [float("nan"), 0, 0], "quaternion_xyzw": [0, 0, 0, 1]}
        with self.assertRaises(ValueError):
            decompose_tick({key: [pose] * 2 for key in ("eef_target", "eef_measured", "eef_last_command")})

    def test_world_transform_is_frozen_without_mutating_input(self):
        transform = np.eye(4)
        transform[:3, 3] = [0.1, 0.2, 0.8]
        reference = {
            "status": "passed",
            "playback_mode": "realtime",
            "resolved_alignment": {"world_to_odom": transform.tolist()},
        }
        config = {"mode": "shared_world_gripper_eef", "initial_midpoint_odom_m": [0.3, 0, 1.05]}
        original = copy.deepcopy(config)
        fixed = frozen_config(reference, config)
        self.assertEqual(config, original)
        self.assertEqual(fixed["mode"], "calibrated")
        np.testing.assert_array_equal(fixed["world_to_odom"], transform)

    def test_backend_order_alternates_between_repeats(self):
        jobs = jobs_for([0, 1, 2], ["urdf", "pinocchio"], 2)
        self.assertEqual(len(jobs), 12)
        self.assertEqual(jobs[0]["backend"], "urdf")
        self.assertEqual(jobs[6]["backend"], "pinocchio")
        with self.assertRaises(ValueError):
            jobs_for([0, 0], ["urdf"], 1)

    def test_missing_optional_backend_is_not_silently_replaced(self):
        with patch("umi_replay.kinematics.importlib.import_module", side_effect=ModuleNotFoundError("pinocchio")):
            with self.assertRaises(ModuleNotFoundError):
                validate_backend("pinocchio")
            self.assertEqual(validate_backend("urdf"), "urdf")

    def test_motion_failure_stops_batch_and_retains_not_run_jobs(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            reference = root / "reference"
            reference.mkdir()
            (reference / "report.json").write_text(
                json.dumps(
                    {
                        "status": "passed",
                        "playback_mode": "realtime",
                        "resolved_alignment": {"world_to_odom": np.eye(4).tolist()},
                        "provenance": {"revision": "test"},
                    }
                )
            )
            for episode in (0, 1, 2):
                pose = {"position": [0, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]}
                (root / f"episode-{episode:03d}.json").write_text(
                    json.dumps(
                        {
                            "provenance": {"revision": "test"},
                            "source_frames": [
                                {"timestamp": index / 30, "eef": [pose, pose], "gripper": [0.5, 0.5]}
                                for index in range(2)
                            ],
                        }
                    )
                )
            config = root / "config.json"
            config.write_text("{}")
            limits = root / "limits.json"
            limits.write_text(
                json.dumps(
                    {group: {"velocity_rad_s": 0.1, "acceleration_rad_s2": 1} for group in ("waist", "arm", "gripper")}
                )
            )
            args = SimpleNamespace(
                reference_run=reference,
                config=config,
                limits_profile=limits,
                backends=["urdf"],
                episodes=[0, 1, 2],
                repeats=1,
                timeout=60,
                converted_directory=root,
                output=root / "batch",
                posture_policy="soft",
                execute=True,
            )

            def failed_child(command, log, timeout):
                directory = Path(command[command.index("--output") + 1])
                directory.mkdir()
                (directory / "report.json").write_text(
                    json.dumps({"status": "failed", "motion_started": True, "error": "injected tracking guard"})
                )
                return 1, False

            with patch("umi_replay.batch.run_child", side_effect=failed_child) as child:
                result = run_batch(args)
            child.assert_called_once()
            self.assertEqual(result["status"], "stopped_after_failure")
            self.assertEqual([job["status"] for job in result["jobs"]], ["failed", "not_run", "not_run"])
            args.output = root / "batch-audit-failure"

            def completed_child(command, log, timeout):
                directory = Path(command[command.index("--output") + 1])
                directory.mkdir()
                (directory / "report.json").write_text(
                    json.dumps({"status": "passed", "motion_started": True, "replay": {}})
                )
                return 0, False

            with patch("umi_replay.batch.run_child", side_effect=completed_child) as child, patch(
                "umi_replay.realtime_analysis.analyze_realtime", return_value={"status": "passed"}
            ), patch(
                "umi_replay.tick_analysis.audit_hold_transitions",
                return_value={"status": "failed_nominal_acceleration"},
            ):
                result = run_batch(args)
            child.assert_called_once()
            self.assertEqual(result["status"], "stopped_after_command_audit_failure")
            self.assertEqual([job["status"] for job in result["jobs"]], ["passed", "not_run", "not_run"])
