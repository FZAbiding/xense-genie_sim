import csv
import json
import tempfile
import unittest
from pathlib import Path

from umi_replay.realtime_analysis import analyze_realtime


class RealtimeAuditTests(unittest.TestCase):
    def fixture(self, directory, velocity, misses):
        replay = {
            "active_joints": ["joint"],
            "planned_velocity_limits_rad_s": [0.1],
            "planned_acceleration_limits_rad_s2": [1.0],
            "joint_lower_limits_rad": [-1.0],
            "joint_upper_limits_rad": [1.0],
            "coupled_matrix": [[1.0]],
            "coupled_ceiling": [1.0],
            "commands": 2,
            "deadline_misses": misses,
            "skipped_source_ticks": 0,
            "source_duration_seconds": 1.0,
            "active_wall_seconds": 1.0,
        }
        (directory / "report.json").write_text(
            json.dumps({"status": "passed", "playback_mode": "realtime", "replay": replay})
        )
        with (directory / "command_joints.csv").open("w") as stream:
            writer = csv.writer(stream)
            writer.writerow(["source_time_s", "joint_position", "joint_velocity"])
            writer.writerows([[0, 0, 0], [1 / 30, 0, velocity]])
        with (directory / "realtime.csv").open("w") as stream:
            fields = [
                "source_time_s",
                "solve_ms",
                "cycle_ms",
                "position_error_left_m",
                "position_error_right_m",
                "rotation_error_left_rad",
                "rotation_error_right_rad",
                "roll_rad",
            ]
            writer = csv.writer(stream)
            writer.writerow(fields)
            writer.writerows([[0, 1, 2, 0, 0, 0, 0, 0], [1 / 30, 1, 2, 0, 0, 0, 0, 0]])

    def test_deadline_miss_does_not_disable_velocity_audit(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            self.fixture(directory, velocity=5.0, misses=1)
            with self.assertRaises(ValueError):
                analyze_realtime(directory)

    def test_nonfinite_command_is_not_reported_as_passed(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            self.fixture(directory, velocity=float("nan"), misses=0)
            with self.assertRaises(ValueError):
                analyze_realtime(directory)

    def test_nonfinite_tracking_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            self.fixture(directory, velocity=0.0, misses=0)
            path = directory / "realtime.csv"
            with path.open() as stream:
                reader = csv.DictReader(stream)
                fields = reader.fieldnames
                rows = list(reader)
            rows[0]["position_error_left_m"] = "nan"
            with path.open("w") as stream:
                writer = csv.DictWriter(stream, fields)
                writer.writeheader()
                writer.writerows(rows)
            with self.assertRaisesRegex(ValueError, "Nonfinite tracking"):
                analyze_realtime(directory)

    def test_nonfinite_constraint_metadata_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            self.fixture(directory, velocity=0.0, misses=0)
            path = directory / "report.json"
            report = json.loads(path.read_text())
            report["replay"]["planned_velocity_limits_rad_s"] = [float("nan")]
            path.write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError, "Nonfinite constraint metadata"):
                analyze_realtime(directory)

    def test_missing_command_rows_are_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            self.fixture(directory, velocity=0.0, misses=0)
            path = directory / "report.json"
            report = json.loads(path.read_text())
            report["replay"]["commands"] = 3
            path.write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError, "Command log length"):
                analyze_realtime(directory)

    def test_missed_deadline_marks_acceleration_audit_incomplete(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            self.fixture(directory, velocity=0.05, misses=1)
            metrics = analyze_realtime(directory)
            self.assertEqual(metrics["status"], "passed_with_incomplete_acceleration_audit")
            self.assertFalse(metrics["acceleration_audit_complete"])
