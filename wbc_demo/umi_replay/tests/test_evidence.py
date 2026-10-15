import csv
import json
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from umi_replay.evidence import contact_sheet, load_verified_run, sha256


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.report = {
            "status": "passed",
            "tracking_pose_frame": "gripper_eef",
            "gripper_eef_model": {"axis_revision": "camera_side_up_v2"},
            "return_home": {"status": "passed"},
            "replay": {"tracking_samples": 2},
        }
        side_metrics = {
            "samples": 2,
            "max_position_error_m": 0.002,
            "p95_position_error_m": 0.00195,
            "max_rotation_error_rad": 0.02,
            "source_time_start": 0.0,
            "source_time_end": 1.0,
        }
        self.metrics = {"sides": {"left": side_metrics.copy(), "right": side_metrics.copy()}}
        self.write_json()
        with (self.root / "eef_tracking.csv").open("w") as stream:
            writer = csv.writer(stream)
            writer.writerow(["source_time_s", "side", "eef_position_error_m", "eef_rotation_error_rad"])
            for side in ("left", "right"):
                writer.writerow([0, side, 0.001, 0.01])
                writer.writerow([1, side, 0.002, 0.02])

    def write_json(self):
        (self.root / "report.json").write_text(json.dumps(self.report))
        (self.root / "eef-metrics.json").write_text(json.dumps(self.metrics))

    def test_accepts_consistent_measured_metrics(self):
        report, metrics = load_verified_run(self.root)
        self.assertEqual(report, self.report)
        self.assertEqual(metrics, self.metrics)

    def test_rejects_historical_virtual_eef(self):
        self.report["tracking_pose_frame"] = "virtual_tool"
        self.write_json()
        with self.assertRaisesRegex(ValueError, "model gripper EEF"):
            load_verified_run(self.root)

    def test_rejects_failed_return_home(self):
        self.report["return_home"]["status"] = "failed"
        self.write_json()
        with self.assertRaisesRegex(ValueError, "return-home"):
            load_verified_run(self.root)

    def test_rejects_old_camera_side_definition(self):
        self.report["gripper_eef_model"].pop("axis_revision")
        self.write_json()
        with self.assertRaisesRegex(ValueError, "camera-side-inverted"):
            load_verified_run(self.root)

    def test_rejects_mismatched_metrics(self):
        self.metrics["sides"]["right"]["max_position_error_m"] = 0.1
        self.write_json()
        with self.assertRaisesRegex(ValueError, "right/max_position_error_m"):
            load_verified_run(self.root)

    def test_sheet_records_video_pts_not_source_phase(self):
        video = self.root / "test.mp4"
        writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"mp4v"), 10, (64, 48))
        self.assertTrue(writer.isOpened())
        for index in range(10):
            writer.write(np.full((48, 64, 3), index * 20, dtype=np.uint8))
        writer.release()
        output = self.root / "sheet.png"
        info = contact_sheet(video, output, 3, 3, "TEST")
        self.assertEqual([entry["video_frame"] for entry in info["thumbnails"]], [0, 4, 9])
        self.assertAlmostEqual(info["thumbnails"][-1]["video_time_s"], 0.9)
        with Image.open(output) as sheet:
            self.assertEqual(sheet.size, (1920, 518))
        self.assertEqual(len(sha256(output)), 64)


if __name__ == "__main__":
    unittest.main()
