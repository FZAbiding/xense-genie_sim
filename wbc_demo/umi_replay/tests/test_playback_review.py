import unittest

from umi_replay.playback_review import browser_summary, stream_summary


class PlaybackReviewTests(unittest.TestCase):
    def test_ros_receive_rate_is_not_mjpeg_delivery_rate(self):
        probe = {
            "statuses": [
                {"monotonic_s": 100.0, "state": {"received": {"UmiEgo": 10}, "render_bundle_age_s": 0.02}},
                {"monotonic_s": 102.0, "state": {"received": {"UmiEgo": 70}, "render_bundle_age_s": 0.04}},
            ],
            "streams": [{"view": "UmiEgo", "delivered_fps": 20.0, "samples": [{"monotonic_s": 100.0}]}],
        }
        result = stream_summary(probe)
        self.assertEqual(result["received_fps"]["UmiEgo"], 30.0)
        self.assertEqual(result["streams"][0]["delivered_fps"], 20.0)
        self.assertNotIn("samples", result["streams"][0])

    def test_failed_stage_is_retained_and_not_used_as_replaying_clock_sample(self):
        probe = {
            "browser": "test",
            "presentations": [{"wall_ms": 1000}, {"wall_ms": 1033.3333333333333}],
            "samples": [
                {
                    "video_time_s": 3.01,
                    "state": {"stage": "replaying", "source_time_s": 1.0, "source_video_start_s": 2.0},
                },
                {"video_time_s": 3.1, "state": {"stage": "failed", "source_time_s": 1.0, "source_video_start_s": 2.0}},
            ],
            "events": [],
            "errors": [],
        }
        result = browser_summary(probe)
        self.assertAlmostEqual(result["presented_source_fps"], 30.0)
        self.assertAlmostEqual(result["sampled_source_phase_gap_ms_max"], 10.0)
        self.assertEqual(result["sampled_stages"], ["replaying", "failed"])
