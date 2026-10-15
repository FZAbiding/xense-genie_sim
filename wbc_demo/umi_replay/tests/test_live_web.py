import queue
import threading
import tempfile
import unittest
from http.server import ThreadingHTTPServer
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from rclpy.qos import ReliabilityPolicy

from umi_replay.live_view import IMAGE_QOS, live_handler, ViewNode


class LiveWebTests(unittest.TestCase):
    def test_image_transport_requests_retransmission_with_bounded_history(self):
        self.assertEqual(IMAGE_QOS.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(IMAGE_QOS.depth, 3)

    def test_cached_frames_do_not_masquerade_as_a_live_renderer(self):
        node = SimpleNamespace(bundle=True, received_at={"UmiEgo": 99.0, "ExternalA": 99.0, "ExternalB": 99.0})
        with patch("umi_replay.live_view.time.monotonic", return_value=100.0):
            self.assertTrue(ViewNode.cameras_ready(node))
            node.received_at["ExternalB"] = 97.0
            self.assertFalse(ViewNode.cameras_ready(node))

    def setUp(self):
        self.session = SimpleNamespace(state={"ready": True}, requests=queue.Queue(), jpeg={})
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.session.source_path = Path(directory.name) / "source.mp4"
        self.session.source_path.write_bytes(b"0123456789")
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), live_handler(self.session, 0))
        self.port = self.server.server_address[1]
        self.server.RequestHandlerClass = live_handler(self.session, self.port)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)

    def test_status_endpoint_is_local_only(self):
        with urlopen(f"http://127.0.0.1:{self.port}/api/status") as response:
            self.assertEqual(response.status, 200)

    def test_native_video_supports_byte_ranges_for_browser_seeking(self):
        request = Request(f"http://127.0.0.1:{self.port}/source.mp4", headers={"Range": "bytes=3-6"})
        with urlopen(request) as response:
            self.assertEqual(response.status, 206)
            self.assertEqual(response.headers["Content-Range"], "bytes 3-6/10")
            self.assertEqual(response.read(), b"3456")

    def test_invalid_video_range_is_rejected(self):
        request = Request(f"http://127.0.0.1:{self.port}/source.mp4", headers={"Range": "bytes=12-"})
        with self.assertRaises(HTTPError) as error:
            urlopen(request)
        self.assertEqual(error.exception.code, 416)

    def test_realtime_camera_phase_keeps_wall_clock_time_between_messages(self):
        node = object.__new__(ViewNode)
        node._source_time = 2.0
        node.stage = "replaying"
        node.metrics = {"playback_rate": 1.0, "phase_monotonic_s": 100.0, "source_duration_s": 4.0}
        with patch("umi_replay.live_view.time.monotonic", return_value=101.0):
            self.assertEqual(node.source_time, 3.0)
            node.stage = "paused"
            self.assertEqual(node.source_time, 2.0)

    def test_frontend_script_is_served_not_a_local_file_only_page(self):
        with urlopen(f"http://127.0.0.1:{self.port}/live_view.js") as response:
            self.assertIn("javascript", response.headers["Content-Type"])
            script = response.read().decode()
            self.assertIn("image.draggable = false", script)
            self.assertIn("pointerdown", script)

    def test_reset_views_is_a_valid_queued_control(self):
        request = Request(f"http://127.0.0.1:{self.port}/api/control", data=b'{"command":"reset_views"}')
        with urlopen(request) as response:
            self.assertEqual(response.status, 202)
        self.assertEqual(self.session.requests.get_nowait()["command"], "reset_views")

    def test_external_origin_cannot_control_robot(self):
        request = Request(
            f"http://127.0.0.1:{self.port}/api/control",
            data=b'{"command":"start"}',
            headers={"Origin": "https://example.invalid"},
        )
        with self.assertRaises(HTTPError) as error:
            urlopen(request)
        self.assertEqual(error.exception.code, 403)
        self.assertTrue(self.session.requests.empty())

    def test_valid_control_is_queued_not_executed_in_http_thread(self):
        request = Request(f"http://127.0.0.1:{self.port}/api/control", data=b'{"command":"pause"}')
        with urlopen(request) as response:
            self.assertEqual(response.status, 202)
        self.assertEqual(self.session.requests.get_nowait(), {"command": "pause"})

    def test_invalid_control_is_rejected(self):
        request = Request(f"http://127.0.0.1:{self.port}/api/control", data=b'{"command":"delete"}')
        with self.assertRaises(HTTPError) as error:
            urlopen(request)
        self.assertEqual(error.exception.code, 400)


if __name__ == "__main__":
    unittest.main()
