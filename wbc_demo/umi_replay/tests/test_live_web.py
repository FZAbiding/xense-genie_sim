import queue
import threading
import unittest
from http.server import ThreadingHTTPServer
from types import SimpleNamespace
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from umi_replay.live_view import live_handler, ViewNode


class LiveWebTests(unittest.TestCase):
    def test_cached_frames_do_not_masquerade_as_a_live_renderer(self):
        node = SimpleNamespace(bundle=True, received_at={"UmiEgo": 99.0, "ExternalA": 99.0, "ExternalB": 99.0})
        with patch("umi_replay.live_view.time.monotonic", return_value=100.0):
            self.assertTrue(ViewNode.cameras_ready(node))
            node.received_at["ExternalB"] = 97.0
            self.assertFalse(ViewNode.cameras_ready(node))

    def setUp(self):
        self.session = SimpleNamespace(state={"ready": True}, requests=queue.Queue(), jpeg={})
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), live_handler(self.session, 0))
        self.port = self.server.server_address[1]
        self.server.RequestHandlerClass = live_handler(self.session, self.port)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)

    def test_status_endpoint_is_local_only(self):
        with urlopen(f"http://127.0.0.1:{self.port}/api/status") as response:
            self.assertEqual(response.status, 200)

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
