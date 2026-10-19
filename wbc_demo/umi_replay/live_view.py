import csv
import json
import math
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from .camera_views import VIEW_NAMES, VIEW_TOPICS, ego_camera, look_at, source_video_info
from .download import sha256
from .geometry import pose_dict, pose_matrix

IMAGE_QOS = QoSProfile(depth=3, reliability=ReliabilityPolicy.RELIABLE)


class ViewNode(Node):
    def __init__(self, audit):
        super().__init__("umi_live_multiview")
        self.track = audit["head_track"]
        self.world = np.array(audit["world_to_odom"])
        self.render_root = audit["render_robot_root"]
        self.source_time = 0.0
        self.stage = "waiting_for_operator"
        self.metrics = {}
        self.frames = {}
        self.pending = {}
        self.bundle = None
        self.bundle_time = None
        self.received = {name: 0 for name in VIEW_NAMES}
        self.received_at = {name: 0.0 for name in VIEW_NAMES}
        self.orbits = [[math.pi / 4, 0.35, 2.4], [-math.pi / 2, 0.25, 1.9]]
        self.camera_publisher = self.create_publisher(TFMessage, "/tf_render", 10)
        self.control_publisher = self.create_publisher(String, "/umi_replay/control", 10)
        self.create_subscription(String, "/umi_replay/progress", self.on_progress, 10)
        for name, topic in zip(VIEW_NAMES, VIEW_TOPICS):
            self.create_subscription(Image, topic, lambda message, view=name: self.on_image(view, message), IMAGE_QOS)
        self.create_timer(1 / 30, self.move_cameras)

    @property
    def source_time(self):
        phase = self._source_time
        if self.stage == "replaying" and self.metrics.get("playback_rate") == 1.0:
            phase += max(0.0, time.monotonic() - self.metrics["phase_monotonic_s"])
            phase = min(phase, self.metrics["source_duration_s"])
        return phase

    @source_time.setter
    def source_time(self, value):
        self._source_time = value
        self.metrics = {}

    def on_progress(self, message):
        value = json.loads(message.data)
        self.source_time = float(value["source_time_s"])
        self.stage = value["stage"]
        self.metrics = {key: item for key, item in value.items() if key not in ("source_time_s", "stage")}

    def on_image(self, name, message):
        conversions = {"rgb8": (3, cv2.COLOR_RGB2BGR), "rgba8": (4, cv2.COLOR_RGBA2BGR), "bgr8": (3, None)}
        if message.encoding not in conversions:
            raise RuntimeError(f"Unsupported rendered image encoding: {message.encoding}")
        channels, conversion = conversions[message.encoding]
        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        frame = rows[:, : message.width * channels].reshape(message.height, message.width, channels)
        if conversion is not None:
            frame = cv2.cvtColor(frame, conversion)
        self.frames[name] = frame.copy()
        self.received[name] += 1
        self.received_at[name] = time.monotonic()
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        self.pending.setdefault(stamp, {})[name] = frame
        if len(self.pending[stamp]) == 3:
            self.bundle = np.hstack([self.pending[stamp][view] for view in VIEW_NAMES])
            self.bundle_time = stamp * 1e-9
        while len(self.pending) > 32:
            del self.pending[next(iter(self.pending))]

    def cameras_ready(self):
        return self.bundle is not None and all(time.monotonic() - stamp < 2.0 for stamp in self.received_at.values())

    def move_cameras(self):
        poses = [ego_camera(self.track, self.source_time, self.world)]
        center = np.array([-0.25, 0, 0.95])
        for azimuth, elevation, distance in self.orbits:
            position = center + distance * np.array(
                [math.cos(elevation) * math.cos(azimuth), math.cos(elevation) * math.sin(azimuth), math.sin(elevation)]
            )
            poses.append(look_at(position, center))
        transforms = []
        stamp = self.get_clock().now().to_msg()
        paths = ["/RenderOVRTX/Cameras/" + name for name in VIEW_NAMES] + [self.render_root["prim_path"]]
        poses.append(pose_matrix(self.render_root["pose"]))
        for path, matrix in zip(paths, poses):
            value = pose_dict(matrix)
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = "world"
            transform.child_frame_id = path
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = (
                value["position"]
            )
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ) = value["quaternion_xyzw"]
            transforms.append(transform)
        self.camera_publisher.publish(TFMessage(transforms=transforms))


class LiveSession:
    def __init__(self, args, converted):
        self.args = args
        self.converted = converted
        self.audit = json.loads(args.scene.with_suffix(".json").read_text())
        if sha256(args.scene) != self.audit["scene_sha256"]:
            raise ValueError("Camera scene differs from its recorded audit")
        if json.loads(args.config.read_text()) != self.audit.get("alignment_config"):
            raise ValueError("Camera and replay configs differ; regenerate the view scene with the same --config")
        if self.audit["source_provenance"] != converted["provenance"]:
            raise ValueError("Camera scene and EEF source differ")
        args.output.mkdir(parents=True, exist_ok=False)
        self.node = ViewNode(self.audit)
        self.source_path, self.source_start, self.source_fps = source_video_info(args.root, converted)
        self.process = None
        self.process_log = None
        self.run_count = 0
        self.active_policy = args.posture_policy
        self.active_mode = args.playback_mode
        self.run_directory = None
        self.writer = None
        self.record_frames = 0
        self.frame_trace = None
        self.last_record = 0.0
        self.last_preview = 0.0
        self.preview_received = {}
        self.running = True
        self.events = []
        self.jpeg = {}
        self.jpeg_versions = {}
        self.stream_condition = threading.Condition()
        self.requests = queue.Queue()
        self.state = {"stage": "waiting_for_cameras", "source_time_s": 0, "ready": False, "busy": False}
        self.autoplay_pending = args.autoplay

    def command(self, value):
        kind = value.get("command")
        if kind == "start":
            self.start_replay(value.get("policy", self.args.posture_policy), value.get("mode", self.args.playback_mode))
        elif kind in ("pause", "resume", "stop"):
            self.node.control_publisher.publish(String(data=kind))
        elif kind == "reset_views":
            self.node.orbits = [[math.pi / 4, 0.35, 2.4], [-math.pi / 2, 0.25, 1.9]]
        elif kind in ("orbit", "zoom"):
            view = int(value.get("view", -1))
            if view not in (0, 1):
                raise ValueError("Only the two external cameras can be manipulated")
            if kind == "orbit":
                horizontal, vertical = float(value.get("horizontal", 0)), float(value.get("vertical", 0))
                if not math.isfinite(horizontal + vertical) or max(abs(horizontal), abs(vertical)) > 1:
                    raise ValueError("Invalid camera drag")
                self.node.orbits[view][0] -= horizontal
                self.node.orbits[view][1] = float(np.clip(self.node.orbits[view][1] + vertical, -0.1, 1.3))
            else:
                factor = float(value.get("factor", 1))
                if not math.isfinite(factor) or not 0.1 <= factor <= 10:
                    raise ValueError("Invalid camera zoom")
                self.node.orbits[view][2] = float(np.clip(self.node.orbits[view][2] * factor, 0.45, 5))
        elif kind == "shutdown":
            self.running = False
        else:
            raise ValueError("Unknown web command")
        self.events.append({**value, "wall_time_s": time.time(), "source_time_s": self.node.source_time})
        self.state["command_count"] = len(self.events)
        self.state["last_command"] = kind

    def start_replay(self, policy, mode=None):
        if policy not in ("soft", "upright"):
            raise ValueError("Choose soft or upright explicitly")
        mode = self.args.playback_mode if mode is None else mode
        if mode not in ("realtime", "validated"):
            raise ValueError("Choose realtime or validated playback")
        if self.process is not None and self.process.poll() is None:
            return
        if not self.node.cameras_ready():
            raise RuntimeError("All three cameras must be ready before motion")
        self.run_count += 1
        self.active_policy = policy
        self.active_mode = mode
        self.node.stage = "initializing"
        self.node.source_time = 0.0
        self.run_directory = self.args.output / f"run-{self.run_count:03d}"
        self.record_path = self.args.output / f"recording-{self.run_count:03d}"
        self.record_path.mkdir()
        self.frame_trace = (self.record_path / "frames.csv").open("w")
        self.frame_writer = csv.writer(self.frame_trace)
        self.frame_writer.writerow(["video_frame", "capture_wall_time_s", "image_ros_time_s", "source_time_s"])
        self.writer = cv2.VideoWriter(
            str(self.record_path / "views.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 30, (1920, 480)
        )
        if not self.writer.isOpened():
            raise RuntimeError("Cannot create multi-view recording")
        self.record_frames = 0
        self.process_log = (self.args.output / f"run-{self.run_count:03d}.log").open("w")
        command = [
            sys.executable,
            str(Path(__file__).resolve().parents[1] / "umi_demo.py"),
            "replay",
            "--input",
            str(self.args.input),
            "--config",
            str(self.args.config),
            "--output",
            str(self.run_directory),
            "--execute",
            "--interactive-controls",
            "--posture-policy",
            policy,
            "--playback-mode",
            mode,
            "--limits-profile",
            str(self.args.limits_profile),
            "--kinematics-backend",
            getattr(self.args, "kinematics_backend", "urdf"),
        ]
        if getattr(self.args, "approach_reference", None):
            command.extend(["--approach-reference", str(self.args.approach_reference)])
        self.process = subprocess.Popen(
            command, stdout=self.process_log, stderr=self.process_log, start_new_session=True
        )

    def stop_recording(self):
        if self.writer is None:
            return
        self.writer.release()
        self.writer = None
        self.frame_trace.close()
        metadata = {
            "frames": self.record_frames,
            "video_fps": 30,
            "panels": list(VIEW_NAMES),
            "topics": list(VIEW_TOPICS),
            "source_provenance": self.converted["provenance"],
            "camera_audit": self.audit,
            "session_events": self.events,
            "frame_time_policy": "all three panels require the same rendered ROS image timestamp; repeated bundles allowed",
            "playback_mode": self.active_mode,
        }
        (self.record_path / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")

    def tick(self):
        for callback in range(12):
            rclpy.spin_once(self.node, timeout_sec=0)
        while not self.requests.empty():
            try:
                self.command(self.requests.get_nowait())
                self.state.pop("error", None)
            except (ValueError, RuntimeError) as error:
                self.state["error"] = str(error)
        if self.autoplay_pending and self.node.bundle is not None:
            self.autoplay_pending = False
            self.start_replay(self.args.posture_policy)
        now = time.monotonic()
        if now - self.last_preview >= 1 / 30:
            for name, frame in self.node.frames.items():
                if self.preview_received.get(name) == self.node.received[name]:
                    continue
                self.jpeg[name] = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()
                self.jpeg_versions[name] = self.jpeg_versions.get(name, 0) + 1
                self.preview_received[name] = self.node.received[name]
            with self.stream_condition:
                self.stream_condition.notify_all()
            self.last_preview = now
        if self.writer is not None and self.node.bundle is not None and now - self.last_record >= 1 / 30:
            self.writer.write(self.node.bundle)
            self.frame_writer.writerow([self.record_frames, time.time(), self.node.bundle_time, self.node.source_time])
            self.record_frames += 1
            self.last_record = now
            if self.record_frames % 30 == 0:
                self.frame_trace.flush()
                cv2.imwrite(str(self.record_path / "latest.png"), self.node.bundle)
        busy = self.process is not None and self.process.poll() is None
        if self.process is not None and not busy and self.writer is not None:
            self.stop_recording()
            self.process_log.close()
            if self.args.auto_close:
                self.running = False
        self.state.update(
            {
                "stage": self.node.stage if self.node.cameras_ready() or busy else "waiting_for_cameras",
                "source_time_s": self.node.source_time,
                "ready": self.node.cameras_ready(),
                "camera_warning": (
                    ""
                    if self.node.cameras_ready()
                    else "等待新渲染帧；请检查仿真/渲染后端日志，旧图片不代表服务仍正常。"
                ),
                "busy": busy,
                "run": self.run_count,
                "received": self.node.received.copy(),
                "record_frames": self.record_frames,
                "policy": self.active_policy,
                "playback_mode": self.active_mode,
                "source_duration_s": len(self.converted["source_frames"]) / self.source_fps,
                "source_video_start_s": self.source_start / self.source_fps,
                "playback_rate": self.node.metrics.get("playback_rate", 0.0),
                "deadline_misses": self.node.metrics.get("deadline_misses", 0),
                "render_bundle_age_s": (
                    max(0.0, self.node.get_clock().now().nanoseconds * 1e-9 - self.node.bundle_time)
                    if self.node.bundle_time is not None
                    else None
                ),
                "camera_orbits": [list(orbit) for orbit in self.node.orbits],
                "output": str(self.args.output),
            }
        )

    def close(self):
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=35)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=10)
        self.stop_recording()
        (self.args.output / "session.json").write_text(
            json.dumps({"events": self.events, "runs": self.run_count}, ensure_ascii=False, indent=2) + "\n"
        )
        self.node.destroy_node()


def live_handler(session, port):
    class Handler(BaseHTTPRequestHandler):
        def allowed(self):
            host = self.headers.get("Host")
            origin = self.headers.get("Origin")
            hosts = (f"127.0.0.1:{port}", f"localhost:{port}")
            return host in hosts and (origin is None or origin in tuple("http://" + value for value in hosts))

        def send_json(self, value, status=200):
            payload = json.dumps(value, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            if not self.allowed():
                self.send_json({"error": "This robot viewer is loopback-only"}, 403)
                return
            path = urlparse(self.path).path
            if path == "/":
                payload = Path(__file__).with_name("live_view.html").read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
            elif path == "/live_view.js":
                payload = Path(__file__).with_name("live_view.js").read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/javascript; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
            elif path == "/api/status":
                self.send_json(session.state.copy())
            elif path == "/source.mp4":
                size = session.source_path.stat().st_size
                start, end = 0, size - 1
                requested = self.headers.get("Range")
                if requested:
                    match = re.fullmatch(r"bytes=(\d+)-(\d*)", requested)
                    if match is None:
                        self.send_json({"error": "Only one explicit byte range is supported"}, 416)
                        return
                    start = int(match[1])
                    end = min(end, int(match[2])) if match[2] else end
                    if not 0 <= start <= end < size:
                        self.send_json({"error": "Byte range outside video"}, 416)
                        return
                self.send_response(206 if requested else 200)
                self.send_header("Content-Type", "video/mp4")
                self.send_header("Accept-Ranges", "bytes")
                self.send_header("Content-Length", str(end - start + 1))
                if requested:
                    self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
                self.end_headers()
                try:
                    with session.source_path.open("rb") as stream:
                        stream.seek(start)
                        remaining = end - start + 1
                        while remaining > 0:
                            payload = stream.read(min(remaining, 256 * 1024))
                            if not payload:
                                break
                            self.wfile.write(payload)
                            remaining -= len(payload)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            elif path.startswith("/stream/") and path.removeprefix("/stream/") in VIEW_NAMES:
                name = path.removeprefix("/stream/")
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                version = -1
                try:
                    while session.running:
                        with session.stream_condition:
                            session.stream_condition.wait_for(
                                lambda: (
                                    session.jpeg.get(name) is not None and session.jpeg_versions.get(name, 0) != version
                                )
                                or not session.running,
                                timeout=1,
                            )
                        payload = session.jpeg.get(name)
                        current = session.jpeg_versions.get(name, 0)
                        if payload is None or current == version:
                            continue
                        version = current
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                            + str(len(payload)).encode()
                            + b"\r\n\r\n"
                            + payload
                            + b"\r\n"
                        )
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
            elif path.startswith("/frame/") and path.removeprefix("/frame/") in VIEW_NAMES:
                payload = session.jpeg.get(path.removeprefix("/frame/"))
                if payload is None:
                    self.send_json({"error": "Camera not ready"}, 503)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
            else:
                self.send_json({"error": "Not found"}, 404)

        def do_POST(self):
            if not self.allowed() or self.path != "/api/control":
                self.send_json({"error": "Forbidden"}, 403)
                return
            try:
                length = int(self.headers.get("Content-Length", 0))
                if not 0 < length <= 4096:
                    raise ValueError("Invalid request length")
                value = json.loads(self.rfile.read(length))
                if not isinstance(value, dict) or value.get("command") not in (
                    "start",
                    "pause",
                    "resume",
                    "stop",
                    "orbit",
                    "zoom",
                    "shutdown",
                    "reset_views",
                ):
                    raise ValueError("Invalid control command")
                session.requests.put(value)
                self.send_json({"queued": True}, 202)
            except (ValueError, json.JSONDecodeError) as error:
                self.send_json({"error": str(error)}, 400)

        def log_message(self, format, *args):
            return

    return Handler


def live(args, converted):
    if not 1024 <= args.port <= 65535:
        raise ValueError("Choose an unprivileged local TCP port")
    os.environ.setdefault("ROS_DOMAIN_ID", "42")
    rclpy.init()
    session = LiveSession(args, converted)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), live_handler(session, args.port))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    handlers = {signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)}
    for signum in handlers:
        signal.signal(signum, lambda received, frame: setattr(session, "running", False))
    print(f"UMI live viewer: http://127.0.0.1:{args.port} — local-only Web UI, no Qt dependency", flush=True)
    try:
        while session.running and rclpy.ok():
            session.tick()
            time.sleep(0.003)
    finally:
        server.shutdown()
        server.server_close()
        session.close()
        for signum, handler in handlers.items():
            signal.signal(signum, handler)
        rclpy.shutdown()
