import argparse
import copy
import json
import os
import queue
import time
from datetime import datetime

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from PyQt5 import QtCore, QtWidgets
from transforms3d.euler import euler2quat, quat2euler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

from dual_arm_ik import GROUP, position_error
from experiments import compare_groups, reach_certificate
from lab_core import (
    ARM_GROUP,
    ROOT,
    MotionCancelled,
    WholeBodyLab,
    finite_targets,
    make_targets,
    pose_values,
    save_json,
    timestamp,
)


def targets_from_values(values):
    targets = []
    for value in values:
        target = PoseStamped()
        target.header.frame_id = "odom"
        target.pose.position.x, target.pose.position.y, target.pose.position.z = value["position"]
        quaternion = value["quaternion_xyzw"]
        target.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
        targets.append(target)
    finite_targets(targets)
    return targets


class LabWorker(QtCore.QThread):
    update = QtCore.pyqtSignal(str, object)

    def __init__(self, output):
        super().__init__()
        self.output = output
        self.commands = queue.Queue()
        self.stop_requested = False
        self.generation = 0
        self.node = None
        self.prepared = None
        self.targets = None
        self.target_revision = 0
        self.report = {"status": "starting", "started_at": timestamp(), "events": []}

    def record(self, event, payload):
        self.report["events"].append({"time": timestamp(), "event": event, "data": payload})
        save_json(self.output / "report.json", self.report)
        self.update.emit(event, payload)
        print(event + ": " + json.dumps(payload, ensure_ascii=False)[:1200], flush=True)

    def feedback(self, index, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return
        self.targets[index].pose = copy.deepcopy(feedback.pose)
        self.target_revision += 1
        self.prepared = None
        self.record("marker_feedback", {"targets": pose_values(self.targets), "side": index})

    def set_targets(self, targets):
        finite_targets(targets)
        self.targets = copy.deepcopy(targets)
        self.target_revision += 1
        self.prepared = None
        for index, target in enumerate(targets):
            self.server.setPose(("left", "right")[index], target.pose)
        self.server.applyChanges()
        self.update.emit("targets", pose_values(targets))

    def create_markers(self):
        self.server = InteractiveMarkerServer(self.node, "wbc_demo_targets")
        for index, target in enumerate(self.targets):
            marker = InteractiveMarker()
            marker.header.frame_id = "odom"
            marker.name = ("left", "right")[index]
            marker.description = ("LEFT target", "RIGHT target")[index]
            marker.scale = 0.25
            marker.pose = target.pose
            sphere = Marker()
            sphere.type = Marker.SPHERE
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.045
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = (0.2, 0.7, 1.0, 0.9)
            visible = InteractiveMarkerControl(always_visible=True)
            visible.markers.append(sphere)
            marker.controls.append(visible)
            for axis, quaternion in (
                ("x", Quaternion(w=1.0)),
                ("y", Quaternion(z=2**-0.5, w=2**-0.5)),
                ("z", Quaternion(y=-(2**-0.5), w=2**-0.5)),
            ):
                for mode, interaction in (
                    ("move", InteractiveMarkerControl.MOVE_AXIS),
                    ("rotate", InteractiveMarkerControl.ROTATE_AXIS),
                ):
                    control = InteractiveMarkerControl()
                    control.name = mode + "_" + axis
                    control.orientation = quaternion
                    control.orientation_mode = InteractiveMarkerControl.FIXED
                    control.interaction_mode = interaction
                    marker.controls.append(control)
            self.server.insert(marker, feedback_callback=lambda feedback, index=index: self.feedback(index, feedback))
        self.server.applyChanges()
        self.feedback_publisher = self.node.create_publisher(
            InteractiveMarkerFeedback, "/wbc_demo_targets/feedback", 10
        )

    def observe(self, actual, targets):
        self.node.show_targets(targets, actual)
        positions = dict(zip(self.node.state.joint_state.name, self.node.state.joint_state.position))
        body = {name: value for name, value in positions.items() if "_body_joint" in name}
        errors = [position_error(first.pose, second.pose) * 1000 for first, second in zip(actual, targets)]
        self.update.emit("telemetry", {"wall_time": time.monotonic(), "errors_mm": errors, "body_rad": body})

    def dispatch(self, command):
        if self.stop_requested:
            return
        kind = command["kind"]
        if kind in ("preview", "execute", "home", "compare", "find") and command["generation"] != self.generation:
            self.record("cancelled", {"message": "Queued action invalidated by software stop"})
            return
        self.node.cancel.clear()
        if kind == "targets":
            self.set_targets(command["targets"])
            return
        if kind == "preset":
            self.set_targets(make_targets(self.node.reference, command["specification"], self.node.center))
            return
        if kind == "marker_test":
            feedback = InteractiveMarkerFeedback()
            feedback.header.frame_id = "odom"
            feedback.marker_name = "left"
            feedback.client_id = "dashboard-self-test"
            feedback.event_type = InteractiveMarkerFeedback.POSE_UPDATE
            feedback.pose = copy.deepcopy(self.targets[0].pose)
            feedback.pose.position.x += 0.05
            self.feedback_publisher.publish(feedback)
            self.record("marker_sent", {"subscribers": self.feedback_publisher.get_subscription_count()})
            return
        if kind == "self_test_passed":
            self.report["self_test"] = "passed"
            self.record("self_test_passed", {})
            return
        self.update.emit("busy", True)
        try:
            if kind == "preview":
                revision = self.target_revision
                prepared = self.node.prepare(copy.deepcopy(self.targets), command["group"])
                if revision != self.target_revision:
                    raise RuntimeError("Target changed during preview; solve again")
                self.prepared = prepared
                payload = {
                    key: self.prepared[key] for key in ("group", "ik_error_m", "solve_seconds", "planning_method")
                }
                self.record("preview", payload)
            elif kind == "execute":
                if self.prepared is None:
                    raise RuntimeError("Targets/group changed: preview must be recomputed before execution")
                prepared, self.prepared = self.prepared, None
                self.record("executed", self.node.execute_prepared(prepared))
            elif kind == "home":
                self.record("home", self.node.return_home())
                self.set_targets(self.node.reference)
            elif kind == "compare":
                seed = copy.deepcopy(self.node.state)
                result = {
                    "groups": {},
                    "targets": pose_values(self.targets),
                    "reach_certificate": reach_certificate(self.node, seed, self.targets),
                }
                self.prepared = None
                for group in (ARM_GROUP, GROUP):
                    try:
                        prepared = self.node.prepare(self.targets, group, seed=seed, plan=False)
                        result["groups"][group] = {
                            "status": "solved",
                            "seconds": prepared["solve_seconds"],
                            "ik_error_m": prepared["ik_error_m"],
                        }
                    except MotionCancelled:
                        raise
                    except RuntimeError as error:
                        result["groups"][group] = {"status": "rejected", "error": str(error)}
                self.record("comparison", result)
            elif kind == "find":
                comparison = {"comparison": []}
                found = compare_groups(self.node, comparison, self.output, execute_found=False)
                save_json(self.output / "counterexample.json", comparison)
                if not found:
                    raise RuntimeError("No certified counterexample found at the current waist pose")
                self.set_targets(targets_from_values(comparison["comparison"][-1]["targets"]))
                self.record("counterexample", comparison["comparison"][-1])
        except MotionCancelled as error:
            self.prepared = None
            self.record("cancelled", {"message": str(error)})
        except Exception as error:
            self.prepared = None
            self.record("rejected", {"action": kind, "error": str(error)})
        finally:
            self.node.cancel.clear()
            self.update.emit("busy", False)

    def run(self):
        try:
            self.node = WholeBodyLab()
            self.node.acquire_control()
            self.node.initialize()
            self.node.trace_file = (self.output / "joint_trace.csv").open("w", newline="")
            self.targets = copy.deepcopy(self.node.reference)
            self.create_markers()
            self.node.progress = self.observe
            self.report["status"] = "running"
            self.record("ready", pose_values(self.targets))
            last_observation = 0.0
            while rclpy.ok() and not self.stop_requested:
                rclpy.spin_once(self.node, timeout_sec=0.02)
                if self.stop_requested:
                    break
                try:
                    self.dispatch(self.commands.get_nowait())
                except queue.Empty:
                    pass
                if time.monotonic() - last_observation > 0.25:
                    self.observe(self.node.forward(self.node.state), self.targets)
                    last_observation = time.monotonic()
        except Exception as error:
            self.report["status"] = "failed"
            self.record("fatal", {"error": str(error)})
        finally:
            if self.node is not None:
                self.node.release()
                self.node.destroy_node()
            if self.report["status"] != "failed":
                self.report["status"] = {"passed": "passed", "failed": "failed"}.get(
                    self.report.get("self_test"), "closed"
                )
            self.report["finished_at"] = timestamp()
            save_json(self.output / "report.json", self.report)


class LabWindow(QtWidgets.QMainWindow):
    def __init__(self, output, self_test=False):
        super().__init__()
        self.output = output
        self.self_test = self_test
        self.test_stage = "starting"
        self.suppress_changes = True
        self.history = []
        self.setWindowTitle("Xense G2 全身 IK 实验台 — 仅仿真")
        self.resize(1180, 900)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.addWidget(
            QtWidgets.QLabel("目标坐标系：odom ｜ 拖动 RViz 的蓝色目标，或输入位姿；修改目标不会自动执行。")
        )
        self.group = QtWidgets.QComboBox()
        self.group.addItem("腰部解锁：5 腰部 + 14 双臂关节", GROUP)
        self.group.addItem("腰部锁定：仅求 14 双臂关节，腰保持原位置目标", ARM_GROUP)
        self.group.currentIndexChanged.connect(self.invalidate)
        layout.addWidget(self.group)
        self.spins = []
        forms = QtWidgets.QHBoxLayout()
        for side in ("左手", "右手"):
            box = QtWidgets.QGroupBox(side + "目标（绝对坐标）")
            form = QtWidgets.QFormLayout(box)
            values = []
            for index, label in enumerate(("X / m", "Y / m", "Z / m", "Roll / deg", "Pitch / deg", "Yaw / deg")):
                spin = QtWidgets.QDoubleSpinBox()
                spin.setRange(-5 if index < 3 else -180, 5 if index < 3 else 180)
                spin.setDecimals(3 if index < 3 else 1)
                spin.setSingleStep(0.02 if index < 3 else 5.0)
                spin.valueChanged.connect(self.schedule_targets)
                form.addRow(label, spin)
                values.append(spin)
            self.spins.append(values)
            forms.addWidget(box)
        layout.addLayout(forms)
        presets = QtWidgets.QHBoxLayout()
        self.preset = QtWidgets.QComboBox()
        self.specifications = yaml.safe_load((ROOT / "stress_targets.yaml").read_text())["targets"]
        for specification in self.specifications:
            self.preset.addItem(specification["name"])
        presets.addWidget(self.preset)
        preset_button = QtWidgets.QPushButton("载入大范围目标（不执行）")
        preset_button.clicked.connect(
            lambda: self.send("preset", specification=self.specifications[self.preset.currentIndex()])
        )
        presets.addWidget(preset_button)
        layout.addLayout(presets)
        buttons = QtWidgets.QGridLayout()
        self.preview = QtWidgets.QPushButton("1. 求解并检查路径")
        self.preview.clicked.connect(lambda: self.send("preview", group=self.group.currentData()))
        self.execute = QtWidgets.QPushButton("2. 执行已检查路径")
        self.execute.setEnabled(False)
        self.execute.clicked.connect(lambda: self.send("execute"))
        self.compare = QtWidgets.QPushButton("同目标锁腰 / 解锁对照（不执行）")
        self.compare.clicked.connect(lambda: self.send("compare"))
        find = QtWidgets.QPushButton("寻找有几何证明的腰部优势案例")
        find.clicked.connect(lambda: self.send("find"))
        for index, button in enumerate((self.preview, self.execute, self.compare, find)):
            buttons.addWidget(button, index // 2, index % 2)
        layout.addLayout(buttons)
        bottom = QtWidgets.QHBoxLayout()
        home = QtWidgets.QPushButton("回初始关节姿态（包含腰部）")
        home.clicked.connect(lambda: self.send("home"))
        stop = QtWidgets.QPushButton("停止 / 保持（软件停止，非硬件急停）")
        stop.setStyleSheet("background: #a12626; color: white; padding: 8px;")
        stop.clicked.connect(self.stop_motion)
        bottom.addWidget(home)
        bottom.addWidget(stop)
        layout.addLayout(bottom)
        self.status = QtWidgets.QLabel("等待仿真与 MoveIt 就绪…")
        self.status.setWordWrap(True)
        self.status.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        layout.addWidget(self.status)
        self.figure = Figure(figsize=(10, 3.2), tight_layout=True)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.error_axis, self.body_axis = self.figure.subplots(1, 2)
        layout.addWidget(self.canvas)
        self.console = QtWidgets.QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumHeight(140)
        layout.addWidget(self.console)
        self.debounce = QtCore.QTimer(self)
        self.debounce.setSingleShot(True)
        self.debounce.timeout.connect(self.send_targets)
        self.worker = LabWorker(output)
        self.worker.update.connect(self.on_update)
        self.worker.start()
        self.suppress_changes = False
        if self_test:
            QtCore.QTimer.singleShot(120000, self.test_timeout)

    def send(self, kind, **values):
        self.worker.commands.put({"kind": kind, "generation": self.worker.generation, **values})

    def invalidate(self):
        if not self.suppress_changes:
            self.execute.setEnabled(False)
            self.send_targets()

    def schedule_targets(self):
        if not self.suppress_changes:
            self.execute.setEnabled(False)
            self.debounce.start(150)

    def send_targets(self):
        values = []
        for spins in self.spins:
            coordinates = [spin.value() for spin in spins]
            quaternion = euler2quat(*[value * 3.141592653589793 / 180 for value in coordinates[3:]])
            values.append(
                {
                    "position": coordinates[:3],
                    "quaternion_xyzw": [float(value) for value in quaternion[1:]] + [float(quaternion[0])],
                }
            )
        self.send("targets", targets=targets_from_values(values))

    def display_targets(self, values):
        self.suppress_changes = True
        self.debounce.stop()
        for spins, value in zip(self.spins, values):
            quaternion = value["quaternion_xyzw"]
            angles = quat2euler([quaternion[3], *quaternion[:3]])
            for spin, coordinate in zip(
                spins, value["position"] + [angle * 180 / 3.141592653589793 for angle in angles]
            ):
                spin.setValue(coordinate)
        self.suppress_changes = False
        self.execute.setEnabled(False)

    def stop_motion(self):
        self.worker.generation += 1
        if self.worker.node is not None:
            self.worker.node.cancel.set()
        self.execute.setEnabled(False)
        self.status.setText("请求停止：执行循环会保持测量位置；求解服务调用需等有界返回。")

    def on_update(self, event, payload):
        if event == "telemetry":
            self.history.append(payload)
            self.history = self.history[-500:]
            if len(self.history) % 2 == 0:
                times = [value["wall_time"] - self.history[0]["wall_time"] for value in self.history]
                self.error_axis.clear()
                self.body_axis.clear()
                for index, label in enumerate(("left", "right")):
                    self.error_axis.plot(times, [value["errors_mm"][index] for value in self.history], label=label)
                for name in payload["body_rad"]:
                    self.body_axis.plot(
                        times,
                        [value["body_rad"][name] * 180 / 3.141592653589793 for value in self.history],
                        label=name[:5],
                    )
                self.error_axis.set_title("TCP target error (mm)")
                self.body_axis.set_title("Body joint angles (deg)")
                for axis in (self.error_axis, self.body_axis):
                    axis.set_xlabel("time (s)")
                    axis.legend(fontsize=7)
                    axis.grid(True, alpha=0.25)
                self.canvas.draw_idle()
            return
        if event == "busy":
            self.preview.setEnabled(not payload)
            self.compare.setEnabled(not payload)
            if payload:
                self.execute.setEnabled(False)
            return
        if event in ("ready", "targets"):
            self.display_targets(payload)
            self.status.setText("就绪：先求解并检查，确认后执行。")
            if event == "ready":
                QtCore.QTimer.singleShot(1500, lambda: self.grab().save(str(self.output / "dashboard-ready.png")))
        elif event == "marker_feedback":
            self.display_targets(payload["targets"])
            self.status.setText("RViz 目标已修改：不会自动执行，请重新预览。")
        elif event == "preview":
            self.execute.setEnabled(True)
            self.status.setText(f"检查通过：{payload['planning_method']}，IK 残差 {payload['ik_error_m']} m")
        elif event == "comparison":
            self.status.setText(
                "对照：" + " | ".join(f"{group}: {value['status']}" for group, value in payload["groups"].items())
            )
        elif event == "counterexample":
            self.group.setCurrentIndex(0)
            self.status.setText("已找到有几何证据的锁腰不可达案例；目标已载入，先预览再执行。")
        else:
            self.status.setText(event + ": " + str(payload)[:250])
        self.console.appendPlainText(event + ": " + json.dumps(payload, ensure_ascii=False)[:800])
        if self.self_test:
            self.advance_test(event, payload)

    def advance_test(self, event, payload):
        if event in ("fatal", "rejected"):
            self.worker.report["self_test"] = "failed"
            self.close()
        elif event == "ready":
            self.test_stage = "marker"
            QtCore.QTimer.singleShot(1500, lambda: self.send("marker_test"))
        elif event == "marker_feedback" and self.test_stage == "marker":
            self.test_stage = "preview_stop"
            QtCore.QTimer.singleShot(300, self.preview.click)
        elif event == "preview" and self.test_stage == "preview_stop":
            self.test_stage = "stopping"
            QtCore.QTimer.singleShot(100, self.execute.click)
            QtCore.QTimer.singleShot(700, self.stop_motion)
        elif event == "cancelled" and self.test_stage == "stopping":
            self.test_stage = "preview_execute"
            QtCore.QTimer.singleShot(300, self.preview.click)
        elif event == "preview" and self.test_stage == "preview_execute":
            self.test_stage = "executing"
            QtCore.QTimer.singleShot(100, self.execute.click)
        elif event == "executed" and self.test_stage == "executing":
            self.test_stage = "locked_preview"
            self.group.setCurrentIndex(1)
            self.spins[0][0].setValue(self.spins[0][0].value() + 0.05)
            self.spins[1][0].setValue(self.spins[1][0].value() + 0.05)
            QtCore.QTimer.singleShot(500, self.preview.click)
        elif event == "preview" and self.test_stage == "locked_preview":
            self.test_stage = "locked_executing"
            QtCore.QTimer.singleShot(100, self.execute.click)
        elif event == "executed" and self.test_stage == "locked_executing":
            if payload["group"] != ARM_GROUP or payload["locked_body_drift_rad"] > 0.015:
                self.test_timeout()
                return
            self.test_stage = "comparing"
            QtCore.QTimer.singleShot(300, self.compare.click)
        elif event == "comparison" and self.test_stage == "comparing":
            if not all(value["status"] == "solved" for value in payload["groups"].values()):
                self.test_timeout()
                return
            self.grab().save(str(self.output / "dashboard.png"))
            self.send("self_test_passed")
        elif event == "self_test_passed":
            self.test_stage = "passed"
            QtCore.QTimer.singleShot(300, self.close)

    def test_timeout(self):
        if self.test_stage != "passed":
            self.worker.report["self_test"] = "failed"
            self.worker.report["self_test_stage"] = self.test_stage
            self.console.appendPlainText("Self-test timeout/failure at " + self.test_stage)
            self.close()

    def closeEvent(self, event):
        if self.worker.isRunning():
            self.worker.stop_requested = True
            self.stop_motion()
            event.ignore()
            QtCore.QTimer.singleShot(200, self.close)
        else:
            event.accept()


def main():
    parser = argparse.ArgumentParser(description="Interactive fixed-base dual-arm + waist IK laboratory")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    output = ROOT / "logs" / ("lab-" + datetime.now().strftime("%Y%m%d-%H%M%S"))
    output.mkdir(parents=True, exist_ok=False)
    os.environ.setdefault("ROS_DOMAIN_ID", "42")
    rclpy.init()
    app = QtWidgets.QApplication([])
    window = LabWindow(output, args.self_test)
    window.show()
    app.exec()
    rclpy.shutdown()
    print(f"Interactive lab evidence: {output}", flush=True)
    if window.worker.report["status"] == "failed":
        raise SystemExit(1)
    if args.self_test and window.worker.report.get("self_test") != "passed":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
