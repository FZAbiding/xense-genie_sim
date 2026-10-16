import argparse
import csv
import json
from pathlib import Path

import numpy as np

from .download import sha256


def planar_fk(joints, lengths=(0.5, 0.4)):
    shoulder, elbow = joints
    upper, forearm = lengths
    return np.array(
        [
            upper * np.cos(shoulder) + forearm * np.cos(shoulder + elbow),
            upper * np.sin(shoulder) + forearm * np.sin(shoulder + elbow),
        ]
    )


def planar_jacobian(joints, lengths=(0.5, 0.4)):
    shoulder, elbow = joints
    upper, forearm = lengths
    return np.array(
        [
            [-upper * np.sin(shoulder) - forearm * np.sin(shoulder + elbow), -forearm * np.sin(shoulder + elbow)],
            [upper * np.cos(shoulder) + forearm * np.cos(shoulder + elbow), forearm * np.cos(shoulder + elbow)],
        ]
    )


def planar_analytic(target, lengths=(0.5, 0.4)):
    horizontal, vertical = target
    upper, forearm = lengths
    radius_squared = horizontal**2 + vertical**2
    if radius_squared < 1e-24:
        raise ValueError("Origin is outside this tutorial's nondegenerate analytic example")
    cosine = (radius_squared - upper**2 - forearm**2) / (2 * upper * forearm)
    if abs(cosine) > 1 + 1e-12:
        raise ValueError("Target outside the two-link workspace")
    solutions = []
    for sign in (1, -1):
        elbow = sign * np.arccos(np.clip(cosine, -1, 1))
        shoulder = np.arctan2(vertical, horizontal) - np.arctan2(
            forearm * np.sin(elbow), upper + forearm * np.cos(elbow)
        )
        solutions.append(np.array([shoulder, elbow]))
    return solutions


def planar_iterative(target, initial, lengths=(0.5, 0.4), max_iterations=200):
    joints = np.asarray(initial, dtype=float).copy()
    history = [joints.copy()]
    for iteration in range(max_iterations):
        error = np.asarray(target) - planar_fk(joints, lengths)
        if np.linalg.norm(error) < 1e-9:
            return np.array(history)
        jacobian = planar_jacobian(joints, lengths)
        increment = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + 0.03**2 * np.eye(2), error)
        increment *= min(1.0, 0.2 / max(np.linalg.norm(increment), 1e-15))
        joints += increment
        history.append(joints.copy())
    raise RuntimeError("Teaching example did not converge; this is not proof of unreachability")


def recorded_metrics(reference_run):
    report = json.loads((reference_run / "report.json").read_text())
    if report.get("status") != "passed" or report.get("playback_mode") != "realtime":
        raise ValueError("A passed realtime run is required")
    with (reference_run / "realtime.csv").open() as stream:
        tracking = list(csv.DictReader(stream))
    with (reference_run / "command_joints.csv").open() as stream:
        commands = list(csv.DictReader(stream))
    frames = json.loads((reference_run / "mapped_targets.json").read_text())
    replay = report["replay"]
    names = replay["active_joints"]
    velocities = np.array([[float(row[name + "_velocity"]) for name in names] for row in commands])
    timestamps = np.array([frame["timestamp"] for frame in frames])
    if not np.allclose(np.diff(timestamps), 1 / 30, atol=2e-5) or len(commands) != replay["commands"]:
        raise ValueError("Expected complete 30 Hz source data and command logs")
    acceleration = np.diff(np.vstack([np.zeros(len(names)), velocities]), axis=0) * 30
    velocity_ratio = np.max(np.abs(velocities) / replay["planned_velocity_limits_rad_s"], axis=1)
    acceleration_ratio = np.max(np.abs(acceleration) / replay["planned_acceleration_limits_rad_s2"], axis=1)
    if not np.isfinite(np.r_[velocity_ratio, acceleration_ratio]).all():
        raise ValueError("Nonfinite command data")
    result = {
        "kind": "offline teaching material from existing v5 logs; no new robot execution",
        "reference_run": str(reference_run),
        "commands": len(commands),
        "deadline_misses": replay["deadline_misses"],
        "active_wall_seconds": replay["active_wall_seconds"],
        "velocity_near_limit_tick_fraction": float(np.mean(velocity_ratio >= 0.99)),
        "acceleration_near_limit_tick_fraction": float(np.mean(acceleration_ratio >= 0.99)),
        "near_limit_definition": "at least one command joint at >=99% of configured bound; not KKT multiplier attribution",
        "acceleration_definition": "discrete command delta velocity / (1/30 s), not measured physical acceleration",
        "per_side": {},
        "inputs_sha256": {
            filename: sha256(reference_run / filename)
            for filename in ("report.json", "realtime.csv", "command_joints.csv", "mapped_targets.json")
        },
    }
    for side, label in enumerate(("left", "right")):
        errors = np.array([float(row[f"position_error_{label}_m"]) for row in tracking])
        points = np.array([frame["eef_odom"][side]["position"] for frame in frames])
        speeds = np.linalg.norm(np.diff(points, axis=0), axis=1) / np.diff(timestamps)
        maximum = int(errors.argmax())
        result["per_side"][label] = {
            "position_error_m_p50_p95_max": np.quantile(errors, [0.5, 0.95, 1]).tolist(),
            "max_position_error_phase_s": float(tracking[maximum]["source_time_s"]),
            "source_speed_m_s_p50_p95_max": np.quantile(speeds, [0.5, 0.95, 1]).tolist(),
        }
    for field in ("feedback_age_s", "solve_ms", "cycle_ms"):
        values = np.array([float(row[field]) for row in tracking])
        result[field + "_p50_p95_max"] = np.quantile(values, [0.5, 0.95, 1]).tolist()
    return result, tracking, frames, acceleration_ratio


def build_materials(reference_run, output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as pyplot

    pyplot.rcParams.update({"font.family": "Noto Sans CJK JP", "axes.unicode_minus": False, "font.size": 11})
    metrics, tracking, frames, acceleration_ratio = recorded_metrics(reference_run)
    target = np.array([0.6, 0.4])
    solutions = planar_analytic(target)
    history = planar_iterative(target, np.radians([-20.0, 80.0]))
    metrics["two_link_example"] = {
        "lengths_m": [0.5, 0.4],
        "target_m": target.tolist(),
        "analytic_solutions_deg": np.degrees(solutions).tolist(),
        "numeric_iterations": len(history) - 1,
        "numeric_final_position_error_m": float(np.linalg.norm(planar_fk(history[-1]) - target)),
        "scope": "synthetic planar position-only example; no collisions or limits; not G2 performance",
    }
    output.mkdir(parents=True, exist_ok=False)

    def save(figure, filename):
        figure.savefig(output / filename, dpi=160, bbox_inches="tight")
        pyplot.close(figure)

    def draw_arm(axis, joints, label, color):
        elbow_point = 0.5 * np.array([np.cos(joints[0]), np.sin(joints[0])])
        points = np.vstack([np.zeros(2), elbow_point, planar_fk(joints)])
        axis.plot(points[:, 0], points[:, 1], "o-", linewidth=4, color=color, label=label)
        axis.set(xlim=(-0.1, 1.0), ylim=(-0.3, 0.9), xlabel="x / m", ylabel="y / m")
        axis.set_aspect("equal")
        axis.grid(alpha=0.2)

    figure, axes = pyplot.subplots(1, 2, figsize=(13, 5))
    joints = np.radians([30.0, 60.0])
    draw_arm(axes[0], joints, "给定关节角 → FK → 手的位置", "#166b95")
    endpoint = planar_fk(joints)
    small_motion = planar_jacobian(joints) @ np.array([0.08, -0.03])
    axes[0].annotate(
        "", endpoint + 3 * small_motion, xytext=endpoint, arrowprops={"arrowstyle": "->", "color": "#dd641d", "lw": 3}
    )
    axes[0].text(0.47, 0.72, "局部方向\n箭头放大 3 倍", color="#ae4915", fontsize=10)
    axes[0].legend(loc="lower left")
    axes[0].set_title("FK：现在手在哪？Jacobian：小转一下会往哪走？")
    for solution, label, color in zip(solutions, ("解析分支 A", "解析分支 B"), ("#166b95", "#dd641d")):
        draw_arm(axes[1], solution, label, color)
    axes[1].scatter(*target, marker="*", s=200, color="#962f53", zorder=5)
    axes[1].legend(loc="lower left")
    axes[1].set_title("IK：同一个手位置，可能有两种肘部姿态")
    figure.suptitle("教学示意：0.5 m + 0.4 m 平面二连杆，不是 G2 实测")
    save(figure, "01-fk-ik-jacobian.png")

    figure, axes = pyplot.subplots(1, 2, figsize=(13, 5))
    endpoints = np.array([planar_fk(joints) for joints in history])
    axes[0].plot(endpoints[:, 0], endpoints[:, 1], "o--", color="#166b95", label="数值迭代末端位置")
    axes[0].scatter(*target, marker="*", s=200, color="#dd641d", label="目标")
    axes[0].set(xlabel="x / m", ylabel="y / m", title="猜一组角度 → 看误差 → 小步修正")
    axes[0].set_aspect("equal")
    residuals = np.linalg.norm(endpoints - target, axis=1)
    axes[1].semilogy(np.maximum(residuals, 1e-16), "o-", color="#166b95", label="位置残差")
    axes[1].set(xlabel="迭代次数（不是仿真时间）", ylabel="残差 / m", title="数值法也能很准；此例未加机器人约束")
    for axis in axes:
        axis.legend()
        axis.grid(alpha=0.2)
    save(figure, "02-numerical-iteration.png")

    figure, axis = pyplot.subplots(figsize=(14, 6))
    axis.set(xlim=(0, 14), ylim=(0, 6))
    axis.axis("off")
    boxes = [
        (1.3, "UMI 目标\n30 Hz / 1×"),
        (4.0, "Python FK / Jacobian\nPinocchio 尚未在线 A/B"),
        (7.1, "双手联合 QP\n限位 / 耦合 / 加速度"),
        (10.0, "ROS 碰撞检查\n位置命令发布"),
        (12.6, "PhysX 位置驱动\n实际关节状态"),
    ]
    for horizontal, label in boxes:
        axis.text(
            horizontal, 4, label, ha="center", va="center", bbox={"boxstyle": "round,pad=0.6", "facecolor": "#e1eff7"}
        )
    for first, second in zip(boxes[:-1], boxes[1:]):
        axis.annotate("", (second[0] - 1.15, 4), (first[0] + 1.15, 4), arrowprops={"arrowstyle": "->", "lw": 2})
    axis.text(4.0, 2.5, "上一条关节命令做热启动\n不是每轮从实测关节重新求解", ha="center", color="#963f21")
    axis.text(
        10.0,
        1.2,
        "实测 FK 对当前 UMI 相位统计误差\n超阈值停止；尚非实测驱动笛卡尔闭环",
        ha="center",
        bbox={"boxstyle": "round,pad=0.6", "facecolor": "#f9e8da"},
    )
    axis.annotate("", (10.0, 1.9), (12.6, 3.3), arrowprops={"arrowstyle": "->", "lw": 2})
    axis.set_title("v5 实际职责链：模型计算快，不等于 QP、通信、执行器和反馈都快", fontsize=16)
    save(figure, "03-v5-control-stack.png")

    figure, axes = pyplot.subplots(2, 2, figsize=(14, 9))
    phases = np.array([float(row["source_time_s"]) for row in tracking])
    times = np.array([frame["timestamp"] for frame in frames])
    for side, label, chinese in ((0, "left", "左手"), (1, "right", "右手")):
        errors = [1000 * float(row[f"position_error_{label}_m"]) for row in tracking]
        axes[0, 0].plot(phases, errors, label=chinese)
        points = np.array([frame["eef_odom"][side]["position"] for frame in frames])
        axes[0, 1].plot(times[1:], np.linalg.norm(np.diff(points, axis=0), axis=1) / np.diff(times), label=chinese)
    axes[0, 0].set(ylabel="误差 / mm", title="真实 v5：实测关节 FK 对当前源目标")
    axes[0, 1].set(ylabel="相邻源帧位移 / 时间 / (m/s)", title="源目标在运动，不是静态点定位测试")
    axes[1, 0].plot(phases, [1000 * float(row["feedback_age_s"]) for row in tracking], label="回调状态年龄")
    axes[1, 0].set(ylabel="年龄 / ms", title="不等于全链路时延，也不是采样原始时间戳")
    with (reference_run / "command_joints.csv").open() as stream:
        command_phases = [float(row["source_time_s"]) for row in csv.DictReader(stream)]
    axes[1, 1].plot(command_phases, acceleration_ratio, label="各关节离散加速度 / 限值的最大值")
    axes[1, 1].axhline(0.99, color="#dd641d", linestyle="--", label="99% 判据")
    fraction = metrics["acceleration_near_limit_tick_fraction"]
    axes[1, 1].set(ylabel="归一化比例", title=f"{fraction:.2%} 命令周期至少一维贴近加速度边界")
    for axis in axes.flat:
        axis.set_xlabel("源相位 / s")
        axis.legend(fontsize=9)
        axis.grid(alpha=0.2)
    figure.suptitle("离线重读既有 run-001；没有重新运行机器人，没有改变原始 v5 指标")
    figure.tight_layout()
    save(figure, "04-v5-tracking-diagnostics.png")

    figure, axis = pyplot.subplots(figsize=(13, 4))
    axis.set(xlim=(0, 12), ylim=(0, 4))
    axis.axis("off")
    for horizontal, label, color in [
        (1.5, "实际位置\np_actual", "#166b95"),
        (5.5, "命令对应 FK\np_command", "#dd641d"),
        (10.0, "同一时刻的目标\np_target", "#962f53"),
    ]:
        axis.scatter(horizontal, 2.5, s=160, color=color)
        axis.text(horizontal, 3.05, label, ha="center", color=color)
    for start, end, height, label in [
        (1.5, 5.5, 2.5, "执行/伺服差"),
        (5.5, 10.0, 2.5, "命令/任务差"),
        (1.5, 10.0, 0.9, "总误差向量 = 任务差向量 + 伺服差向量"),
    ]:
        axis.annotate("", (end, height), (start, height), arrowprops={"arrowstyle": "->", "lw": 2})
        axis.text((start + end) / 2, height - 0.45, label, ha="center")
    axis.set_title("概念分解：向量可以相加，三个误差的范数一般不能直接相加；此图不是实测占比", fontsize=14)
    save(figure, "05-error-decomposition.png")
    (output / "metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2) + "\n")
    return metrics


def main():
    parser = argparse.ArgumentParser(description="Build offline IK learning illustrations from recorded v5 evidence")
    parser.add_argument("--reference-run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build_materials(args.reference_run, args.output), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
