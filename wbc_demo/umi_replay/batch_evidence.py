import csv
import html
import json
import shutil
from pathlib import Path

import numpy as np

from .download import sha256


def export_batch(directory, output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as pyplot

    directory, output = Path(directory), Path(output)
    summary = json.loads((directory / "summary.json").read_text())
    if summary["status"] == "running":
        raise ValueError("Wait until the batch stops before exporting evidence")
    rows, paired, diagnostics = [], {}, {}
    for job in summary["jobs"]:
        row = {key: job[key] for key in ("episode", "backend", "repeat", "status", "directory")}
        row["error"] = job.get("error") or ""
        row["command_audit_status"] = job.get("command_audit", {}).get("status", "unavailable")
        hold_audit = directory / job["directory"] / "hold-aware-audit.json"
        row["hold_transition_audit"] = json.loads(hold_audit.read_text()) if hold_audit.exists() else None
        diagnosis = directory / job["directory"] / "infeasibility-diagnosis.json"
        if diagnosis.exists():
            diagnostics[job["directory"]] = json.loads(diagnosis.read_text())
        report_path = directory / job["directory"] / "report.json"
        if report_path.exists():
            report = json.loads(report_path.read_text())
            paired[(job["episode"], job["repeat"], job["backend"])] = report
            replay = report.get("replay", {})
            row["commands"] = replay.get("commands", 0)
            row["deadline_misses"] = replay.get("deadline_misses", 0)
            row["solve_p95_ms"] = replay.get("solve_ms_p50_p95_max", [None] * 3)[1]
            row["cycle_p95_ms"] = replay.get("cycle_ms_p50_p95_max", [None] * 3)[1]
            row["active_wall_s"] = replay.get("active_wall_seconds")
            row["source_s"] = replay.get("source_duration_seconds")
            row["report_sha256"] = sha256(report_path)
        metrics = job.get("tick_metrics", {})
        geometry = directory / job["directory"] / "tick-geometry.json"
        if geometry.exists():
            row["snapshot_geometry_check"] = json.loads(geometry.read_text())["status"]
        for side in ("left", "right"):
            for kind in ("total", "task", "following"):
                values = metrics.get("per_side", {}).get(side, {}).get(kind + "_position_m_p50_p95_max", [None] * 3)
                row[f"{side}_{kind}_p95_mm"] = values[1] * 1000 if values[1] is not None else None
                row[f"{side}_{kind}_max_mm"] = values[2] * 1000 if values[2] is not None else None
        rows.append(row)
    comparisons = []
    for episode, repeat, backend in paired:
        if backend != "urdf" or (episode, repeat, "pinocchio") not in paired:
            continue
        original, candidate = paired[(episode, repeat, "urdf")], paired[(episode, repeat, "pinocchio")]
        comparison = {"episode": episode, "repeat": repeat}
        for key in ("initial_measured_positions_rad", "initial_approach_goal_rad"):
            first, second = original.get(key, {}), candidate.get(key, {})
            names = [name for name in first.keys() & second.keys() if "_body_joint" in name or "_arm_" in name]
            comparison[key + "_max_difference_rad"] = max(
                (abs(first[name] - second[name]) for name in names), default=None
            )
        comparisons.append(comparison)
    frozen_goal = bool(summary.get("approach_reference_batch"))
    if frozen_goal and any(item["initial_approach_goal_rad_max_difference_rad"] != 0 for item in comparisons):
        raise ValueError("Paired initial joint goals differ despite frozen-reference protocol")
    output.mkdir(parents=True, exist_ok=False)
    fully_validated = (
        summary["status"] == "passed"
        and bool(rows)
        and all(
            row["status"] == "passed"
            and row["command_audit_status"] == "passed"
            and (row["hold_transition_audit"] or {}).get("status") == "passed"
            for row in rows
        )
    )
    payload = {
        "summary": summary,
        "nominal_command_validation_status": "passed" if fully_validated else "not_fully_passed",
        "rows": rows,
        "start_and_goal_comparability": comparisons,
        "source_summary_sha256": sha256(directory / "summary.json"),
        "failure_diagnostics": diagnostics,
        "complete_command_audits": sum(row["command_audit_status"] == "passed" for row in rows),
        "incomplete_command_audits": sum(
            row["command_audit_status"] == "passed_with_incomplete_acceleration_audit" for row in rows
        ),
        "goal_policy": "same frozen initial joint goal" if frozen_goal else "independent BioIK initial goals",
        "limits": "Pilot online comparison: same QP, constraints, interpreter and world transform; initial physical states may differ. Check goal_policy and start/goal differences. No claim of causal accuracy improvement or closed-loop upgrade.",
    }
    (output / "summary.json").write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n")
    fields = sorted({key for row in rows for key in row})
    with (output / "results.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fields)
        writer.writeheader()
        writer.writerows(rows)
    completed = [row for row in rows if row["status"] == "passed"]
    if completed:
        labels = [
            f"{row['episode']:03d} {'U' if row['backend'] == 'urdf' else 'P'}{row['repeat']}" for row in completed
        ]
        indexes = np.arange(len(completed))
        figure, axes = pyplot.subplots(2, 2, figsize=(14, 9))
        for side in ("left", "right"):
            axes[0, 0].plot(indexes, [row[f"{side}_total_p95_mm"] for row in completed], "o-", label=side)
        axes[0, 0].set(ylabel="mm", title="Observed EEF total position P95")
        axes[0, 1].bar(indexes, [row["cycle_p95_ms"] for row in completed], label="full cycle P95")
        axes[0, 1].plot(indexes, [row["solve_p95_ms"] for row in completed], "o-", color="orange", label="IK P95")
        axes[0, 1].axhline(1000 / 30, color="red", linestyle="--", label="33.3 ms budget")
        axes[0, 1].set(ylabel="ms", title="Actual online calculation; not model-only benchmark")
        for kind in ("task", "following"):
            axes[1, 0].plot(indexes, [row[f"left_{kind}_p95_mm"] for row in completed], "o-", label=kind)
        axes[1, 0].set(ylabel="mm", title="Left residual P95; norms are not additive")
        axes[1, 1].plot(
            indexes, [row["active_wall_s"] - row["source_s"] for row in completed], "o-", label="active wall - source"
        )
        axes[1, 1].set(ylabel="s", title="Source-rate timing, no global retiming")
        for axis in axes.flat:
            axis.set_xticks(indexes, labels, rotation=45, ha="right")
            axis.legend(fontsize=8)
            axis.grid(alpha=0.2)
        figure.suptitle(
            f"Online pilot / {'frozen joint goal' if frozen_goal else 'independent BioIK goals'}: U=URDF, P=Pinocchio"
        )
        figure.tight_layout()
        figure.savefig(output / "online-comparison.png", dpi=150)
        pyplot.close(figure)
    galleries = []
    for job in summary["jobs"]:
        source = directory / job["directory"] / "error-decomposition.png"
        if not source.exists():
            continue
        filename = job["directory"] + ".png"
        shutil.copyfile(source, output / filename)
        galleries.append(
            f'<h3>{html.escape(job["directory"])} / {html.escape(job["status"])}</h3><img src="{filename}" alt="synchronized error decomposition">'
        )
    table = []
    for row in rows:
        values = [
            f'{row["episode"]:03d}',
            row["backend"],
            row["repeat"],
            row["status"],
            row["command_audit_status"],
            (row["hold_transition_audit"] or {}).get("status", "unavailable"),
            row.get("commands", 0),
            row.get("deadline_misses", ""),
            row.get("solve_p95_ms"),
            row.get("cycle_p95_ms"),
            row["error"],
        ]
        table.append(
            "<tr>"
            + "".join(
                f"<td>{html.escape(f'{value:.3f}' if isinstance(value, float) else '' if value is None else str(value))}</td>"
                for value in values
            )
            + "</tr>"
        )
    overview = '<img src="online-comparison.png" alt="actual online comparison">' if completed else ""
    (output / "index.html").write_text(
        '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
        "<title>G2 在线模型 A/B 与误差分解</title><style>body{background:#122032;color:#edf3fc;font:16px/1.7 system-ui;margin:24px auto;max-width:1400px;padding:16px}a{color:#8bdbe8}img{width:100%;background:white;margin:12px 0}table{border-collapse:collapse;width:100%}td,th{padding:8px;border-bottom:1px solid #52647a;text-align:left}section{overflow:auto}</style>"
        "<h1>固定底盘 G2：在线模型 A/B 与同步误差分解</h1>"
        "<p>本页展示批处理实际报告；是否执行、是否完成以各行状态为准，不把首帧预检当作整段成功。仅替换模型层，没有升级实测状态笛卡尔闭环，没有解锁底盘。</p>"
        "<p>单一参考世界变换，不逐段重新居中。following 是相对最后发布命令的差，没有驱动确认，不等于已识别的因果占比。初始物理反馈可能不同，不能据此断言精度改善来自后端。</p>"
        f'<p>首帧解策略：{"复用同一已验证关节解，并校验配对解完全一致" if frozen_goal else "各次独立 BioIK，存在分支混杂"}。</p>'
        f'<p>原始运行状态：{html.escape(summary["status"])}；名义命令验收：{payload["nominal_command_validation_status"]}。<a href="summary.json">完整协议、指标与输入哈希</a> · <a href="results.csv">CSV</a> · <a href="../../ONLINE_AB_RESULTS_ZH.md">实验说明</a></p>'
        f'<p>完整命令审计通过 {payload["complete_command_audits"]} 项，加速度审计不完整 {payload["incomplete_command_audits"]} 项。运行完成不等于约束全部通过；HOLD 补充审计见 JSON。</p>'
        "<section><table><tr><th>episode</th><th>backend</th><th>repeat</th><th>execution</th><th>command audit</th><th>HOLD audit</th><th>commands</th><th>misses</th><th>IK P95 ms</th><th>cycle P95 ms</th><th>error</th></tr>"
        + "".join(table)
        + "</table></section>"
        + overview
        + "".join(galleries)
        + "</html>"
    )
    return payload
