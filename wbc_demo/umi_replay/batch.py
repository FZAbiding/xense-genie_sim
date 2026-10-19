import copy
import json
import math
import signal
import subprocess
import sys
from pathlib import Path

from .download import sha256
from .geometry import check_transform


def frozen_config(reference, config):
    if reference.get("status") != "passed" or reference.get("playback_mode") != "realtime":
        raise ValueError("Reference must be a passed realtime run")
    result = copy.deepcopy(config)
    result["mode"] = "calibrated"
    result["world_to_odom"] = check_transform(reference["resolved_alignment"]["world_to_odom"]).tolist()
    result["calibration_status"] = "frozen reference dataset transform; model based, not hardware calibrated"
    return result


def jobs_for(episodes, backends, repeats):
    if (
        repeats < 1
        or not episodes
        or not backends
        or min(episodes) < 0
        or len(set(episodes)) != len(episodes)
        or len(set(backends)) != len(backends)
    ):
        raise ValueError("Nonempty unique episodes/backends and positive repeats required")
    jobs = []
    for repeat in range(repeats):
        order = backends if repeat % 2 == 0 else list(reversed(backends))
        for episode in episodes:
            for backend in order:
                jobs.append(
                    {
                        "episode": episode,
                        "backend": backend,
                        "repeat": repeat + 1,
                        "directory": f"episode-{episode:03d}-{backend}-r{repeat + 1:02d}",
                        "status": "not_run",
                    }
                )
    return jobs


def run_child(command, log, timeout):
    with log.open("w") as stream:
        process = subprocess.Popen(command, stdout=stream, stderr=stream)
        try:
            return process.wait(timeout=timeout), False
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                raise RuntimeError("Child did not acknowledge stop; refusing to launch further motion")
            return process.returncode, True


def run_batch(args):
    from .kinematics import validate_backend

    reference = json.loads((args.reference_run / "report.json").read_text())
    config = frozen_config(reference, json.loads(args.config.read_text()))
    for backend in args.backends:
        validate_backend(backend)
    jobs = jobs_for(args.episodes, args.backends, args.repeats)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        raise ValueError("Positive per-run timeout required")
    from .validation import validate_realtime_profile, validate_realtime_source

    limits_bytes = args.limits_profile.read_bytes()
    validate_realtime_profile(json.loads(limits_bytes))
    sources = {episode: args.converted_directory / f"episode-{episode:03d}.json" for episode in args.episodes}
    for path in sources.values():
        source = json.loads(path.read_text())
        validate_realtime_source(source["source_frames"])
        if source["provenance"]["revision"] != reference["provenance"]["revision"]:
            raise ValueError("Dataset revision differs from the frozen reference")
    approach_batch = getattr(args, "approach_reference_batch", None)
    approach_reports = {}
    if approach_batch:
        from .approach_reference import validate_reference
        from .execution import posture_config

        for episode, source in sources.items():
            path = approach_batch / f"episode-{episode:03d}-urdf-r01/report.json"
            validate_reference(
                json.loads(path.read_text()), sha256(source), posture_config(config, args.posture_policy)
            )
            approach_reports[episode] = path
    args.output.mkdir(parents=True, exist_ok=False)
    config_path = args.output / "fixed-world-config.json"
    config_path.write_text(json.dumps(config, indent=2) + "\n")
    limits_path = args.output / "limits-profile.json"
    limits_path.write_bytes(limits_bytes)
    protocol = {
        "status": "running",
        "execution_requested": args.execute,
        "python": sys.executable,
        "reference_run": str(args.reference_run),
        "reference_report_sha256": sha256(args.reference_run / "report.json"),
        "config_sha256": sha256(config_path),
        "limits_profile_sha256": sha256(limits_path),
        "posture_policy": args.posture_policy,
        "approach_reference_batch": str(approach_batch) if approach_batch else None,
        "jobs": jobs,
        "world_policy": "one transform frozen from reference; no per-episode re-centering or scaling",
        "failure_policy": "stop after motion failure, timeout or failed/incomplete HOLD-transition audit; retain remaining jobs as not_run",
    }
    summary = args.output / "summary.json"

    def save():
        summary.write_text(json.dumps(protocol, indent=2, allow_nan=False) + "\n")

    save()
    try:
        for job in jobs:
            directory = args.output / job["directory"]
            command = [
                sys.executable,
                str(Path(__file__).resolve().parents[1] / "umi_demo.py"),
                "replay",
                "--input",
                str(sources[job["episode"]]),
                "--output",
                str(directory),
                "--config",
                str(config_path),
                "--playback-mode",
                "realtime",
                "--speed",
                "1",
                "--posture-policy",
                args.posture_policy,
                "--limits-profile",
                str(limits_path),
                "--kinematics-backend",
                job["backend"],
            ]
            if args.execute:
                command.append("--execute")
            if approach_batch:
                command.extend(["--approach-reference", str(approach_reports[job["episode"]])])
            job.update(status="running", command=command, input_sha256=sha256(sources[job["episode"]]))
            save()
            returncode, timed_out = run_child(command, args.output / (job["directory"] + ".log"), args.timeout)
            report_path = directory / "report.json"
            report = json.loads(report_path.read_text()) if report_path.exists() else {}
            job.update(
                status=report.get("status", "missing_report"),
                returncode=returncode,
                timed_out=timed_out,
                error=report.get("error"),
                report_sha256=sha256(report_path) if report_path.exists() else None,
            )
            if (directory / "control_ticks.jsonl").exists() and (directory / "control_ticks.jsonl").stat().st_size:
                from .tick_analysis import analyze_ticks

                job["tick_metrics"] = analyze_ticks(directory)
            if report.get("status") == "passed":
                from .realtime_analysis import analyze_realtime

                job["command_audit"] = analyze_realtime(directory)
                job["replay"] = report["replay"]
                from .tick_analysis import audit_hold_transitions

                job["hold_transition_audit"] = audit_hold_transitions(directory)
            save()
            if job.get("hold_transition_audit", {}).get("status") != "passed" and report.get("status") == "passed":
                protocol["status"] = "stopped_after_command_audit_failure"
                break
            if (
                timed_out
                or not report
                or ((returncode != 0 or report.get("status") == "failed") and report.get("motion_started", True))
            ):
                protocol["status"] = "stopped_after_failure"
                break
        else:
            expected = "passed" if args.execute else "initial_pose_preflight_passed_no_execution"
            protocol["status"] = (
                "passed"
                if args.execute and all(job["status"] == expected for job in jobs)
                else "preflight_only" if all(job["status"] == expected for job in jobs) else "completed_with_failures"
            )
    except BaseException as error:
        protocol["status"] = "interrupted_or_failed"
        protocol["error"] = str(error)
        raise
    finally:
        save()
    print(f"{protocol['status']}: {summary}", flush=True)
    return protocol
