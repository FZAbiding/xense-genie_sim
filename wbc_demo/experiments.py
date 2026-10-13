import argparse
import copy
import json
import math
import os
import time
from datetime import datetime

import rclpy
import yaml
from moveit_msgs.srv import GetPositionFK

from dual_arm_ik import GROUP, position_error
from lab_core import ARM_GROUP, ROOT, WholeBodyLab, make_targets, pose_values, save_json, timestamp


def reach_certificate(node, seed, targets):
    request = GetPositionFK.Request()
    request.header.frame_id = "odom"
    request.fk_link_names = ["arm_base_link"]
    request.robot_state = seed
    response = node.call(node.fk, request)
    if response.error_code.val != 1:
        raise RuntimeError("Cannot compute arm-base FK for reach certificate")
    root = response.pose_stamped[0].pose.position
    parent_joints = {joint.find("child").get("link"): joint for joint in node.robot.findall("joint")}
    certificates = []
    for target, tip in zip(targets, ("arm_l_end_link", "arm_r_end_link")):
        current = tip
        bound = 0.0
        while current != "arm_base_link":
            joint = parent_joints[current]
            origin = joint.find("origin")
            xyz = [float(value) for value in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
            bound += math.sqrt(sum(value * value for value in xyz))
            current = joint.find("parent").get("link")
        distance = math.dist(
            [root.x, root.y, root.z], [target.pose.position.x, target.pose.position.y, target.pose.position.z]
        )
        certificates.append(
            {
                "tip": tip,
                "distance_m": distance,
                "conservative_arm_reach_m": bound,
                "outside_locked_arm_reach": distance > bound + 0.001,
            }
        )
    return certificates


def compare_groups(node, report, output, execute_found=False):
    seed = copy.deepcopy(node.state)
    reference = node.forward(seed)
    for drop in (0.35, 0.45, 0.55, 0.65, 0.75):
        specification = {"left": [0.08, 0.0, -drop], "right": [0.08, 0.0, -drop]}
        targets = make_targets(reference, specification, node.center)
        candidate = {
            "drop_m": drop,
            "targets": pose_values(targets),
            "groups": {},
            "reach_certificate": reach_certificate(node, seed, targets),
        }
        free = None
        for group in (ARM_GROUP, GROUP):
            beginning = time.monotonic()
            try:
                prepared = node.prepare(targets, group=group, seed=seed, plan=False)
                candidate["groups"][group] = {
                    "status": "solved",
                    "seconds": time.monotonic() - beginning,
                    "ik_error_m": prepared["ik_error_m"],
                }
                if group == GROUP:
                    free = prepared
            except RuntimeError as error:
                candidate["groups"][group] = {
                    "status": "rejected",
                    "seconds": time.monotonic() - beginning,
                    "error": str(error),
                }
        certified = any(item["outside_locked_arm_reach"] for item in candidate["reach_certificate"])
        found = free is not None and candidate["groups"][ARM_GROUP]["status"] == "rejected"
        candidate["certified_counterexample"] = found and certified
        report["comparison"].append(candidate)
        save_json(output / "report.json", report)
        print(json.dumps(candidate), flush=True)
        if found and certified:
            if execute_found:
                free["path"], free["planning_method"], free["collision_samples"] = node.plan_path(
                    seed, free["goal"], GROUP
                )
                report["comparison_execution"] = node.execute_prepared(free)
                report["comparison_return_home"] = node.return_home()
            return True
    return False


def run(node, args, output, report):
    node.acquire_control()
    node.initialize()
    node.progress = node.show_targets
    node.trace_file = (output / "joint_trace.csv").open("w", newline="")
    report["status"] = "running"
    report["reference"] = pose_values(node.reference)
    if args.compare:
        if not compare_groups(node, report, output, args.execute_found):
            raise RuntimeError("No certified arm-only-unreachable / whole-body-reachable counterexample found")
        return
    specifications = yaml.safe_load((ROOT / "stress_targets.yaml").read_text())["targets"]
    for cycle in range(args.cycles):
        for specification in specifications:
            targets = make_targets(node.reference, specification, node.center)
            result = {
                "cycle": cycle + 1,
                "name": specification["name"],
                "requested": specification,
                "requested_targets": pose_values(targets),
            }
            try:
                prepared = node.prepare(targets, plan=not args.probe_only)
                if args.probe_only:
                    result.update(
                        {
                            "status": "solved_no_execution",
                            "ik_error_m": prepared["ik_error_m"],
                            "solve_seconds": prepared["solve_seconds"],
                        }
                    )
                else:
                    result.update(node.execute_prepared(prepared, args.segment_seconds))
                    result["tcp_displacement_m"] = [
                        position_error(target.pose, initial.pose) for target, initial in zip(targets, node.reference)
                    ]
                    result["return_home"] = node.return_home()
            except RuntimeError as error:
                result.update({"status": "rejected", "error": str(error)})
                report["cases"].append(result)
                save_json(output / "report.json", report)
                print(json.dumps(result), flush=True)
                if args.probe_only:
                    continue
                raise
            report["cases"].append(result)
            save_json(output / "report.json", report)
            print(json.dumps(result), flush=True)
    if any(case["status"] == "rejected" for case in report["cases"]):
        raise RuntimeError("One or more requested stress targets were rejected; see per-case evidence")
    unreachable = make_targets(node.reference, {"left": [5, 0, 0], "right": [5, 0, 0]}, node.center)
    try:
        node.prepare(unreachable, plan=False)
    except RuntimeError as error:
        report["negative_test"] = {"status": "passed", "reason": str(error), "command_sent": False}
    else:
        raise RuntimeError("Expected unreachable target was incorrectly accepted")


def main():
    parser = argparse.ArgumentParser(description="Large-range fixed-base IK stress and waist A/B experiments")
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--segment-seconds", type=float, default=4.0)
    parser.add_argument("--max-joint-velocity", type=float, default=0.3)
    parser.add_argument("--max-joint-acceleration", type=float, default=0.5)
    parser.add_argument("--probe-only", action="store_true")
    parser.add_argument("--compare", action="store_true")
    parser.add_argument("--execute-found", action="store_true")
    parser.add_argument("--output-name", default="")
    args = parser.parse_args()
    if args.cycles < 1 or args.segment_seconds < 1 or args.max_joint_velocity <= 0 or args.max_joint_acceleration <= 0:
        parser.error("Use positive limits, cycles >= 1, segment-seconds >= 1")
    if args.execute_found and not args.compare:
        parser.error("--execute-found requires --compare")
    name = args.output_name or ("compare-" if args.compare else "stress-") + datetime.now().strftime("%Y%m%d-%H%M%S")
    if "/" in name or name in (".", ".."):
        parser.error("output-name must be a directory name, not a path")
    output = ROOT / "logs" / name
    output.mkdir(parents=True, exist_ok=False)
    report = {
        "status": "starting",
        "started_at": timestamp(),
        "cases": [],
        "comparison": [],
        "mode": "comparison" if args.compare else "probe_only" if args.probe_only else "physical_stress",
        "limits": {
            "joint_velocity_rad_s": args.max_joint_velocity,
            "joint_acceleration_rad_s2": args.max_joint_acceleration,
        },
        "scene": "use scene_stress.yaml; no environment obstacles are synchronized to MoveIt",
    }
    save_json(output / "report.json", report)
    os.environ.setdefault("ROS_DOMAIN_ID", "42")
    rclpy.init()
    node = WholeBodyLab(args.max_joint_velocity, args.max_joint_acceleration)
    try:
        run(node, args, output, report)
        report["status"] = "passed"
        print(f"PASS: {report['mode']}; evidence: {output}", flush=True)
    except BaseException as error:
        report["status"] = "failed"
        report["error"] = str(error)
        raise
    finally:
        report["finished_at"] = timestamp()
        save_json(output / "report.json", report)
        node.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
