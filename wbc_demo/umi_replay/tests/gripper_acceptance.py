import argparse
import json
import os
import time
from pathlib import Path
from urllib.request import urlopen

import rclpy

from lab_core import pose_values
from umi_replay.geometry import pose_error
from umi_replay.robot import GripperEEFLab


def main():
    parser = argparse.ArgumentParser(
        description="Explicit simulator-only gripper endpoint acceptance, keeping fingertip EEF poses fixed"
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--viewer-url", default="http://127.0.0.1:8765")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not args.execute:
        parser.error("This test moves the simulated robot; pass --execute explicitly")
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.setdefault("ROS_DOMAIN_ID", "42")
    rclpy.init()
    node = GripperEEFLab()
    result = {"steps": [], "mapping": "normalized 0 closed / 1 open; linear native master-joint range"}
    try:
        node.acquire_control()
        node.initialize()
        node.require_sim_camera()
        targets = node.forward(node.state)
        for label, opening in (("closed", 0.0), ("half", 0.5), ("open", 1.0)):
            node.requested_openings = [opening, opening]
            prepared = node.prepare(targets)
            motion = node.execute_prepared(prepared)
            positions = dict(zip(node.state.joint_state.name, node.state.joint_state.position))
            measured = node.eef.openings_from_positions(positions)
            errors = [
                pose_error(wanted, actual)
                for wanted, actual in zip(pose_values(targets), pose_values(node.forward(node.state)))
            ]
            assert max(abs(value - opening) for value in measured) < 0.02, measured
            assert max(value[0] for value in errors) < 0.02, errors
            result["steps"].append(
                {
                    "label": label,
                    "target_opening": opening,
                    "actual_openings": measured,
                    "master_angles_rad": [positions[name] for name in node.eef.master_names],
                    "eef_errors_m_rad": errors,
                    "motion": motion,
                }
            )
            time.sleep(0.5)
            for name in ("UmiEgo", "ExternalA"):
                with urlopen(args.viewer_url + "/frame/" + name, timeout=5) as response:
                    (args.output / f"{label}-{name}.jpg").write_bytes(response.read())
            (args.output / "gripper-endpoints.json").write_text(json.dumps(result, indent=2) + "\n")
        node.return_home()
        result["status"] = "passed"
    finally:
        node.hold("wbc_fixed_headless")
        if node.command_lock is not None:
            node.command_lock.close()
        node.destroy_node()
        rclpy.shutdown()
        (args.output / "gripper-endpoints.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
