import hashlib
import xml.etree.ElementTree as element_tree
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .geometry import check_transform, pose_dict, pose_matrix


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def joint_origin(joint):
    origin = joint.find("origin")
    matrix = np.eye(4)
    if origin is not None:
        matrix[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        matrix[:3, :3] = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")).as_matrix()
    return check_transform(matrix)


def binary_stl_vertices(path):
    payload = Path(path).read_bytes()
    if len(payload) < 84:
        raise ValueError(f"Truncated STL: {path}")
    count = int.from_bytes(payload[80:84], "little")
    if len(payload) != 84 + count * 50 or count == 0:
        raise ValueError(f"Expected a nonempty binary STL: {path}")
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    vertices = np.frombuffer(payload, dtype=dtype, offset=84)["vertices"].reshape(-1, 3).astype(float)
    if not np.isfinite(vertices).all():
        raise ValueError("Non-finite gripper mesh vertices")
    return vertices


class SwiftpickerEEF:
    def __init__(self, urdf_path, package_root):
        self.robot = element_tree.parse(urdf_path).getroot()
        self.joints = {joint.get("name"): joint for joint in self.robot.findall("joint")}
        self.origins = {name: joint_origin(joint) for name, joint in self.joints.items()}
        self.package_root = Path(package_root)
        self.parents = {joint.find("child").get("link"): joint for joint in self.robot.findall("joint")}
        self.links = {link.get("name"): link for link in self.robot.findall("link")}
        self.frame_names = ["gripper_l_eef", "gripper_r_eef"]
        self.tip_names = ["arm_l_end_link", "arm_r_end_link"]
        self.tip_to_eef = []
        self.finger_points = []
        self.master_names = ["idx31_gripper_l_inner_joint1", "idx71_gripper_r_inner_joint1"]
        self.evidence = {
            "definition": "midpoint of the two distal fingertip faces at the held URDF zero-joint (open) configuration",
            "axes": "X toward fingertips; Z toward the wrist-camera side; Y = Z cross X (opposite URDF left_support)",
            "axis_revision": "camera_side_up_v2",
            "urdf_sha256": file_hash(urdf_path),
            "hardware_calibration": False,
            "sides": [],
        }
        for side, tip in zip(("l", "r"), self.tip_names):
            base = f"gripper_{side}_base_link"
            mount = self.relative_zero(tip, base)
            centers = []
            meshes = []
            finger_points = []
            for finger in ("left", "right"):
                link_name = f"gripper_{side}_{finger}_support_link"
                collision = self.links[link_name].find("collision")
                mesh = collision.find("geometry/mesh")
                resource = mesh.get("filename")
                prefix = "package://genie_sim_robot_model/"
                if not resource.startswith(prefix):
                    raise ValueError("Unexpected Swiftpicker mesh resource")
                path = self.package_root / resource.removeprefix(prefix)
                vertices = binary_stl_vertices(path)
                scale = np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
                vertices *= scale
                placement = self.relative_zero(base, link_name) @ joint_origin(collision)
                vertices = vertices @ placement[:3, :3].T + placement[:3, 3]
                distal = vertices[vertices[:, 2] >= vertices[:, 2].max() - 0.00005]
                center = (distal.min(axis=0) + distal.max(axis=0)) / 2
                centers.append(center)
                local = np.linalg.inv(self.relative_zero(base, link_name)) @ np.r_[center, 1.0]
                finger_points.append((link_name, local))
                meshes.append(
                    {"resource": resource, "sha256": file_hash(path), "distal_face_center_in_base_m": center.tolist()}
                )
            base_to_eef = np.eye(4)
            base_to_eef[:3, 3] = np.mean(centers, axis=0)
            base_to_eef[:3, :3] = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
            camera_link = self.links[f"gripper_{side}_camera_link"]
            camera_collision = camera_link.find("collision")
            camera_resource = camera_collision.find("geometry/mesh").get("filename")
            camera_path = self.package_root / camera_resource.removeprefix("package://genie_sim_robot_model/")
            camera_vertices = binary_stl_vertices(camera_path)
            camera_placement = (
                np.linalg.inv(mount)
                @ self.relative_zero(tip, f"gripper_{side}_camera_link")
                @ joint_origin(camera_collision)
            )
            camera_in_base = camera_vertices @ camera_placement[:3, :3].T + camera_placement[:3, 3]
            camera_center = (camera_in_base.min(axis=0) + camera_in_base.max(axis=0)) / 2
            camera_center_in_eef = base_to_eef[:3, :3].T @ (camera_center - base_to_eef[:3, 3])
            if camera_center_in_eef[2] <= 0:
                raise ValueError("The wrist-camera housing must lie on the positive EEF Z side")
            matrix = check_transform(mount @ base_to_eef)
            if not 0.15 < matrix[2, 3] < 0.25:
                raise ValueError("Unexpected gripper geometry scale")
            self.tip_to_eef.append(matrix)
            self.finger_points.append((base, mount, base_to_eef[:3, :3], finger_points))
            self.evidence["sides"].append(
                {
                    "side": side,
                    "base_to_eef": base_to_eef.tolist(),
                    "tip_to_eef": matrix.tolist(),
                    "meshes": meshes,
                    "camera_housing": {
                        "resource": camera_resource,
                        "sha256": file_hash(camera_path),
                        "bbox_center_in_eef_m": camera_center_in_eef.tolist(),
                    },
                }
            )

    def relative_zero(self, ancestor, descendant):
        matrix = np.eye(4)
        visited = set()
        while descendant != ancestor:
            if descendant in visited or descendant not in self.parents:
                raise ValueError(f"No unique URDF chain from {ancestor} to {descendant}")
            visited.add(descendant)
            joint = self.parents[descendant]
            matrix = joint_origin(joint) @ matrix
            descendant = joint.find("parent").get("link")
        return check_transform(matrix)

    def joint_value(self, joint, positions):
        name = joint.get("name")
        if name in positions:
            return float(positions[name])
        mimic = joint.find("mimic")
        if mimic is None:
            return 0.0
        source = self.joints[mimic.get("joint")]
        return self.joint_value(source, positions) * float(mimic.get("multiplier", "1")) + float(
            mimic.get("offset", "0")
        )

    def joint_motion_weights(self, name, command_names, visiting=()):
        if name in visiting:
            raise ValueError(f"Cyclic URDF mimic chain: {name}")
        weights = np.zeros(len(command_names))
        if name in command_names:
            weights[command_names.index(name)] = 1.0
            return weights
        mimic = self.joints[name].find("mimic")
        if mimic is None:
            return weights
        return float(mimic.get("multiplier", "1")) * self.joint_motion_weights(
            mimic.get("joint"), command_names, (*visiting, name)
        )

    def opening_positions(self, openings):
        values = np.asarray(openings, dtype=float)
        if values.shape != (2,) or not np.isfinite(values).all() or np.any(values < 0) or np.any(values > 1):
            raise ValueError("Two normalized gripper openings in [0,1] are required")
        positions = {}
        for name, opening in zip(self.master_names, values):
            joint = self.joints[name]
            limit = joint.find("limit")
            lower, upper = float(limit.get("lower")), float(limit.get("upper"))
            positions[name] = lower + float(opening) * (upper - lower)
        for joint in self.robot.findall("joint"):
            if joint.find("mimic") is not None and "_gripper_" in joint.get("name"):
                positions[joint.get("name")] = self.joint_value(joint, positions)
        return positions

    def openings_from_positions(self, positions):
        result = []
        for name in self.master_names:
            joint = self.joints[name]
            limit = joint.find("limit")
            lower, upper = float(limit.get("lower")), float(limit.get("upper"))
            result.append((positions.get(name, upper) - lower) / (upper - lower))
        return result

    def offsets_at(self, positions=None):
        if positions is None:
            return self.tip_to_eef
        result = []
        for base, mount, rotation, fingers in self.finger_points:
            centers = []
            for link, point in fingers:
                matrix = np.eye(4)
                while link != base:
                    joint = self.parents[link]
                    motion = np.eye(4)
                    if joint.get("type") != "fixed":
                        axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                        motion[:3, :3] = Rotation.from_rotvec(axis * self.joint_value(joint, positions)).as_matrix()
                    matrix = self.origins[joint.get("name")] @ motion @ matrix
                    link = joint.find("parent").get("link")
                centers.append((matrix @ point)[:3])
            offset = np.eye(4)
            offset[:3, :3] = rotation
            offset[:3, 3] = np.mean(centers, axis=0)
            result.append(check_transform(mount @ offset))
        return result

    def to_eef(self, tips, positions=None):
        if len(tips) != 2:
            raise ValueError("Exactly two flange poses are required")
        return [pose_dict(pose_matrix(pose) @ offset) for pose, offset in zip(tips, self.offsets_at(positions))]

    def to_flange(self, eef, positions=None):
        if len(eef) != 2:
            raise ValueError("Exactly two gripper EEF poses are required")
        return [
            pose_dict(pose_matrix(pose) @ np.linalg.inv(offset))
            for pose, offset in zip(eef, self.offsets_at(positions))
        ]
