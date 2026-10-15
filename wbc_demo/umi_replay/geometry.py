import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def rotation_6d(values):
    values = np.asarray(values, dtype=float)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError("Rotation must contain six finite column-major values")
    first = values[:3]
    second = values[3:]
    if abs(np.linalg.norm(first) - 1) > 0.01 or abs(np.linalg.norm(second) - 1) > 0.01:
        raise ValueError("Rotation columns are not unit vectors")
    if abs(np.dot(first, second)) > 0.01:
        raise ValueError("Rotation columns are not orthogonal")
    first = first / np.linalg.norm(first)
    second = second - np.dot(first, second) * first
    if np.linalg.norm(second) < 1e-8:
        raise ValueError("Degenerate rotation representation")
    second /= np.linalg.norm(second)
    return np.column_stack([first, second, np.cross(first, second)])


def transform(position, quaternion_xyzw):
    position = np.asarray(position, dtype=float)
    quaternion = np.asarray(quaternion_xyzw, dtype=float)
    if position.shape != (3,) or quaternion.shape != (4,) or not np.isfinite(np.r_[position, quaternion]).all():
        raise ValueError("Invalid SE(3) values")
    if abs(np.linalg.norm(quaternion) - 1) > 0.001:
        raise ValueError("Quaternion must have unit norm")
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    matrix[:3, 3] = position
    return matrix


def check_transform(matrix):
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("Transform must be a finite 4x4 matrix")
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError("Invalid homogeneous last row")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or abs(np.linalg.det(rotation) - 1) > 1e-6:
        raise ValueError("Transform rotation must be proper orthonormal")
    return matrix


def pose_dict(matrix):
    matrix = check_transform(matrix)
    return {
        "position": matrix[:3, 3].tolist(),
        "quaternion_xyzw": Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist(),
    }


def pose_matrix(pose):
    return transform(pose["position"], pose["quaternion_xyzw"])


def interpolate_pose(first, second, fraction):
    position = (1 - fraction) * np.array(first["position"]) + fraction * np.array(second["position"])
    rotation = Slerp([0, 1], Rotation.from_quat([first["quaternion_xyzw"], second["quaternion_xyzw"]]))(fraction)
    return {"position": position.tolist(), "quaternion_xyzw": rotation.as_quat().tolist()}


def pose_error(first, second):
    position = float(np.linalg.norm(np.array(first["position"]) - second["position"]))
    rotation = Rotation.from_quat(first["quaternion_xyzw"]).inv() * Rotation.from_quat(second["quaternion_xyzw"])
    return position, float(rotation.magnitude())


def map_poses(poses, world_to_odom, tip_to_eef):
    if len(poses) != 2 or len(tip_to_eef) != 2:
        raise ValueError("Exactly two source poses and two tool transforms are required")
    world_to_odom = check_transform(world_to_odom)
    offsets = [check_transform(matrix) for matrix in tip_to_eef]
    return [
        pose_dict(world_to_odom @ pose_matrix(pose) @ np.linalg.inv(offset)) for pose, offset in zip(poses, offsets)
    ]
