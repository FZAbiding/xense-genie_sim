import math

import numpy as np
import osqp
from scipy import sparse
from scipy.spatial.transform import Rotation

from .geometry import check_transform, pose_matrix
from .validation import validate_realtime_profile


class PlaybackClock:
    def __init__(self, start, duration):
        if not math.isfinite(start) or not math.isfinite(duration) or duration <= 0:
            raise ValueError("A finite start and positive duration are required")
        self.start = start
        self.duration = duration
        self.paused_at = None
        self.pause_seconds = 0.0

    def pause(self, now, paused):
        if paused and self.paused_at is None:
            self.paused_at = now
        elif not paused and self.paused_at is not None:
            self.pause_seconds += now - self.paused_at
            self.paused_at = None

    def phase(self, now):
        end = now if self.paused_at is None else self.paused_at
        return min(self.duration, max(0.0, end - self.start - self.pause_seconds))


class URDFKinematics:
    def __init__(self, eef, root_to_odom, names):
        self.eef = eef
        self.root = check_transform(root_to_odom)
        self.names = list(names)
        self.index = {name: index for index, name in enumerate(names)}
        self.mimic_weights = {
            name: eef.joint_motion_weights(name, self.names)
            for name, joint in eef.joints.items()
            if joint.find("mimic") is not None
        }
        if len(self.index) != len(self.names):
            raise ValueError("Duplicate command joints")
        self.chains = []
        for tip in eef.tip_names:
            chain = []
            link = tip
            while link != "base_link":
                joint = eef.parents[link]
                kind = joint.get("type")
                if kind not in ("fixed", "revolute", "prismatic", "continuous"):
                    raise ValueError(f"Unsupported URDF joint type: {kind}")
                axis_node = joint.find("axis")
                axis = (
                    np.fromstring(axis_node.get("xyz", "1 0 0"), sep=" ")
                    if axis_node is not None
                    else np.array([1.0, 0.0, 0.0])
                )
                if kind != "fixed" and abs(np.linalg.norm(axis) - 1) > 1e-6:
                    raise ValueError("URDF joint axis must be unit length")
                name = joint.get("name")
                chain.append((name, kind, eef.origins[name], axis))
                link = joint.find("parent").get("link")
            self.chains.append(list(reversed(chain)))
        supported = {name for chain in self.chains for name, kind, origin, axis in chain} | set(eef.master_names)
        if not set(self.names) <= supported:
            raise ValueError("Command joints must belong to the waist, arms or grippers")

    def evaluate(self, positions, jacobian=True):
        offsets = self.eef.offsets_at(positions)
        poses = []
        derivatives = []
        for side, (chain, offset) in enumerate(zip(self.chains, offsets)):
            matrix = self.root.copy()
            axes = []
            for name, kind, origin, axis in chain:
                matrix = matrix @ origin
                if name in self.index:
                    axes.append((self.index[name], kind, matrix[:3, :3] @ axis, matrix[:3, 3].copy()))
                value = positions.get(name, 0.0)
                if kind in ("revolute", "continuous"):
                    matrix[:3, :3] = matrix[:3, :3] @ Rotation.from_rotvec(axis * value).as_matrix()
                elif kind == "prismatic":
                    matrix[:3, 3] += matrix[:3, :3] @ (axis * value)
            pose = matrix @ offset
            poses.append(pose)
            if not jacobian:
                continue
            derivative = np.zeros((6, len(self.names)))
            for index, kind, axis, origin in axes:
                if kind in ("revolute", "continuous"):
                    derivative[:3, index] = np.cross(axis, pose[:3, 3] - origin)
                    derivative[3:, index] = axis
                elif kind == "prismatic":
                    derivative[:3, index] = axis
            master = self.eef.master_names[side]
            if master in self.index:
                perturbed = dict(positions)
                perturbed[master] = positions[master] + 1e-6
                for name, weights in self.mimic_weights.items():
                    if name in positions:
                        perturbed[name] = positions[name] + 1e-6 * weights[self.index[master]]
                shifted = self.eef.offsets_at(perturbed)[side]
                derivative[:3, self.index[master]] = matrix[:3, :3] @ ((shifted[:3, 3] - offset[:3, 3]) / 1e-6)
            derivatives.append(derivative)
        return poses, derivatives


def matrix_error(actual, target):
    return np.r_[target[:3, 3] - actual[:3, 3], Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec()]


class DifferentialIK:
    def __init__(self, model, positions, native_limits, native_velocities, coupled, profile, bounds=None):
        validate_realtime_profile(profile)
        self.model = model
        self.names = model.names
        self.reference = np.array([positions[name] for name in self.names])
        self.lower = np.array([native_limits[name][0] for name in self.names])
        self.upper = np.array([native_limits[name][1] for name in self.names])
        for name, limits in (bounds or {}).items():
            index = self.names.index(name)
            if not self.lower[index] <= limits[0] < limits[1] <= self.upper[index]:
                raise ValueError("Posture bounds must lie inside native bounds")
            self.lower[index], self.upper[index] = limits
        groups = [
            "gripper" if name in model.eef.master_names else "waist" if "_body_joint" in name else "arm"
            for name in self.names
        ]
        self.velocity = np.array(
            [min(native_velocities[name], profile[group]["velocity_rad_s"]) for name, group in zip(self.names, groups)]
        )
        self.acceleration = np.array([profile[group]["acceleration_rad_s2"] for group in groups])
        if (
            not np.isfinite(np.r_[self.velocity, self.acceleration]).all()
            or min(self.velocity.min(), self.acceleration.min()) <= 0
        ):
            raise ValueError("Finite positive velocity and acceleration limits are required")
        rows, ceilings = [], []
        for constraint in coupled:
            indexes = [self.names.index(name) for name in constraint["joint_names"]]
            for coefficients, ceiling in zip(constraint["A"], constraint["b"]):
                row = np.zeros(len(self.names))
                row[indexes] = coefficients
                rows.append(row)
                ceilings.append(ceiling)
        self.coupled = np.array(rows).reshape(-1, len(self.names))
        self.ceiling = np.array(ceilings)
        self.gripper_indexes = [self.names.index(name) for name in model.eef.master_names]

    def step(self, positions, targets, openings, previous_velocity, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("Positive finite integration period required")
        current = np.array([positions[name] for name in self.names])
        previous_velocity = np.asarray(previous_velocity)
        if previous_velocity.shape != current.shape or not np.isfinite(np.r_[current, previous_velocity]).all():
            raise ValueError("Invalid joint positions or previous velocity")
        poses, jacobians = self.model.evaluate(positions)
        weights = np.array([1, 1, 1, 0.18, 0.18, 0.18])
        task = np.vstack([weights[:, None] * item for item in jacobians])
        desired = np.concatenate(
            [weights * matrix_error(actual, pose_matrix(target)) / dt for actual, target in zip(poses, targets)]
        )
        gripper_task = np.zeros((2, len(self.names)))
        gripper_task[np.arange(2), self.gripper_indexes] = 0.4
        gripper_positions = self.model.eef.opening_positions(openings)
        gripper_velocity = (
            0.4
            * (
                np.array([gripper_positions[name] for name in self.model.eef.master_names])
                - current[self.gripper_indexes]
            )
            / dt
        )
        task = np.vstack([task, gripper_task])
        desired = np.r_[desired, gripper_velocity]
        damping = np.full(len(self.names), 2e-5)
        preference = 0.3 * (self.reference - current)
        if "idx04_body_joint4" in self.names:
            index = self.names.index("idx04_body_joint4")
            damping[index] = 0.04
            preference[index] = -2 * current[index]
        lower = np.maximum.reduce(
            [-self.velocity, (self.lower - current) / dt, previous_velocity - self.acceleration * dt]
        )
        upper = np.minimum.reduce(
            [self.velocity, (self.upper - current) / dt, previous_velocity + self.acceleration * dt]
        )
        if np.any(lower > upper + 1e-7):
            raise RuntimeError("Joint position/velocity/acceleration bounds are infeasible")
        matrix = np.vstack([np.eye(len(self.names)), self.coupled])
        lower = np.r_[np.minimum(lower, upper), np.full(len(self.ceiling), -np.inf)]
        upper = np.r_[upper, (self.ceiling - self.coupled @ current) / dt]
        solver = osqp.OSQP()
        solver.setup(
            P=sparse.csc_matrix(task.T @ task + np.diag(damping)),
            q=-task.T @ desired - damping * preference,
            A=sparse.csc_matrix(matrix),
            l=lower,
            u=upper,
            verbose=False,
            eps_abs=1e-6,
            eps_rel=1e-6,
            max_iter=4000,
            polishing=False,
            time_limit=0.008,
        )
        solution = solver.solve(raise_error=False)
        self.last_status = solution.info.status
        if solution.info.status_val not in (1, 2) or solution.x is None or not np.isfinite(solution.x).all():
            raise RuntimeError(f"Differential IK QP failed: {solution.info.status}")
        velocity = solution.x
        if np.any(matrix @ velocity < lower - 1e-4) or np.any(matrix @ velocity > upper + 1e-4):
            raise RuntimeError("Differential IK constraint residual is too large")
        result = dict(positions)
        result.update(zip(self.names, np.clip(current + dt * velocity, self.lower, self.upper)))
        for name, joint in self.model.eef.joints.items():
            if joint.find("mimic") is not None:
                result.pop(name, None)
        for name, joint in self.model.eef.joints.items():
            if joint.find("mimic") is not None:
                result[name] = self.model.eef.joint_value(joint, result)
        return result, velocity
