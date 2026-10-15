import numpy as np

from .geometry import check_transform


class PinocchioKinematics:
    def __init__(self, eef, root_to_odom, names, urdf_path):
        import pinocchio

        self.pin = pinocchio
        self.eef = eef
        self.root = check_transform(root_to_odom)
        self.names = list(names)
        if len(set(names)) != len(names):
            raise ValueError("Duplicate command joints")
        self.model = pinocchio.buildModelFromUrdf(str(urdf_path), mimic=False)
        self.velocity_map = np.zeros((self.model.nv, len(names)))
        self.configuration_joints = []
        for joint_id in range(1, self.model.njoints):
            name = self.model.names[joint_id]
            joint = self.model.joints[joint_id]
            if joint.nv != 1 or joint.nq not in (1, 2):
                raise ValueError(f"Unsupported scalar URDF joint: {name}")
            self.configuration_joints.append((name, joint.idx_q, joint.nq))
            self.velocity_map[joint.idx_v] = eef.joint_motion_weights(name, self.names)
        for name in self.names:
            if not self.model.existJointName(name):
                raise ValueError(f"Command joint absent from Pinocchio model: {name}")
        self.base_frame = self.frame_id("base_link")
        self.tip_frames = [self.frame_id(name) for name in eef.tip_names]
        self.point_frames = []
        for side, (base, mount, rotation, fingers) in enumerate(eef.finger_points):
            points = []
            for index, (link, point) in enumerate(fingers):
                parent_id = self.frame_id(link)
                parent = self.model.frames[parent_id]
                placement = parent.placement * pinocchio.SE3(np.eye(3), point[:3])
                frame = pinocchio.Frame(
                    f"umi_eef_point_{side}_{index}",
                    parent.parentJoint,
                    parent_id,
                    placement,
                    pinocchio.FrameType.OP_FRAME,
                )
                points.append(self.model.addFrame(frame))
            self.point_frames.append(points)
        self.data = self.model.createData()

    def frame_id(self, name):
        if not self.model.existFrame(name):
            raise ValueError(f"Required frame absent from Pinocchio model: {name}")
        return self.model.getFrameId(name)

    def configuration(self, positions):
        configuration = self.pin.neutral(self.model)
        for name, index, size in self.configuration_joints:
            value = self.eef.joint_value(self.eef.joints[name], positions)
            if not np.isfinite(value):
                raise ValueError(f"Nonfinite joint position: {name}")
            if size == 1:
                configuration[index] = value
            else:
                configuration[index : index + 2] = [np.cos(value), np.sin(value)]
        return configuration

    def evaluate(self, positions, jacobian=True):
        configuration = self.configuration(positions)
        if jacobian:
            self.pin.computeJointJacobians(self.model, self.data, configuration)
        else:
            self.pin.forwardKinematics(self.model, self.data, configuration)
        self.pin.updateFramePlacements(self.model, self.data)
        anchor = self.root @ self.data.oMf[self.base_frame].inverse().homogeneous
        poses, derivatives = [], []
        for side, (tip_id, points) in enumerate(zip(self.tip_frames, self.point_frames)):
            pose = np.eye(4)
            pose[:3, 3] = np.mean([self.data.oMf[point].translation for point in points], axis=0)
            pose[:3, :3] = self.data.oMf[tip_id].rotation @ self.eef.tip_to_eef[side][:3, :3]
            poses.append(anchor @ pose)
            if not jacobian:
                continue
            point_jacobians = [
                self.pin.getFrameJacobian(self.model, self.data, point, self.pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
                for point in points
            ]
            tip_jacobian = self.pin.getFrameJacobian(
                self.model, self.data, tip_id, self.pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )
            derivative = (
                np.vstack([np.mean([item[:3] for item in point_jacobians], axis=0), tip_jacobian[3:]])
                @ self.velocity_map
            )
            derivative[:3] = anchor[:3, :3] @ derivative[:3]
            derivative[3:] = anchor[:3, :3] @ derivative[3:]
            derivatives.append(derivative)
        return poses, derivatives

    def mass_matrix(self, positions):
        matrix = self.pin.crba(self.model, self.data, self.configuration(positions))
        return self.velocity_map.T @ matrix @ self.velocity_map
