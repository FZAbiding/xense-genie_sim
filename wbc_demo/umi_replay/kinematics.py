import importlib
from pathlib import Path

from .differential_ik import URDFKinematics


def validate_backend(name):
    if name not in ("urdf", "pinocchio"):
        raise ValueError(f"Unknown kinematics backend: {name}")
    if name == "pinocchio":
        importlib.import_module("pinocchio")
    return name


def create_kinematics(name, eef, root, names):
    validate_backend(name)
    if name == "urdf":
        return URDFKinematics(eef, root, names)
    from .pinocchio_backend import PinocchioKinematics

    return PinocchioKinematics(eef, root, names, Path(eef.package_root) / "urdf/genie_g2_crsB_swiftpicker.urdf")
