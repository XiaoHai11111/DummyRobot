"""Offline MJCF gravity torque preview, or a locally fitted current preview."""
import argparse
import json
from pathlib import Path

import numpy as np

from calibration import MOTOR_SIGNS, predict_local

HERE = Path(__file__).resolve().parent


def model_gravity(q_deg, config_path, mjcf_path, gripper_rad=0.0):
    import mujoco

    config = json.loads(Path(config_path).read_text())
    q_deg = np.asarray(q_deg, dtype=float)
    if q_deg.shape != (6,) or not np.isfinite(q_deg).all() or not np.isfinite(gripper_rad):
        raise ValueError("Six finite joint angles and a finite gripper position are required")
    if not 0 <= gripper_rad <= 0.78:
        raise ValueError("Gripper position must be in 0..0.78 rad")
    model = mujoco.MjModel.from_xml_path(str(Path(mjcf_path).resolve()))
    data = mujoco.MjData(model)
    indexes, signs = [], []
    for q, mapping in zip(q_deg, config["robot"]["joints"]):
        joint = model.joint(mapping["model_joint"])
        sign = mapping["model_sign"]
        if sign not in (-1, 1) or int(model.jnt_type[joint.id]) != int(mujoco.mjtJoint.mjJNT_HINGE):
            raise ValueError("This preview supports one-DOF revolute arm joints")
        if not mapping["min_deg"] <= q <= mapping["max_deg"]:
            raise ValueError("Joint angle exceeds the real robot software limits")
        value = np.deg2rad(sign * q + mapping["model_offset_deg"])
        if model.jnt_limited[joint.id] and not model.jnt_range[joint.id, 0] <= value <= model.jnt_range[joint.id, 1]:
            raise ValueError(f"{mapping['model_joint']}: outside MJCF range; reconcile model/hardware limits")
        data.qpos[model.jnt_qposadr[joint.id]] = value
        indexes.append(int(model.jnt_dofadr[joint.id]))
        signs.append(sign)
    for name in config["robot"]["gripper"]["model_joints"]:
        data.qpos[model.jnt_qposadr[model.joint(name).id]] = gripper_rad
    data.qvel[:] = 0
    # qfrc_bias = C(q,v)v + g(q). It excludes constraint/contact and actuator force.
    mujoco.mj_forward(model, data)
    torque = np.asarray(signs) * data.qfrc_bias[indexes].copy()
    return torque


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--q-deg", type=float, nargs=6, required=True)
    parser.add_argument("--local-model", type=Path)
    parser.add_argument("--mapping", type=Path, default=HERE.parent / "DummyWeb/config.json")
    parser.add_argument("--mjcf", type=Path, default=HERE.parent / "DummyMujoco/dummy_description/mjcf/dummy.xml")
    parser.add_argument("--gripper-rad", type=float, default=0.0)
    args = parser.parse_args()
    if args.local_model:
        model = json.loads(args.local_model.read_text())
        current = predict_local(model, args.q_deg)
        print(json.dumps({"joint": model["joint"], "joint_positive_current_a": current,
                          "motor_current_a": float(MOTOR_SIGNS[model["joint"] - 1] * current),
                          "hardware_command_sent": False}, indent=2))
    else:
        torque = model_gravity(args.q_deg, args.mapping, args.mjcf, args.gripper_rad)
        print(json.dumps({"gravity_joint_nm": torque.tolist(), "hardware_command_sent": False,
                          "note": "CAD/model prior only; validate masses, tool load and joint mapping before use"},
                         indent=2))


if __name__ == "__main__":
    main()

