"""Offline MJCF gravity torque preview, or a locally fitted current preview."""
import argparse
import json
from pathlib import Path

import numpy as np

from calibration import MOTOR_SIGNS, predict_local

HERE = Path(__file__).resolve().parent


class GravityTorqueEvaluator:
    """Evaluate zero-velocity MJCF holding torque in robot-joint coordinates."""

    def __init__(self, config_path, mjcf_path, gripper_rad=0.0):
        import mujoco

        if not np.isfinite(gripper_rad) or not 0 <= gripper_rad <= 0.78:
            raise ValueError("Gripper position must be in 0..0.78 rad")
        self.mujoco = mujoco
        self.config = json.loads(Path(config_path).read_text(encoding="utf-8"))
        self.model = mujoco.MjModel.from_xml_path(str(Path(mjcf_path).resolve()))
        self.data = mujoco.MjData(self.model)
        self.mappings = self.config["robot"]["joints"]
        self.signs = []
        self.qpos_addresses = []
        self.dof_addresses = []
        for mapping in self.mappings:
            joint = self.model.joint(mapping["model_joint"])
            sign = mapping["model_sign"]
            if sign not in (-1, 1) or int(self.model.jnt_type[joint.id]) != int(
                    mujoco.mjtJoint.mjJNT_HINGE):
                raise ValueError("This preview supports one-DOF revolute arm joints")
            self.signs.append(sign)
            self.qpos_addresses.append(int(self.model.jnt_qposadr[joint.id]))
            self.dof_addresses.append(int(self.model.jnt_dofadr[joint.id]))
        self.finger_addresses = [
            int(self.model.jnt_qposadr[self.model.joint(name).id])
            for name in self.config["robot"]["gripper"]["model_joints"]
        ]
        self.gripper_rad = float(gripper_rad)

    def joint_torque(self, q_deg, gripper_rad=None):
        """Return six joint-positive N*m values with zero joint velocity."""
        q_deg = np.asarray(q_deg, dtype=float)
        if q_deg.shape != (6,) or not np.isfinite(q_deg).all():
            raise ValueError("Expected six finite joint angles")
        if gripper_rad is None:
            gripper_rad = self.gripper_rad
        if not np.isfinite(gripper_rad) or not 0 <= gripper_rad <= 0.78:
            raise ValueError("Gripper position must be in 0..0.78 rad")
        for q, mapping in zip(q_deg, self.mappings):
            if not mapping["min_deg"] <= q <= mapping["max_deg"]:
                raise ValueError("Joint angle exceeds the real robot software limits")
            joint = self.model.joint(mapping["model_joint"])
            value = np.deg2rad(mapping["model_sign"] * q + mapping["model_offset_deg"])
            if (self.model.jnt_limited[joint.id]
                    and not self.model.jnt_range[joint.id, 0] <= value
                    <= self.model.jnt_range[joint.id, 1]):
                raise ValueError(
                    f"{mapping['model_joint']}: outside MJCF range; "
                    "reconcile model/hardware limits")
        for address, q, mapping in zip(self.qpos_addresses, q_deg, self.mappings):
            self.data.qpos[address] = np.deg2rad(
                mapping["model_sign"] * q + mapping["model_offset_deg"])
        for address in self.finger_addresses:
            self.data.qpos[address] = gripper_rad
        self.data.qvel[:] = 0
        self.mujoco.mj_forward(self.model, self.data)
        return np.asarray([
            sign * self.data.qfrc_bias[address]
            for sign, address in zip(self.signs, self.dof_addresses)
        ])


def model_gravity(q_deg, config_path, mjcf_path, gripper_rad=0.0):
    return GravityTorqueEvaluator(config_path, mjcf_path, gripper_rad).joint_torque(q_deg)


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

