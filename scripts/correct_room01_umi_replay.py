#!/usr/bin/env python3
"""Apply explicitly bounded Cartesian corrections without changing scene registration."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from room01_sim.umi import ArmChain
from room01_sim.kinematics import RobotKinematics


def curve(times, knots, values):
    # Smoothstep within each interval preserves the convex bound on offsets.
    result = np.empty((len(times), 3))
    for i, t in enumerate(times):
        interval = np.clip(np.searchsorted(knots, t, side="right")-1, 0, len(knots)-2)
        u = np.clip((t-knots[interval])/(knots[interval+1]-knots[interval]), 0, 1)
        u = u*u*(3-2*u)
        result[i] = (1-u)*np.asarray(values[interval])+u*np.asarray(values[interval+1])
    return result


def main(args):
    data = dict(np.load(args.prepared / "trajectory.npz"))
    config = json.loads((ROOT / "assets/room01/umi_replay/task_config.json").read_text())
    specification = json.loads(args.corrections.read_text())
    times = data["times"]
    nominal = data["hand_targets"].copy()
    corrected = nominal.copy()
    position_max, rotation_max = 0., 0.
    for index, side in enumerate(("left", "right")):
        spec = specification[side]
        delta = curve(times, spec["times"], spec["position_world_m"])
        rv = curve(times, spec["times"], spec["rotation_vector_deg"])
        position_max = max(position_max, float(np.linalg.norm(delta, axis=1).max()))
        rotation_max = max(rotation_max, float(np.linalg.norm(rv, axis=1).max()))
        corrected[:, index, :3, 3] += delta
        rotation_delta = Rotation.from_rotvec(np.deg2rad(rv)).as_matrix()
        if spec["rotation_frame"] == "world":
            corrected[:, index, :3, :3] = rotation_delta @ nominal[:, index, :3, :3]
        elif spec["rotation_frame"] == "hand_base":
            corrected[:, index, :3, :3] = nominal[:, index, :3, :3] @ rotation_delta
        else:
            raise ValueError("Unknown correction rotation frame")
    if position_max > .010000001 or rotation_max > 5.000001:
        raise ValueError("Correction exceeds the user-approved 1 cm / 5 degree bounds")
    robot = RobotKinematics()
    base = np.eye(4); base[:3, 3] = config["robot_base_position"]
    q, errors = data["targets"].copy(), np.zeros((len(times), 2, 2))
    for side, name in enumerate(("left", "right")):
        chain = ArmChain(robot, name, config["joint_home_rad"], base)
        previous = q[0, side*13:side*13+7]
        for i, target in enumerate(corrected[:, side]):
            previous, distance, angle = chain.solve(target, previous)
            q[i, side*13:side*13+7] = previous
            errors[i, side] = [distance, angle]
    if errors[:, :, 0].max() > .005 or errors[:, :, 1].max() > .05:
        raise ValueError("Corrected trajectory failed IK tolerance")
    # Finger columns are bit-for-bit unchanged.
    audit = {"source_preparation": str(args.prepared), "specification": specification,
             "max_position_correction_m": position_max, "max_rotation_correction_deg": rotation_max,
             "fingers_unchanged": True, "max_ik_position_error_m": float(errors[:, :, 0].max())}
    data.update(targets=q, hand_targets=corrected, nominal_hand_targets=nominal,
                ik_errors=errors, correction_audit_json=np.asarray(json.dumps(audit)))
    args.output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output / "trajectory.npz", **data)
    (args.output / "corrections.json").write_text(json.dumps(audit, indent=2))
    print(json.dumps(audit, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("corrections", type=Path)
    parser.add_argument("--prepared", type=Path, default=ROOT / "outputs/room01/umi_prepared")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/room01/umi_corrected")
    main(parser.parse_args())
