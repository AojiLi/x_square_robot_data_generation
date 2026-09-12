#!/usr/bin/env python3
"""Verify actual contacts, fixed-base stability and hand mimic motion in Isaac."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser()
parser.add_argument("--steps", type=int, default=480)
parser.add_argument("--diagnostic-no-self-collision", action="store_true")
parser.add_argument("--diagnostic-no-mimic", action="store_true")
parser.add_argument("--diagnostic-contacts", action="store_true")
parser.add_argument("--report", type=Path, default=ROOT / "reports/room01_sim/physics_verification.json")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
from room01_sim.bootstrap import launch
launcher = launch(args)
app = launcher.app
exit_code = 0

try:
    import numpy as np
    import torch
    from room01_sim.runtime import RoomTaskSim, tensor
    runtime = RoomTaskSim(app, device=args.device, cameras=False,
                          diagnostic_self_collision=not args.diagnostic_no_self_collision,
                          diagnostic_mimic=not args.diagnostic_no_mimic,
                          diagnostic_contacts=args.diagnostic_contacts)
    print("ROBOT_READY", runtime.robot.is_fixed_base, runtime.robot.num_joints, runtime.robot.num_bodies, flush=True)
    print("JOINT_NAMES", runtime.joint_names, flush=True)
    initial_base = tensor(runtime.robot.data.root_link_pose_w).cpu().numpy().copy()
    for step in range(0, args.steps, 20):
        runtime.advance_physics(min(20, args.steps-step))
        qq = tensor(runtime.robot.data.joint_pos)[0].cpu().numpy()
        vv = tensor(runtime.robot.data.joint_vel)[0].cpu().numpy()
        print("PHYSICS_SAMPLE", runtime.elapsed_steps, "qmax",float(np.abs(qq).max()), "vmax",float(np.abs(vv).max()), "joint", runtime.joint_names[int(np.abs(vv).argmax())], flush=True)
    q = tensor(runtime.robot.data.joint_pos)[0].cpu().numpy()
    target = runtime.targets[0].cpu().numpy()
    settled = runtime.observe(images=False)
    joint_errors = {name: float(q[i]-target[i]) for i, name in enumerate(runtime.joint_names)}
    from room01_sim.kinematics import quaternion_matrix
    base_pose = tensor(runtime.robot.data.root_link_pose_w)[0].cpu().numpy()
    base_matrix = np.eye(4);base_matrix[:3, :3] = quaternion_matrix(base_pose[3:]);base_matrix[:3, 3] = base_pose[:3]
    positions = dict(runtime.config["joint_home_rad"]);positions.update(zip(runtime.joint_names, q))
    fk = runtime.kinematics.forward(positions, base=base_matrix)
    body_poses = tensor(runtime.robot.data.body_link_pose_w)[0].cpu().numpy()
    fk_errors = {name: float(np.linalg.norm(body_poses[i, :3]-fk[name][:3, 3])) for i, name in enumerate(runtime.robot.body_names)}
    print("SETTLED", settled, "JOINT_ERRORS", joint_errors, flush=True)
    results = {
        "fixed_base": bool(runtime.robot.is_fixed_base),
        "joints": runtime.robot.num_joints, "links": runtime.robot.num_bodies,
        "base_drift_m": float(np.linalg.norm(tensor(runtime.robot.data.root_link_pose_w).cpu().numpy()[0, :3]-initial_base[0, :3])),
        "table_cube_z": float(settled["cube_pose_xyzw"][2]),
        "expected_table_cube_z": runtime.config["tabletop_z"]+runtime.config["cube_size"][2]/2,
        "joint_errors_rad": joint_errors,
        "settled_joint_velocity_rad_s": dict(zip(runtime.joint_names, tensor(runtime.robot.data.joint_vel)[0].cpu().numpy().astype(float).tolist())),
        "config_sha256": hashlib.sha256((ROOT / "assets/room01/sim/task_config.json").read_bytes()).hexdigest(),
        "all_states_finite": bool(np.isfinite(q).all()),
        "mimic_joint_errors_rad": {},
        "urdf_fk_link_position_errors_m": fk_errors,
        "contacts": runtime.contact_pairs,
    }
    # Exercise one finger with its physical two-way mimic constraint.
    index = runtime.joint_names.index("right_index_proximal_joint")
    runtime.targets[0, index] = .5
    runtime.advance_physics(240)
    q = tensor(runtime.robot.data.joint_pos)[0].cpu().numpy()
    for name, joint in runtime.kinematics.joints.items():
        mimic = joint["mimic"]
        if mimic:
            target_angle = q[runtime.joint_names.index(mimic["joint"])]*float(mimic.get("multiplier", 1.))+float(mimic.get("offset", 0.))
            results["mimic_joint_errors_rad"][name] = float(q[runtime.joint_names.index(name)]-target_angle)
    # A separate drop exercises the floor collider after the table contact test.
    pose = torch.tensor([[-1.0, -1.1, .4, 0., 0., 0., 1.]], device=runtime.device)
    runtime.cube.write_root_pose_to_sim_index(root_pose=pose)
    runtime.cube.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=runtime.device))
    runtime.advance_physics(360)
    results["floor_cube_z"] = float(tensor(runtime.cube.data.root_link_pose_w)[0, 2])
    results["expected_floor_cube_z"] = .02
    checks = {
        "fixed_base": results["fixed_base"] and results["base_drift_m"] < 1e-4,
        "table_contact": abs(results["table_cube_z"]-results["expected_table_cube_z"]) < .005,
        "floor_contact": abs(results["floor_cube_z"]-.02) < .005,
        "finite_states": results["all_states_finite"],
        "reset_pose_tracking": max(abs(v) for v in joint_errors.values()) < .03,
        "reset_pose_at_rest": max(abs(v) for v in results["settled_joint_velocity_rad_s"].values()) < .05,
        "mimic_constraints": max(abs(v) for v in results["mimic_joint_errors_rad"].values()) < .03,
        "urdf_fk": max(results["urdf_fk_link_position_errors_m"].values()) < .005,
    }
    results["checks"] = checks
    results["result"] = "pass" if all(checks.values()) else "fail"
    output = args.report
    output.write_text(json.dumps(results, indent=2))
    print("PHYSICS_VERIFIED", json.dumps(results), flush=True)
    runtime.close()
    if results["result"] != "pass":
        raise RuntimeError("Physical validation failed; see physics_verification.json")
except Exception as error:
    exit_code = 1
    import traceback
    traceback.print_exc()
    if 'results' not in globals():
        args.report.write_text(json.dumps({"result": "fail", "error": str(error)}, indent=2))
    print("PHYSICS_FAILED", str(error), flush=True)
finally:
    app.close(exit_code=exit_code)
