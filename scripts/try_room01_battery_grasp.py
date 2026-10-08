#!/usr/bin/env python3
"""Bounded contact-based feasibility trial, using simulator pose feedback.

No object attachment, gravity override, post-reset object pose writes, or
changes to the measured battery/foam geometry. This is a diagnostic controller.
"""
import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--candidates", type=Path, required=True)
parser.add_argument("--candidate-id", type=int, default=0)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--record", action="store_true")
parser.add_argument("--insert", action="store_true", help="Continue only after a physically held lift.")
parser.add_argument("--max-wall-seconds", type=float, default=600.)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = args.record
from room01_sim.bootstrap import launch
app = launch(args).app
exit_code = 0
try:
    import numpy as np
    import torch
    from scipy.spatial.transform import Rotation, Slerp
    from pxr import UsdGeom, Usd
    from PIL import Image
    from room01_sim.runtime import RoomTaskSim, tensor
    from room01_sim.kinematics import ACTION_JOINTS, ARM_SUFFIXES, HAND_SUFFIXES, quaternion_matrix

    assets = ROOT / "assets/room01/battery_task"
    document = json.loads(args.candidates.read_text())
    candidate = next(c for c in document["candidates"] if c["candidate_id"] == args.candidate_id)
    side = candidate["side"]
    name = candidate.get("object_name", candidate.get("object"))
    hand_from_battery = np.array(candidate.get("hand_from_battery", candidate.get("hand_base_from_battery")))
    open_fingers = np.array(candidate.get("open_fingers_rad", candidate.get("open_motor_rad")))
    closed_fingers = np.array(candidate.get("closed_fingers_rad", candidate.get("closed_motor_rad")))
    runtime = RoomTaskSim(app, device=args.device, cameras=False, review_camera=args.record,
                          diagnostic_contacts=True, config_path=assets / "task_config.json",
                          scene_path=assets / "room01_battery_task.usda")
    args.output.mkdir(parents=True, exist_ok=False)
    if args.record:
        (args.output / "frames").mkdir()
    spec = json.loads((assets / "scene_spec.json").read_text())
    report = {"status": "running", "candidate": candidate, "configuration": runtime.config,
              "controller": "bounded simulator-pose-feedback diagnostic", "record_fps": 5,
              "object_pose_writes_after_reset": 0, "attachments": False, "gravity_enabled": True,
              "red_fixture_fixed": True, "foam_model": "rigid geometry approximation",
              "measured_nominal_diametral_clearance_m": 0., "physical_grasp_pass": False,
              "physical_insertion_pass": False, "real_robot_validated": False, "phases": []}
    (args.output / "trial.json").write_text(json.dumps(report, indent=2)+"\n")
    deadline = time.monotonic()+args.max_wall_seconds
    control_step = 0
    trace = []
    arm_indices = np.array([ACTION_JOINTS.index(side+"_"+suffix+"_joint") for suffix in ARM_SUFFIXES])
    finger_indices = np.array([ACTION_JOINTS.index(side+"_"+suffix+"_joint") for suffix in HAND_SUFFIXES])
    object_index = list(runtime.objects).index(name)

    def pose_matrix(pose):
        result = np.eye(4)
        result[:3, :3] = quaternion_matrix(pose[3:])
        result[:3, 3] = pose[:3]
        return result

    def observation():
        return runtime.observe(images=False)

    def object_matrix(obs):
        return pose_matrix(obs["object_poses_xyzw"][object_index])

    def hand_matrix(obs):
        return pose_matrix(obs[side+"_hand_pose_xyzw"])

    def save_frame():
        for _ in range(2):
            runtime.sim.render()
        runtime.review_camera.update(0., force_recompute=True)
        rgb = tensor(runtime.review_camera.data.output["rgb"])[0].cpu().numpy()[..., :3].astype(np.uint8)
        Image.fromarray(rgb).save(args.output / "frames" / f"{control_step//6:06d}.png")

    def advance(target_hand, fingers, phase):
        global control_step
        if time.monotonic() >= deadline:
            raise TimeoutError("Trial wall time budget exhausted")
        before = observation()
        actual_hand = hand_matrix(before)
        angular = Rotation.from_matrix(target_hand[:3, :3] @ actual_hand[:3, :3].T).as_rotvec()
        error = np.r_[target_hand[:3, 3]-actual_hand[:3, 3], .15*angular]
        positions = dict(runtime.config["joint_home_rad"])
        positions.update(zip(runtime.joint_names, tensor(runtime.robot.data.joint_pos)[0].cpu().numpy()))
        base = pose_matrix(tensor(runtime.robot.data.root_link_pose_w)[0].cpu().numpy())
        jacobian = runtime.kinematics.arm_jacobian(side, positions, base=base)
        jacobian[3:] *= .15
        delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T+.006**2*np.eye(6), .25*error)
        command = runtime.last_action.astype(np.float64).copy()
        command[arm_indices] += np.clip(delta, -.045, .045)
        command[finger_indices] = fingers
        clipped = np.clip(command, runtime.action_lower, runtime.action_upper)
        max_delta = runtime.action_velocity/runtime.config["control_hz"]
        applied = runtime.last_action+np.clip(clipped-runtime.last_action, -max_delta, max_delta)
        runtime.targets[0, runtime.action_indices] = torch.as_tensor(applied, device=runtime.device, dtype=runtime.targets.dtype)
        runtime.last_action = applied.astype(np.float32)
        runtime.advance_physics(runtime.substeps)
        after = observation()
        current_hand = hand_matrix(after)
        trace.append({"time": runtime.time, "phase": phase, "target_hand": target_hand.copy(),
                      "actual_hand": current_hand, "object_poses": after["object_poses_xyzw"].copy(),
                      "object_velocities": after["object_velocities"].copy(),
                      "joint_position": after["state"].copy(), "command": applied.copy(),
                      "position_error_m": float(np.linalg.norm(target_hand[:3, 3]-current_hand[:3, 3])),
                      "rotation_error_rad": float(np.linalg.norm(Rotation.from_matrix(target_hand[:3, :3] @ current_hand[:3, :3].T).as_rotvec()))})
        control_step += 1
        if args.record and control_step % 6 == 0:
            save_frame()
        if control_step % 30 == 0:
            print("BATTERY_TRIAL_STEP", phase, round(runtime.time, 2), "error_mm", round(trace[-1]["position_error_m"]*1000, 2),
                  "object_z", round(float(after["object_poses_xyzw"][object_index, 2]), 4), flush=True)
        return after

    def phase(name, target, seconds, fingers):
        start = hand_matrix(observation())
        initial_fingers = runtime.last_action[finger_indices].copy()
        interpolation = Slerp([0, 1], Rotation.from_matrix(np.stack([start[:3, :3], target[:3, :3]])))
        count = max(1, round(seconds*runtime.config["control_hz"]))
        first = len(trace)
        for index in range(count):
            fraction = (index+1)/count
            fraction = fraction*fraction*(3-2*fraction)
            pose = np.eye(4)
            pose[:3, :3] = interpolation(fraction).as_matrix()
            pose[:3, 3] = (1-fraction)*start[:3, 3]+fraction*target[:3, 3]
            obs = advance(pose, (1-fraction)*initial_fingers+fraction*fingers, name)
        record = {"name": name, "seconds": seconds, "final_position_error_m": trace[-1]["position_error_m"],
                  "final_rotation_error_rad": trace[-1]["rotation_error_rad"],
                  "object_pose_xyzw": obs["object_poses_xyzw"][object_index].tolist(),
                  "max_position_error_m": max(row["position_error_m"] for row in trace[first:])}
        report["phases"].append(record)
        return obs

    reason = "not_started"
    try:
        # Let the free cylinder settle on the rigid foam approximation before
        # planning from its actual pose. This does not move the object by API.
        runtime.advance_physics(4800)
        initial = observation()
        report["initial_object_poses_xyzw"] = initial["object_poses_xyzw"].tolist()
        initial_center = initial["object_poses_xyzw"][object_index, :3].copy()
        battery = object_matrix(initial)
        # Axial roll is irrelevant to the cylinder's grasp geometry. Retain
        # horizontal heading while avoiding conversion of settling roll into a
        # hand tilt that would unnecessarily penetrate the support plane.
        heading = np.arctan2(battery[1, 0], battery[0, 0])
        battery[:3, :3] = Rotation.from_euler("z", heading).as_matrix()
        grasp = battery @ np.linalg.inv(hand_from_battery)
        above = grasp.copy(); above[2, 3] += .12
        if args.record:
            from pxr import Gf
            target = [(initial_center[0]+1.677)/2, (initial_center[1]-.861)/2, .85]
            runtime.review_camera.set_world_poses_from_view(torch.tensor([[1.18, -1.42, 1.18]], device=runtime.device),
                                                           torch.tensor([target], device=runtime.device))
            runtime.review_camera.set_intrinsic_matrices(torch.tensor([[[1450., 0, 640.], [0, 1450., 480.], [0, 0, 1.]]], device=runtime.device))
            save_frame()
        phase("approach", above, 3., open_fingers)
        phase("descend", grasp, 2.5, open_fingers)
        phase("close", grasp, 2., closed_fingers)
        lifted = grasp.copy(); lifted[2, 3] += .10
        phase("lift", lifted, 2.5, closed_fingers)
        hold_start = len(trace)
        held = phase("hold", lifted, 1., closed_fingers)
        holding = trace[hold_start:]
        heights = np.array([row["object_poses"][object_index, 2] for row in holding])-initial_center[2]
        relative_centers = np.array([(np.linalg.inv(row["actual_hand"]) @ np.r_[row["object_poses"][object_index, :3], 1])[:3] for row in holding])
        relative_drift = np.linalg.norm(relative_centers-relative_centers[0], axis=1).max()
        speeds = np.array([np.linalg.norm(row["object_velocities"][object_index, :3]) for row in holding])
        report["grasp_metrics"] = {"min_hold_lift_m": float(heights.min()), "max_hold_lift_m": float(heights.max()),
                                   "hand_relative_position_drift_m": float(relative_drift), "max_hold_speed_m_s": float(speeds.max()),
                                   "criteria": "During 1s hold: lift >3cm, relative translation drift <5mm, speed <0.15m/s"}
        report["physical_grasp_pass"] = bool(heights.min() > .03 and relative_drift < .005 and speeds.max() < .15)
        reason = "contact_grasp_failed" if not report["physical_grasp_pass"] else "held_lift_verified"
        if args.insert and report["physical_grasp_pass"]:
            current_battery = object_matrix(held)
            grasp_relation = np.linalg.inv(hand_matrix(held)) @ current_battery
            direction = current_battery[:3, 0]
            axis = np.cross(direction, [0., 0., 1.])
            angle = np.arccos(np.clip(direction[2], -1, 1))
            reorientation = Rotation.from_rotvec(axis/max(np.linalg.norm(axis), 1e-12)*angle).as_matrix()
            upright = current_battery.copy(); upright[:3, :3] = reorientation @ upright[:3, :3]
            phase("turn_upright", upright @ np.linalg.inv(grasp_relation), 2.5, closed_fingers)
            rotation = np.asarray(spec["frame"]["rotation_world_from_local"])
            origin = np.asarray(spec["frame"]["world_origin_m"])
            red = np.asarray(spec["props"]["red_position_xy_m"])
            hole_local = np.r_[red, spec["table"]["height_m"]+spec["props"]["red_dims_m"][2]]
            hole_local[0] += spec["props"]["hole_pitch_m"]*(1 if side == "right" else -1)
            hole = rotation @ hole_local+origin
            over_hole = upright.copy(); over_hole[:3, 3] = hole+[0., 0., .0245+.035]
            phase("above_hole", over_hole @ np.linalg.inv(grasp_relation), 3., closed_fingers)
            phase("align", over_hole @ np.linalg.inv(grasp_relation), 1., closed_fingers)
            seated = over_hole.copy(); seated[2, 3] = hole[2]-.013+.0245
            phase("insert", seated @ np.linalg.inv(grasp_relation), 4., closed_fingers)
            release_pose = hand_matrix(observation())
            phase("release", release_pose, 1., open_fingers)
            retreat = release_pose.copy(); retreat[2, 3] += .12
            phase("retreat", retreat, 2., open_fingers)
            final = phase("released_hold", retreat, 1., open_fingers)
            bp = object_matrix(final); direction = bp[:3, 0]
            tip = bp[:3, 3]-direction*.0245 if direction[2] >= 0 else bp[:3, 3]+direction*.0245
            radial = float(np.linalg.norm(tip[:2]-hole[:2])); depth = float(hole[2]-tip[2])
            velocity = final["object_velocities"][object_index]
            report["insertion_metrics"] = {"hole_center_world": hole.tolist(), "battery_tip_world": tip.tolist(),
                "radial_tip_error_m": radial, "depth_m": depth, "axis_vertical_cosine": float(abs(direction[2])),
                "speed_m_s": float(np.linalg.norm(velocity[:3])), "spin_rad_s": float(np.linalg.norm(velocity[3:])),
                "criteria": "After hand retreat: depth 11-14mm, radial tip error <0.5mm, vertical axis within 3deg, speed <1cm/s and spin <0.2rad/s"}
            report["physical_insertion_pass"] = bool(.011 <= depth <= .014 and radial < .0005 and abs(direction[2]) > np.cos(np.deg2rad(3))
                and np.linalg.norm(velocity[:3]) < .01 and np.linalg.norm(velocity[3:]) < .2)
            reason = "rigid_model_insertion_observed" if report["physical_insertion_pass"] else "insertion_not_verified"
    except TimeoutError as error:
        reason = str(error)
        report["status"] = "budget_exhausted"
    except Exception as error:
        reason = f"{type(error).__name__}: {error}"
        report["status"] = "error"
        raise
    finally:
        if trace:
            np.savez_compressed(args.output / "physics_trace.npz", **{k: np.array([row[k] for row in trace]) for k in trace[0]})
        report.update(status="complete" if report["status"] == "running" else report["status"], reason=reason,
                      physics_steps_recorded=len(trace), contact_pairs=runtime.contact_pairs)
        (args.output / "trial.json").write_text(json.dumps(report, indent=2)+"\n")
        print("BATTERY_TRIAL_COMPLETE", json.dumps({k: report[k] for k in ["status", "reason", "physical_grasp_pass", "physical_insertion_pass"]}), flush=True)
    runtime.close()
    exit_code = 0 if report["physical_grasp_pass"] else 2
except Exception:
    import traceback
    traceback.print_exc()
    exit_code = 1
finally:
    app.close(exit_code=exit_code)
