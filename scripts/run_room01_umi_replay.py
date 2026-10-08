#!/usr/bin/env python3
"""Run a contact-only, bounded UMI replay and retain achieved trajectories."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--prepared", type=Path, default=ROOT / "outputs/room01/umi_prepared")
parser.add_argument("--output", type=Path, default=ROOT / "outputs/room01/umi_replays")
parser.add_argument("--time-scale", type=float, default=1.)
parser.add_argument("--record", action="store_true")
parser.add_argument("--seconds", type=float, help="Diagnostic early stop in simulation seconds")
parser.add_argument("--review", action="store_true", help="Also save an external review camera")
parser.add_argument("--servo-integral", action="store_true", help="Compensate measured arm servo bias while keeping the same Cartesian reference")
parser.add_argument("--keep-open", action="store_true", help="Keep the GUI and physics running after the recorded replay")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if not 1 <= args.time_scale <= 2:
    parser.error("The agreed replay time scale must be between 1 and 2")
if args.seconds is not None and args.seconds <= 0:
    parser.error("Diagnostic duration must be positive")
args.enable_cameras = args.record or args.review
from room01_sim.bootstrap import launch
launcher = launch(args)
app = launcher.app
exit_code = 0
args.output.mkdir(parents=True, exist_ok=True)
out = args.output / datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
out.mkdir(exist_ok=False)

try:
    import numpy as np
    from PIL import Image
    from scipy.spatial.transform import Rotation
    from room01_sim.runtime import RoomTaskSim, tensor
    from room01_sim.kinematics import ACTION_JOINTS
    from room01_sim.umi import inverse, sample_poses, source_fingers_from_q

    prepared = np.load(args.prepared / "trajectory.npz")
    asset_dir = ROOT / "assets/room01/umi_replay"
    runtime = RoomTaskSim(app, device=args.device, cameras=args.enable_cameras,
                          review_camera=args.review, diagnostic_contacts=True,
                          config_path=asset_dir / "task_config.json", scene_path=asset_dir / "room01_umi_replay.usda")
    if runtime.config["control_hz"] != 30:
        raise ValueError("This pilot requires 30 Hz control and records every third observation at 10 Hz")
    report = {"status": "running", "source_episode_index": 0, "time_scale": args.time_scale,
              "arm_servo_bias_compensation": args.servo_integral,
              "geometry_correction_m": 0., "rotation_correction_deg": 0.,
              "configuration": runtime.config,
              "registration": json.loads((asset_dir / "registration.json").read_text()),
              "image_fps": 10, "control_hz": runtime.config["control_hz"], "physics_dt": runtime.dt,
              "object_names": list(runtime.objects), "action_joint_names": list(ACTION_JOINTS),
              "label_semantics": "Simulated achieved tool poses and finger angles; original UMI actions are references only",
              "success_definition": "Both target cubes wholly inside the moving box, all hand bases >18cm away, linear speed <2cm/s, angular speed <0.2rad/s for >=1s; box up-axis dot world-up >0.95",
              "training_eligible": False}
    if "correction_audit_json" in prepared:
        correction_audit = json.loads(str(prepared["correction_audit_json"]))
        report["corrections"] = correction_audit
        report["geometry_correction_m"] = correction_audit["max_position_correction_m"]
        report["rotation_correction_deg"] = correction_audit["max_rotation_correction_deg"]
    (out / "replay.json").write_text(json.dumps(report, indent=2))
    rows, controls, checks = [], [], []
    for name in runtime.cameras:
        (out / "images" / name).mkdir(parents=True)
    if args.review:
        (out / "review").mkdir()

    def hand_matrices(observation):
        matrices = np.broadcast_to(np.eye(4), (2, 4, 4)).copy()
        for side, name in enumerate(("left", "right")):
            pose = observation[name+"_hand_pose_xyzw"]
            matrices[side, :3, 3] = pose[:3]
            matrices[side, :3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
        return matrices

    def check_success(observation):
        names = observation["object_names"]
        poses = observation["object_poses_xyzw"]
        velocities = observation["object_velocities"]
        b = names.index("BlackBox")
        box_rotation = Rotation.from_quat(poses[b, 3:]).as_matrix()
        box = runtime.config["task_objects"]["BlackBox"]
        inner_half = np.asarray(box["size"][:2])/2-box["wall_m"]
        results = {"box_upright": bool(box_rotation[2, 2] > .95)}
        hands = hand_matrices(observation)[:, :3, 3]
        for name in runtime.config["target_objects"]:
            index = names.index(name)
            size = np.asarray(runtime.config["task_objects"][name]["size"])
            corners = np.array([[a, b, c] for a in [-.5, .5] for b in [-.5, .5] for c in [-.5, .5]])*size
            corners_world = (Rotation.from_quat(poses[index, 3:]).as_matrix() @ corners.T).T+poses[index, :3]
            local = (box_rotation.T @ (corners_world-poses[b, :3]).T).T
            inside = bool((np.abs(local[:, :2]) <= inner_half+.0005).all()
                          and (local[:, 2] >= -box["size"][2]/2+box["bottom_m"]-.0005).all()
                          and (local[:, 2] <= box["size"][2]/2+.0005).all())
            released = bool(np.linalg.norm(hands-poses[index, :3], axis=1).min() > .18)
            stable = bool(np.linalg.norm(velocities[index, :3]) < .02 and np.linalg.norm(velocities[index, 3:]) < .2)
            results[name] = {"inside": inside, "released": released, "stable": stable,
                             "position": poses[index, :3].tolist()}
        results["success_now"] = results["box_upright"] and all(all(results[name][key] for key in ("inside", "released", "stable")) for name in runtime.config["target_objects"])
        return results

    # Real preceding observation for the first body-frame delta label.
    previous_observation = runtime.observe(images=False)
    previous_tools = hand_matrices(previous_observation) @ inverse(prepared["tool_to_hand_base"])
    for _ in range(3):
        runtime.step(prepared["targets"][0], images=False)
    start = runtime.time
    observation = runtime.observe(images=args.record)
    duration = float(prepared["times"][-1])*args.time_scale+1.5
    if args.seconds is not None:
        duration = min(duration, args.seconds)
    held, longest = 0, 0

    def record(observation, time, evaluation):
        frame = len(rows)
        source_time = min(time/args.time_scale, float(prepared["times"][-1]))
        hands = hand_matrices(observation)
        target = sample_poses(prepared["times"], prepared["hand_targets"], np.array([source_time]))[0]
        row = {"timestamp": time, "source_time": source_time,
               "simulation_timestamp": observation["timestamp"], "joint_positions": observation["state"],
               "joint_velocities": observation["joint_velocity"], "applied_action": runtime.last_action.copy(),
               "hand_poses": hands, "tool_poses": hands @ inverse(prepared["tool_to_hand_base"]),
               "fingers_deg": source_fingers_from_q(observation["state"]),
               "object_poses_xyzw": observation["object_poses_xyzw"],
               "object_velocities": observation["object_velocities"],
               "tracking_position_errors": np.linalg.norm(hands[:, :3, 3]-target[:, :3, 3], axis=1),
               "tracking_rotation_errors": np.linalg.norm(Rotation.from_matrix(hands[:, :3, :3].swapaxes(-1, -2) @ target[:, :3, :3]).as_rotvec(), axis=1)}
        if args.record:
            for name, image in observation["images"].items():
                Image.fromarray(image).save(out / "images" / name / f"{frame:06d}.png")
            row["camera_positions"] = np.array([observation["camera_metadata"][name]["position_world"] for name in runtime.cameras])
            row["camera_orientations_opengl_xyzw"] = np.array([observation["camera_metadata"][name]["orientation_opengl_xyzw"] for name in runtime.cameras])
            row["camera_timestamps"] = np.array([observation["camera_metadata"][name]["timestamp"] for name in runtime.cameras])
        if args.review:
            runtime.sim.render()
            runtime.review_camera.update(0., force_recompute=True)
            image = tensor(runtime.review_camera.data.output["rgb"])[0].cpu().numpy()[..., :3]
            Image.fromarray(image).save(out / "review" / f"{frame:06d}.png")
        rows.append(row)
        checks.append({"time": time, **evaluation})

    record(observation, 0., check_success(observation))
    control_steps = round(duration*runtime.config["control_hz"])
    integral = np.zeros(26)
    arm_indices = np.r_[np.arange(7), np.arange(13, 20)]
    for step in range(control_steps):
        next_time = (step+1)/runtime.config["control_hz"]
        source_time = min(next_time/args.time_scale, float(prepared["times"][-1]))
        action = np.array([np.interp(source_time, prepared["times"], prepared["targets"][:, i]) for i in range(26)])
        reference_action = action.copy()
        if args.servo_integral:
            current_source_time = min(step/runtime.config["control_hz"]/args.time_scale, float(prepared["times"][-1]))
            desired_current = np.array([np.interp(current_source_time, prepared["times"], prepared["targets"][:, i]) for i in range(26)])
            integral[arm_indices] += .8*(desired_current[arm_indices]-observation["state"][arm_indices])/runtime.config["control_hz"]
            integral = np.clip(integral, -.06, .06)
            action += integral
        capture = (step+1) % 3 == 0
        observation, _, _, _, info = runtime.step(action, images=args.record and capture)
        controls.append({"time": next_time, "source_time": source_time, "reference_joint_target": reference_action,
                         "servo_bias": integral.copy(), "target": action,
                         "applied": info["applied_action"], "joint_positions": observation["state"],
                         "hand_poses": hand_matrices(observation)})
        evaluation = check_success(observation)
        held = held+1 if evaluation["success_now"] else 0
        longest = max(longest, held)
        if capture:
            record(observation, next_time, evaluation)
        if step % 30 == 29:
            print("UMI_STEP", json.dumps({"t": next_time, "checks": evaluation,
                                          "tracking_m": rows[-1]["tracking_position_errors"].tolist()}), flush=True)
    arrays = {key: np.stack([row[key] for row in rows]) for key in rows[0]}
    np.savez_compressed(out / "observations.npz", **arrays, previous_tool_poses=previous_tools)
    np.savez_compressed(out / "controls.npz", **{key: np.stack([row[key] for row in controls]) for key in controls[0]})
    success = longest/runtime.config["control_hz"] >= 1.
    report.update(status="complete", task_success=success, training_eligible=success and args.record,
                  longest_stable_success_s=longest/runtime.config["control_hz"], frames=len(rows),
                  source_duration_s=float(prepared["times"][-1]), duration_s=duration,
                  camera_names=list(runtime.cameras), images_recorded=args.record,
                  max_tracking_position_error_m=arrays["tracking_position_errors"].max(axis=0).tolist(),
                  max_tracking_rotation_error_deg=np.rad2deg(arrays["tracking_rotation_errors"]).max(axis=0).tolist(),
                  contact_reports_available=bool(runtime.contact_pairs),
                  checks=checks, contacts=runtime.contact_pairs)
    (out / "replay.json").write_text(json.dumps(report, indent=2))
    print("UMI_REPLAY_COMPLETE", json.dumps({"output": str(out), "success": success,
                                            "tracking_m": report["max_tracking_position_error_m"]}), flush=True)
    if args.keep_open and not args.headless:
        while app.is_running():
            runtime.advance_physics(runtime.substeps, render=True)
    runtime.close()
    exit_code = 0 if success else 2
except Exception as error:
    import traceback
    traceback.print_exc()
    (out / "failure.json").write_text(json.dumps({"status": "error", "error": str(error)}, indent=2))
    exit_code = 1
finally:
    app.close(exit_code=exit_code)
