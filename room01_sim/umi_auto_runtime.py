"""Forward-physics trials for automatic scene initialization (import after Kit)."""
import json
import copy
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation

from room01_sim.umi import inverse, sample_poses, source_fingers_from_q
from room01_sim.umi_layout import CATALOG, cube_corners, trajectory_hash


def hand_matrices(observation):
    result = np.broadcast_to(np.eye(4), (2, 4, 4)).copy()
    for i, side in enumerate(("left", "right")):
        pose = observation[side+"_hand_pose_xyzw"]
        result[i, :3, 3] = pose[:3]
        result[i, :3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
    return result


def evaluate_task(observation, config):
    names = observation["object_names"]; poses = observation["object_poses_xyzw"]
    bi = names.index("BlackBox"); box = config["task_objects"]["BlackBox"]
    rotation = Rotation.from_quat(poses[bi, 3:]).as_matrix()
    inner = np.asarray(box["size"][:2])/2-box["wall_m"]
    hands = hand_matrices(observation)[:, :3, 3]
    result = {"box_upright": bool(rotation[2, 2] > .95)}
    for name in config["target_objects"]:
        i = names.index(name)
        corners = (cube_corners(poses[i], np.asarray(config["task_objects"][name]["size"]))-poses[bi, :3]) @ rotation
        inside = bool((abs(corners[:, :2]) <= inner+.0005).all()
                      and (corners[:, 2] >= -box["size"][2]/2+box["bottom_m"]-.0005).all()
                      and (corners[:, 2] <= box["size"][2]/2+.0005).all())
        velocity = observation["object_velocities"][i]
        result[name] = {"inside": inside, "released": bool(np.linalg.norm(hands-poses[i, :3], axis=1).min() > .18),
                        "stable": bool(np.linalg.norm(velocity[:3]) < .02 and np.linalg.norm(velocity[3:]) < .2),
                        "position": poses[i, :3].tolist()}
    result["success_now"] = result["box_upright"] and all(all(result[name][key] for key in ("inside", "released", "stable")) for name in config["target_objects"])
    return result


def run_trial(runtime, trajectory, metadata, poses, output, *, phase, time_scale=1., record=False,
              deadline=float("inf"), servo_integral=True, early_exit=True):
    from PIL import Image
    output = Path(output); output.mkdir(parents=True, exist_ok=False)
    expected_hash = metadata["trajectory_sha256"]
    if trajectory_hash(trajectory) != expected_hash:
        raise ValueError("Reference trajectory changed before a candidate trial")
    runtime.reset(randomize=False, action_pose=trajectory["targets"][0], object_poses=poses)
    previous = runtime.observe(images=False)
    previous_tools = hand_matrices(previous) @ inverse(trajectory["tool_to_hand_base"])
    for _ in range(3):
        runtime.step(trajectory["targets"][0], images=False)
    observation = runtime.observe(images=record)
    names = observation["object_names"]
    initial = observation["object_poses_xyzw"].copy()
    events = metadata["events"]
    rows, trace, controls, checks = [], [], [], []
    held, longest = 0, 0
    reason = "task_not_completed"; complete = True
    integral = np.zeros(26); arms = np.r_[np.arange(7), np.arange(13, 20)]
    duration = float(trajectory["times"][-1])*time_scale+1.5
    if phase == "grasp_probe":
        duration = min(duration, (max(e["release_s"] for e in events)+.4)*time_scale)
    trial_config = copy.deepcopy(runtime.config)
    for name, pose in poses.items():
        trial_config["task_objects"][name]["position"] = pose[:3]
        trial_config["task_objects"][name]["orientation_xyzw"] = pose[3:]
    report = {"status": "running", "phase": phase, "source_episode_index": metadata["source_episode_index"],
              "trajectory_sha256": expected_hash, "time_scale": time_scale, "arm_servo_bias_compensation": servo_integral,
              "reference_fingerprint": metadata.get("reference_fingerprint"),
              "geometry_correction_m": 0., "rotation_correction_deg": 0.,
              "configuration": trial_config, "initial_object_poses": poses,
              "registration": {"status": metadata["calibration"]["status"], "shared_calibration": metadata["calibration"],
                               "object_initialization": "automatically generated; no measured image poses"},
              "events": events, "images_recorded": record, "camera_names": list(runtime.cameras),
              "object_names": names, "image_fps": 10, "control_hz": 30, "physics_dt": runtime.dt,
              "task_success": False, "training_eligible": False,
              "success_definition": "Both target cubes wholly inside the dynamic box, hands >18cm away, stable for 1s; no object attachment"}
    (output / "replay.json").write_text(json.dumps(report, indent=2))
    if record:
        for name in runtime.cameras:
            (output / "images" / name).mkdir(parents=True)

    def save_observation(obs, timestamp):
        source_time = min(timestamp/time_scale, float(trajectory["times"][-1]))
        hand = hand_matrices(obs)
        target = sample_poses(trajectory["times"], trajectory["hand_targets"], np.array([source_time]))[0]
        row = {"timestamp": timestamp, "source_time": source_time, "simulation_timestamp": obs["timestamp"],
               "joint_positions": obs["state"], "joint_velocities": obs["joint_velocity"], "applied_action": runtime.last_action.copy(),
               "hand_poses": hand, "tool_poses": hand @ inverse(trajectory["tool_to_hand_base"]),
               "fingers_deg": source_fingers_from_q(obs["state"]), "object_poses_xyzw": obs["object_poses_xyzw"],
               "object_velocities": obs["object_velocities"],
               "tracking_position_errors": np.linalg.norm(hand[:, :3, 3]-target[:, :3, 3], axis=1),
               "tracking_rotation_errors": np.linalg.norm(Rotation.from_matrix(hand[:, :3, :3].swapaxes(-1, -2) @ target[:, :3, :3]).as_rotvec(), axis=1)}
        if record:
            for name in runtime.cameras:
                Image.fromarray(obs["images"][name]).save(output / "images" / name / f"{len(rows):06d}.png")
            row["camera_positions"] = np.array([obs["camera_metadata"][n]["position_world"] for n in runtime.cameras])
            row["camera_orientations_opengl_xyzw"] = np.array([obs["camera_metadata"][n]["orientation_opengl_xyzw"] for n in runtime.cameras])
            row["camera_timestamps"] = np.array([obs["camera_metadata"][n]["timestamp"] for n in runtime.cameras])
        rows.append(row); checks.append({"time": timestamp, **evaluate_task(obs, runtime.config)})

    def trace_row(obs, timestamp):
        return {"timestamp": timestamp, "source_time": min(timestamp/time_scale, float(trajectory["times"][-1])),
                "object_poses": obs["object_poses_xyzw"], "object_velocities": obs["object_velocities"],
                "joint_positions": obs["state"], "hand_poses": hand_matrices(obs)}

    save_observation(observation, 0.); trace.append(trace_row(observation, 0.))
    for step in range(round(duration*30)):
        if step % 30 == 0 and time.monotonic() >= deadline:
            complete = False; reason = "wall_time_budget_exhausted"; break
        timestamp = (step+1)/30
        source_time = min(timestamp/time_scale, float(trajectory["times"][-1]))
        reference = np.array([np.interp(source_time, trajectory["times"], trajectory["targets"][:, j]) for j in range(26)])
        if servo_integral:
            current_time = min(step/30/time_scale, float(trajectory["times"][-1]))
            current = np.array([np.interp(current_time, trajectory["times"], trajectory["targets"][:, j]) for j in range(26)])
            integral[arms] += .8*(current[arms]-observation["state"][arms])/30
            integral = np.clip(integral, -.06, .06)
        capture = (step+1) % 3 == 0
        observation, _, _, _, info = runtime.step(reference+integral, images=record and capture)
        controls.append({"time": timestamp, "source_time": source_time, "reference_joint_target": reference,
                         "servo_bias": integral.copy(), "target": reference+integral, "applied": info["applied_action"],
                         "joint_positions": observation["state"], "hand_poses": hand_matrices(observation)})
        trace.append(trace_row(observation, timestamp))
        check = evaluate_task(observation, runtime.config)
        held = held+1 if check["success_now"] else 0; longest = max(longest, held)
        if capture:
            save_observation(observation, timestamp)
        if phase == "grasp_probe" and early_exit:
            failed = []
            for event in events:
                if source_time > event["release_s"]+.15:
                    index = names.index(event["asset"])
                    lift = max(float(t["object_poses"][index, 2]-initial[index, 2]) for t in trace)
                    if lift < .015:
                        failed.append(event["asset"])
            if failed:
                reason = "grasp_failed:"+",".join(failed); break
        if step % 90 == 89:
            print("AUTO_STEP", phase, round(source_time, 2), flush=True)
    arrays = {key: np.stack([r[key] for r in rows]) for key in rows[0]}
    trace_arrays = {key: np.stack([r[key] for r in trace]) for key in trace[0]}
    np.savez_compressed(output / "observations.npz", **arrays, previous_tool_poses=previous_tools)
    np.savez_compressed(output / "physics_trace.npz", **trace_arrays)
    if controls:
        np.savez_compressed(output / "controls.npz", **{k: np.stack([r[k] for r in controls]) for k in controls[0]})
    grasps = {}
    for event in events:
        i = names.index(event["asset"]); side = event["side_index"]
        lift = trace_arrays["object_poses"][:, i, 2]-initial[i, 2]
        distance = np.linalg.norm(trace_arrays["hand_poses"][:, side, :3, 3]-trace_arrays["object_poses"][:, i, :3], axis=-1)
        active = (trace_arrays["source_time"] >= event["closed_s"]) & (trace_arrays["source_time"] < event["release_s"])
        count = int(np.sum(active & (lift > .015) & (distance < .18)))
        grasps[event["asset"]] = {"max_lift_m": float(lift.max()), "held_frames": count, "passed": count >= 6,
                                  "evaluated": bool(trace_arrays["source_time"][-1] >= event["release_s"]+.15)}
    success = complete and phase in ("full", "robustness", "record_full") and longest >= 30
    if success:
        reason = "task_success"
    elif phase in ("grasp_probe", "record_probe") and all(x["passed"] for x in grasps.values()):
        reason = "both_grasps_verified"
    if trajectory_hash(trajectory) != expected_hash:
        raise AssertionError("A trial mutated its fixed reference trajectory")
    report.update(status="complete" if complete else "budget_exhausted", reason=reason,
                  task_success=success, training_eligible=False, grasps=grasps, frames=len(rows),
                  duration_s=float(trace_arrays["timestamp"][-1]), longest_stable_success_s=longest/30,
                  max_tracking_position_error_m=arrays["tracking_position_errors"].max(0).tolist(),
                  max_tracking_rotation_error_deg=np.rad2deg(arrays["tracking_rotation_errors"]).max(0).tolist(),
                  checks=checks, reference_hash_verified=True)
    (output / "replay.json").write_text(json.dumps(report, indent=2))
    print("AUTO_TRIAL_COMPLETE", json.dumps({"path": str(output), "phase": phase, "reason": reason, "grasps": grasps}), flush=True)
    trace_arrays["object_names"] = names
    return report, trace_arrays
