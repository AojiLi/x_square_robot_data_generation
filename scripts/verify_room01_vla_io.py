#!/usr/bin/env python3
"""Exercise the public policy interface and real three-camera/state alignment."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True
from room01_sim.bootstrap import launch
launcher = launch(args)
app = launcher.app
exit_code = 0
try:
    import numpy as np
    from room01_sim.vla_env import Room01VLAEnv
    from room01_sim.runtime import tensor
    from room01_sim.kinematics import quaternion_matrix
    from room01_sim.calibration import project_ros
    env = Room01VLAEnv(app, device=args.device)
    observation, info = env.reset(seed=17)
    pose1 = tensor(env.task.cube.data.root_link_pose_w)[0].cpu().numpy().copy()
    checks = {"observation_space": bool(env.observation_space.contains(observation)),
              "camera_count": len(observation["images"]) == 3,
              "no_simulator_object_pose_in_policy_input": "cube_pose_xyzw" not in observation,
              "same_timestamp": all(abs(value["timestamp"]-float(observation["timestamp"])) < 1e-9 for value in info["camera_metadata"].values())}
    previous_time = float(observation["timestamp"])
    action = observation["state"].copy()
    action[5] += .01
    new_observation, reward, terminated, truncated, step_info = env.step(action)
    checks["step_timing"] = abs(float(new_observation["timestamp"])-previous_time-1/30) < 1e-8
    checks["finite_action_response"] = np.isfinite(new_observation["state"]).all().item() and np.isfinite(reward).item()
    checks["observation_space_after_step"] = bool(env.observation_space.contains(new_observation))
    before_invalid = env.task.time
    invalid = action.copy();invalid[0] = np.nan
    try:
        env.step(invalid)
        checks["reject_invalid_action"] = False
    except ValueError:
        checks["reject_invalid_action"] = env.task.time == before_invalid
    observation2, _ = env.reset(seed=17)
    pose2 = tensor(env.task.cube.data.root_link_pose_w)[0].cpu().numpy().copy()
    checks["seeded_reset"] = bool(np.max(np.abs(pose1-pose2)) < 1e-5)
    # Fixed test pose keeps the target inside each relevant camera's field of view.
    observation3, info3 = env.reset(seed=0, options={"randomize": False})
    expected_state = np.array([env.task.config["joint_home_rad"][name] for name in env.task.config["action_joint_names"]])
    reset_joint_error = float(np.max(np.abs(observation3["state"]-expected_state)))
    checks["actual_reset_uses_configured_home"] = reset_joint_error < .03
    cube = tensor(env.task.cube.data.root_link_pose_w)[0, :3].cpu().numpy()
    projection = {}
    wrist_reset_views = {}
    for name in ["head", "left_wrist", "right_wrist"]:
        metadata = info3["camera_metadata"][name]
        spec = env.task.config["cameras"][name]
        point = quaternion_matrix(metadata["orientation_ros_xyzw"]).T@(cube-np.asarray(metadata["position_world"]))
        pixel = project_ros(point, spec)
        rgb = observation3["images"][name].astype(float)
        mask = (rgb[..., 0] > 65)&(rgb[..., 0] > 1.6*rgb[..., 1])&(rgb[..., 0] > 1.6*rgb[..., 2])
        ys, xs = np.nonzero(mask)
        error = float(np.min(np.hypot(xs-pixel[0], ys-pixel[1]))) if len(xs) else 1e6
        projection[name] = {"projected_target_pixel": pixel.tolist(), "distance_to_red_target_pixel": error}
        checks["camera_projection_"+name] = error < 12.
        if name.endswith("_wrist"):
            rotation = quaternion_matrix(metadata["orientation_ros_xyzw"])
            origin = np.asarray(metadata["position_world"])
            table_pixels = np.array([project_ros(rotation.T@(np.array([1.15, origin[1]+dy, env.task.config["tabletop_z"]])-origin), spec)
                                     for dy in [-.15, .15]])
            line = table_pixels[1]-table_pixels[0]
            tilt = float(np.degrees(np.arctan2(abs(line[1]), abs(line[0]))))
            checks["actual_reset_table_horizontal_"+name] = tilt < 5.
            wrist_reset_views[name] = {"table_line_tilt_deg": tilt, "table_line_pixels": table_pixels.tolist(),
                                      "position_world": metadata["position_world"], "orientation_ros_xyzw": metadata["orientation_ros_xyzw"]}
    result = {"result": "pass" if all(checks.values()) else "fail", "checks": checks,
              "action_dimension": 26, "fps": 30, "physics_hz": round(1/env.task.dt),
              "image_shapes": {name: list(image.shape) for name, image in observation3["images"].items()},
              "projection_checks": projection, "reset_position_difference_m": float(np.linalg.norm(pose1[:3]-pose2[:3])),
              "reset_joint_max_error_rad": reset_joint_error, "wrist_reset_views": wrist_reset_views,
              "config_sha256": hashlib.sha256((ROOT / "assets/room01/sim/task_config.json").read_bytes()).hexdigest(),
              "calibration_status": env.task.config["camera_calibration_status"]}
    (ROOT / "reports/room01_sim/vla_io_verification.json").write_text(json.dumps(result, indent=2))
    print("VLA_IO_VERIFIED", json.dumps(result), flush=True)
    env.close()
    exit_code = 0 if result["result"] == "pass" else 1
except Exception as error:
    import traceback
    traceback.print_exc()
    exit_code = 1
    (ROOT / "reports/room01_sim/vla_io_verification.json").write_text(json.dumps({"result": "fail", "error": str(error)}, indent=2))
finally:
    app.close(exit_code=exit_code)
