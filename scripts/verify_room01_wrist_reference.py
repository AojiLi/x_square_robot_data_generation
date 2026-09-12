#!/usr/bin/env python3
"""Check the actual saved reset against the user-selected wrist reference.

The reference pose is an image-guided estimate, not measured hand-eye calibration.
"""
from pathlib import Path
import argparse
import hashlib
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from pxr import Usd, UsdGeom
from room01_sim.kinematics import RobotKinematics, quaternion_matrix
from room01_sim.calibration import project_ros

parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, default=ROOT / "reports/room01_wrist_view_correction/orientation_verification.json")
args = parser.parse_args()
config = json.loads((ROOT / "assets/room01/sim/task_config.json").read_text())
pose = json.loads((ROOT / "assets/room01/sim/calibration/wrist_reference_pose_estimate.json").read_text())
kinematics = RobotKinematics()
base = np.eye(4); base[:3, 3] = config["robot_base_position"]
yaw = config["robot_base_yaw_rad"]
base[:3, :3] = [[np.cos(yaw), -np.sin(yaw), 0.], [np.sin(yaw), np.cos(yaw), 0.], [0., 0., 1.]]
frames = kinematics.forward(config["joint_home_rad"], base)
reference = kinematics.expand_mimics(pose["joint_positions_rad"])
reset = kinematics.expand_mimics(config["joint_home_rad"])
reset_error = max(abs(reset[name]-value) for name, value in reference.items())
stage = Usd.Stage.Open(str(ROOT / "assets/room01/sim/room01_manipulation.usda"))
frame_map = json.loads((ROOT / "assets/room01/sim/robot_frames.json").read_text())
cache = UsdGeom.XformCache()
saved_errors = {
    name: float(np.max(np.abs(np.asarray(cache.GetLocalToWorldTransform(
        stage.GetPrimAtPath(record["path"].replace("/Robot", "/World/Robot", 1)))).T-frames[name])))
    for name, record in frame_map.items()
}
checks = {
    "actual_reset_matches_reference_pose": reset_error < 1e-6,
    "default_config_matches_active_config": config == json.loads((ROOT / "room01_sim/default_task_config.json").read_text()),
    "saved_usd_pose_matches_actual_reset": max(saved_errors.values()) < 1e-5,
}
detail = {}
for side in ["left", "right"]:
    name = side+"_wrist"
    spec = config["cameras"][name]
    local = np.eye(4)
    local[:3, :3] = quaternion_matrix(spec["local_rotation_xyzw"])
    local[:3, 3] = spec["local_position"]
    camera = frames[spec["parent_link"]]@local
    def project(point):
        optical = np.diag([1., -1., -1.])@camera[:3, :3].T@(point-camera[:3, 3])
        return project_ros(optical, spec)
    palm = project((frames[side+"_hand_base_link"]@np.array([0., 0., .05, 1.]))[:3])
    tips = {finger: project(frames[side+"_"+finger+"_tip_link"][:3, 3])
            for finger in ["thumb", "index", "middle", "ring", "pinky"]}
    finger_y = np.mean([p[1] for n, p in tips.items() if n != "thumb"])
    checks[name+"_palm_side"] = bool(palm[0] < spec["cx"]-60 if side == "left" else palm[0] > spec["cx"]+60)
    checks[name+"_thumb_above_fingers"] = bool(tips["thumb"][1] < finger_y-30)
    checks[name+"_user_reference_pinned"] = spec["calibration"]["active_reference_profile"] == "user_selected_20260824_request_000001"
    checks[name+"_correct_wrist_mount"] = spec["parent_link"] == side+"_in_hand_camera_link"
    table_pixels = np.array([project(np.array([1.15, camera[1, 3]+dy, config["tabletop_z"]])) for dy in [-.15, .15]])
    line = table_pixels[1]-table_pixels[0]
    table_tilt = float(np.degrees(np.arctan2(abs(line[1]), abs(line[0]))))
    checks[name+"_table_horizontal_at_actual_reset"] = table_tilt < 5.
    detail[name] = {"palm_reference_pixel": palm.tolist(), "tip_reference_pixels": {k: v.tolist() for k, v in tips.items()},
                    "local_rotation_xyzw": spec["local_rotation_xyzw"], "native_camera_roll_correction_deg": 180,
                    "native_downward_pitch_estimate_deg": 17.5, "table_line_pixels": table_pixels.tolist(),
                    "table_line_tilt_deg": table_tilt}
report = {"result": "pass" if all(checks.values()) else "fail", "checks": checks, "cameras": detail,
          "reference": config["real_camera_contract"]["wrist_view_reference"],
          "scope": "Actual configured reset, saved USD link poses and nominal wrist-view direction; runtime reset and native renders checked separately. Not measured full-pose calibration.",
          "reference_pose_max_joint_difference_rad": reset_error,
          "saved_usd_transform_max_abs_error": max(saved_errors.values()),
          "config_sha256": hashlib.sha256((ROOT / "assets/room01/sim/task_config.json").read_bytes()).hexdigest(),
          "pose_source": "task_config.json joint_home_rad", "image_post_rotation": False}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(report, indent=2)+"\n")
print(json.dumps(report, indent=2))
sys.exit(0 if report["result"] == "pass" else 1)
