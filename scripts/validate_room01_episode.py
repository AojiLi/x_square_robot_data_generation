#!/usr/bin/env python3
"""Validate complete image sequences and state/action timing before VLA use."""
from pathlib import Path
import argparse
import json
import sys

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from room01_sim.kinematics import RobotKinematics

parser = argparse.ArgumentParser()
parser.add_argument("episode", type=Path)
args = parser.parse_args()
metadata = json.loads((args.episode / "episode.json").read_text())
data = np.load(args.episode / "transitions.npz")
count = metadata["frames"]
checks = {"complete": metadata["status"] == "complete", "three_cameras": len(metadata["camera_names"]) == 3,
          "state_action_shapes": data["state"].shape == data["action"].shape == (count, 26),
          "finite_values": all(np.isfinite(data[key]).all() for key in data.files),
          "action_interval": bool(np.allclose(data["next_timestamp"]-data["timestamp"], 1/metadata["fps"], atol=1e-8)),
          "observation_spacing": bool(np.allclose(np.diff(data["timestamp"]), 1/metadata["fps"], atol=1e-8)),
          "next_state_alignment": bool(np.array_equal(data["next_state"][:-1], data["state"][1:])),
          "camera_quaternions": bool(np.allclose(np.linalg.norm(data["camera_orientation_opengl_xyzw"], axis=-1), 1., atol=1e-4))}
k = RobotKinematics()
lower = np.array([k.joints[name]["lower"] for name in metadata["joint_names"]])
upper = np.array([k.joints[name]["upper"] for name in metadata["joint_names"]])
checks["action_limits"] = bool(((data["action"] >= lower-1e-6)&(data["action"] <= upper+1e-6)).all())
image_count = 0
for name in metadata["camera_names"]:
    expected = [args.episode / "images" / name / f"{i:06d}.png" for i in range(count)]
    checks[name+"_image_count"] = len(list((args.episode / "images" / name).glob("*.png"))) == count
    width, height = metadata["config"]["cameras"][name]["resolution"]
    for path in expected:
        with Image.open(path) as image:
            image.load()
            if image.mode != "RGB" or image.size != (width, height):
                raise ValueError(f"Invalid RGB image {path}")
        image_count += 1
head = metadata["camera_names"].index("head")
checks["fixed_head_camera"] = float(np.max(np.linalg.norm(data["camera_position_world"][:, head]-data["camera_position_world"][0, head], axis=-1))) < 1e-5
report = {"result": "pass" if all(checks.values()) else "fail", "episode": str(args.episode.resolve()), "frames": count,
          "decoded_images": image_count, "fps": metadata["fps"], "duration_seconds": float(count/metadata["fps"]),
          "success_label": metadata["success_label"], "checks": checks,
          "calibration_status": metadata["config"]["camera_calibration_status"]}
(args.episode / "validation.json").write_text(json.dumps(report, indent=2))
(ROOT / "reports/room01_sim/episode_verification.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
sys.exit(0 if report["result"] == "pass" else 1)
