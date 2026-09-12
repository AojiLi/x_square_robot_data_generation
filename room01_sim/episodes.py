"""Lossless, timestamped three-camera episodes with an explicit VLA contract."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import importlib.metadata
import json
import uuid

import numpy as np
from PIL import Image

from room01_sim.kinematics import ACTION_JOINTS


def policy_observation(observation):
    """Exclude simulator-only object poses from the VLA policy input."""
    return {key: observation[key] for key in ["images", "state", "joint_velocity", "language_instruction", "timestamp"] if key in observation}


class EpisodeWriter:
    def __init__(self, output_root, runtime, *, controller_name):
        name = "episode_"+datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")+"_"+uuid.uuid4().hex[:8]
        self.path = Path(output_root) / name
        self.path.mkdir(parents=True, exist_ok=False)
        self.camera_names = list(runtime.config["cameras"])
        for camera in self.camera_names:
            (self.path / "images" / camera).mkdir(parents=True)
        self.rows = []
        self.metadata = {
            "schema_version": "room01_vla_v1", "controller": controller_name,
            "instruction": runtime.config["language_instruction"], "joint_names": list(ACTION_JOINTS),
            "state_units": "radians", "action_units": "radians", "action_semantics": runtime.config["action_semantics"],
            "image_format": "PNG", "image_color_order": "RGB", "camera_names": self.camera_names,
            "fps": runtime.config["control_hz"], "physics_dt": runtime.dt,
            "alignment": "observation at t, applied action over [t,t+1/fps], next state at t+1/fps",
            "pose_quaternion_order": "xyzw", "config": runtime.config,
            "simulator_only_fields": ["cube_pose_xyzw", "next_cube_pose_xyzw"],
            "runtime": {name: importlib.metadata.version(name) for name in ["isaacsim", "isaaclab", "torch"]},
            "status": "recording", "success_label": None,
        }
        (self.path / "episode.json").write_text(json.dumps(self.metadata, indent=2))

    def append(self, observation, applied_action, next_observation, info):
        index = len(self.rows)
        if set(observation["images"]) != set(self.camera_names):
            raise ValueError("Every episode row must contain all three camera views")
        times = [observation["camera_metadata"][name]["timestamp"] for name in self.camera_names]
        if max(times)-min(times) > 1e-9 or abs(times[0]-observation["timestamp"]) > 1e-9:
            raise ValueError("Camera timestamps are not synchronized with the state")
        for name in self.camera_names:
            rgb = observation["images"][name]
            if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
                raise ValueError("Images must be RGB uint8 arrays")
            Image.fromarray(rgb).save(self.path / "images" / name / f"{index:06d}.png")
        self.rows.append({
            "timestamp": observation["timestamp"], "next_timestamp": next_observation["timestamp"],
            "state": np.asarray(observation["state"]), "joint_velocity": np.asarray(observation["joint_velocity"]),
            "action": np.asarray(applied_action), "next_state": np.asarray(next_observation["state"]),
            "cube_pose_xyzw": observation["cube_pose_xyzw"], "next_cube_pose_xyzw": next_observation["cube_pose_xyzw"],
            "camera_position_world": np.array([observation["camera_metadata"][name]["position_world"] for name in self.camera_names]),
            "camera_orientation_opengl_xyzw": np.array([observation["camera_metadata"][name]["orientation_opengl_xyzw"] for name in self.camera_names]),
            "camera_frame_counters": np.array([observation["camera_metadata"][name]["frame_counter"] for name in self.camera_names]),
            "lifted": info.get("object_lifted", False),
        })

    def close(self, *, success_label=None, label_method=None):
        if not self.rows:
            raise ValueError("Cannot finalize an empty episode")
        arrays = {key: np.stack([row[key] for row in self.rows]) for key in self.rows[0]}
        np.savez_compressed(self.path / "transitions.npz", **arrays)
        intervals = arrays["next_timestamp"]-arrays["timestamp"]
        if not np.allclose(intervals, 1/self.metadata["fps"], atol=1e-8):
            raise ValueError("Invalid state/action timing")
        if not all(np.isfinite(array).all() for array in arrays.values()):
            raise ValueError("Episode contains nonfinite values")
        self.metadata.update(status="complete", frames=len(self.rows), success_label=success_label, success_label_method=label_method,
                             transitions_sha256=hashlib.sha256((self.path / "transitions.npz").read_bytes()).hexdigest())
        (self.path / "episode.json").write_text(json.dumps(self.metadata, indent=2))
        return self.path
