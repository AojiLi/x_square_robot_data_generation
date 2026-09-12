"""Synchronized three-view RGB collection at a fixed physics state."""
import json
import time
from pathlib import Path

import numpy as np
from isaaclab.sensors import Camera, CameraCfg

from room01_sim.runtime import tensor

ROOT = Path(__file__).resolve().parents[1]


def create_cameras(config, stage):
    records = json.loads((ROOT / "assets/room01/sim/camera_manifest.json").read_text())
    cameras = {}
    for name, spec in records.items():
        if not stage.GetPrimAtPath(spec["prim_path"]):
            raise ValueError("Missing saved camera "+spec["prim_path"])
        cameras[name] = Camera(CameraCfg(
            prim_path=spec["prim_path"], spawn=None, width=spec["resolution"][0], height=spec["resolution"][1],
            update_period=0., data_types=["rgb"], update_latest_camera_pose=True,
        ))
    return cameras


def capture_frames(runtime, warmup=3):
    before = tensor(runtime.robot.root_view.get_dof_positions()).clone()
    for _ in range(warmup):
        runtime.sim.render()
    images, metadata = {}, {}
    for name, camera in runtime.cameras.items():
        camera.update(0., force_recompute=True)
        rgb = tensor(camera.data.output["rgb"])[0].detach().cpu().numpy()
        if rgb.shape[-1] == 4:
            rgb = rgb[..., :3]
        if rgb.dtype != np.uint8:
            rgb = np.clip(rgb*(255. if np.nanmax(rgb) <= 1. else 1.), 0, 255).astype(np.uint8)
        images[name] = rgb.copy()
        metadata[name] = {
            "timestamp": runtime.time, "physics_step": runtime.elapsed_steps,
            "frame_counter": int(tensor(camera.frame)[0]),
            "position_world": tensor(camera.data.pos_w)[0].detach().cpu().tolist(),
            "orientation_opengl_xyzw": tensor(camera.data.quat_w_opengl)[0].detach().cpu().tolist(),
            "orientation_ros_xyzw": tensor(camera.data.quat_w_ros)[0].detach().cpu().tolist(),
            "optics": runtime.config["cameras"][name],
        }
    drift = float((tensor(runtime.robot.root_view.get_dof_positions())-before).abs().max())
    if drift > 1e-6:
        raise RuntimeError(f"Physics moved during synchronized image readout: {drift} rad")
    return {"images": images, "camera_metadata": metadata}
