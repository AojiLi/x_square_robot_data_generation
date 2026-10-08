#!/usr/bin/env python3
"""Run the workcell and inspect/save actual head and wrist camera images."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser()
parser.add_argument("--capture", action="store_true", help="Capture three cameras, then exit.")
parser.add_argument("--output", type=Path, default=ROOT / "reports/room01_sim/latest_preview")
parser.add_argument("--hold-steps", type=int, default=120)
parser.add_argument("--joint-pose", type=Path, help="Optional review-only joint pose JSON; does not change the saved task reset pose.")
parser.add_argument("--config", type=Path, help="Task configuration; defaults to the original cube workcell.")
parser.add_argument("--scene", type=Path, help="USD scene paired with --config.")
parser.add_argument("--view", default="/World/ReviewSetup", help="Viewport camera path.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.capture = args.capture or args.headless
args.enable_cameras = True
from room01_sim.bootstrap import launch
launcher = launch(args)
app = launcher.app
exit_code = 0
try:
    import numpy as np
    from PIL import Image, ImageDraw
    from room01_sim.runtime import RoomTaskSim, tensor
    runtime = RoomTaskSim(app, device=args.device, cameras=args.capture, review_camera=args.capture,
                          config_path=args.config, scene_path=args.scene)
    if args.joint_pose:
        import torch
        requested = json.loads(args.joint_pose.read_text())["joint_positions_rad"]
        values = runtime.kinematics.expand_mimics(requested)
        q = tensor(runtime.robot.data.joint_pos).clone()
        for index, name in enumerate(runtime.joint_names):
            if name not in values:
                continue
            value = float(values[name])
            joint = runtime.kinematics.joints[name]
            if not np.isfinite(value) or not joint["lower"]-1e-6 <= value <= joint["upper"]+1e-6:
                raise ValueError("Review pose outside joint limits: "+name)
            q[0, index] = value
        runtime.robot.write_joint_state_to_sim_index(position=q, velocity=torch.zeros_like(q))
        runtime.robot.reset()
        runtime.targets.copy_(q)
    from omni.kit.viewport.utility import get_active_viewport
    viewport = get_active_viewport()
    if viewport:
        viewport.camera_path = args.view
    runtime.advance_physics(args.hold_steps)
    if args.capture:
        obs = runtime.observe(images=True)
        args.output.mkdir(parents=True, exist_ok=True)
        runtime.review_camera.update(0., force_recompute=True)
        review = tensor(runtime.review_camera.data.output["rgb"])[0].cpu().numpy()[..., :3]
        Image.fromarray(review.astype(np.uint8)).save(args.output / "ReviewSetup.png")
        frames = []
        for name, rgb in obs["images"].items():
            if rgb.ndim != 3 or float(rgb.std()) < 1.:
                raise RuntimeError(f"Missing or empty camera: {name}")
            Image.fromarray(rgb).save(args.output / (name+".png"))
            review_height = 480
            review_width = round(rgb.shape[1]*review_height/rgb.shape[0])
            display_image = Image.fromarray(rgb).resize((review_width, review_height), Image.Resampling.LANCZOS)
            panel = Image.new("RGB", (review_width, review_height+32), (25, 25, 25))
            panel.paste(display_image, (0, 32))
            ImageDraw.Draw(panel).text((12, 10), f"{name}  |  source {rgb.shape[1]} x {rgb.shape[0]}", fill="white")
            frames.append(panel)
        sheet = Image.new("RGB", (sum(im.width for im in frames), max(im.height for im in frames)))
        x = 0
        for panel in frames:
            sheet.paste(panel, (x, 0)); x += panel.width
        sheet.save(args.output / "three_views.jpg", quality=95)
        (args.output / "camera_metadata.json").write_text(json.dumps(obs["camera_metadata"], indent=2))
        (args.output / "reset_state.json").write_text(json.dumps({
            "pose_source": str(args.joint_pose) if args.joint_pose else "task_config.json joint_home_rad",
            "action_joint_names": runtime.config["action_joint_names"],
            "joint_positions_rad": obs["state"].tolist(),
            "left_hand_pose_xyzw": obs["left_hand_pose_xyzw"].tolist(),
            "right_hand_pose_xyzw": obs["right_hand_pose_xyzw"].tolist(),
            "timestamp": obs["timestamp"],
        }, indent=2)+"\n")
        report = {"result": "pass", "camera_count": len(frames), "same_physics_step": len({v["physics_step"] for v in obs["camera_metadata"].values()}) == 1,
                  "timestamp": obs["timestamp"], "joint_positions_finite": bool(np.isfinite(obs["state"]).all()),
                  "review_joint_pose": str(args.joint_pose) if args.joint_pose else None,
                  "config_sha256": hashlib.sha256((args.config or ROOT / "assets/room01/sim/task_config.json").read_bytes()).hexdigest(),
                  "calibration_status": runtime.config["camera_calibration_status"]}
        (args.output / "verification.json").write_text(json.dumps(report, indent=2))
        print("THREE_CAMERAS_READY", json.dumps(report), flush=True)
    else:
        for _ in range(16):
            runtime.advance_physics(runtime.substeps, render=True)
        print("ROOM01_VIEW_READY", str(args.scene or ROOT / "assets/room01/sim/room01_manipulation.usda"), flush=True)
    if not args.capture:
        while app.is_running():
            if runtime.sim.is_playing():
                runtime.advance_physics(runtime.substeps, render=True)
            else:
                app.update()
    runtime.close()
except Exception as error:
    exit_code = 1
    import traceback
    traceback.print_exc()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "verification.json").write_text(json.dumps({"result": "fail", "error": str(error)}, indent=2))
    print("CAMERA_PREVIEW_FAILED", str(error), flush=True)
finally:
    app.close(exit_code=exit_code)
