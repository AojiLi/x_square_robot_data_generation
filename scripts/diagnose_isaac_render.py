#!/usr/bin/env python3
"""Render the exported USD at the exact gsplat evaluation views in Isaac Sim."""
import argparse
import json
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--mode", choices=["default", "clean", "quality", "path"], default="default")
parser.add_argument("--frames", default="4,124")
parser.add_argument("--factor", type=int, default=2)
parser.add_argument("--warmup", type=int, default=800)
parser.add_argument("--spp", type=int, default=64)
parser.add_argument("--output", type=Path)
args = parser.parse_args()
output = args.output or ROOT / "reports/room01_blur_diagnosis" / f"isaac_{args.mode}_{args.factor}"
output.mkdir(parents=True, exist_ok=True)

from isaacsim import SimulationApp

app = SimulationApp({
    "headless": True, "width": 992//args.factor, "height": 736//args.factor,
    "multi_gpu": False, "renderer": "PathTracing" if args.mode == "path" else "RealTimePathTracing",
    "anti_aliasing": 4 if args.mode == "quality" else (3 if args.mode == "default" else 0),
    "denoiser": args.mode == "default", "samples_per_pixel_per_frame": args.spp,
    "sync_loads": True, "enable_crashreporter": False,
    "extra_args": ["--enable", "isaacsim.replicator.nurec_utils", "--/renderer/multiGpu/enabled=false"],
})

try:
    import carb
    import omni.usd
    import numpy as np
    from PIL import Image
    from pxr import Gf, Usd, UsdGeom
    from isaacsim.replicator.nurec_utils.rendering_setup import setup_for_rendering
    from isaacsim.replicator.nurec_utils.render import RenderTargetFactory, CameraRenderer

    print("ISAAC_DIAG application ready", flush=True)
    if not omni.usd.get_context().open_stage(str(ROOT / "assets/room01/room01_scene.usda")):
        raise RuntimeError("Could not open the exported scene")
    stage = omni.usd.get_context().get_stage()
    ok, nurec, spg, problems = setup_for_rendering(stage)
    if not ok or not nurec or spg:
        raise RuntimeError(f"Unexpected NuRec setup: {ok}, {nurec}, {spg}, {problems}")
    stage.SetEditTarget(stage.GetSessionLayer())
    camera = UsdGeom.Camera(stage.GetPrimAtPath("/World/ScanCamera"))
    camera.CreateFStopAttr(0.0)
    camera.CreateShutterOpenAttr(0.0)
    camera.CreateShutterCloseAttr(0.0)
    settings = carb.settings.get_settings()
    if args.mode == "quality":
        settings.set("/rtx/post/aa/op", 4)  # DLAA: native resolution
        settings.set("/rtx/post/dlss/execMode", 2)
        settings.set("/rtx-transient/dlssg/enabled", False)
    keys = ["/rtx/rendermode", "/rtx/post/aa/op", "/rtx/post/dlss/execMode",
            "/rtx-transient/dlssg/enabled",
            "/rtx/post/motionblur/enabled", "/rtx/post/dof/enabled",
            "/rtx/rtpt/gaussian/skipTonemapping/enabled",
            "/rtx/rtpt/gaussian/maxGaussiansToAccumulate",
            "/rtx/rtpt/gaussian/accumulatedDepth/enabled",
            "/rtx/rtpt/gaussian/accumulatedAlbedo/enabled",
            "/rtx/post/tonemap/op", "/rtx/pathtracing/optixDenoiser/enabled"]
    observed_settings = {key: settings.get(key) for key in keys}
    frames = json.loads((ROOT / "data/room01/transforms.json").read_text())["frames"]
    width, height = 992//args.factor, 736//args.factor
    target = RenderTargetFactory(False, resolution=(width, height)).create(stage, "diagnostic", camera_path=str(camera.GetPath()))
    renderer = CameraRenderer.open(stage, "diagnostic", app, target,
                                   warmup_steps=args.warmup, force_identity_exposure=False)
    records = []
    for index in [int(value) for value in args.frames.split(",")]:
        frame = frames[index]
        focal = 20.0
        camera.CreateFocalLengthAttr(focal)
        camera.CreateHorizontalApertureAttr(focal*frame["w"]/frame["fl_x"])
        camera.CreateVerticalApertureAttr(focal*frame["h"]/frame["fl_y"])
        camera.CreateHorizontalApertureOffsetAttr((frame["w"]/2-frame["cx"])*focal/frame["fl_x"])
        camera.CreateVerticalApertureOffsetAttr((frame["cy"]-frame["h"]/2)*focal/frame["fl_y"])
        matrix = np.array(frame["transform_matrix"])
        q = Gf.Matrix4d(matrix.T.tolist()).ExtractRotationQuat()
        pose = [*matrix[:3, 3], *q.GetImaginary(), q.GetReal()]
        started = time.monotonic()
        rgb = renderer.render_at_pose(pose)
        if rgb is None or rgb.shape != (height, width, 3):
            raise RuntimeError(f"No valid frame for view {index}: {None if rgb is None else rgb.shape}")
        name = Path(frame["file_path"]).stem
        Image.fromarray(rgb).save(output / f"{name}.png")
        record = {"frame_index": index, "frame_id": name, "shape": list(rgb.shape),
                  "mean": float(rgb.mean()), "std": float(rgb.std()), "render_seconds": time.monotonic()-started}
        records.append(record)
        print("ISAAC_DIAG "+json.dumps(record), flush=True)
    renderer.close()
    summary = {"mode": args.mode, "factor": args.factor, "spp": args.spp,
               "settings": observed_settings, "settings_after_render": {key: settings.get(key) for key in keys}, "frames": records,
               "stage": str(ROOT / "assets/room01/room01_scene.usda"), "scene_mutations_saved": False}
    summary["result"] = "pass" if all(row["std"] >= 2 for row in records) else "empty_render"
    (output / "manifest.json").write_text(json.dumps(summary, indent=2)+"\n")
    if summary["result"] != "pass":
        raise RuntimeError("Renderer returned empty scene images; this mode did not pass the scene check")
    print("ISAAC_DIAG complete", flush=True)
finally:
    app.close()
