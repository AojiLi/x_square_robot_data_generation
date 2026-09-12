#!/usr/bin/env python3
"""Reopen the configured file with default Isaac settings and capture its presets."""
import argparse
import json
import hashlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, default=ROOT / "reports/room01_blur_diagnosis/fixed_scene")
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)

from isaacsim import SimulationApp

# Deliberately start with the former default. The USD must restore its own profile.
app = SimulationApp({"headless": True, "multi_gpu": False, "width": 736, "height": 552,
                     "renderer": "RealTimePathTracing", "anti_aliasing": 3,
                     "enable_crashreporter": False,
                     "extra_args": ["--enable", "isaacsim.replicator.nurec_utils"]})
try:
    import carb
    import omni.usd
    import numpy as np
    from PIL import Image
    from pxr import UsdGeom
    from isaacsim.replicator.nurec_utils.rendering_setup import setup_for_rendering
    from isaacsim.replicator.nurec_utils.render import RenderTargetFactory, CameraRenderer
    from omni.kit.viewport.utility import get_active_viewport

    assert omni.usd.get_context().open_stage(str(ROOT / "assets/room01/room01_scene.usda"))
    stage = omni.usd.get_context().get_stage()
    ok, nurec, spg, problems = setup_for_rendering(stage)
    if not ok or not nurec or spg:
        raise RuntimeError(str((ok, nurec, spg, problems)))
    stage.SetEditTarget(stage.GetSessionLayer())
    settings = carb.settings.get_settings()
    keys = ["/rtx/rendermode", "/rtx/post/aa/op", "/rtx/post/dlss/execMode",
            "/rtx-transient/dlssg/enabled", "/rtx/post/dof/enabled",
            "/rtx/post/motionblur/enabled", "/rtx/rtpt/gaussian/skipTonemapping/enabled"]
    applied = {key: settings.get(key) for key in keys}
    if applied["/rtx/post/aa/op"] != 4:
        raise RuntimeError(f"USD did not restore native-resolution DLAA: {applied}")
    factory = RenderTargetFactory(False, resolution=(736, 552))
    records = []
    for name in ["ReviewRoom", "ReviewTable"]:
        camera_path = "/World/"+name
        target = factory.create(stage, name, camera_path=camera_path)
        renderer = CameraRenderer.open(stage, name, app, target, warmup_steps=1600, force_identity_exposure=False)
        matrix = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(camera_path))
        rotation = matrix.ExtractRotationQuat()
        pose = [*matrix.ExtractTranslation(), *rotation.GetImaginary(), rotation.GetReal()]
        rgb = renderer.render_at_pose(pose)
        if rgb is None or rgb.shape != (552, 736, 3) or float(rgb.std()) < 2:
            raise RuntimeError(f"Empty or invalid rendered scene: {name}")
        Image.fromarray(rgb).save(args.output / f"{name}.png")
        records.append({"camera": camera_path, "shape": list(rgb.shape), "mean": float(rgb.mean()), "std": float(rgb.std())})
        renderer.close()
    viewport = get_active_viewport()
    asset = ROOT / "assets/room01/gaussians.usdc"
    digest = hashlib.sha256()
    with asset.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4*1024*1024), b""):
            digest.update(chunk)
    result = {"result": "pass", "settings_loaded_from_usd": applied,
              "viewport_camera_after_open": str(viewport.camera_path) if viewport else None,
              "saved_default_camera": stage.GetMetadataByDictKey("customLayerData", "cameraSettings:boundCamera"),
              "renders": records, "runtime": "Isaac Sim 6.0.1",
              "gaussians": len(stage.GetPrimAtPath("/World/RoomGaussian").GetAttribute("positions").Get()),
              "gaussian_asset_sha256": digest.hexdigest()}
    (args.output / "verification.json").write_text(json.dumps(result, indent=2)+"\n")
    print("PREVIEW_VERIFIED "+json.dumps(result), flush=True)
finally:
    app.close()
