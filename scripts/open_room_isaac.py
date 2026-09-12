#!/usr/bin/env python3
"""Open room01 in an interactive Isaac Sim window; keep it open for review."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
from isaacsim import SimulationApp

app = SimulationApp({
    "headless": False,
    "multi_gpu": False,
    "width": 1280,
    "height": 960,
    "window_width": 1440,
    "window_height": 1000,
    "renderer": "RealTimePathTracing",
    "anti_aliasing": 4,
    "enable_crashreporter": False,
    "extra_args": ["--enable", "isaacsim.replicator.nurec_utils"],
})
try:
    import carb
    import omni.usd
    from omni.kit.viewport.utility import get_active_viewport
    from isaacsim.replicator.nurec_utils.rendering_setup import setup_for_rendering

    scene_path = ROOT / "assets/room01/room01_scene.usda"
    context = omni.usd.get_context()
    if not context.open_stage(str(scene_path)):
        raise RuntimeError(f"Could not open {scene_path}")
    stage = context.get_stage()
    ok, nurec, spg, problems = setup_for_rendering(stage)
    if not ok:
        raise RuntimeError(f"Rendering setup failed: {problems}")
    # Camera navigation and later UI edits stay in the session until explicitly saved.
    stage.SetEditTarget(stage.GetSessionLayer())
    viewport = get_active_viewport()
    if viewport is not None:
        viewport.camera_path = "/World/ReviewRoom"
    for _ in range(120):
        if not app.is_running():
            break
        app.update()
    settings = carb.settings.get_settings()
    print("ROOM01_UI_READY " + json.dumps({
        "scene": str(scene_path),
        "camera": str(viewport.camera_path) if viewport else None,
        "anti_aliasing": settings.get("/rtx/post/aa/op"),
        "render_mode": settings.get("/rtx/rendermode"),
        "gaussians": len(stage.GetPrimAtPath("/World/RoomGaussian").GetAttribute("positions").Get()),
        "headless": False,
    }), flush=True)
    while app.is_running():
        app.update()
finally:
    app.close()
