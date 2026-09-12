#!/usr/bin/env python3
"""Author measured camera presets and an Isaac RTX preview profile in the USD."""
import json
import shutil
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdRender

ROOT = Path(__file__).resolve().parents[1]


def add_review_setup(stage, frames):
    # The phone images are portrait captures stored sideways. Rotating the camera
    # and applying a calibrated crop gives an upright, already-observed view.
    roll = np.array([[0., -1., 0., 0.], [1., 0., 0., 0.],
                     [0., 0., 1., 0.], [0., 0., 0., 1.]])
    presets = {}
    for name, index in [("ReviewRoom", 4), ("ReviewTable", 124),
                         ("ReviewChair", 254), ("ReviewFloor", 374)]:
        original = frames[index]
        width = original["h"]
        height = width*3//4
        crop_top = (original["w"]-height)//2
        fx, fy = original["fl_y"], original["fl_x"]
        cx = original["h"]-1-original["cy"]
        cy = original["cx"]-crop_top
        matrix = np.array(original["transform_matrix"]) @ roll
        path = "/World/"+name
        camera = UsdGeom.Camera.Define(stage, path)
        focal = 20.0
        camera.CreateProjectionAttr(UsdGeom.Tokens.perspective)
        camera.CreateFocalLengthAttr(focal)
        camera.CreateHorizontalApertureAttr(focal*width/fx)
        camera.CreateVerticalApertureAttr(focal*height/fy)
        camera.CreateHorizontalApertureOffsetAttr((width/2-cx)*focal/fx)
        camera.CreateVerticalApertureOffsetAttr((cy-height/2)*focal/fy)
        camera.CreateFStopAttr(0.)
        camera.CreateShutterOpenAttr(0.)
        camera.CreateShutterCloseAttr(0.)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.))
        xform = UsdGeom.Xformable(camera.GetPrim())
        operations = xform.GetOrderedXformOps()
        op = operations[0] if operations else xform.AddTransformOp()
        op.Set(Gf.Matrix4d(matrix.T.tolist()))
        camera.GetPrim().SetCustomData({"sourceFrameIndex": index, "sourceFrameId": Path(original["file_path"]).stem,
                                        "previewNote": "Upright 4:3 crop of the measured capture view"})
        # Verify that arbitrary camera-space points project to the rotated/cropped
        # source pixels, using USD's actual projection matrix.
        points = np.array([[0.1, 0.05, -1.], [-0.2, 0.1, -2.], [0.3, -0.2, -1.5]])
        old_u = original["fl_x"]*points[:, 0]/-points[:, 2]+original["cx"]
        old_v = original["cy"]-original["fl_y"]*points[:, 1]/-points[:, 2]
        expected = np.column_stack([original["h"]-1-old_v, old_u-crop_top])
        rotated = np.column_stack([points, np.ones(len(points))]) @ roll
        clip = rotated @ np.array(camera.GetCamera().frustum.ComputeProjectionMatrix())
        ndc = clip[:, :3]/clip[:, 3:4]
        actual = np.column_stack([(ndc[:, 0]+1)*width/2, (1-ndc[:, 1])*height/2])
        error = float(np.abs(actual-expected).max())
        if error > 0.001:
            raise ValueError(f"Preview camera projection failed: {name}: {error}px")
        presets[name] = {"path": path, "source_frame_index": index, "resolution": [width, height],
                         "clockwise_rotation_degrees": 90, "crop_top_after_rotation": crop_top,
                         "projection_error_pixels": error}
    data = dict(stage.GetRootLayer().customLayerData)
    settings = dict(data.get("renderSettings", {}))
    settings.update({
        "rtx:rendermode": "RealTimePathTracing",
        "rtx:post:aa:op": 4,  # NVIDIA DLAA, native resolution
        "rtx:post:dlss:execMode": 2,  # Quality if an app switches back to DLSS
        "rtx-transient:dlssg:enabled": False,
        "rtx:post:dof:enabled": False,
        "rtx:post:motionblur:enabled": False,
        "rtx:rtpt:gaussian:skipTonemapping:enabled": True,
    })
    data["renderSettings"] = settings
    data["cameraSettings"] = {**dict(data.get("cameraSettings", {})), "boundCamera": "/World/ReviewRoom"}
    data["previewProfile"] = "Isaac RTX Real-Time 2.0 / native DLAA / measured upright camera presets"
    stage.GetRootLayer().customLayerData = data
    render_settings = UsdRender.Settings.Get(stage, "/Render/Settings")
    if not render_settings:
        render_settings = UsdRender.Settings.Define(stage, "/Render/Settings")
    render_settings.CreateCameraRel().SetTargets([Sdf.Path("/World/ReviewRoom")])
    render_settings.CreateResolutionAttr(Gf.Vec2i(*presets["ReviewRoom"]["resolution"]))
    render_settings.CreatePixelAspectRatioAttr(1.0)
    render_settings.CreateAspectRatioConformPolicyAttr("cropAperture")
    stage.SetMetadata("renderSettingsPrimPath", "/Render/Settings")
    return {"presets": presets, "render_settings": settings, "default_camera": "/World/ReviewRoom"}


def main():
    path = ROOT / "assets/room01/room01_scene.usda"
    backup = ROOT / "reports/room01_blur_diagnosis/room01_scene_before_preview_fix.usda"
    backup.parent.mkdir(parents=True, exist_ok=True)
    if not backup.exists():
        shutil.copyfile(path, backup)
    frames = json.loads((ROOT / "data/room01/transforms.json").read_text())["frames"]
    stage = Usd.Stage.Open(str(path))
    report = add_review_setup(stage, frames)
    stage.GetRootLayer().Save()
    output = ROOT / "reports/room01_blur_diagnosis/preview_profile.json"
    output.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
