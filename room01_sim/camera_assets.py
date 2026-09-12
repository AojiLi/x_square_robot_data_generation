"""Author camera mounts, optics and per-camera calibration provenance."""
import json
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, UsdGeom

from room01_sim.kinematics import quaternion_matrix
from room01_sim.calibration import validate_optics

ROOT = Path(__file__).resolve().parents[1]


def camera_path(name, spec, frames):
    return frames[spec["parent_link"]]["path"].replace("/Robot/", "/World/Robot/", 1)+"/RGB_"+name


def author_cameras(stage, config):
    frames = json.loads((ROOT / "assets/room01/sim/robot_frames.json").read_text())
    records = {}
    for name, spec in config["cameras"].items():
        validate_optics(spec)
        path = camera_path(name, spec, frames)
        camera = UsdGeom.Camera.Define(stage, path)
        width, height = spec["resolution"]
        focal = 20.
        camera.CreateProjectionAttr("perspective")
        camera.CreateFocalLengthAttr(focal)
        camera.CreateHorizontalApertureAttr(focal*width/spec["fx"])
        camera.CreateVerticalApertureAttr(focal*height/spec["fy"])
        camera.CreateHorizontalApertureOffsetAttr((width/2-spec["cx"])*focal/spec["fx"])
        camera.CreateVerticalApertureOffsetAttr((spec["cy"]-height/2)*focal/spec["fy"])
        camera.CreateClippingRangeAttr(Gf.Vec2f(spec["near"], spec["far"]))
        camera.CreateFStopAttr(0.)
        local = np.eye(4)
        local[:3, :3] = quaternion_matrix(spec["local_rotation_xyzw"])
        local[:3, 3] = spec["local_position"]
        if spec["convention"] == "ros":
            local[:3, :3] = local[:3, :3]@np.diag([1., -1., -1.])
        elif spec["convention"] == "world":
            local[:3, :3] = local[:3, :3]@np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
        elif spec["convention"] != "opengl":
            raise ValueError("Unknown camera frame convention")
        camera.ClearXformOpOrder()
        camera.AddTransformOp().Set(Gf.Matrix4d(local.T.tolist()))
        prim = camera.GetPrim()
        # Re-authoring a pinhole as fisheye must remove the old model API too.
        for schema in ["OmniLensDistortionOpenCvPinholeAPI", "OmniLensDistortionOpenCvFisheyeAPI"]:
            prim.RemoveAppliedSchema(schema)
        for attribute in prim.GetAttributes():
            if attribute.GetName().startswith("omni:lensdistortion:"):
                prim.RemoveProperty(attribute.GetName())
        fish = spec["model"] in {"fisheye_equidistant", "opencv_fisheye"}
        model = "opencvFisheye" if fish else "opencvPinhole"
        prim.AddAppliedSchema("OmniLensDistortionOpenCvFisheyeAPI" if fish else "OmniLensDistortionOpenCvPinholeAPI")
        prim.CreateAttribute("omni:lensdistortion:model", Sdf.ValueTypeNames.Token).Set(model)
        prefix = "omni:lensdistortion:"+model+":"
        prim.CreateAttribute(prefix+"imageSize", Sdf.ValueTypeNames.Int2).Set(Gf.Vec2i(width, height))
        for key in ["fx", "fy", "cx", "cy"]:
            prim.CreateAttribute(prefix+key, Sdf.ValueTypeNames.Float).Set(spec[key])
        coefficient_names = ["k1", "k2", "k3", "k4"] if fish else ["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"]
        coefficients = spec.get("distortion_coefficients", [0.]*len(coefficient_names))
        if len(coefficients) != len(coefficient_names):
            raise ValueError("Wrong number of camera distortion coefficients for "+model)
        for key, value in zip(coefficient_names, coefficients):
            prim.CreateAttribute(prefix+key, Sdf.ValueTypeNames.Float).Set(float(value))
        prim.SetCustomData({"role": name+" RGB observation", "resolution": Gf.Vec2i(width, height),
                            "calibrationStatus": spec.get("calibration_status", config["camera_calibration_status"]),
                            "calibrationProvenanceJson": json.dumps(spec.get("calibration", {}), sort_keys=True),
                            "sourceMount": spec["parent_link"]})
        records[name] = {"prim_path": path, **spec, "parent_to_camera_opengl": local.tolist()}
    (ROOT / "reports/room01_sim/camera_asset_manifest.json").write_text(json.dumps(records, indent=2))
    (ROOT / "assets/room01/sim/camera_manifest.json").write_text(json.dumps(records, indent=2))
    layer = stage.GetRootLayer()
    metadata = dict(layer.customLayerData)
    metadata["cameraCalibration"] = config["camera_calibration_status"]
    layer.customLayerData = metadata
    return records
