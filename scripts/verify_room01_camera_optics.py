#!/usr/bin/env python3
"""Compare optics with OpenCV and inspect re-opened USD calibration values."""
from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import cv2
import numpy as np
from pxr import Usd
from room01_sim.calibration import project_ros, validate_optics

asset = ROOT / "assets/room01/sim"
cfg = json.loads((asset / "task_config.json").read_text())
records = json.loads((asset / "camera_manifest.json").read_text())
stage = Usd.Stage.Open(str(asset / "room01_manipulation.usda"))
rng = np.random.default_rng(6021)
points = rng.normal(size=(1000, 3))
points[:, 2] = rng.uniform(.1, 3, len(points))
checks, detail = {}, {}
for name, spec in cfg["cameras"].items():
    validate_optics(spec)
    K = np.array([[spec["fx"], 0, spec["cx"]], [0, spec["fy"], spec["cy"]], [0, 0, 1.]])
    fish = spec["model"] in {"fisheye_equidistant", "opencv_fisheye"}
    coefficients = np.array(spec.get("distortion_coefficients", [0.] * (4 if fish else 8)))
    function = cv2.fisheye.projectPoints if fish else cv2.projectPoints
    expected, _ = function(points[:, None, :], np.zeros(3), np.zeros(3), K, coefficients)
    error = float(np.max(np.abs(expected[:, 0, :]-project_ros(points, spec))))
    checks[name+"_opencv_reference"] = error < 1e-9
    prim = stage.GetPrimAtPath(records[name]["prim_path"])
    model = "opencvFisheye" if fish else "opencvPinhole"
    prefix = "omni:lensdistortion:"+model+":"
    checks[name+"_usd_resolution"] = list(prim.GetAttribute(prefix+"imageSize").Get()) == spec["resolution"]
    checks[name+"_usd_model"] = prim.GetAttribute("omni:lensdistortion:model").Get() == model
    keys = ["k1", "k2", "k3", "k4"] if fish else ["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"]
    stored_K = [prim.GetAttribute(prefix+k).Get() for k in ["fx", "fy", "cx", "cy"]]
    stored_D = [prim.GetAttribute(prefix+k).Get() for k in keys]
    checks[name+"_usd_intrinsics"] = bool(np.allclose(stored_K, [spec[k] for k in ["fx", "fy", "cx", "cy"]], rtol=1e-7, atol=1e-6))
    checks[name+"_usd_distortion"] = bool(np.allclose(stored_D, coefficients, rtol=1e-7, atol=1e-9))
    # Plain OpenUSD has no Kit lens plugin registry. Read composed authored
    # schema metadata here; actual RTX behavior is checked by the VLA render.
    optical_apis = [s for s in prim.GetMetadata("apiSchemas").GetAppliedItems() if "LensDistortionOpenCv" in s]
    checks[name+"_one_lens_schema"] = len(optical_apis) == 1
    checks[name+"_provenance"] = json.loads(prim.GetCustomDataByKey("calibrationProvenanceJson")) == spec.get("calibration", {})
    detail[name] = {"max_projection_error_px": error, "image_size_wh": spec["resolution"], "applied_schemas": optical_apis}
checks["layer_calibration_status"] = stage.GetRootLayer().customLayerData["cameraCalibration"] == cfg["camera_calibration_status"]
report = {"result": "pass" if all(checks.values()) else "fail", "checks": checks, "cameras": detail,
          "opencv_version": cv2.__version__, "tested_points_per_camera": len(points),
          "scope": "Numerical implementation and USD persistence; does not validate physical sensor identity, source lens model or real hand-eye calibration."}
path = ROOT / "reports/room01_camera_calibration/projection_reference_verification.json"
path.parent.mkdir(parents=True, exist_ok=True)
path.write_text(json.dumps(report, indent=2)+"\n")
print(json.dumps(report, indent=2))
sys.exit(0 if report["result"] == "pass" else 1)
