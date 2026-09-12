#!/usr/bin/env python3
"""Fit lens boundaries using the archive's current physical-topic mapping."""
from pathlib import Path
import json

import cv2
import numpy as np
from audit_room01_camera_archive import fit_boundary

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports/room01_wrist_fisheye_alignment"
rows = [json.loads(line) for line in (REPORT / "archive_images.jsonl").read_text().splitlines()]
current = [r for r in rows
           if r["source_mapping"]["cam_left_wrist"] == "/camera3/usb_cam3/image_raw/image_compressed"
           and r["source_mapping"]["cam_right_wrist"] == "/camera1/usb_cam1/image_raw/image_compressed"]
result = {"profile": "current_physical_topic_mapping", "requests": len(current),
          "sessions": len({r["session"] for r in current}),
          "excluded_old_mapping_requests": len(rows)-len(current), "cameras": {}}
for name in ["cam_left_wrist", "cam_right_wrist"]:
    maximum = np.zeros((480, 640), np.uint8)
    for row in current:
        record = row["images"][name]
        if not record["decoded"] or record["hash_status"] != "pass":
            raise ValueError("An input image failed the archive audit")
        image = cv2.imread(str(REPORT / "archive" / record["path"]), cv2.IMREAD_GRAYSCALE)
        maximum = np.maximum(maximum, image)
    fit = fit_boundary(maximum)
    if fit["status"] != "fit":
        raise RuntimeError("Aggregate boundary fit failed for "+name)
    result["cameras"][name] = {**fit, "equidistant_assumed_full_fov_deg": 180,
                               "estimated_fx_fy_if_180deg": [v/(np.pi/2) for v in fit["radius_xy_at_640x480"]]}
    cv2.imwrite(str(REPORT / (name+"_current_maximum_boundary_analysis.png")), maximum)
    image = cv2.imread(str(REPORT / "archive" / current[-1]["images"][name]["path"]))
    cx, cy = fit["center_xy_at_640x480"]; rx, ry = fit["radius_xy_at_640x480"]
    cv2.ellipse(image, (round(cx), round(cy)), (round(rx), round(ry)), 0, 0, 360, (80, 255, 0), 1)
    cv2.imwrite(str(REPORT / (name+"_current_boundary_fit.png")), image)
(REPORT / "current_lens_boundary_fit.json").write_text(json.dumps(result, indent=2)+"\n")
print(json.dumps(result, indent=2))
