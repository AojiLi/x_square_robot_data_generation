#!/usr/bin/env python3
"""Estimate the visible lens boundary from saved real camera frames.

This estimates an image-domain ellipse, not a calibrated angular lens model.
Focal estimates assume a 180-degree equidistant imaging circle explicitly.
"""
from pathlib import Path
import hashlib
import json

import cv2
import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports/room01_wrist_fisheye_alignment"
REFERENCE = REPORT / "reference"
records = {}
for name in ["cam_left_wrist", "cam_right_wrist"]:
    images = []
    for folder in sorted(REFERENCE.glob("request_*")):
        path = folder / (name+".png")
        rgb = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2RGB)
        metadata = json.loads((folder / "request.json").read_text())
        assert hashlib.sha256(rgb.tobytes()).hexdigest() == metadata["integrity"][name]["raw_sha256"]
        images.append(rgb)
    h, w = images[0].shape[:2]
    # Combine only for boundary detection; the original reference files stay intact.
    gray = np.max([cv2.cvtColor(im, cv2.COLOR_RGB2GRAY) for im in images], axis=0).astype(np.float32)
    gray = cv2.GaussianBlur(gray, (5, 5), 1.2)
    center = np.array([w/2., h/2.])
    angles = np.linspace(0, 2*np.pi, 720, endpoint=False)
    radii = np.arange(195., 301.)
    points, strengths = [], []
    for angle in angles:
        direction = np.array([np.cos(angle), np.sin(angle)])
        xy = center+radii[:, None]*direction
        x, y = xy.T
        valid = (x >= 10)&(x < w-10)&(y >= 10)&(y < h-10)
        sample = cv2.remap(gray, x.astype(np.float32)[None], y.astype(np.float32)[None], cv2.INTER_LINEAR)[0]
        for j in [int(np.argmax(np.where(valid[5:-5], sample[:-10]-sample[10:], -1)))+5]:
            strength = float(sample[j-5]-sample[j+5])
            if valid[j] and strength > 7 and sample[j+5] < 30:
                points.append(xy[j]); strengths.append(strength)
    points = np.array(points)
    if len(points) < 80:
        raise RuntimeError("Not enough clear lens-boundary arc samples for "+name)
    def residual(p):
        cx, cy, rx, ry = p
        d = points-np.array([cx, cy])
        return (np.sqrt((d[:, 0]/rx)**2+(d[:, 1]/ry)**2)-1)*np.sqrt(rx*ry)
    fit = least_squares(residual, [w/2, h/2, 255, 255],
                        bounds=([285, 205, 230, 230], [355, 275, 285, 285]), loss="soft_l1", f_scale=2.)
    error = residual(fit.x)
    keep = np.abs(error) < 5
    cx, cy, rx, ry = fit.x
    canvas = images[0].copy()
    for x, y in points[keep]:
        cv2.circle(canvas, (round(x), round(y)), 1, (255, 180, 20), -1)
    cv2.ellipse(canvas, (round(cx), round(cy)), (round(rx), round(ry)), 0, 0, 360, (0, 255, 80), 1)
    cv2.imwrite(str(REPORT/(name+"_boundary_fit.png")), cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR))
    records[name] = {
        "source_frames": len(images), "resolution_wh": [w, h],
        "ellipse_center_xy_px": [float(cx), float(cy)], "ellipse_radius_xy_px": [float(rx), float(ry)],
        "arc_candidates": len(points), "arc_inliers": int(keep.sum()),
        "median_boundary_residual_px": float(np.median(np.abs(error[keep]))),
        "equidistant_assumed_full_fov_deg": 180.,
        "estimated_fx_fy_if_180deg": [float(rx/(np.pi/2)), float(ry/(np.pi/2))],
        "status": "image_boundary_fit_only; field of view and distortion require calibration",
    }
(REPORT / "lens_boundary_fit.json").write_text(json.dumps(records, indent=2)+"\n")
print(json.dumps(records, indent=2))
