#!/usr/bin/env python3
"""Decode every copied inference image and make a complete visual inventory.

This audits recorded images. It does not infer calibrated FOV or physical
left/right identity from a filename, dark region, or image-circle boundary.
"""
from pathlib import Path
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports/room01_wrist_fisheye_alignment"
ARCHIVE = REPORT / "archive"
CAMERAS = ["cam_high", "cam_left_wrist", "cam_right_wrist"]
cv2.setNumThreads(1)


def fit_boundary(gray):
    """Robust ellipse estimate from outward bright-to-dark boundary arcs."""
    gray = cv2.resize(gray, (640, 480), interpolation=cv2.INTER_AREA).astype(np.float32)
    gray = cv2.GaussianBlur(gray, (5, 5), 1.2)
    angles = np.arange(360)*np.pi/180
    radii = np.arange(195., 301.)
    x = 320+np.cos(angles[:, None])*radii
    y = 240+np.sin(angles[:, None])*radii
    valid = (x >= 10)&(x < 630)&(y >= 10)&(y < 470)
    sample = cv2.remap(gray, x.astype(np.float32), y.astype(np.float32), cv2.INTER_LINEAR)
    score = np.where(valid[:, 5:-5], sample[:, :-10]-sample[:, 10:], -1)
    j = np.argmax(score, axis=1)+5
    row = np.arange(len(j))
    keep = valid[row, j] & ((sample[row, j-5]-sample[row, j+5]) > 7) & (sample[row, j+5] < 30)
    points = np.c_[x[row, j][keep], y[row, j][keep]]
    if len(points) < 45:
        return {"status": "insufficient_visible_boundary", "arc_candidates": len(points)}
    def residual(p):
        cx, cy, rx, ry = p
        d = points-[cx, cy]
        return (np.sqrt((d[:, 0]/rx)**2+(d[:, 1]/ry)**2)-1)*np.sqrt(rx*ry)
    fit = least_squares(residual, [320, 240, 255, 255],
                        bounds=([280, 195, 220, 220], [365, 280, 300, 300]), loss="soft_l1", f_scale=2.)
    errors = np.abs(residual(fit.x)); inliers = errors < 5
    median = float(np.median(errors[inliers])) if inliers.any() else None
    return {"status": "fit" if inliers.sum() >= 45 and inliers.mean() > .5 else "weak_fit",
            "center_xy_at_640x480": fit.x[:2].tolist(), "radius_xy_at_640x480": fit.x[2:].tolist(),
            "arc_candidates": len(points), "arc_inliers": int(inliers.sum()), "median_residual_px": median}


def inspect_request(item):
    index, folder = item
    metadata = json.loads((folder / "request.json").read_text())
    camera_sync = metadata.get("source_context", {}).get("camera_sync", {})
    record = {"index": index, "session": folder.parent.parent.name, "request": folder.name,
              "status": metadata.get("status"), "images": {},
              "source_mapping": {n: camera_sync.get("cameras", {}).get(n, {}).get("source") for n in CAMERAS},
              "e6_source": metadata.get("source_context", {}).get("e6", {}),
              "max_pairwise_skew_ms": camera_sync.get("max_pairwise_skew_ms")}
    thumbnails = []
    for name in CAMERAS:
        path = folder / (name+".png")
        result = {"path": str(path.relative_to(ARCHIVE))}
        try:
            if path.is_symlink():
                raise ValueError("Symlink is not an image snapshot")
            with Image.open(path) as im:
                im.load(); mode = im.mode; rgb = np.asarray(im.convert("RGB")).copy()
                thumbnail = im.convert("RGB").resize((128, 96), Image.Resampling.LANCZOS)
            digest = hashlib.sha256(rgb.tobytes()).hexdigest()
            expected = metadata.get("integrity", {}).get(name, {}).get("raw_sha256")
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
            h, w = gray.shape
            result.update(decoded=True, size_wh=[w, h], mode=mode, bytes=path.stat().st_size,
                          raw_sha256=digest, recorded_hash=expected,
                          hash_status="pass" if expected == digest else ("missing" if expected is None else "mismatch"),
                          luma_mean=float(gray.mean()), luma_std=float(gray.std()),
                          dark_fraction=float((gray < 12).mean()))
            if name != "cam_high":
                result["boundary"] = fit_boundary(gray)
                # This is a diagnostic appearance feature, not a physical-side classifier.
                g = cv2.resize(gray, (640, 480), interpolation=cv2.INTER_AREA)
                result["dark_region_left_minus_right"] = float((g[120:360, 100:235] < 70).mean()-(g[120:360, 405:540] < 70).mean())
            if gray.std() < 2:
                result["appearance_flag"] = "nearly_constant_image"
        except Exception as error:
            result.update(decoded=False, error=type(error).__name__+": "+str(error))
            thumbnail = Image.new("RGB", (128, 96), (120, 0, 0))
        record["images"][name] = result
        thumbnails.append(thumbnail)
    return record, thumbnails


def summarize_boundaries(images):
    good = [v["boundary"] for v in images if v.get("boundary", {}).get("status") == "fit"]
    result = {"fit_count": len(good), "all_images": len(images)}
    if good:
        arr = np.array([v["center_xy_at_640x480"]+v["radius_xy_at_640x480"] for v in good])
        result.update(parameter_order=["cx", "cy", "rx", "ry"], median=np.median(arr, axis=0).tolist(),
                      p05=np.percentile(arr, 5, axis=0).tolist(), p95=np.percentile(arr, 95, axis=0).tolist())
    return result


def main():
    folders = sorted(p.parent for p in ARCHIVE.glob("*/requests/*/request.json"))
    all_files = sorted(ARCHIVE.glob("*/requests/*/cam_*.png"))
    contacts = REPORT / "contact_sheets"; contacts.mkdir(exist_ok=True)
    thumb_folder = REPORT / "thumbnails"; thumb_folder.mkdir(exist_ok=True)
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 13)
    title_font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 21)
    records = []; sheet = None; current_page = 0
    with ThreadPoolExecutor(max_workers=4) as pool:
        for record, thumbnails in pool.map(inspect_request, enumerate(folders)):
            index = record["index"]; page, slot = divmod(index, 36)
            if slot == 0:
                if sheet is not None:
                    sheet.save(contacts/f"all_requests_{current_page+1:02d}.jpg", quality=91)
                current_page = page
                sheet = Image.new("RGB", (1600, 1225), (22, 23, 25))
                ImageDraw.Draw(sheet).text((15, 12), f"All recorded requests — page {page+1}/{math.ceil(len(folders)/36)} | each card: HEAD / LEFT / RIGHT", fill="white", font=title_font)
            row, col = divmod(slot, 4); x=col*400+8; y=48+row*130
            label=f"#{index:04d} {record['session'][:15]} {record['request'].replace('request_','r')}"
            ImageDraw.Draw(sheet).text((x,y),label,fill="white",font=font)
            for k, im in enumerate(thumbnails):
                sheet.paste(im,(x+k*128,y+22))
                im.save(thumb_folder/f"{index:04d}_{CAMERAS[k]}.jpg",quality=87)
            records.append(record)
            if (index+1)%100 == 0:
                print(f"AUDITED {index+1}/{len(folders)} requests",flush=True)
    if sheet is not None:sheet.save(contacts/f"all_requests_{current_page+1:02d}.jpg",quality=91)
    sizes=Counter(); failures=[]; missing_hash=[]; bytes_total=0
    for r in records:
        for n,v in r["images"].items():
            if not v.get("decoded") or v.get("hash_status")=="mismatch":failures.append({"index":r["index"],"camera":n,**v})
            if v.get("hash_status")=="missing":missing_hash.append({"index":r["index"],"camera":n})
            if v.get("decoded"):sizes[str(v["size_wh"])]+=1;bytes_total+=v["bytes"]
    sessions={}
    for session in sorted({r['session'] for r in records}):
        rows=[r for r in records if r['session']==session]
        sessions[session]={"requests":len(rows),"first_index":rows[0]['index'],"last_index":rows[-1]['index'],
                           "source_mappings":dict(Counter(json.dumps(r['source_mapping'],sort_keys=True) for r in rows)),
                           "wrist_boundaries":{n:summarize_boundaries([r['images'][n] for r in rows]) for n in CAMERAS[1:]}}
    summary={"result":"pass" if not failures and len(all_files)==3*len(records) else "review_needed",
             "source_root":"192.168.110.11:/mnt/data/dzq/inference_records", "read_only_source":True,
             "sessions":len(sessions),"requests":len(records),"discovered_images":len(all_files),
             "decoded_images":sum(v.get('decoded',False) for r in records for v in r['images'].values()),
             "image_bytes":bytes_total,"size_counts":dict(sizes),"failures":failures,"missing_hash":missing_hash,
             "source_mapping_counts":dict(Counter(json.dumps(r['source_mapping'],sort_keys=True) for r in records)),
             "wrist_boundaries":{n:summarize_boundaries([r['images'][n] for r in records]) for n in CAMERAS[1:]},
             "contact_sheet_pages":math.ceil(len(records)/36),
             "scope":"Every copied PNG decoded and checked, every image included in the contact sheets; boundary estimates are not an angular or hand-eye calibration."}
    (REPORT/'archive_images.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
    (REPORT/'archive_sessions.json').write_text(json.dumps(sessions,indent=2)+'\n')
    (REPORT/'archive_audit.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2),flush=True)


if __name__=='__main__':main()
