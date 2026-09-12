#!/usr/bin/env python3
"""Inspect this Polycam capture without extracting or changing source archives.

Requires Pillow. Writes a JSON audit and timestamp-based frame manifest.
Usage: python3 scripts/audit_polycam.py
"""

import hashlib
import io
import json
import math
import statistics
import struct
import zipfile
from collections import Counter
from pathlib import Path

from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reports" / "polycam_audit"


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    audit = {"scope": "File integrity and structural audit; not a reconstruction quality acceptance test."}
    archives = []
    for path in sorted(ROOT.glob("*.zip")):
        with zipfile.ZipFile(path) as archive:
            files = [entry for entry in archive.infolist() if not entry.is_dir()]
            bad = archive.testzip()
            if bad is not None:
                raise ValueError(f"CRC failure: {path.name}: {bad}")
            archives.append({
                "name": path.name,
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
                "file_count": len(files),
                "uncompressed_bytes": sum(entry.file_size for entry in files),
                "groups": dict(sorted(Counter(str(Path(entry.filename).parent) for entry in files).items())),
                "crc_check": "pass",
            })
    audit["archives"] = archives

    with zipfile.ZipFile(ROOT / "Image_room.zip") as images, zipfile.ZipFile(ROOT / "Raw_room.zip") as raw:
        image_names = {entry.filename for entry in images.infolist() if not entry.is_dir()}
        raw_names = {entry.filename for entry in raw.infolist() if not entry.is_dir()}
        common = image_names & raw_names
        differences = sorted(name for name in common if images.read(name) != raw.read(name))
        audit["shared_archive_members"] = {
            "count": len(common), "identical_payloads": len(common) - len(differences),
            "different_payloads": differences,
        }
        ids = sorted(Path(name).stem for name in image_names
                     if str(Path(name).parent) == "keyframes/corrected_cameras" and name.endswith(".json"))
        manifest = []
        dimensions = {}
        invalid_images = []
        for frame_id in ids:
            members = {
                "rgb_original": f"keyframes/images/{frame_id}.jpg",
                "camera_original": f"keyframes/cameras/{frame_id}.json",
                "rgb_corrected": f"keyframes/corrected_images/{frame_id}.jpg",
                "camera_corrected": f"keyframes/corrected_cameras/{frame_id}.json",
                "depth_original": f"keyframes/depth/{frame_id}.png",
                "depth_original_clean_variant": f"keyframes/depth/{frame_id}.Clean.png",
                "depth_corrected": f"keyframes/corrected_depth/{frame_id}.png",
                "depth_corrected_clean_variant": f"keyframes/corrected_depth/{frame_id}.Clean.png",
                "confidence_original": f"keyframes/confidence/{frame_id}.png",
                "confidence_clean_variant": f"keyframes/confidence/{frame_id}.Clean.png",
            }
            missing = [name for name in members.values() if name not in image_names]
            if missing:
                raise ValueError(f"Missing frame members: {missing}")
            corrected = json.loads(images.read(members["camera_corrected"]))
            for key, name in members.items():
                if name.endswith(".json"):
                    continue
                try:
                    with Image.open(io.BytesIO(images.read(name))) as image:
                        image.load()
                        dimension_key = f"{image.width}x{image.height}:{image.mode}"
                        dimensions.setdefault(key, Counter())[dimension_key] += 1
                        if key == "rgb_corrected" and image.size != (corrected["width"], corrected["height"]):
                            raise ValueError("RGB dimensions disagree with camera JSON")
                except Exception as error:
                    invalid_images.append({"member": name, "error": str(error)})
            manifest.append({
                "frame_id": frame_id,
                "archive": "Image_room.zip",
                "members": members,
                "blur_score": corrected["blur_score"],
                "weakly_connected": corrected.get("weakly_connected"),
                "usage": "Source associations only; corrected depth/confidence registration is not certified.",
            })
        if invalid_images:
            raise ValueError(f"Invalid image payloads: {invalid_images}")
        audit["unique_frames"] = len(ids)
        audit["image_decode_checks"] = {key: dict(value) for key, value in dimensions.items()}
        audit["sfm_stats"] = json.loads(images.read("keyframes/sfm_stats.json"))

        cams = [json.loads(images.read(frame["members"]["camera_corrected"])) for frame in manifest]
        originals = [json.loads(images.read(frame["members"]["camera_original"])) for frame in manifest]
        rotation_errors, displacement = [], []
        for camera, original in zip(cams, originals):
            rotation = [[camera[f"t_{i}{j}"] for j in range(3)] for i in range(3)]
            rotation_errors.append(max(abs(sum(rotation[k][i] * rotation[k][j] for k in range(3)) - (i == j))
                                       for i in range(3) for j in range(3)))
            displacement.append(math.sqrt(sum((camera[f"t_{i}3"] - original[f"t_{i}3"]) ** 2 for i in range(3))))
        scores = [camera["blur_score"] for camera in cams]
        audit["camera_checks"] = {
            "max_rotation_orthogonality_error": max(rotation_errors),
            "median_pose_correction_translation_m": statistics.median(displacement),
            "max_pose_correction_translation_m": max(displacement),
            "blur_min_median_max": [min(scores), statistics.median(scores), max(scores)],
            "weakly_connected_true": sum(camera.get("weakly_connected") is True for camera in cams),
            "note": "Rigid matrices and paired files do not prove accurate poses.",
        }
        metadata = json.loads(raw.read("mesh_info.json"))
        audit["mesh_metadata"] = {key: metadata[key] for key in
                                  ["bboxSize", "vertexCount", "triangleCount", "alignmentTransform"]}

    with zipfile.ZipFile(ROOT / "Point_cloud_ply.zip") as archive:
        name = next(name for name in archive.namelist() if name.endswith(".ply"))
        with archive.open(name) as source:
            header = []
            while True:
                line = source.readline().decode("ascii").strip()
                if not line:
                    raise ValueError("Incomplete PLY header")
                header.append(line)
                if line == "end_header":
                    break
            expected = ["property double x", "property double y", "property double z",
                        "property uchar red", "property uchar green", "property uchar blue"]
            if [line for line in header if line.startswith("property ")] != expected:
                raise ValueError("Unexpected PLY layout; inspect before decoding")
            lows, highs = [math.inf] * 3, [-math.inf] * 3
            point_count = 0
            for point in struct.iter_unpack("<dddBBB", source.read()):
                point_count += 1
                for i in range(3):
                    lows[i] = min(lows[i], point[i])
                    highs[i] = max(highs[i], point[i])
            declared = int(next(line for line in header if line.startswith("element vertex ")).split()[-1])
            if point_count != declared:
                raise ValueError("PLY vertex count mismatch")
            audit["ply"] = {"header": header, "points": point_count, "bounds": [lows, highs],
                            "extents": [highs[i] - lows[i] for i in range(3)],
                            "trained_gaussian_splat": False}

    with zipfile.ZipFile(ROOT / "GLTF_room.zip") as archive:
        payload = archive.read(next(name for name in archive.namelist() if name.endswith(".glb")))
        json_size, chunk_type = struct.unpack_from("<II", payload, 12)
        if chunk_type != 0x4E4F534A:
            raise ValueError("Unexpected GLB JSON chunk")
        glb = json.loads(payload[20:20 + json_size])
        primitives = [primitive for mesh in glb["meshes"] for primitive in mesh["primitives"]]
        positions = [glb["accessors"][primitive["attributes"]["POSITION"]] for primitive in primitives]
        audit["glb"] = {
            "vertices": sum(accessor["count"] for accessor in positions),
            "triangles": sum(glb["accessors"][primitive["indices"]]["count"] // 3 for primitive in primitives),
            "materials": len(glb["materials"]), "textures": len(glb["images"]),
            "bounds": [[min(accessor["min"][i] for accessor in positions),
                        max(accessor["max"][i] for accessor in positions)] for i in range(3)],
        }

    (OUT / "audit.json").write_text(json.dumps(audit, indent=2, ensure_ascii=False) + "\n")
    (OUT / "frame_manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({"unique_frames": audit["unique_frames"], "archives_crc": "pass",
                      "image_decodes": "pass", "shared_identical_members": len(common) - len(differences),
                      "report_dir": str(OUT)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
