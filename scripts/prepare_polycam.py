#!/usr/bin/env python3
"""Build a metric, Z-up Nerfstudio dataset from this Polycam export.

Corrected RGB and camera parameters are paired by timestamp. Depth is preserved
as source data, not silently attached as registered photometric supervision.
"""

import argparse
import io
import itertools
import json
import struct
import zipfile
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import binary_propagation
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]


def write_json(path, obj):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2) + "\n")


def transform_points(points, matrix):
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def camera_matrix(camera):
    result = np.eye(4)
    result[:3] = [[camera[f"t_{i}{j}"] for j in range(4)] for i in range(3)]
    return result


def read_glb(payload):
    magic, version, length = struct.unpack_from("<III", payload)
    if magic != 0x46546C67 or version != 2 or length != len(payload):
        raise ValueError("Unexpected GLB container")
    json_length, json_type = struct.unpack_from("<II", payload, 12)
    if json_type != 0x4E4F534A:
        raise ValueError("Missing GLB JSON chunk")
    gltf = json.loads(payload[20:20 + json_length])
    binary_length, binary_type = struct.unpack_from("<II", payload, 20 + json_length)
    if binary_type != 0x004E4942:
        raise ValueError("Missing GLB binary chunk")
    binary = payload[28 + json_length:28 + json_length + binary_length]
    vertices = []
    for mesh in gltf["meshes"]:
        for primitive in mesh["primitives"]:
            accessor = gltf["accessors"][primitive["attributes"]["POSITION"]]
            view = gltf["bufferViews"][accessor["bufferView"]]
            if accessor["componentType"] != 5126 or accessor["type"] != "VEC3":
                raise ValueError("Unsupported GLB vertex accessor")
            start = view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
            stride = view.get("byteStride", 12)
            vertices.append(np.ndarray((accessor["count"], 3), dtype="<f4", buffer=binary,
                                       offset=start, strides=(stride, 4)).copy())
    # This capture has no authored node transforms; do not ignore them silently.
    if any(any(key in node for key in ["matrix", "translation", "rotation", "scale"]) for node in gltf["nodes"]):
        raise ValueError("GLB has node transforms; flatten the scene graph before sampling")
    return gltf, binary, np.concatenate(vertices).astype(np.float64)


def write_transformed_glb(path, gltf, binary, matrix):
    result = json.loads(json.dumps(gltf))
    scene = result["scenes"][result.get("scene", 0)]
    result["nodes"].append({"name": "PolycamMetricZUp", "children": scene["nodes"],
                            "matrix": matrix.T.reshape(-1).tolist()})
    scene["nodes"] = [len(result["nodes"]) - 1]
    result.setdefault("extras", {})["coordinate_system"] = "right-handed Z-up, meters"
    encoded = json.dumps(result, separators=(",", ":")).encode()
    encoded += b" " * (-len(encoded) % 4)
    binary += b"\0" * (-len(binary) % 4)
    body = struct.pack("<II", len(encoded), 0x4E4F534A) + encoded
    body += struct.pack("<II", len(binary), 0x004E4942) + binary
    path.write_bytes(struct.pack("<III", 0x46546C67, 2, 12 + len(body)) + body)


def read_polycam_ply(payload):
    header_end = payload.index(b"end_header\n") + len(b"end_header\n")
    header = payload[:header_end].decode("ascii")
    expected = ["property double x", "property double y", "property double z",
                "property uchar red", "property uchar green", "property uchar blue"]
    if "format binary_little_endian 1.0" not in header or [line for line in header.splitlines()
                                                          if line.startswith("property ")] != expected:
        raise ValueError("Unexpected Polycam point cloud format")
    dtype = [(key, "<f8") for key in "xyz"] + [(key, "u1") for key in ["red", "green", "blue"]]
    data = np.frombuffer(payload, dtype=dtype, offset=header_end)
    declared = int(next(line for line in header.splitlines() if line.startswith("element vertex")).split()[-1])
    if len(data) != declared:
        raise ValueError("Point cloud size mismatch")
    points = np.column_stack([data[key] for key in "xyz"])
    colors = np.column_stack([data[key] for key in ["red", "green", "blue"]])
    return points, colors


def write_ply(path, points, colors):
    dtype = [(key, "<f4") for key in "xyz"] + [(key, "u1") for key in ["red", "green", "blue"]]
    data = np.empty(len(points), dtype=dtype)
    for i, key in enumerate("xyz"):
        data[key] = points[:, i]
    for i, key in enumerate(["red", "green", "blue"]):
        data[key] = colors[:, i]
    header = "ply\nformat binary_little_endian 1.0\ncomment metric Z-up initialization points, not Gaussian splats\n"
    header += f"element vertex {len(points)}\n"
    header += "".join(f"property float {key}\n" for key in "xyz")
    header += "".join(f"property uchar {key}\n" for key in ["red", "green", "blue"])
    with path.open("wb") as target:
        target.write((header + "end_header\n").encode())
        target.write(data.tobytes())


def resolve_ply_axes(points, target_vertices):
    rng = np.random.default_rng(20260909)
    sample = points[rng.choice(len(points), min(5000, len(points)), replace=False)]
    tree = cKDTree(target_vertices)
    candidates = []
    for permutation in itertools.permutations(range(3)):
        for signs in itertools.product([-1, 1], repeat=3):
            rotation = np.eye(3)[list(permutation)] * np.array(signs)[:, None]
            if np.linalg.det(rotation) < 0:
                continue
            distance, _ = tree.query(sample @ rotation.T, workers=4)
            candidates.append({"rotation": rotation.tolist(), "median_m": float(np.median(distance)),
                               "p90_m": float(np.quantile(distance, 0.9))})
    candidates.sort(key=lambda item: item["median_m"] + item["p90_m"])
    best = candidates[0]
    if best["median_m"] > 0.03 or best["p90_m"] > 0.075:
        raise ValueError(f"No sufficiently close PLY/GLB axis match: {best}")
    matrix = np.eye(4)
    matrix[:3, :3] = best["rotation"]
    return matrix, candidates


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "data/room01")
    parser.add_argument("--crop", type=int, default=16)
    parser.add_argument("--voxel", type=float, default=0.02)
    parser.add_argument("--max-points", type=int, default=180000)
    args = parser.parse_args()
    output = args.output.resolve()
    if (output / "transforms.json").exists():
        raise SystemExit("Dataset already exists; choose a new output directory to preserve it.")
    assets = ROOT / "assets/room01"
    assets.mkdir(parents=True, exist_ok=True)
    for directory in ["images", "images_2", "images_4", "masks", "masks_2", "masks_4",
                      "depth_original", "depth_corrected", "confidence_original"]:
        (output / directory).mkdir(parents=True, exist_ok=True)

    with zipfile.ZipFile(ROOT / "Raw_room.zip") as raw:
        metadata = json.loads(raw.read("mesh_info.json"))
    alignment = np.array(metadata["alignmentTransform"]).reshape(4, 4).T
    # GLB Y-up -> canonical world Z-up: X=x, Y=-z, Z=y.
    world_from_glb = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], dtype=float)
    world_from_arkit = world_from_glb @ alignment
    if not np.allclose(world_from_arkit[:3, :3].T @ world_from_arkit[:3, :3], np.eye(3), atol=1e-6):
        raise ValueError("Unexpected scale/shear in Polycam alignment")

    with zipfile.ZipFile(ROOT / "GLTF_room.zip") as archive:
        original = archive.read(next(name for name in archive.namelist() if name.endswith(".glb")))
    gltf, binary, vertices = read_glb(original)
    (assets / "scan_original.glb").write_bytes(original)
    # glTF's standard convention is Y-up. Keep its original asset unchanged;
    # export the Z-up geometry as native USD in scripts/export_scene.py.
    world_vertices = transform_points(vertices, world_from_glb)
    with zipfile.ZipFile(ROOT / "Point_cloud_ply.zip") as archive:
        points, colors = read_polycam_ply(archive.read(next(name for name in archive.namelist() if name.endswith(".ply"))))
    world_from_ply, axis_matches = resolve_ply_axes(points, world_vertices)
    points = transform_points(points, world_from_ply)
    finite = np.isfinite(points).all(axis=1)
    points, colors = points[finite], colors[finite]
    voxel_keys = np.floor(points / args.voxel).astype(np.int32)
    _, keep = np.unique(voxel_keys, axis=0, return_index=True)
    voxel_count = len(keep)
    if len(keep) > args.max_points:
        keep = np.random.default_rng(20260909).choice(keep, args.max_points, replace=False)
    keep = np.sort(keep)
    write_ply(output / "points3D.ply", points[keep], colors[keep])
    print(f"Geometry aligned; {len(keep):,} initial points", flush=True)

    frames, provenance = [], []
    with zipfile.ZipFile(ROOT / "Image_room.zip") as archive:
        names = sorted(name for name in archive.namelist() if name.startswith("keyframes/corrected_images/")
                       and name.endswith(".jpg"))
        for index, name in enumerate(names):
            frame_id = Path(name).stem
            camera_path = f"keyframes/corrected_cameras/{frame_id}.json"
            camera = json.loads(archive.read(camera_path))
            image = Image.open(io.BytesIO(archive.read(name))).convert("RGB")
            if image.size != (camera["width"], camera["height"]):
                raise ValueError("Camera/image dimensions disagree")
            c = args.crop
            image = image.crop((c, c, image.width-c, image.height-c))
            pixels = np.asarray(image)
            black = (pixels == 0).all(axis=2)
            boundary = np.zeros(black.shape, dtype=bool)
            boundary[0] = black[0]; boundary[-1] = black[-1]
            boundary[:, 0] = black[:, 0]; boundary[:, -1] = black[:, -1]
            invalid = binary_propagation(boundary, mask=black)
            mask = Image.fromarray((~invalid).astype(np.uint8) * 255)
            for factor in [1, 2, 4]:
                suffix = "" if factor == 1 else f"_{factor}"
                size = (image.width // factor, image.height // factor)
                rgb = image if factor == 1 else image.resize(size, Image.Resampling.LANCZOS)
                resized_mask = mask if factor == 1 else mask.resize(size, Image.Resampling.NEAREST)
                rgb.save(output / f"images{suffix}" / f"{frame_id}.png")
                resized_mask.save(output / f"masks{suffix}" / f"{frame_id}.png")
            transform = world_from_arkit @ camera_matrix(camera)
            frames.append({"file_path": f"images/{frame_id}.png", "mask_path": f"masks/{frame_id}.png",
                           "fl_x": camera["fx"], "fl_y": camera["fy"],
                           "cx": camera["cx"]-c, "cy": camera["cy"]-c,
                           "w": image.width, "h": image.height,
                           "transform_matrix": transform.tolist()})
            depth_sources = {}
            for destination, source in [("depth_original", "depth"), ("depth_corrected", "corrected_depth"),
                                        ("confidence_original", "confidence")]:
                member = f"keyframes/{source}/{frame_id}.png"
                (output / destination / f"{frame_id}.png").write_bytes(archive.read(member))
                depth_sources[destination] = member
            provenance.append({"frame_id": frame_id, "source_image": name, "source_camera": camera_path,
                               "source_depth": depth_sources, "crop_pixels_each_edge": c,
                               "masked_border_fraction": float(invalid.mean()),
                               "blur_score": camera["blur_score"], "weakly_connected": camera.get("weakly_connected")})
            if (index+1) % 100 == 0:
                print(f"Prepared {index+1}/{len(names)} views", flush=True)
    evaluation = [frame["file_path"] for index, frame in enumerate(frames) if index % 10 == 4]
    training = [frame["file_path"] for index, frame in enumerate(frames) if index % 10 != 4]
    transforms = {"camera_model": "OPENCV", "orientation_override": "none", "ply_file_path": "points3D.ply",
                  "frames": frames, "train_filenames": training, "val_filenames": evaluation,
                  "test_filenames": evaluation}
    write_json(output / "transforms.json", transforms)
    write_json(output / "provenance.json", provenance)
    coordinate_record = {
        "units": "meters", "world_axes": "right-handed Z-up; origin at exported GLB origin",
        "camera_axes": "OpenGL: +X right, +Y up, viewing along -Z",
        "world_from_glb": world_from_glb.tolist(), "world_from_polycam_camera_world": world_from_arkit.tolist(),
        "world_from_exported_ply": world_from_ply.tolist(),
        "polycam_camera_world_from_world": np.linalg.inv(world_from_arkit).tolist(),
        "glb_from_world": np.linalg.inv(world_from_glb).tolist(),
        "alignment_transform_original_column_major": metadata["alignmentTransform"],
        "ply_glb_alignment_candidates": axis_matches,
        "training_required_settings": {"orientation_method": "none", "center_method": "none",
                                       "auto_scale_poses": False, "scale_factor": 1.0},
        "world_bounds_m": [points.min(axis=0).tolist(), points.max(axis=0).tolist()],
        "depth_usage": "Preserved in original pixel grids; not attached to baseline RGB loss.",
        "initialization_points": len(keep), "voxel_size_m": args.voxel, "voxel_count_before_cap": voxel_count,
        "train_views": len(training), "diagnostic_evaluation_views": len(evaluation),
        "evaluation_note": "Evaluation and test aliases use the same 49 views, excluded from RGB optimization. "
                           "Polycam poses and initialization geometry were reconstructed using the full scan; "
                           "this is not an independent-capture generalization benchmark.",
    }
    write_json(output / "coordinate_transforms.json", coordinate_record)
    write_json(assets / "coordinate_transforms.json", coordinate_record)
    print(json.dumps({"dataset": str(output), "train_views": len(training), "eval_views": len(evaluation),
                      "initial_points": len(keep), "best_axis_alignment": axis_matches[0]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
