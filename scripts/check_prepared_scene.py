#!/usr/bin/env python3
"""Validate the prepared capture and preview its initialization geometry."""

import json
import zipfile
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data/room01"
OUT = ROOT / "reports/room01_reconstruction"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    meta = json.loads((DATA / "transforms.json").read_text())
    coordinates = json.loads((DATA / "coordinate_transforms.json").read_text())
    frames = meta["frames"]
    train, evaluation = set(meta["train_filenames"]), set(meta["val_filenames"])
    if train & evaluation or train | evaluation != {f["file_path"] for f in frames}:
        raise ValueError("Training/evaluation partition is incomplete or overlapping")
    with (DATA / "points3D.ply").open("rb") as source:
        while source.readline().strip() != b"end_header":
            pass
        dtype = [(key, "<f4") for key in "xyz"] + [(key, "u1") for key in ["red", "green", "blue"]]
        cloud = np.frombuffer(source.read(), dtype=dtype)
    points = np.column_stack([cloud[key] for key in "xyz"])
    colors = np.column_stack([cloud[key] for key in ["red", "green", "blue"]])
    if not np.isfinite(points).all():
        raise ValueError("Nonfinite initialization points")
    max_rotation_error = 0.0
    for frame in frames:
        transform = np.array(frame["transform_matrix"])
        max_rotation_error = max(max_rotation_error, float(np.abs(transform[:3, :3].T @ transform[:3, :3]-np.eye(3)).max()))
        if not np.allclose(transform[3], [0, 0, 0, 1]) or np.linalg.det(transform[:3, :3]) < 0.99999:
            raise ValueError("Invalid camera transform")
        for factor in [1, 2, 4]:
            directory = "images" if factor == 1 else f"images_{factor}"
            path = DATA / directory / Path(frame["file_path"]).name
            with Image.open(path) as image:
                if image.size != (frame["w"]//factor, frame["h"]//factor):
                    raise ValueError("Image dimensions disagree with scaled calibration")

    # Independently compare several lossless prepared images to the source crop.
    indices = [4, 54, 124, 204, 354, 384]
    import io
    with zipfile.ZipFile(ROOT / "Image_room.zip") as archive:
        for index in indices:
            frame = frames[index]
            source_path = "keyframes/corrected_images/"+Path(frame["file_path"]).stem+".jpg"
            image = Image.open(io.BytesIO(archive.read(source_path))).convert("RGB")
            reference = np.asarray(image.crop((16, 16, image.width-16, image.height-16)))
            actual = np.asarray(Image.open(DATA / frame["file_path"]))
            if not np.array_equal(actual, reference):
                raise ValueError("Prepared pixels changed beyond the documented crop")

    width, height = 496, 368
    canvas = Image.new("RGB", (width*2, (height+30)*len(indices)), (245, 245, 245))
    draw = ImageDraw.Draw(canvas)
    projections = []
    for row, index in enumerate(indices):
        frame = frames[index]
        transform = np.array(frame["transform_matrix"])
        camera_points = (points-transform[:3, 3]) @ transform[:3, :3]
        z = -camera_points[:, 2]
        keep = z > 0.05
        p, color, depth = camera_points[keep], colors[keep], z[keep]
        u = np.rint((frame["fl_x"] * p[:, 0] / depth + frame["cx"]) / 2).astype(int)
        v = np.rint((frame["cy"] - frame["fl_y"] * p[:, 1] / depth) / 2).astype(int)
        keep = (u >= 0) & (u < width) & (v >= 0) & (v < height)
        u, v, color, depth = u[keep], v[keep], color[keep], depth[keep]
        order = np.argsort(depth)
        unique, first = np.unique((v*width+u)[order], return_index=True)
        render = np.full((width*height, 3), 26, dtype=np.uint8)
        render[unique] = color[order[first]]
        photo = Image.open(DATA / "images_2" / Path(frame["file_path"]).name)
        y = row*(height+30)
        canvas.paste(photo, (0, y))
        canvas.paste(Image.fromarray(render.reshape(height, width, 3)), (width, y))
        draw.text((6, y+height+7), f"Frame {index+1}: corrected RGB | metric initialization points (not trained splats)", fill=(20, 20, 20))
        projections.append({"frame_index": index, "visible_point_pixels": len(unique)})
    canvas.save(OUT / "initialization_alignment.jpg", quality=90)
    report = {
        "status": "structural_checks_passed",
        "frames": len(frames), "train_frames": len(train), "evaluation_frames": len(evaluation),
        "initial_points": len(points), "max_camera_rotation_orthogonality_error": max_rotation_error,
        "image_pyramid_and_calibration_dimensions": "pass",
        "six_source_crop_pixel_comparisons": "exact match",
        "camera_world_units": coordinates["units"],
        "ply_to_glb_internal_alignment": coordinates["ply_glb_alignment_candidates"][0],
        "projection_previews": projections,
        "limitation": "Internal alignment, pose structure and dimensions do not certify real-world geometric accuracy.",
    }
    (OUT / "prepared_dataset_checks.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
