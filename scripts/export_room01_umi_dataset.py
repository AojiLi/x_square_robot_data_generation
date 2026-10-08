#!/usr/bin/env python3
"""Export achieved robot motion as an explicitly synthetic LeRobot v2.1 episode."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import struct
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from room01_sim.umi import inverse, pose9, action_chunks


def encoder_environment():
    encoder = ROOT / "tools/video_encoder/root/usr/bin/ffmpeg"
    env = os.environ.copy()
    library = ROOT / "tools/modeling_blender_deb/root/usr/lib"
    extra = ROOT / "tools/video_encoder/root/usr/lib/x86_64-linux-gnu"
    env["LD_LIBRARY_PATH"] = ":".join([str(extra), str(library / "x86_64-linux-gnu"), str(library),
                                      str(library / "x86_64-linux-gnu/blas"), str(library / "x86_64-linux-gnu/lapack"),
                                      env.get("LD_LIBRARY_PATH", "")])
    return str(encoder), env


def numeric_stats(array):
    array = np.asarray(array)
    return {**{name: np.atleast_1d(function(array, axis=0)).tolist()
               for name, function in (("min", np.min), ("max", np.max), ("mean", np.mean), ("std", np.std))},
            "count": [len(array)]}


def export(episode, output, source, allow_failure=False):
    import pyarrow as pa
    import pyarrow.parquet as pq
    report = json.loads((episode / "replay.json").read_text())
    if report["status"] != "complete" or not report["images_recorded"]:
        raise ValueError("Export requires a completed replay with synchronized images")
    if not report["task_success"] and not allow_failure:
        raise ValueError("Failed replay: use --allow-failure only to create a clearly labeled diagnostic dataset")
    if output.exists():
        raise FileExistsError(f"Refusing to replace an existing dataset: {output}")
    arrays = np.load(episode / "observations.npz")
    count = len(arrays["timestamp"])
    if not np.allclose(np.diff(arrays["timestamp"]), .1, atol=1e-7):
        raise ValueError("Replay observations are not uniformly sampled at 10 Hz")
    if np.max(np.ptp(arrays["camera_timestamps"], axis=1)) > 1e-9:
        raise ValueError("Three camera timestamps differ")
    if not np.allclose(arrays["camera_timestamps"][:, 0], arrays["simulation_timestamp"], atol=1e-9):
        raise ValueError("Camera/state timestamps differ")
    tools = arrays["tool_poses"]
    previous = np.concatenate([arrays["previous_tool_poses"][None], tools[:-1]])
    deltas = inverse(previous) @ tools
    state = np.concatenate([pose9(deltas[:, 0]), pose9(deltas[:, 1]), arrays["fingers_deg"].reshape(count, 12)], axis=-1).astype(np.float32)
    action, padding = action_chunks(tools, arrays["fingers_deg"])
    columns = {"observation.state": state, "action": action, "action_is_pad": padding,
               "sample_valid_action_horizon": (~padding.any(axis=1)).astype(np.int64),
               "timestamp": arrays["timestamp"].astype(np.float32), "frame_index": np.arange(count, dtype=np.int64),
               "episode_index": np.zeros(count, np.int64), "index": np.arange(count, dtype=np.int64),
               "task_index": np.zeros(count, np.int64)}
    if not all(np.isfinite(value).all() for value in columns.values()):
        raise ValueError("Nonfinite export labels")
    # Retain the exact HF Array2D metadata and fixed-size list schema expected
    # by the existing UMI loader, without inventing real capture timestamps.
    source_files = list(source.glob("episode_*.parquet"))
    if len(source_files) != 1:
        raise ValueError("Expected one source episode Parquet file")
    source_schema = pq.read_schema(source_files[0])
    fields = [source_schema.field(name) for name in columns]
    hf_metadata = json.loads(source_schema.metadata[b"huggingface"])
    hf_metadata["info"]["features"] = {name: hf_metadata["info"]["features"][name] for name in columns}
    schema = pa.schema(fields, metadata={b"huggingface": json.dumps(hf_metadata).encode()})
    table = pa.Table.from_arrays([pa.array(columns[field.name].tolist(), type=field.type) for field in fields], schema=schema)
    (output / "data/chunk-000").mkdir(parents=True)
    (output / "meta").mkdir()
    parquet = output / "data/chunk-000/episode_000000.parquet"
    pq.write_table(table, parquet, compression="zstd")
    metadata = copy.deepcopy(json.loads((source / "meta/info.json").read_text()))
    camera_keys = {"head": "observation.images.head_rgb", "left_wrist": "observation.images.left_wrist_rgb",
                   "right_wrist": "observation.images.right_wrist_rgb"}
    metadata["features"] = {name: feature for name, feature in metadata["features"].items()
                            if name in columns or name in camera_keys.values()}
    metadata.update(robot_type="quanta_x2_dual_revo2_sim_umi30", total_episodes=1, total_frames=count,
                    total_tasks=1, total_videos=3, total_chunks=1, splits={"train": "0:1"}, fps=10)
    encoder, env = encoder_environment()
    stats = {key: numeric_stats(value) for key, value in columns.items() if key != "action"}
    stats["action"] = numeric_stats(action[~padding])
    transforms, video_audit = {}, {}
    for camera, key in camera_keys.items():
        files = sorted((episode / "images" / camera).glob("*.png"))
        if len(files) != count:
            raise ValueError(f"Camera count mismatch: {camera}: {len(files)} != {count}")
        target = output / "videos/chunk-000" / key / "episode_000000.mp4"
        target.parent.mkdir(parents=True)
        with files[0].open("rb") as image_file:
            header = image_file.read(24)
        if header[:8] != b"\x89PNG\r\n\x1a\n":
            raise ValueError("Native replay image is not PNG")
        source_size = struct.unpack(">II", header[16:24])
        scale = min(640/source_size[0], 480/source_size[1])
        resized = [round(source_size[0]*scale), round(source_size[1]*scale)]
        offset = [(640-resized[0])/2, (480-resized[1])/2]
        transforms[camera] = {"native_size_wh": list(source_size), "output_size_wh": [640, 480],
                              "scale_xy": [scale, scale], "padding_xy": offset,
                              "method": "isotropic scale and centered black letterbox; native PNGs retained in replay"}
        subprocess.run([encoder, "-y", "-loglevel", "error", "-framerate", "10", "-i", str(files[0].parent / "%06d.png"),
                        "-vf", "scale=640:480:force_original_aspect_ratio=decrease,pad=640:480:(ow-iw)/2:(oh-ih)/2,setsar=1",
                        "-frames:v", str(count), "-c:v", "libx264", "-preset", "fast", "-crf", "18", "-pix_fmt", "yuv420p",
                        "-threads", "4", "-movflags", "+faststart", str(target)], check=True, env=env)
        # Decode the delivered video to verify all frames and derive RGB stats.
        decoded = subprocess.run([encoder, "-loglevel", "error", "-i", str(target), "-f", "rawvideo", "-pix_fmt", "rgb24", "pipe:1"],
                                 check=True, stdout=subprocess.PIPE, env=env).stdout
        expected = count*640*480*3
        if len(decoded) != expected:
            raise ValueError(f"Encoded video has an unexpected frame count: {camera}")
        pixels = np.frombuffer(decoded, np.uint8).reshape(count, 480, 640, 3)
        sample = pixels[np.linspace(0, count-1, min(64, count)).round().astype(int)].astype(np.float32)/255.
        stats[key] = {name: func(sample, axis=(0, 1, 2))[:, None, None].tolist()
                      for name, func in (("min", np.min), ("max", np.max), ("mean", np.mean), ("std", np.std))}
        stats[key]["count"] = [len(sample)]
        metadata["features"][key]["info"] = {"video.fps": 10., "video.codec": "h264", "video.pix_fmt": "yuv420p",
                                             "video.height": 480, "video.width": 640, "video.channels": 3,
                                             "video.is_depth_map": False, "has_audio": False}
        video_audit[camera] = {"frames": count, "size": [640, 480], "sha256": hashlib.sha256(target.read_bytes()).hexdigest()}
    (output / "meta/info.json").write_text(json.dumps(metadata, indent=2))
    task = "Put the two objects into the box."
    (output / "meta/tasks.jsonl").write_text(json.dumps({"task_index": 0, "task": task})+"\n")
    (output / "meta/episodes.jsonl").write_text(json.dumps({"episode_index": 0, "tasks": [task], "length": count})+"\n")
    (output / "meta/stats.json").write_text(json.dumps(stats))
    (output / "meta/episodes_stats.jsonl").write_text(json.dumps({"episode_index": 0, "stats": stats})+"\n")
    audit = {"origin": "simulation", "task_success": report["task_success"], "training_eligible": report["training_eligible"],
             "source_episode": str(source), "replay_directory": str(episode),
             "labels": "Recomputed from achieved simulation tool poses/fingers, not copied from commanded UMI motion",
             "tail": "Terminal future targets are repeated only where action_is_pad is true; padded slots excluded from action stats",
             "image_transforms": transforms, "videos": video_audit,
             "parquet_sha256": hashlib.sha256(parquet.read_bytes()).hexdigest(),
             "registration_status": report["registration"]["status"],
             "calibration": report["registration"], "camera_calibration": report["configuration"]["camera_calibration_status"]}
    (output / "meta/simulation_provenance.json").write_text(json.dumps(audit, indent=2))
    (output / "audit").mkdir()
    for name in ("observations.npz", "controls.npz", "replay.json"):
        shutil.copy2(episode / name, output / "audit" / name)
    # Reload the written table; this catches schema conversions and flattening.
    loaded = pq.read_table(parquet)
    for name in ("observation.state", "action", "action_is_pad"):
        if not np.array_equal(np.asarray(loaded[name].to_pylist()), columns[name]):
            raise ValueError(f"Parquet round-trip mismatch: {name}")
    (output / "README.md").write_text(
        "# X2 UMI replay — synthetic robot data\n\n"
        f"Task success: **{report['task_success']}**. Training eligible: **{report['training_eligible']}**.\n\n"
        "This is an Isaac Sim replay, not a real robot recording. Estimated frame registration, camera extrinsics, "
        "wood friction and the assumed 2 mm box floor are recorded in `meta/simulation_provenance.json`. "
        "The original native-resolution PNGs remain in the linked replay directory.\n\n"
        "The 30-D, 10 Hz state/action semantics match the UMI v2 source. Actions have 50 future steps sharing the "
        "current observation anchor. Preserve `action_is_pad` in training. Robot joint commands, achieved states, "
        "object poses and the original replay diagnostics are in `audit/`.\n")
    print(json.dumps({"dataset": str(output), "frames": count, "task_success": report["task_success"]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--source", type=Path, default=ROOT / "data/umi_replay/episode_000000")
    parser.add_argument("--allow-failure", action="store_true")
    args = parser.parse_args()
    export(args.episode, args.output, args.source, args.allow_failure)
