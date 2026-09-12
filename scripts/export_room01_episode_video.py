#!/usr/bin/env python3
"""Make a review video from a completed lossless three-camera episode."""
from pathlib import Path
import argparse
import json
import os
import subprocess

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("episode", type=Path)
parser.add_argument("--output", type=Path, default=ROOT / "reports/room01_sim/three_camera_grasp.mp4")
parser.add_argument("--panel-height", type=int, default=720, help="Review-only letterbox height; source PNGs retain their native size.")
args = parser.parse_args()
metadata = json.loads((args.episode / "episode.json").read_text())
if metadata["status"] != "complete":
    raise ValueError("Only completed episodes can be exported")
if args.panel_height < 2 or args.panel_height % 2:
    raise ValueError("Video panel height must be a positive even integer")
encoder = ROOT / "tools/video_encoder/root/usr/bin/ffmpeg"
env = os.environ.copy()
library = ROOT / "tools/modeling_blender_deb/root/usr/lib"
extra = ROOT / "tools/video_encoder/root/usr/lib/x86_64-linux-gnu"
env["LD_LIBRARY_PATH"] = ":".join([str(extra), str(library / "x86_64-linux-gnu"), str(library),
                                  str(library / "x86_64-linux-gnu/blas"), str(library / "x86_64-linux-gnu/lapack"), env.get("LD_LIBRARY_PATH", "")])
command = [str(encoder), "-y", "-loglevel", "warning"]
for name in metadata["camera_names"]:
    command += ["-framerate", str(metadata["fps"]), "-i", str(args.episode / "images" / name / "%06d.png")]
font = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
filters = [f"[{i}:v]scale=-2:{args.panel_height},setsar=1,drawtext=fontfile={font}:text={name}:fontcolor=white:fontsize=24:box=1:boxcolor=black@0.7:x=12:y=12[v{i}]"
           for i, name in enumerate(metadata["camera_names"])]
filters += ["[v0][v1][v2]hstack=inputs=3[out]"]
args.output.parent.mkdir(parents=True, exist_ok=True)
command += ["-filter_complex", ";".join(filters), "-map", "[out]", "-frames:v", str(metadata["frames"]),
            "-c:v", "libx264", "-preset", "fast", "-crf", "20", "-pix_fmt", "yuv420p", "-threads", "4", "-movflags", "+faststart", str(args.output)]
subprocess.run(command, check=True, env=env)
print(args.output.resolve())
