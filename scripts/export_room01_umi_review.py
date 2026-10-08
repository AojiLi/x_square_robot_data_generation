#!/usr/bin/env python3
"""Make timestamped review media from an actual recorded UMI simulation run."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from export_room01_umi_dataset import encoder_environment


def main(args):
    report = json.loads((args.episode / "replay.json").read_text())
    if not report["images_recorded"] or report["status"] != "complete":
        raise ValueError("Review media requires completed native images")
    args.output.mkdir(parents=True, exist_ok=True)
    encoder, env = encoder_environment()
    font = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    common = [encoder, "-y", "-loglevel", "error"]
    encode = ["-frames:v", str(report["frames"]), "-c:v", "libx264", "-preset", "fast", "-crf", "20",
              "-pix_fmt", "yuv420p", "-threads", "4", "-movflags", "+faststart"]
    command, filters = common.copy(), []
    cameras = ["head", "left_wrist", "right_wrist"]
    for index, camera in enumerate(cameras):
        command += ["-framerate", "10", "-i", str(args.episode / "images" / camera / "%06d.png")]
        filters.append(f"[{index}:v]scale=640:480,setsar=1,drawtext=fontfile={font}:text=X2_{camera}:fontcolor=white:fontsize=22:box=1:boxcolor=black@0.65:x=12:y=12[v{index}]")
    filters.append("[v0][v1][v2]hstack=inputs=3[out]")
    subprocess.run(command+["-filter_complex", ";".join(filters), "-map", "[out]"]+encode+[str(args.output / "three_camera_replay.mp4")],
                   check=True, env=env)
    subprocess.run(common+["-i", str(args.output / "three_camera_replay.mp4"), "-frames:v", "1", "-update", "1", str(args.output / "three_views.jpg")],
                   check=True, env=env)
    if (args.episode / "review").exists():
        subprocess.run(common+["-framerate", "10", "-i", str(args.episode / "review/%06d.png")]+encode+[str(args.output / "external_replay.mp4")],
                       check=True, env=env)
    status = "PASS" if report["task_success"] else "GRASP_PROBE" if report.get("phase") in ("grasp_probe", "record_probe", "free_motion") else "FAILED"
    scale = report["time_scale"]
    comparison_filter = (
        f"[0:v]setpts={scale}*PTS,fps=10,tpad=stop_mode=clone:stop_duration=2,scale=640:480,setsar=1,"
        f"drawtext=fontfile={font}:text=UMI_reference__time_scale_{scale}:fontcolor=white:fontsize=19:box=1:boxcolor=black@0.65:x=10:y=10[a];"
        f"[1:v]scale=640:480,setsar=1,drawtext=fontfile={font}:text=X2_simulation__{status}:fontcolor=white:fontsize=19:box=1:boxcolor=black@0.65:x=10:y=10[b];"
        "[a][b]hstack=inputs=2[out]"
    )
    subprocess.run(common+["-i", str(args.source / "videos/head_rgb.mp4"), "-framerate", "10", "-i", str(args.episode / "images/head/%06d.png"),
                           "-filter_complex", comparison_filter, "-map", "[out]"]+encode+[str(args.output / "source_vs_sim.mp4")], check=True, env=env)
    subprocess.run(common+["-i", str(args.output / "source_vs_sim.mp4"), "-frames:v", "1", "-update", "1", str(args.output / "initial_comparison.jpg")],
                   check=True, env=env)
    print(args.output.resolve())


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    parser.add_argument("--output", type=Path, default=ROOT / "reports/room01_umi_replay")
    parser.add_argument("--source", type=Path, default=ROOT / "data/umi_replay/episode_000000")
    main(parser.parse_args())
