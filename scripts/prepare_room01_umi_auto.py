#!/usr/bin/env python3
"""Freeze a source trajectory and automatically identify task stages/assets."""
import argparse
import copy
import json
from pathlib import Path
import subprocess
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "scripts"))
from room01_sim.umi import load_episode
from room01_sim.umi_layout import CATALOG, BOX, detect_events, identify_objects, freeze_trajectory, trajectory_hash, parked_layout
from export_room01_umi_dataset import encoder_environment
from prepare_room01_umi_replay import author_scene


def prepare(source, output, calibration_path, object_override=None):
    output.mkdir(parents=True, exist_ok=True)
    data = load_episode(source)
    config = json.loads((ROOT / "assets/room01/sim/task_config.json").read_text())
    calibration = json.loads(calibration_path.read_text())
    events = detect_events(data["times"], data["fingers_deg"])
    if len(events) != 2 or len({e['side'] for e in events}) != 2:
        raise ValueError("event_topology_unsupported: this task adapter requires one grasp/release per hand")
    if object_override:
        identity = {"objects_by_side": dict(zip(("left", "right"), object_override)), "method": "explicit task asset override"}
    else:
        encoder, env = encoder_environment()
        raw = subprocess.run([encoder, "-loglevel", "error", "-i", str(source / "videos/head_rgb.mp4"),
                              "-frames:v", "1", "-f", "rawvideo", "-pix_fmt", "rgb24", "pipe:1"],
                             stdout=subprocess.PIPE, check=True, env=env).stdout
        identity = identify_objects(np.frombuffer(raw, np.uint8).reshape(480, 640, 3))
    for event in events:
        event["asset"] = identity["objects_by_side"][event["side"]]
    if len({e["asset"] for e in events}) != 2 or any(e["asset"] not in CATALOG for e in events):
        raise ValueError("identity_unresolved: task requires two distinct catalog assets")
    trajectory = freeze_trajectory(data, config, calibration)
    reference_hash = trajectory_hash(trajectory)
    np.savez_compressed(output / "trajectory.npz", **trajectory)
    # The initial stage is only a reusable actor pool. Candidate poses are set
    # together at reset, before simulated episode time zero.
    placements = parked_layout({events[0]["asset"]: [.83, -2.10, .75, 0, 0, 0, 1],
                                events[1]["asset"]: [.83, -1.80, .75, 0, 0, 0, 1]})
    specs = {}
    for name, spec in {**CATALOG, "BlackBox": BOX}.items():
        specs[name] = {**copy.deepcopy(spec), "prim_path": "/World/Task/"+name,
                       "position": placements[name][:3], "orientation_xyzw": placements[name][3:]}
    config.update(task="umi_fixed_trajectory_auto_layout", language_instruction="Put the two objects into the box.",
                  task_objects=specs, target_objects=[e["asset"] for e in events], primary_object=events[0]["asset"],
                  cube_size=CATALOG[events[0]["asset"]]["size"], cube_position=placements[events[0]["asset"]][:3])
    (output / "task_config.json").write_text(json.dumps(config, indent=2))
    author_scene(output / "scene.usda", config)
    metadata = {"source_directory": str(source.resolve()), "source_episode_index": data["episode_index"],
                "source_audit": data["audit"], "events": events, "identity": identity,
                "calibration": calibration, "trajectory_sha256": reference_hash,
                "limit_rounding_max_deg": float(trajectory["limit_rounding_max_deg"]),
                "trajectory_rule": "No local pose corrections, per-candidate retiming, finger edits or task-object writes after reset"}
    (output / "preparation.json").write_text(json.dumps(metadata, indent=2))
    (output / "rejection.json").unlink(missing_ok=True)
    print(json.dumps({"prepared": str(output), "source_episode": data["episode_index"], "events": events,
                      "objects": identity["objects_by_side"], "trajectory_sha256": reference_hash}), flush=True)
    return metadata


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--calibration", type=Path, default=ROOT / "assets/room01/umi_auto/calibration.json")
    parser.add_argument("--objects", help="Optional left,right catalog IDs; affects identity only, never positions")
    args = parser.parse_args()
    try:
        prepare(args.source, args.output, args.calibration, args.objects.split(",") if args.objects else None)
    except (ValueError, OSError) as error:
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "rejection.json").write_text(json.dumps({"status": "rejected_before_physics", "reason": str(error)}, indent=2))
        raise
