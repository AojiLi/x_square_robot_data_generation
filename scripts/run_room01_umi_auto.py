#!/usr/bin/env python3
"""Budgeted automatic initial-layout search; all trials share one trajectory."""
import argparse
import json
import hashlib
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("prepared", type=Path)
parser.add_argument("--grasp-trials", type=int, default=4)
parser.add_argument("--layout-trials", type=int, default=4)
parser.add_argument("--robustness-trials", type=int, default=2)
parser.add_argument("--wall-seconds", type=float, default=1200.)
parser.add_argument("--record-wall-seconds", type=float, default=600., help="Separate bounded allowance for the final recorded replay")
parser.add_argument("--time-scale", type=float, default=2.)
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--record-best", action=argparse.BooleanOptionalAction, default=True)
parser.add_argument("--fresh-reference", action="store_true", help="Ignore a previously completed matching free-motion trace")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if not 1 <= args.time_scale <= 2 or min(args.grasp_trials, args.layout_trials, args.robustness_trials) < 1 or args.wall_seconds <= 0 or args.record_wall_seconds <= 0:
    parser.error("Positive budgets and a shared 1–2 time scale are required")
args.enable_cameras = args.record_best
from room01_sim.bootstrap import launch
app = launch(args).app
exit_code = 0
run_dir = args.prepared / ("search_"+time.strftime("%Y%m%d_%H%M%S", time.gmtime()))
run_dir.mkdir(parents=True, exist_ok=False)
summary = {"status": "running", "settings": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()
                                            if k in ("grasp_trials", "layout_trials", "robustness_trials", "wall_seconds", "record_wall_seconds", "time_scale", "seed", "record_best")},
           "trials": [], "accepted": False, "training_eligible": False}
summary_path = run_dir / "summary.json"

try:
    import copy
    import numpy as np
    from scipy.spatial.transform import Rotation
    from room01_sim.runtime import RoomTaskSim
    from room01_sim.umi import sample_poses, source_fingers_from_q
    from room01_sim.umi_layout import CATALOG, BOX, HandGeometry, grasp_candidates, parked_layout, box_candidates, trajectory_hash, reference_fingerprint, cube_corners, stable_warmstarts
    from room01_sim.umi_auto_runtime import run_trial
    trajectory = dict(np.load(args.prepared / "trajectory.npz"))
    for array in trajectory.values():
        array.setflags(write=False)
    metadata = json.loads((args.prepared / "preparation.json").read_text())
    events = metadata["events"]
    summary.update(source_episode_index=metadata["source_episode_index"], trajectory_sha256=metadata["trajectory_sha256"],
                   source_directory=metadata["source_directory"], events=events, identity=metadata["identity"])
    summary["algorithm_sha256"] = hashlib.sha256((ROOT / "room01_sim/umi_layout.py").read_bytes()).hexdigest()
    runtime = RoomTaskSim(app, device=args.device, cameras=args.record_best,
                          config_path=args.prepared / "task_config.json", scene_path=args.prepared / "scene.usda")
    metadata["reference_fingerprint"] = reference_fingerprint(runtime.config, metadata["trajectory_sha256"], args.time_scale)
    started = time.monotonic(); deadline = started+args.wall_seconds
    rng = np.random.default_rng(args.seed)

    def persist():
        summary["elapsed_seconds"] = time.monotonic()-started
        summary_path.write_text(json.dumps(summary, indent=2))

    def trial(poses, phase, record=False, early=True):
        index = len(summary["trials"])
        folder = run_dir / f"trial_{index:03d}_{phase}"
        cached = None
        if phase in ("grasp_probe", "full") and not record:
            patterns = ("grasp_probe", "record_probe") if phase == "grasp_probe" else ("full", "record_full")
            history = sorted([p for kind in patterns for p in args.prepared.glob("search_*/trial_*_"+kind+"/replay.json")], reverse=True)
            matches = []
            for path in history:
                old = json.loads(path.read_text())
                if old.get("status") == "complete" and old.get("reference_fingerprint") == metadata["reference_fingerprint"] and old.get("initial_object_poses") == poses:
                    failed = not all(g["passed"] for g in old.get("grasps", {}).values()) if phase == "grasp_probe" else not old["task_success"]
                    matches.append((not failed, path))
            if matches:
                cached = sorted(matches, key=lambda item: item[0])[0][1]
        if cached:
            report = json.loads(cached.read_text()); folder = cached.parent
            trace = dict(np.load(folder / "physics_trace.npz")); trace["object_names"] = report["object_names"]
        else:
            trial_deadline = time.monotonic()+args.record_wall_seconds if record else deadline
            report, trace = run_trial(runtime, trajectory, metadata, poses, folder, phase=phase,
                                      time_scale=args.time_scale, record=record, deadline=trial_deadline,
                                      early_exit=early)
        if phase != "free_motion":
            sanity = {}
            for event in events:
                name = event["asset"]; i = trace["object_names"].index(name)
                first = trace["object_poses"][0, i]
                bottom = cube_corners(first, CATALOG[name]["size"][0])[:, 2].min()
                velocity = trace["object_velocities"][0, i]
                valid = abs(bottom-runtime.config["tabletop_z"]) < .002 and np.linalg.norm(first[:2]-poses[name][:2]) < .005 and np.linalg.norm(velocity[:3]) < .025
                sanity[name] = bool(valid)
                if not valid:
                    report["grasps"][name]["passed"] = False
            report["initial_state_sanity"] = sanity
            if not all(sanity.values()):
                report.update(task_success=False, training_eligible=False, reason="initial_state_not_stable_on_table")
            (folder / "replay.json").write_text(json.dumps(report, indent=2))
        summary["trials"].append({"path": str(folder.resolve()), "phase": phase, "reason": report["reason"],
                                  "task_success": report["task_success"], "grasps": report["grasps"],
                                  "trajectory_sha256": report["trajectory_sha256"], "reference_hash_verified": report["reference_hash_verified"]})
        summary["trials"][-1]["reused_physics_result"] = bool(cached)
        persist(); return report, trace, folder

    # Measure reproducible unloaded servo motion once. This improves geometric
    # proposals without editing the reference to compensate for each object.
    free = {"BlackBox": [1.20, -2.39, .774, 0, 0, 0, 1]}
    for name, (dx, dy) in zip(CATALOG, [(-.03, -.03), (.03, -.03), (-.03, .03), (.03, .03)]):
        free[name] = [1.20+dx, -2.39+dy, .725+CATALOG[name]["size"][2]/2+.0002, 0, 0, 0, 1]
    cached_reference = None
    if not args.fresh_reference:
        for path in sorted(args.prepared.glob("search_*/trial_*_free_motion/replay.json"), reverse=True):
            candidate = json.loads(path.read_text())
            if candidate.get("status") == "complete" and candidate.get("reference_fingerprint") == metadata["reference_fingerprint"]:
                cached_reference = path; break
    if cached_reference:
        free_report = json.loads(cached_reference.read_text())
        free_trace = dict(np.load(cached_reference.parent / "physics_trace.npz"))
        free_trace["object_names"] = free_report["object_names"]
        summary["free_motion_cache"] = {"path": str(cached_reference.parent.resolve()),
                                         "sha256": hashlib.sha256((cached_reference.parent / "physics_trace.npz").read_bytes()).hexdigest()}
        persist()
    else:
        free_report, free_trace, _ = trial(free, "free_motion", early=False)
    if free_report["status"] != "complete":
        raise TimeoutError("wall_time_budget_exhausted during free-motion trace")
    unique_times, unique_indices = np.unique(free_trace["source_time"], return_index=True)
    proposal_trajectory = {"times": unique_times, "hand_targets": free_trace["hand_poses"][unique_indices]}
    fingers = source_fingers_from_q(free_trace["joint_positions"])[unique_indices]
    source_times = trajectory["source_times"]
    proposal_data = {"times": source_times, "fingers_deg": np.stack([
        np.interp(source_times, unique_times, fingers[:, s, j]) for s in range(2) for j in range(6)], -1).reshape(-1, 2, 6)}
    geometry = HandGeometry(); candidate_sets = {}
    for event in events:
        candidates = grasp_candidates(proposal_data, proposal_trajectory, event, event["asset"], geometry)
        candidate_sets[event["asset"]] = candidates
        if not candidates:
            raise ValueError("no_geometric_grasp_region:"+event["asset"])
    (run_dir / "grasp_candidates.json").write_text(json.dumps(candidate_sets, indent=2))
    current = {name: 0 for name in candidate_sets}
    history = [(path, json.loads(path.read_text())) for path in sorted(args.prepared.glob("search_*/trial_*/replay.json"), reverse=True)]
    warmstarts = stable_warmstarts(history, metadata["reference_fingerprint"])
    best_probe = None; best_score = -1; full_poses = None; selected = None
    for attempt in range(args.grasp_trials):
        if time.monotonic() >= deadline:
            break
        from_warmstart = bool(warmstarts)
        if from_warmstart:
            warm = warmstarts.pop(0); poses = warm["poses"]
            active = {name: poses[name] for name in current}
            summary.setdefault("warmstarts", []).append(warm["source"])
        else:
            active = {name: candidate_sets[name][min(index, len(candidate_sets[name])-1)]["pose_xyzw"] for name, index in current.items()}
            poses = parked_layout(active)
        report, trace, folder = trial(poses, "grasp_probe")
        score = sum(g["passed"] for g in report["grasps"].values())+.1*sum(g["max_lift_m"] for g in report["grasps"].values())
        if score > best_score:
            best_probe = (poses, report, trace, folder); best_score = score
        if all(g["passed"] for g in report["grasps"].values()):
            full_poses, screening = box_candidates(trace, events, active, limit=max(12, args.layout_trials))
            summary["box_screening"] = screening
            summary.setdefault("box_screening_attempts", []).append({"probe": str(folder), **screening})
            (run_dir / "box_candidates.json").write_text(json.dumps(full_poses, indent=2))
            if full_poses:
                break
            # Other verified grasp placements can change the carried pose and
            # create a feasible common release region, without changing motion.
            if not from_warmstart:
                for name in current:
                    current[name] += 1
        else:
            for name, metrics in report["grasps"].items():
                if metrics["evaluated"] and not metrics["passed"] and not from_warmstart:
                    current[name] += 1
        persist()
    summary["grasp_verified"] = bool(best_probe and all(g["passed"] for g in best_probe[1]["grasps"].values()))
    if full_poses:
        for layout in full_poses[:args.layout_trials]:
            if time.monotonic() >= deadline:
                break
            report, trace, folder = trial(layout["poses"], "full", early=False)
            if not report["task_success"]:
                continue
            robust = []
            for _ in range(args.robustness_trials):
                perturbed = copy.deepcopy(layout["poses"])
                shift = rng.uniform(-.002, .002, 2)
                for name in perturbed:
                    if name not in runtime.config["target_objects"]:
                        perturbed[name][:2] = (np.array(perturbed[name][:2])+shift).tolist()
                    else:
                        perturbed[name][:2] = (np.array(perturbed[name][:2])+rng.uniform(-.002, .002, 2)).tolist()
                check, _, _ = trial(perturbed, "robustness", early=False)
                robust.append(check["task_success"])
            summary["robustness"] = {"passed": int(sum(robust)), "total": len(robust), "translation_range_m": .002}
            if all(robust) and len(robust) == args.robustness_trials:
                selected = layout["poses"]; break
    if selected:
        summary["accepted"] = True
        summary["accepted_layout"] = selected
        (run_dir / "accepted_layout.json").write_text(json.dumps({"poses": selected, "trajectory_sha256": metadata["trajectory_sha256"],
                                                                  "reference_fingerprint": metadata["reference_fingerprint"],
                                                                  "time_scale": args.time_scale, "source_episode_index": metadata["source_episode_index"],
                                                                  "robustness": summary["robustness"]}, indent=2))
    summary["status"] = "accepted" if selected else "budget_exhausted" if time.monotonic() >= deadline else "rejected"
    if not selected:
        summary["reason"] = summary.get("box_screening", {}).get("reason", "grasp_search_budget_exhausted")
    # Render only a verified winner or the best explicitly labeled probe.
    if args.record_best and (selected or best_probe):
        poses = selected or best_probe[0]
        phase = "record_full" if selected else "record_probe"
        report, _, folder = trial(poses, phase, record=True, early=False)
        report["training_eligible"] = bool(selected and report["task_success"])
        (folder / "replay.json").write_text(json.dumps(report, indent=2))
        summary["recorded_replay"] = str(folder.resolve())
        summary["training_eligible"] = report["training_eligible"]
        summary["recorded_grasps_passed"] = all(g["passed"] for g in report["grasps"].values())
        if summary["grasp_verified"] and not summary["recorded_grasps_passed"]:
            summary.setdefault("additional_rejection_reasons", []).append("grasp_not_reproducible")
        if selected and not report["task_success"]:
            summary.update(accepted=False, status="rejected", reason="recorded_replay_not_reproducible")
    summary["trajectory_unchanged"] = trajectory_hash(trajectory) == metadata["trajectory_sha256"]
    persist(); runtime.close()
    exit_code = 0 if summary["accepted"] else 2
except KeyboardInterrupt:
    summary.update(status="interrupted", reason="interrupted", accepted=False, training_eligible=False)
    summary_path.write_text(json.dumps(summary, indent=2)); exit_code = 130
except TimeoutError as error:
    summary.update(status="budget_exhausted", reason=str(error), accepted=False, training_eligible=False)
    summary_path.write_text(json.dumps(summary, indent=2)); exit_code = 2
except ValueError as error:
    status = "rejected" if str(error).startswith("no_geometric_grasp_region:") else "error"
    summary.update(status=status, reason=str(error), accepted=False, training_eligible=False)
    summary_path.write_text(json.dumps(summary, indent=2)); exit_code = 2 if status == "rejected" else 1
except Exception as error:
    import traceback
    traceback.print_exc()
    summary.update(status="error", reason=str(error), accepted=False, training_eligible=False)
    summary_path.write_text(json.dumps(summary, indent=2)); exit_code = 1
finally:
    print("AUTO_SEARCH_COMPLETE", json.dumps({"summary": str(summary_path.resolve()), "status": summary["status"], "accepted": summary["accepted"]}), flush=True)
    app.close(exit_code=exit_code)
