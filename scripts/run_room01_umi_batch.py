#!/usr/bin/env python3
"""Sequential, resumable GPU batch driver for fixed-trajectory scene synthesis."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from room01_sim.umi import verify_source_package


def main(args):
    args.source_root, args.output = args.source_root.resolve(), args.output.resolve()
    selected = sorted(int(p.name.split("_")[-1]) for p in args.source_root.glob("episode_*")) if args.episodes == "all" else [int(i) for i in args.episodes.split(",")]
    args.output.mkdir(parents=True, exist_ok=True)
    manifest_path = args.output / "batch.json"
    manifest = json.loads(manifest_path.read_text()) if args.resume and manifest_path.exists() else {"schema_version": 1, "episodes": {}}
    settings = {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items() if k not in ("resume", "prepare_only")}
    code_hash = hashlib.sha256()
    for filename in ("room01_sim/umi_layout.py", "room01_sim/umi_auto_runtime.py", "scripts/run_room01_umi_auto.py", "assets/room01/umi_auto/calibration.json"):
        code_hash.update((ROOT / filename).read_bytes())
    settings["algorithm_sha256"] = code_hash.hexdigest()
    settings_hash = hashlib.sha256(json.dumps(settings, sort_keys=True).encode()).hexdigest()
    manifest.update(settings=settings, status="running", requested_episodes=selected)

    def save():
        manifest["updated_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        tmp = manifest_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(manifest, indent=2)); tmp.replace(manifest_path)

    for index in selected:
        source = args.source_root / f"episode_{index:06d}"; prepared = args.output / f"episode_{index:06d}"
        source_error = None
        try:
            source_hash = verify_source_package(source)
        except (ValueError, OSError) as error:
            source_hash, source_error = None, str(error)
        previous = manifest["episodes"].get(str(index), {})
        if args.resume and previous.get("settings_hash") == settings_hash and previous.get("source_hash") == source_hash and previous.get("status") in ("accepted", "rejected", "rejected_before_physics"):
            print("BATCH_RESUME_SKIP", index, previous["status"], flush=True); continue
        prepared.mkdir(parents=True, exist_ok=True)
        entry = {"status": "preparing", "source_hash": source_hash, "settings_hash": settings_hash, "path": str(prepared.resolve())}
        manifest["episodes"][str(index)] = entry; save()
        if source_error:
            entry.update(status="rejected_before_physics", reason=source_error)
            save(); print("BATCH_REJECTED", index, source_error, flush=True); continue
        with (prepared / "prepare.log").open("w") as log:
            result = subprocess.run([sys.executable, str(ROOT / "scripts/prepare_room01_umi_auto.py"), str(source), str(prepared)], stdout=log, stderr=subprocess.STDOUT)
        if result.returncode:
            entry["status"] = "rejected_before_physics"
            rejection = prepared / "rejection.json"
            entry["reason"] = json.loads(rejection.read_text())["reason"] if rejection.exists() else "preparation_error: see prepare.log"
            save(); print("BATCH_REJECTED", index, entry["reason"], flush=True); continue
        entry["status"] = "prepared"; save()
        if args.prepare_only:
            continue
        command = ["bash", "-c", 'source "$1" && shift && exec python "$@"', "umi-batch",
                   str(ROOT / "scripts/isaac_env.sh"), str(ROOT / "scripts/run_room01_umi_auto.py"), str(prepared),
                   "--headless", "--grasp-trials", str(args.grasp_trials), "--layout-trials", str(args.layout_trials),
                   "--robustness-trials", str(args.robustness_trials), "--wall-seconds", str(args.wall_seconds),
                   "--record-wall-seconds", str(args.record_wall_seconds),
                   "--time-scale", str(args.time_scale), "--seed", str(args.seed+index)]
        if not args.record_best:
            command.append("--no-record-best")
        entry["status"] = "searching"; save()
        with (prepared / "physics.log").open("w") as log:
            result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
        summaries = sorted(prepared.glob("search_*/summary.json"))
        if not summaries:
            entry.update(status="error", reason="physics_process_failed_before_summary", exit_code=result.returncode)
        else:
            summary = json.loads(summaries[-1].read_text())
            entry.update(status=summary["status"], summary=str(summaries[-1].resolve()),
                         reason=summary.get("reason"), accepted=summary["accepted"],
                         training_eligible=summary["training_eligible"], trajectory_sha256=summary["trajectory_sha256"],
                         recorded_grasps_passed=summary.get("recorded_grasps_passed"),
                         additional_rejection_reasons=summary.get("additional_rejection_reasons", []),
                         recorded_replay=summary.get("recorded_replay"), trials=len(summary["trials"]))
            if args.export_successes and entry["training_eligible"]:
                dataset = prepared / ("dataset_"+summaries[-1].parent.name)
                with (prepared / "export.log").open("w") as log:
                    exported = subprocess.run([sys.executable, str(ROOT / "scripts/export_room01_umi_dataset.py"),
                                               entry["recorded_replay"], str(dataset), "--source", str(source)], stdout=log, stderr=subprocess.STDOUT)
                entry["dataset"] = str(dataset.resolve()) if exported.returncode == 0 else None
                entry["export_status"] = "complete" if exported.returncode == 0 else "error"
        save(); print("BATCH_EPISODE_COMPLETE", index, entry["status"], flush=True)
    records = [manifest["episodes"][str(i)] for i in selected]
    manifest["error_count"] = sum(e["status"] == "error" or e.get("export_status") == "error" for e in records)
    manifest["status"] = "prepared" if args.prepare_only else "completed_with_errors" if manifest["error_count"] else "complete"
    manifest["accepted_count"] = sum(bool(e.get("accepted")) for e in records)
    manifest["rejected_count"] = sum(e["status"] in ("rejected", "rejected_before_physics", "budget_exhausted") for e in records)
    save(); print("BATCH_COMPLETE", manifest_path.resolve(), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path, default=ROOT / "data/umi_replay")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/room01/umi_auto_batch")
    parser.add_argument("--episodes", default="0,20,84", help="Comma-separated IDs, or all cached source episodes")
    parser.add_argument("--grasp-trials", type=int, default=4)
    parser.add_argument("--layout-trials", type=int, default=4)
    parser.add_argument("--robustness-trials", type=int, default=2)
    parser.add_argument("--wall-seconds", type=float, default=1200.)
    parser.add_argument("--record-wall-seconds", type=float, default=600.)
    parser.add_argument("--time-scale", type=float, default=2.)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--record-best", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--export-successes", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--prepare-only", action="store_true")
    main(parser.parse_args())
