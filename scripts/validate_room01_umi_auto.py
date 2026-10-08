#!/usr/bin/env python3
"""Independently check recorded reference commands and automatic-layout audits."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from room01_sim.umi_layout import trajectory_hash


def main(args):
    metadata = json.loads((args.prepared / "preparation.json").read_text())
    trajectory = dict(np.load(args.prepared / "trajectory.npz"))
    assert trajectory_hash(trajectory) == metadata["trajectory_sha256"]
    summary = json.loads((args.search / "summary.json").read_text())
    errors = []
    for item in summary["trials"]:
        path = Path(item["path"])
        report = json.loads((path / "replay.json").read_text())
        assert report["trajectory_sha256"] == metadata["trajectory_sha256"]
        assert report["geometry_correction_m"] == 0 and report["rotation_correction_deg"] == 0
        assert report["time_scale"] == summary["settings"]["time_scale"]
        control = np.load(path / "controls.npz")
        expected = np.stack([np.interp(control["source_time"], trajectory["times"], trajectory["targets"][:, i]) for i in range(26)], -1)
        error = float(np.max(abs(expected-control["reference_joint_target"])))
        assert error < 1e-10
        errors.append(error)
        if report["phase"] not in ("full", "record_full", "robustness"):
            assert not report["task_success"] and not report["training_eligible"]
        observations = np.load(path / "observations.npz")
        if report["images_recorded"]:
            assert np.max(np.ptp(observations["camera_timestamps"], axis=1)) < 1e-9
            for name in report["camera_names"]:
                assert len(list((path / "images" / name).glob("*.png"))) == report["frames"]
    if summary["accepted"]:
        assert summary["robustness"]["passed"] == summary["robustness"]["total"] >= 1
        if summary.get("recorded_replay"):
            recorded = json.loads((Path(summary["recorded_replay"]) / "replay.json").read_text())
            assert all(g["passed"] for g in recorded["grasps"].values())
    else:
        assert not summary["training_eligible"]
    result = {"engineering_checks_pass": True, "accepted": summary["accepted"],
              "training_eligible": summary["training_eligible"], "trial_count": len(errors),
              "max_reference_command_error_rad": max(errors, default=0.), "reference_trajectory_sha256": metadata["trajectory_sha256"],
              "same_time_scale_all_trials": True, "no_local_trajectory_edits": True,
              "probe_not_mislabeled_as_full_success": True, "search": str(args.search.resolve())}
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True); args.report.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("prepared", type=Path)
    parser.add_argument("search", type=Path)
    parser.add_argument("--report", type=Path)
    main(parser.parse_args())
