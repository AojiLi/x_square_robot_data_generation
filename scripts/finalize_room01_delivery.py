#!/usr/bin/env python3
"""Stamp a delivery only when its asset, physics, camera and episode checks pass."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports/room01_sim"
checks = {name: json.loads((REPORT / filename).read_text()) for name, filename in {
    "assets": "asset_verification.json", "physics": "physics_verification.json", "vla_io": "vla_io_verification.json",
    "episode": "episode_verification.json", "grasp": "grasp_final.json", "blender_review": "blender_review_verification.json",
    "video": "video_verification.json",
}.items()}
checks["actual_reset_first_frame"] = json.loads((ROOT / "reports/room01_wrist_reset_fix/first_frame_verification.json").read_text())
optics_report = ROOT / "reports/room01_camera_calibration/projection_reference_verification.json"
if optics_report.exists():
    checks["camera_optics"] = json.loads(optics_report.read_text())
boundary_report = ROOT / "reports/room01_wrist_fisheye_alignment/render_boundary_verification.json"
if boundary_report.exists():
    checks["fisheye_boundary"] = json.loads(boundary_report.read_text())
orientation_report = ROOT / "reports/room01_wrist_view_correction/orientation_verification.json"
if orientation_report.exists():
    checks["wrist_reference_orientation"] = json.loads(orientation_report.read_text())
failed = [name for name, result in checks.items() if result.get("result") != "pass"]
if failed:
    raise RuntimeError("Cannot finalize failed checks: "+", ".join(failed))
episode = Path(checks["grasp"]["episode"])
if str(episode) != checks["episode"]["episode"]:
    raise RuntimeError("The validated episode differs from the grasp report")
metadata = json.loads((episode / "episode.json").read_text())
config = json.loads((ROOT / "assets/room01/sim/task_config.json").read_text())
if metadata["config"] != config or metadata["status"] != "complete":
    raise RuntimeError("The delivery episode does not use the current configuration")
config_sha256 = hashlib.sha256((ROOT / "assets/room01/sim/task_config.json").read_bytes()).hexdigest()
if checks["actual_reset_first_frame"]["source_episode"] != str(episode.relative_to(ROOT)):
    raise RuntimeError("The validated first frame belongs to a different episode")
for name, report in checks.items():
    if "config_sha256" in report and report["config_sha256"] != config_sha256:
        raise RuntimeError("Stale configuration in validation report: "+name)
paths = ["assets/room01/sim/room01_manipulation.usda", "assets/room01/sim/room_environment.usdc",
         "assets/room01/sim/quanta_x2_physics.usdc", "assets/room01/sim/task_config.json",
         "assets/room01/sim/robot_frames.json", "assets/room01/sim/camera_manifest.json",
         "assets/room01/sim/room01_sim_review.blend", "reports/room01_sim/three_camera_grasp.mp4"]
files = {name: {"bytes": (ROOT / name).stat().st_size, "sha256": hashlib.sha256((ROOT / name).read_bytes()).hexdigest()} for name in paths}
for p in sorted((ROOT / "assets/room01/sim/calibration").glob("*.json")):
    files[str(p.relative_to(ROOT))] = {"bytes": p.stat().st_size, "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
manifest = {"status": "validated_fixed_workcell", "created_utc": datetime.now(timezone.utc).isoformat(),
            "scene": paths[0], "blender_review": paths[-2], "video": paths[-1], "episode": str(episode.relative_to(ROOT)),
            "checks": {name: value["result"] for name, value in checks.items()}, "files": files,
            "source_code_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((ROOT / "room01_sim").glob("*.py"))},
            "camera_calibration_status": config["camera_calibration_status"], "vla_model_trained": False,
            "real_camera_calibration_complete": False,
            "example_frames": metadata["frames"], "example_images": checks["episode"]["decoded_images"],
            "example_first_frames": {name: str((episode / "images" / name / "000000.png").relative_to(ROOT)) for name in metadata["camera_names"]},
            "grasp_stable_hold_seconds": checks["grasp"]["longest_stable_lift_s"]}
reuse_report = ROOT / "reports/room01_camera_calibration/physics_regression_scope.json"
if reuse_report.exists():
    manifest["physics_validation_reuse"] = json.loads(reuse_report.read_text())
(REPORT / "delivery_manifest.json").write_text(json.dumps(manifest, indent=2))
build_path = REPORT / "build_manifest.json"
build = json.loads(build_path.read_text())
build["runtime_validation"] = "pass"
build["delivery_manifest"] = "reports/room01_sim/delivery_manifest.json"
build_path.write_text(json.dumps(build, indent=2))
print(json.dumps({key: value for key, value in manifest.items() if key not in {"files", "source_code_sha256"}}, indent=2))
