"""Inspect the saved Blender review without changing it (run inside Blender)."""
from pathlib import Path
import json
import bpy

ROOT = Path(__file__).resolve().parents[1]
config = json.loads((ROOT / "assets/room01/sim/task_config.json").read_text())
scene = bpy.context.scene
collisions = bpy.data.collections["90 Simulation collision proxies"]
checks = {
    "383_passive_colliders": sum(bool(o.rigid_body) and o.rigid_body.type == "PASSIVE" for o in scene.objects) == 383,
    "one_dynamic_object": sum(bool(o.rigid_body) and o.rigid_body.type == "ACTIVE" for o in scene.objects) == 1,
    "meter_units": scene.unit_settings.scale_length == 1.,
    "collision_collection_hidden": collisions.hide_viewport,
    "collision_meshes_hidden_from_render": all(o.hide_render for o in collisions.objects),
    "calibration_status": scene.get("camera_calibration_status") == config["camera_calibration_status"],
    "reset_pose_profile": scene.get("reset_pose_profile") == config["reset_pose"]["profile"],
    "reset_joint_positions": json.loads(scene.get("joint_home_rad_json", "{}")) == config["joint_home_rad"],
}
for name, spec in config["cameras"].items():
    camera = bpy.data.objects.get("Sim_RGB_"+name)
    checks[name+"_camera_present"] = camera is not None and camera.type == "CAMERA"
    if camera:
        expected = 20*spec["resolution"][0]/spec["fx"]
        checks[name+"_review_aperture"] = abs(camera.data.sensor_width-expected) < 1e-5
        checks[name+"_distortion_limit_disclosed"] = "exact OpenCV lens model" in camera.get("optics_note", "")
report = {"result": "pass" if all(checks.values()) else "fail", "checks": checks,
          "objects": len(scene.objects), "calibration_status": config["camera_calibration_status"],
          "scope": "Blender geometry/camera review; exact fisheye rendering is checked in Isaac Sim."}
(ROOT / "reports/room01_sim/blender_review_verification.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
if report["result"] != "pass":
    raise RuntimeError("Blender review verification failed")
