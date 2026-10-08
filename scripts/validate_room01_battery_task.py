#!/usr/bin/env python3
"""Check battery reset stability and the cooked PhysX openings; render review views.

These are asset checks. No battery is teleported into a hole and no robotic
grasp/insertion or compliant-foam success is inferred from these checks.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, default=ROOT / "reports/room01_battery_task")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True
from room01_sim.bootstrap import launch
app = launch(args).app
code = 0
try:
    import numpy as np
    import torch
    from PIL import Image
    from pxr import Usd, UsdGeom
    from omni.physx import get_physx_scene_query_interface
    from room01_sim.runtime import RoomTaskSim, tensor
    from room01_sim.kinematics import quaternion_xyzw

    assets = ROOT / "assets/room01/battery_task"
    spec = json.loads((assets / "scene_spec.json").read_text())
    manifest = json.loads((assets / "build_manifest.json").read_text())
    runtime = RoomTaskSim(app, device=args.device, cameras=True, review_camera=True,
                          config_path=assets / "task_config.json", scene_path=assets / "room01_battery_task.usda")
    args.output.mkdir(parents=True, exist_ok=True)
    report = {"scope": "CPU mesh tests plus cooked PhysX rays and uncommanded reset stability; not a grasp/insertion trial",
              "robotic_insertion_tested": False, "foam_compliance_validated": False,
              "geometry_source": "scene_spec.json", "asset_checks_pass": False}
    report["scene_spec_sha256"] = runtime.config["scene_spec_sha256"]
    report["relocation"] = runtime.config.get("relocation")
    initial = runtime.observe(images=False)
    runtime.advance_physics(4800, render=True)
    before_final_window = runtime.observe(images=False)
    runtime.advance_physics(960, render=True)
    settled = runtime.observe(images=False)
    robot_position = tensor(runtime.robot.data.root_link_pose_w)[0, :3].cpu().numpy()
    expected_robot_position = np.asarray(runtime.config["robot_base_position"])
    report["robot_placement"] = {"expected_world_m": expected_robot_position.tolist(),
        "actual_world_m": robot_position.tolist(),
        "position_error_m": float(np.linalg.norm(robot_position-expected_robot_position)),
        "pass": bool(np.linalg.norm(robot_position-expected_robot_position) < .001)}
    object_delta = settled["object_poses_xyzw"][:, :3]-initial["object_poses_xyzw"][:, :3]
    final_window_delta = settled["object_poses_xyzw"][:, :3]-before_final_window["object_poses_xyzw"][:, :3]
    speed = np.linalg.norm(settled["object_velocities"][:, :3], axis=1)
    spin = np.linalg.norm(settled["object_velocities"][:, 3:], axis=1)
    expected_z = spec["table"]["height_m"]+spec["props"]["yellow_dims_m"][2]+spec["props"]["battery_diameter_m"]/2
    report["reset_stability"] = {
        "physics_seconds": 6.0, "object_names": list(runtime.objects),
        "initial_to_final_position_change_m": object_delta.tolist(),
        "final_one_second_position_change_m": final_window_delta.tolist(), "linear_speed_m_s": speed.tolist(),
        "angular_speed_rad_s": spin.tolist(),
        "actual_axis_height_m": settled["object_poses_xyzw"][:, 2].tolist(),
        "expected_axis_height_m": expected_z,
        "criteria": "After 5 seconds settling: drift <0.5mm over the next second, final speed <1mm/s, axis height within 1mm of foam support; initial transient is recorded separately.",
        "pass": bool(np.max(np.linalg.norm(final_window_delta, axis=1)) < .0005 and np.max(speed) < .001
                     and np.max(abs(settled["object_poses_xyzw"][:, 2]-expected_z)) < .001),
    }
    rotation = np.asarray(spec["frame"]["rotation_world_from_local"])
    origin = np.asarray(spec["frame"]["world_origin_m"])
    def ray(local, distance):
        start = rotation @ np.asarray(local)+origin
        hit = get_physx_scene_query_interface().raycast_closest(start.tolist(), [0., 0., -1.], distance)
        return {"hit": bool(hit.get("hit", False)),
                "position": [float(v) for v in hit.get("position", [])],
                "collision": str(hit.get("collision", "")), "ray_origin_local_m": list(local)}

    holes = manifest["props"]["red_foam"]["hole_top_centers_parent_m"]
    bottom_z = spec["table"]["height_m"]+spec["props"]["red_dims_m"][2]-spec["props"]["hole_depth_m"]
    red_rays = []
    for hole in holes:
        hit = ray([hole[0], hole[1], hole[2]+.006], .05)
        hit["expected_bottom_z_m"] = bottom_z
        hit["pass"] = hit["hit"] and hit["collision"].endswith("/RedFoam/Body") and abs(hit["position"][2]-bottom_z) < 1e-4
        red_rays.append(hit)
    report["red_hole_bottom_rays"] = red_rays
    red = spec["props"]["red_position_xy_m"]
    red_top_z = spec["table"]["height_m"]+spec["props"]["red_dims_m"][2]
    solid = ray([red[0], red[1]-.017, red_top_z+.006], .05)
    solid["pass"] = solid["hit"] and solid["collision"].endswith("/RedFoam/Body") and abs(solid["position"][2]-red_top_z) < 1e-4
    report["red_solid_top_ray"] = solid
    slot = manifest["table"]["slot"]["center_xy_m"]
    opening = ray([slot[0], slot[1], spec["table"]["height_m"]+.005], .04)
    opening["pass"] = not opening["hit"]
    report["table_through_slot_ray"] = opening
    surface = ray([slot[0], slot[1]-.06, spec["table"]["height_m"]+.005], .04)
    surface["pass"] = surface["hit"] and surface["collision"].endswith("/Table/Tabletop") and abs(surface["position"][2]-.740) < 1e-4
    report["table_solid_surface_ray"] = surface

    # Native robot cameras are captured together before changing the review camera.
    observation = runtime.observe(images=True)
    for name, rgb in observation["images"].items():
        Image.fromarray(rgb).save(args.output / f"{name}.png")
    report["native_cameras"] = {"count": len(observation["images"]),
        "same_physics_step": len({m["physics_step"] for m in observation["camera_metadata"].values()}) == 1,
        "nonempty": all(float(rgb.std()) > 1 for rgb in observation["images"].values())}

    review = runtime.review_camera
    robot_visual = UsdGeom.Imageable(runtime.stage.GetPrimAtPath("/World/Robot"))
    cameras = {}
    for name in ["ReviewSetup", "ReviewBatteryTable", "ReviewBatteryTop", "ReviewBatteryDetail"]:
        camera = UsdGeom.Camera(runtime.stage.GetPrimAtPath("/World/"+name))
        matrix = np.asarray(UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(camera.GetPrim())).T
        cameras[name] = (matrix.copy(), camera.GetFocalLengthAttr().Get(), camera.GetHorizontalApertureAttr().Get(), camera.GetVerticalApertureAttr().Get())
    for name, (matrix, focal, horizontal, vertical) in cameras.items():
        # Object-only inspection is marked in metadata; physics and robot state are unchanged.
        hide_robot = name != "ReviewSetup"
        robot_visual.CreateVisibilityAttr().Set("invisible" if hide_robot else "inherited")
        review.set_world_poses(positions=np.array([matrix[:3, 3]], dtype=np.float32),
                              orientations=np.array([quaternion_xyzw(matrix[:3, :3])], dtype=np.float32), convention="opengl")
        pixel_focal = 1280*focal/horizontal
        intrinsic = torch.tensor([[[pixel_focal, 0, 640.], [0, pixel_focal, 480.], [0, 0, 1.]]], device=runtime.device)
        review.set_intrinsic_matrices(intrinsic, focal_length=focal/10)
        for _ in range(12):
            runtime.sim.render()
            review.update(0., force_recompute=True)
        rgb = tensor(review.data.output["rgb"])[0].cpu().numpy()[..., :3].astype(np.uint8)
        Image.fromarray(rgb).save(args.output / (name+".png"))
    robot_visual.CreateVisibilityAttr().Set("inherited")
    report["review_images"] = {"ReviewSetup": "Robot visible", "other_views": "Robot hidden for object-only inspection; no physics changes or scene save"}
    report["asset_checks_pass"] = bool(report["reset_stability"]["pass"] and all(r["pass"] for r in red_rays)
        and solid["pass"] and opening["pass"] and surface["pass"] and report["native_cameras"]["nonempty"]
        and report["native_cameras"]["same_physics_step"] and report["robot_placement"]["pass"])
    (args.output / "validation.json").write_text(json.dumps(report, indent=2)+"\n")
    print("BATTERY_ASSET_VALIDATION", json.dumps(report), flush=True)
    code = 0 if report["asset_checks_pass"] else 2
    runtime.close()
except Exception:
    import traceback
    traceback.print_exc()
    code = 1
finally:
    app.close(exit_code=code)
