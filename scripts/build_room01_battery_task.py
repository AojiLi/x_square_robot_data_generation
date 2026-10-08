#!/usr/bin/env python3
"""Build the measured battery workcell as a layer over the original room.

Run with .venv-usd/bin/python. The original cube and UMI scenes are not rewritten.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import sys

import numpy as np
from pxr import Gf, Usd, UsdGeom

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from room01_sim.battery_assets import author_props
from room01_sim.battery_table import author_table
from room01_sim.build_scene import ident, look_at, transform
from room01_sim.kinematics import quaternion_matrix, quaternion_xyzw

ASSETS = ROOT / "assets/room01/battery_task"


def build(spec_path=ASSETS / "scene_spec.json"):
    spec = json.loads(Path(spec_path).read_text())
    path = ASSETS / "room01_battery_task.usda"
    stage = Usd.Stage.CreateNew(str(path))
    stage.GetRootLayer().subLayerPaths = ["../sim/room01_manipulation.usda"]
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, "Z")

    removed = ["/World/Task/RedCube", "/World/Room/Visual/_08_White_writing_table"]
    removed.extend(str(p.GetPath()) for p in stage.GetPrimAtPath("/World/Room/Collisions").GetChildren()
                   if p.GetName().startswith("Writing_table_"))
    for old in removed:
        stage.OverridePrim(old).SetActive(False)

    # Small loose accessories in the old model conflict with the new table/base.
    # Keep the desk, monitor and PC fixed; the supplied photos show the powerstrip
    # under the table and do not show the old headset on top of the tower.
    clutter_changes = []
    geometry = json.loads((ROOT / "assets/room01/sim/geometry_manifest.json").read_text())
    for record in geometry["objects"]:
        name = record["name"]
        if any(name.startswith(prefix) for prefix in spec.get("removed_background_object_prefixes", [])):
            for path_to_remove in ["/World/Room/Visual/"+ident(record["collections"][0])+"/"+ident(name),
                                   "/World/Room/Collisions/"+ident(name)]:
                if stage.GetPrimAtPath(path_to_remove):
                    stage.OverridePrim(path_to_remove).SetActive(False)
                    removed.append(path_to_remove)
            continue
        if not name.startswith(("Headset_", "Powerstrip_")):
            continue
        paths = ["/World/Room/Visual/"+ident(record["collections"][0])+"/"+ident(name),
                 "/World/Room/Collisions/"+ident(name)]
        for old in paths:
            prim = stage.GetPrimAtPath(old)
            if not prim:
                continue
            if name.startswith("Headset_"):
                stage.OverridePrim(old).SetActive(False)
                clutter_changes.append({"path": old, "action": "hide unobserved old accessory intersecting relocated tabletop"})
            else:
                current = np.asarray(UsdGeom.Xformable(prim).GetLocalTransformation()).T
                current[:3, 3] += np.asarray(spec["relocation"]["powerstrip_translation_world_m"])
                transform(prim, current)
                clutter_changes.append({"path": old, "action": "move powerstrip under desk; photo-based estimate"})

    frame = spec["frame"]
    world_from_local = np.eye(4)
    world_from_local[:3, :3] = frame["rotation_world_from_local"]
    world_from_local[:3, 3] = frame["world_origin_m"]
    root = UsdGeom.Xform.Define(stage, "/World/BatteryWorkcell")
    transform(root.GetPrim(), world_from_local)
    table = author_table(stage, "/World/BatteryWorkcell/Table", spec["table"])
    props = author_props(stage, "/World/BatteryWorkcell/Props", spec["props"])
    root.GetPrim().SetCustomData({
        "task": spec["task"],
        "measurement_source": "User photos and dimensions, 2026-09-16",
        "foam_model": "Rigid geometry approximation; no calibrated deformation or insertion force",
        "hole_diameter_m": spec["props"]["hole_diameter_m"],
        "hole_depth_m": spec["props"]["hole_depth_m"],
        "nominal_diametral_clearance_m": spec["props"]["hole_diameter_m"]-spec["props"]["battery_diameter_m"],
    })
    def world(point):
        return (world_from_local @ np.r_[point, 1.])[:3].tolist()

    # Keep the established robot/hand-camera calibration; the photos contain no robot.
    config = copy.deepcopy(json.loads((ROOT / "assets/room01/sim/task_config.json").read_text()))
    # User correction: the computer workstation stays fixed. Robot and white
    # table move together into that area, preserving their relative transform.
    translation = np.asarray(spec["relocation"]["translation_from_original_world_m"])
    old_robot = np.asarray(UsdGeom.Xformable(stage.GetPrimAtPath("/World/Robot")).GetLocalTransformation()).T
    relocated_robot = old_robot.copy()
    relocated_robot[:3, 3] += translation
    transform(stage.GetPrimAtPath("/World/Robot"), relocated_robot)
    config["robot_base_position"] = (np.asarray(config["robot_base_position"])+translation).tolist()
    config["relocation"] = spec["relocation"]
    config.update(task=spec["task"],
                  language_instruction="Pick up the batteries from the yellow foam blocks and insert them into the holes in the red foam fixture.",
                  tabletop_z=spec["table"]["height_m"],
                  reward_model="unconfigured_battery_insertion",
                  task_success_validated=False,
                  scene_spec=os.path.relpath(spec_path, ROOT),
                  scene_spec_sha256=hashlib.sha256(Path(spec_path).read_bytes()).hexdigest())
    config["task_objects"] = {}
    for battery in props["batteries"]:
        local_q = battery["orientation_local_xyzw"]
        position = world(battery["position_local_m"])
        orientation = quaternion_xyzw(world_from_local[:3, :3] @ quaternion_matrix(local_q)).tolist()
        config["task_objects"][battery["name"]] = {
            "prim_path": battery["prim_path"], "position": position,
            "orientation_xyzw": orientation, "mass_kg": spec["props"]["battery_mass_kg"],
            "diameter_m": spec["props"]["battery_diameter_m"], "length_m": spec["props"]["battery_length_m"],
        }
    config["primary_object"] = next(iter(config["task_objects"]))
    primary = config["task_objects"][config["primary_object"]]
    # Legacy observation aliases remain loadable; the cube reward is explicitly disabled.
    config["cube_position"] = primary["position"]
    config["cube_mass_kg"] = primary["mass_kg"]
    config["cube_size"] = [spec["props"]["battery_length_m"], spec["props"]["battery_diameter_m"], spec["props"]["battery_diameter_m"]]
    config["object_spawn_xy_range"] = [[v, v] for v in primary["position"][:2]]
    config["parameter_status"] = {
        "geometry": "User measurements for table, foam, battery and hole diameter/depth; hole pitch and object placement estimated from photos.",
        "masses_friction_gains": "Battery mass and friction are provisional; existing robot gains retained.",
        "foam_compliance": "Rigid approximation only; equal nominal diameters require a separate compliance/contact validation before insertion-force claims.",
    }
    config["insertion_geometry"] = {
        "hole_diameter_m": spec["props"]["hole_diameter_m"], "hole_depth_m": spec["props"]["hole_depth_m"],
        "hole_pitch_m": spec["props"]["hole_pitch_m"],
        "nominal_exposed_battery_length_m": spec["props"]["battery_length_m"]-spec["props"]["hole_depth_m"],
        "diameter_source": "user confirms same diameter as battery",
        "hole_pitch_source": "photo estimate",
    }
    for obsolete in ["scripted_grasp_preparation", "workspace_status"]:
        config.pop(obsolete, None)
    (ASSETS / "task_config.json").write_text(json.dumps(config, indent=2)+"\n")

    # Whole workcell plus front table, overhead and close inspection cameras.
    # ReviewSetup retains the robot in frame; ReviewBatteryTable matches the user photos.
    for name, eye, target, focal in [
        ("ReviewSetup", [.05, .65, 1.80], [1.57, -.35, .76], 24.),
        ("ReviewBatteryTable", world([.02, -.92, 1.46]), world([0., .005, .73]), 28.),
        ("ReviewBatteryTop", world([0., -.015, 1.96]), world([0., 0., .74]), 30.),
        ("ReviewBatteryDetail", world([-.04, -.30, 1.055]), world([-.04, -.075, .776]), 52.),
    ]:
        prim = stage.GetPrimAtPath("/World/"+name)
        if prim:
            UsdGeom.Xformable(prim).ClearXformOpOrder()
        look_at(stage, "/World/"+name, eye, target, focal=focal)
    custom = dict(stage.GetRootLayer().customLayerData)
    custom.update(task=spec["task"], cameraSettings={"boundCamera": "/World/ReviewSetup"},
                  measurementSource="User 2026-09-16; see scene_spec.json for measured and estimated fields",
                  taskValidation="Geometry and reset checks only; robotic insertion unvalidated")
    stage.GetRootLayer().customLayerData = custom
    stage.GetRootLayer().Save()
    manifest = {"scene": str(path.relative_to(ROOT)), "spec_sha256": config["scene_spec_sha256"],
                "disabled_old_prims": removed, "moved_background_prims": [],
                "loose_accessory_adjustments": clutter_changes,
                "relocation": spec["relocation"], "robot_base_position_world_m": config["robot_base_position"],
                "table": table, "props": props,
                "dynamic_objects": config["task_objects"], "validation_scope": spec["validation_scope"]}
    (ASSETS / "build_manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    print("BATTERY_SCENE_BUILT", path, flush=True)
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--spec", type=Path, default=ASSETS / "scene_spec.json")
    build(parser.parse_args().spec)
