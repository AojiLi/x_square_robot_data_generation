#!/usr/bin/env python3
"""Check the authored workcell's geometry, references and physical structure."""
from pathlib import Path
import hashlib
import json
import sys

import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils

ROOT = Path(__file__).resolve().parents[1]
ASSET = ROOT / "assets/room01/sim"
stage = Usd.Stage.Open(str(ASSET / "room01_manipulation.usda"))
prims = list(Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()))
colliders = [p for p in prims if p.HasAPI(UsdPhysics.CollisionAPI)]
bodies = [p for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)]
joints = [p for p in prims if p.IsA(UsdPhysics.RevoluteJoint)]
config = json.loads((ASSET / "task_config.json").read_text())
cameras = json.loads((ASSET / "camera_manifest.json").read_text())
_, assets, unresolved = UsdUtils.ComputeAllDependencies(Sdf.AssetPath(str(ASSET / "room01_manipulation.usda")))
missing = [str(path) for path in unresolved if Path(str(path)).name not in {"OmniPBR.mdl", "OmniGlass.mdl"}]
invalid_mass = []
for body in bodies:
    api = UsdPhysics.MassAPI(body)
    inertia = api.GetDiagonalInertiaAttr().Get()
    if api.GetMassAttr().Get() <= 0 or inertia is None or min(inertia) <= 0:
        invalid_mass.append(str(body.GetPath()))
mimics = []
for joint in joints:
    for rel in joint.GetRelationships():
        if rel.GetName().startswith("physxMimicJoint:") and rel.GetName().endswith(":referenceJoint"):
            mimics.append({"joint": str(joint.GetPath()), "reference": [str(p) for p in rel.GetTargets()]})
assembly_pairs = [(f"/World/Robot/{side}_wrist_pitch_link", f"/World/Robot/{side}_elbow_yaw_link") for side in ["left", "right"]]
checks = {
    "meter_units": UsdGeom.GetStageMetersPerUnit(stage) == 1.,
    "z_up": UsdGeom.GetStageUpAxis(stage) == "Z",
    "environment_colliders": sum(str(p.GetPath()).startswith("/World/Room/") for p in colliders) == 383,
    "positive_mass_and_inertia": not invalid_mass,
    "36_joints_26_drives_10_mimics": len(joints) == 36 and len(config["action_joint_names"]) == 26 and len(mimics) == 10,
    "no_missing_file_dependencies": not missing,
    "collision_geometry_hidden_from_rgb": all(UsdGeom.Imageable(p).ComputeVisibility() == "invisible" for p in colliders if str(p.GetPath()).startswith(("/World/Room/", "/World/Robot/"))),
    "three_native_cameras": len(cameras) == 3 and all(stage.GetPrimAtPath(c["prim_path"]).IsA(UsdGeom.Camera) for c in cameras.values()),
    "valid_mimic_targets": all(len(m["reference"]) == 1 and stage.GetPrimAtPath(m["reference"][0]).IsA(UsdPhysics.RevoluteJoint) for m in mimics),
    "wrist_internal_assembly_pairs_filtered": all(b in [str(t) for t in UsdPhysics.FilteredPairsAPI(stage.GetPrimAtPath(a)).GetFilteredPairsRel().GetTargets()] for a, b in assembly_pairs),
    "remaining_self_collision_enabled": stage.GetPrimAtPath("/World/Robot/root_joint").GetAttribute("physxArticulation:enabledSelfCollisions").Get() is True,
    "all_robot_colliders_enabled": all(UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is True for p in colliders if str(p.GetPath()).startswith("/World/Robot/")),
    "original_urdf_unchanged": hashlib.sha256((ROOT / "assets/robots/quanta_x2/quanta_x2_dual_revo2_bridge.urdf").read_bytes()).hexdigest() == "9f57ade374229fc51f1dc84283679c36f2d45a7bdd88a590fa3d3c9e58548e94",
}
report = {"result": "pass" if all(checks.values()) else "fail", "checks": checks,
          "environment_colliders": sum(str(p.GetPath()).startswith("/World/Room/") for p in colliders),
          "robot_colliders": sum(str(p.GetPath()).startswith("/World/Robot/") for p in colliders),
          "rigid_bodies_including_object": len(bodies), "joints": len(joints), "mimic_constraints": len(mimics),
          "missing_dependencies": missing, "builtin_mdl_dependencies": [str(x) for x in unresolved if str(x) not in missing],
          "sha256": {name: hashlib.sha256((ASSET / name).read_bytes()).hexdigest() for name in ["room01_manipulation.usda", "room_environment.usdc", "quanta_x2_physics.usdc", "task_config.json", "robot_frames.json", "camera_manifest.json"]}}
(ROOT / "reports/room01_sim/asset_verification.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
sys.exit(0 if report["result"] == "pass" else 1)
