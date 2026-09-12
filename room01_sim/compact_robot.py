"""Weld fixed URDF links while preserving every frame and visual placement.

The input display asset contains zero-mass rigid bodies for optical frames. This
builder retains those frames as ordinary Xforms and aggregates real URDF masses
and full inertia tensors into each surviving articulated link.
"""
from pathlib import Path
import json
import re

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from room01_sim.kinematics import RobotKinematics, origin_transform, quaternion_matrix, quaternion_xyzw


def identifier(name):
    name = re.sub(r"[^A-Za-z0-9_]", "_", name)
    return "_"+name if name[0].isdigit() else name


def gf_quat(rotation):
    x, y, z, w = quaternion_xyzw(rotation)
    return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))


def set_transform(prim, matrix):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Matrix4d(np.asarray(matrix).T.tolist()))


def compact(stage, output_path, positions, report_path, locked_joints=()):
    # Work on a self-contained layer; namespace edits can then update all internal references.
    layer = stage.Flatten()
    layer.Export(str(output_path))
    result = Usd.Stage.Open(str(output_path))
    k = RobotKinematics()
    locked_joints = set(locked_joints)
    poses = k.forward(positions)
    source_paths = {k.root: "/Robot/"+identifier(k.root)}
    for joint in k.joints.values():
        prim = result.GetPrimAtPath("/Robot/joints/"+joint["name"])
        targets = UsdPhysics.Joint(prim).GetBody1Rel().GetTargets()
        if targets:
            source_paths[joint["child"]] = str(targets[0])
    owners = {k.root: k.root}
    pending = list(k.joints.values())
    while pending:
        ready = [j for j in pending if j["parent"] in owners]
        for joint in ready:
            owners[joint["child"]] = owners[joint["parent"]] if joint["type"] == "fixed" or joint["name"] in locked_joints else joint["child"]
            pending.remove(joint)
        if not ready:
            raise ValueError("Disconnected URDF graph")
    retained = set(owners.values())
    frame_map = {}
    for name, owner in owners.items():
        path = "/Robot/"+identifier(owner)
        if name != owner:
            path += "/FixedFrames/"+identifier(name)
        local = np.linalg.inv(poses[owner])@poses[name]
        frame_map[name] = {"path": path, "rigid_body": owner, "body_to_frame": local.tolist()}

    # Re-express joint frame zero in the surviving parent rigid body's coordinates.
    for name, joint in k.joints.items():
        prim = result.GetPrimAtPath("/Robot/joints/"+name)
        if not prim:
            continue
        if joint["type"] == "fixed" or name in locked_joints:
            result.RemovePrim(prim.GetPath())
            continue
        api = UsdPhysics.Joint(prim)
        parent = joint["parent"]
        relative = np.linalg.inv(poses[owners[parent]])@poses[parent]
        old_position = np.asarray(api.GetLocalPos0Attr().Get(), dtype=float)
        old_quaternion = api.GetLocalRot0Attr().Get()
        old_rotation = quaternion_matrix([*old_quaternion.GetImaginary(), old_quaternion.GetReal()])
        api.CreateLocalPos0Attr(Gf.Vec3f(*(relative[:3, :3]@old_position+relative[:3, 3])))
        api.CreateLocalRot0Attr(gf_quat(relative[:3, :3]@old_rotation))
        api.CreateBody0Rel().SetTargets(["/Robot/"+identifier(owners[parent])])
        api.CreateBody1Rel().SetTargets(["/Robot/"+identifier(joint["child"])])
        api.CreateCollisionEnabledAttr(False)
        # CAD hinge pieces intentionally overlap. Exclude only the two bodies
        # directly connected by this joint, retaining other self-collisions.
        body0 = result.GetPrimAtPath("/Robot/"+identifier(owners[parent]))
        UsdPhysics.FilteredPairsAPI.Apply(body0).CreateFilteredPairsRel().AddTarget("/Robot/"+identifier(joint["child"]))

    # The two-axis wrist has nested CAD housings separated by the roll link.
    # Their convex hulls overlap through the joint's internal cavity (about
    # 27 mm inscribed radius at the side-facing reset). Treat these two
    # connected housings as one mechanical assembly for self-collision only.
    # Keep every housing's object/environment collisions and all other pairs.
    assembly_filtered_pairs = []
    for side in ["left", "right"]:
        a, b = side+"_wrist_pitch_link", side+"_elbow_yaw_link"
        UsdPhysics.FilteredPairsAPI.Apply(result.GetPrimAtPath("/Robot/"+a)).CreateFilteredPairsRel().AddTarget("/Robot/"+b)
        assembly_filtered_pairs.append({"body_a": a, "body_b": b,
            "reason": "Nested two-axis wrist housings; convex hull fills the internal joint cavity.",
            "diagnostic": "reports/room01_wrist_reset_fix/assembly_convex_overlap.json"})

    mass_records = []
    total_mass = 0.
    for owner in sorted(retained):
        pieces = []
        for name in owners:
            if owners[name] != owner:
                continue
            inertial = k.links[name].find("inertial")
            if inertial is None:
                continue
            mass = float(inertial.find("mass").get("value"))
            if mass <= 0:
                continue
            relative = np.linalg.inv(poses[owner])@poses[name]@origin_transform(inertial.find("origin"))
            element = inertial.find("inertia")
            xx, xy, xz, yy, yz, zz = [float(element.get(key)) for key in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]]
            inertia = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
            rotated = relative[:3, :3]@inertia@relative[:3, :3].T
            pieces.append((mass, relative[:3, 3], rotated))
        mass = sum(item[0] for item in pieces)
        if mass <= 0:
            raise ValueError(f"No physical mass for articulated link {owner}")
        center = sum(m*p for m, p, _ in pieces)/mass
        inertia = sum(i+m*((p-center)@(p-center)*np.eye(3)-np.outer(p-center, p-center)) for m, p, i in pieces)
        eigenvalues, axes = np.linalg.eigh(inertia)
        if min(eigenvalues) <= 0:
            raise ValueError(f"Non-positive inertia for {owner}: {eigenvalues}")
        if np.linalg.det(axes) < 0:
            axes[:, 0] *= -1
        prim = result.GetPrimAtPath("/Robot/"+identifier(owner))
        api = UsdPhysics.MassAPI.Apply(prim)
        api.CreateMassAttr(mass)
        api.CreateCenterOfMassAttr(Gf.Vec3f(*center))
        api.CreateDiagonalInertiaAttr(Gf.Vec3f(*eigenvalues))
        api.CreatePrincipalAxesAttr(gf_quat(axes))
        mass_records.append({"link": owner, "mass_kg": mass, "inertia_eigenvalues": eigenvalues.tolist()})
        total_mass += mass

    world_before = {}
    cache = UsdGeom.XformCache()
    for name, owner in owners.items():
        if name == owner:
            continue
        old_path = source_paths[name]
        prim = result.GetPrimAtPath(old_path)
        if not prim:
            raise ValueError("Missing fixed frame "+old_path)
        world_before[name] = np.asarray(cache.GetLocalToWorldTransform(prim))
        prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
        prim.RemoveAPI(UsdPhysics.MassAPI)
        prim.RemoveAppliedSchema("PhysxRigidBodyAPI")
        parent_path = "/Robot/"+identifier(owner)+"/FixedFrames"
        UsdGeom.Xform.Define(result, parent_path)
        editor = Usd.NamespaceEditor(result)
        if not editor.MovePrimAtPath(old_path, frame_map[name]["path"]) or not editor.ApplyEdits():
            raise RuntimeError("Could not weld frame "+name)
        set_transform(result.GetPrimAtPath(frame_map[name]["path"]), frame_map[name]["body_to_frame"])
    result.GetRootLayer().Save()
    cache = UsdGeom.XformCache()
    max_error = max(np.max(np.abs(world_before[name]-np.asarray(cache.GetLocalToWorldTransform(result.GetPrimAtPath(frame_map[name]["path"]))))) for name in world_before)
    report = {"result": "pass" if max_error < 1e-5 else "fail", "rigid_bodies_before": len(owners),
              "rigid_bodies_after": len(retained), "welded_fixed_frames": len(world_before),
              "fixed_frame_world_transform_max_abs_error": float(max_error), "total_urdf_mass_kg": total_mass,
              "task_locked_joints": sorted(locked_joints), "mass_properties": mass_records, "frames": frame_map}
    report["assembly_filtered_pairs"] = assembly_filtered_pairs
    Path(report_path).write_text(json.dumps(report, indent=2))
    Path(output_path).with_name("robot_frames.json").write_text(json.dumps(frame_map, indent=2))
    if report["result"] != "pass":
        raise ValueError("Fixed frame placement changed during welding")
    return result
