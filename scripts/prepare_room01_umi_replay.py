#!/usr/bin/env python3
"""Prepare the audited first-episode UMI replay and its separate USD layer."""
import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from room01_sim.umi import load_episode, sample_poses, finger_targets, ArmChain
from room01_sim.kinematics import RobotKinematics, ACTION_JOINTS, ARM_SUFFIXES


def author_scene(path, config):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    stage.GetRootLayer().subLayerPaths = [os.path.relpath(ROOT / "assets/room01/sim/room01_manipulation.usda", path.parent)]
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    stage.OverridePrim("/World/Task/RedCube").SetActive(False)
    material = UsdShade.Material.Define(stage, "/World/PhysicsMaterials/UmiWood")
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr(.75)
    physics.CreateDynamicFrictionAttr(.6)
    physics.CreateRestitutionAttr(0.)
    for name, spec in config["task_objects"].items():
        root = UsdGeom.Xform.Define(stage, spec["prim_path"]).GetPrim()
        xf = UsdGeom.Xformable(root)
        xf.AddTranslateOp().Set(Gf.Vec3d(*spec["position"]))
        q = spec.get("orientation_xyzw", [0, 0, 0, 1])
        xf.AddOrientOp().Set(Gf.Quatf(q[3], Gf.Vec3f(*q[:3])))
        UsdPhysics.RigidBodyAPI.Apply(root)
        root.AddAppliedSchema("PhysxRigidBodyAPI")
        for key, value, kind in [("enableCCD", True, Sdf.ValueTypeNames.Bool),
                                 ("solverPositionIterationCount", 32, Sdf.ValueTypeNames.Int),
                                 ("solverVelocityIterationCount", 8, Sdf.ValueTypeNames.Int)]:
            root.CreateAttribute("physxRigidBody:"+key, kind, custom=False).Set(value)
        mass = UsdPhysics.MassAPI.Apply(root)
        mass.CreateMassAttr(spec["mass_kg"])
        if name == "BlackBox":
            x, y, z = spec["size"]
            wall, bottom = spec["wall_m"], spec["bottom_m"]
            slabs = [([0, 0, -z/2+bottom/2], [x, y, bottom]),
                     ([-x/2+wall/2, 0, bottom/2], [wall, y, z-bottom]),
                     ([x/2-wall/2, 0, bottom/2], [wall, y, z-bottom]),
                     ([0, -y/2+wall/2, bottom/2], [x-2*wall, wall, z-bottom]),
                     ([0, y/2-wall/2, bottom/2], [x-2*wall, wall, z-bottom])]
        else:
            slabs = [([0, 0, 0], spec["size"])]
        # Uniform density in the five non-overlapping wooden slabs.
        volumes = np.array([np.prod(size) for _, size in slabs])
        masses = spec["mass_kg"]*volumes/volumes.sum()
        centers = np.array([pos for pos, _ in slabs])
        com = np.average(centers, axis=0, weights=masses)
        inertia = np.zeros(3)
        for body_mass, (pos, size) in zip(masses, slabs):
            dims = np.asarray(size); offset = np.asarray(pos)-com
            inertia += body_mass/12*(np.sum(dims**2)-dims**2)
            inertia += body_mass*(np.sum(offset**2)-offset**2)
        mass.CreateCenterOfMassAttr(Gf.Vec3f(*com))
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        visual = UsdShade.Material.Define(stage, "/World/Looks/Umi"+name)
        shader = UsdShade.Shader.Define(stage, str(visual.GetPath())+"/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*spec["color"]))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.55)
        visual.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        for index, (pos, size) in enumerate(slabs):
            shape = UsdGeom.Cube.Define(stage, spec["prim_path"]+f"/Shape{index}")
            shape.CreateSizeAttr(1.)
            shape.AddTranslateOp().Set(Gf.Vec3d(*pos))
            shape.AddScaleOp().Set(Gf.Vec3f(*size))
            prim = shape.GetPrim()
            UsdPhysics.CollisionAPI.Apply(prim)
            prim.AddAppliedSchema("PhysxCollisionAPI")
            prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float, custom=False).Set(.0005)
            prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float, custom=False).Set(0.)
            binding = UsdShade.MaterialBindingAPI.Apply(prim)
            binding.Bind(visual)
            binding.Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")
    stage.GetRootLayer().Save()


def prepare(args):
    from room01_sim.umi import SIDES
    data = load_episode(args.source)
    config = json.loads((ROOT / "assets/room01/sim/task_config.json").read_text())
    source = data["source_poses"]
    # Estimated Y-up tracking convention and heading from the initial hand span.
    yup_to_zup = np.array([[-1., 0, 0], [0, 0, 1], [0, 1, 0]])
    span = yup_to_zup @ (source[0, 0, :3, 3]-source[0, 1, :3, 3])
    yaw = np.arctan2(span[0], span[1])
    world = np.eye(4)
    world[:3, :3] = Rotation.from_euler("z", yaw).as_matrix() @ yup_to_zup
    source_midpoint = source[0, :, :3, 3].mean(axis=0)
    world[:3, 3] = np.array([args.initial_x, -1.937, .86])-world[:3, :3] @ source_midpoint
    tool_to_base = np.broadcast_to(np.eye(4), (2, 4, 4)).copy()
    tool_to_base[0, :3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    tool_to_base[1, :3, :3] = [[0, 0, 1], [1, 0, 0], [0, 1, 0]]
    # Explicit geometric estimates, used only to register the task objects.
    grasp_offsets = np.array([[.05, -.036, .087], [.05, .036, .087]])
    grasp_times = [7.5, 1.4]
    hand = world @ source @ tool_to_base
    grasp_centers = np.array([
        hand[np.argmin(abs(data["times"]-time)), side, :3, :3] @ grasp_offsets[side]
        + hand[np.argmin(abs(data["times"]-time)), side, :3, 3]
        for side, time in enumerate(grasp_times)
    ])
    z_change = config["tabletop_z"]+.025-grasp_centers[:, 2].mean()+args.z_offset
    world[2, 3] += z_change
    hand = world @ source @ tool_to_base
    grasp_centers[:, 2] += z_change
    drop_centers = np.array([
        hand[np.argmin(abs(data["times"]-time)), side, :3, :3] @ grasp_offsets[side]
        + hand[np.argmin(abs(data["times"]-time)), side, :3, 3]
        for side, time in enumerate([10.4, 4.4])
    ])
    box_center = np.r_[drop_centers[:, :2].mean(axis=0), config["tabletop_z"]+.102/2]
    box_q = Rotation.from_euler("z", np.pi/2).as_quat().tolist()
    objects = {}
    for name, side, color in [("RedBig", 0, [.62, .025, .018]), ("Blue", 1, [.015, .32, .65])]:
        objects[name] = {"prim_path": "/World/Task/"+name,
                         "position": [*grasp_centers[side, :2], config["tabletop_z"]+.025+.0002],
                         "size": [.05]*3, "mass_kg": .072, "color": color}
    for name, offset, size, color in [("Green", [-.025, -.035], .05, [.015, .42, .18]),
                                      ("RedSmall", [-.022, .025], .04, [.62, .025, .018])]:
        objects[name] = {"prim_path": "/World/Task/"+name,
                         "position": [*(box_center[:2]+offset), config["tabletop_z"]+.002+size/2+.0002],
                         "size": [size]*3, "mass_kg": .072 if size == .05 else .042, "color": color}
    objects["BlackBox"] = {"prim_path": "/World/Task/BlackBox", "position": box_center.tolist(),
                            "orientation_xyzw": box_q, "size": [.174, .144, .102], "mass_kg": .110,
                            "wall_m": .002, "bottom_m": .002, "color": [.018, .018, .018]}
    config.update(task="umi_two_blocks_into_box", language_instruction="Put the two objects into the box.",
                  task_objects=objects, primary_object="RedBig", target_objects=["RedBig", "Blue"],
                  cube_position=objects["RedBig"]["position"], cube_size=[.05]*3, cube_mass_kg=.072)
    calibration = {
        "status": "estimated_registration_not_measured_calibration",
        "source_episode": str(args.source), "source_validation": data["audit"],
        "tracking_world_to_sim_world": world.tolist(), "training_tool_to_urdf_hand_base": tool_to_base.tolist(),
        "assumptions": ["UMI tracking +Y is up; initial hand span defines table lateral axis",
                        "Source training-target origin is provisionally identified with the URDF hand base",
                        "Fixed rotations reconcile canonical forward/left/up with mirrored URDF hand frames",
                        "Wood friction estimated at static .75 / dynamic .6; base uses existing table material",
                        "Bottom thickness assumed equal to the approximately measured 2 mm walls"],
        "initial_object_pose_method": "Head RGB establishes identities and outside/inside ordering; metric positions use estimated hand-space grasp centers and release centers, not a calibrated RGB reconstruction",
        "grasp_reference_times_s": grasp_times, "grasp_center_in_hand_base_m": grasp_offsets.tolist(),
        "grasp_height_residual_m": (grasp_centers[:, 2]-config["tabletop_z"]-.025).tolist(),
        "finger_mapping": {"source_order": ["thumb_flex", "thumb_aux", "index", "middle", "ring", "little"],
                           "urdf_indices": [1, 0, 2, 3, 4, 5],
                           "evidence": "names and 59/90/81-degree limits; hardware-to-URDF identity not measured"},
        "limits": {"local_position_correction_m": .01, "local_rotation_correction_deg": 5., "max_time_scale": 2.},
    }
    times = np.arange(round(float(data["times"][-1])*30)+1)/30
    sampled_source = sample_poses(data["times"], source, times)
    target_hand = world @ sampled_source @ tool_to_base
    fingers = np.stack([np.interp(times, data["times"], data["fingers_deg"][:, side, drive])
                        for side in range(2) for drive in range(6)], axis=-1).reshape(-1, 2, 6)
    robot = RobotKinematics()
    base = np.eye(4); base[:3, 3] = config["robot_base_position"]
    chains = [ArmChain(robot, side, config["joint_home_rad"], base) for side in SIDES]
    q = np.zeros((len(times), 26))
    errors = np.zeros((len(times), 2, 2))
    for side in range(2):
        previous = np.array([config["joint_home_rad"][name] for name in chains[side].names])
        for index, target in enumerate(target_hand[:, side]):
            previous, distance, angle = chains[side].solve(target, previous)
            q[index, side*13:side*13+7] = previous
            errors[index, side] = [distance, angle]
        q[:, side*13+7:side*13+13] = finger_targets(fingers[:, side])
    lower = np.array([robot.joints[name]["lower"] for name in ACTION_JOINTS])
    upper = np.array([robot.joints[name]["upper"] for name in ACTION_JOINTS])
    clipping = np.maximum(lower-q, 0)+np.maximum(q-upper, 0)
    q = np.clip(q, lower, upper)
    if errors[:, :, 0].max() > .005 or errors[:, :, 1].max() > .05:
        raise ValueError("Registration is not reachable within 5 mm / 0.05 rad IK tolerance; adjust the whole workcell placement before replay")
    config["joint_home_rad"].update(dict(zip(ACTION_JOINTS, q[0].tolist())))
    config["joint_home_rad"] = robot.expand_mimics(config["joint_home_rad"])
    summary = {"frames_30hz": len(times), "max_ik_position_error_m": errors[:, :, 0].max(axis=0).tolist(),
               "max_ik_rotation_error_deg": np.rad2deg(errors[:, :, 1]).max(axis=0).tolist(),
               "max_finger_limit_clip_deg": float(np.rad2deg(clipping).max()),
               "objects": objects, "initial_hand_positions": target_hand[0, :, :3, 3].tolist()}
    out = ROOT / "assets/room01/umi_replay"
    out.mkdir(parents=True, exist_ok=True)
    (out / "task_config.json").write_text(json.dumps(config, indent=2))
    (out / "registration.json").write_text(json.dumps(calibration, indent=2))
    args.output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output / "trajectory.npz", times=times, targets=q, hand_targets=target_hand,
                        tool_to_hand_base=tool_to_base, world_from_tracking=world, ik_errors=errors,
                        source_times=data["times"], source_poses=source, source_fingers_deg=data["fingers_deg"],
                        source_frame_ids=data["source_frame_ids"])
    (args.output / "preparation.json").write_text(json.dumps(summary, indent=2))
    author_scene(out / "room01_umi_replay.usda", config)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=ROOT / "data/umi_replay/episode_000000")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/room01/umi_prepared")
    parser.add_argument("--initial-x", type=float, default=.68)
    parser.add_argument("--z-offset", type=float, default=0.)
    prepare(parser.parse_args())
