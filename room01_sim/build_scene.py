"""Build layered USD assets for the fixed-base, three-camera white-table task.

Uses OpenUSD only. PhysX schemas are authored by their documented names and
validated by the pinned Isaac Sim runtime before delivery.
"""
from pathlib import Path
import hashlib
import json
import math
import re
import sys

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from room01_sim.kinematics import ACTION_JOINTS, RobotKinematics, home_pose
from room01_sim.control import joint_servo

OUT = ROOT / "assets/room01/sim"
REPORT = ROOT / "reports/room01_sim"
REPORT.mkdir(exist_ok=True, parents=True)
MANIFEST = json.loads((OUT / "geometry_manifest.json").read_text())
GEOMETRY = np.load(OUT / "evaluated_geometry.npz")
FLOOR = MANIFEST["floor_z_source"]


def ident(name):
    name = re.sub(r"[^A-Za-z0-9_]", "_", name)
    return "_"+name if name[0].isdigit() else name


def new_stage(path, root_name):
    stage = Usd.Stage.CreateNew(str(path))
    root = UsdGeom.Xform.Define(stage, root_name)
    stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, "Z")
    stage.SetTimeCodesPerSecond(30.)
    return stage


def transform(prim, matrix):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Matrix4d(np.asarray(matrix, dtype=float).T.tolist()))


def attribute(prim, name, value, typ=Sdf.ValueTypeNames.Float):
    prim.CreateAttribute(name, typ, custom=False).Set(value)


def physics_material(stage, path, static=.8, dynamic=.65, restitution=0.):
    material = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(static)
    api.CreateDynamicFrictionAttr(dynamic)
    api.CreateRestitutionAttr(restitution)
    material.GetPrim().AddAppliedSchema("PhysxMaterialAPI")
    attribute(material.GetPrim(), "physxMaterial:frictionCombineMode", "average", Sdf.ValueTypeNames.Token)
    attribute(material.GetPrim(), "physxMaterial:restitutionCombineMode", "min", Sdf.ValueTypeNames.Token)
    return material


def bind_physics(prim, material):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")


def add_collision(prim, material, approximation=None, contact=.002):
    UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
    if approximation:
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approximation)
    prim.AddAppliedSchema("PhysxCollisionAPI")
    attribute(prim, "physxCollision:contactOffset", contact)
    attribute(prim, "physxCollision:restOffset", 0.)
    bind_physics(prim, material)


def make_cube(stage, path, center, size):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.)
    cube.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in center]))
    cube.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in size]))
    return cube.GetPrim()


def preview_material(stage, path, item):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path+"/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*item["color"]))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(item["roughness"])
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(item["metallic"])
    shader.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(item.get("ior", 1.5))
    opacity = .12 if item.get("transmission", 0.) > .5 else item.get("opacity", 1.)
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    reader = None
    for usage in ["color", "opacity"]:
        texture = item.get(usage+"_texture")
        if not texture:
            continue
        if reader is None:
            reader = UsdShade.Shader.Define(stage, path+"/UV")
            reader.CreateIdAttr("UsdPrimvarReader_float2")
            reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
        tex = UsdShade.Shader.Define(stage, path+"/"+usage.title()+"Texture")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texture))
        tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB" if usage == "color" else "raw")
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
        tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        tex.CreateOutput("r", Sdf.ValueTypeNames.Float)
        shader.GetInput("diffuseColor" if usage == "color" else "opacity").ConnectToSource(tex.ConnectableAPI(), "rgb" if usage == "color" else "r")
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def mesh_from_record(stage, path, record, visual=True):
    key = record["key"]
    mesh = UsdGeom.Mesh.Define(stage, path)
    points = GEOMETRY[key+"_points"]
    triangles = GEOMETRY[key+"_triangles"]
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(triangles), 3, np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(triangles.ravel()))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateExtentAttr(Vt.Vec3fArray.FromNumpy(np.stack([points.min(0), points.max(0)])))
    if visual:
        mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(GEOMETRY[key+"_normals"]))
        mesh.SetNormalsInterpolation("faceVarying")
        mesh.CreateDoubleSidedAttr(True)
        if key+"_uv" in GEOMETRY:
            UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, "faceVarying").Set(Vt.Vec2fArray.FromNumpy(GEOMETRY[key+"_uv"]))
    transform(mesh.GetPrim(), record["matrix_world"])
    mesh.GetPrim().SetCustomData({"blender_object": record["name"]})
    return mesh


def build_environment():
    stage = new_stage(OUT / "room_environment.usdc", "/Room")
    root = stage.GetDefaultPrim()
    # One documented translation aligns the source floor to the simulation Z=0 plane.
    UsdGeom.Xformable(root).AddTranslateOp().Set(Gf.Vec3d(0., 0., -FLOOR))
    root.SetCustomData({"source_floor_z": FLOOR, "floor_alignment_translation_z": -FLOOR,
                        "purpose": "Full room appearance and static collision environment"})
    UsdGeom.Xform.Define(stage, "/Room/Visual")
    UsdGeom.Xform.Define(stage, "/Room/Collisions")
    UsdGeom.Scope.Define(stage, "/Room/Looks")
    UsdGeom.Scope.Define(stage, "/Room/PhysicsMaterials")
    materials = {name: preview_material(stage, "/Room/Looks/"+ident(name), item)
                 for name, item in MANIFEST["materials"].items()}
    mat_floor = physics_material(stage, "/Room/PhysicsMaterials/Carpet", .85, .7)
    mat_table = physics_material(stage, "/Room/PhysicsMaterials/Laminate", .65, .5)
    mat_static = physics_material(stage, "/Room/PhysicsMaterials/Room", .6, .5)
    collision_manifest = []
    for record in MANIFEST["objects"]:
        collection = record["collections"][0]
        path = "/Room/Visual/"+ident(collection)+"/"+ident(record["name"])
        mesh = mesh_from_record(stage, path, record)
        material_indices = GEOMETRY[record["key"]+"_material_indices"]
        for index, name in enumerate(record["materials"]):
            if name not in materials:
                continue
            if len(record["materials"]) == 1:
                UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(materials[name])
            else:
                subset = UsdGeom.Subset.Define(stage, path+f"/Material_{index}")
                subset.CreateElementTypeAttr("face")
                subset.CreateFamilyNameAttr("materialBind")
                subset.CreateIndicesAttr(Vt.IntArray.FromNumpy(np.flatnonzero(material_indices == index).astype(np.int32)))
                UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(materials[name])
        if record["name"] == "Distant_photo_facade":
            mesh.GetPrim().SetCustomData({"role": "Photographic exterior backdrop; not surveyed geometry; no collision"})

    # Use thick floor/ceiling solids with the L-shaped room footprint.
    rectangles = [(-2.426, 1.346, -2.530, 1.570), (1.346, 2.339, -1.463, 1.570)]
    for i, (xmin, xmax, ymin, ymax) in enumerate(rectangles):
        for kind, z in [("Floor", FLOOR-.06), ("Ceiling", MANIFEST["ceiling_z_source"]+.06)]:
            path = f"/Room/Collisions/{kind}_{i}"
            prim = make_cube(stage, path, ((xmin+xmax)/2, (ymin+ymax)/2, z), (xmax-xmin, ymax-ymin, .12))
            add_collision(prim, mat_floor if kind == "Floor" else mat_static)
            collision_manifest.append({"prim": path, "source": kind, "approximation": "box"})

    skip_prefixes = ("Carpet_", "Blind_", "Window_vertical_seal", "Skirting_", "Table_marker_", "Laminate_scuff",
                     "Retail_box_print", "Monitor_brand", "Keyboard_legend", "Flightcase_care", "Flightcase_handwritten",
                     "Flightcase_shipping", "Flightcase_green", "Thermostat_", "Smoke_", "Ceiling_", "Vent_", "Camera_",
                     "Wall_camera_", "Distant_", "Floor_slab", "Window_handle", "Window_lever", "Window_outer_rail", "Window_mullion")
    box_prefixes = ("West_wall", "East_wall", "Inset_", "Door_leaf", "Wood_sidelight", "Door_edge_filler", "Partition_glass",
                    "Partition_upright", "Partition_crossbar", "Window_glass", "Writing_table_leg", "Writing_table_foot",
                    "Writing_table_apron", "Optical_table_body", "Optical_table_leg", "Workbench_", "Monitor_", "Computer_",
                    "Flightcase_body", "Large_case_body", "Sorting_tray_", "Pin_holder", "White_retail_box", "Interface_box")
    detail_skip = ("_legend", "_rivet", "_screw", "_stitch", "_slot", "_print", "_label", "_key", "_tape", "_cable", "_coil", "_marker", "ethernet", "_lead", "_loop")
    for record in MANIFEST["objects"]:
        name = record["name"]
        number = int(record["collections"][0][:2])
        if number in {6, 7, 14, 15} or name.startswith(skip_prefixes):
            continue
        if any(part in name.lower() for part in detail_skip):
            continue
        low, high = np.asarray(record["bounds_local"])
        size = high-low
        if np.max(size) < .035:
            continue
        path = "/Room/Collisions/"+ident(name)
        exact = name in {"Writing_table_top", "Optical_table_top", "Writing_table_edge", "Writing_table_grommet_rim"}
        if exact:
            mesh = mesh_from_record(stage, path, record, visual=False)
            prim = mesh.GetPrim()
            add_collision(prim, mat_table, "none", .001)
            approximation = "static triangle mesh"
        elif name.startswith(box_prefixes) or min(size) < .008 or max(size)/max(min(size), .001) > 30:
            parent = UsdGeom.Xform.Define(stage, path)
            transform(parent.GetPrim(), record["matrix_world"])
            prim = make_cube(stage, path+"/Box", (low+high)/2, np.maximum(size, .012))
            add_collision(prim, mat_static)
            approximation = "oriented box"
        elif number >= 8:
            mesh = mesh_from_record(stage, path, record, visual=False)
            prim = mesh.GetPrim()
            add_collision(prim, mat_static, "convexHull")
            approximation = "convex hull per component"
        else:
            continue
        collision_manifest.append({"prim": str(prim.GetPath()), "source": name, "approximation": approximation})
    UsdGeom.Imageable(stage.GetPrimAtPath("/Room/Collisions")).CreateVisibilityAttr("invisible")
    stage.GetRootLayer().Save()
    (REPORT / "environment_collision_manifest.json").write_text(json.dumps(collision_manifest, indent=2))
    return len(collision_manifest)


def default_config():
    return json.loads((ROOT / "room01_sim/default_task_config.json").read_text())


def build_robot(config):
    stage = new_stage(OUT / "quanta_x2_fixed.usda", "/Robot")
    stage.GetDefaultPrim().GetReferences().AddReference("../../robots/quanta_x2/quanta_x2_robot.usdc")
    k = RobotKinematics()
    q = k.expand_mimics(config["joint_home_rad"])
    poses = k.forward(q)
    link_paths = {k.root: "/Robot/"+ident(k.root)}
    for joint in k.joints.values():
        joint_prim = stage.GetPrimAtPath("/Robot/joints/"+joint["name"])
        targets = UsdPhysics.Joint(joint_prim).GetBody1Rel().GetTargets()
        if targets:
            link_paths[joint["child"]] = str(targets[0])
    for name, pose in poses.items():
        path = link_paths.get(name, "/Robot/"+ident(name))
        prim = stage.GetPrimAtPath(path)
        if prim:
            transform(prim, pose)
    root_joint = stage.GetPrimAtPath("/Robot/root_joint")
    root_joint.AddAppliedSchema("PhysxArticulationAPI")
    attribute(root_joint, "physxArticulation:enabledSelfCollisions", True, Sdf.ValueTypeNames.Bool)
    attribute(root_joint, "physxArticulation:solverPositionIterationCount", 16, Sdf.ValueTypeNames.Int)
    attribute(root_joint, "physxArticulation:solverVelocityIterationCount", 4, Sdf.ValueTypeNames.Int)
    hand_material = physics_material(stage, "/Robot/TrainingMaterials/FingerContact", .9, .75)
    # Expose only collision instances so millimeter contact offsets can be authored
    # on their meshes. Visual instances remain shared.
    while True:
        collision_instances = [p for p in stage.Traverse() if p.IsInstance() and "/collisions" in str(p.GetPath())]
        if not collision_instances:
            break
        for prim in collision_instances:
            prim.SetInstanceable(False)
    for parent in list(stage.Traverse()):
        if parent.HasAPI(UsdPhysics.CollisionAPI) and not parent.IsA(UsdGeom.Mesh):
            approximation = UsdPhysics.MeshCollisionAPI(parent).GetApproximationAttr().Get() or "convexHull"
            for child in Usd.PrimRange(parent):
                if child.IsA(UsdGeom.Mesh):
                    UsdPhysics.CollisionAPI.Apply(child)
                    UsdPhysics.MeshCollisionAPI.Apply(child).CreateApproximationAttr(approximation)
            parent.RemoveAPI(UsdPhysics.CollisionAPI)
            parent.RemoveAPI(UsdPhysics.MeshCollisionAPI)
    for prim in stage.Traverse():
        if prim.GetName() == "collisions":
            UsdGeom.Imageable(prim).CreateVisibilityAttr("invisible")
    robot_looks = {
        "hand": preview_material(stage, "/Robot/TrainingLooks/Revo2", {"color": [.017, .02, .024], "roughness": .48, "metallic": .12}),
        "arm": preview_material(stage, "/Robot/TrainingLooks/WhiteShell", {"color": [.72, .745, .73], "roughness": .3, "metallic": .08}),
        "adapter": preview_material(stage, "/Robot/TrainingLooks/Adapter", {"color": [.045, .065, .11], "roughness": .45, "metallic": .45}),
        "wheel": preview_material(stage, "/Robot/TrainingLooks/Tire", {"color": [.012, .013, .016], "roughness": .8, "metallic": 0.}),
    }
    for name, path in link_paths.items():
        visual = stage.GetPrimAtPath(path+"/visuals")
        if not visual:
            continue
        category = None
        if any(word in name for word in ["hand_base", "thumb", "index", "middle", "ring", "pinky"]):
            category = "hand"
        elif any(word in name for word in ["shoulder", "elbow", "wrist"]):
            category = "arm"
        elif any(word in name for word in ["flange", "adapter", "in_hand"]):
            category = "adapter"
        elif "wheel" in name:
            category = "wheel"
        if category:
            UsdShade.MaterialBindingAPI.Apply(visual).Bind(robot_looks[category], UsdShade.Tokens.strongerThanDescendants)
    applied_mimics = []
    for name, joint in k.joints.items():
        if joint["type"] == "fixed":
            continue
        prim = stage.GetPrimAtPath("/Robot/joints/"+name)
        if not prim:
            raise ValueError(f"Missing robot joint: {name}")
        prim.AddAppliedSchema("PhysicsJointStateAPI:angular")
        attribute(prim, "state:angular:physics:position", math.degrees(q.get(name, 0.)))
        attribute(prim, "state:angular:physics:velocity", 0.)
        if any(word in name for word in ["thumb", "index", "middle", "ring", "pinky"]):
            prim.AddAppliedSchema("PhysxJointAPI")
            attribute(prim, "physxJoint:armature", joint_servo(name, bool(joint["mimic"]))[2])
        mimic = joint["mimic"]
        if mimic:
            # PhysX defines q + gearing*q_reference + offset = 0.
            axis = "rot"+str(prim.GetAttribute("physics:axis").Get())
            prim.AddAppliedSchema("PhysxMimicJointAPI:"+axis)
            prefix = "physxMimicJoint:"+axis+":"
            attribute(prim, prefix+"gearing", -float(mimic.get("multiplier", 1.)))
            attribute(prim, prefix+"offset", -math.degrees(float(mimic.get("offset", 0.))))
            attribute(prim, prefix+"naturalFrequency", 500.)
            attribute(prim, prefix+"dampingRatio", 1.)
            prim.CreateRelationship(prefix+"referenceJoint", custom=False).SetTargets(["/Robot/joints/"+mimic["joint"]])
            prim.RemoveAPI(UsdPhysics.DriveAPI, "angular")
            for prop in prim.GetAttributes():
                if prop.GetName().startswith("drive:angular:"):
                    prop.Block()
            applied_mimics.append({"joint": name, "reference": mimic["joint"], "multiplier": float(mimic.get("multiplier", 1.))})
            continue
        stiffness, damping, armature = joint_servo(name)
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr("force")
        # USD revolute drives use degrees, unlike URDF and the action interface.
        drive.CreateStiffnessAttr(stiffness*math.pi/180.)
        drive.CreateDampingAttr(damping*math.pi/180.)
        drive.CreateMaxForceAttr(joint["effort"])
        drive.CreateTargetPositionAttr(math.degrees(q.get(name, 0.)))
        prim.AddAppliedSchema("PhysxJointAPI")
        attribute(prim, "physxJoint:maxJointVelocity", math.degrees(joint["velocity"]))
        attribute(prim, "physxJoint:armature", armature)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            prim.AddAppliedSchema("PhysxCollisionAPI")
            attribute(prim, "physxCollision:contactOffset", .001)
            attribute(prim, "physxCollision:restOffset", 0.)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim.AddAppliedSchema("PhysxRigidBodyAPI")
            attribute(prim, "physxRigidBody:maxDepenetrationVelocity", 1.)
            attribute(prim, "physxRigidBody:enableCCD", True, Sdf.ValueTypeNames.Bool)
            if any(word in prim.GetName() for word in ["hand", "thumb", "index", "middle", "ring", "pinky"]):
                bind_physics(prim, hand_material)
    stage.GetRootLayer().customLayerData = {"purpose": "Fixed-base manipulation robot with URDF hand couplings", "mimicCouplings": len(applied_mimics), "baseMode": "fixed"}
    stage.GetRootLayer().Save()
    from room01_sim.compact_robot import compact
    compact(stage, OUT / "quanta_x2_physics.usdc", q, REPORT / "robot_fixed_link_welding.json", config.get("locked_joint_names", []))
    (REPORT / "robot_mimic_manifest.json").write_text(json.dumps(applied_mimics, indent=2))


def look_at(stage, path, position, target, focal=24.):
    camera = UsdGeom.Camera.Define(stage, path)
    matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*position), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
    camera.AddTransformOp().Set(matrix)
    camera.CreateFocalLengthAttr(focal)
    camera.CreateHorizontalApertureAttr(36.)
    camera.CreateVerticalApertureAttr(24.)
    camera.CreateClippingRangeAttr(Gf.Vec2f(.01, 100.))
    return camera


def build_task(config):
    stage = new_stage(OUT / "room01_manipulation.usda", "/World")
    stage.DefinePrim("/World/Room").GetReferences().AddReference("room_environment.usdc")
    robot = stage.DefinePrim("/World/Robot")
    robot.GetReferences().AddReference("quanta_x2_physics.usdc")
    base_matrix = np.eye(4)
    yaw = config["robot_base_yaw_rad"]
    base_matrix[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0.], [math.sin(yaw), math.cos(yaw), 0.], [0., 0., 1.]]
    base_matrix[:3, 3] = config["robot_base_position"]
    transform(robot, base_matrix)
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    scene.GetPrim().AddAppliedSchema("PhysxSceneAPI")
    for name, value, typ in [("timeStepsPerSecond", round(1/config["physics_dt"]), Sdf.ValueTypeNames.UInt), ("enableCCD", True, Sdf.ValueTypeNames.Bool), ("solverType", "TGS", Sdf.ValueTypeNames.Token), ("enableGPUDynamics", True, Sdf.ValueTypeNames.Bool), ("broadphaseType", "GPU", Sdf.ValueTypeNames.Token)]:
        attribute(scene.GetPrim(), "physxScene:"+name, value, typ)
    contact_material = physics_material(stage, "/World/PhysicsMaterials/TrainingProp", .75, .6)
    cube = UsdGeom.Xform.Define(stage, "/World/Task/RedCube").GetPrim()
    UsdGeom.Xformable(cube).AddTranslateOp().Set(Gf.Vec3d(*config["cube_position"]))
    shape = make_cube(stage, "/World/Task/RedCube/Shape", (0., 0., 0.), config["cube_size"])
    add_collision(shape, contact_material, contact=.001)
    UsdPhysics.RigidBodyAPI.Apply(cube)
    mass = UsdPhysics.MassAPI.Apply(cube)
    mass.CreateMassAttr(config["cube_mass_kg"])
    dims = np.asarray(config["cube_size"])
    inertia = config["cube_mass_kg"]/12.*np.array([dims[1]**2+dims[2]**2, dims[0]**2+dims[2]**2, dims[0]**2+dims[1]**2])
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
    cube.AddAppliedSchema("PhysxRigidBodyAPI")
    attribute(cube, "physxRigidBody:enableCCD", True, Sdf.ValueTypeNames.Bool)
    attribute(cube, "physxRigidBody:solverPositionIterationCount", 16, Sdf.ValueTypeNames.Int)
    attribute(cube, "physxRigidBody:solverVelocityIterationCount", 4, Sdf.ValueTypeNames.Int)
    material = preview_material(stage, "/World/Looks/RedCube", {"color": [.62, .025, .018], "roughness": .55, "metallic": 0.})
    UsdShade.MaterialBindingAPI.Apply(shape).Bind(material)
    dome = UsdLux.DomeLight.Define(stage, "/World/Lighting/Sky")
    dome.CreateIntensityAttr(1200.)
    dome.CreateColorAttr(Gf.Vec3f(.82, .9, 1.))
    sun = UsdLux.DistantLight.Define(stage, "/World/Lighting/Daylight")
    sun.CreateIntensityAttr(3000.)
    sun.CreateAngleAttr(7.)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-55., 0., 20.))
    for name, center, size, intensity in [("CeilingFill", (0., -.4, 2.56), (3.6, 3.2), 1500.), ("DeskFill", (.2, -1.9, 2.3), (1.2, 1.2), 650.)]:
        light = UsdLux.RectLight.Define(stage, "/World/Lighting/"+name)
        light.CreateWidthAttr(size[0]); light.CreateHeightAttr(size[1]); light.CreateIntensityAttr(intensity)
        light.AddTranslateOp().Set(Gf.Vec3d(*center))
    look_at(stage, "/World/ReviewSetup", (1.25, -.60, 1.55), (.68, -1.95, .82))
    look_at(stage, "/World/ReviewTable", (.0, -2.0, 1.4), (1.0, -1.94, .72))
    from room01_sim.camera_assets import author_cameras
    author_cameras(stage, config)
    stage.GetRootLayer().customLayerData = {
        "cameraSettings": {"boundCamera": "/World/ReviewSetup"},
        "renderSettings": {"rtx:rendermode": "RealTimePathTracing", "rtx:post:aa:op": 4, "rtx-transient:dlssg:enabled": False, "rtx:post:dof:enabled": False, "rtx:post:motionblur:enabled": False},
        "task": config["task"], "cameraCalibration": config["camera_calibration_status"],
        "physicsParameters": "Initial simulation values; physical and camera verification reports are stored alongside the task code.",
    }
    stage.GetRootLayer().Save()


def main():
    config_path = OUT / "task_config.json"
    if not config_path.exists():
        config_path.write_text(json.dumps(default_config(), indent=2))
    config = json.loads(config_path.read_text())
    colliders = build_environment()
    build_robot(config)
    build_task(config)
    report = {"environment_colliders": colliders, "visual_meshes": len(MANIFEST["objects"]), "triangle_count": MANIFEST["triangles"], "floor_z": 0., "tabletop_z": config["tabletop_z"], "task": config["task"], "source_blend_sha256": MANIFEST["source_sha256"], "runtime_validation": "pending"}
    (REPORT / "build_manifest.json").write_text(json.dumps(report, indent=2))
    print("SIM_SCENE_BUILT", json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
