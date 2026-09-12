"""Save a separate Blender review file with robot, cameras and collision proxies.

The Isaac USD/Python task remains the authority for articulated robot dynamics.
"""
from pathlib import Path
import json
import numpy as np
import bpy
from mathutils import Matrix

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports/room01_sim"
data = json.loads((REPORT / "review_geometry.json").read_text())
arrays = np.load(REPORT / "review_geometry.npz")
config = json.loads((ROOT / "assets/room01/sim/task_config.json").read_text())
scene = bpy.context.scene
for obj in list(scene.objects):
    if obj.parent is None:
        obj.location.z += 1.466
scene.render.fps = 30
scene.unit_settings.system = "METRIC"
scene.unit_settings.scale_length = 1.

collections = {}
for role, name in [("environment_collision", "90 Simulation collision proxies"), ("robot_visual", "91 Quanta X2 fixed workcell"), ("task_cube", "92 Dynamic training object"), ("cameras", "93 Simulation cameras")]:
    coll = bpy.data.collections.new(name)
    scene.collection.children.link(coll)
    collections[role] = coll
materials = {}
for item in data["meshes"]:
    key = item["key"]
    points, counts, indices = arrays[key+"_points"], arrays[key+"_counts"], arrays[key+"_indices"]
    ends = np.cumsum(counts)
    starts = np.r_[0, ends[:-1]]
    faces = [indices[a:b].tolist() for a, b in zip(starts, ends)]
    center = (points.min(axis=0)+points.max(axis=0))/2
    mesh = bpy.data.meshes.new(key)
    mesh.from_pydata((points-center).tolist(), [], faces)
    mesh.update()
    name = item["prim"].split("/")[-1]
    if item["role"] == "robot_visual":
        name = "Robot_"+item["prim"].split("/visuals/")[0].split("/")[-1]
    obj = bpy.data.objects.new(name, mesh)
    obj.location = center.tolist()
    collections[item["role"]].objects.link(obj)
    obj["simulation_prim"] = item["prim"]
    if item["role"] == "environment_collision":
        obj.display_type = "WIRE"
        obj.hide_render = True
        obj.color = (.1, .85, .25, 1.)
        obj["collision_approximation"] = item["approximation"]
        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)
        bpy.ops.rigidbody.object_add()
        obj.rigid_body.type = "PASSIVE"
        obj.rigid_body.collision_shape = "MESH" if item["approximation"] == "none" else "CONVEX_HULL"
        obj.rigid_body.friction = .65
        obj.rigid_body.use_margin = True
        obj.rigid_body.collision_margin = .001
        obj.select_set(False)
    else:
        mk = (*item["color"], item["roughness"], item["metallic"])
        if mk not in materials:
            material = bpy.data.materials.new("Simulation_material_"+str(len(materials)))
            material.use_nodes = True
            bsdf = material.node_tree.nodes.get("Principled BSDF")
            bsdf.inputs["Base Color"].default_value = (*item["color"], 1.)
            bsdf.inputs["Roughness"].default_value = item["roughness"]
            bsdf.inputs["Metallic"].default_value = item["metallic"]
            material.diffuse_color = (*item["color"], 1.)
            materials[mk] = material
        mesh.materials.append(materials[mk])
        if item["role"] == "robot_visual":
            for polygon in mesh.polygons:
                polygon.use_smooth = True
            mesh.use_auto_smooth = True
        else:
            bpy.context.view_layer.objects.active = obj
            obj.select_set(True)
            bpy.ops.rigidbody.object_add()
            obj.rigid_body.type = "ACTIVE"
            obj.rigid_body.mass = config["cube_mass_kg"]
            obj.rigid_body.friction = .6
            obj.rigid_body.collision_shape = "CONVEX_HULL"
            obj.select_set(False)

for item in data["cameras"]:
    camera = bpy.data.cameras.new("Sim_"+item["name"])
    camera.lens = item["focal_mm"]
    camera.sensor_width = item["sensor_width_mm"]
    camera.sensor_fit = "HORIZONTAL"
    obj = bpy.data.objects.new(camera.name, camera)
    collections["cameras"].objects.link(obj)
    obj.matrix_world = Matrix(item["matrix_world"])
    obj["optics_note"] = "Review camera only; exact OpenCV lens model is authored in the Isaac USD asset."
    if item["name"] == "ReviewSetup":
        scene.camera = obj
collections["environment_collision"].hide_viewport = True
for name in ["02 West wall", "06 Ceiling tiles and services"]:
    if bpy.data.collections.get(name):
        bpy.data.collections[name].hide_viewport = True
if bpy.data.collections.get("04 Entry door and glazed partition"):
    bpy.data.collections["04 Entry door and glazed partition"].hide_viewport = False
if scene.rigidbody_world:
    scene.rigidbody_world.substeps_per_frame = 32
    scene.rigidbody_world.solver_iterations = 16
scene["physics_status"] = "Environment collision review and task object; articulated robot control and three-camera data collection run in Isaac Sim through room01_sim."
scene["room_floor_z"] = 0.
scene["simulation_scene"] = "assets/room01/sim/room01_manipulation.usda"
scene["camera_calibration_status"] = config["camera_calibration_status"]
scene["reset_pose_profile"] = config["reset_pose"]["profile"]
scene["joint_home_rad_json"] = json.dumps(config["joint_home_rad"], sort_keys=True)
from mathutils import Vector
for screen in bpy.data.screens:
    for area in screen.areas:
        if area.type == "VIEW_3D":
            space = area.spaces.active
            space.region_3d.view_location = Vector((.65, -1.94, .85))
            space.region_3d.view_rotation = Vector((-2.05, 1.14, .75)).to_track_quat('Z', 'Y')
            space.region_3d.view_distance = 3.7
            space.overlay.show_extras = False
bpy.ops.file.pack_all()
out = ROOT / "assets/room01/sim/room01_sim_review.blend"
bpy.ops.wm.save_as_mainfile(filepath=str(out))
print("SIMULATION_REVIEW_BLEND_SAVED", str(out), len(scene.objects), flush=True)
