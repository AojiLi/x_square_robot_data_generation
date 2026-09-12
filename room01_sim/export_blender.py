"""Extract evaluated geometry and bake color textures from the saved Blender model.

Run in Blender with the source .blend already open. The source file is never saved.
The intermediate package is consumed by build_scene.py in the OpenUSD environment.
"""
from pathlib import Path
import hashlib
import json
import sys

import bpy
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "assets/room01/sim"
TEXTURES = OUT / "textures"
OUT.mkdir(parents=True, exist_ok=True)
TEXTURES.mkdir(exist_ok=True)
scene = bpy.context.scene
source = Path(bpy.data.filepath)

for collection in bpy.data.collections:
    if not collection.name.startswith("99 "):
        collection.hide_viewport = False
for layer in bpy.context.view_layer.layer_collection.children:
    if not layer.name.startswith("99 "):
        layer.exclude = False
        layer.hide_viewport = False
bpy.context.view_layer.update()

objects = sorted([
    obj for obj in scene.objects
    if obj.type in {"MESH", "CURVE", "FONT"}
    and not any(c.name.startswith("99 ") for c in obj.users_collection)
], key=lambda obj: obj.name)


def file_name(name):
    return "".join(c if c.isalnum() or c in "_-" else "_" for c in name)


def principled(material):
    return next((n for n in material.node_tree.nodes if n.type == "BSDF_PRINCIPLED"), None)


def ensure_uv(obj):
    if obj.type != "MESH" or obj.data.uv_layers:
        return
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.uv.smart_project(island_margin=0.015)
    bpy.ops.object.mode_set(mode="OBJECT")


def bake_procedural(material, material_objects):
    representative = max(material_objects, key=lambda o: float(np.prod(np.maximum(o.dimensions, .01))))
    ensure_uv(representative)
    base = principled(material).inputs["Base Color"]
    nodes, links = material.node_tree.nodes, material.node_tree.links
    output = next(n for n in nodes if n.type == "OUTPUT_MATERIAL")
    original = [(l.from_socket, l.to_socket) for l in output.inputs["Surface"].links]
    emission = nodes.new("ShaderNodeEmission")
    if base.is_linked:
        links.new(base.links[0].from_socket, emission.inputs["Color"])
    else:
        emission.inputs["Color"].default_value = base.default_value
    links.new(emission.outputs[0], output.inputs["Surface"])
    image = bpy.data.images.new("USD_Bake_"+material.name, width=1024, height=1024, alpha=False)
    image.colorspace_settings.name = "sRGB"
    target = nodes.new("ShaderNodeTexImage")
    target.image = image
    for node in nodes:
        node.select = False
    target.select = True
    nodes.active = target
    bpy.ops.object.select_all(action="DESELECT")
    representative.select_set(True)
    bpy.context.view_layer.objects.active = representative
    scene.render.engine = "CYCLES"
    scene.cycles.samples = 1
    scene.render.bake.margin = 8
    scene.render.bake.use_clear = True
    bpy.ops.object.bake(type="EMIT")
    path = TEXTURES / (file_name(material.name)+"_basecolor.png")
    image.filepath_raw = str(path)
    image.file_format = "PNG"
    image.save()
    nodes.remove(target)
    nodes.remove(emission)
    for start, end in original:
        links.new(start, end)
    print("BAKED_COLOR", material.name, representative.name, flush=True)
    return str(path.relative_to(OUT))


materials = {}
used_materials = {mat for obj in objects for mat in obj.data.materials if mat is not None}
for mat in sorted(used_materials, key=lambda m: m.name):
    node = principled(mat) if mat.use_nodes else None
    color = tuple(node.inputs["Base Color"].default_value[:3]) if node else tuple(mat.diffuse_color[:3])
    item = {
        "name": mat.name, "color": color,
        "roughness": float(node.inputs["Roughness"].default_value) if node else .6,
        "metallic": float(node.inputs["Metallic"].default_value) if node else 0.,
        "transmission": float(node.inputs["Transmission Weight"].default_value) if node else 0.,
        "ior": float(node.inputs["IOR"].default_value) if node else 1.5,
        "opacity": float(node.inputs["Alpha"].default_value) if node else 1.,
        "color_texture": None, "opacity_texture_channel": None,
        "normal_bump_baked": False,
    }
    if node and node.inputs["Base Color"].is_linked:
        upstream = node.inputs["Base Color"].links[0].from_node
        if upstream.type == "TEX_IMAGE" and upstream.image:
            image = upstream.image
            suffix = Path(image.filepath).suffix.lower() or ".png"
            path = TEXTURES / (file_name(mat.name)+suffix)
            if image.packed_file:
                path.write_bytes(bytes(image.packed_file.data))
            else:
                path.write_bytes(Path(bpy.path.abspath(image.filepath)).read_bytes())
            item["color_texture"] = str(path.relative_to(OUT))
            item["color"] = [1., 1., 1.]
        else:
            related = [o for o in objects if mat.name in o.data.materials and o.type == "MESH"]
            item["color_texture"] = bake_procedural(mat, related)
            item["color"] = [1., 1., 1.]
    # The white care-icon decal uses source luminance as opacity in Blender.
    if node and node.inputs["Alpha"].is_linked:
        source_image = next((n.image for n in mat.node_tree.nodes if n.type == "TEX_IMAGE" and n.image), None)
        if source_image:
            path = TEXTURES / (file_name(mat.name)+"_opacity.png")
            if source_image.packed_file:
                path.write_bytes(bytes(source_image.packed_file.data))
                item["opacity_texture"] = str(path.relative_to(OUT))
                item["opacity_texture_channel"] = "r"
    materials[mat.name] = item

arrays = {}
records = []
depsgraph = bpy.context.evaluated_depsgraph_get()
for index, obj in enumerate(objects):
    ensure_uv(obj)
    evaluated = obj.evaluated_get(depsgraph)
    mesh = evaluated.to_mesh(preserve_all_data_layers=True, depsgraph=depsgraph)
    if mesh is None or len(mesh.polygons) == 0:
        evaluated.to_mesh_clear()
        continue
    mesh.calc_loop_triangles()
    mesh.calc_normals_split()
    prefix = f"m{index:04d}"
    points = np.empty(len(mesh.vertices)*3, dtype=np.float32)
    mesh.vertices.foreach_get("co", points)
    points = points.reshape(-1, 3)
    triangles = np.array([t.vertices[:] for t in mesh.loop_triangles], dtype=np.int32)
    triangle_loops = np.array([t.loops[:] for t in mesh.loop_triangles], dtype=np.int32)
    normals = np.empty(len(mesh.loops)*3, dtype=np.float32)
    mesh.loops.foreach_get("normal", normals)
    arrays[prefix+"_points"] = points
    arrays[prefix+"_triangles"] = triangles
    arrays[prefix+"_normals"] = normals.reshape(-1, 3)[triangle_loops].reshape(-1, 3)
    if mesh.uv_layers.active:
        uv = np.empty(len(mesh.loops)*2, dtype=np.float32)
        mesh.uv_layers.active.data.foreach_get("uv", uv)
        arrays[prefix+"_uv"] = uv.reshape(-1, 2)[triangle_loops].reshape(-1, 2)
    arrays[prefix+"_material_indices"] = np.array([t.material_index for t in mesh.loop_triangles], dtype=np.int32)
    matrix = np.array(obj.matrix_world, dtype=float)
    low, high = points.min(axis=0), points.max(axis=0)
    ancestors = []
    parent = obj.parent
    while parent:
        ancestors.append(parent.name)
        parent = parent.parent
    records.append({
        "name": obj.name, "key": prefix,
        "collections": [c.name for c in obj.users_collection], "ancestors": ancestors,
        "matrix_world": matrix.tolist(), "bounds_local": [low.tolist(), high.tolist()],
        "materials": [m.name if m else None for m in mesh.materials],
        "vertices": len(points), "triangles": len(triangles),
    })
    evaluated.to_mesh_clear()

np.savez_compressed(OUT / "evaluated_geometry.npz", **arrays)
manifest = {
    "source": str(source.relative_to(ROOT)), "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
    "blender_version": bpy.app.version_string, "meters_per_unit": scene.unit_settings.scale_length,
    "floor_z_source": float(scene["room_floor_z"]), "ceiling_z_source": float(scene["room_ceiling_z"]),
    "objects": records, "materials": materials,
    "vertices": sum(r["vertices"] for r in records), "triangles": sum(r["triangles"] for r in records),
    "material_scope": "Base colors, photographic textures, baked procedural base colors, metal/roughness. Procedural micro-normal detail is not baked.",
}
(OUT / "geometry_manifest.json").write_text(json.dumps(manifest, indent=2))
print("BLENDER_EXPORTED", len(records), manifest["vertices"], manifest["triangles"], flush=True)
