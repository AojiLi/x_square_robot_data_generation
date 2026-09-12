"""Extract the simulation's collision shapes and robot appearance for Blender review."""
from pathlib import Path
import json
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reports/room01_sim"
stage = Usd.Stage.Open(str(ROOT / "assets/room01/sim/room01_manipulation.usda"))
cache = UsdGeom.XformCache()
arrays, records = {}, []
cube_vertices = np.array([[-.5, -.5, -.5], [.5, -.5, -.5], [.5, .5, -.5], [-.5, .5, -.5],
                          [-.5, -.5, .5], [.5, -.5, .5], [.5, .5, .5], [-.5, .5, .5]])
cube_faces = np.array([[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]])

for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
    path = str(prim.GetPath())
    role = None
    if path.startswith("/World/Room/Collisions/") and prim.HasAPI(UsdPhysics.CollisionAPI):
        role = "environment_collision"
    elif path.startswith("/World/Robot/") and "/visuals/" in path and prim.IsA(UsdGeom.Mesh):
        role = "robot_visual"
    elif path == "/World/Task/RedCube/Shape":
        role = "task_cube"
    if role is None:
        continue
    if prim.IsA(UsdGeom.Cube):
        points = cube_vertices*UsdGeom.Cube(prim).GetSizeAttr().Get()
        counts, indices = np.full(6, 4, np.int32), cube_faces.flatten().astype(np.int32)
    elif prim.IsA(UsdGeom.Mesh):
        mesh = UsdGeom.Mesh(prim)
        points = np.asarray(mesh.GetPointsAttr().Get())
        counts, indices = np.asarray(mesh.GetFaceVertexCountsAttr().Get()), np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
    else:
        continue
    matrix = np.asarray(cache.GetLocalToWorldTransform(prim))
    points = (np.c_[points, np.ones(len(points))]@matrix)[:, :3].astype(np.float32)
    key = f"mesh_{len(records):04d}"
    arrays[key+"_points"], arrays[key+"_counts"], arrays[key+"_indices"] = points, counts, indices
    color, roughness, metallic = [.7, .72, .73], .45, 0.
    if role != "environment_collision":
        material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        if material:
            for shader_prim in material.GetPrim().GetChildren():
                shader = UsdShade.Shader(shader_prim)
                for source in ["diffuseColor", "diffuse_color_constant"]:
                    value = shader.GetInput(source).Get() if shader.GetInput(source) else None
                    if value is not None:
                        color = list(value)
                if shader.GetInput("roughness"):
                    roughness = shader.GetInput("roughness").Get()
                if shader.GetInput("metallic"):
                    metallic = shader.GetInput("metallic").Get()
    approximate = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() if prim.HasAPI(UsdPhysics.MeshCollisionAPI) else "box"
    records.append({"key": key, "prim": path, "role": role, "color": color, "roughness": roughness, "metallic": metallic, "approximation": approximate})

cameras = []
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Camera) and ("/RGB_" in str(prim.GetPath()) or prim.GetName() == "ReviewSetup"):
        camera = UsdGeom.Camera(prim)
        cameras.append({"name": prim.GetName(), "matrix_world": np.asarray(cache.GetLocalToWorldTransform(prim)).T.tolist(),
                        "focal_mm": camera.GetFocalLengthAttr().Get(), "sensor_width_mm": camera.GetHorizontalApertureAttr().Get()})
np.savez_compressed(OUT / "review_geometry.npz", **arrays)
(OUT / "review_geometry.json").write_text(json.dumps({"meshes": records, "cameras": cameras}, indent=2))
print("REVIEW_GEOMETRY_EXPORTED", len(records), len(cameras))
