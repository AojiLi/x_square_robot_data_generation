#!/usr/bin/env python3
"""Package the trained visual scene, source mesh and calibrated scan cameras.

Run with .venv-usd/bin/python after converting the Gaussian PLY to gaussians.usdc.
This does not invent collision geometry, robot poses or physical measurements.
"""

import json
import struct
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdRender, UsdShade, UsdVol, Vt

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets/room01"
DATA = ROOT / "data/room01"


def read_glb():
    payload = (ASSETS / "scan_original.glb").read_bytes()
    size = struct.unpack_from("<I", payload, 12)[0]
    return json.loads(payload[20:20+size]), payload[28+size:]


def accessor(gltf, binary, index):
    item = gltf["accessors"][index]
    view = gltf["bufferViews"][item["bufferView"]]
    dtype = {5126: "<f4", 5125: "<u4", 5123: "<u2"}[item["componentType"]]
    components = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}[item["type"]]
    element_size = np.dtype(dtype).itemsize
    offset = view.get("byteOffset", 0)+item.get("byteOffset", 0)
    stride = view.get("byteStride", components*element_size)
    return np.ndarray((item["count"], components), dtype=dtype, buffer=binary,
                      offset=offset, strides=(stride, element_size)).copy()


def export_reference_mesh():
    gltf, binary = read_glb()
    target = ASSETS / "scan_mesh.usdc"
    stage = Usd.Stage.CreateNew(str(target))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    root = UsdGeom.Xform.Define(stage, "/ScanMesh")
    stage.SetDefaultPrim(root.GetPrim())
    matrix = np.array(json.loads((ASSETS / "coordinate_transforms.json").read_text())["world_from_glb"])
    (ASSETS / "textures").mkdir(exist_ok=True)
    materials = []
    for i, source in enumerate(gltf["materials"]):
        material = UsdShade.Material.Define(stage, f"/ScanMesh/Materials/Material_{i}")
        shader = UsdShade.Shader.Define(stage, f"{material.GetPath()}/Surface")
        shader.CreateIdAttr("UsdPreviewSurface")
        pbr = source.get("pbrMetallicRoughness", {})
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(pbr.get("roughnessFactor", 1)))
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(pbr.get("metallicFactor", 1)))
        factor = pbr.get("baseColorFactor", [1, 1, 1, 1])
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*factor[:3]))
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(factor[3]))
        if "baseColorTexture" in pbr:
            image_index = gltf["textures"][pbr["baseColorTexture"]["index"]]["source"]
            image = gltf["images"][image_index]
            view = gltf["bufferViews"][image["bufferView"]]
            suffix = ".jpg" if image["mimeType"] == "image/jpeg" else ".png"
            texture_path = Path("textures") / f"material_{i}{suffix}"
            start = view.get("byteOffset", 0)
            (ASSETS / texture_path).write_bytes(binary[start:start+view["byteLength"]])
            texture = UsdShade.Shader.Define(stage, f"{material.GetPath()}/Texture")
            texture.CreateIdAttr("UsdUVTexture")
            texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texture_path.as_posix()))
            texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
            texture.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*factor))
            reader = UsdShade.Shader.Define(stage, f"{material.GetPath()}/UV")
            reader.CreateIdAttr("UsdPrimvarReader_float2")
            reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(texture.ConnectableAPI(), "rgb")
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        materials.append(material)
    vertex_count, triangle_count = 0, 0
    for i, mesh_source in enumerate(gltf["meshes"]):
        for j, primitive in enumerate(mesh_source["primitives"]):
            if primitive.get("mode", 4) != 4:
                raise ValueError("Only triangle primitives are supported in this capture")
            mesh = UsdGeom.Mesh.Define(stage, f"/ScanMesh/Part_{i}_{j}")
            points = accessor(gltf, binary, primitive["attributes"]["POSITION"])
            points = (points @ matrix[:3, :3].T+matrix[:3, 3]).astype(np.float32)
            indices = accessor(gltf, binary, primitive["indices"]).reshape(-1).astype(np.int32)
            mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points))
            mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(indices))
            mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(indices)//3, 3, dtype=np.int32)))
            mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            mesh.CreateExtentAttr(Vt.Vec3fArray.FromNumpy(np.stack([points.min(0), points.max(0)])))
            mat_index = primitive.get("material", 0)
            mesh.CreateDoubleSidedAttr(bool(gltf["materials"][mat_index].get("doubleSided", False)))
            if "NORMAL" in primitive["attributes"]:
                normals = accessor(gltf, binary, primitive["attributes"]["NORMAL"])
                normals = (normals @ matrix[:3, :3].T).astype(np.float32)
                mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(normals))
                mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
            if "TEXCOORD_0" in primitive["attributes"]:
                uv = accessor(gltf, binary, primitive["attributes"]["TEXCOORD_0"])
                uv[:, 1] = 1-uv[:, 1]
                UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                                       UsdGeom.Tokens.vertex).Set(Vt.Vec2fArray.FromNumpy(uv))
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(materials[mat_index])
            vertex_count += len(points)
            triangle_count += len(indices)//3
    stage.GetRootLayer().customLayerData = {"purpose": "Source scan reference for geometry review; no collision APIs authored"}
    stage.GetRootLayer().Save()
    return {"vertices": vertex_count, "triangles": triangle_count, "materials": len(materials)}


def set_camera(camera, frame, time=Usd.TimeCode.Default()):
    w, h = frame["w"]/2, frame["h"]/2
    fx, fy, cx, cy = (frame[key]/2 for key in ["fl_x", "fl_y", "cx", "cy"])
    focal = 20.0
    camera.CreateFocalLengthAttr().Set(focal, time)
    camera.CreateHorizontalApertureAttr().Set(focal*w/fx, time)
    camera.CreateVerticalApertureAttr().Set(focal*h/fy, time)
    camera.CreateHorizontalApertureOffsetAttr().Set((w/2-cx)*focal/fx, time)
    camera.CreateVerticalApertureOffsetAttr().Set((cy-h/2)*focal/fy, time)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.))
    camera.CreateProjectionAttr(UsdGeom.Tokens.perspective)
    matrix = Gf.Matrix4d(np.array(frame["transform_matrix"]).T.tolist())
    xform = UsdGeom.Xformable(camera.GetPrim())
    operations = xform.GetOrderedXformOps()
    operation = operations[0] if operations else xform.AddTransformOp()
    operation.Set(matrix, time)


def main():
    mesh_summary = export_reference_mesh()
    frames = json.loads((DATA / "transforms.json").read_text())["frames"]
    splat_stage = Usd.Stage.Open(str(ASSETS / "gaussians.usdc"))
    splat = UsdVol.ParticleField3DGaussianSplat(splat_stage.GetDefaultPrim())
    if not splat:
        raise ValueError("Gaussian asset is not a typed ParticleField3DGaussianSplat")
    expected_gaussians = len(splat.GetPositionsAttr().Get())
    stage = Usd.Stage.CreateNew(str(ASSETS / "room01_scene.usda"))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    stage.DefinePrim("/World/RoomGaussian").GetReferences().AddReference("gaussians.usdc")
    preview = UsdGeom.Camera.Define(stage, "/World/ScanCamera")
    set_camera(preview, frames[4])
    replay = UsdGeom.Camera.Define(stage, "/World/CaptureReplay")
    set_camera(replay, frames[0])
    first_timestamp = int(Path(frames[0]["file_path"]).stem)
    stage.SetTimeCodesPerSecond(30)
    stage.SetFramesPerSecond(30)
    stage.SetStartTimeCode(0)
    trajectories = {"opengl": [], "opencv": []}
    for frame in frames:
        seconds = (int(Path(frame["file_path"]).stem)-first_timestamp)/1e6
        set_camera(replay, frame, Usd.TimeCode(seconds*30))
        gl = np.array(frame["transform_matrix"])
        for convention, matrix in [("opengl", gl), ("opencv", gl @ np.diag([1., -1., -1., 1.]))]:
            q = Gf.Matrix4d(matrix.T.tolist()).ExtractRotationQuat()
            xyz = q.GetImaginary()
            values = [seconds, *matrix[:3, 3], *xyz, q.GetReal()]
            trajectories[convention].append(" ".join(f"{float(value):.9f}" for value in values))
    stage.SetEndTimeCode(seconds*30)
    settings = UsdRender.Settings.Define(stage, "/Render/Settings")
    settings.CreateCameraRel().SetTargets([preview.GetPath()])
    settings.CreateResolutionAttr(Gf.Vec2i(496, 368))
    stage.SetMetadata("renderSettingsPrimPath", str(settings.GetPath()))
    stage.GetRootLayer().customLayerData = {
        "sceneStatus": "Visual reconstruction; collision geometry and robot calibration pending",
        "sourceScanMesh": "scan_mesh.usdc", "coordinateManifest": "coordinate_transforms.json",
        "captureTimestampOriginMicroseconds": str(first_timestamp),
        "cameraNote": "ScanCamera is capture view 5; CaptureReplay includes per-frame intrinsics and poses",
    }
    from configure_scene_preview import add_review_setup
    add_review_setup(stage, frames)
    stage.GetRootLayer().Save()
    for convention, lines in trajectories.items():
        (ASSETS / f"scan_camera_{convention}.tum").write_text(
            f"# Relative seconds, meters, camera-to-world; Z-up world, {convention} camera axes\n"
            "# timestamp tx ty tz qx qy qz qw\n"+"\n".join(lines)+"\n")

    # Check the SDK's actual camera projection, not just the authored properties.
    frame = frames[4]
    test_points = np.array([[0.1, 0.05, -1.], [-0.2, 0.1, -2.], [0.3, -0.2, -1.5]])
    frustum = preview.GetCamera().frustum
    projection = np.array(frustum.ComputeProjectionMatrix())
    homogeneous = np.column_stack([test_points, np.ones(len(test_points))]) @ projection
    ndc = homogeneous[:, :3]/homogeneous[:, 3:4]
    usd_pixels = np.column_stack([(ndc[:, 0]+1)*496/2, (1-ndc[:, 1])*368/2])
    expected = np.column_stack([frame["fl_x"]/2*test_points[:, 0]/-test_points[:, 2]+frame["cx"]/2,
                                frame["cy"]/2-frame["fl_y"]/2*test_points[:, 1]/-test_points[:, 2]])
    projection_error = float(np.abs(usd_pixels-expected).max())
    if projection_error > 0.001:
        raise ValueError(f"USD camera does not match calibrated intrinsics: {projection_error}px")
    reopened = Usd.Stage.Open(str(ASSETS / "room01_scene.usda"))
    imported = UsdVol.ParticleField3DGaussianSplat(reopened.GetPrimAtPath("/World/RoomGaussian"))
    n = len(imported.GetPositionsAttr().Get())
    if n != expected_gaussians or UsdGeom.GetStageMetersPerUnit(reopened) != 1 or UsdGeom.GetStageUpAxis(reopened) != "Z":
        raise ValueError("Unexpected Gaussian count, units or up axis after reopening")
    checks = {"openusd_version": list(Usd.GetVersion()), "typed_particle_field": True, "gaussians": n,
              "stage_units_m": 1., "stage_up_axis": "Z", "camera_projection_max_error_pixels": projection_error,
              "source_mesh": mesh_summary, "capture_views": len(frames), "relative_duration_s": seconds,
              "collision_apis_authored": False, "isaac_sim_runtime_render_test": "not yet run"}
    target = ROOT / "reports/room01_reconstruction/usd_scene_checks.json"
    target.write_text(json.dumps(checks, indent=2)+"\n")
    print(json.dumps(checks, indent=2))


if __name__ == "__main__":
    main()
