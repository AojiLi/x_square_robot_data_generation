"""Parameterized foam-and-battery task assets, in metres, authored using OpenUSD.

The parent frame has +X to the photo's right, +Y toward the rear of the
white table, and +Z up. All returned poses are in that parent frame. The
three red-foam holes are blind *geometric cavities*: their collider is a
static triangle mesh with ``approximation=none``, never a convex hull.

External dimensions, battery diameter and hole diameter/depth come from the
user's measurements. Placement, hole pitch, mass, friction and rigid foams
are provisional; this module does not simulate foam deformation or claim
that these physical properties have been calibrated.
"""
from __future__ import annotations

from copy import deepcopy
import json
import math
from typing import Any

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt


DEFAULT_SPEC = {
    "table_top_z_m": 0.740,
    "yellow_dims_m": [0.080, 0.098, 0.052],
    "yellow_positions_xy_m": [[-0.260, -0.060], [0.260, -0.060]],
    "yellow_yaw_deg": [0.0, 0.0],
    "yellow_bevel_m": 0.0015,
    "battery_diameter_m": 0.0125,
    "battery_length_m": 0.049,
    "battery_mass_kg": 0.020,
    "battery_yaw_deg": [55.0, 15.0],
    "battery_offsets_xy_m": [[0.0, 0.0], [0.0, 0.0]],
    "battery_poses": None,
    "red_dims_m": [0.072, 0.048, 0.048],
    "red_position_xy_m": [0.0, -0.060],
    "red_yaw_deg": 0.0,
    "hole_diameter_m": 0.0125,
    "hole_depth_m": 0.013,
    "hole_pitch_m": 0.020,
    "battery_contact_offset_m": 0.0002,
    "red_contact_offset_m": 0.0002,
    "yellow_contact_offset_m": 0.0005,
    "tape_visual": True,
}


def _positive(value, name):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _dims(value, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)) or np.any(result <= 0):
        raise ValueError(f"{name} must contain three finite positive dimensions")
    return result


def _quaternion(value):
    """Validate and normalize a quaternion supplied in WXYZ order."""
    q = np.asarray(value, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-10:
        raise ValueError("quaternion_wxyz must be a finite nonzero quaternion")
    return q / np.linalg.norm(q)


def _yaw(degrees):
    angle = math.radians(float(degrees)) * 0.5
    if not math.isfinite(angle):
        raise ValueError("yaw must be finite")
    return np.array([math.cos(angle), 0.0, 0.0, math.sin(angle)])


def _mesh_builder():
    """Weld exact shared boundaries without depending on a meshing library."""
    points, faces, indices = [], [], {}

    def vertex(point):
        key = tuple(round(float(v), 12) for v in point)
        if key not in indices:
            indices[key] = len(points)
            points.append(tuple(float(v) for v in point))
        return indices[key]

    def triangle(a, b, c):
        ia, ib, ic = vertex(a), vertex(b), vertex(c)
        if len({ia, ib, ic}) == 3:
            faces.append((ia, ib, ic))

    def quad(a, b, c, d):
        triangle(a, b, c)
        triangle(a, c, d)

    return points, faces, triangle, quad


def red_foam_mesh(dimensions=(0.072, 0.048, 0.048), hole_diameter=0.0125,
                  hole_depth=0.013, hole_pitch=0.020, edge_segments=24):
    """Return a closed, outward-wound mesh of one solid with three blind bores.

    Each top rectangle surrounds one circular opening. Identical boundary
    sampling welds the three top patches into one surface. Every bore has a
    cylindrical sidewall and a bottom; there are no top-cover triangles.
    """
    length, width, height = _dims(dimensions, "red_dims_m")
    radius = _positive(hole_diameter, "hole_diameter_m") / 2.0
    depth = _positive(hole_depth, "hole_depth_m")
    pitch = _positive(hole_pitch, "hole_pitch_m")
    if depth >= height:
        raise ValueError("hole_depth_m must be smaller than red foam height for blind holes")
    if radius >= width / 2 or radius >= pitch / 2 or pitch + radius >= length / 2:
        raise ValueError("hole diameter/pitch must leave material between bores and outer walls")
    if int(edge_segments) != edge_segments or edge_segments < 4:
        raise ValueError("edge_segments must be an integer >= 4")
    edge_segments = int(edge_segments)
    top, bottom, bore_bottom = height / 2, -height / 2, height / 2 - depth
    points, faces, tri, quad = _mesh_builder()
    bounds = [-length / 2, -pitch / 2, pitch / 2, length / 2]
    for cell, center_x in enumerate((-pitch, 0.0, pitch)):
        x0, x1 = bounds[cell:cell + 2]
        y0, y1 = -width / 2, width / 2
        corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        outer = []
        for side in range(4):
            start = np.array(corners[side])
            stop = np.array(corners[(side + 1) % 4])
            for index in range(edge_segments):
                xy = start + (stop - start) * index / edge_segments
                outer.append(np.array([xy[0], xy[1], top]))
        inner = []
        for point in outer:
            direction = point[:2] - np.array([center_x, 0.0])
            direction *= radius / np.linalg.norm(direction)
            inner.append(np.array([center_x + direction[0], direction[1], top]))
        for index, a in enumerate(outer):
            next_index = (index + 1) % len(outer)
            b, c, d = outer[next_index], inner[next_index], inner[index]
            quad(a, b, c, d)
            # Inner wall normals face the empty bore. Bottom cap faces up.
            low_d = np.array([d[0], d[1], bore_bottom])
            low_c = np.array([c[0], c[1], bore_bottom])
            quad(d, c, low_c, low_d)
            tri([center_x, 0.0, bore_bottom], low_d, low_c)
            side = index // edge_segments
            is_exterior = side in (0, 2) or (side == 3 and cell == 0) or (side == 1 and cell == 2)
            if is_exterior:
                low_a = [a[0], a[1], bottom]
                low_b = [b[0], b[1], bottom]
                quad(a, low_a, low_b, b)
                # A downward-oriented bottom fan shares all sidewall vertices.
                tri([0.0, 0.0, bottom], low_b, low_a)
    return np.asarray(points, dtype=np.float64), np.asarray(faces, dtype=np.int32)


def rounded_box_mesh(dimensions, bevel=0.0015, bevel_segments=4):
    """Rounded cuboid with exact outer extents and no visual-only oversized skin."""
    half = _dims(dimensions, "box dimensions") / 2
    bevel = float(bevel)
    if not 0 < bevel < float(np.min(half)):
        raise ValueError("box bevel must be positive and smaller than half its shortest side")
    inner = half - bevel
    samples = []
    for axis in range(3):
        outer_band = inner[axis] + bevel * np.sin(np.linspace(0, math.pi / 2, bevel_segments + 1))
        samples.append(np.concatenate([-outer_band[::-1], outer_band]))
    points, faces, tri, quad = _mesh_builder()
    for axis in range(3):
        other = [value for value in range(3) if value != axis]
        for sign in (-1, 1):
            grid = []
            for u in samples[other[0]]:
                row = []
                for v in samples[other[1]]:
                    cube = np.zeros(3)
                    cube[axis], cube[other[0]], cube[other[1]] = sign * half[axis], u, v
                    nearest = np.clip(cube, -inner, inner)
                    delta = cube - nearest
                    point = nearest + bevel * delta / np.linalg.norm(delta)
                    row.append(point)
                grid.append(row)
            for i in range(len(grid) - 1):
                for j in range(len(grid[i]) - 1):
                    corners = [grid[i][j], grid[i + 1][j], grid[i + 1][j + 1], grid[i][j + 1]]
                    normal = np.cross(corners[1] - corners[0], corners[2] - corners[0])
                    if normal[axis] * sign < 0:
                        corners.reverse()
                    quad(*corners)
    return np.asarray(points, dtype=np.float64), np.asarray(faces, dtype=np.int32)


def _attribute(prim, name, value, value_type=Sdf.ValueTypeNames.Float):
    prim.CreateAttribute(name, value_type, custom=False).Set(value)


def _preview_material(stage, path, color, roughness, metallic=0.0, opacity=1.0):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _physics_material(stage, path, static, dynamic):
    material = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(static)
    api.CreateDynamicFrictionAttr(dynamic)
    api.CreateRestitutionAttr(0.0)
    material.GetPrim().AddAppliedSchema("PhysxMaterialAPI")
    _attribute(material.GetPrim(), "physxMaterial:frictionCombineMode", "average", Sdf.ValueTypeNames.Token)
    return material


def _bind(prim, material):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def _collision(prim, material, contact, mesh=False):
    UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
    if mesh:
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("none")
    prim.AddAppliedSchema("PhysxCollisionAPI")
    _attribute(prim, "physxCollision:contactOffset", float(contact))
    _attribute(prim, "physxCollision:restOffset", 0.0)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")


def _pose(stage, path, position, quaternion_wxyz):
    position = np.asarray(position, dtype=float)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("position_m must have three finite values")
    q = _quaternion(quaternion_wxyz)
    prim = UsdGeom.Xform.Define(stage, path)
    prim.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*position))
    prim.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[0]), Gf.Vec3d(*q[1:])))
    return prim.GetPrim(), position, q


def _author_mesh(stage, path, points, faces, material):
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(points, dtype=np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(faces), 3, dtype=np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.asarray(faces, dtype=np.int32).ravel()))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateExtentAttr(Vt.Vec3fArray.FromNumpy(np.asarray([points.min(0), points.max(0)], dtype=np.float32)))
    mesh.CreateDoubleSidedAttr(False)
    _bind(mesh.GetPrim(), material)
    return mesh.GetPrim()


def _cube(stage, path, position, dimensions, material):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*position))
    cube.AddScaleOp().Set(Gf.Vec3f(*dimensions))
    _bind(cube.GetPrim(), material)
    return cube.GetPrim()


def _cylinder(stage, path, length, radius, x, material=None):
    cylinder = UsdGeom.Cylinder.Define(stage, path)
    cylinder.CreateAxisAttr("X")
    cylinder.CreateHeightAttr(float(length))
    cylinder.CreateRadiusAttr(float(radius))
    cylinder.AddTranslateOp().Set(Gf.Vec3d(float(x), 0.0, 0.0))
    cylinder.CreateExtentAttr([Gf.Vec3f(-length / 2, -radius, -radius), Gf.Vec3f(length / 2, radius, radius)])
    if material is not None:
        _bind(cylinder.GetPrim(), material)
    return cylinder.GetPrim()


def author_props(stage, parent_path, spec=None):
    """Author the five task objects and return JSON-serializable metadata.

    ``spec`` partially overrides :data:`DEFAULT_SPEC`. ``battery_poses`` may
    be a list of two dictionaries, each containing ``position_m`` and/or
    ``quaternion_wxyz`` in the parent frame. Defaults place each battery on
    its matching yellow block, with the local battery cylinder along +X.
    Metadata quaternions use XYZW and explicitly carry their order.

    An existing parent may be supplied (e.g. a rotated table frame). This
    function never clears or changes its transform. Returned coordinates
    therefore require the parent transform before use by a world-frame API.
    """
    if not isinstance(stage, Usd.Stage):
        raise TypeError("stage must be an open Usd.Stage")
    parent_path = str(parent_path).rstrip("/")
    path = Sdf.Path(parent_path)
    if not path.IsAbsolutePath() or not path.IsPrimPath() or parent_path == "":
        raise ValueError("parent_path must be an absolute USD prim path")
    values = deepcopy(DEFAULT_SPEC)
    if spec:
        unknown = set(spec) - set(values)
        if unknown:
            raise ValueError("Unknown battery asset spec fields: " + ", ".join(sorted(unknown)))
        values.update(deepcopy(spec))
    yellow = _dims(values["yellow_dims_m"], "yellow_dims_m")
    red = _dims(values["red_dims_m"], "red_dims_m")
    diameter = _positive(values["battery_diameter_m"], "battery_diameter_m")
    length = _positive(values["battery_length_m"], "battery_length_m")
    mass = _positive(values["battery_mass_kg"], "battery_mass_kg")
    top_z = float(values["table_top_z_m"])
    if not math.isfinite(top_z):
        raise ValueError("table_top_z_m must be finite")
    for key in ("battery_contact_offset_m", "red_contact_offset_m", "yellow_contact_offset_m"):
        _positive(values[key], key)
    xy = np.asarray(values["yellow_positions_xy_m"], dtype=float)
    if xy.shape != (2, 2) or not np.all(np.isfinite(xy)):
        raise ValueError("yellow_positions_xy_m must contain two finite XY pairs")
    offsets = np.asarray(values["battery_offsets_xy_m"], dtype=float)
    if offsets.shape != (2, 2) or not np.all(np.isfinite(offsets)):
        raise ValueError("battery_offsets_xy_m must contain two finite XY pairs")
    red_xy = np.asarray(values["red_position_xy_m"], dtype=float)
    if red_xy.shape != (2,) or not np.all(np.isfinite(red_xy)):
        raise ValueError("red_position_xy_m must be a finite XY pair")
    if len(values["battery_yaw_deg"]) != 2 or len(values["yellow_yaw_deg"]) != 2:
        raise ValueError("battery_yaw_deg and yellow_yaw_deg must each contain two angles")
    poses = values["battery_poses"] or [{}, {}]
    if len(poses) != 2 or any(not isinstance(item, dict) for item in poses):
        raise ValueError("battery_poses must contain two pose dictionaries")
    red_points, red_faces = red_foam_mesh(red, values["hole_diameter_m"], values["hole_depth_m"], values["hole_pitch_m"])
    yellow_points, yellow_faces = rounded_box_mesh(yellow, values["yellow_bevel_m"])
    # Validate all pose overrides before creating any prims.
    battery_positions, battery_rotations = [], []
    for i, pose in enumerate(poses):
        position = np.asarray(pose.get("position_m", [xy[i, 0] + offsets[i, 0], xy[i, 1] + offsets[i, 1], top_z + yellow[2] + diameter / 2]), dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("battery position_m must contain three finite values")
        quaternion = _quaternion(pose.get("quaternion_wxyz", _yaw(values["battery_yaw_deg"][i])))
        battery_positions.append(position)
        battery_rotations.append(quaternion)
    UsdGeom.Xform.Define(stage, parent_path)
    UsdGeom.Scope.Define(stage, parent_path + "/Looks")
    UsdGeom.Scope.Define(stage, parent_path + "/PhysicsMaterials")
    look_path = parent_path + "/Looks/"
    looks = {
        "yellow": _preview_material(stage, look_path + "YellowFoam", (0.93, 0.75, 0.08), 0.91),
        "red": _preview_material(stage, look_path + "DarkRedFoam", (0.24, 0.035, 0.031), 0.88),
        "blue": _preview_material(stage, look_path + "CyanBatteryWrap", (0.010, 0.36, 0.59), 0.29, 0.16),
        "metal": _preview_material(stage, look_path + "SilverTerminals", (0.62, 0.65, 0.67), 0.23, 0.88),
        "tape": _preview_material(stage, look_path + "TranslucentTape", (0.82, 0.84, 0.80), 0.38, opacity=0.45),
    }
    physics_path = parent_path + "/PhysicsMaterials/"
    physics = {
        "yellow": _physics_material(stage, physics_path + "RigidYellowFoam", 0.85, 0.70),
        "red": _physics_material(stage, physics_path + "RigidRedFoam", 0.65, 0.50),
        "battery": _physics_material(stage, physics_path + "Battery", 0.55, 0.40),
    }
    metadata = {
        "parent_path": parent_path,
        "coordinate_frame": "parent local: +X photo right, +Y table rear, +Z up; apply parent transform for world poses",
        "units": "metres, kilograms, degrees",
        "quaternion_order": "xyzw",
        "spec": values,
        "yellow_foams": [],
        "batteries": [],
        "assumptions": [
            "Default battery diameter 0.0125 m and length 0.049 m are user-confirmed measurements.",
            "Default hole diameter 0.0125 m and depth 0.013 m are user-confirmed; 0.020 m pitch remains a photo estimate.",
            "Equal nominal battery/hole diameters imply zero clearance; rigid models do not reproduce compliant foam insertion.",
            "XY placement and battery yaw are photo estimates, not surveyed poses.",
            "Both foams use rigid static collision; no foam deformation or insertion compliance is simulated.",
            "Red foam is assumed fixed to the table based on visible tape; confirm fixation.",
            "Battery mass, all friction values and contact parameters are uncalibrated estimates.",
        ],
    }
    for i in range(2):
        foam_path = parent_path + f"/YellowFoam_{i + 1}"
        position = [float(xy[i, 0]), float(xy[i, 1]), float(top_z + yellow[2] / 2)]
        foam, _, q = _pose(stage, foam_path, position, _yaw(values["yellow_yaw_deg"][i]))
        foam.SetCustomData({"role": "yellow support foam", "physics_model": "static rigid; deformation not simulated"})
        body = _author_mesh(stage, foam_path + "/Body", yellow_points, yellow_faces, looks["yellow"])
        _collision(body, physics["yellow"], values["yellow_contact_offset_m"], mesh=True)
        metadata["yellow_foams"].append({"path": foam_path, "local_position_m": position,
                                        "local_quaternion_xyzw": [*q[1:].tolist(), float(q[0])], "size_m": yellow.tolist()})
        battery_path = parent_path + f"/Battery_{i + 1}"
        battery, pos, q = _pose(stage, battery_path, battery_positions[i], battery_rotations[i])
        rb = UsdPhysics.RigidBodyAPI.Apply(battery)
        rb.CreateRigidBodyEnabledAttr(True)
        rb.CreateKinematicEnabledAttr(False)
        UsdPhysics.MassAPI.Apply(battery).CreateMassAttr(mass)
        battery.AddAppliedSchema("PhysxRigidBodyAPI")
        _attribute(battery, "physxRigidBody:enableCCD", True, Sdf.ValueTypeNames.Bool)
        _attribute(battery, "physxRigidBody:enableGyroscopicForces", True, Sdf.ValueTypeNames.Bool)
        _attribute(battery, "physxRigidBody:solverPositionIterationCount", 16, Sdf.ValueTypeNames.Int)
        _attribute(battery, "physxRigidBody:solverVelocityIterationCount", 4, Sdf.ValueTypeNames.Int)
        _attribute(battery, "physxRigidBody:sleepThreshold", 0.00001)
        battery.SetCustomData({"role": "graspable battery", "local_axis": "X", "mass_source": "provisional estimate"})
        radius = diameter / 2
        terminal = min(0.001, length * 0.025)
        _cylinder(stage, battery_path + "/Wrap", length - 2 * terminal, radius, 0.0, looks["blue"])
        _cylinder(stage, battery_path + "/NegativeTerminal", terminal, radius * 0.96,
                  -length / 2 + terminal / 2, looks["metal"])
        _cylinder(stage, battery_path + "/PositiveRim", terminal / 2, radius * 0.96,
                  length / 2 - terminal * 0.75, looks["metal"])
        _cylinder(stage, battery_path + "/PositiveButton", terminal / 2, radius * 0.36,
                  length / 2 - terminal * 0.25, looks["metal"])
        collider = _cylinder(stage, battery_path + "/Collision", length, radius, 0.0)
        UsdGeom.Imageable(collider).CreateVisibilityAttr("invisible")
        _collision(collider, physics["battery"], values["battery_contact_offset_m"])
        metadata["batteries"].append({"path": battery_path, "local_position_m": pos.tolist(),
                                      "local_quaternion_xyzw": [*q[1:].tolist(), float(q[0])],
                                      "size_m": [length, diameter, diameter], "mass_kg": mass,
                                      "local_axis": "X", "dynamic": True,
                                      "name": ("BatteryLeft" if i == 0 else "BatteryRight"),
                                      "prim_path": battery_path, "position_local_m": pos.tolist(),
                                      "orientation_local_xyzw": [*q[1:].tolist(), float(q[0])]})
    red_path = parent_path + "/RedFoam"
    red_prim, red_position, q = _pose(stage, red_path, [*red_xy, top_z + red[2] / 2], _yaw(values["red_yaw_deg"]))
    red_prim.SetCustomData({"role": "three-hole insertion fixture", "physics_model": "static rigid blind-bore mesh", "hole_dimensions_source": "default diameter/depth user-confirmed; pitch photo estimate"})
    red_body = _author_mesh(stage, red_path + "/Body", red_points, red_faces, looks["red"])
    _collision(red_body, physics["red"], values["red_contact_offset_m"], mesh=True)
    if values["tape_visual"]:
        for i, x in enumerate((-red[0] * 0.24, red[0] * 0.24)):
            _cube(stage, red_path + f"/TapeFront_{i + 1}", [x, -red[1] / 2 - 0.00012, -red[2] / 4],
                  [0.009, 0.00018, red[2] / 2], looks["tape"])
            _cube(stage, red_path + f"/TapeFoot_{i + 1}", [x, -red[1] / 2 - 0.009, -red[2] / 2 + 0.00012],
                  [0.009, 0.018, 0.00018], looks["tape"])
    yaw = math.radians(values["red_yaw_deg"])
    hole_centers = []
    for x in (-values["hole_pitch_m"], 0.0, values["hole_pitch_m"]):
        hole_centers.append([float(red_xy[0] + x * math.cos(yaw)), float(red_xy[1] + x * math.sin(yaw)), float(top_z + red[2])])
    metadata["red_foam"] = {"path": red_path, "local_position_m": red_position.tolist(),
                             "local_quaternion_xyzw": [*q[1:].tolist(), float(q[0])], "size_m": red.tolist(),
                             "hole_top_centers_parent_m": hole_centers, "hole_diameter_m": values["hole_diameter_m"],
                             "hole_depth_m": values["hole_depth_m"], "hole_pitch_m": values["hole_pitch_m"],
                             "collision_approximation": "none", "dynamic": False,
                             "nominal_radial_clearance_m": (values["hole_diameter_m"] - diameter) / 2,
                             "hole_dimension_sources": {"diameter": "user measurement", "depth": "user measurement", "pitch": "photo estimate"}}
    stage.GetPrimAtPath(parent_path).CreateAttribute("batteryTask:metadataJson", Sdf.ValueTypeNames.String, custom=True).Set(json.dumps(metadata))
    return metadata
