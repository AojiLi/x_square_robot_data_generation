"""Measured white table for the September 2026 battery insertion scene.

All geometry is in metres in the table frame: X is photograph-right, Y is
photograph-back, Z is up, with the floor at Z=0. ``author_table`` does not add a
placement transform. The caller can position its root once in the room frame.
The tabletop is a closed triangle mesh with an actual capsule-shaped opening;
its collision uses the same mesh and never a convex hull or a slot bottom cap.
"""

import math

import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, Vt


DEFAULT_SPEC = {
    "length_m": 1.10,
    "depth_m": 0.57,
    "height_m": 0.740,
    "tabletop_thickness_m": 0.024,
    "slot_total_length_m": 0.230,
    "slot_straight_length_m": 0.153,
    "slot_back_edge_distance_m": 0.092,
    "slot_center_x_m": 0.0,
    "outer_corner_radius_m": 0.025,
    "include_photo_markers": True,
}


def _rounded_rectangle(length, depth, radius, segments=16):
    """Counterclockwise polygon with exact cardinal extents."""
    a, b = length / 2, depth / 2
    if radius <= 0 or radius >= min(a, b):
        raise ValueError("Rounded rectangle radius must fit inside its footprint")
    result = []
    for cx, cy, start in ((a-radius, b-radius, 0),
                          (-a+radius, b-radius, math.pi/2),
                          (-a+radius, -b+radius, math.pi),
                          (a-radius, -b+radius, 3*math.pi/2)):
        result.extend((cx+radius*math.cos(t), cy+radius*math.sin(t))
                      for t in np.linspace(start, start+math.pi/2, segments+1))
    return np.asarray(result, dtype=float)


def _capsule(length, width, segments=48):
    radius, half_straight = width / 2, (length-width) / 2
    result = [(half_straight+radius*math.cos(t), radius*math.sin(t))
              for t in np.linspace(-math.pi/2, math.pi/2, segments+1)]
    result.extend((-half_straight+radius*math.cos(t), radius*math.sin(t))
                  for t in np.linspace(math.pi/2, 3*math.pi/2, segments+1))
    return np.asarray(result, dtype=float)


def _cross2(a, b):
    return a[..., 0]*b[..., 1] - a[..., 1]*b[..., 0]


def _radial_boundary(polygon, origin, directions):
    """Intersect rays from an interior point with a convex polygon boundary."""
    start = polygon-origin
    edge = np.roll(polygon, -1, axis=0)-polygon
    denominator = _cross2(directions[:, None, :], edge[None, :, :])
    with np.errstate(divide="ignore", invalid="ignore"):
        distances = _cross2(start, edge)[None, :] / denominator
        fractions = _cross2(start[None, :, :], directions[:, None, :]) / denominator
    good = ((distances > 0) & (fractions >= -1e-9)
            & (fractions <= 1+1e-9) & (np.abs(denominator) > 1e-12))
    distance = np.min(np.where(good, distances, np.inf), axis=1)
    if not np.all(np.isfinite(distance)):
        raise ValueError("Cannot intersect all table boundary rays")
    return origin + distance[:, None]*directions


def _resolve_spec(spec):
    resolved = dict(DEFAULT_SPEC)
    if spec:
        unknown = set(spec)-set(resolved)
        if unknown:
            raise ValueError(f"Unknown table spec fields: {sorted(unknown)}")
        resolved.update(spec)
    for key, value in resolved.items():
        if key != "include_photo_markers" and not math.isfinite(float(value)):
            raise ValueError(f"Non-finite table dimension: {key}")
    width = resolved["slot_total_length_m"]-resolved["slot_straight_length_m"]
    center_y = (resolved["depth_m"]/2-resolved["slot_back_edge_distance_m"]-width/2)
    if width <= 0 or resolved["slot_straight_length_m"] <= 0:
        raise ValueError("Capsule slot needs positive width and straight length")
    if not 0 < resolved["tabletop_thickness_m"] < resolved["height_m"]:
        raise ValueError("Tabletop thickness must be positive and less than height")
    if (abs(resolved["slot_center_x_m"])+resolved["slot_total_length_m"]/2+0.01
            >= resolved["length_m"]/2 or
            abs(center_y)+width/2+0.01 >= resolved["depth_m"]/2):
        raise ValueError("Slot must be inside the table, with clearance to its edge")
    return resolved, width, np.array([resolved["slot_center_x_m"], center_y])


def tabletop_geometry(spec=None):
    """Return watertight vertices, outward-wound triangles, and material groups.

    Aligned polar rings allow an explicitly triangulated annulus without a
    triangulation dependency or unconstrained triangles crossing the aperture.
    The pale slot rim is a flush material strip, so tabletop height stays exact.
    """
    spec, width, origin = _resolve_spec(spec)
    length, depth, top = (spec[k] for k in ("length_m", "depth_m", "height_m"))
    bottom = top-spec["tabletop_thickness_m"]
    outer = _rounded_rectangle(length, depth, spec["outer_corner_radius_m"])
    inner = _capsule(spec["slot_total_length_m"], width)+origin
    lip = _capsule(spec["slot_total_length_m"]+0.006, width+0.006)+origin
    vertices2d = np.concatenate((outer, inner, lip))-origin
    angles = np.unique(np.round(np.arctan2(vertices2d[:, 1], vertices2d[:, 0]), 12))
    directions = np.stack((np.cos(angles), np.sin(angles)), axis=1)
    outer, lip, inner = (_radial_boundary(p, origin, directions) for p in (outer, lip, inner))
    n = len(angles)
    # Outer top, lip top, inner top, outer stripe top/bottom, outer bottom,
    # and inner bottom. There is intentionally no surface across the hole.
    levels = [(outer, top), (lip, top), (inner, top),
              (outer, top-0.001), (outer, top-0.004),
              (outer, bottom), (inner, bottom)]
    points = np.concatenate([np.column_stack((xy, np.full(n, z))) for xy, z in levels])
    triangles, groups = [], {name: [] for name in ("White", "DarkEdge", "SlotRim")}

    def connect(a, b, material, reverse=False):
        for i in range(n):
            j = (i+1) % n
            faces = [(a*n+i, a*n+j, b*n+j), (a*n+i, b*n+j, b*n+i)]
            if reverse:
                faces = [tuple(reversed(face)) for face in faces]
            groups[material].extend((len(triangles), len(triangles)+1))
            triangles.extend(faces)

    connect(0, 1, "White")
    connect(1, 2, "SlotRim")
    connect(5, 6, "White", reverse=True)
    connect(0, 3, "White", reverse=True)
    connect(3, 4, "DarkEdge", reverse=True)
    connect(4, 5, "White", reverse=True)
    connect(2, 6, "SlotRim")
    return points, np.asarray(triangles, dtype=np.int32), groups


def vertical_ray_hits(points, triangles, xy, origin_z=2.0):
    """Return unique downward ray hit heights for geometry diagnostics."""
    tri = np.asarray(points)[np.asarray(triangles)]
    direction = np.array([0.0, 0.0, -1.0])
    edge1, edge2 = tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]
    p = np.cross(direction, edge2)
    determinant = np.einsum("ij,ij->i", edge1, p)
    valid = np.abs(determinant) > 1e-12
    inverse = np.divide(1.0, determinant, out=np.zeros_like(determinant), where=valid)
    s = np.array([*xy, origin_z])-tri[:, 0]
    u = np.einsum("ij,ij->i", s, p)*inverse
    q = np.cross(s, edge1)
    v = q[:, 2]*-inverse
    t = np.einsum("ij,ij->i", edge2, q)*inverse
    mask = valid & (u >= -1e-9) & (v >= -1e-9) & (u+v <= 1+1e-9) & (t >= 0)
    return np.unique(np.round(origin_z-t[mask], 9)).tolist()


def _material(stage, path, color, roughness=0.6):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path+"/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _collision(prim, material, mesh=False):
    UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
    if mesh:
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("none")
    prim.AddAppliedSchema("PhysxCollisionAPI")
    prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(0.001)
    prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")


def _mesh(stage, path, points, triangles):
    mesh = UsdGeom.Mesh.Define(stage, path)
    points = np.asarray(points, np.float32)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(triangles), 3, np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.asarray(triangles, np.int32).ravel()))
    mesh.CreateExtentAttr(Vt.Vec3fArray.FromNumpy(np.stack([points.min(0), points.max(0)])))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateOrientationAttr("rightHanded")
    return mesh.GetPrim()


def _cube(stage, path, center, size, material, physics=None):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*center))
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material)
    if physics:
        _collision(cube.GetPrim(), physics)
    return cube.GetPrim()


def author_table(stage, parent_path, spec=None):
    """Author table rooted at ``parent_path`` and return JSON-ready metadata."""
    spec, width, slot_center = _resolve_spec(spec)
    root = UsdGeom.Xform.Define(stage, parent_path).GetPrim()
    root.SetCustomData({"role": "Measured white table, battery insertion task",
                        "units": "metres", "slot": "Open capsule through visual and collision mesh"})
    looks = str(parent_path)+"/Looks"
    UsdGeom.Scope.Define(stage, looks)
    materials = {name: _material(stage, looks+"/"+name, color) for name, color in {
        "White": (0.84, 0.85, 0.82), "DarkEdge": (0.09, 0.10, 0.095),
        "SlotRim": (0.63, 0.64, 0.58), "MarkerRed": (0.37, 0.075, 0.065),
    }.items()}
    physics = UsdShade.Material.Define(stage, looks+"/LaminatePhysics")
    api = UsdPhysics.MaterialAPI.Apply(physics.GetPrim())
    api.CreateStaticFrictionAttr(0.65)
    api.CreateDynamicFrictionAttr(0.50)
    api.CreateRestitutionAttr(0.0)
    points, triangles, groups = tabletop_geometry(spec)
    table = _mesh(stage, str(parent_path)+"/Tabletop", points, triangles)
    _collision(table, physics, mesh=True)
    for name, indices in groups.items():
        subset = UsdGeom.Subset.Define(stage, str(table.GetPath())+"/"+name)
        subset.CreateElementTypeAttr("face")
        subset.CreateFamilyNameAttr("materialBind")
        subset.CreateIndicesAttr(Vt.IntArray(indices))
        UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(materials[name])
    UsdGeom.Subset.SetFamilyType(UsdGeom.Imageable(table), "materialBind", "partition")
    top = spec["height_m"]
    leg_height = top-spec["tabletop_thickness_m"]
    # Leg/frame cross sections and offsets are photograph-based approximations.
    leg_centers = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx, cy = sx*(spec["length_m"]/2-0.034), sy*(spec["depth_m"]/2-0.034)
            section = _rounded_rectangle(0.036, 0.036, 0.010, segments=8)
            n = len(section)
            low = section*0.82 + (cx, cy)
            high = section + (cx, cy)
            vertices = np.concatenate((np.column_stack((low, np.zeros(n))),
                                       np.column_stack((high, np.full(n, leg_height)))))
            faces = []
            for i in range(n):
                j = (i+1) % n
                faces.extend(((i, j, n+j), (i, n+j, n+i)))
            faces.extend((0, i+1, i) for i in range(1, n-1))
            faces.extend((n, n+i, n+i+1) for i in range(1, n-1))
            leg = _mesh(stage, str(parent_path)+f"/Leg_{'L' if sx < 0 else 'R'}_{'Front' if sy < 0 else 'Back'}", vertices, faces)
            UsdShade.MaterialBindingAPI.Apply(leg).Bind(materials["White"])
            _collision(leg, physics, mesh=True)
            leg_centers.append([cx, cy])
    frame_z = leg_height-0.018
    for sy in (-1, 1):
        _cube(stage, str(parent_path)+f"/ApronY_{'Front' if sy < 0 else 'Back'}",
              (0, sy*(spec["depth_m"]/2-0.024), frame_z),
              (spec["length_m"]-0.06, 0.019, 0.036), materials["White"], physics)
    for sx in (-1, 1):
        _cube(stage, str(parent_path)+f"/ApronX_{'L' if sx < 0 else 'R'}",
              (sx*(spec["length_m"]/2-0.024), 0, frame_z),
              (0.019, spec["depth_m"]-0.06, 0.036), materials["White"], physics)
    if spec["include_photo_markers"]:
        for side, x in (("Left", -0.25), ("Right", 0.25)):
            # Tiny red outline labels at the front edge, visual only.
            y = -spec["depth_m"]/2+0.013
            for index, (dx, dy, lx, ly) in enumerate((
                (-0.006, 0, 0.0006, 0.017), (0.006, 0, 0.0006, 0.017),
                (0, -0.0085, 0.012, 0.0006), (0, 0.0085, 0.012, 0.0006))):
                _cube(stage, str(parent_path)+f"/PhotoMarker_{side}_{index}",
                      (x+dx, y+dy, top+0.00004), (lx, ly, 0.00004), materials["MarkerRed"])
    half_straight = spec["slot_straight_length_m"]/2
    sample_x = np.linspace(-half_straight-width*0.35, half_straight+width*0.35, 7)
    rays = [{"xy_m": (slot_center+[dx, 0]).tolist(),
             "hit_z_m": vertical_ray_hits(points, triangles, slot_center+[dx, 0])}
            for dx in sample_x]
    probe_xy = slot_center+[0, -width/2-0.010]
    return {
        "root_path": str(parent_path), "spec": spec,
        "coordinate_frame": "+X photo-right; +Y photo-back; +Z up; floor Z=0",
        "tabletop_bounds_m": [points.min(axis=0).tolist(), points.max(axis=0).tolist()],
        "floor_z_m": 0.0, "tabletop_z_m": top,
        "slot": {"center_xy_m": slot_center.tolist(), "total_length_m": spec["slot_total_length_m"],
                 "straight_length_m": spec["slot_straight_length_m"], "width_m": width,
                 "back_edge_distance_m": spec["slot_back_edge_distance_m"],
                 "open_through_tabletop": True, "collision_approximation": "none"},
        "leg_centers_xy_m": leg_centers,
        "validation": {"slot_centerline_vertical_rays": rays,
                       "all_slot_rays_open": all(not ray["hit_z_m"] for ray in rays),
                       "solid_surface_probe_xy_m": probe_xy.tolist(),
                       "solid_surface_probe_hit_z_m": vertical_ray_hits(points, triangles, probe_xy),
                       "tabletop_triangles": len(triangles)},
        "estimated_parameters": ["tabletop_thickness_m", "outer_corner_radius_m", "slot_center_x_m",
                                 "leg cross sections and locations", "apron dimensions", "surface materials",
                                 "front photo marker positions"],
    }
