"""CPU checks that the battery insertion assets preserve physical openings."""

import unittest
from collections import Counter

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from room01_sim.battery_assets import author_props, red_foam_mesh


def first_vertical_hit(points, triangles, xy):
    """Return the highest triangle intersection of a ray directed downwards."""
    faces = np.asarray(points)[np.asarray(triangles, dtype=int)]
    a, b, c = faces[:, 0], faces[:, 1], faces[:, 2]
    ab, ac = b[:, :2] - a[:, :2], c[:, :2] - a[:, :2]
    delta = np.asarray(xy) - a[:, :2]
    det = ab[:, 0] * ac[:, 1] - ab[:, 1] * ac[:, 0]
    usable = np.abs(det) > 1e-14
    u = np.zeros(len(faces))
    v = np.zeros(len(faces))
    u[usable] = (
        delta[usable, 0] * ac[usable, 1]
        - delta[usable, 1] * ac[usable, 0]
    ) / det[usable]
    v[usable] = (
        ab[usable, 0] * delta[usable, 1]
        - ab[usable, 1] * delta[usable, 0]
    ) / det[usable]
    hit = usable & (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9)
    if not np.any(hit):
        return None
    z = a[:, 2] + u * (b[:, 2] - a[:, 2]) + v * (c[:, 2] - a[:, 2])
    return float(np.max(z[hit]))


class RedFoamGeometryTests(unittest.TestCase):
    def setUp(self):
        self.points, self.triangles = red_foam_mesh()
        self.points = np.asarray(self.points)
        self.triangles = np.asarray(self.triangles, dtype=int)

    def test_measured_external_dimensions(self):
        np.testing.assert_allclose(self.points.min(axis=0), [-.036, -.024, -.024], atol=1e-8)
        np.testing.assert_allclose(self.points.max(axis=0), [.036, .024, .024], atol=1e-8)
        self.assertEqual(self.triangles.ndim, 2)
        self.assertEqual(self.triangles.shape[1], 3)
        self.assertTrue(np.isfinite(self.points).all())
        self.assertGreaterEqual(self.triangles.min(), 0)
        self.assertLess(self.triangles.max(), len(self.points))

    def test_blind_holes_have_clear_entrances_and_measured_bottoms(self):
        # Rays through each center and a point inside its wall must reach the
        # recessed bottom, not an invisible face across the opening.
        for center_x in (-.020, 0., .020):
            for dx, dy in ((0., 0.), (.004, 0.), (0., -.004)):
                with self.subTest(center_x=center_x, offset=(dx, dy)):
                    hit = first_vertical_hit(self.points, self.triangles, (center_x + dx, dy))
                    self.assertIsNotNone(hit)
                    self.assertAlmostEqual(hit, .011, places=7)
        for xy in ((0., .015), (0., -.015), (-.032, 0.), (.032, 0.), (.010, 0.)):
            with self.subTest(solid=xy):
                self.assertAlmostEqual(first_vertical_hit(self.points, self.triangles, xy), .024, places=7)

    def test_closed_manifold_has_no_degenerate_faces(self):
        # Weld any duplicated shading vertices before inspecting topology.
        _, inverse = np.unique(np.round(self.points, 10), axis=0, return_inverse=True)
        triangles = inverse[self.triangles]
        edge_counts = Counter()
        directed_counts = Counter()
        for triangle in triangles:
            for first, second in zip(triangle, np.roll(triangle, -1)):
                edge_counts[tuple(sorted((int(first), int(second))))] += 1
                directed_counts[(int(first), int(second))] += 1
        self.assertTrue(edge_counts)
        self.assertTrue(all(count == 2 for count in edge_counts.values()), "Mesh has an open or nonmanifold edge")
        self.assertTrue(
            all(directed_counts[(a, b)] == directed_counts[(b, a)] for a, b in edge_counts),
            "Adjacent triangles must have consistent orientation",
        )
        faces = self.points[self.triangles]
        twice_area = np.linalg.norm(np.cross(faces[:, 1] - faces[:, 0], faces[:, 2] - faces[:, 0]), axis=1)
        self.assertTrue(np.all(twice_area > 1e-13), "Mesh has a degenerate face")

    def test_removed_volume_matches_three_blind_holes(self):
        faces = self.points[self.triangles]
        volume = abs(np.einsum("ij,ij->i", faces[:, 0], np.cross(faces[:, 1], faces[:, 2])).sum() / 6.)
        # Polygonal cross-sections approximate the circular measurements.
        circular_reference = .072 * .048 * .048 - 3 * np.pi * .00625**2 * .013
        self.assertAlmostEqual(volume, circular_reference, delta=2e-7)

    def test_changed_measurements_move_hole_walls_and_bottoms(self):
        points, triangles = red_foam_mesh(
            dimensions=(.080, .060, .060),
            hole_diameter=.016,
            hole_depth=.040,
            hole_pitch=.024,
        )
        points = np.asarray(points)
        np.testing.assert_allclose(points.max(axis=0) - points.min(axis=0), [.080, .060, .060], atol=1e-8)
        for x in (-.024, 0., .024):
            self.assertAlmostEqual(first_vertical_hit(points, triangles, (x, .006)), -.010, places=7)
            self.assertAlmostEqual(first_vertical_hit(points, triangles, (x, .011)), .030, places=7)

    def test_invalid_holes_fail_before_authoring(self):
        invalid = [
            {"hole_depth": 0.},
            {"hole_depth": -.001},
            {"hole_depth": .048},
            {"hole_depth": .060},
            {"hole_diameter": 0.},
            {"hole_diameter": -.001},
            {"hole_diameter": .049},
            {"hole_pitch": 0.},
            {"hole_pitch": -.020},
            {"hole_pitch": .010},
            {"hole_pitch": .040},
        ]
        for kwargs in invalid:
            with self.subTest(**kwargs), self.assertRaises(ValueError):
                red_foam_mesh(**kwargs)


class BatteryUsdTests(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(self.stage, "/Task")

    def test_dynamic_batteries_use_measured_cylinder_and_mass(self):
        metadata = author_props(self.stage, "/Task")
        self.assertEqual(len(metadata["batteries"]), 2)
        self.assertAlmostEqual(metadata["red_foam"]["hole_diameter_m"], .0125)
        self.assertAlmostEqual(metadata["red_foam"]["hole_depth_m"], .013)
        # The measured equal diameters are valid scene inputs. This geometry
        # check deliberately makes no claim about physical insertion success.
        for index in (1, 2):
            root_path = f"/Task/Battery_{index}"
            root = self.stage.GetPrimAtPath(root_path)
            self.assertTrue(root.HasAPI(UsdPhysics.RigidBodyAPI))
            self.assertTrue(UsdPhysics.RigidBodyAPI(root).GetRigidBodyEnabledAttr().Get())
            self.assertTrue(root.HasAPI(UsdPhysics.MassAPI))
            self.assertAlmostEqual(UsdPhysics.MassAPI(root).GetMassAttr().Get(), .020, places=7)
            collider = UsdGeom.Cylinder(self.stage.GetPrimAtPath(root_path + "/Collision"))
            self.assertTrue(collider)
            self.assertEqual(collider.GetAxisAttr().Get(), "X")
            self.assertAlmostEqual(collider.GetRadiusAttr().Get(), .00625, places=7)
            self.assertAlmostEqual(collider.GetHeightAttr().Get(), .049, places=7)
            self.assertTrue(collider.GetPrim().HasAPI(UsdPhysics.CollisionAPI))
            self.assertEqual(collider.GetVisibilityAttr().Get(), "invisible")
            record = next(item for item in metadata["batteries"] if item["path"] == root_path)
            self.assertEqual(len(record["local_position_m"]), 3)
            self.assertEqual(len(record["local_quaternion_xyzw"]), 4)
            self.assertEqual(len(record["size_m"]), 3)
            np.testing.assert_allclose(sorted(record["size_m"]), [.0125, .0125, .049], atol=1e-8)

    def test_red_collision_uses_open_mesh_without_convex_filling(self):
        author_props(self.stage, "/Task")
        body = self.stage.GetPrimAtPath("/Task/RedFoam/Body")
        self.assertTrue(body.IsA(UsdGeom.Mesh))
        self.assertTrue(body.HasAPI(UsdPhysics.CollisionAPI))
        self.assertTrue(body.HasAPI(UsdPhysics.MeshCollisionAPI))
        self.assertEqual(UsdPhysics.MeshCollisionAPI(body).GetApproximationAttr().Get(), "none")
        mesh = UsdGeom.Mesh(body)
        points = np.asarray(mesh.GetPointsAttr().Get())
        counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
        self.assertTrue(np.all(counts == 3))
        triangles = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
        for x in (-.020, 0., .020):
            self.assertAlmostEqual(first_vertical_hit(points, triangles, (x, 0.)), .011, places=7)
        self.assertTrue(self.stage.GetPrimAtPath("/Task/YellowFoam_1"))
        self.assertTrue(self.stage.GetPrimAtPath("/Task/YellowFoam_2"))

    def test_pose_override_is_local_normalized_and_preserves_parent_transform(self):
        parent = UsdGeom.Xform(self.stage.GetPrimAtPath("/Task"))
        parent.AddTranslateOp().Set(Gf.Vec3d(1., 2., 3.))
        parent.AddRotateZOp().Set(35.)
        parent_matrix = parent.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        position = [.12, -.08, .85]
        metadata = author_props(self.stage, "/Task", {
            "table_top_z_m": .800,
            "yellow_positions_xy_m": [[-.3, -.1], [.3, -.1]],
            "battery_poses": [{"position_m": position, "quaternion_wxyz": [2., 0., 2., 0.]}, {}],
        })
        first, second = metadata["batteries"]
        np.testing.assert_allclose(first["local_position_m"], position, atol=1e-8)
        np.testing.assert_allclose(first["local_quaternion_xyzw"], [0., 2**-.5, 0., 2**-.5], atol=1e-8)
        np.testing.assert_allclose(second["local_position_m"], [.3, -.1, .800 + .052 + .00625], atol=1e-8)
        np.testing.assert_allclose(parent.ComputeLocalToWorldTransform(Usd.TimeCode.Default()), parent_matrix, atol=1e-8)
        battery = UsdGeom.Xformable(self.stage.GetPrimAtPath(first["path"]))
        world_position = battery.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        np.testing.assert_allclose(world_position, parent_matrix.Transform(Gf.Vec3d(*position)), atol=1e-8)
        local_matrix = battery.GetLocalTransformation()
        np.testing.assert_allclose(local_matrix.TransformDir(Gf.Vec3d(1., 0., 0.)), [0., 0., -1.], atol=1e-8)


if __name__ == "__main__":
    unittest.main()
