"""Geometry invariants for the measured, open-slot battery task table."""

import unittest

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

from room01_sim.battery_table import author_table, tabletop_geometry, vertical_ray_hits


class MeasuredTableTests(unittest.TestCase):
    def test_measured_dimensions_and_open_aperture(self):
        points, triangles, _ = tabletop_geometry()
        np.testing.assert_allclose(points.min(0), [-0.55, -0.285, 0.716], atol=1e-10)
        np.testing.assert_allclose(points.max(0), [0.55, 0.285, 0.740], atol=1e-10)
        # Test both centerline and near-wall rays through the full capsule.
        for x in np.linspace(-0.11, 0.11, 23):
            self.assertEqual(vertical_ray_hits(points, triangles, [x, 0.1545]), [])
        for y in np.linspace(0.118, 0.191, 15):
            self.assertEqual(vertical_ray_hits(points, triangles, [0.0, y]), [])
        for xy in [[0, 0], [-0.12, 0.1545], [0.12, 0.1545], [0, 0.195]]:
            self.assertEqual(vertical_ray_hits(points, triangles, xy), [0.716, 0.740])

    def test_tabletop_is_closed_and_consistently_wound(self):
        points, triangles, groups = tabletop_geometry()
        edges = np.concatenate((triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]]))
        _, counts = np.unique(np.sort(edges, axis=1), axis=0, return_counts=True)
        self.assertTrue(np.all(counts == 2), "Every edge needs exactly two adjacent faces")
        # Each edge must occur once in each direction, including the hole walls.
        self.assertEqual(set(map(tuple, edges)), set(map(tuple, edges[:, ::-1])))
        signed_volume = (points[triangles[:, 0]] * np.cross(
            points[triangles[:, 1]], points[triangles[:, 2]])).sum()/6
        expected_volume = (1.10*0.57-(4-np.pi)*0.025**2
                           -(0.153*0.077+np.pi*(0.077/2)**2))*0.024
        self.assertAlmostEqual(signed_volume, expected_volume, delta=1e-6)
        # PhysX consumes float32; reject triangles that collapse at that precision.
        tri = points.astype(np.float32)[triangles]
        areas = np.linalg.norm(np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]), axis=1)/2
        self.assertGreater(float(areas.min()), 1e-10)
        self.assertEqual(sorted(sum(groups.values(), [])), list(range(len(triangles))))

    def test_authored_static_collision_does_not_fill_hole(self):
        stage = Usd.Stage.CreateInMemory()
        report = author_table(stage, "/Table", {"include_photo_markers": False})
        self.assertEqual(report["slot"]["center_xy_m"], [0.0, 0.15449999999999997])
        self.assertAlmostEqual(report["slot"]["width_m"], 0.077)
        self.assertTrue(report["validation"]["all_slot_rays_open"])
        self.assertEqual(UsdGeom.Xformable(stage.GetPrimAtPath("/Table")).GetOrderedXformOps(), [])
        collisions = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
        self.assertEqual(len(collisions), 9)
        for prim in collisions:
            self.assertFalse(prim.HasAPI(UsdPhysics.RigidBodyAPI))
            if prim.IsA(UsdGeom.Mesh):
                self.assertEqual(UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get(), "none")
        for prim in stage.Traverse():
            if prim.GetName().startswith("Leg_"):
                points = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
                self.assertEqual(float(points[:, 2].min()), 0.0)
                self.assertAlmostEqual(float(points[:, 2].max()), 0.716, places=6)

    def test_slot_placement_is_parameterized(self):
        stage = Usd.Stage.CreateInMemory()
        spec = {"slot_center_x_m": 0.05, "slot_back_edge_distance_m": 0.11,
                "include_photo_markers": False}
        report = author_table(stage, "/ShiftedTable", spec)
        np.testing.assert_allclose(report["slot"]["center_xy_m"], [0.05, 0.1365])
        self.assertTrue(report["validation"]["all_slot_rays_open"])
        with self.assertRaises(ValueError):
            tabletop_geometry({"slot_straight_length_m": 0.3})
        with self.assertRaises(ValueError):
            tabletop_geometry({"slot_center_x_m": 1.0})


if __name__ == "__main__":
    unittest.main()
