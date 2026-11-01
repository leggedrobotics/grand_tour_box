"""Optional Open3D/Cupoch tests; run separately with requirements-geometry.txt."""

import contextlib
import io
from pathlib import Path
import tempfile
import unittest
import sys

import numpy as np
import open3d as o3d

PACKAGE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PACKAGE / "python"))
from box_auto.scripts.special import orient_normals_etna as orient
from box_auto.scripts.special import SciroVideo_improved as mesh_video


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.directory = Path(self.tmp.name)

    def test_closed_components_choose_winding_and_preserve_vertices(self):
        first = o3d.geometry.TriangleMesh.create_tetrahedron()
        second = o3d.geometry.TriangleMesh.create_tetrahedron().translate([5, 0, 0])
        faces = np.asarray(second.triangles).copy()[:, [0, 2, 1]]
        second.triangles = o3d.utility.Vector3iVector(faces)
        source = first + second
        path = self.directory / "source.ply"
        o3d.io.write_triangle_mesh(str(path), source)
        for inward in (False, True):
            result = orient.fix_normals(str(path), method="component", inward=inward)
            np.testing.assert_allclose(result.vertices, source.vertices)
            labels = np.asarray(result.cluster_connected_triangles()[0])
            vertices, triangles = np.asarray(result.vertices), np.asarray(result.triangles)
            for label in np.unique(labels):
                faces = triangles[labels == label]
                points = vertices[faces] - vertices[np.unique(faces)].mean(axis=0)
                volume6 = np.einsum("ij,ij->i", points[:, 0], np.cross(points[:, 1], points[:, 2])).sum()
                self.assertGreater(-volume6 if inward else volume6, 0)

    def test_open_surface_requires_reference_and_camera_changes_normals(self):
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector([[0, 0, 0], [1, 0, 0], [0, 1, 0]])
        mesh.triangles = o3d.utility.Vector3iVector([[0, 1, 2]])
        path = self.directory / "plane.ply"
        o3d.io.write_triangle_mesh(str(path), mesh)
        with self.assertRaises(ValueError):
            orient.fix_normals(str(path), method="component")
        for camera, normal in (([0, 0, 5], [0, 0, 1]), ([0, 0, -5], [0, 0, -1])):
            result = orient.fix_normals(str(path), method="camera", camera=camera)
            np.testing.assert_allclose(np.asarray(result.triangle_normals), [normal])
        mesh_video.orient_normals_towards_point(mesh, [0, 0, -5])
        np.testing.assert_allclose(np.asarray(mesh.triangle_normals), [[0, 0, -1]])
        mesh.triangle_uvs = o3d.utility.Vector2dVector([[0, 0], [1, 0], [0, 1]])
        with self.assertRaises(ValueError):
            mesh_video.orient_normals_towards_point(mesh, [0, 0, 5])

    def test_mesh_smoothing_is_explicit_and_empty_mesh_fails(self):
        mesh = o3d.geometry.TriangleMesh.create_tetrahedron()
        path = self.directory / "source.ply"
        o3d.io.write_triangle_mesh(str(path), mesh)
        original = mesh_video.load_mesh_fixed(str(path))
        smoothed = mesh_video.load_mesh_fixed(str(path), smooth_iters=1)
        np.testing.assert_allclose(original.vertices, mesh.vertices)
        self.assertFalse(np.allclose(smoothed.vertices, mesh.vertices))
        with self.assertRaises(ValueError):
            mesh_video.load_mesh_fixed(str(self.directory / "missing.ply"))

    def test_gpu_normals_preserve_positions_colors_and_plane_direction(self):
        from box_auto.scripts.special import calculate_point_cloud_normals as normals

        x, y = np.meshgrid(np.linspace(-1, 1, 30), np.linspace(-1, 1, 30))
        points = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        cloud.colors = o3d.utility.Vector3dVector(np.full(points.shape, 0.5))
        source, output = self.directory / "source.ply", self.directory / "normals.ply"
        o3d.io.write_point_cloud(str(source), cloud)
        with contextlib.redirect_stdout(io.StringIO()):
            normals.main([str(source), str(output), "--camera", "0", "0", "5"])
        result = o3d.io.read_point_cloud(str(output))
        np.testing.assert_allclose(result.points, points, atol=1e-6)
        np.testing.assert_allclose(result.normals, np.tile([0, 0, 1], (len(points), 1)), atol=1e-5)
        np.testing.assert_allclose(result.colors, o3d.io.read_point_cloud(str(source)).colors, atol=1 / 255)
        before = output.read_bytes()
        with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
            normals.main([str(source), str(output)])
        self.assertEqual(output.read_bytes(), before)
