"""CPU tests for URDF asset loading. Contract: docs/renderer_urdf_loader_v1_contract.md."""
import math
from pathlib import Path
import sys
import textwrap
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from renderer_validation.urdf_asset import (UrdfError, load_urdf_asset, box_mesh, sphere_mesh,
                                            cylinder_mesh, rotation_from_rpy, origin_transform,
                                            outward_normal_violations, open_edges, signed_volume,
                                            SPHERE_SEGMENTS, SPHERE_RINGS, CYLINDER_SEGMENTS,
                                            DEFAULT_MATERIAL_NAME)
from renderer_validation.scene import MeshScene

INTERCEPTOR = ROOT / "resources/robots/quad/quad_navrl_ref5in_v2.urdf"
TARGET = ROOT / "resources/models/environment_assets/objects/navrl_target_drone_v2.urdf"
DISTRACTORS = [ROOT / f"resources/models/environment_assets/objects/navrl_distractor_{n}.urdf"
               for n in ("sphere", "box", "pole")]


def single(points, faces):
    return MeshScene(points, faces, [0] * len(faces), [0] * len(faces))


def write(directory, body, name="robot.urdf"):
    path = Path(directory) / name
    path.write_text(textwrap.dedent(body).strip())
    return path


class RotationConventionTest(unittest.TestCase):
    def test_rpy_is_fixed_axis_z_then_y_then_x(self):
        roll, pitch, yaw = 0.3, -0.7, 1.1
        rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
        ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
        rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        np.testing.assert_allclose(rotation_from_rpy(roll, pitch, yaw), rz @ ry @ rx, atol=1e-14)

    def test_rotation_is_orthonormal_and_right_handed(self):
        matrix = rotation_from_rpy(0.4, 1.2, -2.0)
        np.testing.assert_allclose(matrix @ matrix.T, np.eye(3), atol=1e-14)
        self.assertAlmostEqual(float(np.linalg.det(matrix)), 1.0, places=12)

    def test_missing_origin_is_the_identity(self):
        np.testing.assert_array_equal(origin_transform(None), np.eye(4))

    def test_non_finite_and_malformed_origins_are_refused(self):
        import xml.etree.ElementTree as ElementTree
        for attributes in ('xyz="1 2 nan"', 'xyz="1 2"', 'rpy="0 0 abc"'):
            with self.assertRaises(UrdfError):
                origin_transform(ElementTree.fromstring(f"<origin {attributes}/>"))


class TessellationTest(unittest.TestCase):
    """A closed, outward-wound surface whose volume converges is the evidence that this is a shape."""

    def test_every_primitive_is_closed_and_wound_outward(self):
        for name, (points, faces) in (("box", box_mesh([0.4, 0.3, 0.2])),
                                      ("sphere", sphere_mesh(0.05)),
                                      ("cylinder", cylinder_mesh(0.01, 0.11))):
            mesh = single(points, faces)
            self.assertEqual(outward_normal_violations(mesh), 0, name)
            self.assertEqual(open_edges(mesh), [], name)

    def test_box_tessellation_is_exact_before_float32_storage(self):
        """Separate the two layers: the tessellation is exact, the stored vertices are float32.

        MeshScene stores float32 because Warp needs it, which costs about one part in 1e7. A
        volume test that ignores this looks like a tessellation error and is not one.
        """
        points, faces = box_mesh([0.283, 0.283, 0.12])
        corners = np.asarray(points, dtype=np.float64)[np.asarray(faces)]
        exact = 0.283 * 0.283 * 0.12
        in_float64 = float(np.sum(np.einsum("ij,ij->i", corners[:, 0],
                                            np.cross(corners[:, 1], corners[:, 2]))) / 6.0)
        self.assertEqual(in_float64, exact)
        stored = single(points, faces)
        self.assertEqual(stored.vertices.dtype, np.float32)
        self.assertLess(abs(signed_volume(stored) / exact - 1.0), float(np.finfo(np.float32).eps))

    def test_sphere_and_cylinder_volume_converge_from_below(self):
        exact = 4.0 / 3.0 * math.pi * 0.05 ** 3
        coarse = signed_volume(single(*sphere_mesh(0.05, 16, 8)))
        fine = signed_volume(single(*sphere_mesh(0.05, 64, 32)))
        self.assertLess(coarse, fine)           # an inscribed surface only grows with resolution
        self.assertLess(fine, exact)
        self.assertLess(abs(fine / exact - 1.0), 0.01)
        exact = math.pi * 0.01 ** 2 * 0.11
        coarse = signed_volume(single(*cylinder_mesh(0.01, 0.11, 16)))
        fine = signed_volume(single(*cylinder_mesh(0.01, 0.11, 128)))
        self.assertLess(coarse, fine)
        self.assertLess(fine, exact)
        self.assertLess(abs(fine / exact - 1.0), 0.01)

    def test_cylinder_axis_is_z_and_centred(self):
        points, _ = cylinder_mesh(0.06, 1.6)
        points = np.asarray(points)
        self.assertAlmostEqual(points[:, 2].min(), -0.8, places=12)
        self.assertAlmostEqual(points[:, 2].max(), 0.8, places=12)
        radius = np.linalg.norm(points[:, :2], axis=1)
        self.assertAlmostEqual(float(radius.max()), 0.06, places=12)

    def test_no_degenerate_triangle_at_the_sphere_poles(self):
        points, faces = sphere_mesh(0.05)
        single(points, faces)   # MeshScene refuses zero-area triangles
        self.assertEqual(len(faces), SPHERE_SEGMENTS * 2 + SPHERE_SEGMENTS * (SPHERE_RINGS - 2) * 2)

    def test_cylinder_triangle_count_follows_the_declared_segments(self):
        self.assertEqual(len(cylinder_mesh(1.0, 1.0)[1]), CYLINDER_SEGMENTS * 4)


class RealAssetTest(unittest.TestCase):
    def test_interceptor_has_nine_links_and_three_materials(self):
        asset = load_urdf_asset(INTERCEPTOR)
        self.assertEqual(asset.robot_name, "quadrotor_ref5in_v2")
        self.assertEqual(len(asset.link_names), 9)
        self.assertEqual(asset.link_names[0], "base_link")
        self.assertEqual(list(asset.material_names), ["White", "Orange", "Blue"])
        self.assertEqual(len(set(asset.mesh.face_instance.tolist())), 9)
        self.assertEqual(asset.default_material_visuals, 0)

    def test_every_shipped_asset_is_closed_and_wound_outward(self):
        for path in [INTERCEPTOR, TARGET] + DISTRACTORS:
            asset = load_urdf_asset(path)
            self.assertEqual(outward_normal_violations(asset.mesh), 0, path.name)
            self.assertEqual(open_edges(asset.mesh), [], path.name)

    def test_target_is_the_declared_red_box(self):
        asset = load_urdf_asset(TARGET)
        self.assertEqual(list(asset.material_names), ["TargetRed"])
        np.testing.assert_allclose(asset.material_rgba[0], [0.9, 0.2, 0.2, 0.95])
        self.assertLess(abs(signed_volume(asset.mesh) / (0.283 * 0.283 * 0.12) - 1.0),
                        float(np.finfo(np.float32).eps))

    def test_joint_origins_place_the_motors_where_the_urdf_says(self):
        """The four motor links sit at +-0.0777817 in x and y, per the URDF's own arithmetic."""
        asset = load_urdf_asset(INTERCEPTOR)
        centres = {}
        for instance, name in enumerate(asset.link_names):
            selected = asset.mesh.face_instance == instance
            vertices = asset.mesh.vertices[np.unique(asset.mesh.triangles[selected])]
            centres[name] = vertices.mean(axis=0)
        for index in range(4):
            centre = centres[f"motor_{index}"]
            self.assertAlmostEqual(abs(float(centre[0])), 0.0777817, places=6)
            self.assertAlmostEqual(abs(float(centre[1])), 0.0777817, places=6)
        np.testing.assert_allclose(centres["base_link"], [0, 0, 0], atol=1e-9)

    def test_loading_is_deterministic(self):
        first, second = load_urdf_asset(INTERCEPTOR), load_urdf_asset(INTERCEPTOR)
        np.testing.assert_array_equal(first.mesh.vertices, second.mesh.vertices)
        np.testing.assert_array_equal(first.mesh.triangles, second.mesh.triangles)
        self.assertEqual(first.as_dict(), second.as_dict())
        self.assertEqual(first.source_sha256, second.source_sha256)

    def test_receipt_records_the_tessellation_and_the_colour_choice(self):
        record = load_urdf_asset(INTERCEPTOR).as_dict()
        self.assertEqual(record["tessellation"]["sphere_segments"], SPHERE_SEGMENTS)
        self.assertIn("unconverted", record["colour_note"])
        self.assertEqual(record["triangles"], 736)


class RefusalTest(unittest.TestCase):
    """Everything the contract says is out of scope must raise, not silently produce geometry."""

    def setUp(self):
        import tempfile
        self.directory = tempfile.mkdtemp()

    def test_mesh_geometry_is_refused_with_a_pointer_to_the_contract(self):
        path = write(self.directory, """
            <robot name="m"><link name="a"><visual><geometry>
            <mesh filename="x.stl"/></geometry></visual></link></robot>""")
        with self.assertRaisesRegex(UrdfError, "contract"):
            load_urdf_asset(path)

    def test_non_fixed_joints_are_refused(self):
        path = write(self.directory, """
            <robot name="m">
              <link name="a"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
              <link name="b"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
              <joint name="j" type="revolute"><parent link="a"/><child link="b"/></joint>
            </robot>""")
        with self.assertRaisesRegex(UrdfError, "fixed joints only"):
            load_urdf_asset(path)

    def test_two_roots_and_unknown_links_are_refused(self):
        path = write(self.directory, """
            <robot name="m">
              <link name="a"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
              <link name="b"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
            </robot>""")
        with self.assertRaisesRegex(UrdfError, "one root link"):
            load_urdf_asset(path)
        path = write(self.directory, """
            <robot name="m">
              <link name="a"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
              <joint name="j" type="fixed"><parent link="a"/><child link="ghost"/></joint>
            </robot>""", "ghost.urdf")
        with self.assertRaisesRegex(UrdfError, "unknown link"):
            load_urdf_asset(path)

    def test_degenerate_dimensions_are_refused(self):
        for geometry in ('<box size="1 0 1"/>', '<sphere radius="0"/>', '<cylinder radius="1" length="-1"/>'):
            path = write(self.directory, f"""
                <robot name="m"><link name="a"><visual><geometry>
                {geometry}</geometry></visual></link></robot>""", "d.urdf")
            with self.assertRaises(UrdfError):
                load_urdf_asset(path)

    def test_undeclared_material_reference_is_refused(self):
        path = write(self.directory, """
            <robot name="m"><link name="a"><visual><geometry><box size="1 1 1"/></geometry>
            <material name="Nowhere"/></visual></link></robot>""", "mat.urdf")
        with self.assertRaisesRegex(UrdfError, "undeclared material"):
            load_urdf_asset(path)

    def test_a_visual_without_a_material_uses_the_declared_default_and_is_counted(self):
        path = write(self.directory, """
            <robot name="m"><link name="a"><visual><geometry>
            <box size="1 1 1"/></geometry></visual></link></robot>""", "nomat.urdf")
        asset = load_urdf_asset(path)
        self.assertEqual(list(asset.material_names), [DEFAULT_MATERIAL_NAME])
        self.assertEqual(asset.default_material_visuals, 1)

    def test_a_robot_with_no_visual_geometry_is_refused(self):
        path = write(self.directory, """
            <robot name="m"><link name="a"><collision><geometry>
            <box size="1 1 1"/></geometry></collision></link></robot>""", "novis.urdf")
        with self.assertRaisesRegex(UrdfError, "no visual geometry"):
            load_urdf_asset(path)

    def test_malformed_xml_and_wrong_root_are_refused(self):
        path = write(self.directory, "<robot name='m'><link", "bad.urdf")
        with self.assertRaisesRegex(UrdfError, "well-formed"):
            load_urdf_asset(path)
        path = write(self.directory, "<sdf/>", "sdf.urdf")
        with self.assertRaisesRegex(UrdfError, "expected <robot>"):
            load_urdf_asset(path)


class IsolationTest(unittest.TestCase):
    def test_loader_imports_no_simulator_and_no_urdf_library(self):
        import ast
        source = (ROOT / "tools/renderer_validation/urdf_asset.py").read_text()
        imported = set()
        for node in ast.walk(ast.parse(source)):
            if isinstance(node, ast.Import):
                imported.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                imported.add(node.module.split(".")[0])
        self.assertEqual(imported - {"dataclasses", "hashlib", "math", "pathlib", "xml", "numpy"}, set())


if __name__ == "__main__":
    unittest.main()
