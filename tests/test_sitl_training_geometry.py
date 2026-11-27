"""The optional model correction changes collision geometry, not the plant or native control."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock
from xml.etree import ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('training_geometry_test_module', ROOT / 'tools/sitl_training_geometry.py')
G = importlib.util.module_from_spec(spec)
spec.loader.exec_module(G)
SDF = ROOT / 'docs/evidence/sitl_2026-11-05/vehicle/2026-11-05_dev07/navrl_quad.sdf'


class GeometryTest(unittest.TestCase):
    def setUp(self):
        self.source = SDF.read_bytes()
        self.urdf = G.URDF.read_bytes()

    def test_native_control_is_exact_historical_bytes(self):
        result, rec = G.build(self.source, self.urdf)
        self.assertEqual(result, self.source)
        self.assertTrue(rec['native_byte_identity'])
        self.assertEqual(rec['removed_collision_names'], [])
        self.assertEqual(G.sha(result), G.SOURCE_SHA)

    def test_training_box_changes_only_four_collision_elements(self):
        result, rec = G.build(self.source, self.urdf, 'training-box')
        self.assertFalse(rec['native_byte_identity'])
        self.assertCountEqual(rec['removed_collision_names'], G.ROTOR_NAMES)
        old, new = ET.fromstring(self.source), ET.fromstring(result)
        self.assertEqual(len(new.findall('.//collision')), 1)
        self.assertEqual(new.findtext('.//collision/geometry/box/size'), '0.28 0.28 0.08')
        # Non-vacuous plant checks: all inertias, thrust/motor plugins, joints,
        # sensors and visuals survive with their exact element bytes and values.
        # ElementTree includes the whitespace *after* an element in tostring.
        def element_bytes(node):
            node = copy.deepcopy(node)
            node.tail = None
            return ET.tostring(node)
        for tag in ('inertial', 'plugin', 'include', 'joint', 'visual'):
            a, b = old.findall('.//' + tag), new.findall('.//' + tag)
            self.assertGreater(len(a), 0, tag)
            self.assertEqual([element_bytes(e) for e in a], [element_bytes(e) for e in b], tag)
        # Reverse the four deletions to prove every other original byte survived.
        removed = []
        import re
        for match in re.finditer(rb'<collision\b[^>]*>.*?</collision\s*>', self.source, re.DOTALL):
            if ET.fromstring(match.group()).get('name') in G.ROTOR_NAMES:
                removed.append((match.start(), match.end()))
        expected = self.source
        for start, end in reversed(removed):
            expected = expected[:start] + expected[end:]
        self.assertEqual(result, expected)

    def test_rejects_unknown_shapes_offsets_extra_and_missing_colliders(self):
        mutations = (
            self.source.replace(b'<radius>0.128</radius>', b'<radius>0.01</radius>', 1),
            self.source.replace(b'<size>0.28 0.28 0.08</size>', b'<size>0.3 0.3 0.08</size>', 1),
            self.source.replace(b'0.13 -0.13 0 0 0 0', b'0.14 -0.13 0 0 0 0', 1),
            self.source.replace(b'rotor_0_collision', b'extra_collision', 1),
            self.source.replace(b'</model>', b'<link name="unexpected"><collision name="x"/></link></model>', 1),
        )
        self.assertGreater(len(mutations), 0)
        for bad in mutations:
            for mode in G.MODES:
                with self.subTest(mode=mode), self.assertRaises(ValueError):
                    G.build(bad, self.urdf, mode)
        with self.assertRaises(ValueError):
            G.build(self.source, self.urdf.replace(b'0.28 0.28 0.08', b'0.30 0.30 0.08'), 'training-box')
        with self.assertRaises(ValueError):
            G.build(self.source, self.urdf, 'always-on')

    def test_preparation_refuses_existing_directory_and_wrong_source(self):
        # Restrict all writes to a temporary stand-in for ~/motar_tmp.
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(G.Path, 'home', return_value=Path(tmp)):
            root = Path(tmp) / 'motar_tmp'
            root.mkdir()
            native = G.prepare(root / 'native', source=SDF)
            fixed = G.prepare(root / 'fixed', mode='training-box', source=SDF)
            self.assertEqual(native['output_sdf_sha256'], G.SOURCE_SHA)
            self.assertNotEqual(native['output_sdf_sha256'], fixed['output_sdf_sha256'])
            self.assertEqual(SDF.read_bytes(), self.source)
            for bad in (root / 'native', Path(tmp) / 'elsewhere'):
                with self.assertRaises(ValueError):
                    G.prepare(bad, source=SDF)
            wrong = root / 'wrong.sdf'
            wrong.write_bytes(self.source + b'\n')
            with self.assertRaises(ValueError):
                G.prepare(root / 'bad-source', source=wrong)

    def test_plan_adapter_changes_spawn_only_and_rejects_tampering(self):
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(G.Path, 'home', return_value=Path(tmp)):
            root = Path(tmp) / 'motar_tmp'
            root.mkdir()
            package = root / 'fixed'
            G.prepare(package, 'training-box', source=SDF)
            plan = {'sdf_path': SDF, 'env': {'PX4_SIM_MODEL': 'gazebo-classic_navrl_quad'},
                    'spawn': {'argv': ['gz', 'model', '--spawn-file=' + str(SDF), '--model-name=navrl_quad']},
                    'px4': {'argv': ['px4', '-d', 'etc']}}
            validate = mock.Mock()
            owned = types.SimpleNamespace(direct_launch_plan=lambda *a: copy.deepcopy(plan), validate_plan=validate)
            result = G.launch_plan(owned, root / 'episode', 'world', {}, package)
            self.assertEqual(result['px4'], plan['px4'])
            self.assertEqual(result['env'], plan['env'])
            self.assertEqual(result['sdf_path'], package / 'navrl_quad/navrl_quad.sdf')
            self.assertEqual(result['spawn']['argv'][2], '--spawn-file=' + str(result['sdf_path']))
            validate.assert_called_once()
            f = package / 'navrl_quad/navrl_quad.sdf'
            f.write_bytes(self.source)  # Receipt alone cannot attest the corrected geometry.
            with self.assertRaises(ValueError):
                G.launch_plan(owned, root / 'episode', 'world', {}, package)
            self.assertEqual(validate.call_count, 1)

    def test_model_config_cannot_be_relabelled_by_updating_its_receipt(self):
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(G.Path, 'home', return_value=Path(tmp)):
            package = Path(tmp) / 'motar_tmp/package'
            G.prepare(package, 'training-box', source=SDF)
            config = package / 'navrl_quad/model.config'
            config.write_bytes(G.MODEL_CONFIG.replace(b'navrl_quad.sdf', b'other.sdf'))
            receipt_path = package / 'geometry.json'
            receipt = json.loads(receipt_path.read_text())
            receipt['model_config_sha256'] = G.sha(config.read_bytes())
            receipt_path.write_text(json.dumps(receipt))
            owned = mock.Mock()
            with self.assertRaises(ValueError):
                G.launch_plan(owned, package.parent / 'episode', 'world', {}, package)
            owned.direct_launch_plan.assert_not_called()


if __name__ == '__main__':
    unittest.main()
