"""ROS2 packaging and explicit-source dispatch without ROS or device access."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest import mock
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'ros2' / 'motar_camera_policy'
ENTRY = PACKAGE / 'motar_camera_policy' / 'entry.py'


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Ros2PackageTests(unittest.TestCase):
    def fixture(self, directory):
        source = Path(directory) / 'chosen-source'
        (source / 'tools').mkdir(parents=True)
        (source / 'tools' / 'camera_ros2_node.py').write_text('''
import json
from pathlib import Path
import sys

def main(ros_args=None, *, config_path, source_root, check_only=False):
    print(json.dumps({
        "check_only": check_only,
        "config": str(config_path), "source_root": str(source_root),
        "config_type": type(config_path).__name__, "root_type": type(source_root).__name__,
        "ros_args": ros_args, "tools_path": sys.path[0],
        "config_content": json.loads(Path(config_path).read_text()),
        "forbidden_imports": [name for name in ("rclpy", "torch", "depthai", "pymavlink")
                              if name in sys.modules],
    }))
    return 2 if check_only else 0
''')
        config = Path(directory) / 'explicit-config.json'
        config.write_text(json.dumps({'fixture': True}))
        return source, config

    def run_entry(self, *args, **kwargs):
        env = dict(os.environ, PYTHONNOUSERSITE='1', CUDA_VISIBLE_DEVICES='')
        return subprocess.run([sys.executable, '-B', str(ENTRY)] + [str(a) for a in args],
                              env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              timeout=10, **kwargs)

    def test_ament_metadata_and_required_runtime_dependencies(self):
        xml = ET.parse(PACKAGE / 'package.xml').getroot()
        self.assertEqual(xml.findtext('name'), 'motar_camera_policy')
        self.assertEqual(xml.findtext('export/build_type'), 'ament_python')
        self.assertTrue((PACKAGE / 'resource' / xml.findtext('name')).is_file())
        self.assertEqual(xml.findtext('license'), 'BSD-3-Clause')
        dependencies = {e.text for e in xml.findall('exec_depend')}
        self.assertTrue({'rclpy', 'sensor_msgs', 'nav_msgs', 'geometry_msgs', 'std_msgs', 'std_srvs',
                         'launch', 'launch_ros', 'python3-numpy'}.issubset(dependencies))
        self.assertIn('lib/motar_camera_policy', (PACKAGE / 'setup.cfg').read_text())
        self.assertIn('camera_policy = motar_camera_policy.entry:main', (PACKAGE / 'setup.py').read_text())

    def test_template_has_no_invented_measurements_or_policy_binding(self):
        value = json.loads((PACKAGE / 'config' / 'unconfigured.json').read_text())
        self.assertEqual(set(value), {'format', 'binding', 'policy_contract_path', 'policy_contract_sha256', 'outputs'})
        self.assertEqual(value['format'], 'motar-ros2-node-v1')
        self.assertIsNone(value['policy_contract_path'])
        self.assertIsNone(value['policy_contract_sha256'])
        binding = value['binding']
        self.assertEqual(binding['status'], 'UNCONFIGURED')
        self.assertTrue(all(v is None for v in binding['frames'].values()))
        self.assertTrue(all(v is None for v in binding['geometry'].values()))
        self.assertTrue(all(v is None for v in binding['producer'].values()))
        self.assertIsNone(binding['clock']['source'])
        self.assertTrue(all(v.startswith('/motar/') for v in binding['topics'].values()))
        self.assertEqual(value['outputs'], {'velocity_topic': '/motar/policy/velocity_enu',
                                            'diagnostic_topic': '/motar/policy/diagnostics'})

    def test_import_has_no_ros_policy_or_device_side_effects(self):
        code = '''
import importlib.abc
import importlib.util
from pathlib import Path
import sys

class RejectRuntimeImports(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'rclpy', 'torch', 'depthai', 'pymavlink', 'launch', 'launch_ros'}:
            raise AssertionError('unexpected runtime import: ' + fullname)
sys.meta_path.insert(0, RejectRuntimeImports())
for index, path in enumerate(sys.argv[1:]):
    spec = importlib.util.spec_from_file_location('package_import_%d' % index, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
print('IMPORT_ONLY')
'''
        result = subprocess.run([sys.executable, '-B', '-c', code,
                                 str(PACKAGE / 'motar_camera_policy' / '__init__.py'), str(ENTRY),
                                 str(PACKAGE / 'launch' / 'camera_policy.launch.py')],
                                text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), 'IMPORT_ONLY')

    def test_check_dispatches_only_explicit_source_and_forwards_ros_arguments(self):
        with tempfile.TemporaryDirectory() as directory:
            source, config = self.fixture(directory)
            result = self.run_entry('--source-root', source, '--config', config, '--check',
                                    '--ros-args', '-r', '__node:=test_policy')
            self.assertEqual(result.returncode, 2, result.stderr)
            value = json.loads(result.stdout)
            self.assertTrue(value['check_only'])
            self.assertEqual(value['source_root'], str(source.resolve()))
            self.assertEqual(value['config'], str(config.resolve()))
            self.assertEqual(value['config_type'], 'PosixPath')
            self.assertEqual(value['root_type'], 'PosixPath')
            self.assertEqual(value['ros_args'], ['--ros-args', '-r', '__node:=test_policy'])
            self.assertEqual(value['config_content'], {'fixture': True})
            self.assertEqual(value['tools_path'], str(source / 'tools'))
            self.assertEqual(value['forbidden_imports'], [])

    def test_explicit_run_dispatch_is_distinct_from_check(self):
        with tempfile.TemporaryDirectory() as directory:
            source, config = self.fixture(directory)
            result = self.run_entry('--source-root', source, '--config', config)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertFalse(json.loads(result.stdout)['check_only'])

    def test_missing_or_relative_source_and_missing_config_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            source, config = self.fixture(directory)
            for args in (
                ['--config', config, '--check'],
                ['--source-root', 'relative-source', '--config', config, '--check'],
                ['--source-root', Path(directory), '--config', config, '--check'],
                ['--source-root', source, '--config', Path(directory) / 'missing.json', '--check'],
            ):
                with self.subTest(args=args):
                    result = self.run_entry(*args)
                    self.assertEqual(result.returncode, 2)
                    self.assertEqual(result.stdout, '')
                    self.assertIn('error:', result.stderr)

    def test_dispatch_restores_import_paths_and_private_module_on_return_or_error(self):
        entry = load(ENTRY, 'camera_ros2_package_test_entry')
        with tempfile.TemporaryDirectory() as directory:
            source, config = self.fixture(directory)
            paths = list(sys.path)
            old_module = types.ModuleType('existing_selected_source')
            with mock.patch.dict(sys.modules, {'_motar_camera_ros2_selected_source': old_module}):
                with mock.patch('builtins.print'):
                    self.assertEqual(entry.main(['--source-root', str(source), '--config', str(config), '--check']), 2)
                self.assertEqual(sys.path, paths)
                self.assertIs(sys.modules['_motar_camera_ros2_selected_source'], old_module)
                (source / 'tools' / 'camera_ros2_node.py').write_text('raise RuntimeError("fixture import failure")\n')
                with self.assertRaisesRegex(RuntimeError, 'fixture import failure'):
                    entry.main(['--source-root', str(source), '--config', str(config), '--check'])
                self.assertEqual(sys.path, paths)
                self.assertIs(sys.modules['_motar_camera_ros2_selected_source'], old_module)

    def test_launch_requires_both_paths_without_starting_a_graph(self):
        module = load(PACKAGE / 'launch' / 'camera_policy.launch.py', 'camera_ros2_package_test_launch')
        launch = types.ModuleType('launch')
        actions = types.ModuleType('launch.actions')
        substitutions = types.ModuleType('launch.substitutions')
        ros_actions = types.ModuleType('launch_ros.actions')
        launch.LaunchDescription = lambda contents: contents
        actions.DeclareLaunchArgument = lambda name, **kwargs: {'argument': name, **kwargs}
        substitutions.LaunchConfiguration = lambda name: {'configuration': name}
        ros_actions.Node = lambda **kwargs: {'node': kwargs}
        with mock.patch.dict(sys.modules, {'launch': launch, 'launch.actions': actions,
                                          'launch.substitutions': substitutions,
                                          'launch_ros': types.ModuleType('launch_ros'),
                                          'launch_ros.actions': ros_actions}):
            contents = module.generate_launch_description()
        self.assertEqual([item['argument'] for item in contents[:2]], ['source_root', 'config'])
        self.assertTrue(all('default_value' not in item for item in contents[:2]))
        node = contents[2]['node']
        self.assertEqual(node['package'], 'motar_camera_policy')
        self.assertEqual(node['executable'], 'camera_policy')
        self.assertEqual(node['arguments'], ['--source-root', {'configuration': 'source_root'},
                                             '--config', {'configuration': 'config'}])


if __name__ == '__main__':
    unittest.main()
