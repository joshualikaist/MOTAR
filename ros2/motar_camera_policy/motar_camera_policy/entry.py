"""Load the requested source entrypoint; no ROS or device imports at package import."""
import argparse
import importlib.util
from pathlib import Path
import sys


def _dispatch(argv, source_file):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', required=True, type=Path,
                        help='absolute MOTAR source tree containing tools/' + source_file)
    parser.add_argument('--config', required=True, type=Path,
                        help='explicit ROS2 camera-policy JSON configuration')
    parser.add_argument('--check', action='store_true',
                        help='metadata validation only; does not require ROS or restore a policy')
    args, ros_args = parser.parse_known_args(argv)
    if not args.source_root.is_absolute():
        parser.error('--source-root must be an explicit absolute path')
    root = args.source_root.resolve()
    source = root / 'tools' / source_file
    if not source.is_file():
        parser.error('--source-root does not contain tools/' + source_file)
    config = args.config.expanduser().resolve()
    if not config.is_file():
        parser.error('--config must name an existing JSON configuration')

    # Keep this path active during node execution: its lazy source imports must
    # resolve in the selected checkout, not in an editable installation elsewhere.
    module_name = '_motar_camera_ros2_selected_source' if source_file == 'camera_ros2_node.py' else '_motar_px4_ros2_selected_source'
    original_path = list(sys.path)
    previous = sys.modules.get(module_name)
    spec = importlib.util.spec_from_file_location(module_name, source)
    if spec is None or spec.loader is None:
        parser.error('cannot load the explicitly selected source entrypoint')
    node = importlib.util.module_from_spec(spec)
    try:
        sys.path.insert(0, str(root / 'tools'))
        sys.modules[module_name] = node
        spec.loader.exec_module(node)
        return node.main(ros_args, config_path=config, source_root=root, check_only=args.check)
    finally:
        sys.path[:] = original_path
        if previous is None:
            sys.modules.pop(module_name, None)
        else:
            sys.modules[module_name] = previous


def main(argv=None):
    return _dispatch(argv, 'camera_ros2_node.py')


def gateway_main(argv=None):
    return _dispatch(argv, 'camera_ros2_px4_node.py')


if __name__ == '__main__':
    raise SystemExit(main())
