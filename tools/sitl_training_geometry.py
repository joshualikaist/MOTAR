"""Prepare an opt-in Gazebo collision correction; never install or launch it.

The recorded navrl_quad has four Iris-size rotor collision cylinders in addition
to the training URDF's only collision (a 0.28 x 0.28 x 0.08 m base box). Native
mode preserves the original bytes. training-box removes only those four XML
elements; every other byte, including dynamics/plugins/sensors, stays intact.
Historical assets and experiment launchers are never modified.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import re
from xml.etree import ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SOURCE = Path('/home/fair/workspaces/px4_ws/PX4-Autopilot/Tools/simulation/'
              'gazebo-classic/sitl_gazebo-classic/models/navrl_quad/navrl_quad.sdf')
SOURCE_SHA = 'f9188e906a23dfa498328beef0eb69b57b50e4d2c8acc06c8ba7d3f022fcd8b2'
URDF = ROOT / 'resources/robots/quad/quad_navrl_collide.urdf'
URDF_SHA = 'ad9a3110037cb01f94caa11054e367e557e9089b4edd520d0aa59b78578988be'
MODES = ('native', 'training-box')
ROTOR_NAMES = tuple('rotor_%d_collision' % i for i in range(4))
BOX = (0.28, 0.28, 0.08)
MODEL_CONFIG = (b'<?xml version="1.0"?><model><name>navrl_quad</name><version>1.0</version>'
                b'<sdf version="1.6">navrl_quad.sdf</sdf><author><name>MOTAR</name></author>'
                b'<description>Opt-in collision geometry diagnostic</description></model>')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def numbers(text, expected):
    if text is None or tuple(float(x) for x in text.split()) != tuple(expected):
        raise ValueError('unqualified collision geometry: %r != %r' % (text, expected))


def _validated(source, urdf):
    sdf = ET.fromstring(source)
    train = ET.fromstring(urdf)
    collisions = train.findall('.//collision')
    if len(collisions) != 1 or train.find('link[@name="base_link"]/collision') is not collisions[0]:
        raise ValueError('training URDF must have exactly one base collision')
    box = collisions[0].find('geometry/box')
    origin = collisions[0].find('origin')
    if box is None or origin is None:
        raise ValueError('missing training base box/origin')
    numbers(box.get('size'), BOX)
    numbers(origin.get('xyz'), (0, 0, 0))
    numbers(origin.get('rpy', '0 0 0'), (0, 0, 0))
    models = sdf.findall('model')
    if len(models) != 1 or models[0].get('name') != 'navrl_quad':
        raise ValueError('expected the single navrl_quad model')
    model = models[0]
    all_collisions = model.findall('.//collision')
    if sorted(c.get('name') for c in all_collisions) != sorted(('base_link_inertia_collision',) + ROTOR_NAMES):
        raise ValueError('unknown, duplicate or missing SDF collision')
    base = model.find('link[@name="base_link"]')
    if base is None or len(base.findall('collision')) != 1:
        raise ValueError('missing base collision')
    numbers(base.findtext('pose'), (0, 0, 0, 0, 0, 0))
    c = base.find('collision')
    numbers(c.findtext('pose'), (0, 0, 0, 0, 0, 0))
    numbers(c.findtext('geometry/box/size'), BOX)
    for i, (x, y) in enumerate(((.13, -.13), (-.13, .13), (.13, .13), (-.13, -.13))):
        link = model.find('link[@name="rotor_%d"]' % i)
        if link is None or len(link.findall('collision')) != 1:
            raise ValueError('unknown rotor link')
        numbers(link.findtext('pose'), (x, y, 0, 0, 0, 0))
        c = link.find('collision')
        if c.get('name') != ROTOR_NAMES[i]:
            raise ValueError('rotor collision attached to the wrong link')
        numbers(c.findtext('pose'), (0, 0, 0, 0, 0, 0))
        numbers(c.findtext('geometry/cylinder/radius'), (.128,))
        numbers(c.findtext('geometry/cylinder/length'), (.005,))
    return sdf


def build(source, urdf, mode='native'):
    """Return (SDF bytes, receipt); validation applies even to the OFF control."""
    if mode not in MODES:
        raise ValueError('unknown geometry mode')
    tree = _validated(source, urdf)
    removed = []
    if mode == 'training-box':
        def replace(match):
            node = ET.fromstring(match.group(0))
            if node.get('name') in ROTOR_NAMES:
                removed.append(node.get('name'))
                return b''
            return match.group(0)
        result = re.sub(rb'<collision\b[^>]*>.*?</collision\s*>', replace, source, flags=re.DOTALL)
        if sorted(removed) != sorted(ROTOR_NAMES):
            raise ValueError('could not isolate exactly four rotor collision elements')
        expected = copy.deepcopy(tree)
        for link in expected.findall('model/link'):
            for c in link.findall('collision'):
                if c.get('name') in ROTOR_NAMES:
                    # Preserve the surrounding whitespace, just as the byte splice does.
                    prior = list(link).index(c)
                    if prior:
                        link[prior - 1].tail = (link[prior - 1].tail or '') + (c.tail or '')
                    else:
                        link.text = (link.text or '') + (c.tail or '')
                    link.remove(c)
        if ET.tostring(expected) != ET.tostring(ET.fromstring(result)):
            raise ValueError('model changed beyond rotor collisions')
    else:
        result = source
    return result, {'schema': 'motar.sitl-training-geometry.v1', 'mode': mode,
                    'source_sdf_sha256': sha(source), 'training_urdf_sha256': sha(urdf),
                    'output_sdf_sha256': sha(result), 'removed_collision_names': removed,
                    'base_box_size_m': list(BOX), 'execution_supported': False,
                    'native_byte_identity': result == source,
                    'qualification': 'STRUCTURAL_ONLY; flight/contact response NOT_TESTED'}


def prepare(directory, mode='native', source=SOURCE, urdf=URDF):
    """Write a fresh external model package. Existing assets are read-only."""
    directory = Path(directory).resolve()
    source, urdf = Path(source).resolve(), Path(urdf).resolve()
    # Other Git worktrees/main/raw are also protected; packages belong in motar_tmp.
    scratch = (Path.home() / 'motar_tmp').resolve()
    if scratch not in directory.parents or directory.exists():
        raise ValueError('output must be a fresh directory below ~/motar_tmp')
    data, training = source.read_bytes(), urdf.read_bytes()
    if sha(data) != SOURCE_SHA or sha(training) != URDF_SHA:
        raise ValueError('source SDF or training URDF differs from the reviewed assets')
    result, receipt = build(data, training, mode)
    receipt.update(source_sdf=str(source), training_urdf=str(urdf))
    model = directory / 'navrl_quad'
    model.mkdir(parents=True)
    (model / 'navrl_quad.sdf').write_bytes(result)
    config = MODEL_CONFIG
    (model / 'model.config').write_bytes(config)
    receipt['model_config_sha256'] = sha(config)
    receipt['output_directory'] = str(directory)
    (directory / 'geometry.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return receipt


def launch_plan(owned, episode_dir, world_name, base_env, package):
    """Pure opt-in plan adapter; reuses owned-session validation and starts nothing.

    PX4 keeps its original airframe/parameters. Only the spawn SDF is replaced;
    original GAZEBO_MODEL_PATH remains available for included mesh resources.
    The package receipt and its actual bytes are verified before plan creation.
    """
    package = Path(package).resolve()
    receipt = json.loads((package / 'geometry.json').read_text())
    source = Path(receipt['source_sdf']).read_bytes()
    urdf = Path(receipt['training_urdf']).read_bytes()
    if sha(source) != SOURCE_SHA or sha(urdf) != URDF_SHA:
        raise ValueError('package source assets have drifted')
    output, expected = build(source, urdf, receipt['mode'])
    sdf = package / 'navrl_quad/navrl_quad.sdf'
    if sdf.read_bytes() != output or any(receipt.get(k) != v for k, v in expected.items()):
        raise ValueError('model package differs from the selected correction')
    if ((package / 'navrl_quad/model.config').read_bytes() != MODEL_CONFIG
            or receipt['model_config_sha256'] != sha(MODEL_CONFIG)):
        raise ValueError('model.config differs')
    plan = owned.direct_launch_plan(episode_dir, world_name, base_env)
    old = str(plan['sdf_path'])
    plan['sdf_path'] = sdf
    args = plan['spawn']['argv']
    if args.count('--spawn-file=' + old) != 1:
        raise ValueError('unknown spawn plan')
    args[args.index('--spawn-file=' + old)] = '--spawn-file=' + str(sdf)
    plan['geometry_receipt'] = {'path': str(package / 'geometry.json'),
                                'sha256': sha((package / 'geometry.json').read_bytes())}
    # The native resource paths are retained: the model's plugins and meshes are unchanged.
    owned.validate_plan(plan, episode_dir)
    return plan


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=MODES, default='native')
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args(argv)
    print(json.dumps(prepare(args.out, args.mode), indent=2))


if __name__ == '__main__':
    main()
