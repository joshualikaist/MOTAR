"""CPU CAM-K2-v3 inference from measured, aligned RGB-D and estimator state.

No simulator, synthetic depth model, target pose, LiDAR device, or MAVLink sender
is constructed. The 288 scan slots remain the trained no-return representation;
the production no_lidar adapter fills camera-visible governor bins from depth.
Run in a fresh process: production constants are fixed at import time.

The CLI replays an explicit NPZ input contract and writes proposed commands only.
Physical camera calibration, target appearance and closed-loop flight remain
separate qualifications. See the camera runtime record in the SITL evidence.
"""
import argparse
import importlib
import importlib.util
import inspect
import json
import math
import os
from pathlib import Path
import sys
import types

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MAX_AGE_S = 0.15
MAX_SKEW_S = 0.025
PERIOD_TOLERANCE_S = 0.025


class InputError(ValueError):
    """A rejected input must not advance the policy or perception history."""


def _finite(value, shape, name):
    a = np.asarray(value, dtype=np.float64)
    if a.shape != shape or not np.isfinite(a).all():
        raise InputError('%s must be finite with shape %s' % (name, shape))
    return a


def quaternion_rotation(q):
    """Estimator FLU-body -> ENU-world quaternion, xyzw; never HIL truth."""
    x, y, z, w = _finite(q, (4,), 'quat_xyzw')
    if abs(np.linalg.norm([x, y, z, w]) - 1.0) > 1e-3:
        raise InputError('quat_xyzw must be normalized')
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def validate_state(state):
    required = {'pos', 'vel', 'quat_xyzw', 'yaw_rate_rps', 't'}
    if set(state) != required:
        raise InputError('state requires only estimator fields %s' % sorted(required))
    p = _finite(state['pos'], (3,), 'pos')
    v = _finite(state['vel'], (3,), 'vel')
    r = quaternion_rotation(state['quat_xyzw'])
    try:
        rate, t = float(state['yaw_rate_rps']), float(state['t'])
    except (ValueError, TypeError):
        raise InputError('invalid estimator timestamp/yaw rate') from None
    if not math.isfinite(rate) or not math.isfinite(t) or t < 0:
        raise InputError('invalid estimator timestamp/yaw rate')
    yaw = math.atan2(r[1, 0], r[0, 0])
    return p, v, r, yaw, rate, t


def _intrinsics(c):
    try:
        w, h = c['width'], c['height']
        if type(w) is not int or type(h) is not int or not (1 <= w <= 4096 and 1 <= h <= 4096):
            raise ValueError()
        fx, fy, cx, cy = (float(c[k]) for k in ('fx', 'fy', 'cx', 'cy'))
        if not all(math.isfinite(x) for x in (fx, fy, cx, cy)) or min(fx, fy) <= 0:
            raise ValueError()
        if not (-0.5 <= cx <= w-0.5 and -0.5 <= cy <= h-0.5):
            raise ValueError()
    except (KeyError, TypeError, ValueError):
        raise InputError('invalid calibrated intrinsics') from None
    return w, h, fx, fy, cx, cy


def project_rgbd(frame, state, contract):
    """Rectify a calibrated body-mounted camera into the trained yaw-level camera.

    RGB and optical-z depth must already be rectified and aligned to the SAME
    camera intrinsics. Level/coincident cameras use inverse nearest sampling.
    For a tilted/translated camera, measured 3-D points are reprojected with a
    nearest-depth z buffer. Disocclusions remain explicit invalid pixels; no
    background or range is invented. This is an adapter, not sensor equivalence.
    """
    _, _, body_r, yaw, _, _ = validate_state(state)
    if frame.get('aligned') is not True or frame.get('rectified') is not True:
        raise InputError('RGB/depth must be explicitly aligned and rectified')
    sw, sh, sfx, sfy, scx, scy = _intrinsics(frame['intrinsics'])
    cam = contract['camera']
    tw, th, tfx, tfy, tcx, tcy = _intrinsics(cam)
    rgb = np.asarray(frame['rgb'])
    z = np.asarray(frame['depth_z_m'])
    invalid = np.asarray(frame['invalid'])
    if rgb.shape != (sh, sw, 3) or rgb.dtype != np.uint8:
        raise InputError('rgb must be aligned uint8 RGB [H,W,3]')
    if z.shape != (sh, sw) or not np.issubdtype(z.dtype, np.floating):
        raise InputError('depth_z_m must be floating optical metres [H,W]')
    if invalid.shape != z.shape or invalid.dtype != np.bool_:
        raise InputError('invalid must be a same-frame bool mask [H,W]')
    mount = _finite(frame['mount_flu_m'], (3,), 'mount_flu_m')
    optical = _finite(frame['camera_to_body_rotation'], (3, 3), 'camera_to_body_rotation')
    if not np.allclose(optical.T @ optical, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(optical), 1):
        raise InputError('camera_to_body_rotation must be a proper FLU rotation')
    cy, sy = math.cos(yaw), math.sin(yaw)
    yaw_r = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.]])
    rel = yaw_r.T @ body_r
    rotation = rel @ optical
    offset = rel @ mount - np.asarray(cam['translation_m'])
    depth = contract['depth']
    bad = invalid | ~np.isfinite(z) | (z < depth['minz_m']) | (z > depth['maxz_m'])
    if np.allclose(rotation, np.eye(3), atol=1e-8) and np.allclose(offset, 0, atol=1e-8):
        uu = (np.arange(tw)-tcx)/tfx*sfx + scx
        vv = (np.arange(th)-tcy)/tfy*sfy + scy
        # Reject unsupported FOV; silently padding it would invent observations.
        if min(uu.min(), vv.min()) < -0.5 or uu.max() > sw-0.5 or vv.max() > sh-0.5:
            raise InputError('source camera does not cover the trained FOV')
        xi = np.clip(np.floor(uu + .5).astype(int), 0, sw-1)
        yi = np.clip(np.floor(vv + .5).astype(int), 0, sh-1)
        out_rgb = rgb[yi[:, None], xi[None, :]].copy()
        out_z = z[yi[:, None], xi[None, :]].astype(np.float64)
        out_bad = bad[yi[:, None], xi[None, :]].copy()
        method = 'inverse-nearest'
    else:
        u, v = np.meshgrid((np.arange(sw)-scx)/sfx, (np.arange(sh)-scy)/sfy)
        good = ~bad
        points = np.stack([z[good], -u[good]*z[good], -v[good]*z[good]], axis=-1)
        points = points @ rotation.T + offset
        front = points[:, 0] > 1e-6
        points, colours = points[front], rgb[good][front]
        xx = np.floor(-points[:, 1]/points[:, 0]*tfx+tcx+.5).astype(int)
        yy = np.floor(-points[:, 2]/points[:, 0]*tfy+tcy+.5).astype(int)
        keep = (xx >= 0) & (xx < tw) & (yy >= 0) & (yy < th)
        indices = yy[keep]*tw + xx[keep]
        zs, colours = points[keep, 0], colours[keep]
        order = np.lexsort((zs, indices))
        indices, zs, colours = indices[order], zs[order], colours[order]
        first = np.r_[True, indices[1:] != indices[:-1]] if len(indices) else np.zeros(0, bool)
        out_z = np.zeros((th, tw), np.float64)
        out_rgb = np.zeros((th, tw, 3), np.uint8)
        out_bad = np.ones((th, tw), bool)
        out_z.flat[indices[first]] = zs[first]
        out_rgb.reshape(-1, 3)[indices[first]] = colours[first]
        out_bad.flat[indices[first]] = False
        out_bad |= (out_z < depth['minz_m']) | (out_z > depth['maxz_m'])
        method = 'measured-point-z-buffer'
    u, v = np.meshgrid((np.arange(tw)-tcx)/tfx, (np.arange(th)-tcy)/tfy)
    cosine = 1 / np.sqrt(1+u*u+v*v)
    ranges = np.full((th, tw), float(depth['maxz_m']), np.float64)
    np.divide(out_z, cosine, out=ranges, where=~out_bad)
    # Match depth-model ray-range far clamping, retaining the independent mask.
    ranges = np.minimum(ranges, float(depth['maxz_m'])).astype(np.float32)
    return (out_rgb.transpose(2, 0, 1)[None].astype(np.float32)/255., ranges[None],
            out_bad[None], cosine.astype(np.float32),
            {'projection': method, 'invalid_pixels': int(out_bad.sum()),
             'pixels': th*tw, 'synthetic_noise_applied': False})


def gazebo_frame(packet):
    """S4's RGB-D wire packet; ranges/lidar_t are deliberately never inspected."""
    hfov = math.radians(87.)
    f = 160/(2*math.tan(hfov/2))
    z = np.asarray(packet['depth'])
    return {'rgb': packet['rgb'], 'depth_z_m': z, 'invalid': ~np.isfinite(z) | (z <= 0),
            'rgb_t': packet['rgb_t'], 'depth_t': packet['depth_t'],
            'intrinsics': {'width': 160, 'height': 90, 'fx': f, 'fy': f, 'cx': 79.5, 'cy': 44.5},
            'aligned': True, 'rectified': True, 'mount_flu_m': [.1, 0, .03],
            'camera_to_body_rotation': np.eye(3)}


def _load(name, relative):
    spec = importlib.util.spec_from_file_location(name, str(ROOT / relative))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class CameraPolicyRuntime:
    """One sequential CPU policy/tracker session, without any device or sender."""

    def __init__(self, contract):
        from camera_policy_contract import validate_contract
        # Callers cannot change the checked geometry/environment after loading.
        contract = json.loads(json.dumps(contract, allow_nan=False))
        problems = validate_contract(contract)
        if problems:
            raise ValueError('; '.join(problems))
        # A prior F/perception import has already frozen different constants.
        if any(k == 'aerial_gym' or k.startswith('aerial_gym.') for k in sys.modules):
            raise RuntimeError('camera runtime requires a fresh process without aerial_gym imports')
        inherited = {k: v for k, v in os.environ.items() if k.startswith('NAVRL_')}
        collisions = [k for k, v in inherited.items() if contract['runtime_env'].get(k) != v]
        if collisions:
            raise ValueError('inherited NAVRL environment conflicts with contract: %s' % sorted(collisions))
        os.environ.update(contract['runtime_env'])
        self.contract = contract
        import torch
        self.torch = torch
        # Bare packages expose only simulator-free leaf modules, without Warp.
        for name, rel in (('aerial_gym', 'aerial_gym'), ('aerial_gym.task', 'aerial_gym/task'),
                          ('aerial_gym.task.navrl_task', 'aerial_gym/task/navrl_task'),
                          ('aerial_gym.config', 'aerial_gym/config'),
                          ('aerial_gym.config.task_config', 'aerial_gym/config/task_config')):
            mod = types.ModuleType(name)
            mod.__path__ = [str(ROOT / rel)]
            sys.modules[name] = mod
        sys.modules['aerial_gym'].AERIAL_GYM_DIRECTORY = str(ROOT)
        pm = importlib.import_module('aerial_gym.task.navrl_task.navrl_perception')
        self.ablation = importlib.import_module('aerial_gym.task.navrl_task.navrl_sensor_ablation')
        self.validity = importlib.import_module('aerial_gym.task.navrl_task.navrl_depth_validity')
        self.sg = importlib.import_module('aerial_gym.task.navrl_task.speed_governor')
        tc = importlib.import_module('aerial_gym.config.task_config.navrl_task_config').task_config
        tc.vision.detector_hfov_deg = contract['camera']['hfov_deg']
        tc.vision.detector_vfov_deg = contract['camera']['vfov_deg']
        self.ablation.install_perception(pm, 'no_lidar')
        if pm.STRUCTURED_OBS_DIM != 898 or (pm.VBEAMS, pm.HBEAMS) != (4, 72):
            raise RuntimeError('camera observation schema differs from trained 898-D')
        self.tc = tc
        self.perception = pm.NavRLPerceptionModule(num_envs=1, device=torch.device('cpu'),
                                                 cfg=tc.perception, step_dt=.1, camera_cfg=tc.vision)
        self.bearings = pm.lidar_bin_bearings(torch.device('cpu'))
        self.gov_cfg = self.sg.SpeedGovernorConfig.from_environ(os.environ)
        if (self.gov_cfg.mode != 'riskcap' or self.gov_cfg.yaw_cap_enabled
                or self.gov_cfg.lateral_channel_enabled or self.gov_cfg.open_width_enabled):
            raise RuntimeError('camera runtime supports the trained directional riskcap only')
        self.vfov = inspect.signature(self.sg.directional_lidar_clearance).parameters['vertical_fov_deg'].default
        audit = _load('camera_runtime_offline_audit', 'tools/navrl_reflection_offline_audit.py')
        ckpt = contract['checkpoint']['path']
        state = torch.load(ckpt, map_location='cpu', weights_only=False)
        schema = audit.read_checkpoint_schema(state)
        if schema['structured_obs_dim'] != 898:
            raise RuntimeError('checkpoint has wrong observation schema')
        exported = audit.apply_schema_environment(schema)
        # The shared audit also exports truncated-normal's default dmin for
        # squashed checkpoints, whose training snapshot correctly omits it.
        # It is not read by the trained squashed action model.
        relevant = {k: v for k, v in exported.items()
                    if not (k == 'NAVRL_TRUNCATED_DMIN' and schema['cfg_action_policy'] == 'squashed_gaussian')}
        if any(contract['runtime_env'].get(k) != v for k, v in relevant.items()):
            raise RuntimeError('checkpoint schema differs from recorded environment')
        del state
        original = torch.load
        def cpu_load(*a, **kw):
            kw.setdefault('map_location', 'cpu')
            return original(*a, **kw)
        torch.load = cpu_load
        try:
            self.player = audit.build_player(ckpt, schema, 'cpu', ROOT / 'aerial_gym/rl_training/rl_games/ppo_navrl_perception_transformer.yaml')
        finally:
            torch.load = original
        self.audit = audit
        self.reset()

    def reset(self):
        # Perception rolls histories during inference, producing inference-mode
        # tensors. Reset them under the same mode, including after failed ticks.
        with self.torch.inference_mode():
            self.perception.reset_idx(self.torch.tensor([0]))
        # Vectorized training resets individual environments while retaining its
        # global counter. This standalone reset starts a whole new history clock.
        self.perception.step_count = 0
        self.prev = self.torch.zeros((1, 4), dtype=self.torch.float32)
        self._last_stamps = None
        self._last_now = None
        self.failed = False

    def invalidate(self):
        """Stop reuse after the companion rejects a tick following inference."""
        self.failed = True

    def _validate_timing(self, frame, state, now_s):
        try:
            stamps = np.array([frame['rgb_t'], frame['depth_t'], state['t']], np.float64)
            now = float(now_s)
        except (TypeError, ValueError, KeyError):
            raise InputError('missing/invalid shared-clock timestamps') from None
        if stamps.shape != (3,) or not np.isfinite(stamps).all() or not math.isfinite(now):
            raise InputError('timestamps must be finite shared-clock seconds')
        ages = now-stamps
        if np.any(stamps < 0) or np.any(ages < -1e-9) or np.any(ages > MAX_AGE_S+1e-9):
            raise InputError('stale/future RGB-D or estimator sample')
        if np.ptp(stamps) > MAX_SKEW_S+1e-9:
            raise InputError('RGB-D/estimator capture skew exceeds declared bound')
        if self._last_stamps is not None and np.any(stamps <= self._last_stamps):
            raise InputError('duplicate/regressing RGB-D or estimator sample')
        dt = float(self.contract['observation']['control_dt_s'])
        if self._last_now is not None and abs(now-self._last_now-dt) > PERIOD_TOLERANCE_S+1e-9:
            raise InputError('control period differs from trained 10 Hz; reset required')
        return stamps, now

    def step(self, frame, state, *, now_s):
        if self.failed:
            raise RuntimeError('runtime failed after state advance; reset required')
        p, v, _, yaw, rate, _ = validate_state(state)
        stamps, now = self._validate_timing(frame, state, now_s)
        rgb, ranges, invalid, cosine, projection = project_rgbd(frame, state, self.contract)
        torch = self.torch
        depth, near = self.validity.apply_mask(torch.from_numpy(ranges), torch.from_numpy(invalid),
                                              torch.from_numpy(cosine), min_z_m=self.contract['depth']['minz_m'],
                                              window=self.contract['depth']['window'], density=self.contract['depth']['density'])
        T = lambda x: torch.as_tensor(x, dtype=torch.float32).reshape(1, -1)
        # Shape compatibility only, exactly as trained: no measured or rendered
        # LiDAR is accepted, looked up or used in perception/the governor.
        no_lidar = torch.full((1, 288), float(self.tc.lidar_max_range))
        qyaw = [0., 0., math.sin(yaw/2), math.cos(yaw/2)]
        self.failed = True
        with torch.inference_mode():
            obs, diag = self.perception.observe(rgb=torch.from_numpy(rgb), depth=depth, lidar_m=no_lidar,
                drone_pos_w=T(p), drone_vel_w=T(v), vehicle_quat=T(qyaw),
                yaw_rate=torch.tensor([rate/float(self.tc.yaw_rate_max)]), previous_action=self.prev,
                max_velocity=self.tc.max_velocity, flight_altitude=self.tc.flight_altitude, training=False)
            if tuple(obs.shape) != (1, 898) or not bool(torch.isfinite(obs).all()):
                raise RuntimeError('nonfinite/wrong-shape actor observation')
            a = self.audit.deterministic_actions(self.player, obs).reshape(1, 4)
            if not bool(torch.isfinite(a).all()) or bool((a.abs() > 1.000001).any()):
                raise RuntimeError('invalid actor output')
            a = a.clamp(-1., 1.)
            cmd = a[:, :2] * float(self.tc.max_velocity)
            scan = self.ablation.governor_scan(self.perception, 'no_lidar', no_lidar.reshape(1, 4, 72),
                                              float(self.tc.lidar_max_range), self.vfov)
            half = self.sg.speed_dependent_half_width(self.gov_cfg, cmd.norm(dim=1), open_m=None)
            clearance = self.sg.directional_lidar_clearance(scan, self.bearings, cmd,
                max_range_m=float(self.tc.lidar_max_range), path_half_width_m=half,
                target_return_mask=self.perception.last_target_like)
            clearance = torch.where(self.ablation.out_of_fov_commands(self.perception, cmd),
                                    torch.zeros_like(clearance), clearance)
            governed, _ = self.sg.apply_speed_governor(cmd, clearance, self.gov_cfg, lateral_clearance_m=None)
            prev = torch.cat([(governed/float(self.tc.max_velocity)).clamp(-1., 1.), a[:, 2:]], dim=1)
            if not bool(torch.isfinite(governed).all()) or not bool(torch.isfinite(prev).all()):
                raise RuntimeError('invalid governed command')
        vx, vy = governed[0].tolist()
        self.prev = prev
        self._last_stamps, self._last_now, self.failed = stamps, now, False
        return {'t': now, 'velocity_world_enu_mps': [math.cos(yaw)*vx-math.sin(yaw)*vy,
                                                   math.sin(yaw)*vx+math.cos(yaw)*vy, 0.],
                'velocity_vehicle_flu_mps': [vx, vy, 0.],
                'yaw_rate_rps': float(a[0, 3])*float(self.tc.yaw_rate_max),
                'previous_action': prev[0].tolist(), 'actor_output': a[0].tolist(),
                'observation': obs[0].tolist(), 'camera_visible': bool(diag['camera_visible'][0]),
                'lidar_visible': bool(diag['lidar_visible'][0]), 'depth_near_pixels': int(near.sum()),
                'projection': projection, 'checkpoint_sha256': self.contract['checkpoint']['sha256'],
                'command_sent': False}


def replay(runtime, data, output):
    """Recorded inputs only; no target state or simulator is ever constructed."""
    meta = json.loads(str(data['camera_metadata'].item()))
    required = ('rgb', 'depth_z_m', 'invalid', 'rgb_t', 'depth_t', 'pos', 'vel', 'quat_xyzw', 'yaw_rate_rps', 'state_t', 'now_s')
    counts = {len(data[k]) for k in required}
    if len(counts) != 1 or next(iter(counts)) == 0:
        raise InputError('replay arrays must contain equal nonzero frame counts')
    count = 0
    with Path(output).open('x') as stream:
        for i in range(next(iter(counts))):
            frame = dict(meta, **{k: data[k][i] for k in ('rgb', 'depth_z_m', 'invalid', 'rgb_t', 'depth_t')})
            state = {k: data[k][i] for k in ('pos', 'vel', 'quat_xyzw', 'yaw_rate_rps')}
            state['t'] = data['state_t'][i]
            result = runtime.step(frame, state, now_s=data['now_s'][i])
            stream.write(json.dumps(result, allow_nan=False) + '\n')
            count += 1
    return {'status': 'CPU_REPLAY_COMPLETE', 'frames': count, 'commands_sent': 0,
            'gazebo': 'NOT_TESTED', 'hardware': 'NOT_TESTED'}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--contract', required=True)
    ap.add_argument('--replay', required=True, help='aligned measured input NPZ, allow_pickle=False')
    ap.add_argument('--output', required=True, help='new JSONL path for proposed commands, never sent')
    args = ap.parse_args(argv)
    from camera_policy_contract import load_contract
    if Path(args.output).exists():
        raise ValueError('output exists; choose a fresh path')
    runtime = CameraPolicyRuntime(load_contract(args.contract))
    with np.load(args.replay, allow_pickle=False) as data:
        report = replay(runtime, data, args.output)
    print(json.dumps(report, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
