"""RT-S3W (2026-11-28): the camera flight runtime serving the CAM-S3 S3-W checkpoint (variant CAM-S3W).

Covers camera_policy_contract_cams2._cams3_block (sector rule from the receipt, measured depth training-only, 90 deg/s
yaw ceiling), the cams3 branch of camera_policy_runtime (navrl_cams3.install_sector after the no_lidar ablation) and a
CPU parity check in a fresh process: the same analytic scene through the TRAINING observation path (a second
perception of the same sector-installed module, the OAK-D depth model's deterministic part) and through the runtime (a
624x468 measured frame) gives the same 898-D observation. Independent controls:
  * the sector ranges both paths read equal a numpy re-implementation of the rule (k-th smallest range over every pixel
    of every column whose pinhole bearing lies in the 5-degree bin) on bins without target pixels;
  * negative control: the same training path with navrl_perception's ORIGINAL camera_no_return_to_lidar_range (the
    S2b-FT observation rule) differs on scenes with 0.2 m bars between the sampled columns;
  * yaw: a full yaw action publishes 1.5708 rad/s for CAM-S3W and 1.0472 rad/s for CAM-S2b (S2b-FT).
Every refusal is paired with an accepted positive control. Parts that need the local S3-W bundle
(~/motar_tmp/jetson_handoff_s3w_20261128) or the S2b-FT record are skipped where absent.
Limits: the scene is analytic (vertical bars, a red sphere, sky), not navrl_detector.render_raw_rgbd; the GPU
training/runtime parity of CAM-S2 (P2-6) is not run here; the per-step time is this host's, not the Orin NX's.
"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import camera_policy_contract as K               # noqa: E402
import camera_policy_contract_cams2 as S         # noqa: E402

HANDOFF = Path('/home/fair/motar_tmp/jetson_handoff_s3w_20261128')
RECEIPT = HANDOFF / 's3w_training_receipt.json'
CHECKPOINT = HANDOFF / 's3w_ep18000.pth'
OBS_NORM = HANDOFF / 's3w_ep18000_obs_norm.npz'
HAVE_S3W = RECEIPT.is_file() and CHECKPOINT.is_file() and OBS_NORM.is_file()
S2B_RECEIPT = Path('/home/fair/motar_tmp/cams2b_20261124/train_ft/receipt.json')
S2B_CHECKPOINT = (ROOT.parent / 'MOTAR-camonly/aerial_gym/rl_training/rl_games/runs/'
                  'ppo_261125_1344_navrl_v2-cams2b-ft-yaw60-camonly-s1/nn/last_gen_ppo_ep_15000_rew_51.018074.pth')
HAVE_S2B = S2B_RECEIPT.is_file() and S2B_CHECKPOINT.is_file()


def child_env():
    env = {key: v for key, v in os.environ.items() if not key.startswith('NAVRL_')}
    env.update(CUDA_VISIBLE_DEVICES='', PYTHONNOUSERSITE='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1',
               OPENBLAS_NUM_THREADS='1')
    return env


def run_child(script, args, tag):
    result = subprocess.run([sys.executable, '-B', '-c', script, str(ROOT / 'tools')] + [str(a) for a in args],
                            env=child_env(), capture_output=True, text=True, timeout=900, cwd=str(ROOT))
    if result.returncode != 0:
        raise AssertionError(result.stdout[-3000:] + result.stderr[-4000:])
    line = [x for x in result.stdout.splitlines() if x.startswith(tag + ' ')]
    if len(line) != 1:
        raise AssertionError(result.stdout[-2000:])
    return json.loads(line[0].split(' ', 1)[1]), result.stderr


class Cams3BlockPure(unittest.TestCase):
    """_cams3_block on synthetic receipts (no files, no torch); each refusal has an accepted control."""
    ENV = {'NAVRL_CAMS3_SECTOR_K': '24', 'NAVRL_DEPTH_MEAS': '1'}
    RECEIPT = {'cams3_sector': {'settings': {'NAVRL_CAMS3_SECTOR_K': 24}}, 'cams3_meas': {'settings': {}}}

    def block(self, env=None, receipt=None, variant='CAM-S3W'):
        return S._cams3_block(self.RECEIPT if receipt is None else receipt, self.ENV if env is None else env, variant)

    def test_accepts_and_records_measured_depth_off_on_vehicle(self):
        b = self.block()
        self.assertEqual(b['sector_k'], 24)
        self.assertIs(b['measured_depth_on_vehicle'], False)
        self.assertIs(b['measured_depth_in_training'], True)
        self.assertIn('MC_YAWRATE_MAX', b['yaw_limit_note'])

    def test_no_block_for_cams2_receipts(self):
        self.assertIsNone(S._cams3_block({}, {}, 'CAM-S2b'))
        self.assertIsNone(S._cams3_block({}, {}, 'CAM-S2a'))

    def test_refusals(self):
        cases = {
            'S3W without sector': dict(env={}, receipt={}),
            'sector for S2b': dict(variant='CAM-S2b'),
            'record disagrees': dict(receipt={'cams3_sector': {'settings': {'NAVRL_CAMS3_SECTOR_K': 12}},
                                              'cams3_meas': {}}),
            'record missing': dict(receipt={'cams3_meas': {}}),
            'meas record without switch': dict(env={'NAVRL_CAMS3_SECTOR_K': '24'}),
            'meas switch without record': dict(receipt={'cams3_sector': self.RECEIPT['cams3_sector']}),
            'meas switch garbage': dict(env={'NAVRL_CAMS3_SECTOR_K': '24', 'NAVRL_DEPTH_MEAS': 'yes'}),
            'bad k': dict(env={'NAVRL_CAMS3_SECTOR_K': '0', 'NAVRL_DEPTH_MEAS': '1'}),
        }
        for name, kw in cases.items():
            with self.subTest(name):
                with self.assertRaises(ValueError):
                    self.block(**kw)
        with self.assertRaises(ValueError):          # a sector record without the setting, on a CAM-S2b receipt
            S._cams3_block({'cams3_sector': {'settings': {}}}, {}, 'CAM-S2b')
        self.assertEqual(self.block()['sector_k'], 24)    # positive control after the refusals

    def test_measured_depth_never_reaches_the_flight_environment(self):
        flight = S.flight_environment(dict(self.ENV, NAVRL_DEPTH_MEAS_SEED='4943', NAVRL_YAW_RATE_MAX='1.5708'))
        self.assertEqual(flight, {'NAVRL_CAMS3_SECTOR_K': '24', 'NAVRL_YAW_RATE_MAX': '1.5708'})


@unittest.skipUnless(HAVE_S3W, 'local S3-W handoff bundle not available')
class Cams3wContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract = S.build_contract(RECEIPT, CHECKPOINT, OBS_NORM, previous_action_delay_steps=1,
                                        ground=dict(K.GROUND_DEFAULT, enabled=False))

    def test_fields_come_from_the_receipt(self):
        c, env = self.contract, json.loads(RECEIPT.read_text())['env_navrl']
        self.assertEqual((c['lineage'], c['variant']), ('CAM-S2', 'CAM-S3W'))
        self.assertEqual(c['action']['yaw_rate_max_rps'], float(env['NAVRL_YAW_RATE_MAX']))
        self.assertEqual(c['action']['yaw_rate_max_rps'], 1.5708)
        self.assertEqual(c['cams3']['sector_k'], int(env['NAVRL_CAMS3_SECTOR_K']))
        self.assertEqual(c['runtime_env']['NAVRL_CAMS3_SECTOR_K'], '24')
        self.assertFalse(any(k.startswith('NAVRL_DEPTH_MEAS') for k in c['runtime_env']))
        self.assertEqual(set(c['sources']), set(S.CAMS3_SOURCE_PATHS))
        self.assertIn(S.CAMS3_PATH, c['sources'])
        self.assertEqual(c['checkpoint']['sha256'],
                         '2626a4b24eb29b11c469904ac1a4e06de0349a04e9c83164b23a1e6308fa98ee')
        self.assertEqual(K.validate_contract(c), [])

    def test_refusals_each_with_the_accepted_contract_as_control(self):
        def edited(fn):
            c = json.loads(json.dumps(self.contract))
            fn(c)
            return K.validate_contract(c)
        mutations = {
            'sector k': lambda c: c['cams3'].__setitem__('sector_k', 12),
            'meas on vehicle': lambda c: c['cams3'].__setitem__('measured_depth_on_vehicle', True),
            'meas key in env': lambda c: c['runtime_env'].__setitem__('NAVRL_DEPTH_MEAS', '1'),
            'sector env': lambda c: c['runtime_env'].__setitem__('NAVRL_CAMS3_SECTOR_K', '1'),
            'yaw scale': lambda c: c['action'].__setitem__('yaw_rate_max_rps', 1.0472),
            'block removed': lambda c: c.pop('cams3'),
            'variant': lambda c: c.__setitem__('variant', 'CAM-S2b'),
            'cams3 source dropped': lambda c: c['sources'].pop(S.CAMS3_PATH),
        }
        for name, fn in mutations.items():
            with self.subTest(name):
                self.assertTrue(edited(fn), name)
        self.assertEqual(edited(lambda c: None), [])

    def test_receipt_refusals(self):
        receipt = json.loads(RECEIPT.read_text())

        def build(edit):
            d = json.loads(json.dumps(receipt))
            edit(d)
            with tempfile.TemporaryDirectory() as tmp:
                run = Path(tmp)
                (run / 'receipt.json').write_text(json.dumps(d))
                shutil.copy(HANDOFF / 'SHA256SUMS', run / 'SHA256SUMS')
                return S.build_contract(run / 'receipt.json', CHECKPOINT, OBS_NORM, previous_action_delay_steps=1,
                                        ground=dict(K.GROUND_DEFAULT, enabled=False))
        self.assertEqual(build(lambda d: None)['cams3']['sector_k'], 24)       # positive control
        edits = {
            'sector unset': lambda d: d['env_navrl'].pop('NAVRL_CAMS3_SECTOR_K'),
            'sector record differs': lambda d: d['cams3_sector']['settings'].__setitem__('NAVRL_CAMS3_SECTOR_K', 8),
            'cams3 source pin': lambda d: d.__setitem__('cams3_sha256', '0' * 64),
            'meas record missing': lambda d: d.pop('cams3_meas'),
            'unknown yaw': lambda d: d['env_navrl'].__setitem__('NAVRL_YAW_RATE_MAX', '2.0'),
        }
        for name, edit in edits.items():
            with self.subTest(name):
                with self.assertRaises(ValueError):
                    build(edit)


@unittest.skipUnless(HAVE_S2B, 'local S2b-FT training record not available')
class S2bContractUnchanged(unittest.TestCase):
    def test_s2b_contract_has_no_cams3_block_and_the_old_sources(self):
        with tempfile.TemporaryDirectory() as tmp:
            npz = Path(tmp) / 'n.npz'
            subprocess.run([sys.executable, '-B', str(ROOT / 'tools/camera_policy_export_obs_norm.py'),
                            str(S2B_CHECKPOINT), str(npz)], env=child_env(), check=True, capture_output=True,
                           timeout=300)
            c = S.build_contract(S2B_RECEIPT, S2B_CHECKPOINT, npz, previous_action_delay_steps=1,
                                 ground=dict(K.GROUND_DEFAULT, enabled=False))
        self.assertEqual(c['variant'], 'CAM-S2b')
        self.assertNotIn('cams3', c)
        self.assertEqual(set(c['sources']), set(S.SOURCE_PATHS))
        self.assertNotIn(S.CAMS3_PATH, c['sources'])
        self.assertEqual(c['action']['yaw_rate_max_rps'], 1.0472)
        self.assertNotIn('NAVRL_CAMS3_SECTOR_K', c['runtime_env'])


PARITY = r'''
import json, math, sys, time
import numpy as np
sys.path.insert(0, sys.argv[1])
import camera_policy_contract as K, camera_policy_contract_cams2 as S
from camera_policy_runtime import CameraPolicyRuntime
receipt, checkpoint, obs_norm = sys.argv[2:5]
c = S.build_contract(receipt, checkpoint, obs_norm, previous_action_delay_steps=1,
                     ground=dict(K.GROUND_DEFAULT, enabled=False))
r = CameraPolicyRuntime(c)
import torch, importlib
pm_mod = importlib.import_module('aerial_gym.task.navrl_task.navrl_perception')
dm = importlib.import_module('aerial_gym.task.navrl_task.navrl_depth_model')
cams2 = importlib.import_module('aerial_gym.task.navrl_task.navrl_cams2')
camsafe = importlib.import_module('aerial_gym.task.navrl_task.navrl_camsafe')
cams3 = importlib.import_module('aerial_gym.task.navrl_task.navrl_cams3')
sector_fn = pm_mod.camera_no_return_to_lidar_range
assert getattr(sector_fn, 'cams3_sector_k', None) == 24, 'runtime did not install the sector rule'
original_fn = sector_fn.cams3_original
tc = r.tc
assert float(tc.yaw_rate_max) == 1.5708, tc.yaw_rate_max
train = pm_mod.NavRLPerceptionModule(num_envs=1, device=torch.device('cpu'), cfg=tc.perception, step_dt=.1,
                                     camera_cfg=tc.vision)
plain = pm_mod.NavRLPerceptionModule(num_envs=1, device=torch.device('cpu'), cfg=tc.perception, step_dt=.1,
                                     camera_cfg=tc.vision)
env = json.loads(open(receipt).read())['env_navrl']
dms = {'NAVRL_DEPTH_MINZ_M': float(env['NAVRL_DEPTH_MINZ_M']), 'NAVRL_DEPTH_MAXZ_M': float(env['NAVRL_DEPTH_MAXZ_M']),
       'NAVRL_DEPTH_SIGMA_D_PX': 0., 'NAVRL_DEPTH_FB_PXM': float(env['NAVRL_DEPTH_FB_PXM']),
       'NAVRL_DEPTH_HOLE_P': 0., 'NAVRL_DEPTH_SEED': 0}
W, H = train.width, train.height
fx, fy, cx, cy = train.fx, train.fy, train.cx, train.cy
far_nohit = float(train.camera_obstacle_max_range)
uu, vv = np.meshgrid(np.arange(W), np.arange(H))
d_cam = np.stack([np.ones((H, W)), -(uu - cx) / fx, -(vv - cy) / fy], -1)
norm = np.linalg.norm(d_cam, axis=-1)
d_unit = d_cam / norm[..., None]
cos = 1. / norm
# radius 0.1 m bars (0.2 m wide), several between the sampled columns at 3-8 m
BARS = [(3.0, 0.8), (5.0, -1.2), (2.2, -0.4), (7.5, 1.5), (4.0, 2.6), (6.0, 0.45), (3.6, -0.15), (5.5, 1.05)]
GREY, SKY, RED = np.array([128, 128, 128], np.uint8), np.array([150, 180, 230], np.uint8), np.array([255, 0, 0], np.uint8)

def render(pos, yaw, target):
    ce, se = math.cos(yaw), math.sin(yaw)
    Ry = np.array([[ce, -se, 0], [se, ce, 0], [0, 0, 1.]])
    origin = np.asarray(pos) + Ry @ np.array([.08, 0., .03])
    d = d_unit @ Ry.T
    t = np.full((H, W), np.inf)
    colour = np.broadcast_to(SKY, (H, W, 3)).copy()
    for bx, by in BARS:
        ox, oy = origin[0] - bx, origin[1] - by
        a = d[..., 0]**2 + d[..., 1]**2
        b = 2 * (ox * d[..., 0] + oy * d[..., 1])
        cc = ox * ox + oy * oy - .1**2
        disc = b * b - 4 * a * cc
        with np.errstate(invalid='ignore', divide='ignore'):
            tt = (-b - np.sqrt(disc)) / (2 * a)
        zh = origin[2] + tt * d[..., 2]
        ok = (disc > 0) & (tt > 0) & (zh >= 0) & (zh <= 3) & (tt < t)
        t[ok] = tt[ok]
        colour[ok] = GREY
    oc = origin - np.asarray(target)
    b = 2 * (d @ oc)
    disc = b * b - 4 * (oc @ oc - .15**2)
    with np.errstate(invalid='ignore'):
        tt = (-b - np.sqrt(disc)) / 2
    ok = (disc > 0) & (tt > 0) & (tt < t)
    t[ok] = tt[ok]
    colour[ok] = RED
    return t, colour

def quat_yaw(yaw):
    return [0., 0., math.sin(yaw / 2), math.cos(yaw / 2)]

# Independent numpy re-implementation of the sector rule (navrl_cams3 docstring rule 2): bins of 5 degrees centred on
# the perception's lidar angles inside the FOV, columns by pinhole bearing atan((cx - u) / fx), k-th smallest of every
# pixel of those columns.
angles = train._lidar_angles.detach().cpu().double().numpy()
inside = np.abs(angles) <= float(train.hfov) * .5
alpha = np.arctan((float(train.cx) - np.arange(W)) / float(train.fx))
half = math.pi / len(angles)
bin_cols = [np.flatnonzero((alpha >= th - half) & (alpha < th + half)) for th in angles[inside]]

def numpy_sector(depth_hw, k=24):
    return np.array([np.sort(depth_hw[:, cols].reshape(-1))[k - 1] for cols in bin_cols])

vmax = float(tc.max_velocity)
hist = torch.zeros((1, 3, 4))
prev_train = torch.zeros((1, 4))
r.reset()
with torch.inference_mode():
    train.reset_idx(torch.tensor([0]))
    plain.reset_idx(torch.tensor([0]))
train.step_count = plain.step_count = 0
max_diff, plain_diff, sector_np_diff, sector_rt_diff, sector_bins_checked = 0., 0., 0., 0, 0
plain_diff_steps, prev_mismatch, yaw_err, times, sector_rt_finite = 0, 0, 0., [], 0
for i in range(14):
    t_s = 10. + .1 * i
    yaw = .05 * i
    pos = [.12 * i, .03 * i, 1.5 + .01 * math.sin(i)]
    vel = [1.2, .3, .1 * math.cos(i)]
    rate = .5
    target = [4.2 + .1 * i, .2 - .05 * i, 1.5]
    t, colour = render(pos, yaw, target)
    hit = np.isfinite(t)
    rng32 = np.where(hit, t, far_nohit).astype(np.float32)
    rgb_tr = torch.from_numpy(colour.transpose(2, 0, 1)[None].astype(np.float32) / 255.)
    with torch.inference_mode():
        depth_tr = dm.apply(train, torch.from_numpy(rng32[None]), dms)
        st = dict(rgb=rgb_tr, depth=depth_tr, lidar_m=torch.full((1, 288), float(tc.lidar_max_range)),
                  drone_pos_w=torch.tensor([pos], dtype=torch.float32), drone_vel_w=torch.tensor([vel], dtype=torch.float32),
                  vehicle_quat=torch.tensor([quat_yaw(yaw)], dtype=torch.float32),
                  yaw_rate=torch.tensor([rate / float(tc.yaw_rate_max)]), max_velocity=tc.max_velocity,
                  flight_altitude=tc.flight_altitude, training=False)
        obs_tr, _ = train.observe(previous_action=prev_train, **st)
        train_sector = train._cams3_sector_last[0].double().numpy().copy()
        # negative control: the S2b-FT rule (original column sample) on the same frame and history
        pm_mod.camera_no_return_to_lidar_range = original_fn
        try:
            obs_plain, _ = plain.observe(previous_action=prev_train, **dict(st, depth=dm.apply(
                plain, torch.from_numpy(rng32[None]), dms)))
        finally:
            pm_mod.camera_no_return_to_lidar_range = sector_fn
    # sector values against numpy on bins without target (red) pixels: depth_no_target == depth there
    red = np.all(colour == RED, axis=-1)
    expect = numpy_sector(depth_tr[0].double().numpy())
    for b, cols in enumerate(bin_cols):
        if not red[:, cols].any():
            sector_bins_checked += 1
            sector_np_diff = max(sector_np_diff, abs(float(expect[b]) - float(train_sector[b])))
    # runtime path: 624x468 frame, 2x2 sub-samples (one at the trained depth, the others farther or invalid)
    z = (t * cos).astype(np.float64)
    zf = np.repeat(np.repeat(z, 2, 0), 2, 1)
    cf = np.repeat(np.repeat(colour, 2, 0), 2, 1)
    trained_invalid = ~hit | (z < dms['NAVRL_DEPTH_MINZ_M']) | (z > dms['NAVRL_DEPTH_MAXZ_M'])
    inv = np.repeat(np.repeat(trained_invalid, 2, 0), 2, 1)
    rng = np.random.default_rng(i)
    sub = rng.integers(0, 4, (H, W))
    keep = np.zeros((2 * H, 2 * W), bool)
    yy, xx = np.meshgrid(np.arange(H), np.arange(W), indexing='ij')
    keep[2 * yy + sub // 2, 2 * xx + sub % 2] = True
    other = ~keep & ~inv
    flagged = other & (rng.random(other.shape) < .5)
    zf = np.where(other & ~flagged, zf + .3, zf)
    invf = inv | flagged
    zf = np.where(invf, 0., zf)
    cam = c['camera']
    frame = dict(rgb=cf, depth_z_m=zf.astype(np.float32), invalid=invf,
                 intrinsics={'width': 2 * W, 'height': 2 * H, 'fx': 2 * cam['fx'], 'fy': 2 * cam['fy'],
                             'cx': 2 * cam['cx'] + .5, 'cy': 2 * cam['cy'] + .5},
                 aligned=True, rectified=True, mount_flu_m=[.08, 0., .03], camera_to_body_rotation=np.eye(3),
                 rgb_t=t_s, depth_t=t_s)
    state = dict(pos=pos, vel=vel, quat_xyzw=quat_yaw(yaw), yaw_rate_rps=rate, t=t_s)
    t0 = time.perf_counter()
    out = r.step(frame, state, now_s=t_s + .1)
    times.append(time.perf_counter() - t0)
    obs_rt = np.asarray(out['observation'])
    max_diff = max(max_diff, float(np.abs(obs_rt - obs_tr[0].numpy()).max()))
    d_plain = float(np.abs(obs_plain[0].numpy() - obs_tr[0].numpy()).max())
    plain_diff = max(plain_diff, d_plain)
    plain_diff_steps += int(d_plain > 1e-4)
    # The runtime encodes no return as perception.max_camera_range (MaxZ 20 m), the training render as its no-hit
    # range (10 m); camera_no_return_to_lidar_range maps every value at or beyond the camera range to "no return", so
    # the two encodings are compared after that mapping (the observation parity above checks the consequence).
    rt_sector = np.asarray(out['cams3_sector_ranges_m'])
    cap = float(train.camera_obstacle_max_range) - 1e-3
    enc = lambda x: np.where(x >= cap, np.inf, x)
    same_class = np.array_equal(np.isinf(enc(rt_sector)), np.isinf(enc(train_sector)))
    finite = ~np.isinf(enc(train_sector))
    sector_rt_diff = max(sector_rt_diff, 0. if not finite.any() else float(np.abs(rt_sector[finite] - train_sector[finite]).max()),
                         0. if same_class else float('inf'))
    sector_rt_finite += int(finite.sum())
    a = torch.tensor([out['actor_output']], dtype=torch.float32).clamp(-1., 1.)
    yaw_err = max(yaw_err, abs(out['yaw_rate_rps'] - float(a[0, 3]) * 1.5708))
    hist = torch.roll(hist, 1, dims=1)
    hist[:, 0] = a
    applied = cams2.delayed(hist, torch.tensor([1]))
    governed = r._govern((torch.clamp(applied[:, 0:3], -1., 1.) * vmax)[:, 0:2].clone(), perception=train)
    capped, _ = camsafe.cap_blind(governed, float(train.hfov) * .5, 1.0)
    prev_train = torch.clamp(a[:, 0:4], -1., 1.).clone()
    prev_train[:, 0:2] = (capped / max(1e-6, vmax)).clamp(-1., 1.)
    prev_mismatch += int(not np.allclose(out['previous_action'], prev_train[0].numpy(), atol=1e-6))
# yaw scale at a full yaw action
policy_actions = r.audit.deterministic_actions
def full_yaw(player, obs):
    a = policy_actions(player, obs).reshape(1, 4).clone()
    a[0, 3] = 1.
    return a
r.audit.deterministic_actions = full_yaw
out = r.step(dict(frame, rgb_t=t_s + .1, depth_t=t_s + .1), dict(state, t=t_s + .1), now_s=t_s + .2)
print('CAMS3W_PARITY ' + json.dumps(dict(steps=14, max_abs_diff=max_diff, plain_diff=plain_diff,
      plain_diff_steps=plain_diff_steps, sector_numpy_diff=sector_np_diff, sector_bins_checked=sector_bins_checked,
      sector_runtime_diff=sector_rt_diff, sector_runtime_finite_bins=sector_rt_finite, prev_mismatch_steps=prev_mismatch, yaw_err=yaw_err,
      full_yaw_rps=out['yaw_rate_rps'], variant=out['variant'], sector_k=out['cams3_sector_k'],
      step_ms_median=1e3 * float(np.median(times[2:])), step_ms_max=1e3 * float(np.max(times[2:])),
      torch_threads=torch.get_num_threads())))
'''

S2B_YAW = r'''
import json, sys
import numpy as np
sys.path.insert(0, sys.argv[1])
import camera_policy_contract as K, camera_policy_contract_cams2 as S
from camera_policy_runtime import CameraPolicyRuntime
c = S.build_contract(sys.argv[2], sys.argv[3], sys.argv[4], previous_action_delay_steps=1,
                     ground=dict(K.GROUND_DEFAULT, enabled=False))
r = CameraPolicyRuntime(c)
import importlib
pm_mod = importlib.import_module('aerial_gym.task.navrl_task.navrl_perception')
installed = getattr(pm_mod.camera_no_return_to_lidar_range, 'cams3_sector_k', None)
policy_actions = r.audit.deterministic_actions
def full_yaw(player, obs):
    a = policy_actions(player, obs).reshape(1, 4).clone()
    a[0, 3] = 1.
    return a
r.audit.deterministic_actions = full_yaw
cam = c['camera']
W, H = cam['width'], cam['height']
z = np.full((2 * H, 2 * W), 4., np.float32)
frame = dict(rgb=np.full((2 * H, 2 * W, 3), 128, np.uint8), depth_z_m=z, invalid=np.zeros(z.shape, bool),
             intrinsics={'width': 2 * W, 'height': 2 * H, 'fx': 2 * cam['fx'], 'fy': 2 * cam['fy'],
                         'cx': 2 * cam['cx'] + .5, 'cy': 2 * cam['cy'] + .5},
             aligned=True, rectified=True, mount_flu_m=[.08, 0., .03], camera_to_body_rotation=np.eye(3),
             rgb_t=1., depth_t=1.)
out = r.step(frame, dict(pos=[0., 0., 1.5], vel=[0., 0., 0.], quat_xyzw=[0., 0., 0., 1.], yaw_rate_rps=0., t=1.),
             now_s=1.1)
print('S2B_YAW ' + json.dumps(dict(variant=out['variant'], full_yaw_rps=out['yaw_rate_rps'], sector_installed=installed,
      has_sector_report='cams3_sector_k' in out, cams3=r.cams3 is None)))
'''


@unittest.skipUnless(HAVE_S3W, 'local S3-W handoff bundle not available')
class Cams3wTrainingRuntimeParity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rep, cls.stderr = run_child(PARITY, [RECEIPT, CHECKPOINT, OBS_NORM], 'CAMS3W_PARITY')

    def test_observation_parity_with_the_sector_rule(self):
        rep = self.rep
        self.assertEqual((rep['variant'], rep['sector_k']), ('CAM-S3W', 24))
        self.assertLess(rep['max_abs_diff'], 1e-5, rep)
        self.assertEqual(rep['prev_mismatch_steps'], 0, rep)

    def test_sector_rule_matches_an_independent_reimplementation(self):
        rep = self.rep
        self.assertGreater(rep['sector_bins_checked'], 100, rep)        # non-vacuous
        self.assertEqual(rep['sector_numpy_diff'], 0., rep)
        self.assertGreater(rep['sector_runtime_finite_bins'], 50, rep)  # non-vacuous: bins with a return
        self.assertLess(rep['sector_runtime_diff'], 1e-5, rep)

    def test_s2b_rule_differs_on_thin_bars(self):
        rep = self.rep
        self.assertGreater(rep['plain_diff'], 1e-3, rep)                 # negative control is live
        self.assertGreater(rep['plain_diff_steps'], 0, rep)

    def test_yaw_scale_90_deg_per_s(self):
        self.assertLess(self.rep['yaw_err'], 1e-6, self.rep)
        self.assertAlmostEqual(self.rep['full_yaw_rps'], 1.5708, places=6)
        self.assertIn('install_sector', self.stderr)

    def test_host_step_time_reported(self):
        # Host number only (Orin NX timing is measured on the vehicle); a loose sanity bound.
        self.assertLess(self.rep['step_ms_median'], 2000., self.rep)
        print('\nRT-S3W host step time: median %.1f ms, max %.1f ms (%d torch threads)'
              % (self.rep['step_ms_median'], self.rep['step_ms_max'], self.rep['torch_threads']))


@unittest.skipUnless(HAVE_S2B, 'local S2b-FT training record not available')
class S2bRuntimeYaw(unittest.TestCase):
    def test_s2b_publishes_60_deg_per_s_without_the_sector_rule(self):
        with tempfile.TemporaryDirectory() as tmp:
            npz = Path(tmp) / 'n.npz'
            subprocess.run([sys.executable, '-B', str(ROOT / 'tools/camera_policy_export_obs_norm.py'),
                            str(S2B_CHECKPOINT), str(npz)], env=child_env(), check=True, capture_output=True,
                           timeout=300)
            rep, _ = run_child(S2B_YAW, [S2B_RECEIPT, S2B_CHECKPOINT, npz], 'S2B_YAW')
        self.assertEqual(rep['variant'], 'CAM-S2b')
        self.assertAlmostEqual(rep['full_yaw_rps'], 1.0472, places=6)
        self.assertIsNone(rep['sector_installed'])
        self.assertFalse(rep['has_sector_report'])
        self.assertTrue(rep['cams3'])


class ObsNormExport(unittest.TestCase):
    def test_refuses_overwrite_and_reads_both_key_layouts(self):
        import numpy as np
        import torch
        sys.path.insert(0, str(ROOT / 'tools'))
        import camera_policy_export_obs_norm as E
        m = {'running_mean_std.running_mean': torch.arange(898, dtype=torch.float32),
             'running_mean_std.running_var': torch.ones(898), 'running_mean_std.count': torch.tensor(5.)}
        mean, var, count = E.normaliser({'model': m})
        self.assertEqual((mean.dtype, mean.shape, float(count)), (np.float64, (898,), 5.))
        m2 = {'_orig_mod.' + k: v for k, v in m.items()}
        self.assertTrue(np.array_equal(E.normaliser({'model': m2})[0], mean))
        with self.assertRaises(ValueError):
            E.normaliser({'model': {'running_mean_std.running_mean': torch.zeros(10),
                                    'running_mean_std.running_var': torch.ones(10),
                                    'running_mean_std.count': torch.tensor(1.)}})
        with self.assertRaises(ValueError):
            E.normaliser({'model': {}})
        with tempfile.TemporaryDirectory() as tmp:
            existing = Path(tmp) / 'x.npz'
            existing.write_bytes(b'')
            with self.assertRaises(FileExistsError):
                E.export(Path(tmp) / 'missing.pth', existing)


if __name__ == '__main__':
    unittest.main()
