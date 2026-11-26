"""Runtime v1 (2026-11-24): the camera flight runtime serving the CAM-S2 camera-only policies.

Covers camera_policy_contract_cams2.py (lineage contract from a training receipt), the CAM-S2 branch of
camera_policy_runtime.py (min-valid projection with invalid depth = no return, altitude/mount from navrl_cams2, blind
cap, trained previous-action channel, 0.35 s capture-age bound, near-constant slot report) and a CPU parity check: the
same simulated scene through the TRAINING observation path (navrl_depth_model.apply on a trained-resolution render,
the training previous-action rule built from navrl_cams2.delayed and navrl_camsafe.cap_blind, the task's observe
arguments) and through the runtime (a 624x468 measured frame) gives the same 898-D observation.

Every refusal is paired with an accepted positive control. The integration parts run in fresh CPU processes and are
skipped where the local CAM-S2a training record or checkpoint is absent.

Limits of the CPU parity (what it does NOT cover) and the GPU check that would close them (needs a preregistration and
the user's approval; NOT run):
  * the scene is an analytic renderer of vertical bars, a red sphere and sky, not navrl_detector.render_raw_rgbd; the
    governor on the training side is the runtime's _govern replica (shared with CAM-K2 v3), not
    navrl_task._apply_sensor_speed_governor; P9 is off on the training side (it is training-only by design);
  * GPU check: launch tools/realveh_entry.py with the receipt's env_navrl, except NAVRL_P9_ERROR_MODEL/SHA256 unset,
    NAVRL_DEPTH_SIGMA_D_PX=0, NAVRL_DEPTH_HOLE_P=0 and NAVRL_CAMS2_DELAY_MIN=MAX=k (k = 1, then 2), 1 environment, the
    ep12000 checkpoint in play mode, 200 steps, density 70 and 175. An eval-only recorder wraps
    NavRLPerceptionModule.observe OUTSIDE the depth-model wrapper and stores per step the rendered rgb/depth (before
    the depth model), drone_pos_w, drone_vel_w, vehicle_quat, the body yaw rate, previous_action and the returned
    observation, plus the policy action. Offline on CPU, each record becomes a runtime frame (z = depth x cos,
    invalid = depth-model invalid mask | renderer no-hit, each pixel repeated 2x2, intrinsics 2x the trained ones,
    mount [0.08, 0, 0.03], identity camera rotation), is stepped through CameraPolicyRuntime with
    previous_action_delay_steps = k and now_s = capture + 0.1, and must give max |obs_runtime - obs_recorded| < 1e-5 on
    every step and the recorded previous_action on every step after the first.
"""
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import camera_policy_runtime as R          # noqa: E402
import camera_policy_contract as K         # noqa: E402

RECEIPT = Path('/home/fair/motar_tmp/cams2a_20261123/train_s1/receipt.json')
HANDOFF = Path('/home/fair/motar_tmp/jetson_handoff_cams2a_20261124')
CHECKPOINT = HANDOFF / 'cams2a_s1_ep12000.pth'
OBS_NORM = HANDOFF / 'cams2a_s1_ep12000_obs_norm.npz'
HAVE_S2A = RECEIPT.is_file() and CHECKPOINT.is_file() and OBS_NORM.is_file()


def trained_camera():
    w, h, hfov, vfov = 312, 234, 66.7, 52.6
    return {'width': w, 'height': h, 'hfov_deg': hfov, 'vfov_deg': vfov,
            'fx': w / (2 * math.tan(math.radians(hfov / 2))), 'fy': h / (2 * math.tan(math.radians(vfov / 2))),
            'cx': (w - 1) / 2, 'cy': (h - 1) / 2, 'translation_m': [.08, 0., .03]}


def geometry_contract():
    # Geometry-only: the fields project_rgbd_min_valid reads.
    return {'lineage': 'CAM-S2', 'camera': trained_camera(),
            'depth': {'minz_m': .315, 'maxz_m': 20.}, 'observation': {'control_dt_s': .1}}


def full_res_intrinsics(cam, cy=None):
    """The 624x468 source whose 2x2 blocks are exactly the trained pixels (cy may be overridden)."""
    return {'width': 2 * cam['width'], 'height': 2 * cam['height'], 'fx': 2 * cam['fx'], 'fy': 2 * cam['fy'],
            'cx': 2 * cam['cx'] + .5, 'cy': 2 * cam['cy'] + .5 if cy is None else cy}


def frame(z, invalid, rgb=None, intrinsics=None, mount=(.08, 0., .03)):
    h, w = z.shape
    return {'rgb': np.full((h, w, 3), 128, np.uint8) if rgb is None else rgb, 'depth_z_m': z.astype(np.float32),
            'invalid': invalid, 'intrinsics': intrinsics, 'aligned': True, 'rectified': True,
            'mount_flu_m': list(mount), 'camera_to_body_rotation': np.eye(3), 'rgb_t': 1., 'depth_t': 1.}


STATE = {'pos': [0., 0., 1.5], 'vel': [0., 0., 0.], 'quat_xyzw': [0., 0., 0., 1.], 'yaw_rate_rps': 0., 't': 1.}


class MinValidReduction(unittest.TestCase):
    def test_block_rule_min_over_valid_and_all_invalid(self):
        z = np.array([[1., 2., 5., 5.], [3., .5, 5., 5.]])
        invalid = np.array([[False, False, True, True], [False, True, True, True]])
        rgb = np.zeros((2, 4, 3), np.uint8)
        rgb[0, 0] = [10, 0, 0]
        rgb[1, 1] = [99, 0, 0]
        f = frame(z, invalid, rgb, {'width': 4, 'height': 2, 'fx': 4., 'fy': 4., 'cx': 1.5, 'cy': .5})
        out = R.downscale_min_valid_2x2(f)
        # Block 0: 0.5 is flagged, so the nearest VALID sample is 1.0 with its colour; block 1 has none -> invalid.
        self.assertEqual(out['depth_z_m'].tolist(), [[1., 0.]])
        self.assertEqual(out['invalid'].tolist(), [[False, True]])
        self.assertEqual(out['rgb'][0, 0].tolist(), [10, 0, 0])
        self.assertEqual(out['intrinsics'], {'width': 2, 'height': 1, 'fx': 2., 'fy': 2., 'cx': .5, 'cy': 0.})
        # MinZ/MaxZ make an out-of-range sample invalid before the minimum (the OAK-D reports 0 there).
        out = R.downscale_min_valid_2x2(f, minz_m=1.5)
        self.assertEqual(out['depth_z_m'].tolist(), [[2., 0.]])
        with self.assertRaises(R.InputError):
            R.downscale_min_valid_2x2(dict(f, depth_z_m=np.zeros((3, 4), np.float32), invalid=np.zeros((3, 4), bool),
                                           rgb=np.zeros((3, 4, 3), np.uint8)))

    def test_level_projection_equals_block_rule_and_never_invents_near(self):
        c = geometry_contract()
        cam = c['camera']
        rng = np.random.default_rng(3)
        z = rng.uniform(.5, 15., (468, 624))
        invalid = rng.random((468, 624)) < .3
        invalid[:20] = True                                   # the real stream's invalid top band
        invalid[100:104, 200:210] = True                      # whole blocks invalid
        f = frame(z, invalid, intrinsics=full_res_intrinsics(cam))
        rgb, ranges, si, nr, cosine, report = R.project_rgbd_min_valid(f, STATE, c)
        self.assertTrue(report['level_camera'])
        ref = R.downscale_min_valid_2x2(f, minz_m=.315, maxz_m=20.)
        hit = ~(si[0] | nr[0])
        np.testing.assert_array_equal(hit, ~ref['invalid'])
        np.testing.assert_allclose(ranges[0][hit] * cosine[hit], ref['depth_z_m'][hit], rtol=1e-6)
        self.assertTrue((ranges[0][~hit] == 20.).all())       # no return holds MaxZ, never a near surface
        self.assertGreater(int((~hit).sum()), 0)              # non-vacuous: invalid blocks exist
        self.assertGreater(int(hit.sum()), 60000)             # positive control: valid blocks kept
        self.assertTrue(si[0][:10].all())                     # top band: imaged, invalid -> no return
        # The CAM-K2 path on the same frame would turn the dense invalid band into a near blob; this path cannot:
        self.assertEqual(float(ranges[0][:10].min()), 20.)

    def test_real_principal_point_is_reprojected_not_block_reduced(self):
        c = geometry_contract()
        cam = c['camera']
        # A horizontal edge 0.5 m below the camera axis at 4 m: below it a near wall (2 m), above it far (8 m).
        intr = full_res_intrinsics(cam, cy=246.7)            # the measured 624x468 principal point
        v = (np.arange(468) - intr['cy']) / intr['fy']
        z = np.where(v[:, None] * 4. > .5, 2., 8.) * np.ones((1, 624))
        f = frame(z, np.zeros((468, 624), bool), intrinsics=intr)
        _, ranges, _, _, cosine, _ = R.project_rgbd_min_valid(f, STATE, c)
        zt = ranges[0] * cosine
        trained_row = int(np.argmax(zt[:, 156] < 3.))
        expected = int(math.ceil(cam['cy'] + .5 / 4. * cam['fy'] - .5))
        self.assertLessEqual(abs(trained_row - expected), 1)
        # Control: a plain 2x2 reduction keeps the source principal point and lands the edge ~6 rows lower.
        plain = R.downscale_min_valid_2x2(f)
        plain_row = int(np.argmax(plain['depth_z_m'][:, 156] < 3.))
        self.assertGreaterEqual(plain_row - expected, 5)

    def test_tilted_full_resolution_source_leaves_no_interior_holes(self):
        c = geometry_contract()
        cam = c['camera']
        pitch = math.radians(8.)                                  # nose down
        q = [0., math.sin(pitch / 2), 0., math.cos(pitch / 2)]
        state = dict(STATE, quat_xyzw=q)
        f = frame(np.full((468, 624), 6.), np.zeros((468, 624), bool), intrinsics=full_res_intrinsics(cam))
        _, _, si, nr, _, report = R.project_rgbd_min_valid(f, state, c)
        self.assertFalse(report['level_camera'])
        interior = nr[0][40:-40, 20:-20]
        self.assertEqual(int(interior.sum()), 0)
        self.assertEqual(report['unsampled_imaged_pixels'], 0)
        # Control (review r1 P1-1): reprojecting a 312x234 block-reduced source at this attitude leaves sampling holes,
        # which this lineage would read as free space: refused, not accepted silently.
        half = R.downscale_min_valid_2x2(f)
        with self.assertRaisesRegex(R.InputError, 'received no source sample'):
            R.project_rgbd_min_valid(half, state, c)
        # The tilt band the camera does not image is no-return (top rows for nose-down), never invented.
        self.assertGreater(int(nr[0][:5].sum()), 0)

    def test_source_resolution_guard(self):
        """Review r1 P1-1: a source coarser than the trained camera is refused; 624x468 and an exact 312x234 accepted."""
        c = geometry_contract()
        cam = c['camera']
        full = frame(np.full((468, 624), 3.), np.zeros((468, 624), bool), intrinsics=full_res_intrinsics(cam, cy=246.7))
        exact = frame(np.full((234, 312), 3.), np.zeros((234, 312), bool),
                      intrinsics={k: cam[k] for k in ('width', 'height', 'fx', 'fy', 'cx', 'cy')})
        # The vehicle side's source_decimation 3: 208x156, f/3 (158 px/rad against the trained 237 px/rad).
        coarse = frame(np.full((156, 208), 3.), np.zeros((156, 208), bool),
                       intrinsics={'width': 208, 'height': 156, 'fx': 2 * cam['fx'] / 3, 'fy': 2 * cam['fy'] / 3,
                                   'cx': 103.5, 'cy': 77.5})
        accepted = 0
        for f in (full, exact):
            out = R.project_rgbd_min_valid(f, STATE, c)
            self.assertEqual(out[5]['unsampled_imaged_pixels'], 0)
            self.assertGreater(out[5]['valid_pixels'], 60000)
            accepted += 1
        self.assertEqual(accepted, 2)
        with self.assertRaisesRegex(R.InputError, 'coarser than the trained camera'):
            R.project_rgbd_min_valid(coarse, STATE, c)
        # The guard is on the focal length, not on the pixel count: a 3x-decimated stream with a narrower lens is fine
        # only if fx/fy reach the trained values.
        with self.assertRaises(R.InputError):
            R.project_rgbd_min_valid(dict(exact, intrinsics=dict(exact['intrinsics'], fy=cam['fy'] * .99)), STATE, c)

    def test_full_resolution_attitudes_have_no_sampling_holes(self):
        c = geometry_contract()
        cam = c['camera']
        f = frame(np.full((468, 624), 5.), np.zeros((468, 624), bool), intrinsics=full_res_intrinsics(cam, cy=246.7))
        cases = 0
        for roll, pitch in ((0., 2.5), (10., 10.), (20., 0.), (0., 20.), (0., 30.), (-15., -10.)):
            r, p = math.radians(roll), math.radians(pitch)
            q = [math.sin(r/2)*math.cos(p/2), math.cos(r/2)*math.sin(p/2), -math.sin(r/2)*math.sin(p/2),
                 math.cos(r/2)*math.cos(p/2)]
            out = R.project_rgbd_min_valid(f, dict(STATE, quat_xyzw=q), c)
            self.assertEqual(out[5]['unsampled_imaged_pixels'], 0)
            self.assertGreater(out[5]['imaged_pixels'], 30000)
            cases += 1
        self.assertEqual(cases, 6)

    def test_floor_filter_removes_floor_returns_and_keeps_a_wall(self):
        c = geometry_contract()
        cam = c['camera']
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp) / 'height.txt'
            evidence.write_text('CPU fixture only')
            c.update(ground=dict(K.GROUND_DEFAULT), ceiling=dict(K.CEILING_DEFAULT),
                     estimator_height=K.estimator_height_assertion(evidence))
            intr = full_res_intrinsics(cam)
            v = (np.arange(468) - intr['cy']) / intr['fy']
            # Floor 1.53 m below the camera below the horizon, a wall at 6 m (left half) above the floor band.
            floor_z = np.where(v > 1e-3, 1.53 / np.maximum(v, 1e-3), 30.)[:, None] * np.ones((1, 624))
            z = np.where(np.arange(624)[None] < 312, np.minimum(floor_z, 6.), floor_z)
            f = frame(z, np.zeros((468, 624), bool), intrinsics=intr)
            rgb, ranges, si, nr, cosine, _ = R.project_rgbd_min_valid(f, STATE, c)
            out, si2, nr2, info = R.remove_floor_and_ceiling(ranges, si, nr, cosine, STATE, c)
            self.assertGreater(info['ground']['removed_valid_pixels'], 1000)
            kept_wall = ~(si2[0] | nr2[0])[:, :150]
            self.assertGreater(int(kept_wall.sum()), 5000)        # positive control: the wall stays
            self.assertTrue(nr2[0][-20:, 200:].all())             # far-right floor rows -> no return
            del c['estimator_height']
            c['estimator_height'] = None
            with self.assertRaises(R.InputError):                 # no operator assertion -> frame refused
                R.remove_floor_and_ceiling(ranges, si, nr, cosine, STATE, c)

    def test_rejects_unaligned_and_contradicting_coverage(self):
        c = geometry_contract()
        f = frame(np.full((468, 624), 3.), np.zeros((468, 624), bool), intrinsics=full_res_intrinsics(c['camera']))
        with self.assertRaises(R.InputError):
            R.project_rgbd_min_valid(dict(f, aligned=False), STATE, c)
        cov = np.ones((468, 624), bool)
        cov[0, 0] = False
        with self.assertRaises(R.InputError):
            R.project_rgbd_min_valid(dict(f, coverage=cov), STATE, c)
        R.project_rgbd_min_valid(f, STATE, c)                 # positive control


class CheckpointBinding(unittest.TestCase):
    """Review r1 P2-3: the checkpoint's env_state must agree with the receipt (pure function, no torch)."""
    def test_env_state_cross_check(self):
        import camera_policy_contract_cams2 as S
        env = {'NAVRL_YAW_RATE_MAX': '3.0', 'NAVRL_MAX_VELOCITY': '2.5', 'NAVRL_DETECT_WIDTH': '312',
               'NAVRL_DETECT_HEIGHT': '234'}
        state = {'cfg_yaw_rate_max': 3.0, 'cfg_max_velocity': 2.5, 'cfg_detect_width': 312, 'cfg_detect_height': 234}
        pairs = {a: b for a, b, _ in S.CHECKPOINT_ENV_PAIRS}
        for key, env_key, kind in S.CHECKPOINT_ENV_PAIRS:
            if key not in state:
                state[key] = {'float': 1.5, 'int': 7, 'str': 'x'}[kind]
                env[env_key] = {'float': '1.5', 'int': '7', 'str': 'x'}[kind]
        state.update(S.CHECKPOINT_ENV_FIXED)
        self.assertEqual(S.checkpoint_env_problems(state, env), [])                      # positive control
        refused = 0
        for key, value in (('cfg_yaw_rate_max', 1.0472), ('cfg_detect_width', 160), ('cfg_speed_governor_mode', 'off'),
                           ('cfg_perception_perturb', True), ('cfg_detector_checkpoint_name', 'det.pth')):
            with self.subTest(key):
                self.assertNotEqual(S.checkpoint_env_problems(dict(state, **{key: value}), env), [])
                refused += 1
        self.assertEqual(refused, 5)
        self.assertNotEqual(S.checkpoint_env_problems({k: v for k, v in state.items() if k != 'cfg_max_velocity'}, env), [])
        self.assertIn('cfg_yaw_rate_max', pairs)


class NonRuntimeEntryPoints(unittest.TestCase):
    """Review r1 P3-3: the ROS 2 node and the MAVLink companion refuse a CAM-S2 contract by name; oakd_camera's FOV
    check accepts the CAM-S2 trained camera and keeps the CAM-K2 default."""
    def test_node_and_companion_refuse_cams2_clearly(self):
        node = (ROOT / 'tools/camera_ros2_node.py').read_text()
        companion = (ROOT / 'tools/camera_companion.py').read_text()
        self.assertIn("if contract.get('lineage') == 'CAM-S2':", node)
        self.assertIn("if contract.get('lineage') == 'CAM-S2':", companion)
        import camera_companion
        fake = {'lineage': 'CAM-S2', 'variant': 'CAM-S2a'}
        import camera_policy_contract
        saved = camera_policy_contract.load_contract
        camera_policy_contract.load_contract = lambda path: fake
        try:
            with self.assertRaisesRegex(SystemExit, 'CAM-S2 contracts are not supported'):
                camera_companion.main(['--check', 'unused.json'])
        finally:
            camera_policy_contract.load_contract = saved

    def test_oakd_training_fov_check_per_camera(self):
        try:
            import cv2  # noqa: F401
        except ImportError:
            self.skipTest('cv2 not installed')
        import oakd_camera as O
        cam = trained_camera()
        cam.pop('translation_m')
        hfov = math.radians(70.)
        k = [[624 / (2 * math.tan(hfov / 2)), 0, 309.5], [0, 624 / (2 * math.tan(hfov / 2)), 246.7], [0, 0, 1]]
        rect = O.Rectifier(k, [0.] * 5, width=624, height=468)
        rect.require_training_fov(cam)                                        # positive control: covers 66.7x52.6
        narrow = O.Rectifier([[700., 0, 311.5], [0, 700., 233.5], [0, 0, 1]], [0.] * 5, width=624, height=468)
        with self.assertRaisesRegex(O.OakDInputError, '312x234 trained'):
            narrow.require_training_fov(cam)


class TimingBound(unittest.TestCase):
    def test_delay_two_only_for_cams2(self):
        k2 = {'runtime_env': {K.TRAINED_DELAY_ENV: '2'}, 'observation': {'control_dt_s': .1}}
        with self.assertRaises(ValueError):
            K.capture_age_bound_s(k2)
        self.assertEqual(K.capture_age_bound_s(dict(k2, runtime_env={})), .15)   # CAM-K2 unchanged
        s2 = dict(k2, lineage='CAM-S2')
        self.assertEqual(K.capture_age_bound_s(s2), .35)
        with self.assertRaises(ValueError):
            K.capture_age_bound_s(dict(s2, runtime_env={K.TRAINED_DELAY_ENV: '3'}))
        self.assertEqual(K.SUPPORTED_TRAINED_DELAY_STEPS, (0,))


@unittest.skipUnless(HAVE_S2A, 'local CAM-S2a training record/checkpoint not available')
class Cams2Contract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import camera_policy_contract_cams2 as S
        cls.S = S
        cls.contract = S.build_contract(RECEIPT, CHECKPOINT, OBS_NORM, previous_action_delay_steps=1,
                                        ground=dict(K.GROUND_DEFAULT, enabled=False))

    def test_fields_come_from_the_receipt(self):
        c, env = self.contract, json.loads(RECEIPT.read_text())['env_navrl']
        self.assertEqual(c['variant'], 'CAM-S2a')
        self.assertEqual((c['camera']['width'], c['camera']['height']), (312, 234))
        self.assertEqual((c['camera']['hfov_deg'], c['camera']['vfov_deg']), (66.7, 52.6))
        self.assertEqual(c['camera']['translation_m'], [.08, 0., .03])
        self.assertEqual(c['observation']['flight_altitude_m'], 1.5)
        self.assertEqual((c['depth']['minz_m'], c['depth']['maxz_m'], c['depth']['validity']), (.315, 20., 'none'))
        self.assertEqual((c['action']['yaw_rate_max_rps'], c['action']['max_velocity_mps']), (3.0, 2.5))
        self.assertEqual(c['action']['trained_delay_steps'], [1, 2])
        self.assertEqual(c['action']['blind_cap_mps'], 1.0)
        self.assertEqual(K.capture_age_bound_s(c), .35)
        self.assertEqual(c['checkpoint']['sha256'][:8], '9fc9dfb7')
        # Training-only keys are gone, the rest of the training environment is kept verbatim.
        leaked = [k for k in c['runtime_env'] if k.startswith(('NAVRL_P9_', 'NAVRL_DEPTH_', 'NAVRL_VEH_RAND'))]
        self.assertEqual(leaked, [])
        self.assertTrue(any(k.startswith('NAVRL_P9_') for k in env))          # non-vacuous: training had P9
        kept = {k: v for k, v in env.items() if k in c['runtime_env']}
        self.assertEqual(kept, c['runtime_env'])
        self.assertGreater(len(kept), 100)
        self.assertEqual(K.validate_contract(c), [])                           # dispatch from the CAM-K2 module

    def test_refusals_each_with_the_accepted_contract_as_control(self):
        c = self.contract
        cases = {
            'p9_in_flight_env': lambda d: d['runtime_env'].update(NAVRL_P9_ERROR_MODEL='/x.json'),
            'altitude_edit': lambda d: d['observation'].update(flight_altitude_m=1.0),
            'camera_edit': lambda d: d['camera'].update(width=160),
            'yaw_edit': lambda d: d['action'].update(yaw_rate_max_rps=1.0472),
            'k_hat_zero': lambda d: d['action'].update(previous_action_delay_steps=0),
            'k_hat_three': lambda d: d['action'].update(previous_action_delay_steps=3),
            'outdoor': lambda d: d.update(K.outdoor_blocks()),
            'plane_rule': lambda d: d['ground'].update(enabled=True, invalid_plane_rule=True),
            'source_set': lambda d: d['sources'].pop('tools/camera_policy_runtime.py'),
        }
        refused = 0
        for name, edit in cases.items():
            with self.subTest(name):
                d = json.loads(json.dumps(c))
                edit(d)
                self.assertNotEqual(K.validate_contract(d), [])
                refused += 1
        self.assertEqual(refused, len(cases))
        self.assertEqual(K.validate_contract(json.loads(json.dumps(c))), [])
        with self.assertRaises(ValueError):
            self.S.build_contract(RECEIPT, CHECKPOINT, OBS_NORM, previous_action_delay_steps=None)
        k2 = self.S.build_contract(RECEIPT, CHECKPOINT, OBS_NORM, previous_action_delay_steps=2,
                                   ground=dict(K.GROUND_DEFAULT, enabled=False))
        self.assertEqual(k2['action']['previous_action_delay_steps'], 2)

    def test_receipt_variants_and_forbidden_training_settings(self):
        receipt = json.loads(RECEIPT.read_text())
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            listing = (RECEIPT.parent / 'checkpoint_sha256_all.txt').read_text()

            def build(env_edit):
                d = json.loads(json.dumps(receipt))
                env_edit(d['env_navrl'])
                run = tmp / ('run%d' % len(list(tmp.iterdir())))
                run.mkdir()
                (run / 'receipt.json').write_text(json.dumps(d))
                (run / 'checkpoint_sha256_all.txt').write_text(listing)
                return self.S.build_contract(run / 'receipt.json', CHECKPOINT, OBS_NORM, previous_action_delay_steps=1,
                                             ground=dict(K.GROUND_DEFAULT, enabled=False))
            s2b = build(lambda e: e.update(NAVRL_YAW_RATE_MAX='1.0472'))
            self.assertEqual((s2b['variant'], s2b['action']['yaw_rate_max_rps']), ('CAM-S2b', 1.0472))
            for name, edit in {'blob_near': lambda e: e.update(NAVRL_DEPTH_VALIDITY='blob_near'),
                               'yawclip': lambda e: e.update(NAVRL_YAWCLIP='1'),
                               'unknown_yaw': lambda e: e.update(NAVRL_YAW_RATE_MAX='2.0'),
                               'delay_record': lambda e: e.update(NAVRL_ACTION_DELAY_STEPS_MAX='1'),
                               'k2_resolution': lambda e: e.update(NAVRL_CAMERA_WIDTH='160')}.items():
                with self.subTest(name), self.assertRaises(ValueError):
                    build(edit)
            run = tmp / 'unlisted'
            run.mkdir()
            shutil.copy(RECEIPT, run / 'receipt.json')
            with self.assertRaises(ValueError):                 # checkpoint not listed by the receipt's run
                self.S.build_contract(run / 'receipt.json', CHECKPOINT, OBS_NORM, previous_action_delay_steps=1)


PARITY = r'''
import json, math, sys
import numpy as np
sys.path.insert(0, sys.argv[1])
import camera_policy_contract as K, camera_policy_contract_cams2 as S
from camera_policy_runtime import CameraPolicyRuntime, InputError
receipt, checkpoint, obs_norm = sys.argv[2:5]
K_STEPS = int(sys.argv[5])
MUTANT = sys.argv[6] if len(sys.argv) > 6 else ''
c = S.build_contract(receipt, checkpoint, obs_norm, previous_action_delay_steps=K_STEPS,
                     ground=dict(K.GROUND_DEFAULT, enabled=False))
r = CameraPolicyRuntime(c)
import torch, importlib, types
# Review r1 P2-1: force lateral (out-of-FOV) actions on some steps so the blind cap acts on the published command and,
# k steps later, on the delayed previous-action channel. Both paths use the same (forced) action.
FORCE = {4: (.1, .95), 5: (-.2, -.9), 6: (.05, .9), 10: (.0, -1.)}
step_no = [0]
policy_actions = r.audit.deterministic_actions
def forced_actions(player, obs):
    a = policy_actions(player, obs).reshape(1, 4).clone()
    if step_no[0] in FORCE:
        a[0, 0], a[0, 1] = FORCE[step_no[0]]
    return a
r.audit.deterministic_actions = forced_actions
if MUTANT == 'no_prev_cap':
    # The mutant the review found surviving: the delayed command governed but NOT blind-capped.
    def mutant_command(self, a, governed, perception=None):
        torch_, c2 = self.torch, self.cams2
        perception = self.perception if perception is None else perception
        vmax_, half = float(self.tc.max_velocity), float(perception.hfov) * .5
        published, blind = c2['camsafe'].cap_blind(governed, half, c2['blind_mps'])
        history = torch_.roll(self._action_history, 1, dims=0)
        history[0] = a[0]
        delayed = c2['cams2'].delayed(history.unsqueeze(0), torch_.tensor([c2['k_hat']]))
        d = self._govern(delayed[:, :2].clamp(-1., 1.) * vmax_, perception)
        return published, torch_.cat([(d / vmax_).clamp(-1., 1.), a[:, 2:]], 1), history, blind
    r._cams2_command = types.MethodType(mutant_command, r)
pm_mod = importlib.import_module('aerial_gym.task.navrl_task.navrl_perception')
dm = importlib.import_module('aerial_gym.task.navrl_task.navrl_depth_model')
cams2 = importlib.import_module('aerial_gym.task.navrl_task.navrl_cams2')
camsafe = importlib.import_module('aerial_gym.task.navrl_task.navrl_camsafe')
tc = r.tc
assert tc.flight_altitude == 1.5 and list(tc.vision.camera_translation) == [.08, 0., .03]
# Training observation path: a second perception from the same (ablation-installed) module and configuration; the
# OAK-D depth model's deterministic part (noise and holes off; MinZ/MaxZ from the receipt) applied by its own apply().
train = pm_mod.NavRLPerceptionModule(num_envs=1, device=torch.device('cpu'), cfg=tc.perception, step_dt=.1,
                                     camera_cfg=tc.vision)
# Review r1 P3-2: negative control by code: the same training path with the CAM-K2 altitude 1.0 m.
train_alt = pm_mod.NavRLPerceptionModule(num_envs=1, device=torch.device('cpu'), cfg=tc.perception, step_dt=.1,
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
BARS = [(3.0, 0.8), (5.0, -1.2), (2.2, -0.4), (7.5, 1.5), (0.3, 0.16), (4.0, 2.6)]   # (x, y) radius 0.1, z in [0, 3]
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

vmax = float(tc.max_velocity)
hist = torch.zeros((1, int(c['action']['trained_delay_steps'][1]) + 1, 4))
prev_train = torch.zeros((1, 4))
prev_k2 = torch.zeros((1, 4))
r.reset()
with torch.inference_mode():
    train.reset_idx(torch.tensor([0]))
    train_alt.reset_idx(torch.tensor([0]))
train.step_count = train_alt.step_count = 0
prev_mismatch, alt_diff_code = 0, 0.
results, max_diff, k2_diff, alt_diff = [], 0., 0., 0.
minz_pixels, nohit_pixels, blind_seen = 0, 0, 0
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
    # ---- training path ----
    rgb_tr = torch.from_numpy(colour.transpose(2, 0, 1)[None].astype(np.float32) / 255.)
    with torch.inference_mode():
        depth_tr = dm.apply(train, torch.from_numpy(rng32[None]), dms)
        st = dict(rgb=rgb_tr, depth=depth_tr, lidar_m=torch.full((1, 288), float(tc.lidar_max_range)),
                  drone_pos_w=torch.tensor([pos], dtype=torch.float32), drone_vel_w=torch.tensor([vel], dtype=torch.float32),
                  vehicle_quat=torch.tensor([quat_yaw(yaw)], dtype=torch.float32),
                  yaw_rate=torch.tensor([rate / float(tc.yaw_rate_max)]), max_velocity=tc.max_velocity,
                  flight_altitude=tc.flight_altitude, training=False)
        obs_tr, _ = train.observe(previous_action=prev_train, **st)
        obs_alt, _ = train_alt.observe(previous_action=prev_train, **dict(st, flight_altitude=1.0,
                                       depth=dm.apply(train_alt, torch.from_numpy(rng32[None]), dms)))
    minz_pixels += int((torch.from_numpy(rng32 * cos.astype(np.float32)) < dms['NAVRL_DEPTH_MINZ_M']).sum())
    nohit_pixels += int((~hit).sum())
    # ---- runtime path: 624x468 measured frame, 2x2 sub-samples (one at the trained depth, the others farther or
    # flagged invalid); invalid where training is invalid or saw nothing (OAK-D reports 0) ----
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
    step_no[0] = i
    out = r.step(frame, state, now_s=t_s + .1)              # 0.1 s capture age: one step, inside the trained support
    obs_rt = np.asarray(out['observation'])
    alt_diff_code = max(alt_diff_code, float(np.abs(obs_rt - obs_alt[0].numpy()).max()))
    assert out['latency']['implied_delay_steps'] == 1 and not out['latency']['above_trained_max']
    diff = float(np.abs(obs_rt - obs_tr[0].numpy()).max())
    max_diff = max(max_diff, diff)
    blind_seen += int(out['blind_capped'])
    a = torch.tensor([out['actor_output']], dtype=torch.float32).clamp(-1., 1.)
    # ---- training previous-action rule (navrl_task.step + transform_action_to_command, navrl_cams2, navrl_camsafe) ----
    hist = torch.roll(hist, 1, dims=1)
    hist[:, 0] = a
    applied = cams2.delayed(hist, torch.tensor([K_STEPS]))
    vel_vehicle = torch.clamp(applied[:, 0:3], -1., 1.) * vmax
    governed = r._govern(vel_vehicle[:, 0:2].clone(), perception=train)
    capped, _ = camsafe.cap_blind(governed, float(train.hfov) * .5, 1.0)
    prev_train = torch.clamp(a[:, 0:4], -1., 1.).clone()
    prev_train[:, 0:2] = (capped / max(1e-6, vmax)).clamp(-1., 1.)
    # negative control: the CAM-K2 rule (current undelayed governed xy) differs from what the runtime fed
    k2_prev = torch.cat([(r._govern(a[:, :2] * vmax, perception=train) / vmax).clamp(-1, 1), a[:, 2:]], 1)
    k2_diff = max(k2_diff, float((k2_prev - torch.tensor([out['previous_action']])).abs().max()))
    prev_mismatch += int(not np.allclose(out['previous_action'], prev_train[0].numpy(), atol=1e-6))
    results.append(dict(step=i, max_abs_diff=diff, visible=out['camera_visible'], near=out['near_constant_slots']))
if MUTANT:
    print('CAMS2_PARITY ' + json.dumps(dict(k=K_STEPS, mutant=MUTANT, prev_mismatch_steps=prev_mismatch,
          max_abs_diff=max_diff, blind_capped_steps=blind_seen)))
    sys.exit(0)
# Review r1 P3-1: structural slots are fed exactly; a perturbed one is caught
structural = r.structural['index']
assert len(r.structural_violations(obs_rt)) == 0
bad_obs = obs_rt.copy()
bad_obs[structural[0]] += 1e-3
assert list(r.structural_violations(bad_obs)) == [structural[0]]
# near-constant slots: exactly-constant trained slots are fed exactly; a perturbed slot is reported
nc = r.near_constant
exact = nc['index'][(np.abs(nc['mean'] - np.round(nc['mean'])) < 1e-9) & (nc['scale'] < 3.2e-3 + 1e-9)]
assert len(exact) > 100, len(exact)
assert np.array_equal(obs_rt[exact], np.round(r.near_constant['mean'][np.isin(nc['index'], exact)])), 'exact slots'
bad = obs_rt.copy()
bad[exact[0]] += .5
assert r.near_constant_report(bad)['outside'] >= 1 and r.near_constant_report(obs_rt)['outside'] == results[-1]['near']['outside']
def refused(age):
    r.reset()                                                 # isolates the age check from the 10 Hz cadence check
    try:
        r.step(dict(frame, rgb_t=t_s + .1, depth_t=t_s + .1), dict(state, t=t_s + .1), now_s=t_s + .1 + age)
    except InputError as exc:
        return str(exc)
    return None
stale_refused = refused(.36) is not None                     # beyond the 0.35 s capture-age bound
latency_message = refused(.26)                               # implies 3 steps > trained maximum 2 (review r1 P1-2)
latency_refused = latency_message is not None and 'above the trained maximum' in latency_message
latency_accepted = refused(.24) is None                      # positive control: 2 steps, accepted
print('CAMS2_PARITY ' + json.dumps(dict(k=K_STEPS, steps=len(results), max_abs_diff=max_diff,
      visible_steps=sum(x['visible'] for x in results), k2_prev_diff=k2_diff, alt_diff=alt_diff_code,
      prev_mismatch_steps=prev_mismatch, structural_slots=int(len(structural)), latency_refused=latency_refused,
      latency_accepted=latency_accepted,
      exact_slots=int(len(exact)), near_outside_last=results[-1]['near']['outside'],
      stale_refused=stale_refused, blind_capped_steps=blind_seen, minz_pixels=minz_pixels,
      nohit_pixels=nohit_pixels)))
'''


@unittest.skipUnless(HAVE_S2A, 'local CAM-S2a training record/checkpoint not available')
class TrainingRuntimeParity(unittest.TestCase):
    def run_parity(self, k, mutant=''):
        env = {key: v for key, v in os.environ.items() if not key.startswith('NAVRL_')}
        env.update(CUDA_VISIBLE_DEVICES='', PYTHONNOUSERSITE='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1',
                   OPENBLAS_NUM_THREADS='1')
        result = subprocess.run([sys.executable, '-B', '-c', PARITY, str(ROOT / 'tools'), str(RECEIPT),
                                 str(CHECKPOINT), str(OBS_NORM), str(k)] + ([mutant] if mutant else []),
                                env=env, capture_output=True, text=True, timeout=600, cwd=str(ROOT))
        self.assertEqual(result.returncode, 0, result.stdout[-3000:] + result.stderr[-4000:])
        line = [x for x in result.stdout.splitlines() if x.startswith('CAMS2_PARITY ')]
        self.assertEqual(len(line), 1, result.stdout[-2000:])
        return json.loads(line[0].split(' ', 1)[1])

    def test_observation_parity_for_both_trained_delays(self):
        for k in (1, 2):
            with self.subTest(k=k):
                rep = self.run_parity(k)
                self.assertEqual(rep['steps'], 14)
                self.assertLess(rep['max_abs_diff'], 1e-5)
                self.assertGreater(rep['visible_steps'], 0)        # the target is detected in both paths
                self.assertGreater(rep['k2_prev_diff'], 1e-3)      # negative control: the CAM-K2 rule differs
                self.assertGreater(rep['alt_diff'], .1)            # negative control: the 1.0 m normaliser differs
                self.assertGreater(rep["exact_slots"], 100)
                self.assertTrue(rep['stale_refused'])              # 0.36 s > 0.35 s bound
                self.assertTrue(rep['latency_refused'])            # 0.26 s: 3 implied steps > 2 (P1-2)
                self.assertTrue(rep['latency_accepted'])           # positive control: 0.24 s
                self.assertEqual(rep['prev_mismatch_steps'], 0)
                self.assertGreater(rep['blind_capped_steps'], 0)   # the blind cap acted (P2-1)
                self.assertGreaterEqual(rep['structural_slots'], 400)
                self.assertGreater(rep['minz_pixels'], 0)          # the MinZ (invalid = no return) case occurs
                self.assertGreater(rep['nohit_pixels'], 0)         # sky: training 10 m no-hit vs runtime invalid

    def test_mutant_without_the_previous_action_blind_cap_is_killed(self):
        rep = self.run_parity(1, 'no_prev_cap')
        self.assertGreater(rep['blind_capped_steps'], 0)
        self.assertGreater(rep['prev_mismatch_steps'], 0)
        self.assertGreater(rep['max_abs_diff'], 1e-3)


INTEGRATION = r'''
import json, math, sys
import numpy as np
sys.path.insert(0, sys.argv[1])
import camera_policy_contract as K, camera_policy_contract_cams2 as S
from camera_policy_runtime import CameraPolicyRuntime
c = S.build_contract(sys.argv[2], sys.argv[3], sys.argv[4], previous_action_delay_steps=1,
                     ground=dict(K.GROUND_DEFAULT, enabled=False))
r = CameraPolicyRuntime(c)
import torch
cam = c['camera']
f = dict(rgb=np.full((468, 624, 3), 128, np.uint8), depth_z_m=np.full((468, 624), 4., np.float32),
         invalid=np.zeros((468, 624), bool), aligned=True, rectified=True, mount_flu_m=[.08, 0., .03],
         intrinsics={'width': 624, 'height': 468, 'fx': 2 * cam['fx'], 'fy': 2 * cam['fy'], 'cx': 309.5, 'cy': 246.7},
         camera_to_body_rotation=np.eye(3), rgb_t=1., depth_t=1.)
f['invalid'][:20] = True
s = dict(pos=[0., 0., 1.5], vel=[0., 0., 0.], quat_xyzw=[0., 0., 0., 1.], yaw_rate_rps=0., t=1.)
out = r.step(f, s, now_s=1.)
assert out['lineage'] == 'CAM-S2' and len(out['observation']) == 898 and out['near_pixels'] == 0
assert out['rgb_blanked_pixels'] == 0 and not out['command_sent']
# vehicle interface: level-body proposal plus the capture yaw; the gateway rotates once with the latest yaw
assert out['proposal_frame'] == 'level_body_flu' and out['capture_yaw_rad'] == 0.
assert out['latency']['implied_delay_steps'] == 0 and len(r.structural['index']) >= 400
# the height feature uses 1.5 m (robot history: 5 x [vel 3, yaw rate, previous action 4, height, 1] from slot 768)
heights = [out['observation'][768 + 10 * h + 8] for h in range(5)]
# newest entry (h = 4, the trained near-constant slot 817 holds its constant 1); older entries are the reset zeros
assert abs(heights[4] - 1.5 / 4.5) < 1e-6 and out['observation'][817] == 1., heights
# blind cap on the published command: pure lateral 2.5 m/s -> 1.0 m/s, direction kept; forward is untouched (control)
with torch.inference_mode():
    lateral = torch.tensor([[0., 1., 0., 0.]])
    pub, prev, hist, blind = r._cams2_command(lateral, torch.tensor([[0., 2.5]]))
    assert bool(blind[0]) and abs(float(pub.norm()) - 1.0) < 1e-6 and float(pub[0, 0]) == 0. and float(pub[0, 1]) > 0
    fwd, _, _, blind_f = r._cams2_command(torch.tensor([[1., 0., 0., 0.]]), torch.tensor([[2.5, 0.]]))
    assert not bool(blind_f[0]) and abs(float(fwd[0, 0]) - 2.5) < 1e-6
    # previous action after a reset: the delayed slot is the zero command, z/yaw the current action
    r.reset()
    _, prev0, _, _ = r._cams2_command(torch.tensor([[.5, .5, .3, -.7]]), torch.tensor([[1.2, 1.2]]))
    assert prev0[0, :2].abs().max() == 0. and abs(float(prev0[0, 2]) - .3) < 1e-7 and abs(float(prev0[0, 3]) + .7) < 1e-7
# CAM-S2 applied zero hold: newest slot and z/yaw zeroed, the delayed xy kept
r.reset()
out1 = r.step(dict(f, rgb_t=2., depth_t=2.), dict(s, t=2.), now_s=2.)
r.applied_zero_hold()
assert r._action_history[0].abs().max() == 0. and r.prev[0, 2:].abs().max() == 0.
print('CAMS2_INTEGRATION ' + json.dumps(dict(variant=out['variant'], capture_bound=r.capture_age_bound_s,
      sensor_invalid=out['sensor_invalid_pixels'], no_return=out['no_return_pixels'],
      near_constant=out['near_constant_slots'])))
'''


@unittest.skipUnless(HAVE_S2A, 'local CAM-S2a training record/checkpoint not available')
class RuntimeIntegration(unittest.TestCase):
    def test_real_stream_geometry_blind_cap_and_hold(self):
        env = {key: v for key, v in os.environ.items() if not key.startswith('NAVRL_')}
        env.update(CUDA_VISIBLE_DEVICES='', PYTHONNOUSERSITE='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
        result = subprocess.run([sys.executable, '-B', '-c', INTEGRATION, str(ROOT / 'tools'), str(RECEIPT),
                                 str(CHECKPOINT), str(OBS_NORM)],
                                env=env, capture_output=True, text=True, timeout=300, cwd=str(ROOT))
        self.assertEqual(result.returncode, 0, result.stdout[-3000:] + result.stderr[-4000:])
        line = [x for x in result.stdout.splitlines() if x.startswith('CAMS2_INTEGRATION ')]
        rep = json.loads(line[0].split(' ', 1)[1])
        self.assertEqual(rep['variant'], 'CAM-S2a')
        self.assertEqual(rep['capture_bound'], .35)
        self.assertGreater(rep['sensor_invalid'], 0)          # the invalid top band reached the trained image ...
        self.assertGreater(rep['no_return'] + rep['sensor_invalid'], 0)



def _quat(roll, pitch, yaw):
    cr, sr, cp, sp = math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]


class LatencyPortEquivalence(unittest.TestCase):
    """2026-11-26 latency port: project_rgbd_min_valid_v1_fast must equal project_rgbd_min_valid_reference (the committed v1
    arithmetic) array for array, bit for bit, including refusals and ties (quantised depths)."""

    @staticmethod
    def _run(fn, f, s, c):
        try:
            return 'ok', fn(f, s, c)
        except R.InputError as e:
            return 'refused', str(e)

    def _assert_same(self, a, b):
        self.assertEqual(a[0], b[0])
        if a[0] == 'refused':
            self.assertEqual(a[1], b[1])
            return
        for x, y in zip(a[1][:5], b[1][:5]):
            x, y = np.asarray(x), np.asarray(y)
            self.assertEqual((x.shape, x.dtype), (y.shape, y.dtype))
            self.assertTrue(np.array_equal(x, y))
        self.assertEqual(a[1][5], b[1][5])

    def test_random_scenes_attitudes_coverage_and_refusals(self):
        c = geometry_contract()
        cam = c['camera']
        rng = np.random.default_rng(20261126)
        kinds = {'ok': 0, 'refused': 0, 'ties': 0, 'tilted_ok': 0}
        for i in range(36):
            h, w = 468, 624
            if i % 3 == 0:                                    # quantised depths: many equal z per trained pixel
                z = rng.choice([.2, .5, 1., 2., 3.3, 5., 9.9, 25.], size=(h, w))
                kinds['ties'] += 1
            else:
                z = rng.uniform(0., 25., (h, w))
            z[rng.random((h, w)) < .05] = np.nan
            f = frame(z, rng.random((h, w)) < rng.uniform(0, .5), rng.integers(0, 256, (h, w, 3), dtype=np.uint8),
                      full_res_intrinsics(cam, cy=246.7 if i % 2 else None), mount=(.08, 0., .03) if i % 2 else (.08, 0., .03))
            if i % 4 == 1:
                cov = np.ones((h, w), bool)
                cov[:int(rng.integers(1, 40))] = False
                f['coverage'] = cov
                f['depth_z_m'] = np.where(cov, f['depth_z_m'], np.nan).astype(np.float32)
            if i % 6 == 2:                                     # a source at the trained resolution: tilted -> refused
                f = dict(f, intrinsics={'width': 312, 'height': 234, 'fx': cam['fx'], 'fy': cam['fy'],
                                        'cx': cam['cx'], 'cy': cam['cy']},
                         depth_z_m=f['depth_z_m'][::2, ::2].copy(), invalid=f['invalid'][::2, ::2].copy(),
                         rgb=f['rgb'][::2, ::2].copy())
                f.pop('coverage', None)
            roll, pitch = (0., 0.) if i % 7 == 0 else (rng.normal(0, .15), rng.normal(0, .15))
            s = dict(STATE, quat_xyzw=_quat(roll, pitch, rng.uniform(-3, 3)))
            a = self._run(R.project_rgbd_min_valid_reference, f, s, c)
            b = self._run(R.project_rgbd_min_valid_v1_fast, f, s, c)
            self._assert_same(a, b)
            kinds[a[0]] += 1
            kinds['tilted_ok'] += a[0] == 'ok' and (roll, pitch) != (0., 0.)
        # Non-vacuous: accepted tilted frames, refusals and tie-heavy frames all occurred.
        self.assertGreater(kinds['tilted_ok'], 10)
        self.assertGreater(kinds['refused'], 0)
        self.assertGreater(kinds['ties'], 5)

    def test_first_min_per_pixel_equals_the_lexsort_rule(self):
        rng = np.random.default_rng(7)
        for n, size, levels in ((0, 5, 3), (1, 5, 3), (5000, 300, 4), (20000, 50, 2)):
            idx = rng.integers(0, size, n)
            keys = rng.integers(0, levels, n).astype(np.float64)      # few levels: ties everywhere
            pixels, pos = R._first_min_per_pixel(idx, keys, size)
            order = np.lexsort((keys, idx))
            first = np.r_[True, idx[order][1:] != idx[order][:-1]] if n else np.zeros(0, bool)
            self.assertEqual(pixels.tolist(), idx[order][first].tolist())
            self.assertEqual(pos.tolist(), order[first].tolist())

    def test_mutant_with_last_tie_differs(self):
        """Negative control: picking the LAST of equal keys changes colours on a tie-heavy frame."""
        c = geometry_contract()
        cam = c['camera']
        rng = np.random.default_rng(11)
        z = np.full((468, 624), 3.0)
        f = frame(z, np.zeros((468, 624), bool), rng.integers(0, 256, (468, 624, 3), dtype=np.uint8),
                  full_res_intrinsics(cam), mount=(0., 0., 0.))
        f['mount_flu_m'] = [.08, 0., .03]
        good = R.project_rgbd_min_valid_v1_fast(f, STATE, c)
        original = R._first_min_per_pixel

        def last_tie(indices, keys, size):
            rev = np.arange(len(indices))[::-1]
            pixels, pos = original(indices[rev], keys[rev], size)
            return pixels, rev[pos]
        R._first_min_per_pixel = last_tie
        try:
            bad = R.project_rgbd_min_valid_v1_fast(f, STATE, c)
        finally:
            R._first_min_per_pixel = original
        self.assertTrue(np.array_equal(good[1], bad[1]))
        self.assertFalse(np.array_equal(good[0], bad[0]))


class ThreadPinning(unittest.TestCase):
    class FakeTorch:
        def __init__(self, interop_ok=True):
            self.n, self.i, self.ok = 4, 4, interop_ok

        def set_num_threads(self, n):
            self.n = n

        def set_num_interop_threads(self, n):
            if not self.ok:
                raise RuntimeError('too late')
            self.i = n

        def get_num_threads(self):
            return self.n

        def get_num_interop_threads(self):
            return self.i

    def test_pins_when_asked_and_leaves_alone_otherwise(self):
        t = self.FakeTorch()
        r = R.pin_cpu_threads(t, 1)
        self.assertEqual((r['torch_num_threads'], r['torch_num_interop_threads'], r['interop_pinned']), (1, 1, True))
        t = self.FakeTorch()
        r = R.pin_cpu_threads(t, None)
        self.assertEqual((r['torch_num_threads'], r['torch_num_interop_threads']), (4, 4))
        t = self.FakeTorch(interop_ok=False)
        r = R.pin_cpu_threads(t, 1)
        self.assertEqual((r['torch_num_threads'], r['interop_pinned']), (1, False))
        for bad in (0, -1, 1.0, '1'):
            with self.assertRaises(ValueError):
                R.pin_cpu_threads(self.FakeTorch(), bad)

    def test_ros2_node_pins_one_thread(self):
        text = (ROOT / 'tools' / 'camera_ros2_node.py').read_text()
        self.assertIn('CameraPolicyRuntime(contract, torch_threads=1)', text)


def _random_frames(seed, count, cam):
    """Random CAM-S2 frames and states: tilted and level, coverage masks, tie-heavy depths, trained-resolution sources
    (refused when tilted)."""
    rng = np.random.default_rng(seed)
    out = []
    for i in range(count):
        h, w = 468, 624
        z = (rng.choice([.2, .5, 1., 2., 3.3, 5., 9.9, 25.], size=(h, w)) if i % 3 == 0
             else rng.uniform(0., 25., (h, w)))
        z[rng.random((h, w)) < .05] = np.nan
        f = frame(z, rng.random((h, w)) < rng.uniform(0, .5), rng.integers(0, 256, (h, w, 3), dtype=np.uint8),
                  full_res_intrinsics(cam, cy=246.7 if i % 2 else None))
        if i % 4 == 1:
            cov = np.ones((h, w), bool)
            cov[:int(rng.integers(1, 40))] = False
            f['coverage'] = cov
            f['depth_z_m'] = np.where(cov, f['depth_z_m'], np.nan).astype(np.float32)
        if i % 6 == 2:
            f = dict(f, intrinsics={'width': 312, 'height': 234, 'fx': cam['fx'], 'fy': cam['fy'], 'cx': cam['cx'],
                                    'cy': cam['cy']}, depth_z_m=f['depth_z_m'][::2, ::2].copy(),
                     invalid=f['invalid'][::2, ::2].copy(), rgb=f['rgb'][::2, ::2].copy())
            f.pop('coverage', None)
        roll, pitch = (0., 0.) if i % 7 == 0 else (rng.normal(0, .15), rng.normal(0, .15))
        out.append((f, dict(STATE, quat_xyzw=_quat(roll, pitch, rng.uniform(-3, 3)))))
    return out


class CanonicalV2(unittest.TestCase):
    """Runtime v2 (fixed-order float64, no matrix product) against v1: the same semantics; on any machine they may differ
    only where a sample lies within rounding of a trained-pixel border, so a frame may differ in at most a few pixels
    (on the maintainer's x86 workstation: none, 2026-11-26)."""

    def test_v2_matches_v1_up_to_border_rounding(self):
        c = geometry_contract()
        n_ok = n_px = 0
        for f, s in _random_frames(5, 24, c['camera']):
            a = LatencyPortEquivalence._run(R.project_rgbd_min_valid_reference, f, s, c)
            b = LatencyPortEquivalence._run(R.project_rgbd_min_valid, f, s, c)
            self.assertEqual(a[0], b[0])
            if a[0] == 'refused':
                continue
            n_ok += 1
            differ = np.zeros(a[1][1].shape[1:], bool)
            for x, y in zip(a[1][:4], b[1][:4]):
                x, y = np.asarray(x)[0], np.asarray(y)[0]
                differ |= (x != y).any(axis=0) if x.ndim == 3 else (x != y)
            n_px = max(n_px, int(differ.sum()))
        self.assertGreater(n_ok, 10)
        self.assertLessEqual(n_px, 4)

    def test_rows_and_cols_follow_the_fixed_order(self):
        R3 = [[.1, .2, .3], [.4, .5, .6], [.7, .8, .9]]
        a, b, c = np.array([1.5]), np.array([-2.25]), np.array([3.125])
        rows = R._v2_rows(R3, a, b, c, [1., 2., 3.])
        for i in range(3):
            self.assertEqual(rows[i][0], ((R3[i][0]*1.5 + R3[i][1]*-2.25) + R3[i][2]*3.125) + (i + 1.))
        cols = R._v2_cols(R3, a, b, c)
        for j in range(3):
            self.assertEqual(cols[j][0], (1.5*R3[0][j] + -2.25*R3[1][j]) + 3.125*R3[2][j])


class CudaV2Equivalence(unittest.TestCase):
    """GPU_REQUIRED (tools/run_tests.py): CudaProjector equals the CPU v2 projection bit for bit, refusals included."""

    @classmethod
    def setUpClass(cls):
        import torch
        if not torch.cuda.is_available():
            raise RuntimeError('CudaV2Equivalence needs a CUDA device (GPU_REQUIRED profile)')
        cls.cuda = R.CudaProjector(torch)

    def test_bit_identical_to_cpu_v2(self):
        c = geometry_contract()
        kinds = {'ok': 0, 'refused': 0}
        for f, s in _random_frames(9, 24, c['camera']):
            a = LatencyPortEquivalence._run(R.project_rgbd_min_valid, f, s, c)
            b = LatencyPortEquivalence._run(self.cuda, f, s, c)
            if a[0] == 'ok':                                   # the report names the device; everything else equal
                self.assertEqual((a[1][5]['device'], b[1][5]['device']), ('cpu', 'cuda'))
                a = ('ok', a[1][:5] + (dict(a[1][5], device=None),))
                b = ('ok', b[1][:5] + (dict(b[1][5], device=None),))
            LatencyPortEquivalence._assert_same(self, a, b)
            kinds[a[0]] += 1
        self.assertGreater(kinds['ok'], 10)
        self.assertGreater(kinds['refused'], 0)

    def test_warm_up_returns_call_times(self):
        c = geometry_contract()
        f, s = _random_frames(1, 1, c['camera'])[0]
        times = self.cuda.warm_up(f, s, c, repeats=2)
        self.assertEqual(len(times), 2)
        self.assertTrue(all(t > 0 for t in times))

    def _intermediates(self, cuda):
        """Review r1 P2-1: the v2 row and splat arithmetic, bitwise, on 200k random points (CPU numpy vs `cuda`)."""
        import torch
        rng = np.random.default_rng(31)
        n = 200000
        q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
        Rm = [[float(q[i, j]) for j in range(3)] for i in range(3)]
        off = [float(x) for x in rng.normal(0, .1, 3)]
        a, b, c = rng.uniform(.3, 12, n), rng.normal(0, 3, n), rng.normal(0, 3, n)
        dst = (312, 234, 237.04, 236.73, 155.5, 116.5)
        rows_np = R._v2_rows(Rm, a, b, c, off)
        g = [torch.from_numpy(x).cuda() for x in (a, b, c)]
        rows_t = R._v2_rows(Rm, g[0], g[1], g[2], off)
        same_rows = all(np.array_equal(x, y.cpu().numpy()) for x, y in zip(rows_np, rows_t))
        sp_np = R._v2_splat_np(*rows_np, dst, offset=True)
        sp_t = cuda._splat(*rows_t, dst, offset=True)
        same_splat = all(np.array_equal(x, y.cpu().numpy()) for x, y in zip(sp_np, sp_t))
        return same_rows, same_splat, len(sp_np[0])

    def test_intermediate_arithmetic_is_bitwise_identical(self):
        same_rows, same_splat, kept = self._intermediates(self.cuda)
        self.assertTrue(same_rows)
        self.assertTrue(same_splat)
        self.assertGreater(kept, 1000)

    def test_reciprocal_division_mutant_is_caught(self):
        """Negative control: a GPU splat that divides via a reciprocal (as torch does for tensor / scalar) differs."""
        import torch

        class Mutant(R.CudaProjector):
            def _splat(self, d0, d1, d2, dst, offset=False):
                tw, th, tfx, tfy, tcx, tcy = dst
                sel = torch.nonzero(d0 > 1e-6).squeeze(1)
                inv = 1. / d0[sel]
                xf = -d1[sel]*inv*tfx + tcx
                yf = -d2[sel]*inv*tfy + tcy
                xx, yy = torch.floor(xf + .5), torch.floor(yf + .5)
                keep = (xx >= 0) & (xx < tw) & (yy >= 0) & (yy < th)
                xi, yi = xx[keep].to(torch.int64), yy[keep].to(torch.int64)
                if offset:
                    dx, dy = xf[keep] - xx[keep], yf[keep] - yy[keep]
                    return sel[keep], yi*tw + xi, dx*dx + dy*dy
                return sel[keep], yi*tw + xi
        _, same_splat, _ = self._intermediates(Mutant(torch))
        self.assertFalse(same_splat)



class ProjectionReadiness(unittest.TestCase):
    """Vehicle-side request 2026-11-26: a CUDA runtime refuses steps until warmed up; a slow first call is flagged."""

    class FakeCuda(R.CudaProjector):
        def __init__(self, times):                       # no device: the readiness logic only
            self.times = times

        def warm_up(self, frame, state, contract, repeats=3):
            return self.times[:repeats]

    def _runtime(self, times):
        rt = R.CameraPolicyRuntime.__new__(R.CameraPolicyRuntime)
        rt.failed, rt.contract, rt.projection_device = False, {}, 'cuda'
        rt._project, rt.projection_ready, rt.projection_warm_up = self.FakeCuda(times), False, None
        return rt

    def test_step_refused_before_warm_up_and_not_after(self):
        rt = self._runtime([1.4, .03, .03])
        with self.assertRaises(R.ProjectionNotReady):
            rt.step({}, {}, now_s=0.)
        rt.warm_up_projection({}, {})
        self.assertTrue(rt.projection_ready)
        self.assertFalse(rt.projection_warm_up['slow_first_call'])
        with self.assertRaises(Exception) as ctx:            # past the gate: fails later on the empty state
            rt.step({}, {}, now_s=0.)
        self.assertNotIsInstance(ctx.exception, R.ProjectionNotReady)

    def test_slow_first_call_is_flagged_and_warned(self):
        rt = self._runtime([49.5, .03, .03])
        with self.assertWarns(RuntimeWarning):
            rt.warm_up_projection({}, {})
        self.assertTrue(rt.projection_warm_up['slow_first_call'])
        self.assertTrue(rt.projection_ready)

    def test_cpu_runtime_needs_no_warm_up(self):
        rt = R.CameraPolicyRuntime.__new__(R.CameraPolicyRuntime)
        self.assertTrue(getattr(rt, 'projection_ready', True))


if __name__ == '__main__':
    unittest.main()
