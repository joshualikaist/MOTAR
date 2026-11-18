"""tools/oakd_recording_convert.py on synthetic captures (CPU only; no DepthAI, device or hardware).

The synthetic captures are written with the field recorder's own frame encoder and declare source "synthetic", so the
converted folders are never reported as real data. Every refusal is paired with a positive control.
"""
import contextlib
import hashlib
import io
import importlib.util
import json
import math
from pathlib import Path
import re
import sys
import tempfile
import types
import unittest
from unittest import mock

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / 'tools') not in sys.path:
    sys.path.insert(0, str(ROOT / 'tools'))
import oakd_camera                                                  # noqa: E402
import oakd_outdoor_replay as replay                                # noqa: E402
import oakd_recording_convert as V                                  # noqa: E402

_SPEC = importlib.util.spec_from_file_location('oakd_field_recorder_for_convert_tests',
                                               ROOT / 'tools/oakd_field_recorder.py')
REC = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(REC)

PRO_W_SIZE = (1280, 800)
PRO_W_K = np.array([[600.0, 0.0, 641.3], [0.0, 600.0, 397.8], [0.0, 0.0, 1.0]])
PRO_W_D = np.array([-0.12, 0.03, 0.0005, -0.0003, -0.002] + [0.0] * 9)
FISHEYE_D = np.array([0.05, -0.01, 0.002, -0.0005])
CONTRACT = {'camera': {'translation_m': [0.10, 0.0, 0.03]}}
IDENTITY4 = np.eye(4).tolist()
# A non-symmetric proper rotation IMU -> camera (IMU x = camera -z, IMU y = camera x, IMU z = camera -y).
R_IMU_CAM = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0]])


def up_for(pitch_deg, roll_deg=0.0):
    """Up vector in the optical frame for a camera pitched nose-down by pitch_deg (and rolled by roll_deg)."""
    p, r = math.radians(pitch_deg), math.radians(roll_deg)
    up = np.array([0.0, -math.cos(p), -math.sin(p)])
    rz = np.array([[math.cos(r), -math.sin(r), 0], [math.sin(r), math.cos(r), 0], [0, 0, 1]])
    return rz @ up


def write_capture(folder, frames, *, k=PRO_W_K, d=PRO_W_D, model='Perspective', size=PRO_W_SIZE, sensor='OV9782',
                  extrinsic=IDENTITY4, pitch=0.0, status='COMPLETE', accel_std=0.02):
    """frames: list of (rgb uint8 [H,W,3], depth uint16 [H,W], up vector in the optical frame, t)."""
    folder = Path(folder)
    (folder / 'frames').mkdir(parents=True)
    rotation = np.asarray(extrinsic, np.float64)[:3, :3] if extrinsic is not None else np.eye(3)
    entries = []
    for index, (rgb, depth, up, t) in enumerate(frames):
        yuv = cv2.cvtColor(rgb, cv2.COLOR_RGB2YUV_I420)
        a_imu = rotation.T @ (9.81 * np.asarray(up, np.float64))
        rng = np.random.default_rng(index)
        accel = np.column_stack([np.linspace(t - 0.4, t, 40)] + [a_imu[i] + rng.normal(0, accel_std, 40)
                                                                for i in range(3)])
        stamps = {'t_device_s': t, 't_depth_device_s': t + 0.001, 't_host_synced_s': t + 5, 't_host_monotonic_s': t}
        camera = {'exposure_time_us': 1000.0 + index, 'sensitivity_iso': 100 + index, 'colour_temperature_k': 5500,
                  'lens_position': 0, 'lens_position_raw': 0.0}
        data = REC.encode_frame(REC.frame_arrays(yuv, 'YUV420p', size[0], size[1], depth, accel, None, stamps, index,
                                                 camera))
        relative = 'frames/%06d.npz' % index
        (folder / relative).write_bytes(data)
        entries.append({'file': relative, 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest(),
                        't_device_s': t})
    meta = {'format': REC.FORMAT, 'source': 'synthetic', 'status': status, 'scene': 'synthetic fixture',
            'camera_height_m': 1.0, 'camera_height_uncertainty_m': 0.02, 'pitch_deg_nominal': pitch,
            'device': {'product_name': 'SYNTHETIC'}, 'sensors': {'CAM_A': sensor},
            'pipeline': {'colour': {'sensor': sensor, 'size': list(size)}},
            'calibration': {'sockets': {'CAM_A': {'intrinsics_at_output': np.asarray(k).tolist(),
                                                  'output_size': list(size), 'distortion_model': model,
                                                  'distortion_coefficients': np.asarray(d).tolist()}},
                            'imu_to_cam_a': {'matrix_4x4': extrinsic}},
            'calibration_eeprom': {'fixture': True}, 'frames': entries}
    (folder / 'capture.json').write_text(json.dumps(meta))
    return folder


def plain_frame(size=PRO_W_SIZE, value=90, depth_mm=3000):
    rgb = np.full((size[1], size[0], 3), value, np.uint8)
    depth = np.full((size[1], size[0]), depth_mm, np.uint16)
    return rgb, depth


def project(points, k, d, model):
    pts = np.asarray(points, np.float64).reshape(-1, 1, 3)
    if model == 'Fisheye':
        out, _ = cv2.fisheye.projectPoints(pts, np.zeros((3, 1)), np.zeros((3, 1)), k, d.reshape(4, 1))
    else:
        out, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), k, d)
    return out.reshape(-1, 2)


class ConvertTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def capture(self, name, frames=None, **kw):
        if frames is None:
            frames = [plain_frame() + (up_for(kw.get('pitch', 0.0) or 0.0), 1.0 + 0.1 * i) for i in range(2)]
        return write_capture(self.root / name, frames, **kw)

    def convert(self, capture, name, **kw):
        kw.setdefault('depth_model', 'undistorted_pinhole')
        return V.convert(capture, self.root / name, **kw)

    # ---- the replay accepts the output
    def test_output_loads_through_the_replay_loader_and_frame_inputs(self):
        cap = self.capture('cap', pitch=0.0)
        recording, rows = self.convert(cap, 'out')
        out = self.root / 'out'
        meta = replay.load_recording(out)
        self.assertEqual(meta['source'], 'synthetic')
        self.assertEqual(meta['intrinsics']['width'], 640)
        self.assertEqual(len(meta['frames']), 2)
        last = None
        for relative in meta['frames']:
            frame, state, roll, pitch, t = replay.frame_inputs(out, meta, relative, CONTRACT)
            self.assertEqual(frame['rgb'].shape, (360, 640, 3))
            self.assertEqual(frame['coverage'].dtype, np.bool_)
            self.assertTrue(frame['coverage'].all())
            self.assertAlmostEqual(roll, 0.0, places=1)
            self.assertAlmostEqual(pitch, 0.0, places=1)
            self.assertAlmostEqual(float(frame['depth_z_m'][180, 320]), 3.0, places=5)
            self.assertTrue(last is None or t > last)
            last = t
        conversion = meta['device']['conversion']
        self.assertEqual(conversion['preset'], 'trained_crop')
        self.assertEqual(conversion['depth_coordinate_model'], 'undistorted_pinhole')
        self.assertEqual(conversion['gravity']['source'], 'imu')
        self.assertEqual(meta['device']['calibration_eeprom'], {'fixture': True})

    # ---- crop geometry
    def test_crop_intrinsics_match_the_trained_camera_convention(self):
        source = (ROOT / 'tools/camera_policy_contract.py').read_text()
        self.assertRegex(source, r'width, height, hfov, vfov = 160, 90, 66\.0, 54\.0')
        self.assertIn('"cx": (width - 1) / 2.0', source)
        self.assertIn('width / (2.0 * math.tan(math.radians(hfov / 2.0)))', source)
        trained = V.trained_crop_intrinsics(160, 90)
        self.assertAlmostEqual(trained[0, 0], 160 / (2 * math.tan(math.radians(33))))
        self.assertAlmostEqual(trained[1, 1], 90 / (2 * math.tan(math.radians(27))))
        self.assertEqual((trained[0, 2], trained[1, 2]), (79.5, 44.5))
        k = V.trained_crop_intrinsics(640, 360)
        for i, j in ((0, 0), (79, 44), (159, 89), (17, 63)):
            u, v = 4 * i + 1.5, 4 * j + 1.5                            # centre of the 4x4 block of trained pixel
            np.testing.assert_allclose([(u - k[0, 2]) / k[0, 0], (v - k[1, 2]) / k[1, 1]],
                                       [(i - 79.5) / trained[0, 0], (j - 44.5) / trained[1, 1]], atol=1e-12)
        self.assertAlmostEqual(math.degrees(2 * math.atan((640 / 2) / k[0, 0])), 66.0)
        self.assertAlmostEqual(math.degrees(2 * math.atan((360 / 2) / k[1, 1])), 54.0)
        # The adapter's own coverage check accepts the crop K and refuses a 1 % narrower one.
        oakd_camera.Rectifier(k, [0.0] * 5, width=640, height=360).require_training_fov()
        narrow = k.copy()
        narrow[0, 0] *= 1.01
        with self.assertRaises(oakd_camera.OakDInputError):
            oakd_camera.Rectifier(narrow, [0.0] * 5, width=640, height=360).require_training_fov()
        for bad in ((600, 360), (640, 300), (0, 0), (1440, 810)):
            with self.assertRaises(V.ConvertError):
                V.trained_crop_intrinsics(*bad)
        self.assertEqual(V.trained_crop_intrinsics(480, 270).shape, (3, 3))

    def test_projected_point_lands_at_the_predicted_rectified_pixel(self):
        targets = [(320, 180), (100, 50), (600, 300), (20, 340), (610, 15)]
        checked = 0
        for model, d in (('Perspective', PRO_W_D), ('Fisheye', FISHEYE_D)):
            k_out = V.trained_crop_intrinsics(640, 360)
            rays = [((u - k_out[0, 2]) / k_out[0, 0], (v - k_out[1, 2]) / k_out[1, 1], 1.0) for u, v in targets]
            points = [np.array(r) * z for r, z in zip(rays, (3.0, 5.0, 2.0, 7.0, 4.0))]
            raw_px = project(points, PRO_W_K, d, model)
            pinhole_px = [(PRO_W_K[0, 0] * p[0] / p[2] + PRO_W_K[0, 2], PRO_W_K[1, 1] * p[1] / p[2] + PRO_W_K[1, 2])
                          for p in points]
            geometry = V.Geometry('trained_crop', PRO_W_K, d, model, PRO_W_SIZE, 'undistorted_pinhole')
            for (u, v), (ur, vr) in zip(targets, raw_px):               # the maps themselves
                self.assertLess(abs(geometry.rgb_map[0][v, u] - ur), 0.5)
                self.assertLess(abs(geometry.rgb_map[1][v, u] - vr), 0.5)
            # End to end: Gaussian blobs at the distorted raw positions, depth squares at the pinhole positions.
            yy, xx = np.mgrid[0:PRO_W_SIZE[1], 0:PRO_W_SIZE[0]].astype(np.float64)
            luma = np.full(xx.shape, 30.0)
            for ur, vr in raw_px:
                luma += 200.0 * np.exp(-((xx - ur) ** 2 + (yy - vr) ** 2) / (2 * 2.0 ** 2))
            rgb = np.repeat(np.clip(luma, 0, 255).astype(np.uint8)[:, :, None], 3, axis=2)
            depth = np.full(xx.shape, 5000, np.uint16)
            for up_, vp in pinhole_px:
                depth[int(round(vp)) - 3:int(round(vp)) + 4, int(round(up_)) - 3:int(round(up_)) + 4] = 1234
            cap = write_capture(self.root / ('blob_' + model), [(rgb, depth, up_for(0.0), 1.0)], d=d, model=model)
            self.convert(cap, 'blob_out_' + model)
            with np.load(self.root / ('blob_out_' + model) / 'frames/000000.npz') as data:
                out_rgb, out_depth = data['rgb'][:, :, 0].astype(np.float64), data['depth_mm']
            for u, v in targets:
                window = out_rgb[max(v - 10, 0):v + 11, max(u - 10, 0):u + 11] - 30.0
                weights = np.clip(window, 0, None)
                wy, wx = np.mgrid[max(v - 10, 0):v + 11, max(u - 10, 0):u + 11]
                cu, cv = (weights * wx).sum() / weights.sum(), (weights * wy).sum() / weights.sum()
                self.assertLess(math.hypot(cu - u, cv - v), 0.5, (model, u, v, cu, cv))
                self.assertEqual(int(out_depth[v, u]), 1234)
                checked += 1
        self.assertEqual(checked, 2 * len(targets))

    def test_depth_zero_stays_zero_and_never_bleeds(self):
        rng = np.random.default_rng(3)
        rgb, _ = plain_frame()
        depth = rng.integers(500, 20000, (PRO_W_SIZE[1], PRO_W_SIZE[0])).astype(np.uint16)
        depth[300:420, 500:700] = 0
        depth[rng.random(depth.shape) < 0.05] = 0
        source_values = set(np.unique(depth).tolist())
        for model in V.DEPTH_MODELS:
            cap = write_capture(self.root / ('bleed_' + model), [(rgb, depth, up_for(0.0), 1.0)])
            self.convert(cap, 'bleed_out_' + model, depth_model=model)
            with np.load(self.root / ('bleed_out_' + model) / 'frames/000000.npz') as data:
                out, coverage = data['depth_mm'], data['coverage']
            self.assertTrue(set(np.unique(out).tolist()) <= source_values)   # no interpolated value
            geometry = V.Geometry('trained_crop', PRO_W_K, PRO_W_D, 'Perspective', PRO_W_SIZE, model)
            mx, my = geometry.depth_map
            ix, iy = np.rint(mx).astype(int), np.rint(my).astype(int)
            inside = (ix >= 0) & (ix < PRO_W_SIZE[0]) & (iy >= 0) & (iy < PRO_W_SIZE[1])
            clear = (np.abs(mx - np.floor(mx) - 0.5) > 1e-3) & (np.abs(my - np.floor(my) - 0.5) > 1e-3) & inside
            expected = depth[np.clip(iy, 0, PRO_W_SIZE[1] - 1), np.clip(ix, 0, PRO_W_SIZE[0] - 1)]
            self.assertGreater(clear.mean(), 0.99)
            np.testing.assert_array_equal(out[clear], expected[clear])
            zeros_in = (out == 0) & coverage
            self.assertGreater(int(zeros_in.sum()), 1000)                     # the zero block survives
            self.assertGreater(int(((out > 0) & coverage).sum()), 100000)     # and valid depth is kept
            self.assertTrue(np.all(expected[zeros_in & clear] == 0))         # every zero comes from a source zero

    def test_depth_model_choice_is_required_and_changes_the_depth_map(self):
        with self.assertRaises(V.ConvertError):
            V.Geometry('trained_crop', PRO_W_K, PRO_W_D, 'Perspective', PRO_W_SIZE, 'guess')
        a = V.Geometry('trained_crop', PRO_W_K, PRO_W_D, 'Perspective', PRO_W_SIZE, 'distorted_rgb')
        b = V.Geometry('trained_crop', PRO_W_K, PRO_W_D, 'Perspective', PRO_W_SIZE, 'undistorted_pinhole')
        self.assertIs(a.depth_map, a.rgb_map)
        shift = np.hypot(a.depth_map[0] - b.depth_map[0], a.depth_map[1] - b.depth_map[1])
        self.assertGreater(float(shift.max()), 5.0)                       # the two models differ at the corners
        self.assertLess(float(shift[180, 320]), 1.0)
        with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):   # no default on the CLI
            V.main(['--capture', str(self.root), '--dry-run'])

    # ---- presets and FOV
    def test_presets_and_fov_shortfall(self):
        imx_size, imx_k = (1014, 760), np.array([[650.0, 0, 506.5], [0, 650.0, 379.5], [0, 0, 1]])
        imx_d = np.array([0.01, -0.005] + [0.0] * 12)
        cap = self.capture('imx', frames=[plain_frame(imx_size) + (up_for(0.0), 1.0)], k=imx_k, d=imx_d,
                           size=imx_size, sensor='IMX378')
        recording, _ = self.convert(cap, 'imx_out')
        self.assertEqual(recording['device']['conversion']['preset'], 'adapter')
        self.assertEqual(recording['intrinsics'], {'width': 1014, 'height': 760, 'fx': 650.0, 'fy': 650.0,
                                                   'cx': 506.5, 'cy': 379.5})
        rect = oakd_camera.Rectifier(imx_k, imx_d)
        geometry = V.Geometry('adapter', imx_k, imx_d, 'Perspective', imx_size, 'distorted_rgb')
        np.testing.assert_array_equal(geometry.rgb_map[0], rect.mapx)
        np.testing.assert_array_equal(geometry.rgb_map[1], rect.mapy)
        self.assertTrue(geometry.fov_check.startswith('PASS'))
        narrow_k = np.array([[800.0, 0, 506.5], [0, 800.0, 379.5], [0, 0, 1]])
        narrow = self.capture('imx_narrow', frames=[plain_frame(imx_size) + (up_for(0.0), 1.0)], k=narrow_k,
                              d=imx_d, size=imx_size, sensor='IMX378')
        with self.assertRaises(V.ConvertError) as caught:
            self.convert(narrow, 'imx_narrow_out')
        self.assertEqual(caught.exception.code, 'FOV_SHORTFALL')
        self.assertFalse((self.root / 'imx_narrow_out').exists())
        recording, _ = self.convert(narrow, 'imx_narrow_allowed', allow_fov_shortfall=True)
        self.assertTrue(recording['device']['conversion']['trained_fov_check'].startswith('FAIL_ALLOWED'))
        # Pro W crop: a long lens cannot cover 66 deg -> refused; the wide fixture (control) is covered.
        long_k = PRO_W_K.copy()
        long_k[0, 0] = long_k[1, 1] = 1100.0
        with self.assertRaises(V.ConvertError) as caught:
            V.Geometry('trained_crop', long_k, PRO_W_D, 'Perspective', PRO_W_SIZE, 'distorted_rgb')
        self.assertEqual(caught.exception.code, 'FOV_SHORTFALL')
        wide = V.Geometry('trained_crop', PRO_W_K, PRO_W_D, 'Perspective', PRO_W_SIZE, 'distorted_rgb')
        self.assertEqual(wide.uncovered, 0)
        self.assertEqual(int(wide.coverage.sum()), 640 * 360)
        unknown = self.capture('unknown', sensor='IMX214')
        with self.assertRaises(V.ConvertError) as caught:
            self.convert(unknown, 'unknown_out')
        self.assertEqual(caught.exception.code, 'PRESET')
        recording, _ = self.convert(unknown, 'unknown_forced', preset='trained_crop')
        self.assertEqual(len(recording['frames']), 2)
        with self.assertRaises(V.ConvertError) as caught:
            V.Geometry('trained_crop', PRO_W_K, PRO_W_D, 'Equirectangular', PRO_W_SIZE, 'distorted_rgb')
        self.assertEqual(caught.exception.code, 'UNSUPPORTED_DISTORTION_MODEL')

    # ---- gravity
    def test_up_optical_level_and_ten_degrees_nose_down(self):
        for pitch, expected in ((0.0, [0.0, -1.0, 0.0]),
                                (10.0, [0.0, -math.cos(math.radians(10)), -math.sin(math.radians(10))])):
            for extrinsic in (IDENTITY4, np.vstack([np.column_stack([R_IMU_CAM, [1.0, 2.0, 3.0]]),
                                                    [0, 0, 0, 1]]).tolist()):
                name = 'up_%g_%d' % (pitch, len(str(extrinsic)))
                cap = self.capture(name, frames=[plain_frame() + (up_for(pitch), 1.0)], extrinsic=extrinsic,
                                   pitch=pitch)
                self.convert(cap, name + '_out')
                with np.load(self.root / (name + '_out') / 'frames/000000.npz') as data:
                    up = data['up_optical']
                np.testing.assert_allclose(up, expected, atol=0.01)
                roll_deg, pitch_deg = replay.roll_pitch_from_up(up)
                self.assertAlmostEqual(pitch_deg, pitch, delta=0.5)
                self.assertAlmostEqual(roll_deg, 0.0, delta=0.5)
        # Using R^T instead of R would give a different vector for the non-symmetric rotation (non-vacuous).
        a_imu = R_IMU_CAM.T @ up_for(10.0)
        self.assertGreater(np.linalg.norm(R_IMU_CAM.T @ a_imu - up_for(10.0)), 0.5)

    def test_imu_refusals_and_nominal_fallback(self):
        cap = self.capture('mismatch', frames=[plain_frame() + (up_for(10.0), 1.0)], pitch=-10.0)
        with self.assertRaises(V.ConvertError) as caught:
            self.convert(cap, 'mismatch_out')
        self.assertEqual(caught.exception.code, 'IMU_NOMINAL_PITCH_MISMATCH')
        recording, rows = self.convert(cap, 'mismatch_unchecked', check_nominal=False)
        self.assertTrue(recording['device']['conversion']['gravity']['nominal_check'].startswith('NOT_CHECKED'))
        self.assertAlmostEqual(rows[0]['imu_pitch_deg'], 10.0, delta=0.5)
        shaky = self.capture('shaky', accel_std=2.0)
        with self.assertRaises(V.ConvertError) as caught:
            self.convert(shaky, 'shaky_out')
        self.assertEqual(caught.exception.code, 'IMU_NOT_STILL')
        bad = np.eye(4)
        bad[0, 0] = 2.0
        with self.assertRaises(V.ConvertError) as caught:
            self.convert(self.capture('bad_ext', extrinsic=bad.tolist()), 'bad_ext_out')
        self.assertEqual(caught.exception.code, 'BAD_IMU_EXTRINSICS')
        recording, rows = self.convert(self.capture('noext', extrinsic=None, pitch=10.0), 'noext_out')
        self.assertEqual(recording['device']['conversion']['gravity']['source'], 'nominal_pitch')
        with np.load(self.root / 'noext_out/frames/000000.npz') as data:
            self.assertNotIn('up_optical', data.files)
            np.testing.assert_array_equal(data['roll_pitch_deg'], [0.0, 10.0])
        meta = replay.load_recording(self.root / 'noext_out')
        _, _, roll, pitch, _ = replay.frame_inputs(self.root / 'noext_out', meta, meta['frames'][0], CONTRACT)
        self.assertEqual((roll, pitch), (0.0, 10.0))
        with self.assertRaises(V.ConvertError) as caught:
            self.convert(self.capture('nogravity', extrinsic=None, pitch=None), 'nogravity_out')
        self.assertEqual(caught.exception.code, 'NO_GRAVITY_SOURCE')

    # ---- capture integrity, time, manifest
    def test_timestamps_incomplete_and_hashes(self):
        frames = [plain_frame() + (up_for(0.0), t) for t in (1.0, 1.0)]
        with self.assertRaises(V.ConvertError) as caught:
            self.convert(write_capture(self.root / 'same_t', frames), 'same_t_out')
        self.assertEqual(caught.exception.code, 'NON_INCREASING_TIMESTAMPS')
        frames = [plain_frame() + (up_for(0.0), t) for t in (1.0, 0.9)]
        with self.assertRaises(V.ConvertError):
            self.convert(write_capture(self.root / 'back_t', frames), 'back_t_out')
        with self.assertRaises(V.ConvertError) as caught:
            self.convert(self.capture('incomplete', status='INCOMPLETE'), 'incomplete_out')
        self.assertEqual(caught.exception.code, 'INCOMPLETE_CAPTURE')
        tampered = self.capture('tampered')
        path = tampered / 'frames/000001.npz'
        data = bytearray(path.read_bytes())
        data[-1] ^= 0xFF
        path.write_bytes(bytes(data))
        with self.assertRaises(V.ConvertError) as caught:
            V.load_capture(tampered)
        self.assertEqual(caught.exception.code, 'FRAME_HASH_MISMATCH')
        good = self.capture('good')
        self.assertEqual(len(V.load_capture(good)['frames']), 2)
        recording, rows = self.convert(good, 'good_out')
        self.assertEqual([r['t'] for r in rows], [1.0, 1.1])
        out = self.root / 'good_out'
        self.assertEqual(V.verify_manifest(out), 3)                      # two frames and recording.json
        with self.assertRaises(V.ConvertError) as caught:
            self.convert(good, 'good_out')
        self.assertEqual(caught.exception.code, 'OUTPUT_EXISTS')
        frame = out / 'frames/000000.npz'
        data = bytearray(frame.read_bytes())
        data[len(data) // 2] ^= 0x01
        frame.write_bytes(bytes(data))
        with self.assertRaises(V.ConvertError) as caught:
            V.verify_manifest(out)
        self.assertEqual(caught.exception.code, 'FRAME_HASH_MISMATCH')

    def test_dry_run_writes_nothing_and_cli(self):
        cap = self.capture('dry')
        recording, rows = V.convert(cap, None, depth_model='distorted_rgb', dry_run=True)
        self.assertEqual(len(rows), 2)
        self.assertEqual(sorted(p.name for p in self.root.iterdir()), ['dry'])
        printed = io.StringIO()
        with contextlib.redirect_stdout(printed):
            self.assertEqual(V.main(['--capture', str(cap), '--depth-coordinate-model', 'distorted_rgb',
                                     '--dry-run']), 0)
            self.assertEqual(V.main(['--capture', str(cap), '--depth-coordinate-model', 'undistorted_pinhole',
                                     '--output', str(self.root / 'cli_out')]), 0)
        self.assertEqual([json.loads(line)['frames'] for line in printed.getvalue().splitlines()], [2, 2])
        self.assertEqual(len(replay.load_recording(self.root / 'cli_out')['frames']), 2)
        with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
            V.main(['--capture', str(cap), '--depth-coordinate-model', 'distorted_rgb'])
        with contextlib.redirect_stderr(io.StringIO()) as err:
            self.assertEqual(V.main(['--capture', str(self.root / 'missing'), '--depth-coordinate-model',
                                     'distorted_rgb', '--dry-run']), 2)
        self.assertIn('CAPTURE_UNREADABLE', err.getvalue())
        self.assertTrue(re.match(r'^[0-9a-f]{64}$', recording['device']['capture']['capture_json_sha256']))

    def test_colour_decoding_matches_depthai_get_cv_frame_formula(self):
        rng = np.random.default_rng(1)
        rgb = rng.integers(0, 256, (8, 16, 3), dtype=np.uint8)
        yuv = cv2.cvtColor(rgb, cv2.COLOR_RGB2YUV_I420)
        expected = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_IYUV)[:, :, ::-1]
        cap = write_capture(self.root / 'tiny', [(rgb, np.full((8, 16), 1000, np.uint16), up_for(0.0), 1.0)],
                            size=(16, 8), k=np.array([[8.0, 0, 7.5], [0, 8.0, 3.5], [0, 0, 1]]), d=np.zeros(14))
        meta = V.load_capture(cap)
        decoded, depth, _, t = V.decode_frame(cap, meta['frames'][0], (16, 8))
        np.testing.assert_array_equal(decoded, expected)
        self.assertGreater(int(decoded.sum()), 0)
        self.assertEqual(t, 1.0)

    # ---- repair r1
    def test_colour_3a_is_copied_through_and_the_replay_still_loads(self):
        cap = self.capture('3a')
        recording, rows = self.convert(cap, '3a_out')
        out = self.root / '3a_out'
        meta = replay.load_recording(out)
        summary = meta['device']['colour_3a']['summary']
        self.assertEqual(summary['exposure_time_us'], {'min': 1000.0, 'max': 1001.0, 'frames': 2})
        self.assertEqual(summary['sensitivity_iso'], {'min': 100, 'max': 101, 'frames': 2})
        self.assertEqual(len(meta['device']['colour_3a']['per_frame']), 2)
        for index, relative in enumerate(meta['frames']):
            with np.load(out / relative, allow_pickle=False) as data:
                self.assertEqual(float(data['exposure_time_us']), 1000.0 + index)
                self.assertEqual(int(data['colour_temperature_k']), 5500)
                self.assertTrue(bool(data['auto_exposure_configured']))
            frame, state, roll, pitch, t = replay.frame_inputs(out, meta, relative, CONTRACT)   # extra keys ignored
            self.assertEqual(frame['rgb'].shape, (360, 640, 3))
        self.assertEqual(sorted(p.name for p in self.root.iterdir()), ['3a', '3a_out'])

    def test_gravity_source_choice(self):
        cap = self.capture('grav', frames=[plain_frame() + (up_for(10.0), 1.0)], pitch=10.0)
        recording, _ = self.convert(cap, 'grav_nominal', gravity_source='nominal')
        self.assertEqual(recording['device']['conversion']['gravity']['source'], 'nominal_pitch')
        self.assertEqual(recording['device']['conversion']['gravity']['requested'], 'nominal')   # review r2 N3
        with np.load(self.root / 'grav_nominal/frames/000000.npz') as data:
            self.assertNotIn('up_optical', data.files)
            np.testing.assert_array_equal(data['roll_pitch_deg'], [0.0, 10.0])
        recording, rows = self.convert(cap, 'grav_imu', gravity_source='imu')                  # control
        self.assertEqual(recording['device']['conversion']['gravity']['source'], 'imu')
        self.assertEqual(recording['device']['conversion']['gravity']['requested'], 'imu')
        self.assertAlmostEqual(rows[0]['imu_pitch_deg'], 10.0, delta=0.5)
        recording, _ = self.convert(cap, 'grav_auto')                                          # default: auto
        self.assertEqual((recording['device']['conversion']['gravity']['source'],
                          recording['device']['conversion']['gravity']['requested']), ('imu', 'auto'))
        recording, _ = self.convert(self.capture('grav_noext_auto', extrinsic=None, pitch=0.0), 'grav_noext_auto_out')
        self.assertEqual((recording['device']['conversion']['gravity']['source'],
                          recording['device']['conversion']['gravity']['requested']), ('nominal_pitch', 'auto'))
        noext = self.capture('grav_noext', extrinsic=None, pitch=0.0)
        with self.assertRaises(V.ConvertError) as caught:
            self.convert(noext, 'grav_noext_imu', gravity_source='imu')
        self.assertEqual(caught.exception.code, 'NO_IMU_EXTRINSICS')
        dead = write_capture(self.root / 'grav_dead', [plain_frame() + (up_for(0.0), 1.0)], pitch=0.0)
        meta = json.loads((dead / 'capture.json').read_text())
        with np.load(dead / 'frames/000000.npz') as data:
            arrays = {k: data[k] for k in data.files}
        arrays.update(accel=np.zeros((0, 4)), accel_mean=np.full(3, np.nan), accel_std=np.full(3, np.nan),
                      accel_count=np.array(0, np.int64))                       # a dead IMU
        payload = REC.encode_frame(arrays)
        (dead / 'frames/000000.npz').write_bytes(payload)
        meta['frames'][0]['sha256'] = hashlib.sha256(payload).hexdigest()
        (dead / 'capture.json').write_text(json.dumps(meta))
        with self.assertRaises(V.ConvertError) as caught:
            self.convert(dead, 'grav_dead_auto')
        self.assertEqual(caught.exception.code, 'IMU_NO_SAMPLES')
        recording, _ = self.convert(dead, 'grav_dead_nominal', gravity_source='nominal')     # still convertible
        self.assertEqual(len(recording['frames']), 1)
        old = self.capture('grav_old', pitch=None)
        with self.assertRaises(V.ConvertError) as caught:
            self.convert(old, 'grav_old_out')
        self.assertEqual(caught.exception.code, 'NO_NOMINAL_PITCH')
        recording, _ = self.convert(old, 'grav_old_unchecked', check_nominal=False)
        self.assertTrue(recording['device']['conversion']['gravity']['nominal_check'].startswith('NOT_CHECKED'))

    def test_roll_limit_of_the_nominal_check(self):
        cap = self.capture('roll', frames=[plain_frame() + (up_for(0.0, roll_deg=10.0), 1.0)], pitch=0.0)
        with self.assertRaises(V.ConvertError) as caught:
            self.convert(cap, 'roll_out')
        self.assertEqual(caught.exception.code, 'IMU_NOMINAL_PITCH_MISMATCH')
        self.assertIn('roll', str(caught.exception))
        ok = self.capture('roll_ok', frames=[plain_frame() + (up_for(0.0, roll_deg=3.0), 1.0)], pitch=0.0)
        recording, rows = self.convert(ok, 'roll_ok_out')                                       # control
        self.assertAlmostEqual(abs(rows[0]['imu_roll_deg']), 3.0, delta=0.5)

    def test_coverage_includes_the_depth_source(self):
        # Barrel distortion keeps every colour source inside the raw image while the pinhole depth source of the
        # top and bottom rows falls outside it (pinhole half-VFOV atan(400 / 850) < 27 deg).
        k = np.array([[850.0, 0, 639.5], [0, 850.0, 399.5], [0, 0, 1]])
        d = np.array([-0.35] + [0.0] * 13)
        colour_only = V.Geometry('trained_crop', k, d, 'Perspective', PRO_W_SIZE, 'distorted_rgb')
        self.assertEqual(colour_only.uncovered, 0)                                              # control
        with self.assertRaises(V.ConvertError) as caught:
            V.Geometry('trained_crop', k, d, 'Perspective', PRO_W_SIZE, 'undistorted_pinhole')
        self.assertEqual(caught.exception.code, 'FOV_SHORTFALL')
        both = V.Geometry('trained_crop', k, d, 'Perspective', PRO_W_SIZE, 'undistorted_pinhole',
                          allow_fov_shortfall=True)
        rgb_cov = V.source_coverage(*both.rgb_map, PRO_W_SIZE)
        self.assertTrue(rgb_cov.all())
        self.assertGreater(both.uncovered, 0)
        self.assertFalse(both.coverage[0].any() or both.coverage[-1].any())
        self.assertTrue(both.coverage[180].all())

    def test_output_size_estimate_disk_refusal_and_partial_cleanup(self):
        cap = self.capture('disk')
        printed = io.StringIO()
        with contextlib.redirect_stdout(printed):
            self.assertEqual(V.main(['--capture', str(cap), '--depth-coordinate-model', 'undistorted_pinhole',
                                     '--dry-run']), 0)
        report = json.loads(printed.getvalue())
        self.assertGreater(report['output_mb_estimate'], 0)
        self.assertGreater(report['output_bytes'], 0)
        tiny = types.SimpleNamespace(free=1 << 20)
        with mock.patch.object(V.shutil, 'disk_usage', return_value=tiny):
            with self.assertRaises(V.ConvertError) as caught:
                self.convert(cap, 'disk_out')
        self.assertEqual(caught.exception.code, 'INSUFFICIENT_DISK')
        self.assertEqual(sorted(p.name for p in self.root.iterdir()), ['disk'])               # nothing written
        # a refusal after frames were written removes this call's .partial folder; a rerun is not blocked
        frames = [plain_frame() + (up_for(0.0), 1.0), plain_frame() + (up_for(20.0), 1.1)]
        late = write_capture(self.root / 'late', frames, pitch=0.0)
        with self.assertRaises(V.ConvertError) as caught:
            self.convert(late, 'late_out')
        self.assertEqual(caught.exception.code, 'IMU_NOMINAL_PITCH_MISMATCH')
        self.assertFalse((self.root / 'late_out').exists())
        self.assertFalse((self.root / 'late_out.partial').exists())
        recording, _ = self.convert(late, 'late_out', gravity_source='nominal')                 # rerun, control
        self.assertEqual(len(recording['frames']), 2)
        (self.root / 'blocked.partial').mkdir()
        with self.assertRaises(V.ConvertError) as caught:
            self.convert(cap, 'blocked')
        self.assertEqual(caught.exception.code, 'OUTPUT_EXISTS')


if __name__ == '__main__':
    unittest.main()
