"""CPU controls for the additive recorder; no simulator, CUDA or checkpoints."""
import contextlib
import gzip
import io
import itertools
import json
import os
import shutil
import struct
import pathlib
import resource
import tempfile
import types
import unittest
from unittest import mock

import numpy as np
import sys
_SOURCE = pathlib.Path(__file__).resolve().parent
if _SOURCE.name == 'tests':
    _SOURCE = _SOURCE.parent / 'tools'
_saved_path = list(sys.path)
sys.path.insert(0, str(_SOURCE))
try:
    import sitl_s4r1 as R
finally:
    sys.path[:] = _saved_path

STREAMS = ('LOCAL_POSITION_NED', 'ATTITUDE', 'HIL_STATE_QUATERNION')


def ulog_record(typ, payload):
    return struct.pack('<HB', len(payload), ord(typ)) + payload


def ulog_bytes(topics=R.ULOG_REQUIRED, t0_us=0, t1_us=10 ** 9):
    wire = b'ULog\x01\x12\x35' + b'\0' * 9
    for mid, name in enumerate(topics):
        wire += ulog_record('A', struct.pack('<BH', 0, mid) + name.encode())
        wire += ulog_record('D', struct.pack('<HQ', mid, t0_us))
        wire += ulog_record('D', struct.pack('<HQ', mid, t1_us))
    return wire


class FrameSource:
    """A fake plugin client: absolute stamps start at `base` (boot-dependent), 0.1 s per read. `edit(j, frame)` may
    change read j (j = 0 is the hover read; step k uses read k + 1)."""
    def __init__(self, base=3.0, rgb_value=1, edit=None):
        self.j, self.base, self.rgb_value, self.edit = 0, base, rgb_value, edit

    def __call__(self):
        return self

    def frames(self):
        t = self.base + 0.1 * self.j
        f = dict(sim_now=t, lidar_t=t - .1, rgb_t=t - .1, depth_t=t - .2, target_t=t - .3, target_count=5 + self.j,
                 ranges=np.ones((4, 72)), rgb=np.full((9, 16, 3), self.rgb_value, np.uint8),
                 depth=np.ones((9, 16), np.float32))
        if self.edit is not None:
            self.edit(self.j, f)
        self.j += 1
        return f

    def set_target(self, *args):
        return 3.0

    def close(self):
        return 99


def fake_extractor(n, ok=True):
    """Stands in for the PX4-env pyulog subprocess: writes a gzip reconstruction covering the n control steps."""
    def run(argv, **kw):
        if not ok:
            return types.SimpleNamespace(returncode=1, stderr='parse failed')
        ts = [int(round((10.0 + 0.1 * k) * 1e6)) for k in range(n)]
        topics = {name: {'timestamp': ts, 'x': [0.0] * n} for name in
                  ('vehicle_local_position_setpoint', 'trajectory_setpoint', 'vehicle_attitude_groundtruth')}
        with open(argv[-1], 'xb') as f:
            f.write(gzip.compress(json.dumps({'topics': topics, 'dropouts': []}).encode(), mtime=0))
        return types.SimpleNamespace(returncode=0, stderr='')
    return run


def spec_for(env):
    rng = np.random.default_rng(env)
    return {'env': env, 'centers': rng.normal(size=(3, 3)), 'halves': np.ones((3, 3)), 'quats': np.zeros((3, 4)),
            'p0': np.array([env, 0., 1.]), 'yaw0': 0.1 * env, 'tpos': np.zeros((R.SF.FULL_HORIZON, 3)),
            'bounds_min': -np.ones(3), 'bounds_max': np.ones(3), 'target_source': 'zero_action_cell',
            'isaac_outcome': 'timeout'}


def build_member(ep, out_dir, item, spec, n=3, shift=0.0, base=100.0, obs_delta=None, logger_rc=0, seed=4501,
                 edit=None, extract_ok=True, audit=None):
    """One synthetic boot through the real recorder pieces, R.checked_gzip (stubbed pyulog subprocess) and
    R.finish_episode; F1's result has arm = cell."""
    ep = pathlib.Path(ep)
    ep.mkdir(parents=True)
    clock = itertools.count()
    tap = R.Tap(clock=lambda: float(next(clock)))
    tap.out_dir = out_dir
    tap.counts.update({k: 2 for k in STREAMS})
    tap.raw_link = types.SimpleNamespace(pos=(10.0, 0., 0., -1., 0., 0., 0.), att=(10.0, 0, 0, 0), truth=None, hb=None)
    client = R.client_factory(tap, FrameSource(base, edit=edit))()
    client.frames()                                                    # hover read, before the control phase
    for k in range(n):
        client.frames()
        s = {key: np.array([0.1 * k + shift, 0., 1.]) for key in ('pos', 'vel', 'render_pos')}
        s.update(vquat=np.array([0., 0., 0., 1.]), render_vquat=np.array([0., 0., 0., 1.]))
        obs = np.arange(898, dtype=np.float32) + np.float32(shift)
        if obs_delta is not None and k >= obs_delta[0]:
            obs = obs + np.float32(obs_delta[1])
        tap.control_step(s, np.ones((9, 16, 3), np.float32), np.ones((1, 9, 16), np.float32),
                         np.ones((1, 288), np.float32), obs)
        tap.pending.update(action=np.ones(4), governed=np.ones(2), prev_action=np.ones(4))
        tap.pending['actor_output'] = np.ones((1, 4))
        client.set_target(1., 2., 3.)
    tap.post_control()
    contact = item['contact']
    tap.logger_stop = {'returncode': logger_rc}
    if contact:
        tap.contact_control = {'ok': True, 'last_t': 10.0}
        tap.contact_api = types.SimpleNamespace(
            contact_audit=audit or (lambda *a, **kw: ({'messages': 5, 'bar_events': []}, [])))
        (ep / 'logs').mkdir()
        (ep / 'logs/contacts.log').write_text('time {sec: 10}\n')
    legacy = np.zeros((n, 27))
    legacy[:, 0] = np.arange(n)
    legacy[:, 1] = 10.0 + 0.1 * np.arange(n)
    legacy[:, 2] = legacy[:, 23] = 0.1 * np.arange(n) + shift
    np.save(ep / 'steps.npy', legacy)
    np.save(ep / 'sensor_steps.npy', np.zeros((n, 16)))
    (ep / 'session.json').write_text(json.dumps({'launches': [
        {'role': 'gzserver', 'argv': ['gzserver', 'w.world', '--seed', str(seed)]}]}))
    log = ep / 'rootfs/log/2026-11-10'
    log.mkdir(parents=True)
    (log / 'a.ulg').write_bytes(ulog_bytes())
    res = {'outcome': 'timeout', 'steps': n, 'late_max_s': 0.05, 'param_problems': [], 'arm': 'lit_rendered',
           'cell': 'lit_rendered', 's4_arm': 'ground', 'cell_world': {'world_sha256': 'w%d' % spec['env']},
           'runtime_binding': {'files': {'px4': {'sha256': 'p'}, 'world': {'sha256': 'x'}}},
           'ulog': R.checked_gzip(ep, tap, run=fake_extractor(n, extract_ok)),
           'start_check': {'truth_alt_m': 1.0 + shift, 'ekf_alt_m': 1.0, 'ekf_minus_truth_xy_m': 0.01,
                           'truth_from_spawn_xy_m': 0.02},
           'hover_guard': {'lidar_min_m': 1.0, 'depth_min_m': 1.0}}
    return R.finish_episode(tap, res, ep, contact, out_dir, spec)


class FakeAT:
    @staticmethod
    def kernel_faults():
        return 0

    @staticmethod
    def git_state(root):
        return {'head': 'h', 'dirty': False}

    @staticmethod
    def idle_check():
        return True, {'cores': 'idle'}


def build_attempt(d, n_rows=12):
    """A complete synthetic 12-boot attempt written through SF.run_unit's real dict(item, **res) ledger path."""
    d = pathlib.Path(d)
    d.mkdir(parents=True, exist_ok=True)
    with mock.patch.object(R.F1, 'scene_envs', return_value=[0, 1, 2]):
        sched = R.schedule()
    specs = {e: spec_for(e) for e in (0, 1)}

    def run_one(i, item, spec):
        shift = 0.01 * item['repeat'] + (0.003 if item['contact'] else 0.0)
        return build_member(d / ('ep%03d' % i), d, item, spec, shift=shift, base=100.0 + 7 * i)
    with contextlib.redirect_stdout(io.StringIO()):
        status, reason, done = R.SF.run_unit(sched[:n_rows], specs, run_one, FakeAT, d / 'episodes.jsonl',
                                             key='arm', primary=None, invalid_abort_a=0, busy_wait_s=0)
    (d / 'attempt.json').write_text(json.dumps({'schedule': sched, 'git': {'head': 'h', 'dirty': False},
                                                'kernel_faults_before': 0}))
    (d / 'final.json').write_text(json.dumps({'status': status, 'reason': reason, 'kernel_faults_after': 0,
                                              'git_after': {'head': 'h', 'dirty': False}}))
    return sched


class TestS4R1(unittest.TestCase):
    def test_schedule_balanced_and_fixed(self):
        with mock.patch.object(R.F1, 'scene_envs', return_value=[0, 1, 2]):
            s = R.schedule()
            smoke = R.smoke_schedule()
        self.assertEqual(len(s), 12)
        self.assertEqual([(r['env'], r['contact']) for r in s[:4]], [(0, False), (0, True), (1, True), (1, False)])
        for e in (0, 1):
            for c in (False, True):
                self.assertEqual(sum(x['env'] == e and x['contact'] == c for x in s), 3)
        self.assertEqual([(x['arm'], x['env'], x['repeat'], x['contact']) for x in smoke],
                         [('contact_on', 0, 0, True)])
        with mock.patch.object(R.F1, 'scene_envs', return_value=[0, 2]):
            self.assertRaises(ValueError, R.schedule)

    def fixture(self):
        tap = R.Tap(clock=lambda: 12.0)
        tap.counts.update({k: 1 for k in STREAMS})
        client = R.client_factory(tap, FrameSource(3.0))()
        client.frames()  # hover
        f = client.frames()  # first chosen packet
        s = {key: np.array([0., 0., 1.]) for key in ('pos', 'vel', 'vquat', 'render_pos', 'render_vquat')}
        tap.control_step(s, f['rgb'], f['depth'], f['ranges'], np.arange(898, dtype=np.float32))
        tap.pending.update(action=np.ones(4), actor_output=np.ones((1, 4)), governed=np.ones(2), prev_action=np.ones(4))
        tap.post_control()
        tap.contact_control = dict(R.OFF_CONTACT)
        tap.logger_stop = {'returncode': 0}
        return tap, client

    def test_raw_payload_roundtrip_positive_and_corruption(self):
        tap, client = self.fixture()
        self.assertEqual(client.set_target(1, 2, 3), 3.)
        self.assertEqual(client.close(), 99)
        with tempfile.TemporaryDirectory() as d:
            rec = R.dump_tap(tap, d, d)
            self.assertEqual(rec['packets'], 2)
            p = pathlib.Path(d) / 'repeatability'
            a = np.load(p / 'packet0001.npz')
            meta = json.loads((p / 'packets.json').read_text())[1]
            for key in ('rgb', 'depth', 'ranges'):
                self.assertEqual(R.array_signature(a[key]), meta['signatures'][key])
            bad = a['rgb'].copy()
            bad[0, 0, 0] = 2
            self.assertNotEqual(R.array_signature(bad), meta['signatures']['rgb'])
            summary = json.loads((p / 'summary.json').read_text())
            self.assertEqual(summary['sensor_production_counter'], 'NOT_RECORDED')
            self.assertGreater(summary['recorder_overhead_s']['frames']['calls'], 0)
            with gzip.open(p / 'events.jsonl.gz', 'rt') as f:
                events = [json.loads(x) for x in f]
            self.assertEqual([e['i'] for e in events], list(range(len(events))))

    def test_recording_counts_and_phase_valid_and_invalid(self):
        tap, _ = self.fixture()
        ok = {'steps': 1, 'ulog': {'all_verified': True}}
        self.assertEqual(R.recording_problems(tap, ok, False), [])
        self.assertTrue(any(x.startswith('ULOG_INTEGRITY') for x in R.recording_problems(tap, {'steps': 1}, False)))
        tap.logger_stop = {'returncode': 1}
        self.assertTrue(any(x.startswith('LOGGER_STOP') for x in R.recording_problems(tap, ok, False)))
        tap.logger_stop = {'returncode': 0}
        tap.contact_control = {'ok': False}
        self.assertIn('CONTACT_CONTROL', R.recording_problems(tap, ok, True))
        tap.contact_control = {'ok': True}
        self.assertEqual(R.recording_problems(tap, ok, True), [])
        self.assertIn('RECORDING_COUNT', R.recording_problems(tap, dict(ok, steps=2), True))
        self.assertEqual([x['phase'] for x in tap.events if x['kind'] == 'chosen_frame'], ['control'])
        self.assertEqual(tap.phase, 'post_terminal_landing')

    def test_mav_proxy_preserves_calls_and_landing_boundary(self):
        class M:
            def command_long_send(self, *a, **kw):
                return [a, kw]
        tap, _ = self.fixture()
        tap.mark('control')
        p = R.MavProxy(M(), tap)
        args = (1, 1, 176, 0, 1, 4, 6)
        self.assertEqual(p.command_long_send(*args), [args, {}])
        self.assertEqual(tap.events[-1]['phase'], 'post_terminal_landing')
        self.assertEqual(tap.events[-1]['kind'], 'mav_send')
        tap.mark('control')
        p.command_long_send(1, 1, 400, 0, 1)
        self.assertEqual(tap.phase, 'control')

    def test_link_passthrough_without_state_events_and_packet_state_snapshot(self):
        msg = types.SimpleNamespace(get_type=lambda: 'ATTITUDE', to_dict=lambda: {'time_boot_ms': 10},
                                    get_srcSystem=lambda: 1, get_srcComponent=lambda: 1, get_seq=lambda: 8)
        conn = types.SimpleNamespace(mav=object(), recv_match=lambda *a, **kw: msg)
        tap = R.Tap()
        cp = R.ConnProxy(conn, tap)
        self.assertIs(cp.recv_match(), msg)
        self.assertEqual(tap.counts['ATTITUDE'], 1)
        link = types.SimpleNamespace(pos=(3., 1., 2.), stop=False)
        proxy = R.LinkProxy(link, tap)
        n = len(tap.events)
        for _ in range(100):
            self.assertEqual(proxy.pos, (3., 1., 2.))
        self.assertEqual(len(tap.events), n)                           # no per-read event (MAJOR-2)
        proxy.stop = True
        self.assertTrue(link.stop)
        tap.raw_link = link
        R.client_factory(tap, FrameSource())().frames()
        self.assertEqual(tap.packets[-1]['link_state_at_return']['pos'], (3., 1., 2.))
        self.assertIsNone(tap.packets[-1]['link_state_at_return']['truth'])

    def test_contact_control_on_pass_stale_and_dead_subscriber(self):
        with tempfile.TemporaryDirectory() as d:
            tap = R.Tap()
            tap.contact_enabled = True
            session = mock.Mock(episode_dir=pathlib.Path(d))
            tap.session = session
            last = {'t': 10.2}
            tap.contact_api = types.SimpleNamespace(floor_control=lambda p: (True, {'last_t': last['t']}))
            link = types.SimpleNamespace(pos=(10.0, 0, 0, 0), read_params=lambda *a: {'P': 1.0})
            proxy = R.LinkProxy(link, tap)
            self.assertEqual(proxy.read_params(['P']), {'P': 1.0})
            self.assertTrue(tap.contact_control['ok'])
            session.check.assert_called_with(required=('contacts',))
            last['t'] = 5.0
            self.assertRaises(RuntimeError, proxy.read_params, ['P'])
            self.assertIn('not current', tap.contact_control['problem'])
            last['t'] = 10.2
            session.check.side_effect = RuntimeError('contacts exited with 1')
            self.assertRaises(RuntimeError, proxy.read_params, ['P'])
            # OFF: no control, no subscriber check
            tap2 = R.Tap()
            tap2.session = mock.Mock()
            R.LinkProxy(link, tap2).read_params(['P'])
            tap2.session.check.assert_not_called()
            self.assertIsNone(tap2.contact_control)

    def test_episode_and_policy_off_identity(self):
        class Base:
            def observe_with(self, s, rgb, depth, lidar):
                return np.array([[1., 2.]]), {'ok': True}, lidar, {'shadow': 1}

            def command(self, a, lidar):
                return a[:2] * 2, .3, a
        tap, _ = self.fixture()
        tap.steps.clear()
        ep = R.episode_class(tap, object(), factory=lambda b: Base)()
        s = {'pos': np.ones(3)}
        rgb, depth, lidar = np.ones(3), np.ones(2), np.ones(4)
        result = ep.observe_with(s, rgb, depth, lidar)
        np.testing.assert_array_equal(result[0], [[1., 2.]])

        class Policy:
            def act(self, obs):
                return np.array([[.1, .2, .3, .4]])
        pol = R.policy_proxy(Policy(), tap)
        a = pol.act(result[0])
        cmd = ep.command(a[0], lidar)
        np.testing.assert_array_equal(a, [[.1, .2, .3, .4]])
        np.testing.assert_array_equal(cmd[0], [.2, .4])
        np.testing.assert_array_equal(tap.steps[0]['actor_output'], a)
        np.testing.assert_array_equal(tap.steps[0]['obs'], result[0])

    def test_session_check_never_walks_tree_and_disk_check_is_throttled(self):
        class Owned:
            def __init__(self, ep, **kw):
                self.kw, self._procs, self.calls = kw, {'contacts': {}}, []

            def check(self, required=()):
                self.calls.append(tuple(required))
        fake_os = types.SimpleNamespace(OwnedSession=Owned, direct_launch_plan=None, launch_px4_gazebo=None)
        with tempfile.TemporaryDirectory() as d:
            tap = R.Tap()
            s = R.session_module(fake_os, tap, True, d).OwnedSession(d)
            self.assertEqual(s.kw, {'deadline_s': R.EPISODE_WALL_S, 'rss_cap_bytes': 8 << 30})
            free = mock.Mock(return_value=10 << 30)
            with mock.patch.object(pathlib.Path, 'rglob', side_effect=AssertionError('tree walk')), \
                    mock.patch.object(os, 'walk', side_effect=AssertionError('tree walk')), \
                    mock.patch.object(R, 'output_size', side_effect=AssertionError('tree walk')), \
                    mock.patch.object(R, 'disk_free', free):
                for _ in range(50):
                    s.check(required=('gzserver', 'px4'))
                self.assertEqual(len(s.calls), 50)
                self.assertEqual(s.calls[0], ('gzserver', 'px4', 'contacts'))   # one check, subscriber included
                self.assertEqual(free.call_count, 1)                            # throttled statvfs
                free.return_value = 0
                tap.next_resource_check = 0.0
                with self.assertRaisesRegex(RuntimeError, 'DISK_LOW'):
                    s.check(required=('gzserver', 'px4'))
                # the client's frame read does no session check at all
                tap.session = mock.Mock()
                R.client_factory(tap, FrameSource())().frames()
                tap.session.check.assert_not_called()
            off = R.session_module(fake_os, R.Tap(), False, d).OwnedSession(d)
            with mock.patch.object(R, 'disk_free', return_value=10 << 30):
                off.check(required=('gzserver', 'px4'))
            self.assertEqual(off.calls, [('gzserver', 'px4')])

    def test_budget_counter_without_tree_walk_positive_and_cap(self):
        with tempfile.TemporaryDirectory() as d:
            b = R.Budget(d)                                              # the one walk, at a boundary
            with mock.patch.object(pathlib.Path, 'rglob', side_effect=AssertionError('tree walk')):
                for i in range(5):
                    b.write_bytes(pathlib.Path(d) / ('f%d' % i), b'x' * 10)
                self.assertEqual(b.written, 50)
                with mock.patch.object(R, 'MAX_OUTPUT_BYTES', 55):
                    self.assertRaisesRegex(RuntimeError, 'OUTPUT_CAP', b.write_bytes, pathlib.Path(d) / 'g', b'y' * 10)
                    self.assertFalse((pathlib.Path(d) / 'g').exists())
                with mock.patch.object(R, 'disk_free', return_value=0):
                    self.assertRaisesRegex(RuntimeError, 'DISK_LOW', b.check, 1)

    def test_output_cap_accepts_small_and_rejects_full(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(R.resource_problems(d), [])
            with mock.patch.object(R, 'MAX_OUTPUT_BYTES', 1):
                (pathlib.Path(d) / 'x').write_bytes(b'xx')
                self.assertIn('OUTPUT_CAP', R.resource_problems(d))

    def test_compare_relative_stamps_stage_first_k_and_maxabs(self):
        item = {'contact': False}
        with tempfile.TemporaryDirectory() as d:
            d = pathlib.Path(d)
            build_member(d / 'a', d, item, spec_for(0), n=4, base=100.0)
            build_member(d / 'b', d, item, spec_for(0), n=4, base=250.0)
            build_member(d / 'c', d, item, spec_for(0), n=4, base=100.0, obs_delta=(2, 0.5))
            same = R.compare_members(d / 'a', d / 'b')
            # absolute boot times differ; relative stamps do not (MAJOR-1)
            self.assertIsNone(same['stages']['sensor_stamp_relative']['first_exact_k'])
            self.assertAlmostEqual(same['absolute_offsets']['t0_difference_s'], -150.0)
            self.assertEqual(same['earliest']['status'], 'NO_RECORDED_DIFFERENCE')
            self.assertTrue(same['hover_packet']['payload_exact_equal'])
            diff = R.compare_members(d / 'a', d / 'c')
            ob = diff['stages']['observation']
            self.assertEqual((ob['first_exact_k'], ob['first_k_above_tol']), (2, 2))
            self.assertAlmostEqual(ob['max_abs'], 0.5)
            self.assertEqual(ob['max_abs_at_fixed_steps'], {'0': 0.0})
            self.assertIsNone(diff['stages']['actor']['first_exact_k'])
            self.assertEqual(diff['earliest'], dict(diff['earliest'], status='SINGLE_EARLIEST', k=2,
                                                    earliest_stages=['observation']))
            # a below-tolerance difference is exact-different but not above the reporting tolerance
            build_member(d / 'e', d, item, spec_for(0), n=4, base=100.0, obs_delta=(1, 1e-7))
            small = R.compare_members(d / 'a', d / 'e')['stages']['observation']
            self.assertEqual(small['first_exact_k'], 1)
            self.assertIsNone(small['first_k_above_tol'])
            # corrupting a stored signature is detected as an exact difference
            p = d / 'b/repeatability/steps.json'
            x = json.loads(p.read_text())
            x[0]['signatures']['actor_output']['sha256'] = 'corrupted'
            p.write_text(json.dumps(x))
            self.assertEqual(R.compare_members(d / 'a', d / 'b')['stages']['actor']['first_exact_k'], 0)

    def test_tie_rule(self):
        def stages(**firsts):
            return {s: {'first_exact_k': firsts.get(s)} for s in R.STAGE_NAMES}
        self.assertEqual(R.tie_reading(stages())['status'], 'NO_RECORDED_DIFFERENCE')
        allt = R.tie_reading(stages(**{s: 0 for s in R.STAGE_NAMES}))
        self.assertEqual((allt['status'], allt['k']), ('ALL_TIED', 0))
        self.assertIn('uninformative', allt['note'])
        two = R.tie_reading(stages(state=3, observation=3, actor=5))
        self.assertEqual((two['status'], two['earliest_stages']), ('TIED', ['state', 'observation']))
        one = R.tie_reading(stages(actor=4, governed_request=4, sensor_payload=2))
        self.assertEqual((one['status'], one['earliest_stages']), ('SINGLE_EARLIEST', ['sensor_payload']))

    def test_analyze_positive_control_and_negatives(self):
        with tempfile.TemporaryDirectory() as root:
            d = pathlib.Path(root) / 'attempt01'
            build_attempt(d)
            rows = R.SF.read_jsonl(d / 'episodes.jsonl')
            self.assertEqual([r['arm'] for r in rows[:4]], ['contact_off', 'contact_on', 'contact_on', 'contact_off'])
            self.assertTrue(all(r['cell'] == 'lit_rendered' and r['valid'] for r in rows))
            report = R.analyze(d)
            self.assertEqual(report['integrity_problems'], [])
            self.assertEqual(report['reading'], 'REPORTED_DESCRIPTIVE')
            self.assertEqual(len(report['comparisons']), 30)
            self.assertEqual(report['outcome_agreement'],
                             {'within_off': {'pairs': 6, 'equal': 6}, 'within_on': {'pairs': 6, 'equal': 6},
                              'between_conditions': {'pairs': 18, 'equal': 18}})
            sp = report['spread_by_scene']['0']
            self.assertEqual((sp['within_off']['pairs'], sp['between_conditions']['pairs']), (3, 9))
            self.assertGreater(sp['between_conditions']['truth_pos_dist_max_m']['max'], 0)
            self.assertGreater(sp['within_on']['start_state']['truth_pos_dist_m']['n'], 0)
            # review r2 MINOR-2: the estimated-position mean and fixed-step summaries
            bc = sp['between_conditions']
            self.assertEqual(bc['est_pos_dist_mean_m']['n'], 9)
            self.assertGreater(bc['est_pos_dist_mean_m']['max'], 0)
            fixed = bc['est_pos_dist_at_fixed_steps_m']
            self.assertEqual((fixed['0']['n'], fixed['10']['n']), (9, 0))
            # review r2 MINOR-1: shifts 0.01*repeat (+0.003 ON) give within distances {0.01, 0.02} and between
            # distances {0.003 x3, 0.007 x2, 0.013 x2, 0.017, 0.023}: 5 below, 3 inside, 1 above
            for env in ('0', '1'):
                q2 = report['spread_by_scene'][env]['question2']
                self.assertEqual(q2['primary_statistic'], 'truth_pos_dist_max_m')
                self.assertEqual({k: q2['primary'][k] for k in ('within_n', 'between_n', 'below', 'inside', 'above')},
                                 {'within_n': 6, 'between_n': 9, 'below': 5, 'inside': 3, 'above': 1})
                self.assertAlmostEqual(q2['primary']['within_range'][0], 0.01)
                self.assertAlmostEqual(q2['primary']['within_range'][1], 0.02)
                self.assertEqual(q2['primary_at_fixed_steps']['0']['inside'], 3)
                self.assertEqual(q2['primary_at_fixed_steps']['10']['status'], 'NOT_AVAILABLE')
                self.assertEqual((q2['secondary']['inside'], q2['secondary']['within_range']), (9, [0.0, 0.0]))
            saved = json.loads((d / 'reading.json').read_text())
            self.assertEqual(saved['spread_by_scene']['0']['question2']['primary']['inside'], 3)
            self.assertTrue((d / 'reading.json').is_file())
            self.assertRaises(FileExistsError, R.analyze, d)              # written once

            serial = itertools.count()

            def negative(mutate, expect):
                e = pathlib.Path(root) / ('neg%d' % next(serial))
                shutil.copytree(d, e)
                (e / 'reading.json').unlink()
                mutate(e)
                out = R.analyze(e, write=False)
                self.assertEqual(out['reading'], 'INCOMPLETE_INVALID', expect)
                self.assertTrue(any(expect in x for x in out['integrity_problems']), out['integrity_problems'])

            def rewrite(e, fn):
                rs = fn(R.SF.read_jsonl(e / 'episodes.jsonl'))
                (e / 'episodes.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rs))

            def wrong_arm(e):
                rewrite(e, lambda rs: [dict(r, arm='lit_rendered') if r['index'] == 3 else r for r in rs])
            negative(wrong_arm, "schedule's prefix")
            negative(lambda e: rewrite(e, lambda rs: rs[:-1]), 'fixed12 boots incomplete')

            def changed(e):
                with open(e / 'ep004/repeatability/step0001.npz', 'ab') as f:
                    f.write(b'\0')
            negative(changed, 'recording bytes changed')

            def ulog_changed(e):
                gz = next((e / 'ep005').glob(R.F1.ULOG_GLOB + '.gz'))
                gz.write_bytes(gz.read_bytes() + b'\0')
            negative(ulog_changed, 'gzip SHA-256 differs')

            def contact_changed(e):
                with open(e / 'ep001/logs/contacts.log', 'a') as f:
                    f.write('x\n')
            negative(contact_changed, 'recording bytes changed: logs/contacts.log')

            def seed_lost(e):
                rewrite(e, lambda rs: [dict(r, gazebo_seed={'seed': None}) if r['index'] == 2 else r for r in rs])
            negative(seed_lost, 'Gazebo seed not bound')

            def reconstruction_unbound(e):
                def drop(rs):
                    for r in rs:
                        if r['index'] == 6:
                            del r['repeatability']['files'][R.ULOG_RECONSTRUCTION]
                    return rs
                rewrite(e, drop)
            negative(reconstruction_unbound, 'ulog_reconstruction.json.gz not bound')       # review r2 LOW

            def smoke_attempt(e):
                a = json.loads((e / 'attempt.json').read_text())
                (e / 'attempt.json').write_text(json.dumps(dict(a, development_smoke=True)))
            negative(smoke_attempt, 'smoke has no reading')

    def test_finish_episode_sets_condition_arm_and_preserves_physical_outcome(self):
        with tempfile.TemporaryDirectory() as d:
            d = pathlib.Path(d)
            on = build_member(d / 'on', d, {'contact': True}, spec_for(0))
            self.assertEqual((on['arm'], on['cell'], on['s4_arm'], on['recording_problems']),
                             ('contact_on', 'lit_rendered', 'ground', []))
            self.assertIn('logs/contacts.log', on['repeatability']['files'])
            off = build_member(d / 'off', d, {'contact': False}, spec_for(0))
            self.assertEqual((off['arm'], off['contact_control']), ('contact_off', R.OFF_CONTACT))
            self.assertEqual(off['gazebo_seed']['seed'], 4501)
            bad = build_member(d / 'bad', d, {'contact': False}, spec_for(0), seed=7, logger_rc=1)
            self.assertEqual((bad['outcome'], bad['physical_outcome_before_recording_check']),
                             ('HARNESS_INVALID', 'timeout'))
            self.assertTrue(any(x.startswith('SEED_NOT_BOUND') for x in bad['recording_problems']))
            self.assertTrue(any(x.startswith('LOGGER_STOP') for x in bad['recording_problems']))
            self.assertEqual(bad['arm'], 'contact_off')
            # review r2 LOW: every NATIVE_BOUND file is required; an audit exception still leaves a row
            noext = build_member(d / 'noext', d, {'contact': False}, spec_for(0), extract_ok=False)
            self.assertIn('NATIVE_FILE_MISSING: ulog_reconstruction.json.gz', noext['recording_problems'])
            self.assertEqual(noext['physical_outcome_before_recording_check'], 'timeout')

            def broken(*a, **kw):
                raise ValueError('parser bug')
            crash = build_member(d / 'crash', d, {'contact': True}, spec_for(0), audit=broken)
            self.assertIn('CONTACT_AUDIT: ValueError: parser bug', crash['recording_problems'])
            self.assertEqual((crash['outcome'], crash['arm']), ('HARNESS_INVALID', 'contact_on'))
            self.assertIn('logs/contacts.log', crash['repeatability']['files'])

    def test_run_episode_wiring_overrides_f1_arm(self):
        real = R.private_module

        def fake(name, path):
            if name == '_s4r1_f1':
                return types.SimpleNamespace(run_episode=lambda *a, **kw: {
                    'outcome': 'timeout', 'arm': 'lit_rendered', 'cell': 'lit_rendered', 'steps': 0})
            if name == '_s4r1_s4':
                return types.SimpleNamespace()
            return real(name, path)
        fake_os = types.SimpleNamespace(OwnedSession=object, direct_launch_plan=None, launch_px4_gazebo=None)
        with tempfile.TemporaryDirectory() as d, mock.patch.object(R, 'private_module', side_effect=fake):
            res = R.run_episode(None, fake_os, None, spec_for(1), pathlib.Path(d) / 'ep000', None, False, d)
        self.assertEqual((res['arm'], res['cell'], res['contact']), ('contact_off', 'lit_rendered', False))
        self.assertEqual((res['outcome'], res['physical_outcome_before_recording_check']),
                         ('HARNESS_INVALID', 'timeout'))
        self.assertIn('RECORDING_COUNT', res['recording_problems'])

    def test_instrumentation_smoke_gate_positive_and_refusals(self):
        with mock.patch.object(R.F1, 'scene_envs', return_value=[0, 1]):
            item = dict(R.smoke_schedule()[0])

        def smoke(d, **kw):
            d = pathlib.Path(d)
            d.mkdir()
            res = build_member(d / 'ep000', d, item, spec_for(0), **kw)
            ok, why = R.SF.episode_valid(res)
            (d / 'episodes.jsonl').write_text(json.dumps(dict(item, index=0, valid=ok, invalid_reason=why, **res)) + '\n')
            return d
        with tempfile.TemporaryDirectory() as root:
            root = pathlib.Path(root)
            self.assertEqual(R.instrumentation_smoke_problems(smoke(root / 'good')), [])
            self.assertTrue(R.instrumentation_smoke_problems(smoke(root / 'logger', logger_rc=1)))
            self.assertTrue(R.instrumentation_smoke_problems(smoke(root / 'seed', seed=1)))
            item['contact'], item['arm'] = False, 'contact_off'
            self.assertTrue(any('not env0 contact ON' in x for x in R.instrumentation_smoke_problems(smoke(root / 'off'))))
            self.assertTrue(R.instrumentation_smoke_problems(root / 'missing'))

    def test_runtime_problems_exact_parent_and_no_rlimit(self):
        inf = resource.RLIM_INFINITY
        xauth = tempfile.NamedTemporaryFile(prefix='xauth')
        self.addCleanup(xauth.close)
        env = {'PYTHONPATH': str(R.ROOT), 'DISPLAY': ':1', 'XAUTHORITY': xauth.name}
        good = dict(parent=('timeout', R.expected_parent_argv('--launch')), rlimit=(inf, inf),
                    environ=env, cwd=R.ROOT, executable=R.PYTHON,
                    aerial_gym_origin=str(R.ROOT / 'aerial_gym/__init__.py'), display_probe=lambda e: (True, 'ok'))
        self.assertEqual(R.runtime_problems('--launch', **good), [])
        self.assertEqual(R.expected_parent_argv('--launch'),
                         ['timeout', '-k', '60', '1800', R.PYTHON, '-B', 'tools/sitl_s4r1.py', '--launch'])
        smoke_good = dict(good, parent=('timeout', R.expected_parent_argv('--smoke')))
        self.assertEqual(R.runtime_problems('--smoke', **smoke_good), [])
        bad = {
            'rlimit': (8 << 30, 8 << 30),
            'parent': ('timeout', ['timeout', '60', '1800', 'python', 'tools/sitl_s4r1.py', '--launch']),
            'environ': dict(env, PYTHONPATH='/elsewhere'),
            'cwd': pathlib.Path('/'),
            'executable': '/usr/bin/python3',
            'aerial_gym_origin': '/home/fair/workspaces/aerial_gym_ws/src/MOTAR/aerial_gym/__init__.py'}
        for key, value in bad.items():
            self.assertEqual(len(R.runtime_problems('--launch', **dict(good, **{key: value}))), 1, key)
        for argv in (R.expected_parent_argv('--smoke'), R.expected_parent_argv('--launch')[:-1],
                     ['timeout', '-k', '60', '1800', R.PYTHON, '-B', 'tools/other.py', '--launch'],
                     ['timeout', '1800'] + R.expected_parent_argv('--launch')[4:]):
            self.assertTrue(R.runtime_problems('--launch', **dict(good, parent=('timeout', argv))))
        self.assertTrue(R.runtime_problems('--launch', **dict(good, parent=('bash', R.expected_parent_argv('--launch')))))
        self.assertTrue(R.runtime_problems('--launch', **dict(good, environ={'PYTHONPATH': str(R.ROOT)})))

    def test_prereg_states_the_exact_commands(self):
        text = (R.ROOT / R.PREREG).read_text()
        for mode in ('--smoke', '--launch'):
            self.assertIn(R.launch_command(mode), text)
        self.assertNotIn('ulimit -v', R.launch_command())

    def test_checked_gzip_never_raises_and_keeps_native(self):
        with tempfile.TemporaryDirectory() as d:
            d = pathlib.Path(d)
            tap = R.Tap()
            tap.out_dir = d
            r = R.checked_gzip(d, tap)
            self.assertEqual((r['all_verified'], r['problems']), (False, ['ULOG_COUNT: 0']))
            log = d / 'rootfs/log/x'
            log.mkdir(parents=True)
            (log / 'a.ulg').write_bytes(ulog_bytes(R.ULOG_REQUIRED[:5]))
            np.save(d / 'steps.npy', np.array([[0, .5] + [0] * 25, [1, .6] + [0] * 25]))
            r = R.checked_gzip(d, tap)
            self.assertIn('ULOG_TOPIC: ' + R.ULOG_REQUIRED[5], r['problems'])
            self.assertTrue((log / 'a.ulg').is_file())
            (d / 'ulog_integrity.json').unlink()
            (log / 'a.ulg').write_bytes(ulog_bytes())
            (d / 'steps.npy').unlink()
            r = R.checked_gzip(d, tap)                                   # np.load failure: returned, not raised
            self.assertTrue(r['problems'][0].startswith('ULOG_CHECK_ERROR'))
            self.assertTrue((log / 'a.ulg').is_file())
            np.save(d / 'steps.npy', np.array([[0, .5] + [0] * 25, [1, .6] + [0] * 25]))

            def run(argv, **kw):
                with open(argv[-1], 'xb') as f:
                    f.write(gzip.compress(json.dumps({'topics': {'trajectory_setpoint': {'timestamp': [1]}}}).encode()))
                return types.SimpleNamespace(returncode=0, stderr='')
            r = R.checked_gzip(d, tap, run=run)
            self.assertTrue(r['all_verified'], r)
            self.assertFalse((log / 'a.ulg').exists())
            self.assertEqual(tap.ulog_data['topics']['trajectory_setpoint']['timestamp'], [1])
            fail = R.checked_gzip(d, tap, run=run)                       # the .ulg is gone now: a count problem
            self.assertFalse(fail['all_verified'])

    def test_seed_injection_into_gzserver_argv(self):
        plan = {'gzserver': {'argv': ['gzserver', 'w.world']}, 'spawn': {'argv': ['gz']}, 'px4': {'argv': ['px4']}}
        fake_os = types.SimpleNamespace(OwnedSession=object, direct_launch_plan=lambda *a, **kw: plan,
                                        launch_px4_gazebo=None)
        tap = R.Tap()
        p = R.session_module(fake_os, tap, False).direct_launch_plan('ep', 'w', {})
        self.assertEqual(p['gzserver']['argv'], ['gzserver', 'w.world', '--seed', '4501'])
        self.assertIs(tap.plan, p)

    def test_ulog_strict_eof_positive_and_truncation(self):
        wire = b'ULog\x01\x12\x35' + b'\0' * 9
        wire += ulog_record('A', struct.pack('<BH', 0, 7) + b'vehicle_attitude_groundtruth')
        wire += ulog_record('D', struct.pack('<HQ', 7, 1000000))
        wire += ulog_record('D', struct.pack('<HQ', 7, 1100000))
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / 'x.ulg'
            p.write_bytes(wire)
            good = R.ulog_framing(p)
            self.assertEqual(good['problems'], [])
            self.assertEqual(good['topics']['vehicle_attitude_groundtruth'], 2)
            p.write_bytes(wire[:-1])
            self.assertIn('ULOG_TRUNCATED_PAYLOAD', R.ulog_framing(p)['problems'])
            p.write_bytes(wire + b'\1')
            self.assertIn('ULOG_TRUNCATED_HEADER', R.ulog_framing(p)['problems'])
            p.write_bytes(b'bad')
            self.assertIn('ULOG_HEADER', R.ulog_framing(p)['problems'])

    def test_bracket_discloses_interval_and_missing(self):
        topic = {'timestamp': [1000000, 1100000, 1200000], 'vx': [1., 2., 3.]}
        r = R.nearest_bracket(topic, 1.05)
        self.assertEqual([x['fields']['vx'] for x in r['samples']], [1., 2.])
        self.assertAlmostEqual(r['alignment_uncertainty_s'], .05)
        self.assertEqual(R.nearest_bracket(None, 1.)['status'], 'NOT_RECORDED')

    def test_canonical_timeline_relative_stamps_and_not_recorded_contact(self):
        with tempfile.TemporaryDirectory() as d:
            d = pathlib.Path(d)
            build_member(d / 'ep', d, {'contact': False}, spec_for(0), n=3, base=100.0)
            t = json.loads((d / 'ep/repeatability/timeline.json').read_text())
            self.assertEqual(len(t), 3)
            self.assertAlmostEqual(t[0]['sensor_stamp_relative_s']['sim_now'], 0.0)
            self.assertAlmostEqual(t[2]['sensor_stamp_relative_s']['sim_now'], 0.2)
            self.assertEqual(t[0]['contact'], 'NOT_RECORDED: subscriber OFF')
            self.assertEqual(t[0]['target_send_next']['xyz'], [1., 2., 3.])
            self.assertEqual(t[0]['controller_setpoint']['status'], 'RECORDED')     # stub reconstruction
            self.assertEqual(t[2]['controller_setpoint']['samples'][0]['sim_s'], 10.2)

    def test_contact_parser_uses_s4_model_without_mutating_old_parser(self):
        tap = R.Tap()
        pair = {'c1': 'navrl_quad_s4::base_link::base_link_inertia_collision',
                'c2': 'asphalt_plane::link::collision'}
        self.assertEqual(tap.contact_api._kind(pair)[2], 'floor')
        self.assertIsNone(R.AGQ._kind(pair))
        self.assertIsNone(tap.contact_api._kind({'c1': 'other::c', 'c2': 'bar_0::l::c'}))

    def test_native_logger_stop_ownership_positive_and_refusals(self):
        with tempfile.TemporaryDirectory() as d:
            rootfs = pathlib.Path(d) / 'rootfs'
            expected = pathlib.Path('/home/fair/workspaces/px4_ws/PX4-Autopilot/build/px4_sitl_default/bin/px4')
            plan = {'px4': {'argv': [str(expected), '-d', str(expected.parent.parent / 'etc')],
                            'cwd': rootfs, 'env': {}}}

            class Session:
                episode_dir = pathlib.Path(d)
                _procs = {'px4': {'record': {'argv': plan['px4']['argv'], 'cwd': str(rootfs)}}}

                def check(self, required=()):
                    return None

                def start(self, *a, **kw):
                    self.started = (a, kw)

                def wait(self, *a, **kw):
                    return self.rc
            session = Session()
            session.rc = 0
            with mock.patch.object(pathlib.Path, 'is_file', return_value=True), \
                    mock.patch.object(pathlib.Path, 'resolve', return_value=expected), \
                    mock.patch.object(R.SF, 'sha256_file', return_value='fixture'):
                self.assertEqual(R.stop_owned_logger(session, plan)['returncode'], 0)
                session.rc = 1
                self.assertRaises(RuntimeError, R.stop_owned_logger, session, plan)
                session.rc = 0
                session._procs = {'px4': {'record': {'argv': ['foreign'], 'cwd': str(rootfs)}}}
                self.assertRaises(RuntimeError, R.stop_owned_logger, session, plan)

    def test_nonfinite_telemetry_is_preserved_with_explicit_tag(self):
        out = R.jsonable({'vx': float('nan'), 'pos': np.array([float('inf')])})
        self.assertEqual(out['vx'], {'nonfinite': 'nan'})
        self.assertEqual(out['pos'], [{'nonfinite': 'inf'}])
        json.dumps(out, allow_nan=False)
        self.assertEqual(R.max_abs(np.array([np.inf, 1.]), np.array([np.inf, 1.])), 0.0)
        self.assertEqual(R.max_abs(np.array([np.inf]), np.array([1.])), 0.0)      # finite-in-both only (r2 MAJOR-1)
        self.assertIsNone(R.max_abs(np.ones(2), np.ones(3)))

    def test_selected_sensor_arrays_are_stored_exactly(self):
        tap, _ = self.fixture()
        with tempfile.TemporaryDirectory() as d:
            R.dump_tap(tap, d, d)
            saved = np.load(pathlib.Path(d) / 'repeatability/step0000.npz')
            for key in ('rgb', 'depth', 'lidar', 'obs', 'actor_output', 'action', 'governed'):
                np.testing.assert_array_equal(saved[key], tap.steps[0][key])
            tap.steps[0]['rgb'][0, 0, 0] = 99
            self.assertNotEqual(saved['rgb'][0, 0, 0], tap.steps[0]['rgb'][0, 0, 0])


    # ------------------------------------------------------------------------------------- review r2 repairs
    def test_diff_stats_finite_maxabs_and_nonfinite_mismatch_count(self):
        inf, nan = np.inf, np.nan
        r = R.diff_stats(np.array([inf, 1., 2.]), np.array([5., 1., 2.5]))
        self.assertEqual(r, {'max_abs': 0.5, 'nonfinite_mismatch_n': 1})
        same = np.array([inf, nan, -inf, 1.])
        self.assertEqual(R.diff_stats(same, same.copy()), {'max_abs': 0.0, 'nonfinite_mismatch_n': 0})
        for x, y in ((inf, -inf), (nan, inf), (nan, 1.), (1., -inf)):
            self.assertEqual(R.diff_stats(np.array([x]), np.array([y])), {'max_abs': 0.0, 'nonfinite_mismatch_n': 1})
        self.assertEqual(R.diff_stats(np.full(3, inf), np.ones(3)), {'max_abs': 0.0, 'nonfinite_mismatch_n': 3})
        self.assertEqual(R.diff_stats(np.array([3], np.uint8), np.array([250], np.uint8))['max_abs'], 247.0)
        self.assertIsNone(R.diff_stats(np.ones((4, 72)), np.ones((4, 73))))

    def test_compare_nonfinite_flip_is_counted_not_incomparable(self):
        def flip(j, f):                                                 # steps k >= 1: one no-hit beam, one NaN depth
            if j >= 2:
                f['ranges'][0, 0], f['depth'][0, 0] = np.inf, np.nan
                f['ranges'][1, 1] = 1.25

        def flip_pattern_only(j, f):
            if j >= 2:
                f['ranges'][0, 0], f['depth'][0, 0] = np.inf, np.nan

        def wider(j, f):
            if j >= 3:
                f['ranges'] = np.ones((4, 73))

        def hover_inf(j, f):
            if j == 0:
                f['ranges'][2, 2] = np.inf
        item = {'contact': False}
        with tempfile.TemporaryDirectory() as d:
            d = pathlib.Path(d)
            for name, edit in (('a', None), ('b', flip), ('c', flip_pattern_only), ('w', wider), ('h', hover_inf)):
                build_member(d / name, d, item, spec_for(0), n=4, edit=edit)
            sp = R.compare_members(d / 'a', d / 'b')['stages']['sensor_payload']
            self.assertEqual((sp['first_exact_k'], sp['first_k_above_tol']), (1, 1))
            self.assertEqual(sp['max_abs'], 0.25)                                   # finite, not INCOMPARABLE
            self.assertEqual((sp['nonfinite_mismatch_n'], sp['nonfinite_mismatch_steps'],
                              sp['first_k_nonfinite_mismatch']), (6, 3, 1))
            self.assertEqual((sp['incomparable_steps'], sp['incomparable_k']), (0, None))
            self.assertEqual((sp['max_abs_at_fixed_steps'], sp['nonfinite_mismatch_at_fixed_steps']),
                             ({'0': 0.0}, {'0': 0}))
            # identical non-finite patterns count zero; the finite difference remains
            bc = R.compare_members(d / 'b', d / 'c')['stages']['sensor_payload']
            self.assertEqual((bc['max_abs'], bc['nonfinite_mismatch_n'], bc['first_exact_k']), (0.25, 0, 1))
            cc = R.compare_members(d / 'c', d / 'c')['stages']['sensor_payload']
            self.assertEqual((cc['first_exact_k'], cc['nonfinite_mismatch_n']), (None, 0))
            # a true shape mismatch is still incomparable, and only at the steps where it occurs
            sw = R.compare_members(d / 'a', d / 'w')['stages']['sensor_payload']
            self.assertEqual((sw['incomparable_steps'], sw['incomparable_k'], sw['first_exact_k']), (2, 2, 2))
            self.assertEqual((sw['max_abs'], sw['nonfinite_mismatch_n']), (0.0, 0))
            # the hover packet follows the same rule
            hv = R.compare_members(d / 'a', d / 'h')['hover_packet']
            self.assertEqual(hv, {'payload_exact_equal': False, 'max_abs': 0.0, 'nonfinite_mismatch_n': 1,
                                  'incomparable_keys': []})
            self.assertEqual(R.compare_members(d / 'a', d / 'a')['hover_packet']['payload_exact_equal'], True)
            # spread summaries keep the numbers (no silent drop)
            summary = R.spread_summary([dict(R.compare_members(d / 'a', d / 'b'), env=0, kind='within_off',
                                             outcome_equal=True, trajectory=R.trajectory_spread(d / 'a', d / 'b'),
                                             start_check_abs_diff={})])
            st = summary['0']['within_off']['stages']['sensor_payload']
            self.assertEqual((st['max_abs']['n'], st['max_abs']['max'], st['nonfinite_mismatch_n']['max']),
                             (1, 0.25, 6))

    def test_step0_offset_carried_by_relative_stamps_not_by_reference_free_readings(self):
        def late_step0(j, f):                                           # step 0 served one physics step later
            if j == 1:
                for key in R.STAMP_FIELDS:
                    f[key] += 0.004

        def count_jump(j, f):
            if j >= 3:
                f['target_count'] += 2

        def older_lidar(j, f):
            if j == 3:
                f['lidar_t'] -= 0.001
        item = {'contact': False}
        with tempfile.TemporaryDirectory() as d:
            d = pathlib.Path(d)
            for name, edit in (('a', None), ('late', late_step0), ('cnt', count_jump), ('age', older_lidar)):
                build_member(d / name, d, item, spec_for(0), n=4, edit=edit)
            late = R.compare_members(d / 'a', d / 'late')
            rel = late['stages']['sensor_stamp_relative']
            self.assertEqual((rel['first_exact_k'], rel['max_abs'], rel['max_abs_target_count']), (1, 4000.0, 0))
            self.assertAlmostEqual(late['absolute_offsets']['t0_difference_s'], -0.004)
            self.assertEqual(late['reference_free']['frame_age']['first_exact_k'], None)
            self.assertEqual((late['reference_free']['sim_now_increment']['first_exact_k'],
                              late['reference_free']['sim_now_increment']['max_abs_us']), (1, 4000))
            cnt = R.compare_members(d / 'a', d / 'cnt')['stages']['sensor_stamp_relative']
            self.assertEqual((cnt['max_abs'], cnt['max_abs_target_count'], cnt['first_k_above_tol']), (0.0, 2, 2))
            age = R.compare_members(d / 'a', d / 'age')
            self.assertEqual((age['reference_free']['frame_age']['first_exact_k'],
                              age['reference_free']['frame_age']['max_abs_us']), (2, 1000))
            self.assertEqual(age['stages']['sensor_stamp_relative']['max_abs'], 1000.0)
            self.assertIsNone(R.compare_members(d / 'a', d / 'a')['reference_free']['frame_age']['first_exact_k'])

    def test_question2_range_reading(self):
        r = R.range_reading([0.01, 0.02, 0.015, 0.01, 0.02, 0.012], [0.005, 0.015, 0.03, float('inf'), None])
        self.assertEqual({k: r[k] for k in ('within_n', 'between_n', 'below', 'inside', 'above')},
                         {'within_n': 6, 'between_n': 4, 'below': 1, 'inside': 1, 'above': 2})
        self.assertEqual(r['within_range'], [0.01, 0.02])
        edges = R.range_reading([1, 2], [1, 2])
        self.assertEqual((edges['inside'], edges['below'], edges['above']), (2, 0, 0))   # bounds are inside
        self.assertEqual(R.range_reading([], [1.0])['status'], 'NOT_AVAILABLE')
        self.assertEqual(R.range_reading([None], [1.0])['status'], 'NOT_AVAILABLE')

    def test_display_preflight_positive_and_refusals(self):
        with tempfile.TemporaryDirectory() as d:
            xa = pathlib.Path(d) / 'Xauthority'
            xa.write_bytes(b'cookie')
            env = {'DISPLAY': ':1', 'XAUTHORITY': str(xa), 'HOME': d}
            calls = []

            def probe(e):
                calls.append(dict(e))
                return True, 'ok'
            self.assertEqual(R.display_problems(env, probe), [])
            self.assertEqual(len(calls), 1)
            for bad in ({'DISPLAY': ':1'}, dict(env, XAUTHORITY=''), dict(env, XAUTHORITY=str(xa) + '.missing'),
                        dict(env, XAUTHORITY=d), dict(env, DISPLAY=':0'), {'XAUTHORITY': str(xa)}):
                self.assertEqual(len(R.display_problems(bad, probe)), 1, bad)
            self.assertEqual(len(calls), 1)                              # the X probe runs only when both hold
            self.assertEqual(len(R.display_problems(env, lambda e: (False, 'cannot open display'))), 1)
            if os.geteuid() != 0:
                xa.chmod(0)
                self.assertIn('not a readable file', R.display_problems(env, probe)[0])
                xa.chmod(0o600)
            # the real probe's command and environment, with a fake subprocess
            seen = {}

            def run(argv, **kw):
                seen.update(argv=argv, env=kw['env'], timeout=kw['timeout'])
                return types.SimpleNamespace(returncode=rc, stderr='err')
            rc = 0
            self.assertEqual(R.display_reachable(env, run)[0], True)
            self.assertEqual(seen['argv'], ['/usr/bin/xdpyinfo', '-display', ':1'])
            self.assertEqual((seen['env']['DISPLAY'], seen['env']['XAUTHORITY'], seen['timeout']), (':1', str(xa), 10))
            rc = 1
            self.assertEqual(R.display_reachable(env, run)[0], False)

            def raises(exc):
                def run(argv, **kw):
                    raise exc
                return run
            self.assertFalse(R.display_reachable(env, raises(OSError('no xdpyinfo')))[0])
            self.assertFalse(R.display_reachable(env, raises(R.subprocess.TimeoutExpired('x', 10)))[0])

    def test_prereg_quotes_scope_and_names_xauthority(self):
        text = (R.ROOT / R.PREREG).read_text()
        self.assertIn(R.SCOPE, text)                                     # review r2 LOW: SCOPE identical
        self.assertIn('XAUTHORITY', text)
        self.assertIn('4GiB launch free', R.SCOPE)
        self.assertEqual(R.LAUNCH_FREE_BYTES, 4 << 30)
        self.assertIn('wait_heartbeat NOT_RECORDED', R.SCOPE)

    def test_long_boot_600_steps_save_extract_and_disk_accounting(self):
        """Review r2 MINOR-5: the 600-step path (first live in attempt01) on CPU: persistence of 601 packets and
        600 steps, the checked ULog path with a stub extractor, timeline brackets, byte accounting, the cap
        refusal mid-persist, member checks and the fixed step 599."""
        made = []

        class Tracking(R.Budget):
            def __init__(self, root):
                super().__init__(root)
                made.append(self)
        item = {'contact': False}
        with tempfile.TemporaryDirectory() as d:
            d = pathlib.Path(d)
            with mock.patch.object(R, 'Budget', Tracking):
                a = build_member(d / 'a', d, item, spec_for(1), n=600)
            self.assertEqual(a['recording_problems'], [])
            self.assertEqual((a['repeatability']['packets'], a['repeatability']['steps']), (601, 600))
            for name in R.NATIVE_BOUND:
                self.assertIn(name, a['repeatability']['files'])
            writers = [b for b in made if b.written]
            self.assertEqual(len(writers), 1)
            rep = d / 'a/repeatability'
            self.assertEqual(writers[0].written, sum(f.stat().st_size for f in rep.iterdir()))
            self.assertEqual(len(list(rep.glob('packet*.npz'))), 601)
            timeline = json.loads((rep / 'timeline.json').read_text())
            self.assertEqual(len(timeline), 600)
            self.assertEqual(timeline[599]['controller_setpoint']['status'], 'RECORDED')
            self.assertAlmostEqual(timeline[599]['controller_setpoint']['samples'][0]['sim_s'], 69.9)
            self.assertEqual(R.member_problems(d / 'a', dict(a, index=0)), [])
            b = build_member(d / 'b', d, item, spec_for(1), n=600, obs_delta=(300, 0.5))
            cmp = R.compare_members(d / 'a', d / 'b')
            ob = cmp['stages']['observation']
            self.assertEqual(sorted(ob['max_abs_at_fixed_steps'], key=int), [str(k) for k in R.FIXED_STEPS])
            self.assertEqual((ob['first_exact_k'], ob['max_abs_at_fixed_steps']['200'],
                              ob['max_abs_at_fixed_steps']['599']), (300, 0.0, 0.5))
            self.assertIn('599', R.trajectory_spread(d / 'a', d / 'b')['truth_pos_dist_at_fixed_steps_m'])
            # the output cap refuses mid-persist; the physical outcome and the row are kept
            with mock.patch.object(R, 'MAX_OUTPUT_BYTES', R.output_size(d) + 400_000):
                capped = build_member(d / 'c', d, item, spec_for(1), n=600)
            self.assertEqual((capped['outcome'], capped['physical_outcome_before_recording_check']),
                             ('HARNESS_INVALID', 'timeout'))
            self.assertTrue(any(x.startswith('RECORDING_WRITE') and 'OUTPUT_CAP' in x
                                for x in capped['recording_problems']), capped['recording_problems'])
            self.assertLess(R.output_size(d / 'c'), 400_000)


if __name__ == '__main__':
    unittest.main()
