"""S4R-1: additive, phase-aligned Gazebo sensor repeatability recorder. NOT_RUN.

S4F1/S4 control functions are imported into private module instances. The historical
results remain bound to their recorded source. Initial-target readiness is a newer,
CPU-qualified implementation; live launch is blocked pending a separate execution
amendment and fresh smoke. --smoke/--launch also require scoped approval.
Design review r1 (AMEND_FIRST) repairs: ledger arm = condition (BLOCKER-1); relative stamps,
per-stage max-abs, tie rule and trajectory spread (MAJOR-1); no tree walk in the control loop,
incremental byte counter (MAJOR-2); no RLIMIT_AS, owned-session RSS watchdog, fixed
instrumentation smoke (MAJOR-3); exact launch argv (MAJOR-4).
Design review r2 (AMEND_FIRST) repairs: max-abs over entries finite in both boots plus a separate non-finite
mismatch count, incomparable only for a shape mismatch or missing data (MAJOR-1); a fixed question-2 statistic and
within range (MINOR-1); estimated-position summaries (MINOR-2); reference-free frame ages and sim_now increments,
stamp microseconds and target counts reported apart (MINOR-3); DISPLAY with a readable XAUTHORITY and a reachable X
display (MINOR-4); 4 GiB launch floor, aligned SCOPE, all native files required (LOW).
Amendment 1 (AMENDMENT, 2026-11-10, after smoke01_contact_on_env0 ABORTED): bytes-like values (pymavlink BAD_DATA and
UNKNOWN_<id> carry a bytearray) are recorded losslessly as hex; the contact-log continuity rule counts real missing,
duplicate and off-grid simulator steps on the sorted arrivals and tolerates subscriber arrival reordering of at most
CONTACT_LATE_MAX_STEPS steps for at most CONTACT_LATE_MAX_FRACTION of the window's messages (counts kept descriptive);
--smoke and --launch exit non-zero unless COMPLETE (and, for --smoke, the attempt's smoke gate passes); new smoke
folder smoke02_contact_on_env0; values kept as a tagged repr are counted (unserializable_values) and a nonzero count is
a recorded warning (recording_warnings).
Amendment 2: a new smoke03_contact_on_env0 and full attempt02 after attempt01 stopped on LATE 0.168s. The 0.15s
validity bound and first-invalid stop rule remain in force; prior attempt01 stays INCOMPLETE_INVALID.

  tools/sitl_s4r1.py --schedule | --smoke | --launch | --analyze [DIR] | --compare A B
"""
import argparse
import collections
import contextlib
import gzip
import hashlib
import importlib.util
import json
import itertools
import math
import os
import pathlib
import resource
import struct
import subprocess
import sys
import time
import types

import numpy as np
import sitl_s3b_formal as SF
import sitl_s3c as S3C
import sitl_s4f1 as F1
import sitl_s4_gazebo as G
import sitl_agq_geometry_diag as AGQ

ROOT = SF.ROOT
PROTOCOL = 's4r1-v1'
PREREG = 'docs/plans/prereg_sitl_s4r1_v1_2026-11-10.md'
AMENDMENT1 = 'docs/plans/prereg_sitl_s4r1_v1_amendment1_2026-11-10.md'
AMENDMENT = 'docs/plans/prereg_sitl_s4r1_v1_amendment2_2026-11-10.md'
UNIT_DIR = pathlib.Path.home() / 'motar_tmp/s4r1_20261110'
ATTEMPT_DIR = UNIT_DIR / 'attempt02'
SMOKE_DIR = UNIT_DIR / 'smoke03_contact_on_env0'
PYTHON = '/home/fair/miniconda3/envs/aerialgym/bin/python'
SCRIPT = 'tools/sitl_s4r1.py'
DISPLAY = ':1'
GAZEBO_SEED = 4501
MAX_OUTPUT_BYTES = 3 << 30
LAUNCH_FREE_BYTES = 4 << 30          # review r2 LOW: 4 GiB - 2.7 GB worst case leaves > 1 GiB
MIN_FREE_BYTES = 1 << 30
EPISODE_WALL_S = 180
OUTER_WALL_S = 1800
SMOKE_OUTER_WALL_S = 600
KILL_AFTER_S = 60
RSS_CAP_BYTES = 8 << 30          # owned-session RSS watchdog (all owned processes + this runner); no RLIMIT_AS
RESOURCE_CHECK_EVERY_S = 1.0     # in-flight: statvfs only, at most once per second; tree walks only at boot boundaries
CONTACT_FRESH_S = AGQ.CONTROL_FRESH_S
# Amendment 1 contact-log continuity (set after smoke01, which measured 0 missing steps, 3 late arrivals of 1,876 window
# messages, at most 2 steps late): the late bound equals the pre-existing contact-interval join width and is set at the
# observed maximum (no margin). A late message can still open a new interval in AGQ's descriptive bar_events.
CONTACT_LATE_MAX_STEPS = AGQ.INTERVAL_JOIN_NS // AGQ.STEP_NS      # 2 physics steps (8 ms simulator time)
CONTACT_LATE_MAX_FRACTION = 0.01
CONTACT_SCOPE_CLAUSE = ('contact log: no missing, duplicate or off-grid simulator step in the reading window, '
                        'arrivals at most 2 steps late and at most 1% late; ')
SCOPE_V1_SMOKE, SCOPE_SMOKE = 'smoke01_contact_on_env0', 'smoke03_contact_on_env0'
SCOPE = ('S4R-1 v1 amendment2: one fixed instrumentation smoke smoke03_contact_on_env0 (env0, contact subscriber ON, '
         'not a result, 600s outer, a short boot) then attempt02: 12 fresh Gazebo/PX4 boots, G8 r2 first two eligible '
         '70-bar scenes env0/env1, 3 repeats per scene with world-contact subscriber OFF/ON, fixed alternating order; '
         'S4F1 lit_rendered ground arm, Frozen Policy F, navrl_quad_s4, body-fixed 87deg '
         'RGB/depth/gpu_ray, pinned historical bridge, explicit new Gazebo seed4501; '
         'MAVLink recv_match arrivals and sends recorded (wait_heartbeat NOT_RECORDED), all delivered raw frames and '
         'actor observations recorded; ' + CONTACT_SCOPE_CLAUSE +
         'no retries, stop on first operational/recording invalid member, 180s per boot, 1800s outer; '
         'owned-session RSS cap 8GiB, no RLIMIT_AS; DISPLAY :1 with a readable inherited XAUTHORITY; '
         '3GiB maximum output, 4GiB launch free and 1GiB minimum free disk; '
         'descriptive only: first recorded divergence per stage and within/between-condition spread, no gate')
OWN_SOURCES = ('tools/sitl_s4r1.py', 'tests/test_sitl_s4r1.py', PREREG, AMENDMENT1, AMENDMENT) + F1.OWN_SOURCES + (
    'tools/sitl_agq_geometry_diag.py',)
OFF_CONTACT = {'status': 'NOT_RECORDED', 'reason': 'subscriber OFF'}


def namespace(m, **overrides):
    return types.SimpleNamespace(**dict(dict(vars(m)), **overrides))


def private_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


JSON_SCALARS = (str, int, float, bool, type(None))


def jsonable(v):
    """Amendment 1: bytes, bytearray and memoryview -> {'bytes_hex': ...} (lossless); a type json cannot encode is kept
    as an explicitly tagged repr instead of failing the whole recording."""
    if isinstance(v, (bytes, bytearray, memoryview)):
        return {'bytes_hex': (v.tobytes() if isinstance(v, memoryview) else bytes(v)).hex()}
    if isinstance(v, float) and not math.isfinite(v):
        return {'nonfinite': str(v)}
    if isinstance(v, np.ndarray):
        return jsonable(v.tolist())
    if isinstance(v, np.generic):
        return jsonable(v.item())
    if isinstance(v, dict):
        return {str(k): jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [jsonable(x) for x in v]
    if isinstance(v, pathlib.PurePath):
        return str(v)
    if isinstance(v, JSON_SCALARS):
        return v
    return {'unserializable_type': '%s.%s' % (type(v).__module__, type(v).__qualname__), 'repr': repr(v)}


UNSERIALIZABLE_KEYS = frozenset(('unserializable_type', 'repr'))


def tagged_types(v):
    """Amendment 1 (review r1 MINOR-2): the type names of every value jsonable kept as a tagged repr, in an already
    jsonable value (counted, so that a silent loss is visible as a recorded warning)."""
    if isinstance(v, dict):
        if set(v) == UNSERIALIZABLE_KEYS:
            return [v['unserializable_type']]
        return [t for x in v.values() if isinstance(x, (dict, list)) for t in tagged_types(x)]
    if isinstance(v, list):
        return [t for x in v if isinstance(x, (dict, list)) for t in tagged_types(x)]
    return []


def array(v):
    if hasattr(v, 'detach'):
        v = v.detach().cpu().numpy()
    return np.array(v, copy=True)


def array_signature(a):
    a = np.asarray(a)
    return {'shape': list(a.shape), 'dtype': a.dtype.str,
            'sha256': hashlib.sha256(a.tobytes(order='C')).hexdigest(), 'bytes': a.nbytes}


def schedule():
    envs = F1.scene_envs()[:2]
    if envs != [0, 1]:
        raise ValueError('first eligible scenes changed: %r' % envs)
    out = []
    for rep in range(3):
        for scene, env in enumerate(envs):
            order = (False, True) if (rep + scene) % 2 == 0 else (True, False)
            for contact in order:
                out.append({'arm': 'contact_on' if contact else 'contact_off', 'bars': 70,
                            'env': env, 'repeat': rep, 'contact': contact, 'cell': 'lit_rendered'})
    return out


def smoke_schedule():
    """The fixed instrumentation smoke: env0's first contact-ON item of the schedule (not a result)."""
    items = [s for s in schedule() if s['env'] == 0 and s['contact']]
    return [dict(items[0], smoke=True)]


# ------------------------------------------------------------------------------------------------ resources
def output_size(path):
    """Full walk of an output tree: only at boot boundaries (MAJOR-2), never inside the control loop."""
    p = pathlib.Path(path)
    return sum(f.stat().st_size for f in p.rglob('*') if f.is_file()) if p.exists() else 0


def disk_free(path):
    p = pathlib.Path(path)
    while not p.exists():
        p = p.parent
    st = os.statvfs(str(p))
    return st.f_bavail * st.f_frsize


def resource_problems(path=ATTEMPT_DIR):
    """Boot-boundary check: free disk and the full-tree output size."""
    problems = []
    if disk_free(path) < MIN_FREE_BYTES:
        problems.append('DISK_LOW')
    if output_size(path) >= MAX_OUTPUT_BYTES:
        problems.append('OUTPUT_CAP')
    return problems


def cheap_resource_problems(path=ATTEMPT_DIR):
    """In-flight check: one statvfs call, no tree walk."""
    return ['DISK_LOW'] if disk_free(path) < MIN_FREE_BYTES else []


class Budget:
    """Output budget: one tree walk when created (a boot boundary), then an incremental counter updated by this
    recorder's own writes, plus statvfs for free disk."""
    def __init__(self, root):
        self.root = pathlib.Path(root)
        self.base = output_size(self.root)
        self.written = 0

    def check(self, amount):
        if self.base + self.written + amount > MAX_OUTPUT_BYTES:
            raise RuntimeError('OUTPUT_CAP before write')
        if disk_free(self.root) - amount < MIN_FREE_BYTES:
            raise RuntimeError('DISK_LOW before write')

    def add(self, n):
        self.written += int(n)

    def write_bytes(self, path, data):
        self.check(len(data))
        with pathlib.Path(path).open('xb') as f:
            f.write(data)
        self.add(len(data))

    def savez(self, path, arrays):
        self.check(sum(a.nbytes for a in arrays.values()) + (1 << 20))
        with pathlib.Path(path).open('xb') as f:
            np.savez_compressed(f, **arrays)
        self.add(pathlib.Path(path).stat().st_size)


# ------------------------------------------------------------------------------------------------ recorder
class Tap:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.events, self.packets, self.steps = [], [], []
        self._seq = itertools.count()
        self.counts = collections.Counter()
        self.overhead = {}
        self.contact_api = private_module('_s4r1_contact', ROOT / 'tools/sitl_agq_geometry_diag.py')
        self.contact_api.VEHICLE = 'navrl_quad_s4::'
        self.phase = 'boot'
        self.link = self.raw_link = self.session = None
        self.control_start = self.control_end = None
        self.contact_control = None
        self.pending = None
        self.logger_stop = None
        self.plan = None
        self.ulog_data = None
        self.unserializable = []                 # amendment 1: tagged_types of every recorder file written
        self.contact_enabled = False
        self.out_dir = ATTEMPT_DIR
        self.next_resource_check = 0.0
        self.mark('boot')

    def event(self, kind, **fields):
        e = dict(i=next(self._seq), kind=kind, mono=self.clock(), phase=self.phase, **fields)
        self.events.append(e)
        return e

    def mark(self, phase):
        self.phase = phase
        self.event('phase', name=phase)

    def charge(self, site, t0):
        """Recorder overhead (perf_counter seconds) per proxy site: count, sum, max."""
        dt = time.perf_counter() - t0
        c = self.overhead.setdefault(site, [0, 0.0, 0.0])
        c[0] += 1
        c[1] += dt
        c[2] = max(c[2], dt)

    def link_state(self):
        """Latest receiver-thread state at a packet read: references to the immutable tuples, no events."""
        link = self.raw_link
        if link is None:
            return None
        return {n: (tuple(v) if v is not None else None) for n in ('pos', 'att', 'truth', 'hb')
                for v in (getattr(link, n, None),)}

    def control_step(self, s, rgb, depth, lidar, obs):
        t0 = time.perf_counter()
        if self.phase != 'control':
            self.mark('control')
            self.control_start = self.clock()
        k = len(self.steps)
        self.pending = dict(k=k, packet_id=len(self.packets) - 1, mono=self.clock(),
                            state={key: array(v) for key, v in s.items() if v is not None},
                            rgb=array(rgb), depth=array(depth), lidar=array(lidar), obs=array(obs))
        self.steps.append(self.pending)
        self.event('chosen_frame', k=k, packet_id=self.pending['packet_id'])
        self.charge('control_step', t0)

    def post_control(self):
        if self.phase == 'control':
            self.control_end = self.clock()
            self.mark('post_terminal_landing')


class MavProxy:
    def __init__(self, obj, tap):
        self.obj, self.tap = obj, tap

    def __getattr__(self, name):
        fn = getattr(self.obj, name)
        if not (name.endswith('_send') and callable(fn)):
            return fn

        def send(*args, **kwargs):
            # PX4 AUTO/LAND request after the loop marks the phase boundary.
            if name == 'command_long_send' and len(args) > 6 and args[2] == 176 and args[4:7] == (1, 4, 6):
                self.tap.post_control()
            e = self.tap.event('mav_send', call=name, args=jsonable(args), kwargs=jsonable(kwargs))
            try:
                return fn(*args, **kwargs)
            except BaseException as exc:
                e['error'] = '%s: %s' % (type(exc).__name__, exc)
                raise
            finally:
                e['mono_return'] = self.tap.clock()
        return send


class ConnProxy:
    def __init__(self, obj, tap):
        self.obj, self.tap = obj, tap
        self.mav = MavProxy(obj.mav, tap)

    def __getattr__(self, name):
        return getattr(self.obj, name)

    def recv_match(self, *args, **kwargs):
        m = self.obj.recv_match(*args, **kwargs)
        if m is not None:
            t0 = time.perf_counter()
            typ = m.get_type()
            self.tap.counts[typ] += 1
            self.tap.event('mav_recv', type=typ, fields=jsonable(m.to_dict()),
                           source=[m.get_srcSystem(), m.get_srcComponent()],
                           sequence=m.get_seq())
            self.tap.charge('mav_recv', t0)
        return m


class LinkProxy:
    """Pass-through link; state reads are not logged (review r1 MAJOR-2: the 2 ms wait loop stays untouched).
    The latest state is snapshotted into each packet record instead."""
    def __init__(self, link, tap):
        object.__setattr__(self, '_link', link)
        object.__setattr__(self, '_tap', tap)

    def __getattr__(self, name):
        return getattr(self._link, name)

    def __setattr__(self, name, value):
        setattr(self._link, name, value)

    def read_params(self, *args, **kwargs):
        result = self._link.read_params(*args, **kwargs)
        if self._tap.contact_enabled:
            self._tap.contact_control = contact_control(self._tap, getattr(self._link, 'pos', None))
            if not self._tap.contact_control.get('ok'):
                raise RuntimeError('HARNESS_INVALID CONTACT_CONTROL %s' % self._tap.contact_control)
        self._tap.mark('preflight_parameters_verified')
        return result


def contact_control(tap, pos):
    """ON only: the subscriber still runs, the floor-contact positive control passes, and its window is current (last
    message within CONTACT_FRESH_S of PX4's newest LOCAL_POSITION_NED time; AGQ session_control's freshness rule)."""
    try:
        tap.session.check(required=('contacts',))
    except Exception as exc:                                             # noqa: BLE001 - the control fails
        return {'ok': False, 'problem': 'contact subscriber: %s' % exc}
    ok, detail = tap.contact_api.floor_control(pathlib.Path(tap.session.episode_dir) / 'logs/contacts.log')
    now = None if pos is None else float(pos[0])
    detail = dict(detail, px4_time_s=now, fresh_s=CONTACT_FRESH_S)
    if ok and (now is None or abs(float(detail['last_t']) - now) > CONTACT_FRESH_S):
        ok, detail['problem'] = False, 'control window not current'
    return dict(detail, ok=bool(ok))


def arrival_continuity(times_ns, start_sim_s=None, final_sim_s=None, step_ns=AGQ.STEP_NS, tol_ns=AGQ.STEP_TOL_NS,
                       late_max_steps=CONTACT_LATE_MAX_STEPS, late_max_fraction=CONTACT_LATE_MAX_FRACTION):
    """Amendment 1: contact-log continuity over AGQ's reading window [start - COVER_BEFORE_S, final + COVER_AFTER_S].
    Loss is judged on the sorted distinct simulator times (a missing physics step is missing whatever the arrival
    order); duplicates and off-grid times are counted apart; arrival order is measured as late arrivals (a time below
    the largest time already received) and their lateness in steps. Returns (summary, problems)."""
    lo = None if start_sim_s is None else round((start_sim_s - AGQ.COVER_BEFORE_S) * 1e9)
    hi = None if final_sim_s is None else round((final_sim_s + AGQ.COVER_AFTER_S) * 1e9)

    def inside(t):
        return (lo is None or t >= lo) and (hi is None or t <= hi)
    out = {'rule': 'S4R-1 amendment 1', 'window_ns': [lo, hi], 'step_ns': step_ns, 'late_max_steps': late_max_steps,
           'late_max_fraction': late_max_fraction, 'messages': len(times_ns), 'messages_in_window': 0,
           'distinct_in_window': 0, 'missing_steps_in_window': 0, 'gaps_in_window': 0, 'duplicates_in_window': 0,
           'off_grid_in_window': 0, 'late_arrivals': 0, 'late_arrivals_in_window': 0, 'max_late_steps_in_window': 0,
           'late_samples': [], 'gap_samples': []}
    running = None
    for t in times_ns:
        if inside(t):
            out['messages_in_window'] += 1
        if running is not None and t < running:
            late = (running - t) / step_ns
            out['late_arrivals'] += 1
            if inside(t):
                out['late_arrivals_in_window'] += 1
                out['max_late_steps_in_window'] = max(out['max_late_steps_in_window'], late)
                if len(out['late_samples']) < 10:
                    out['late_samples'].append([t * 1e-9, running * 1e-9])
        running = t if running is None else max(running, t)
    counts = collections.Counter(times_ns)
    distinct = sorted(counts)
    out['distinct_in_window'] = sum(1 for t in distinct if inside(t))
    out['duplicates_in_window'] = sum(c - 1 for t, c in counts.items() if c > 1 and inside(t))
    for a, b in zip(distinct, distinct[1:]):
        if not ((lo is None or b >= lo) and (hi is None or a <= hi)):
            continue
        n = int(round((b - a) / step_ns))
        if n < 1 or abs((b - a) - n * step_ns) > tol_ns:
            out['off_grid_in_window'] += 1
        elif n > 1:
            out['gaps_in_window'] += 1
            out['missing_steps_in_window'] += n - 1
            if len(out['gap_samples']) < 10:
                out['gap_samples'].append([a * 1e-9, b * 1e-9])
    problems = []
    if out['missing_steps_in_window']:
        problems.append('RECORDER_LOSS: %d missing simulator steps in the reading window (%d gaps in the sorted '
                        'arrivals)' % (out['missing_steps_in_window'], out['gaps_in_window']))
    if out['duplicates_in_window']:
        problems.append('RECORDER_DUPLICATE: %d repeated simulator times in the reading window'
                        % out['duplicates_in_window'])
    if out['off_grid_in_window']:
        problems.append('RECORDER_GRID: %d simulator-time intervals off the %d ns step in the reading window'
                        % (out['off_grid_in_window'], step_ns))
    if out['max_late_steps_in_window'] * step_ns > late_max_steps * step_ns + tol_ns:
        problems.append('RECORDER_ORDER: an arrival %.3g steps late in the reading window (tolerance %d)'
                        % (out['max_late_steps_in_window'], late_max_steps))
    if out['late_arrivals_in_window'] > late_max_fraction * out['messages_in_window']:
        problems.append('RECORDER_ORDER: %d late arrivals of %d messages in the reading window (tolerance %g)'
                        % (out['late_arrivals_in_window'], out['messages_in_window'], late_max_fraction))
    return out, problems


def _agq_continuity_problem(x):
    """AGQ's arrival-order continuity messages, which count a reordered arrival as both a gap and an order fault."""
    return x.startswith('RECORDER_ORDER:') or (
        x.startswith('RECORDER_LOSS:') and 'simulator-time gaps in the reading window' in x)


def audit_contacts(api, path, start_sim_s, final_sim_s):
    """AGQ's full-log audit (coverage, floor and bar events, message counts unchanged), with its arrival-order continuity
    problems replaced by arrival_continuity (amendment 1). The replaced AGQ messages stay in the summary."""
    summary, issues = api.contact_audit(path, start_sim_s=start_sim_s, final_sim_s=final_sim_s)
    times = []
    if pathlib.Path(path).is_file():
        for line in AGQ._lines(path)[0]:
            if not line.strip():
                continue
            try:
                times.append(AGQ.contacts_message(line)['t_ns'])
            except AGQ.ParseError:
                continue                                                 # AGQ counts these as other_lines
    continuity, own = arrival_continuity(times, start_sim_s, final_sim_s)
    replaced = [x for x in issues if _agq_continuity_problem(x)]
    summary = dict(summary, continuity_amendment1=continuity, agq_continuity_problems_replaced=replaced)
    return summary, [x for x in issues if not _agq_continuity_problem(x)] + own


def link_factory(tap, base=SF.ParamLink):
    def make():
        link = base()
        link.m = ConnProxy(link.m, tap)
        tap.raw_link = link
        tap.link = LinkProxy(link, tap)
        tap.mark('initialization_takeoff_hover')
        return tap.link
    return make


def client_factory(tap, base=G.S4Client):
    class Client:
        def __init__(self):
            self.client = base()
            self.initial_read = False

        @contextlib.contextmanager
        def bounded_io(self, *args, **kwargs):
            self.initial_read = True
            tap.event('initial_target_wait_begin')
            try:
                with self.client.bounded_io(*args, **kwargs):
                    yield
            finally:
                self.initial_read = False

        def record_initial_target_readiness(self, report):
            tap.event('initial_target_ready', report=report)

        def set_target(self, *xyz):
            e = tap.event('target_request', xyz=list(xyz))
            try:
                ack = self.client.set_target(*xyz)
                e.update(ack_sim_s=ack, mono_return=tap.clock())
                if tap.phase == 'control' and tap.pending is not None:
                    # the loop sends step k+1's target after step k's command (sitl_s4._fly)
                    tap.pending['target_send_next'] = {'xyz': list(xyz), 'ack_sim_s': ack,
                                                       'request_mono': e['mono'], 'return_mono': e['mono_return']}
                return ack
            except BaseException as exc:
                e['error'] = str(exc)
                raise

        def frames(self):
            # no session or resource check here (review r1 MAJOR-2): the historical loop has none
            e = tap.event('frame_request', packet_id=len(tap.packets))
            f = self.client.frames()
            e['mono_return'] = tap.clock()
            t0 = time.perf_counter()
            if not self.initial_read and any(f.get(k) is None for k in ('ranges', 'rgb', 'depth')):
                raise RuntimeError('HARNESS_INVALID MISSING_SENSOR_PAYLOAD')
            # Exact payload copies stay in memory; persisted after the flight (no runtime disk work).
            p = {'packet_id': len(tap.packets), 'phase_at_read': tap.phase,
                 'purpose': 'initial_target_readiness' if self.initial_read else 'control_candidate',
                 'request_mono': e['mono'], 'return_mono': e['mono_return'],
                 'header': {k: v for k, v in f.items() if k not in ('ranges', 'rgb', 'depth')},
                 'link_state_at_return': tap.link_state(),
                 'wire_header': {'magic': 'MS4F', 'version': 1,
                                 'lidar_h_w': list(f['ranges'].shape) if f['ranges'] is not None else None,
                                 'rgb_h_w': list(f['rgb'].shape[:2]) if f['rgb'] is not None else None,
                                 'depth_h_w': list(f['depth'].shape) if f['depth'] is not None else None},
                 'raw': {k: array(f[k]) for k in ('ranges', 'rgb', 'depth') if f[k] is not None}}
            tap.packets.append(p)
            tap.charge('frames', t0)
            return f

        def close(self):
            return self.client.close()
    return Client


def episode_class(tap, B, factory=G.gazebo_episode_class):
    base = factory(B)

    class Episode(base):
        def observe_with(self, s, rgb, depth, lidar):
            result = super().observe_with(s, rgb, depth, lidar)
            tap.control_step(s, rgb, depth, lidar, result[0])
            return result

        def command(self, action, lidar):
            result = super().command(action, lidar)
            t0 = time.perf_counter()
            tap.pending.update(action=array(action), governed=array(result[0]), yaw_rate=float(result[1]),
                               prev_action=array(result[2]), command_mono=tap.clock())
            tap.charge('command', t0)
            return result
    return Episode


def policy_proxy(policy, tap):
    class Policy:
        def act(self, obs):
            e = tap.event('policy_call', k=len(tap.steps) - 1)
            result = policy.act(obs)
            e['mono_return'] = tap.clock()
            tap.pending['actor_output'] = array(result)
            return result
    return Policy()


def session_module(OS, tap, contact, out_dir=ATTEMPT_DIR):
    tap.contact_enabled = contact
    tap.out_dir = out_dir

    class Session(OS.OwnedSession):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **dict(kwargs, deadline_s=EPISODE_WALL_S, rss_cap_bytes=RSS_CAP_BYTES))
            tap.session = self

        def __exit__(self, exc_type, exc, tb):
            error = None
            try:
                if tap.plan is not None and 'px4' in self._procs:
                    tap.logger_stop = stop_owned_logger(self, tap.plan)
            except Exception as problem:
                tap.logger_stop = {'error': str(problem)}
                error = problem
            result = super().__exit__(exc_type, exc, tb)
            if error is not None:
                raise RuntimeError('HARNESS_INVALID LOGGER_STOP: %s' % error)
            return result

        def check(self, required=()):
            """One owned-session check (the contact subscriber added when ON), then at most once per second a
            statvfs free-disk check; no tree walk (review r1 MAJOR-2)."""
            req = tuple(required)
            if contact and 'contacts' in getattr(self, '_procs', {}) and 'contacts' not in req:
                req += ('contacts',)
            result = super().check(required=req)
            now = time.monotonic()
            if now >= tap.next_resource_check:
                tap.next_resource_check = now + RESOURCE_CHECK_EVERY_S
                problems = cheap_resource_problems(out_dir)
                if problems:
                    raise RuntimeError('HARNESS_INVALID ' + ','.join(problems))
            return result

    def plan(*args, **kwargs):
        p = OS.direct_launch_plan(*args, **kwargs)
        p['gzserver']['argv'] += ['--seed', str(GAZEBO_SEED)]
        tap.plan = p
        tap.event('launch_plan', seed=GAZEBO_SEED, gzserver_argv=p['gzserver']['argv'],
                  spawn_argv=p['spawn']['argv'], px4_argv=p['px4']['argv'])
        return p

    def launch(session, plan):
        result = OS.launch_px4_gazebo(session, plan)
        if contact:
            session.start('contacts', list(AGQ.CONTACT_ARGV), env=plan['env'], cwd=session.episode_dir)
            tap.event('contact_subscriber_started', argv=list(AGQ.CONTACT_ARGV))
        return result
    return namespace(OS, OwnedSession=Session, direct_launch_plan=plan, launch_px4_gazebo=launch)


def write_json(path, value, budget=None):
    """Returns tagged_types of what it wrote (amendment 1)."""
    value = jsonable(value)
    data = (json.dumps(value, allow_nan=False, indent=1) + '\n').encode()
    if budget is not None:
        budget.write_bytes(path, data)
    else:
        with pathlib.Path(path).open('xb') as f:
            f.write(data)
    return tagged_types(value)


def dump_tap(tap, ep_dir, budget):
    """Persist the in-memory recording after the flight; every write goes through the byte counter."""
    if not isinstance(budget, Budget):
        budget = Budget(budget)
    d = pathlib.Path(ep_dir) / 'repeatability'
    d.mkdir()
    packets, steps = [], []
    for p in tap.packets:
        filename = 'packet%04d.npz' % p['packet_id']
        raw = p['raw']
        budget.savez(d / filename, raw)
        packets.append(dict({k: v for k, v in p.items() if k != 'raw'},
                            file=filename, signatures={k: array_signature(v) for k, v in raw.items()}))
    for s in tap.steps:
        filename = 'step%04d.npz' % s['k']
        raw = {k: v for k, v in s.items() if isinstance(v, np.ndarray)}
        raw.update({'state_' + k: v for k, v in s['state'].items()})
        budget.savez(d / filename, raw)
        steps.append(dict({k: v for k, v in s.items() if k != 'state' and not isinstance(v, np.ndarray)},
                          file=filename, signatures={k: array_signature(v) for k, v in raw.items()}))
    events = [jsonable(e) for e in list(tap.events)]
    tagged = tagged_types(events)
    event_text = ''.join(json.dumps(e, allow_nan=False) + '\n' for e in events)
    budget.write_bytes(d / 'events.jsonl.gz', gzip.compress(event_text.encode(), compresslevel=1, mtime=0))
    tagged += write_json(d / 'packets.json', packets, budget)
    tagged += write_json(d / 'steps.json', steps, budget)
    tagged += write_json(d / 'summary.json', {
        'protocol': PROTOCOL, 'counts': dict(tap.counts),
        'control_start_mono': tap.control_start, 'control_end_mono': tap.control_end,
        'contact_control': tap.contact_control, 'logger_stop': tap.logger_stop,
        'recorder_overhead_s': {k: {'calls': v[0], 'sum': v[1], 'max': v[2]} for k, v in tap.overhead.items()},
        'sensor_production_counter': 'NOT_RECORDED',
        'frame_identity': 'source timestamp + exact payload SHA256; packet_id is client receipt index',
        'packet_state': 'link_state_at_return: latest receiver-thread pos/att/truth/hb tuples at packet return',
        'accepted_setpoint': 'controller setpoint topic, posthoc ULog reconstruction; no per-packet acceptance ACK',
        'mavlink_not_recorded': 'messages consumed inside pymavlink wait_heartbeat are not logged'}, budget)
    manifest = {str(p.relative_to(ep_dir)): {'bytes': p.stat().st_size, 'sha256': SF.sha256_file(p)}
                for p in sorted(d.iterdir()) if p.is_file()}
    tap.unserializable = list(getattr(tap, 'unserializable', [])) + tagged
    preflight = sum(p.get('purpose') == 'initial_target_readiness' for p in packets)
    return {'files': manifest, 'packets': len(packets), 'steps': len(steps),
            'packet_contract': 'initial-target-readiness-v1' if preflight else 'historical-single-hover',
            'preflight_packets': preflight if preflight else 1}


def packet_mapping_problems(packets, steps, n):
    """Validate receipt IDs and exact chosen-frame mapping; keep legacy records readable.

    New startup polls are a contiguous explicit prefix, never discarded or counted
    as policy steps. Historical records retain their single hover-packet contract.
    """
    if not isinstance(n, int) or n <= 0 or len(steps) != n:
        return ['RECORDING_COUNT']
    if any(p.get('packet_id') != i for i, p in enumerate(packets)):
        return ['PACKET_MAPPING']
    ids = [s.get('packet_id') for s in steps]
    preflight = sum(p.get('purpose') == 'initial_target_readiness' for p in packets)
    if preflight:
        if preflight < 2 or any(p.get('purpose') != 'initial_target_readiness' for p in packets[:preflight]):
            return ['PACKET_MAPPING']
        if any(p.get('purpose') != 'control_candidate' for p in packets[preflight:]):
            return ['PACKET_MAPPING']
    else:
        preflight = 1
    if len(packets) != n + preflight:
        return ['RECORDING_COUNT']
    if ids != list(range(preflight, len(packets))) or [s.get('k') for s in steps] != list(range(n)):
        return ['PACKET_MAPPING']
    return []


def recording_problems(tap, result, contact):
    p = []
    n = result.get('steps', 0)
    for typ in ('LOCAL_POSITION_NED', 'ATTITUDE', 'HIL_STATE_QUATERNION'):
        if tap.counts[typ] <= 0:
            p.append('MAVLINK_STREAM_MISSING: ' + typ)
    if result.get('ulog', {}).get('all_verified') is not True:
        p.append('ULOG_INTEGRITY: ' + str(result.get('ulog', {}).get('problems')))
    p += packet_mapping_problems(tap.packets, tap.steps, n)
    if tap.control_start is None or tap.control_end is None:
        p.append('CONTROL_PHASE_MISSING')
    for s in tap.steps:
        if not {'obs', 'action', 'actor_output', 'governed', 'prev_action'} <= s.keys():
            p.append('ACTOR_RECORD_MISSING')
            break
    if contact and not (tap.contact_control or {}).get('ok'):
        p.append('CONTACT_CONTROL')
    if not contact and tap.contact_control != OFF_CONTACT:
        p.append('OFF_CONTACT_SEMANTICS')
    if (tap.logger_stop or {}).get('returncode') != 0:
        p.append('LOGGER_STOP: %s' % (tap.logger_stop,))
    return p


def stop_owned_logger(session, plan):
    """Owned logger stop+join before teardown, shared design with S5 safety preparation."""
    session.check(required=('px4', 'gzserver'))
    rootfs = pathlib.Path(session.episode_dir) / 'rootfs'
    if pathlib.Path(plan['px4']['cwd']) != rootfs:
        raise RuntimeError('LOGGER_STOP_ROOTFS_MISMATCH')
    binary = pathlib.Path(plan['px4']['argv'][0])
    expected = pathlib.Path('/home/fair/workspaces/px4_ws/PX4-Autopilot/build/px4_sitl_default/bin/px4')
    if plan['px4']['argv'] != [str(expected), '-d', str(expected.parent.parent / 'etc')]:
        raise RuntimeError('LOGGER_STOP_INSTANCE_MISMATCH')
    owned = session._procs.get('px4', {}).get('record', {})
    if owned.get('argv') != plan['px4']['argv'] or pathlib.Path(owned.get('cwd', '/')) != rootfs:
        raise RuntimeError('LOGGER_STOP_OWNERSHIP_MISMATCH')
    alias = binary.with_name('px4-logger')
    if not alias.is_file() or alias.resolve() != binary.resolve():
        raise RuntimeError('LOGGER_STOP_BINARY_MISMATCH')
    t0 = time.monotonic()
    session.start('logger_stop', [str(alias), 'stop'], cwd=rootfs, env=plan['px4']['env'])
    rc = session.wait('logger_stop', timeout=15.)
    out = {'returncode': rc, 'wall_s': time.monotonic()-t0, 'argv': [str(alias), 'stop'],
           'binary_sha256': SF.sha256_file(alias), 'completed_before_px4_teardown': True}
    if rc != 0:
        raise RuntimeError('LOGGER_STOP_FAILED %s' % out)
    return out


ULOG_TOPICS = ('vehicle_attitude_groundtruth', 'vehicle_local_position_groundtruth',
               'vehicle_attitude', 'vehicle_local_position', 'vehicle_local_position_setpoint',
               'trajectory_setpoint', 'vehicle_command', 'vehicle_command_ack', 'vehicle_status')
ULOG_REQUIRED = ULOG_TOPICS[:6]
PX4_PYTHON = pathlib.Path('/home/fair/miniconda3/envs/px4/bin/python3.8')
ULOG_RECONSTRUCTION = 'ulog_reconstruction.json.gz'


def ulog_framing(path):
    """Strict EOF framing; adapted from existing-data audit, no forgiving pyulog truncation."""
    path = pathlib.Path(path)
    op = gzip.open if path.suffix == '.gz' else open
    topics, spans, subs, errors = collections.Counter(), {}, {}, []
    with op(path, 'rb') as f:
        header = f.read(16)
        if len(header) != 16 or header[:7] != b'ULog\x01\x12\x35':
            return {'problems': ['ULOG_HEADER'], 'topics': {}, 'spans_s': {}}
        while True:
            h = f.read(3)
            if not h:
                break
            if len(h) != 3:
                errors.append('ULOG_TRUNCATED_HEADER')
                break
            n, typ = struct.unpack('<HB', h)
            b = f.read(n)
            if len(b) != n:
                errors.append('ULOG_TRUNCATED_PAYLOAD')
                break
            if typ == ord('A'):
                if n < 4:
                    errors.append('ULOG_BAD_SUBSCRIPTION')
                    break
                _, mid = struct.unpack('<BH', b[:3])
                subs[mid] = b[3:].decode('utf8', 'replace')
            elif typ == ord('D'):
                if n < 10:
                    errors.append('ULOG_BAD_DATA')
                    break
                mid = struct.unpack('<H', b[:2])[0]
                name = subs.get(mid)
                if name is None:
                    errors.append('ULOG_UNKNOWN_SUBSCRIPTION')
                    break
                stamp = struct.unpack('<Q', b[2:10])[0] * 1e-6
                topics[name] += 1
                spans.setdefault(name, [stamp, stamp])[1] = stamp
    return {'problems': errors, 'topics': dict(topics), 'spans_s': spans}


def extract_ulog(path, output):
    """Runs under the PX4 env's Python (pyulog). Writes gzip-compressed JSON (mtime 0)."""
    from pyulog import ULog
    ulog = ULog(str(path), list(ULOG_TOPICS))
    if ulog.file_corruption:
        raise ValueError('ULOG_CORRUPTION reported by pyulog')
    data = {}
    for ds in ulog.data_list:
        if ds.multi_id == 0:
            data[ds.name] = {name: values.tolist() for name, values in ds.data.items()}
    text = json.dumps(jsonable({'topics': data, 'dropouts': [dict(timestamp=x.timestamp, duration=x.duration)
                                                              for x in ulog.dropouts]}), allow_nan=False)
    with open(output, 'xb') as f:
        f.write(gzip.compress(text.encode(), compresslevel=1, mtime=0))
    return data


def checked_gzip(ep_dir, tap, run=subprocess.run):
    """Replaces F1.gzip_ulogs. Never raises (review r1 MINOR-5): any failure is returned as a ULog problem, which
    marks the boot recording-invalid while the physical record (F1/S4's result, steps.npy, native ULog) is kept."""
    ep_dir = pathlib.Path(ep_dir)
    files = sorted(ep_dir.glob(F1.ULOG_GLOB))
    native = {'native_paths': [str(f) for f in files]}
    try:
        if len(files) != 1:
            return dict(native, count=len(files), all_verified=False, problems=['ULOG_COUNT: %d' % len(files)])
        report = ulog_framing(files[0])
        issues = list(report['problems'])
        for topic in ULOG_REQUIRED:
            if report['topics'].get(topic, 0) < 2:
                issues.append('ULOG_TOPIC: ' + topic)
        legacy = np.load(ep_dir / 'steps.npy')
        if len(legacy):
            start, end = float(legacy[0, 1]), float(legacy[-1, 1])
            for topic in ULOG_REQUIRED:
                span = report['spans_s'].get(topic, [float('inf'), -float('inf')])
                if span[0] > start or span[1] < end - .2:
                    issues.append('ULOG_CONTROL_COVERAGE: ' + topic)
            report['control_sim_s'] = [start, end]
        report['problems'] = issues
        write_json(ep_dir / 'ulog_integrity.json', report)
        if issues:
            # Native partial log remains untouched; no gzip or retry.
            return dict(native, count=len(files), all_verified=False, problems=issues,
                        native_sha256=SF.sha256_file(files[0]))
        output = ep_dir / ULOG_RECONSTRUCTION
        Budget(tap.out_dir).check(files[0].stat().st_size)          # post-flight: one walk, outside the loop
        env = dict(os.environ, CUDA_VISIBLE_DEVICES='', PYTHONNOUSERSITE='1', OMP_NUM_THREADS='1',
                   MKL_NUM_THREADS='1')
        proc = run([str(PX4_PYTHON), '-B', str(pathlib.Path(__file__).resolve()),
                    '--extract-ulog', str(files[0]), str(output)],
                   env=env, capture_output=True, text=True, timeout=60)
        if proc.returncode:
            return dict(native, count=len(files), all_verified=False,
                        problems=['ULOG_PARSE: ' + proc.stderr[-1000:]], native_sha256=SF.sha256_file(files[0]))
        with gzip.open(output, 'rb') as f:
            tap.ulog_data = json.loads(f.read().decode())
        return F1.gzip_ulogs(ep_dir)
    except Exception as exc:                                             # noqa: BLE001 - recorded, not raised
        return dict(native, count=len(files), all_verified=False,
                    problems=['ULOG_CHECK_ERROR: %s: %s' % (type(exc).__name__, exc)])


def nearest_bracket(topic, sim_t):
    """Expose the alignment interval; do not equate controller samples with transport ACKs."""
    if not topic or 'timestamp' not in topic:
        return {'status': 'NOT_RECORDED'}
    stamps = np.asarray(topic['timestamp'], dtype=np.float64) * 1e-6
    j = int(np.searchsorted(stamps, sim_t, side='right'))
    indices = sorted(set(i for i in (j-1, j) if 0 <= i < len(stamps)))
    return {'status': 'RECORDED', 'query_sim_s': sim_t, 'clock_basis': 'PX4 simulation time',
            'samples': [dict(sim_s=float(stamps[i]), fields={k: v[i] for k, v in topic.items()}) for i in indices],
            'alignment_uncertainty_s': float(max((abs(stamps[i]-sim_t) for i in indices), default=0.))}


STAMP_FIELDS = ('sim_now', 'lidar_t', 'rgb_t', 'depth_t', 'target_t')


def canonical_timeline(tap, ep_dir):
    legacy = np.load(pathlib.Path(ep_dir) / 'steps.npy')
    topics = (tap.ulog_data or {}).get('topics', {})
    out = []
    sends = [e for e in tap.events if e['kind'] == 'mav_send' and e.get('call') == 'set_position_target_local_ned_send'
             and e['phase'] == 'control']
    t0 = tap.packets[tap.steps[0]['packet_id']]['header']['sim_now'] if tap.steps else None
    contact = ('NOT_RECORDED per step: whole-log contact audit in contacts.json' if tap.contact_enabled
               else 'NOT_RECORDED: subscriber OFF')
    for s, old in zip(tap.steps, legacy):
        p = tap.packets[s['packet_id']]
        sim_t = float(old[1])
        out.append({'k': s['k'], 'phase': 'control', 'sim_s': sim_t, 'elapsed_sim_s': sim_t-float(legacy[0, 1]),
                    'elapsed_mono_s': s['mono']-tap.control_start, 'packet_id': s['packet_id'],
                    'sensor_header': p['header'],
                    'sensor_stamp_relative_s': {n: p['header'][n] - t0 for n in STAMP_FIELDS},
                    'sensor_age_s': {name: sim_t-p['header'][name] for name in ('lidar_t', 'rgb_t', 'depth_t')},
                    'arrival_to_observe_s': s['mono']-p['return_mono'],
                    'estimated_position': old[2:5].tolist(), 'truth_position': old[23:26].tolist(),
                    'actor_action': old[9:13].tolist(), 'governed_body_xy': old[13:15].tolist(),
                    'requested_setpoint': sends[s['k']] if s['k'] < len(sends) else {'status': 'NOT_RECORDED'},
                    'target_send_next': s.get('target_send_next', {'status': 'NOT_RECORDED'}),
                    'controller_setpoint': nearest_bracket(topics.get('vehicle_local_position_setpoint'), sim_t),
                    'trajectory_setpoint': nearest_bracket(topics.get('trajectory_setpoint'), sim_t),
                    'truth_body_attitude': nearest_bracket(topics.get('vehicle_attitude_groundtruth'), sim_t),
                    'accepted_packet_id': 'NOT_RECORDED: MAVLink velocity setpoints have no per-packet ACK',
                    'contact': contact})
    return out


def scene_binding(spec):
    keys = ('centers', 'halves', 'quats', 'p0', 'yaw0', 'tpos', 'bounds_min', 'bounds_max')
    return {k: array_signature(np.asarray(spec[k])) for k in keys}


def seed_binding(ep_dir):
    """The Gazebo seed from the owned session's receipt (the gzserver argv actually launched; review r1 MINOR-6)."""
    receipt = pathlib.Path(ep_dir) / 'session.json'
    if not receipt.is_file():
        return {'seed': None, 'problem': 'session.json missing'}
    launches = json.loads(receipt.read_text()).get('launches') or []
    argv = next((x.get('argv') for x in launches if x.get('role') == 'gzserver'), None) or []
    seed = None
    if argv.count('--seed') == 1 and argv.index('--seed') + 1 < len(argv):
        try:
            seed = int(argv[argv.index('--seed') + 1])
        except ValueError:
            seed = None
    return {'seed': seed, 'gzserver_argv': argv, 'session_sha256': SF.sha256_file(receipt)}


NATIVE_BOUND = ('steps.npy', 'sensor_steps.npy', 'session.json', 'ulog_integrity.json', ULOG_RECONSTRUCTION)


def finish_episode(tap, result, ep_dir, contact, out_dir, spec):
    """Everything after F1.run_episode: ledger identity, recording checks, persistence and the byte manifest."""
    ep_dir = pathlib.Path(ep_dir)
    tap.post_control()
    tap.mark('shutdown_complete')
    # F1.run_episode sets arm = cell; the ledger arm must be the scheduled condition (review r1 BLOCKER-1)
    result['cell'] = result.get('cell') or 'lit_rendered'
    result['arm'] = 'contact_on' if contact else 'contact_off'
    result['contact'] = contact
    if not contact:
        tap.contact_control = dict(OFF_CONTACT)
    problems = recording_problems(tap, result, contact)
    try:                                                                 # review r2 LOW: never lose the row
        result['gazebo_seed'] = seed_binding(ep_dir)
    except Exception as exc:                                             # noqa: BLE001 - recorded
        result['gazebo_seed'] = {'seed': None, 'problem': '%s: %s' % (type(exc).__name__, exc)}
    if result['gazebo_seed'].get('seed') != GAZEBO_SEED:
        problems.append('SEED_NOT_BOUND: %s' % result['gazebo_seed'].get('seed'))
    contact_audit = None
    if contact:
        try:
            steps_file = ep_dir / 'steps.npy'
            old_steps = np.load(steps_file) if steps_file.is_file() else np.zeros((0, 27))
            if len(old_steps):
                contact_audit, issues = audit_contacts(
                    tap.contact_api, ep_dir / 'logs/contacts.log', float(old_steps[0, 1]), float(old_steps[-1, 1]))
                problems.extend('CONTACT_AUDIT: ' + x for x in issues)
            else:
                problems.append('CONTACT_AUDIT: no control steps')
        except Exception as exc:                                         # noqa: BLE001 - recorded
            problems.append('CONTACT_AUDIT: %s: %s' % (type(exc).__name__, exc))
    result['contact_control'] = tap.contact_control
    result['logger_stop'] = tap.logger_stop
    try:
        ep_dir.mkdir(parents=True, exist_ok=True)
        budget = Budget(out_dir)                                         # post-flight boot boundary: one walk
        result['repeatability'] = dump_tap(tap, ep_dir, budget)
        tap.unserializable = list(getattr(tap, 'unserializable', [])) + write_json(
            ep_dir / 'repeatability/timeline.json', canonical_timeline(tap, ep_dir), budget) + write_json(
            ep_dir / 'repeatability/contacts.json', contact_audit if contact else tap.contact_control, budget)
        files = {str(x.relative_to(ep_dir)): {'bytes': x.stat().st_size, 'sha256': SF.sha256_file(x)}
                 for x in sorted((ep_dir / 'repeatability').iterdir()) if x.is_file()}
        native = NATIVE_BOUND + (('logs/contacts.log',) if contact else ())
        for name in native:
            path = ep_dir / name
            if path.is_file():
                files[name] = {'bytes': path.stat().st_size, 'sha256': SF.sha256_file(path)}
            else:
                problems.append('NATIVE_FILE_MISSING: %s' % name)            # review r2 LOW: all required
        result['repeatability']['files'] = files
    except Exception as exc:                                             # noqa: BLE001 - recorded
        problems.append('RECORDING_WRITE: %s' % exc)
    # amendment 1 (review r1 MINOR-2): values kept as a tagged repr (recorder files and the ULog extraction) are
    # counted; a nonzero count is a recorded warning, not a validity problem
    tagged = list(getattr(tap, 'unserializable', [])) + tagged_types(getattr(tap, 'ulog_data', None) or {})
    result['unserializable_values'] = {'count': len(tagged), 'types': sorted(set(tagged))}
    result['recording_warnings'] = (['UNSERIALIZABLE_VALUES %d (%s): kept as tagged repr'
                                     % (len(tagged), ', '.join(sorted(set(tagged))))] if tagged else [])
    if problems:
        result['physical_outcome_before_recording_check'] = result.get('outcome')
        result['outcome'] = 'HARNESS_INVALID'
    result['recording_problems'] = problems
    result['scene_binding'] = scene_binding(spec)
    return result


def run_episode(B, OS, V0, spec, ep_dir, policy, contact, out_dir=ATTEMPT_DIR,
                link_base=None, client_base=None):
    tap = Tap()
    tap.out_dir = out_dir
    # Private globals preserve historical functions while making recording local.
    s4 = private_module('_s4r1_s4', ROOT / 'tools/sitl_s4.py')
    f1 = private_module('_s4r1_f1', ROOT / 'tools/sitl_s4f1.py')
    s4.G = namespace(G, gazebo_episode_class=lambda b: episode_class(tap, b))
    f1.S4 = s4
    f1.gzip_ulogs = lambda d: checked_gzip(d, tap)
    os_proxy = session_module(OS, tap, contact, out_dir)
    try:
        result = f1.run_episode(B, os_proxy, V0, spec, ep_dir, policy_proxy(policy, tap), 'lit_rendered',
                                link_factory=link_factory(tap, link_base or SF.ParamLink),
                                client_factory=client_factory(tap, client_base or G.S4Client))
    except Exception as exc:                                             # noqa: BLE001 - recorded
        result = {'outcome': 'HARNESS_INVALID', 'error': str(exc), 'steps': len(tap.steps)}
    return finish_episode(tap, result, ep_dir, contact, out_dir, spec)


# ------------------------------------------------------------------------------------------------ launch
def expected_parent_argv(mode):
    wall = OUTER_WALL_S if mode == '--launch' else SMOKE_OUTER_WALL_S
    return ['timeout', '-k', str(KILL_AFTER_S), str(wall), PYTHON, '-B', SCRIPT, mode]


def launch_command(mode='--launch'):
    """The exact command the tool accepts (also printed in the prereg)."""
    return ('cd %s && CUDA_VISIBLE_DEVICES="" PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 '
            'PYTHONPATH=%s DISPLAY=%s taskset -c 6,7,12,13 nice -n 10 %s'
            % (ROOT, ROOT, DISPLAY, ' '.join(expected_parent_argv(mode))))


def _parent():
    parent = pathlib.Path('/proc') / str(os.getppid())
    return ((parent / 'comm').read_text().strip(),
            [a.decode() for a in (parent / 'cmdline').read_bytes().split(b'\0') if a])


XDPYINFO = '/usr/bin/xdpyinfo'


def display_reachable(environ, run=subprocess.run):
    """Opens DISPLAY with the inherited XAUTHORITY (xdpyinfo, 10 s), before any directory exists (review r2 MINOR-4)."""
    env = {'PATH': '/usr/bin:/bin', 'HOME': environ.get('HOME', str(pathlib.Path.home())),
           'DISPLAY': environ['DISPLAY'], 'XAUTHORITY': environ['XAUTHORITY']}
    try:
        proc = run([XDPYINFO, '-display', environ['DISPLAY']], env=env, capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.SubprocessError) as exc:
        return False, '%s: %s' % (type(exc).__name__, exc)
    return proc.returncode == 0, 'xdpyinfo returncode %s %s' % (proc.returncode, (proc.stderr or '')[-300:])


def display_problems(environ, probe=None):
    """DISPLAY must be :1, XAUTHORITY must be set to a readable file, and the display must open with them. The X probe
    runs only when the first two hold, so each deviation gives exactly one problem."""
    p = []
    if environ.get('DISPLAY') != DISPLAY:
        p.append('DISPLAY must be %s' % DISPLAY)
    xauth = environ.get('XAUTHORITY')
    if not xauth:
        p.append('XAUTHORITY must be set (inherited from the user desktop session)')
    elif not (os.path.isfile(xauth) and os.access(xauth, os.R_OK)):
        p.append('XAUTHORITY %s is not a readable file' % xauth)
    if not p:
        ok, detail = (probe or display_reachable)(environ)
        if not ok:
            p.append('X display %s does not open with XAUTHORITY %s: %s' % (DISPLAY, xauth, detail))
    return p


def runtime_problems(mode='--launch', parent=None, rlimit=None, environ=None, cwd=None, executable=None,
                     aerial_gym_origin=None, display_probe=None):
    """Refusals specific to S4R-1's process: no address-space cap (review r1 MAJOR-3: it would reach gzserver's GPU
    OpenGL mappings; memory is bounded by the owned-session RSS watchdog), the exact direct timeout parent argv, the
    absolute interpreter, the worktree cwd/PYTHONPATH/aerial_gym (MAJOR-4), and DISPLAY with a readable XAUTHORITY
    that opens the display (review r2 MINOR-4). CPU affinity, nice, threads,
    CUDA hiding and user-site are refused by SF.launch_problems (AT.environment_problems)."""
    p = []
    soft, hard = rlimit if rlimit is not None else resource.getrlimit(resource.RLIMIT_AS)
    if soft != resource.RLIM_INFINITY or hard != resource.RLIM_INFINITY:
        p.append('RLIMIT_AS must be unlimited (no ulimit -v): memory is bounded by the owned-session RSS watchdog')
    try:
        comm, args = parent if parent is not None else _parent()
        want = expected_parent_argv(mode)
        if comm != 'timeout' or not args or os.path.basename(args[0]) != 'timeout' or args[1:] != want[1:]:
            p.append('direct parent must be exactly %r, got %r %r' % (want, comm, args))
    except OSError as exc:
        p.append('cannot verify timeout parent: %s' % exc)
    environ = os.environ if environ is None else environ
    if environ.get('PYTHONPATH') != str(ROOT):
        p.append('PYTHONPATH must be exactly %s' % ROOT)
    p += display_problems(environ, display_probe)
    cwd = pathlib.Path.cwd() if cwd is None else pathlib.Path(cwd)
    if cwd.resolve() != ROOT.resolve():
        p.append('working directory must be %s' % ROOT)
    if (sys.executable if executable is None else executable) != PYTHON:
        p.append('interpreter must be %s' % PYTHON)
    if aerial_gym_origin is None:
        spec = importlib.util.find_spec('aerial_gym')
        aerial_gym_origin = spec.origin if spec is not None else None
    if aerial_gym_origin is None or ROOT.resolve() not in pathlib.Path(aerial_gym_origin).resolve().parents:
        p.append('aerial_gym resolves to %s, not inside %s' % (aerial_gym_origin, ROOT))
    return p


def member_problems(ep_dir, row):
    """Per-boot integrity used by the smoke gate and --analyze: manifest bytes, ULog gzip, contact log (ON), seed,
    logger stop, contact semantics and packet/step counts."""
    ep_dir = pathlib.Path(ep_dir)
    p = []
    idx = row.get('index')
    rep = row.get('repeatability') or {}
    manifest = rep.get('files') or {}
    if not manifest:
        p.append('member%s: recording manifest absent' % idx)
    for name, receipt in manifest.items():
        path = ep_dir / name
        if not path.is_file() or path.stat().st_size != receipt['bytes'] or SF.sha256_file(path) != receipt['sha256']:
            p.append('member%s: recording bytes changed: %s' % (idx, name))
    required = ('repeatability/steps.json', 'repeatability/packets.json') + NATIVE_BOUND
    for name in required + (('logs/contacts.log',) if row.get('contact') else ()):
        if name not in manifest:
            p.append('member%s: %s not bound' % (idx, name))
    ulog = row.get('ulog') or {}
    if ulog.get('all_verified') is not True or not ulog.get('files'):
        p.append('member%s: ULog not verified' % idx)
    for rec in ulog.get('files') or []:
        p += ['member%s: %s' % (idx, x) for x in F1.verify_ulog_record(ep_dir, rec)]
    if row.get('contact'):
        if 'logs/contacts.log' not in manifest:
            p.append('member%s: contact log not bound' % idx)
        if not (row.get('contact_control') or {}).get('ok'):
            p.append('member%s: contact control not ok' % idx)
    elif row.get('contact_control') != OFF_CONTACT:
        p.append('member%s: OFF contact semantics' % idx)
    if (row.get('gazebo_seed') or {}).get('seed') != GAZEBO_SEED:
        p.append('member%s: Gazebo seed not bound' % idx)
    if (row.get('logger_stop') or {}).get('returncode') != 0:
        p.append('member%s: logger stop not verified' % idx)
    try:
        packets = json.loads((ep_dir / 'repeatability/packets.json').read_text())
        steps = json.loads((ep_dir / 'repeatability/steps.json').read_text())
        mapping = packet_mapping_problems(packets, steps, row.get('steps'))
        counts_ok = rep.get('steps') == len(steps) and rep.get('packets') == len(packets)
    except (OSError, ValueError, TypeError, AttributeError):
        mapping, counts_ok = ['PACKET_MAPPING'], False
    if mapping or not counts_ok:
        p.append('member%s: packet/step counts' % idx)
    return p


def instrumentation_smoke_problems(smoke_dir=SMOKE_DIR):
    """attempt02 refuses unless the fixed smoke (env0, contact ON) is the expected item and passes recording,
    contact, ULog, seed and logger-stop checks (S3C.smoke_problems adds COMPLETE, VALID, HEAD and source bytes)."""
    smoke_dir = pathlib.Path(smoke_dir)
    led = smoke_dir / 'episodes.jsonl'
    if not led.is_file():
        return ['no instrumentation smoke ledger in %s' % smoke_dir]
    rows = SF.read_jsonl(led)
    if len(rows) != 1:
        return ['instrumentation smoke has %d rows, not 1' % len(rows)]
    r = rows[0]
    p = []
    if (r.get('arm'), r.get('env'), r.get('contact')) != ('contact_on', 0, True):
        p.append('instrumentation smoke is not env0 contact ON: %r' % ((r.get('arm'), r.get('env'), r.get('contact')),))
    if not r.get('valid') or r.get('recording_problems'):
        p.append('instrumentation smoke invalid: %s %s' % (r.get('invalid_reason'), r.get('recording_problems')))
    return p + ['smoke ' + x for x in member_problems(smoke_dir / ('ep%03d' % r.get('index', 0)), r)]


def launch(smoke=False):
    import sitl_dev08_attempt as AT
    sources = sorted(set(OWN_SOURCES) | set(AT.SOURCES))
    out_dir = SMOKE_DIR if smoke else ATTEMPT_DIR
    extra = runtime_problems('--smoke' if smoke else '--launch')
    extra += F1.plugin_pin_problems(F1.S4.s4_extra_problems())
    extra += F1.world_model_problems(F1.world_model_files())
    extra += F1.component_smoke_problems()
    extra += resource_problems(out_dir)
    if disk_free(UNIT_DIR.parent) < LAUNCH_FREE_BYTES:
        extra.append('LAUNCH_DISK_LOW: less than 4GiB free')
    if not smoke:
        extra += instrumentation_smoke_problems(SMOKE_DIR)

    def make_one(B, OS, V0, policy):
        refs = {}

        def one(i, item, spec):
            p = resource_problems(out_dir)                               # boot boundary: full walk
            if p:
                raise RuntimeError(','.join(p))
            snap = F1.host_snapshot()
            if snap['gpu_ok'] is not True:
                raise RuntimeError('GPU_BUSY_OR_UNCHECKED: no wait/retry')
            result = run_episode(B, OS, V0, spec, out_dir / ('ep%03d' % i), policy, item['contact'], out_dir)
            result['host_gpu_before'] = snap
            binding = dict(scene=result['scene_binding'],
                           world=(result.get('cell_world') or {}).get('world_sha256'))
            refs.setdefault(item['env'], binding)
            if refs[item['env']] != binding:
                result.update(outcome='HARNESS_INVALID', error='SCENE_INITIAL_CONDITION_BINDING_CHANGED')
            after = resource_problems(out_dir)                           # boot boundary: full walk
            if after:
                result['recording_problems'] = list(result.get('recording_problems') or []) + after
                result.setdefault('physical_outcome_before_recording_check', result.get('outcome'))
                result['outcome'] = 'HARNESS_INVALID'
            return result
        return one
    S3C.launch_unit(SCOPE, UNIT_DIR, out_dir, sources, smoke_schedule if smoke else schedule,
                    {'protocol': PROTOCOL, 'prereg': PREREG, 'amendment': AMENDMENT, 'gazebo_seed': GAZEBO_SEED,
                     'instrumentation_smoke': bool(smoke), 'launch_command': launch_command(
                         '--smoke' if smoke else '--launch'),
                     'bounds': {'max_output_bytes': MAX_OUTPUT_BYTES, 'min_free_bytes': MIN_FREE_BYTES,
                                'launch_free_bytes': LAUNCH_FREE_BYTES, 'rss_cap_bytes': RSS_CAP_BYTES,
                                'episode_wall_s': EPISODE_WALL_S,
                                'outer_wall_s': SMOKE_OUTER_WALL_S if smoke else OUTER_WALL_S},
                     'world_models': F1.world_model_files()}, make_one, extra_problems=extra, invalid_abort=0,
                    smoke_dir=None if smoke else SMOKE_DIR, smoke=smoke)
    return finished_exit_code(out_dir, smoke)


EXIT_OK, EXIT_NOT_COMPLETE, EXIT_SMOKE_GATE = 0, 3, 4


def finished_exit_code(out_dir, smoke):
    """Amendment 1: the exit status an outer chain can gate on, from the run's final.json. 0 only for COMPLETE (and, for
    --smoke, a smoke that instrumentation_smoke_problems accepts); 3 when not COMPLETE or no final record; 4 for a
    COMPLETE smoke the gate refuses. A refusal before anything exists is SystemExit('refused: ...') -> exit status 1."""
    final = pathlib.Path(out_dir) / 'final.json'
    status = json.loads(final.read_text()).get('status') if final.is_file() else None
    if status != 'COMPLETE':
        return EXIT_NOT_COMPLETE
    gate = instrumentation_smoke_problems(out_dir) if smoke else []
    if gate:
        print(json.dumps({'smoke_gate_problems': gate}), flush=True)
        return EXIT_SMOKE_GATE
    return EXIT_OK


# ------------------------------------------------------------------------------------------------ analysis
STATE_KEYS = ('state_pos', 'state_vel', 'state_vquat', 'state_render_pos', 'state_render_vquat')
# (stage, source, keys) in recorded pipeline order; 'packet' arrays come from the step's chosen packet
STAGES = (('sensor_stamp_relative', 'header', None),
          ('sensor_payload', 'packet', ('ranges', 'rgb', 'depth')),
          ('state', 'step', STATE_KEYS),
          ('perception_input', 'step', ('lidar', 'rgb', 'depth')),
          ('observation', 'step', ('obs',)),
          ('actor', 'step', ('actor_output',)),
          ('governed_request', 'step', ('governed', 'action', 'prev_action')))
STAGE_NAMES = tuple(s for s, _, _ in STAGES)
# fixed reporting tolerances (absolute, in each stage's own units; relative stamps in integer microseconds and
# target-count steps); a descriptive reading aid, not a gate
REPORT_TOL = {'sensor_stamp_relative': 0, 'sensor_payload': 1e-6, 'state': 1e-3, 'perception_input': 1e-6,
              'observation': 1e-6, 'actor': 1e-6, 'governed_request': 1e-6}
FIXED_STEPS = (0, 10, 25, 50, 100, 200, 300, 400, 500, 599)
TRUTH_POS, EST_POS, EST_VEL = slice(23, 26), slice(2, 5), slice(5, 8)


def diff_stats(a, b):
    """Same-index comparison of two equal-shape arrays (review r2 MAJOR-1): the maximum |a - b| over the entries that
    are finite in both arrays, and separately the number of entries whose non-finite pattern differs (finite against
    +-inf or NaN, +inf against -inf, inf against NaN). Equal non-finite entries (same-sign inf, NaN with NaN) are
    equal and not counted. None only if the shapes differ."""
    a, b = np.asarray(a), np.asarray(b)
    if a.shape != b.shape:
        return None
    x, y = a.astype(np.float64), b.astype(np.float64)
    fx, fy = np.isfinite(x), np.isfinite(y)
    both = fx & fy
    m = float(np.abs(x[both] - y[both]).max()) if both.any() else 0.0
    same_nonfinite = ~fx & ~fy & ((x == y) | (np.isnan(x) & np.isnan(y)))
    mismatch = (~fx | ~fy) & ~same_nonfinite
    return {'max_abs': m, 'nonfinite_mismatch_n': int(mismatch.sum())}


def max_abs(a, b):
    """Max |a - b| over entries finite in both arrays; None if the shapes differ (see diff_stats)."""
    r = diff_stats(a, b)
    return None if r is None else r['max_abs']


def stamp_vector(header, t0, count0):
    """Sensor stamps relative to the boot's own first control-phase stamp (step 0's packet sim_now), quantized to
    integer microseconds (the physics step is 4 ms; this removes only double rounding of the subtraction), and the
    target-applied counter relative to step 0's. Any step-0 offset between two boots (for example one physics step of
    jitter in serving step 0) is carried into every later step (review r2 MINOR-3); the reference-free readings below
    do not carry it."""
    return np.array([int(round((header[f] - t0) * 1e6)) for f in STAMP_FIELDS]
                    + [int(header['target_count']) - int(count0)], np.int64)


AGE_FIELDS = ('lidar_t', 'rgb_t', 'depth_t', 'target_t')


def frame_ages_us(header):
    """Reference-free per-step frame ages: sim_now minus each source stamp of the same packet, integer microseconds."""
    return np.array([int(round((header['sim_now'] - header[f]) * 1e6)) for f in AGE_FIELDS], np.int64)


def _member(ep):
    d = pathlib.Path(ep) / 'repeatability'
    return d, json.loads((d / 'steps.json').read_text()), json.loads((d / 'packets.json').read_text())


def _npz(d, name, cache):
    if name not in cache:
        cache.clear()
        with np.load(d / name) as z:
            cache[name] = {k: z[k] for k in z.files}
    return cache[name]


def tie_reading(stages):
    """Fixed tie rule: stages whose first exact difference is at the same earliest step are tied and not ordered;
    a tie of every stage, or any tie at k = 0, says nothing about order."""
    firsts = {s: v['first_exact_k'] for s, v in stages.items() if v['first_exact_k'] is not None}
    if not firsts:
        return {'status': 'NO_RECORDED_DIFFERENCE', 'k': None, 'earliest_stages': []}
    k = min(firsts.values())
    earliest = [s for s in STAGE_NAMES if firsts.get(s) == k]
    status = ('ALL_TIED' if len(earliest) == len(STAGE_NAMES) else 'TIED' if len(earliest) > 1
              else 'SINGLE_EARLIEST')
    note = 'tied stages are not ordered' if len(earliest) > 1 else 'earliest recorded difference, not a cause'
    if k == 0:
        note += '; a difference at k=0 is uninformative about order within the control phase'
    return {'status': status, 'k': k, 'earliest_stages': earliest, 'note': note}


def trajectory_spread(left, right, fixed_steps=FIXED_STEPS):
    """Same-index distances between two boots' steps.npy trajectories (truth and estimated position) and the
    start-state differences at step 0."""
    a, b = np.load(pathlib.Path(left) / 'steps.npy'), np.load(pathlib.Path(right) / 'steps.npy')
    n = min(len(a), len(b))
    out = {'common_steps': n, 'steps': [len(a), len(b)]}
    if n == 0:
        return dict(out, status='NO_COMMON_STEPS')
    for name, cols in (('truth_pos', TRUTH_POS), ('est_pos', EST_POS)):
        d = np.linalg.norm(a[:n, cols] - b[:n, cols], axis=1)
        out[name + '_dist_max_m'] = float(d.max())
        out[name + '_dist_mean_m'] = float(d.mean())
        out[name + '_dist_at_fixed_steps_m'] = {str(k): float(d[k]) for k in fixed_steps if k < n}
    out['start'] = {'truth_pos_dist_m': float(np.linalg.norm(a[0, TRUTH_POS] - b[0, TRUTH_POS])),
                    'est_pos_dist_m': float(np.linalg.norm(a[0, EST_POS] - b[0, EST_POS])),
                    'est_vel_diff_mps': float(np.linalg.norm(a[0, EST_VEL] - b[0, EST_VEL])),
                    'truth_yaw_diff_rad': float(abs(math.remainder(a[0, 26] - b[0, 26], 2 * math.pi))),
                    'sim_s_left_right': [float(a[0, 1]), float(b[0, 1])]}
    return out


def _new_stage(tol):
    return {'first_exact_k': None, 'first_k_above_tol': None, 'max_abs': 0.0, 'reporting_tolerance': tol,
            'max_abs_at_fixed_steps': {}, 'nonfinite_mismatch_n': 0, 'nonfinite_mismatch_steps': 0,
            'first_k_nonfinite_mismatch': None, 'nonfinite_mismatch_at_fixed_steps': {},
            'incomparable_steps': 0, 'incomparable_k': None, 'incomparable_at_fixed_steps': []}


def _payload_diff(keys, sx, sy, load_x, load_y):
    """Differing keys by signature; for each, diff_stats of the stored arrays. Returns (differ, max_abs over comparable
    keys, non-finite mismatch count, incomparable keys)."""
    bad = [q for q in keys if sx['signatures'].get(q) != sy['signatures'].get(q)]
    m, nf, incomparable = 0.0, 0, []
    for q in bad:
        ax, ay = load_x().get(q), load_y().get(q)
        r = None if ax is None or ay is None else diff_stats(ax, ay)
        if r is None:
            incomparable.append(q)
        else:
            m, nf = max(m, r['max_abs']), nf + r['nonfinite_mismatch_n']
    return bool(bad), m, nf, incomparable


def _first_and_max(reading, k, differ, value):
    if differ and reading['first_exact_k'] is None:
        reading['first_exact_k'] = k
    reading['max_abs_us'] = max(reading['max_abs_us'], value)


def compare_members(left, right, tol=None, fixed_steps=FIXED_STEPS):
    """Same-index recorded differences per stage: exact (SHA-256 / relative stamps), max-abs over entries finite in
    both boots, the separate non-finite mismatch count, first step above a fixed reporting tolerance, values at fixed
    steps, steps that are incomparable (shape mismatch or missing array only), the tie reading, the hover packet,
    absolute step-0 offsets and reference-free stamp readings. No claim about a causal initiator."""
    tol = dict(REPORT_TOL, **(tol or {}))
    dl, a, pa = _member(left)
    dr, b, pb = _member(right)
    n = min(len(a), len(b))
    stages = {s: _new_stage(tol[s]) for s in STAGE_NAMES}
    stages['sensor_stamp_relative'].update(max_abs_target_count=0, units='max_abs in integer microseconds over '
                                           'sim_now/lidar_t/rgb_t/depth_t/target_t; target_count apart')
    ref_free = {'frame_age': {'first_exact_k': None, 'max_abs_us': 0, 'fields': list(AGE_FIELDS)},
                'sim_now_increment': {'first_exact_k': None, 'max_abs_us': 0,
                                      'definition': 'step k packet sim_now minus step k-1 packet sim_now, k >= 1'}}
    out = {'common_steps': n, 'steps': [len(a), len(b)], 'stages': stages, 'reference_free': ref_free}
    if n:
        ha, hb = pa[a[0]['packet_id']]['header'], pb[b[0]['packet_id']]['header']
        t0a, t0b, c0a, c0b = ha['sim_now'], hb['sim_now'], ha['target_count'], hb['target_count']
        out['absolute_offsets'] = {'t0_sim_now_s': [t0a, t0b], 't0_difference_s': t0a - t0b,
                                   'target_count0': [c0a, c0b],
                                   'k0_header_difference': {f: ha[f] - hb[f] for f in STAMP_FIELDS + ('target_count',)}}
    ca, cb = {}, {}
    prev = None
    for k in range(n):
        x, y = a[k], b[k]
        px, py = pa[x['packet_id']], pb[y['packet_id']]
        ages_x, ages_y = frame_ages_us(px['header']), frame_ages_us(py['header'])
        _first_and_max(ref_free['frame_age'], k, not np.array_equal(ages_x, ages_y),
                       int(np.abs(ages_x - ages_y).max()))
        if prev is not None:
            ix = int(round((px['header']['sim_now'] - prev[0]) * 1e6))
            iy = int(round((py['header']['sim_now'] - prev[1]) * 1e6))
            _first_and_max(ref_free['sim_now_increment'], k, ix != iy, abs(ix - iy))
        prev = (px['header']['sim_now'], py['header']['sim_now'])
        for stage, source, keys in STAGES:
            st = stages[stage]
            nf, incomparable = 0, []
            if source == 'header':
                vx, vy = stamp_vector(px['header'], t0a, c0a), stamp_vector(py['header'], t0b, c0b)
                d = np.abs(vx - vy)
                differ, m, count = bool(d.any()), float(d[:-1].max()), int(d[-1])
                st['max_abs_target_count'] = max(st['max_abs_target_count'], count)
                above = m > st['reporting_tolerance'] or count > st['reporting_tolerance']
            else:
                sx, sy = (px, py) if source == 'packet' else (x, y)
                fx, fy = sx['file'], sy['file']
                differ, m, nf, incomparable = _payload_diff(keys, sx, sy, lambda: _npz(dl, fx, ca),
                                                            lambda: _npz(dr, fy, cb))
                above = m > st['reporting_tolerance']
            if differ and st['first_exact_k'] is None:
                st['first_exact_k'] = k
            if above and st['first_k_above_tol'] is None:
                st['first_k_above_tol'] = k
            if nf:
                st['nonfinite_mismatch_n'] += nf
                st['nonfinite_mismatch_steps'] += 1
                if st['first_k_nonfinite_mismatch'] is None:
                    st['first_k_nonfinite_mismatch'] = k
            if incomparable:
                st['incomparable_steps'] += 1
                if st['incomparable_k'] is None:
                    st['incomparable_k'] = k
            st['max_abs'] = max(st['max_abs'], m)
            if k in fixed_steps:
                st['max_abs_at_fixed_steps'][str(k)] = m
                st['nonfinite_mismatch_at_fixed_steps'][str(k)] = nf
                if incomparable:
                    st['incomparable_at_fixed_steps'].append(k)
    out['earliest'] = tie_reading(stages)
    hover = [p for p in pa if p['phase_at_read'] != 'control'][:1], [p for p in pb if p['phase_at_read'] != 'control'][:1]
    if hover[0] and hover[1]:
        hx, hy = hover[0][0], hover[1][0]
        differ, m, nf, incomparable = _payload_diff(('ranges', 'rgb', 'depth'), hx, hy,
                                                    lambda: _npz(dl, hx['file'], ca), lambda: _npz(dr, hy['file'], cb))
        out['hover_packet'] = {'payload_exact_equal': not differ, 'max_abs': m, 'nonfinite_mismatch_n': nf,
                               'incomparable_keys': incomparable}
    else:
        out['hover_packet'] = {'status': 'NOT_RECORDED'}
    out.update(accepted_command='POSTHOC_ULOG_REQUIRED', contacts='SEE_RAW_ON; OFF_NOT_RECORDED',
               interpretation='Recorded ordering only; sample/arrival alignment uncertainty prevents causal attribution')
    return out


def _stats(vals):
    vals = [v for v in vals if isinstance(v, (int, float)) and not isinstance(v, bool)]
    if not vals:
        return {'n': 0}
    return {'n': len(vals), 'median': float(np.median(vals)), 'max': float(max(vals)), 'min': float(min(vals))}


def pair_kind(left, right):
    if left['contact'] != right['contact']:
        return 'between_conditions'
    return 'within_on' if left['contact'] else 'within_off'


Q2_PRIMARY = 'truth_pos_dist_max_m'
Q2_SECONDARY = 'observation.first_k_above_tol'


def range_reading(within, between):
    """Question 2 (review r2 MINOR-1): each between-condition value against the min-max range of the pooled
    within-condition values (within OFF and within ON together) of the same scene: below, inside or above. None
    values are left out; descriptive, no gate."""
    w = [float(v) for v in within if v is not None]
    b = [float(v) for v in between if v is not None]
    if not w or not b:
        return {'status': 'NOT_AVAILABLE', 'within_n': len(w), 'between_n': len(b)}
    lo, hi = min(w), max(w)
    below, above = sum(v < lo for v in b), sum(v > hi for v in b)
    return {'status': 'REPORTED', 'within_range': [lo, hi], 'within_n': len(w), 'between_n': len(b),
            'below': below, 'inside': len(b) - below - above, 'above': above}


def question2(cs):
    """Per scene: the primary per-pair statistic (truth-position distance maximum over the pair's common steps), the
    same distance at each fixed step (pairs that reached it), and the secondary statistic (observation first step
    above tolerance; no such step = inf, i.e. later than every common step)."""
    within = [c for c in cs if c['kind'] in ('within_off', 'within_on')]
    between = [c for c in cs if c['kind'] == 'between_conditions']

    def primary(c):
        return c['trajectory'].get(Q2_PRIMARY)

    def at(k):
        return lambda c: c['trajectory'].get('truth_pos_dist_at_fixed_steps_m', {}).get(str(k))

    def secondary(c):
        v = c['stages']['observation']['first_k_above_tol']
        return float('inf') if v is None else v
    return {'primary_statistic': Q2_PRIMARY, 'primary': range_reading(map(primary, within), map(primary, between)),
            'primary_at_fixed_steps': {str(k): range_reading(map(at(k), within), map(at(k), between))
                                       for k in FIXED_STEPS},
            'secondary_statistic': Q2_SECONDARY + ' (none = inf)',
            'secondary': range_reading(map(secondary, within), map(secondary, between)),
            'within_pool': 'within_off and within_on pooled'}


def spread_summary(comparisons):
    """Per scene and pair kind (within OFF, within ON, between): pair counts, trajectory distances, start-state spread,
    the per-stage first exact step, max-abs, non-finite mismatches and incomparable steps, the reference-free stamp
    readings and the tie readings; per scene the fixed question-2 reading."""
    out = {}
    for env in sorted({c['env'] for c in comparisons}):
        out[str(env)] = {}
        for kind in ('within_off', 'within_on', 'between_conditions'):
            cs = [c for c in comparisons if c['env'] == env and c['kind'] == kind]
            tr = [c['trajectory'] for c in cs if 'truth_pos_dist_max_m' in c['trajectory']]
            out[str(env)][kind] = {
                'pairs': len(cs),
                'truth_pos_dist_max_m': _stats([t['truth_pos_dist_max_m'] for t in tr]),
                'truth_pos_dist_mean_m': _stats([t['truth_pos_dist_mean_m'] for t in tr]),
                'est_pos_dist_max_m': _stats([t['est_pos_dist_max_m'] for t in tr]),
                'est_pos_dist_mean_m': _stats([t['est_pos_dist_mean_m'] for t in tr]),
                'truth_pos_dist_at_fixed_steps_m': {str(k): _stats([t['truth_pos_dist_at_fixed_steps_m'].get(str(k))
                                                                    for t in tr]) for k in FIXED_STEPS},
                'est_pos_dist_at_fixed_steps_m': {str(k): _stats([t['est_pos_dist_at_fixed_steps_m'].get(str(k))
                                                                  for t in tr]) for k in FIXED_STEPS},
                'start_state': {q: _stats([t['start'][q] for t in tr]) for q in
                                ('truth_pos_dist_m', 'est_pos_dist_m', 'est_vel_diff_mps', 'truth_yaw_diff_rad')},
                'start_check_abs_diff': {q: _stats([c['start_check_abs_diff'].get(q) for c in cs])
                                         for q in ('truth_alt_m', 'ekf_alt_m', 'ekf_minus_truth_xy_m',
                                                   'truth_from_spawn_xy_m')},
                'stages': {s: {q: _stats([c['stages'][s][q] for c in cs])
                               for q in ('first_exact_k', 'first_k_above_tol', 'max_abs', 'nonfinite_mismatch_n',
                                         'first_k_nonfinite_mismatch', 'incomparable_steps')} for s in STAGE_NAMES},
                'stamp_target_count_max_abs': _stats([c['stages']['sensor_stamp_relative']['max_abs_target_count']
                                                      for c in cs]),
                'reference_free': {r: {q: _stats([c['reference_free'][r][q] for c in cs])
                                       for q in ('first_exact_k', 'max_abs_us')}
                                   for r in ('frame_age', 'sim_now_increment')},
                'absolute_t0_difference_s': _stats([abs(c['absolute_offsets']['t0_difference_s']) for c in cs
                                                    if 'absolute_offsets' in c]),
                'earliest_status': dict(collections.Counter(c['earliest']['status'] for c in cs)),
                'outcome_equal': sum(c['outcome_equal'] for c in cs)}
        out[str(env)]['question2'] = question2([c for c in comparisons if c['env'] == env])
    return out


def analyze(attempt_dir=ATTEMPT_DIR, write=True):
    d = pathlib.Path(attempt_dir)
    attempt = json.loads((d / 'attempt.json').read_text())
    final = json.loads((d / 'final.json').read_text())
    rows = SF.read_jsonl(d / 'episodes.jsonl')
    problems = S3C.arm_integrity_problems(attempt, final, rows)
    if attempt.get('development_smoke'):
        problems.append('a development/instrumentation smoke has no reading')
    if final.get('status') != 'COMPLETE':
        problems.append('attempt status %s' % final.get('status'))
    sched = attempt.get('schedule', [])
    if len(sched) != 12 or len(rows) != 12:
        problems.append('fixed12 boots incomplete')
    ident = ('arm', 'env', 'repeat', 'contact')
    if [tuple(r.get(q) for q in ident) for r in rows] != [tuple(s.get(q) for q in ident) for s in sched[:len(rows)]]:
        problems.append('schedule repetition/condition identity differs')
    bindings, cores = {}, []
    for r in rows:
        ep = d / ('ep%03d' % r['index'])
        if not r.get('valid') or r.get('recording_problems'):
            problems.append('invalid member%d' % r['index'])
        binding = dict(scene=r.get('scene_binding'), world=(r.get('cell_world') or {}).get('world_sha256'))
        bindings.setdefault(r['env'], binding)
        if bindings[r['env']] != binding or binding['world'] is None:
            problems.append('initial-condition binding changed or missing')
        cores.append(SF.binding_core(r.get('runtime_binding')))
        problems += member_problems(ep, r)
    if cores and (not cores[0] or any(c != cores[0] for c in cores)):
        problems.append('runtime resource binding differs between boots')
    comparisons = []
    if not problems:
        for left, right in itertools.combinations(rows, 2):
            if left['env'] != right['env']:
                continue
            le, re_ = d / ('ep%03d' % left['index']), d / ('ep%03d' % right['index'])
            comparison = compare_members(le, re_)
            sl, sr = left.get('start_check') or {}, right.get('start_check') or {}
            comparison.update(
                left=left['index'], right=right['index'], env=left['env'], kind=pair_kind(left, right),
                outcomes=[left['outcome'], right['outcome']], outcome_equal=left['outcome'] == right['outcome'],
                start_checks=[sl, sr], trajectory=trajectory_spread(le, re_),
                start_check_abs_diff={q: abs(float(sl[q]) - float(sr[q])) for q in
                                      ('truth_alt_m', 'ekf_alt_m', 'ekf_minus_truth_xy_m', 'truth_from_spawn_xy_m')
                                      if isinstance(sl.get(q), (int, float)) and isinstance(sr.get(q), (int, float))},
                hover_guards=[left.get('hover_guard'), right.get('hover_guard')])
            comparisons.append(comparison)
    report = {'protocol': PROTOCOL, 'reading': 'INCOMPLETE_INVALID' if problems else 'REPORTED_DESCRIPTIVE',
              'integrity_problems': problems, 'boots_recorded': len(rows), 'comparisons': comparisons,
              'reporting_tolerances': REPORT_TOL, 'fixed_steps': list(FIXED_STEPS),
              'stamp_reference': ('each boot\'s step-0 packet sim_now (relative; carries any step-0 offset into every '
                                  'step); absolute step-0 offsets and reference-free frame ages and sim_now increments '
                                  'reported separately'),
              'spread_by_scene': spread_summary(comparisons) if comparisons else {},
              'outcome_agreement': {kind: {'pairs': sum(x['kind'] == kind for x in comparisons),
                                           'equal': sum(x['kind'] == kind and x['outcome_equal'] for x in comparisons)}
                                    for kind in ('within_off', 'within_on', 'between_conditions')},
              'output_bytes': output_size(d), 'source_frame_counter': 'NOT_RECORDED',
              'recording_warnings': {r['index']: r['recording_warnings'] for r in rows if r.get('recording_warnings')},
              'limits': ['n3/scene/condition cannot establish equivalence or a causal subscriber effect',
                         'first recorded divergence is not necessarily a causal initiator',
                         'tied stages are not ordered; a k=0 tie is uninformative about order',
                         'instrumented boots do not establish noninterference with historical recorders',
                         'whether --seed controls PX4 SITL sensor-plugin noise engines is NOT_TESTED',
                         'the instrumentation smoke was a short env0 boot: the 600-step path was first exercised '
                         'live in attempt01 (CPU-tested only before)',
                         'question 2 is a descriptive range reading (k of 9 between pairs inside the pooled 6 '
                         'within pairs per scene), not a test',
                         'no OAK-D, camera-only, new-vehicle, S5 or real-flight qualification']}
    if write:
        write_json(d / 'reading.json', report)
    return report


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--launch', action='store_true')
    ap.add_argument('--smoke', action='store_true')
    ap.add_argument('--schedule', action='store_true')
    ap.add_argument('--analyze', nargs='?', const=str(ATTEMPT_DIR))
    ap.add_argument('--compare', nargs=2)
    ap.add_argument('--extract-ulog', nargs=2, metavar=('INPUT', 'OUTPUT'))
    args = ap.parse_args(argv)
    if args.extract_ulog:
        extract_ulog(*args.extract_ulog)
    elif args.launch:
        return launch()
    elif args.smoke:
        return launch(smoke=True)
    elif args.analyze:
        print(json.dumps(analyze(args.analyze), indent=1))
    elif args.compare:
        print(json.dumps(compare_members(*args.compare), indent=1))
    else:
        print(json.dumps({'scope': SCOPE, 'schedule': schedule(), 'smoke': smoke_schedule(),
                          'launch_command': launch_command(), 'smoke_command': launch_command('--smoke'),
                          'measurement': 'NOT_RUN'}, indent=1))
    return EXIT_OK


if __name__ == '__main__':
    sys.exit(main())
