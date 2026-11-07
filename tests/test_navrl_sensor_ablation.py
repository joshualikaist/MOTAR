"""C0 gate of the camera-only study: the opt-in LiDAR ablation (aerial_gym/task/navrl_task/navrl_sensor_ablation.py).

CPU only. navrl_perception.py is loaded by file path under private names (no aerial_gym import, no Isaac Gym), the
same way tests/test_navrl_latency_compensate.py does. The ablation wraps each private copy's class, so nothing
leaks into other tests. The governor is the real `_apply_sensor_speed_governor`, extracted from navrl_task.py's
source and bound to a small task object.

Checks, each paired with a positive control:
  leak          with LiDAR removed in some bins, random LiDAR values in those bins leave the actor observation,
                the visibility and confidence, the tracker state and the governed command bit-identical; with
                `full` the same perturbation changes them (non-vacuous)
  kept bins     where LiDAR is kept, perturbing it does change the observation
  transparency  `full` is bit-identical to the unwrapped module, observation and governor
  replica       the wrapper's raw-LiDAR target mask equals perception's own mask under `full`, with targets in view
  governor      K2: with out-of-FOV LiDAR removed, moving commands outside the FOV are capped at exactly 2.0 m/s,
                and in-FOV clearances equal an independent computation from the observation's static scan
  obs_only      no_lidar_obs_only: the observation ignores the LiDAR but the governor still uses it
  isolation     no production module imports navrl_sensor_ablation
Run: PYTHONNOUSERSITE=1 python tests/test_navrl_sensor_ablation.py
"""
import ast
import contextlib
import importlib.util
import math
import os
from pathlib import Path
import sys
import textwrap
import time
import types
import unittest

import torch

ROOT = Path(__file__).resolve().parents[1]
_TASK_DIR = ROOT / "aerial_gym/task/navrl_task"
HB, VB = 72, 4
N = 4
STEPS = 6
LIDAR_MAX = 12.0

for _package in ("aerial_gym", "aerial_gym.task", "aerial_gym.task.navrl_task"):
    sys.modules.setdefault(_package, types.ModuleType(_package))
for _name in ("navrl_corridor", "navrl_search_state"):
    _full = "aerial_gym.task.navrl_task." + _name
    if _full not in sys.modules:
        _spec = importlib.util.spec_from_file_location(_full, _TASK_DIR / (_name + ".py"))
        _mod = importlib.util.module_from_spec(_spec)
        sys.modules[_full] = _mod
        _spec.loader.exec_module(_mod)


def _load(name, path, env=None):
    """Load a module by path under a private name; `env` is applied only while it executes, then restored."""
    saved = {k: os.environ.get(k) for k in (env or {})}
    try:
        os.environ.update(env or {})
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    finally:
        for k, v in saved.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v


_COUNTER = [0]


def perception_module():
    _COUNTER[0] += 1
    mod = _load("navrl_perception_ablation_test_%d" % _COUNTER[0], _TASK_DIR / "navrl_perception.py",
                {"NAVRL_LIDAR_HBEAMS": str(HB), "NAVRL_LIDAR_VBEAMS": str(VB)})
    assert mod.HBEAMS == HB and mod.VBEAMS == VB
    return mod


ABL = _load("navrl_sensor_ablation_test", _TASK_DIR / "navrl_sensor_ablation.py")
SG = _load("speed_governor_ablation_test", _TASK_DIR / "speed_governor.py")


def make_perception(mod):
    camera = types.SimpleNamespace(detector_max_range=20.0, detector_hfov_deg=87.0, detector_vfov_deg=58.0,
                                   camera_width=80, camera_height=45, camera_translation=[0.1, 0.0, 0.03],
                                   camera_target_radius=0.15, tracker_memory_s=5.0,
                                   camera_obstacle_max_range=10.0)       # production far plane (review r1 L7)
    cfg = types.SimpleNamespace(lidar_max_range=LIDAR_MAX, min_target_pixels=2, pixel_threshold=0.55,
                                detection_dropout_prob=0.0, detection_latency_s=0.0, range_error_m=0.0,
                                latency_compensate=False, latency_lidar_backup=False, latency_obstacle_fix=False,
                                latency_ego_motion_fix=False, pose_noise_seed=9163, rgb_noise_std=0.0,
                                depth_noise_std=0.0, history_interval_s=0.5, detector_checkpoint="")
    return mod.NavRLPerceptionModule(N, "cpu", cfg, 0.1, camera)


def scene(seed):
    """Per-step camera frames (bars as near columns, a target blob in envs 0-1) and a base LiDAR scan."""
    g = torch.Generator().manual_seed(seed)
    frames = []
    for k in range(STEPS):
        depth = torch.full((N, 45, 80), 10.0)
        rgb = torch.full((N, 3, 45, 80), 0.15)
        for e in range(N):
            for _ in range(5):
                c = int(torch.randint(0, 78, (1,), generator=g))
                depth[e, :, c:c + 2] = float(torch.empty(1).uniform_(1.5, 9.0, generator=g))
            if e < 2:
                col = 34 + 2 * k + 4 * e
                rgb[e, 0, 20:24, col:col + 5] = 0.88
                rgb[e, 1, 20:24, col:col + 5] = 0.08
                rgb[e, 2, 20:24, col:col + 5] = 0.045
                depth[e, 20:24, col:col + 5] = 3.0 + 0.1 * k
        frames.append((rgb, depth))
    # Mostly open (no return) with a few bars, so corridor clearances fall inside riskcap's 3-5 m release band and
    # the governor's output depends on the LiDAR (otherwise every command sits at the 2.0 m/s floor and a governor
    # leak check would pass vacuously).
    lidar = []
    for _ in range(STEPS):
        x = torch.full((N, VB, HB), LIDAR_MAX)
        for e in range(N):
            bins = torch.randint(0, HB, (8,), generator=g)
            x[e, :, bins] = torch.empty(8).uniform_(3.2, 9.0, generator=g)
        lidar.append(x.reshape(N, VB * HB))
    return frames, lidar



def yaw_quat(deg):
    h = math.radians(deg) / 2.0
    return torch.tensor([[0.0, 0.0, math.sin(h), math.cos(h)]]).repeat(N, 1)


def bins_near(mod, bearing_deg, half_width_deg=8.0):
    b = mod.lidar_bin_bearings("cpu")
    d = torch.atan2(torch.sin(b - math.radians(bearing_deg)), torch.cos(b - math.radians(bearing_deg))).abs()
    return d < math.radians(half_width_deg)


def association_scene(mod, kind):
    """The camera sees the target straight ahead at 3 m for steps 0-2, then loses it; a LiDAR return stays on the
    target's surface (review r1 B1). kind 'in_fov': the target stays ahead. kind 'out_fov': from step 3 the vehicle
    is yawed by 60 deg, so the target lies at -60 deg, outside the 87 deg FOV."""
    frames, lidar, quats = [], [], []
    for k in range(STEPS):
        depth = torch.full((N, 45, 80), 10.0)
        rgb = torch.full((N, 3, 45, 80), 0.15)
        if k < 3:
            rgb[:, 0, 20:24, 38:43] = 0.88
            rgb[:, 1, 20:24, 38:43] = 0.08
            rgb[:, 2, 20:24, 38:43] = 0.045
            depth[:, 20:24, 38:43] = 3.0
        frames.append((rgb, depth))
        yawed = kind == "out_fov" and k >= 3
        quats.append(yaw_quat(60.0) if yawed else yaw_quat(0.0))
        x = torch.full((N, VB, HB), LIDAR_MAX)
        x[:, :, bins_near(mod, -60.0 if yawed else 0.0)] = 3.0
        lidar.append(x.reshape(N, VB * HB))
    return frames, lidar, quats


def same_bar_scene(mod, depth_m=4.0):
    """One bar straight ahead at horizontal distance depth_m, seen consistently by both sensors: the camera columns
    that map to the 0 deg bin read depth_m, and the LiDAR's rows at +20, +10, 0, -10 deg read the slant range
    depth_m / cos(elevation) in that bin (review r1 M1)."""
    frames, lidar = [], []
    elevation = torch.linspace(math.radians(20.0), math.radians(-10.0), VB)
    for _ in range(STEPS):
        depth = torch.full((N, 45, 80), 10.0)
        depth[:, :, 38:43] = depth_m
        frames.append((torch.full((N, 3, 45, 80), 0.15), depth))
        x = torch.full((N, VB, HB), LIDAR_MAX)
        b0 = bins_near(mod, 0.0, 2.0)
        x[:, :, b0] = (depth_m / torch.cos(elevation)).view(1, VB, 1)
        lidar.append(x.reshape(N, VB * HB))
    return frames, lidar


def perturb(lidar, bins_mask, seed):
    """Same scan, other random values in the selected horizontal bins."""
    g = torch.Generator().manual_seed(seed)
    out = []
    for L in lidar:
        x = L.reshape(N, VB, HB).clone()
        noise = torch.empty(N, VB, HB).uniform_(3.0, 6.0, generator=g)    # inside the release band
        x[:, :, bins_mask] = noise[:, :, bins_mask]
        out.append(x.reshape(N, VB * HB))
    return out


def run(perc, frames, lidar, governor=None, commands=None, quats=None, vel=None):
    """Observe the sequence; after each step optionally govern `commands` with the same raw LiDAR."""
    out = []
    pos = torch.tensor([[0.0, 0.0, 1.0]]).repeat(N, 1)
    vel = torch.tensor([[0.4, 0.0, 0.0]]).repeat(N, 1) if vel is None else vel
    identity = torch.tensor([[0.0, 0.0, 0.0, 1.0]]).repeat(N, 1)
    for k, ((rgb, depth), L) in enumerate(zip(frames, lidar)):
        quat = identity if quats is None else quats[k]
        with torch.random.fork_rng():
            torch.manual_seed(1000 + k)
            obs, diag = perc.observe(rgb=rgb, depth=depth, lidar_m=L.clone(), drone_pos_w=pos, drone_vel_w=vel,
                                     vehicle_quat=quat, yaw_rate=torch.zeros(N), previous_action=torch.zeros(N, 4),
                                     max_velocity=2.5, flight_altitude=1.0, training=False)
        rec = {"obs": obs.clone(), "visible": diag["visible"].clone(), "confidence": perc.last_confidence.clone(),
               "lidar_visible": diag["lidar_visible"].clone(), "camera_visible": diag["camera_visible"].clone(),
               "state": perc.tracker.state.clone(), "cov": perc.tracker.cov.clone(),
               "target_like": perc.last_target_like.clone()}
        if governor is not None:
            governor.obs_dict["depth_range_pixels"] = (L.reshape(N, 1, VB, HB) / LIDAR_MAX).clamp(0.0, 1.0)
            rec["governed"] = govern(governor, commands)
        out.append(rec)
    return out


@contextlib.contextmanager
def _speed_governor_visible():
    """The governor method runs `from aerial_gym.task.navrl_task import speed_governor`; expose the private copy on
    the stub package only for the duration of the call, then restore it."""
    pkg = sys.modules["aerial_gym.task.navrl_task"]
    missing = object()
    old = getattr(pkg, "speed_governor", missing)
    pkg.speed_governor = SG
    try:
        yield
    finally:
        if old is missing:
            delattr(pkg, "speed_governor")
        else:
            pkg.speed_governor = old


def govern(governor, commands):
    with _speed_governor_visible():
        return governor._apply_sensor_speed_governor(commands.clone()).clone()


def equal_runs(a, b, keys):
    return all(torch.equal(x[k], y[k]) for x, y in zip(a, b) for k in keys)


_GOV_SRC = None


def governor_class():
    """The real `_apply_sensor_speed_governor` from navrl_task.py, in its own globals namespace."""
    global _GOV_SRC
    if _GOV_SRC is None:
        source = (_TASK_DIR / "navrl_task.py").read_text()
        tree = ast.parse(source)
        cls = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "NavRLTask"][0]
        fn = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "_apply_sensor_speed_governor"][0]
        _GOV_SRC = textwrap.dedent(ast.get_source_segment(source, fn))
    ns = {"torch": torch, "math": math, "time": time, "apply_speed_governor": SG.apply_speed_governor,
          "directional_lidar_clearance": SG.directional_lidar_clearance}
    exec(compile(_GOV_SRC, "<navrl_task.NavRLTask._apply_sensor_speed_governor>", "exec"), ns)
    return type("GovernorTask", (), {"_apply_sensor_speed_governor": ns["_apply_sensor_speed_governor"]}), ns


class _Slot:
    def __setitem__(self, idx, value):
        self.value = value


class _Sink(dict):
    def __missing__(self, key):
        self[key] = _Slot()
        return self[key]


def make_governor(task_cls, perc, mod):
    t = task_cls()
    t.speed_governor_cfg = SG.SpeedGovernorConfig(mode="riskcap", fixed_cap_mps=2.0,
                                                  free_speed_cap_mps=3.53553390593, path_half_width_m=0.45,
                                                  hard_margin_m=0.45, slow_distance_m=3.0, release_distance_m=5.0)
    t._speed_governor_diag_enabled = False
    t._contact_geom_enabled = False
    t._speed_governor_bearings = mod.lidar_bin_bearings("cpu")
    t.task_config = types.SimpleNamespace(lidar_max_range=LIDAR_MAX)
    t.perception = perc
    t.obs_dict = {}
    t._speed_governor_last = _Sink()
    t.device = "cpu"
    return t


# Commands at 3.5 m/s, one per env: forward, 40 deg (inside the FOV, near its edge: at 3 m its corridor contains the
# 45 deg bin, which is outside the FOV), 60 and 135 deg (outside the FOV).
_ANGLES = [0.0, math.radians(40.0), math.radians(60.0), math.radians(135.0)]
COMMANDS = torch.tensor([[3.5 * math.cos(a), 3.5 * math.sin(a)] for a in _ANGLES])


def setup(condition, seed=7):
    mod = perception_module()
    if condition is not None:
        ABL.install_perception(mod, condition)
    perc = make_perception(mod)
    task_cls, ns = governor_class()
    if condition is not None:
        ABL.install_governor(task_cls, condition)
    return mod, perc, make_governor(task_cls, perc, mod), ns


class SensorAblationC0(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.frames, cls.lidar = scene(11)
        mod = perception_module()
        p = make_perception(mod)
        cls.inside = ABL.fov_inside(p)
        assert 0 < int(cls.inside.sum()) < HB

    def _pair(self, condition, bins):
        """Two identical stacks, one fed LiDAR perturbed in `bins`."""
        a, b = setup(condition), setup(condition)
        ra = run(a[1], self.frames, self.lidar, a[2], COMMANDS)
        rb = run(b[1], self.frames, perturb(self.lidar, bins, 99), b[2], COMMANDS)
        return ra, rb

    def test_conditions_known(self):
        self.assertEqual(set(ABL.CONDITIONS), {"full", "no_lidar_in_fov", "no_lidar_out_fov", "no_lidar",
                                               "no_lidar_obs_only", "no_lidar_in_fov_obs_only"})
        self.assertEqual(ABL.condition_from_env({"NAVRL_SENSOR_ABLATION": "no_lidar"}), "no_lidar")
        with self.assertRaises(ValueError):
            ABL.condition_from_env({})

    def test_leak_and_positive_controls(self):
        keys_obs = ("obs", "visible", "confidence", "state", "cov")
        cases = {"no_lidar_in_fov": self.inside, "no_lidar_out_fov": ~self.inside,
                 "no_lidar": torch.ones(HB, dtype=torch.bool)}
        for condition, removed in cases.items():
            with self.subTest(condition=condition):
                ra, rb = self._pair(condition, removed)
                self.assertTrue(equal_runs(ra, rb, keys_obs + ("governed",)), condition)
                # Positive control 1: the same perturbation under `full` changes observation and governor.
                fa, fb = self._pair("full", removed)
                self.assertFalse(equal_runs(fa, fb, ("obs",)), "perturbation is vacuous under full")
                self.assertFalse(equal_runs(fa, fb, ("governed",)), "governor perturbation is vacuous under full")
                # Positive control 2: where LiDAR is kept, perturbing it changes the observation.
                if not bool(removed.all()):
                    ka, kb = self._pair(condition, ~removed)
                    self.assertFalse(equal_runs(ka, kb, ("obs",)), "kept bins are ignored")

    def test_removal_equals_lidar_returning_nothing(self):
        """K1: removing the LiDAR in some bins is exactly an unwrapped module whose LiDAR returns nothing there."""
        cases = {"no_lidar_in_fov": self.inside, "no_lidar_out_fov": ~self.inside,
                 "no_lidar": torch.ones(HB, dtype=torch.bool)}
        for condition, removed in cases.items():
            with self.subTest(condition=condition):
                silent = []
                for L in self.lidar:
                    x = L.reshape(N, VB, HB).clone()
                    x[:, :, removed] = LIDAR_MAX
                    silent.append(x.reshape(N, VB * HB))
                ref, abl = setup(None), setup(condition)
                ra = run(ref[1], self.frames, silent)
                rb = run(abl[1], self.frames, self.lidar)
                self.assertTrue(equal_runs(ra, rb, ("obs", "visible", "confidence", "state", "cov", "target_like")))
                static = rb[-1]["obs"][:, :VB * HB].reshape(N, VB, HB)
                if bool((removed & ~self.inside).any()):
                    self.assertTrue(bool((static[:, :, removed & ~self.inside] == 1.0).all()))

    def test_full_is_transparent(self):
        ref, abl = setup(None), setup("full")
        ra = run(ref[1], self.frames, self.lidar, ref[2], COMMANDS)
        rb = run(abl[1], self.frames, self.lidar, abl[2], COMMANDS)
        self.assertTrue(equal_runs(ra, rb, ("obs", "visible", "confidence", "state", "cov", "target_like",
                                            "governed")))

    def test_raw_target_mask_replica(self):
        _, perc, _, _ = setup("full")
        frames, lidar = self.frames, []
        for L in self.lidar:          # a LiDAR return on the target's surface, so the mask is non-empty
            x = L.reshape(N, VB, HB).clone()
            x[:, :, :] = torch.where(self.inside.view(1, 1, -1), torch.full_like(x, 3.1), x)
            lidar.append(x.reshape(N, VB * HB))
        hits = 0
        pos = torch.tensor([[0.0, 0.0, 1.0]]).repeat(N, 1)
        quat = torch.tensor([[0.0, 0.0, 0.0, 1.0]]).repeat(N, 1)
        for (rgb, depth), L in zip(frames, lidar):
            with torch.random.fork_rng():
                torch.manual_seed(5)
                perc.observe(rgb=rgb, depth=depth, lidar_m=L.clone(), drone_pos_w=pos, drone_vel_w=torch.zeros(N, 3),
                             vehicle_quat=quat, yaw_rate=torch.zeros(N), previous_action=torch.zeros(N, 4),
                             max_velocity=2.5, flight_altitude=1.0, training=False)
            self.assertTrue(torch.equal(perc._ablation_raw_target_like, perc.last_target_like))
            hits += int(perc.last_target_like.sum())
        self.assertGreater(hits, 0, "replica check is vacuous: no target-like LiDAR returns")

    def test_governor_k2_rule_and_in_fov_clearance(self):
        out_cmd = torch.tensor([abs(a) > math.radians(43.5) for a in _ANGLES])
        for condition in ("no_lidar", "no_lidar_out_fov"):
            with self.subTest(condition=condition):
                mod, perc, gov, ns = setup(condition)
                recs = run(perc, self.frames, self.lidar, gov, COMMANDS)
                for rec, L in zip(recs, self.lidar):
                    speed = rec["governed"].norm(dim=1)
                    self.assertTrue(torch.allclose(speed[out_cmd], torch.full_like(speed[out_cmd], 2.0)), speed)
                    # Independent clearance for the in-FOV commands: governor scan rebuilt from the observation.
                    static = rec["obs"][:, :VB * HB].reshape(N, VB, HB) * LIDAR_MAX
                    raw = L.reshape(N, VB, HB)
                    removed = ABL.removed_bins(perc, condition)
                    scan = raw.clone()
                    rin = removed & self.inside
                    elevation = torch.linspace(math.radians(20.0), math.radians(-10.0), VB).view(1, VB, 1)
                    cam = static[:, :, rin]
                    scan[:, :, rin] = torch.where(cam >= LIDAR_MAX * 0.995, torch.full_like(cam, LIDAR_MAX),
                                                  cam / torch.cos(elevation))
                    scan[:, :, removed & ~self.inside] = LIDAR_MAX
                    clearance = SG.directional_lidar_clearance(
                        scan, mod.lidar_bin_bearings("cpu"), COMMANDS, max_range_m=LIDAR_MAX, path_half_width_m=0.45,
                        target_return_mask=rec["target_like"])
                    expect, _ = SG.apply_speed_governor(COMMANDS, clearance, gov.speed_governor_cfg)
                    self.assertTrue(torch.equal(rec["governed"][~out_cmd], expect[~out_cmd]))
        # Positive control: with the out-of-FOV LiDAR kept, an outside command in open space is not capped at 2.0.
        _, perc, gov, _ = setup("no_lidar_in_fov")
        open_lidar = [torch.full((N, VB * HB), LIDAR_MAX) for _ in range(STEPS)]
        recs = run(perc, self.frames, open_lidar, gov, COMMANDS)
        self.assertTrue(bool((recs[-1]["governed"].norm(dim=1)[out_cmd] > 2.0 + 1e-6).all()))

    def test_obs_only_keeps_governor_lidar(self):
        for condition, removed in (("no_lidar_obs_only", torch.ones(HB, dtype=torch.bool)),
                                   ("no_lidar_in_fov_obs_only", self.inside)):
            with self.subTest(condition=condition):
                ra, rb = self._pair(condition, removed)
                self.assertTrue(equal_runs(ra, rb, ("obs", "visible", "confidence", "state", "cov")))
                self.assertFalse(equal_runs(ra, rb, ("governed",)), "obs_only governor ignores the LiDAR")

    def test_obs_only_governor_matches_full_with_target_hits(self):
        """Review r2 M-r2-1: while the camera sees the target (so no association runs), the `*_obs_only` governor is the
        raw-LiDAR governor of `full`, bit for bit. That covers its target mask (the replica on the raw LiDAR) and the
        absence of the K2 cap. Positive controls: the scene has target-mask hits, dropping the mask changes the
        governed command, and commands outside the FOV run above 2.0 m/s in `full`."""
        mod = perception_module()
        near = bins_near(mod, 0.0, 15.0)
        lidar = []
        for _ in range(STEPS):
            x = torch.full((N, VB, HB), LIDAR_MAX)
            x[:, :, near] = 3.1                      # on the target's surface in envs 0-1; an obstacle in envs 2-3
            lidar.append(x.reshape(N, VB * HB))
        _, pf, gf, _ = setup("full")
        full = run(pf, self.frames, lidar, gf, COMMANDS)
        self.assertGreater(sum(int(r["target_like"].sum()) for r in full), 0, "no target-mask hits")
        self.assertEqual(sum(int(r["lidar_visible"].sum()) for r in full), 0)
        bearings = mod.lidar_bin_bearings("cpu")
        unmasked_differs = False
        for rec, L in zip(full, lidar):
            c = SG.directional_lidar_clearance(L.reshape(N, VB, HB), bearings, COMMANDS, max_range_m=LIDAR_MAX,
                                               path_half_width_m=0.45, target_return_mask=None)
            g, _ = SG.apply_speed_governor(COMMANDS, c, gf.speed_governor_cfg)
            unmasked_differs |= not torch.equal(g, rec["governed"])
        self.assertTrue(unmasked_differs, "the target mask does not matter in this scene")
        outside = torch.tensor([abs(a) > math.radians(43.5) for a in _ANGLES])
        self.assertTrue(bool((full[-1]["governed"].norm(dim=1)[outside] > 2.0 + 1e-6).all()))
        for condition in ("no_lidar_obs_only", "no_lidar_in_fov_obs_only"):
            with self.subTest(condition=condition):
                _, pc, gc, _ = setup(condition)
                self.assertTrue(equal_runs(full, run(pc, self.frames, lidar, gc, COMMANDS), ("governed",)))

    def test_association_leak_with_camera_loss(self):
        """Review r1 B1: the LiDAR target association runs only when the camera has lost the target. Scenes where it
        does, inside and outside the FOV; under `full` it must be active (non-zero) and change under perturbation;
        where the condition removes those bearings it must stay off and every output bit-identical; where the
        condition keeps them it must stay active."""
        keys = ("obs", "visible", "lidar_visible", "confidence", "state", "cov")
        mod = perception_module()
        removes = {"no_lidar": {"in_fov": True, "out_fov": True}, "no_lidar_in_fov": {"in_fov": True, "out_fov": False},
                   "no_lidar_out_fov": {"in_fov": False, "out_fov": True},
                   "no_lidar_obs_only": {"in_fov": True, "out_fov": True},
                   "no_lidar_in_fov_obs_only": {"in_fov": True, "out_fov": False}}
        for kind in ("in_fov", "out_fov"):
            frames, lidar, quats = association_scene(mod, kind)
            target_bins = bins_near(mod, 0.0) | bins_near(mod, -60.0)
            other = perturb(lidar, target_bins, 5)
            zero_vel = torch.zeros(N, 3)
            fa = run(setup("full")[1], frames, lidar, quats=quats, vel=zero_vel)
            fb = run(setup("full")[1], frames, other, quats=quats, vel=zero_vel)
            active = sum(int(r["lidar_visible"].sum()) for r in fa)
            self.assertGreater(active, 0, "%s: the association never ran under full" % kind)
            self.assertEqual(sum(int(r["camera_visible"][:].sum()) for r in fa[3:]), 0, "camera did not lose it")
            self.assertFalse(equal_runs(fa, fb, ("lidar_visible", "confidence", "state")), kind)
            for condition, spec in removes.items():
                with self.subTest(kind=kind, condition=condition):
                    perc_a = setup(condition)[1]
                    removed_here = target_bins & ABL.removed_bins(perc_a, condition)
                    ra = run(perc_a, frames, lidar, quats=quats, vel=zero_vel)
                    if spec[kind]:
                        self.assertTrue(bool(removed_here.any()))
                        mine = perturb(lidar, removed_here, 5)      # only bins this condition removes
                        rb = run(setup(condition)[1], frames, mine, quats=quats, vel=zero_vel)
                        self.assertEqual(sum(int(r["lidar_visible"].sum()) for r in ra), 0)
                        self.assertTrue(equal_runs(ra, rb, keys))
                    else:          # bearings kept: the association still runs there
                        self.assertGreater(sum(int(r["lidar_visible"].sum()) for r in ra), 0)

    def test_same_bar_same_clearance(self):
        """Review r1 M1: one bar 4 m ahead seen by the LiDAR (full) and by the camera (in-FOV LiDAR removed) gives
        the same governed speed, riskcap's 2.0 + 0.5 * (3.5355 - 2.0) at 4 m."""
        mod = perception_module()
        frames, lidar = same_bar_scene(mod, 4.0)
        forward = torch.tensor([[3.5, 0.0]]).repeat(N, 1)
        expect = 2.0 + 0.5 * (3.53553390593 - 2.0)
        for condition in ("full", "no_lidar_in_fov", "no_lidar"):
            with self.subTest(condition=condition):
                _, perc, gov, _ = setup(condition)
                speed = run(perc, frames, lidar, gov, forward)[-1]["governed"].norm(dim=1)
                self.assertTrue(torch.allclose(speed, torch.full_like(speed, expect), atol=1e-4), (condition, speed))

    def test_k2_threshold_between_43_and_44_deg(self):
        """Review r1 L6: in open space a 43 deg command (inside the 87 deg FOV) is not capped, a 44 deg one is."""
        _, perc, gov, _ = setup("no_lidar")
        frames = [(torch.full((N, 3, 45, 80), 0.15), torch.full((N, 45, 80), 10.0)) for _ in range(2)]
        open_lidar = [torch.full((N, VB * HB), LIDAR_MAX) for _ in range(2)]
        cmd = torch.tensor([[3.5 * math.cos(math.radians(a)), 3.5 * math.sin(math.radians(a))]
                            for a in (43.0, -43.0, 44.0, -44.0)])
        speed = run(perc, frames, open_lidar, gov, cmd)[-1]["governed"].norm(dim=1)
        self.assertTrue(torch.allclose(speed[:2], torch.full((2,), 3.5), atol=1e-5), speed)
        self.assertTrue(torch.allclose(speed[2:], torch.full((2,), 2.0), atol=1e-5), speed)

    def test_double_install_and_unsupported_governor_refused(self):
        mod = perception_module()
        ABL.install_perception(mod, "no_lidar")
        with self.assertRaises(RuntimeError):
            ABL.install_perception(mod, "no_lidar")
        _, perc, gov, _ = setup("no_lidar")
        run(perc, self.frames[:1], self.lidar[:1])
        gov.speed_governor_cfg = SG.SpeedGovernorConfig(mode="riskcap", lateral_margin_m=0.5)
        gov.obs_dict["depth_range_pixels"] = (self.lidar[0].reshape(N, 1, VB, HB) / LIDAR_MAX).clamp(0, 1)
        with self.assertRaises(RuntimeError):
            govern(gov, COMMANDS)
        # Positive control: the supported configuration runs.
        gov.speed_governor_cfg = SG.SpeedGovernorConfig(mode="riskcap")
        self.assertEqual(tuple(govern(gov, COMMANDS).shape), (N, 2))
        # Review r1 L6: another governor mode refuses; a sensor lineage refuses; governing before any observation
        # refuses. Each with its positive control in `full` or on the supported path.
        gov.speed_governor_cfg = SG.SpeedGovernorConfig(mode="fixed")
        with self.assertRaises(RuntimeError):
            govern(gov, COMMANDS)
        _, pfull, gfull, _ = setup("full")
        run(pfull, self.frames[:1], self.lidar[:1])
        gfull.speed_governor_cfg = SG.SpeedGovernorConfig(mode="fixed")
        gfull.obs_dict["depth_range_pixels"] = gov.obs_dict["depth_range_pixels"]
        self.assertEqual(tuple(govern(gfull, COMMANDS).shape), (N, 2))
        _, plin, _, _ = setup("no_lidar")
        plin.sensor_lineage_s_l1a = True
        with self.assertRaises(RuntimeError):
            run(plin, self.frames[:1], self.lidar[:1])
        _, pfresh, gfresh, _ = setup("no_lidar")
        gfresh.obs_dict["depth_range_pixels"] = gov.obs_dict["depth_range_pixels"]
        with self.assertRaises(RuntimeError):
            govern(gfresh, COMMANDS)

    def test_no_production_import(self):
        import re
        pattern = re.compile(r"^\s*(?:import\s+[\w.]*navrl_sensor_ablation\b|from\s+[\w.]*navrl_sensor_ablation\s+import"
                             r"|from\s+[\w.]+\s+import\s+[^\n]*\bnavrl_sensor_ablation\b)", re.M)
        hits = []
        for base in ("aerial_gym", "motar_eval", "tools"):
            for path in (ROOT / base).rglob("*.py"):
                # The opt-in entry points: camonly_entry.py (C-study evaluations) and camtrain_entry.py (CAM-K1
                # training, D-2026-11-06-1), each selected only through its own wrapper (camonly_python,
                # camtrain_python). No production module may import the ablation.
                if path.name in ("navrl_sensor_ablation.py", "camonly_entry.py", "camtrain_entry.py"):
                    continue
                if pattern.search(path.read_text(errors="ignore")):
                    hits.append(str(path.relative_to(ROOT)))
        self.assertEqual(hits, [])
        # Positive control: the allowed importers are found by the same pattern.
        self.assertTrue(pattern.search((ROOT / "tools/camonly_entry.py").read_text()))
        self.assertTrue(pattern.search((ROOT / "tools/camtrain_entry.py").read_text()))


class CamonlyEntryPoint(unittest.TestCase):
    """tools/camonly_entry.py, run for real in a child process against stub `isaacgym` / `aerial_gym` packages that
    record the call order (the real ones need Isaac Gym). With the real package, `aerial_gym/__init__.py` imports
    isaacgym itself before the origin guard returns, as in a native `runner.py` run (review r1 L3); the stub order
    checks the entry point's own calls only."""

    STUBS = {
        "isaacgym/__init__.py": "import builtins\nbuiltins.ORDER = getattr(builtins, 'ORDER', []) + ['isaacgym']\n",
        "aerial_gym/__init__.py": "",
        "aerial_gym/rl_training/__init__.py": "",
        "aerial_gym/rl_training/rl_games/__init__.py": "",
        "aerial_gym/rl_training/rl_games/navrl_import_origin.py":
            "import builtins\ndef assert_runtime_origin():\n    builtins.ORDER = getattr(builtins, 'ORDER', []) + ['origin']\n"
            "    return {}\n",
        "aerial_gym/rl_training/rl_games/quiet_rl_io.py":
            "import builtins, contextlib, os\ndef quiet_startup_enabled():\n    return os.environ.get('STUB_QUIET') == '1'\n"
            "@contextlib.contextmanager\ndef devnull_stdio():\n    builtins.ORDER = getattr(builtins, 'ORDER', []) + ['quiet']\n"
            "    yield\n",
        "aerial_gym/task/__init__.py": "",
        "aerial_gym/task/navrl_task/__init__.py": "",
        "aerial_gym/task/navrl_task/navrl_sensor_ablation.py":
            "import builtins, os\ndef condition_from_env():\n    return os.environ['NAVRL_SENSOR_ABLATION']\n"
            "def install(c):\n    builtins.ORDER = getattr(builtins, 'ORDER', []) + ['install:' + c]\n"
            "    return {'condition': c}\n",
        "rl/runner.py": "import builtins, json, os, sys\nprint('RUNNER ' + json.dumps({'name': __name__, 'argv': sys.argv,"
                        " 'path0': sys.path[0], 'order': builtins.ORDER, 'path': os.environ['PATH']}))\n",
    }

    def _run(self, root, receipt, quiet=False, wrapper=False):
        import subprocess
        for rel, text in self.STUBS.items():
            p = Path(root) / rel
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(text)
        env = {"PATH": "/usr/bin:/bin", "PYTHONPATH": root, "PYTHONNOUSERSITE": "1", "PYTHONDONTWRITEBYTECODE": "1",
               "NAVRL_SENSOR_ABLATION": "no_lidar", "NAVRL_SENSOR_ABLATION_RECEIPT": receipt,
               "STUB_QUIET": "1" if quiet else "0", "CAMONLY_REAL_PYTHON": sys.executable}
        head = (["bash", str(ROOT / "tools/camonly_python")] if wrapper
                else [sys.executable, "-B", str(ROOT / "tools/camonly_entry.py")])
        return subprocess.run(head + [str(Path(root) / "rl/runner.py"), "--file", "x.yaml", "--play"], env=env,
                              capture_output=True, text=True)

    def _got(self, r):
        import json
        self.assertEqual(r.returncode, 0, r.stderr)
        line = [x for x in r.stdout.splitlines() if x.startswith("RUNNER ")][0]
        return json.loads(line[len("RUNNER "):])

    def test_quiet_startup_and_wrapper_path(self):
        """Review r1 L3 and M3: the quiet-startup branch keeps the order; tools/camonly_python puts the
        interpreter's bin directory first on PATH (ninja for the gymtorch build), as a native run has it."""
        import os
        import tempfile
        with tempfile.TemporaryDirectory() as root:
            got = self._got(self._run(root, str(Path(root) / "r1.json"), quiet=True))
            self.assertEqual(got["order"], ["origin", "quiet", "isaacgym", "install:no_lidar"])   # review r2 L-r2-3
            got = self._got(self._run(root, str(Path(root) / "r2.json"), wrapper=True))
            self.assertEqual(got["path"].split(":")[0], os.path.dirname(sys.executable))
            self.assertEqual(got["order"], ["origin", "isaacgym", "install:no_lidar"])

    def test_entry_order_argv_and_receipt(self):
        import json
        import tempfile
        with tempfile.TemporaryDirectory() as root:
            receipt = str(Path(root) / "receipt.json")
            r = self._run(root, receipt)
            self.assertEqual(r.returncode, 0, r.stderr)
            line = [x for x in r.stdout.splitlines() if x.startswith("RUNNER ")][0]
            got = json.loads(line[len("RUNNER "):])
            self.assertEqual(got["name"], "__main__")
            self.assertEqual(got["argv"], [str(Path(root) / "rl/runner.py"), "--file", "x.yaml", "--play"])
            self.assertEqual(got["path0"], str(Path(root) / "rl"))
            self.assertEqual(got["order"], ["origin", "isaacgym", "install:no_lidar"])
            self.assertEqual(json.loads(Path(receipt).read_text())["condition"], "no_lidar")
            again = self._run(root, receipt)          # the receipt is never overwritten
            self.assertNotEqual(again.returncode, 0)
            self.assertIn("FileExistsError", again.stderr)


if __name__ == "__main__":
    unittest.main()
