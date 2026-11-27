"""CPU checks of the B1-R classical baselines and their opt-in player hook.

Preregistration: docs/plans/prereg_b1r_classical_baseline_v1.md. The controller is checked as pure
functions; the hook is checked in a child process with a stand-in ``aerial_gym`` package, so no
stub module or B1-R variable ever enters this (shared PUBLIC_CPU) process.
"""
import ast
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import textwrap
import unittest

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval import classical_baseline as cb  # noqa: E402
from motar_eval import h_r3_arms  # noqa: E402

ANGLES = torch.linspace(math.pi, -math.pi + 2 * math.pi / 72, 72)
ELEV = torch.deg2rad(torch.tensor([20., 10., 0., -10.]))
Q0 = torch.tensor([[0., 0., 0., 1.]])
ORIGIN = torch.tensor([[0., 0., 1.]])
FAR = torch.full((1, 288), 12.0)


def yaw_quat(yaw):
    return torch.tensor([[0., 0., math.sin(yaw / 2), math.cos(yaw / 2)]])


def act(arm, belief, *, vel=(0., 0., 0.), quat=Q0, scan=FAR, active=True, mask=None, n_gain=4, v_nom=2.0,
        params=cb.PARAMS):
    return cb.baseline_action(arm, ORIGIN, torch.tensor([vel]), quat, torch.tensor([belief]),
                              torch.tensor([active]), scan, ANGLES, ELEV, mask, n_gain=n_gain, v_nom=v_nom,
                              params=params)


def scan_with_return(bearing, horizontal_m):
    """Raw LiDAR [1,288] with one vertical return (all four rows) at ``bearing``."""
    r = torch.full((4, 72), 12.0)
    j = int(torch.argmin(torch.atan2(torch.sin(ANGLES - bearing), torch.cos(ANGLES - bearing)).abs()))
    r[:, j] = horizontal_m / torch.cos(ELEV)
    return r.reshape(1, 288), j


class GuidanceTest(unittest.TestCase):
    def test_los_rate_sign_and_magnitude(self):
        lam, rate = cb.los_geometry(torch.tensor([[5., 0.], [0., 4.]]), torch.tensor([[0., 1.], [2., 0.]]))
        self.assertAlmostEqual(float(lam[0]), 0.0, places=6)
        self.assertAlmostEqual(float(rate[0]), 0.2, places=6)        # counter-clockwise
        self.assertAlmostEqual(float(lam[1]), math.pi / 2, places=6)
        self.assertAlmostEqual(float(rate[1]), -0.5, places=6)       # clockwise

    def test_pn_equals_pp_on_the_los_without_los_rate(self):
        rel, rel_v, own = torch.tensor([[3., 4.]]), torch.tensor([[-.6, -.8]]), torch.tensor([[.6, .8]])
        pn, info = cb.course_command("B-PN", rel, rel_v, own, n_gain=4)
        pp, _ = cb.course_command("B-PP", rel, rel_v, own, n_gain=4)
        self.assertAlmostEqual(float(info["los_rate"][0]), 0.0, places=6)
        self.assertAlmostEqual(float(pn[0]), float(pp[0]), places=6)

    def test_pn_turns_with_the_los_rate_and_saturates(self):
        rel, own = torch.tensor([[5., 0.]]), torch.tensor([[1., 0.]])
        psi, info = cb.course_command("B-PN", rel, torch.tensor([[-1., 1.]]), own, n_gain=3)
        self.assertAlmostEqual(float(psi[0]), 3 * 0.2 * 0.1, places=6)
        self.assertFalse(bool(info["pn_saturated"][0]))
        psi, info = cb.course_command("B-PN", torch.tensor([[.3, 0.]]), torch.tensor([[-1., 2.]]), own, n_gain=5)
        self.assertAlmostEqual(float(psi[0]), 3.0 * 0.1, places=6)
        self.assertTrue(bool(info["pn_saturated"][0]))

    def test_slow_pursuer_uses_the_los(self):
        psi, info = cb.course_command("B-PN", torch.tensor([[0., 5.]]), torch.tensor([[1., 0.]]),
                                      torch.tensor([[0.1, 0.]]), n_gain=4)
        self.assertAlmostEqual(float(psi[0]), math.pi / 2, places=6)
        self.assertTrue(bool(info["slow"][0]))

    def test_unknown_arm_is_refused(self):
        with self.assertRaises(ValueError):
            cb.course_command("B3", torch.zeros(1, 2), torch.zeros(1, 2), torch.zeros(1, 2), n_gain=4)


class ActionTest(unittest.TestCase):
    def test_bounds_z_and_direction(self):
        torch.manual_seed(7)
        n = 512
        state = torch.cat([torch.randn(n, 2) * 8, torch.ones(n, 1), torch.randn(n, 2), torch.zeros(n, 1)], dim=1)
        yaw = torch.rand(n) * 2 * math.pi
        quat = torch.stack([torch.zeros(n), torch.zeros(n), torch.sin(yaw / 2), torch.cos(yaw / 2)], dim=1)
        for arm in cb.ARMS:
            for v_nom in (1.5, 2.0, 2.5, 3.0):
                a, info = cb.baseline_action(arm, ORIGIN.expand(n, 3), torch.randn(n, 3), quat, state,
                                             torch.rand(n) > 0.2, torch.rand(n, 288) * 12, ANGLES, ELEV,
                                             torch.rand(n, 4, 72) > 0.95, n_gain=4, v_nom=v_nom)
                self.assertTrue(bool(torch.isfinite(a).all()))
                self.assertLessEqual(float(a.abs().max()), 1.0)
                self.assertEqual(int((a[:, 2] != 0).sum()), 0)
                self.assertEqual(int(info["raw_out_of_bounds"].sum()), 0)

    def test_direction_preserving_per_axis_bound(self):
        a, _ = act("B-PP", (4., 4., 1., 0., 0., 0.), v_nom=3.0)
        self.assertAlmostEqual(float(a[0, 0]), float(a[0, 1]), places=6)
        self.assertAlmostEqual(float(a[0, :2].norm()) * 2.5, 3.0, places=5)   # diagonal: 3.0 m/s request
        a, _ = act("B-PP", (6., 0., 1., 0., 0., 0.), v_nom=3.0)
        self.assertAlmostEqual(float(a[0, 0]), 1.0, places=6)                  # on body x: 2.5 m/s
        self.assertAlmostEqual(float(a[0, 1]), 0.0, places=6)

    def test_vehicle_frame_and_yaw_law(self):
        a, _ = act("B-PP", (0., 5., 1., 0., 0., 0.), quat=yaw_quat(math.pi / 2))
        self.assertAlmostEqual(float(a[0, 0]), 0.8, places=5)
        self.assertAlmostEqual(float(a[0, 1]), 0.0, places=5)
        self.assertAlmostEqual(float(a[0, 3]), 0.0, places=5)
        a, _ = act("B-PP", (0., 5., 1., 0., 0., 0.))
        self.assertAlmostEqual(float(a[0, 3]), 1.0, places=6)                  # 2.0*pi/2 > 3.0 -> clamped
        a, _ = act("B-PP", (5., 1., 1., 0., 0., 0.))
        self.assertAlmostEqual(float(a[0, 3]), 2.0 * math.atan2(1, 5) / 3.0, places=5)

    def test_no_belief_scans_without_translation(self):
        a, info = act("B-PN", (0., 0., 0., 0., 0., 0.), active=False)
        self.assertEqual(a[0].tolist()[:3], [0.0, 0.0, 0.0])
        self.assertAlmostEqual(float(a[0, 3]), 0.2, places=6)
        self.assertTrue(bool(info["no_belief"][0]))

    def test_deterministic(self):
        torch.manual_seed(3)
        args = (ORIGIN.expand(64, 3), torch.randn(64, 3), Q0.expand(64, 4), torch.randn(64, 6) * 5,
                torch.rand(64) > .3, torch.rand(64, 288) * 12, ANGLES, ELEV, torch.rand(64, 4, 72) > .9)
        a1, _ = cb.baseline_action("B-PN", *args, n_gain=5, v_nom=2.5)
        a2, _ = cb.baseline_action("B-PN", *args, n_gain=5, v_nom=2.5)
        self.assertTrue(torch.equal(a1, a2))


class FilterTest(unittest.TestCase):
    def test_filter_equals_the_h_r3_reference_without_exclusion(self):
        torch.manual_seed(11)
        lidar, vel = torch.rand(256, 288) * 3 + 0.2, torch.randn(256, 2)
        pts = cb.lidar_points_xy(lidar, ANGLES, ELEV, 12.0)
        self.assertTrue(torch.equal(pts, h_r3_arms.lidar_points_xy(lidar, ANGLES, ELEV, 12.0)))
        ours, _ = cb.approach_filter(vel.clone(), pts, 1.5)
        p, ref = h_r3_arms.PARAMS, vel.clone()
        near_r, near_i = pts.norm(dim=-1).topk(p["safety_points"], dim=1, largest=False)
        for k in range(p["safety_points"]):
            r = near_r[:, k:k + 1]
            q = torch.gather(pts, 1, near_i[:, k].view(-1, 1, 1).expand(-1, 1, 2)).squeeze(1) / r.clamp(min=1e-6)
            w = ((p["safety_radius_m"] - r) / (p["safety_radius_m"] - p["safety_full_m"])).clamp(0, 1)
            ref = ref - w * (ref * q).sum(1, keepdim=True).clamp(min=0) * q
        self.assertTrue(torch.equal(ours, ref))
        self.assertEqual(cb.PARAMS["safety_radius_floor_m"], p["safety_radius_m"])
        for key in ("safety_full_m", "safety_points", "belief_exclusion_m"):
            self.assertEqual(cb.PARAMS[key], p[key])

    def test_speed_scaled_onset_radius(self):
        expected = {1.5: 1.5, 2.0: 1.5, 2.5: 1.8054, 3.0: 2.3198}
        for v, r in expected.items():
            self.assertAlmostEqual(cb.safety_radius(v), r, places=4)

    def test_faster_nominal_speed_starts_braking_earlier(self):
        scan, _ = scan_with_return(0.0, 1.7)
        slow, _ = act("B-PP", (6., 0., 1., 0., 0., 0.), scan=scan, v_nom=2.0)
        fast, info = act("B-PP", (6., 0., 1., 0., 0., 0.), scan=scan, v_nom=2.5)
        self.assertAlmostEqual(float(slow[0, 0]), 0.8, places=5)      # 1.7 m is outside r_s = 1.5
        self.assertLess(float(fast[0, 0]), 1.0)                         # inside r_s = 1.805
        self.assertTrue(bool(info["filter_active"][0]))

    def test_bar_ahead_removes_the_approach_component(self):
        scan, _ = scan_with_return(0.0, 1.0)
        a, info = act("B-PP", (6., 0., 1., 0., 0., 0.), scan=scan)
        self.assertLess(float(a[0, 0]), 0.1)
        self.assertTrue(bool(info["filter_active"][0]))

    def test_target_return_near_the_belief_is_excluded(self):
        scan, _ = scan_with_return(0.0, 1.0)
        a, info = act("B-PP", (1.0, 0., 1., 0., 0., 0.), scan=scan)
        self.assertAlmostEqual(float(a[0, 0]), 0.8, places=5)
        self.assertEqual(int(info["excluded_belief"][0]), 4)
        no_excl = dict(cb.PARAMS, belief_exclusion_m=0.0)
        a, _ = act("B-PP", (1.0, 0., 1., 0., 0., 0.), scan=scan, params=no_excl)
        self.assertLess(float(a[0, 0]), 0.1)        # without exclusion the target blocks its own capture

    def test_sensor_associated_mask_is_excluded(self):
        scan, j = scan_with_return(0.0, 1.0)
        mask = torch.zeros(1, 4, 72, dtype=torch.bool)
        mask[0, :, j] = True
        a, info = act("B-PP", (3.0, 0., 1., 0., 0., 0.), scan=scan, mask=mask)
        self.assertAlmostEqual(float(a[0, 0]), 0.8, places=5)
        self.assertEqual((int(info["excluded_mask"][0]), int(info["excluded_belief"][0])), (4, 0))

    def test_belief_exclusion_needs_an_active_track(self):
        scan, _ = scan_with_return(0.0, 1.0)
        _, info = act("B-PP", (1.0, 0., 1., 0., 0., 0.), scan=scan, active=False)
        self.assertEqual(int(info["excluded_belief"][0]), 0)


class FirewallTest(unittest.TestCase):
    def test_controller_imports_only_math_and_torch(self):
        tree = ast.parse((ROOT / "motar_eval/classical_baseline.py").read_text())
        names = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names |= {a.name.split(".")[0] for a in node.names}
            elif isinstance(node, ast.ImportFrom):
                names.add((node.module or "").split(".")[0])
        self.assertEqual(names - {"__future__"}, {"math", "torch"})

    def test_no_ground_truth_names_in_controller_or_hook(self):
        banned = ("target_position", "target_vel_w", "segmentation", "semantic", "obstacle_position",
                  "states", "silhouette", "captured_now", "crashed_now")
        for rel in ("motar_eval/classical_baseline.py", "tools/b1r_hook.py"):
            source = (ROOT / rel).read_text()
            code = "\n".join(line.split("#")[0] for line in source.splitlines())
            for name in banned:
                self.assertNotIn('"%s"' % name, code, rel)
                self.assertNotIn(".%s" % name, code, rel)


HOOK_CHILD = textwrap.dedent("""
    import json, math, torch
    from types import SimpleNamespace as NS
    import aerial_gym.rl_training.rl_games.navrl_players as m
    n = 3
    tracker = NS(state=torch.tensor([[6., 0., 1., 0., 0., 0.]] * n), active=torch.tensor([True, True, False]),
                 memory_s=5.0)
    perception = NS(tracker=tracker, _lidar_angles=torch.linspace(math.pi, -math.pi + 2 * math.pi / 72, 72),
                    last_target_like=torch.zeros(n, 4, 72, dtype=torch.bool))
    class Task:
        vision_mode = True
        task_config = NS(max_velocity=2.5, yaw_rate_max=3.0, lidar_max_range=12.0)
        obs_dict = {"robot_position": torch.tensor([[0., 0., 1.]] * n), "robot_linvel": torch.zeros(n, 3),
                    "robot_vehicle_orientation": torch.tensor([[0., 0., 0., 1.]] * n)}
        def _lidar_distance_m(self):
            return torch.full((n, 288), 12.0)
    Task.perception = perception
    player = m.NavRLPpoPlayerContinuous(NS(env=Task()))
    action = player.get_action(torch.zeros(n, 10), True)
    print(json.dumps({"patched": hasattr(m.NavRLPpoPlayerContinuous.get_action, "__b1r_original__"),
                      "action": action.tolist()}))
""")
STARTUP_CHILD = textwrap.dedent("""
    import json, sys
    print(json.dumps({"torch": "torch" in sys.modules, "aerial_gym": "aerial_gym" in sys.modules,
                      "baseline": "motar_eval.classical_baseline" in sys.modules, "hook": "b1r_hook" in sys.modules}))
""")
STANDIN_PLAYER = textwrap.dedent("""
    import torch
    class NavRLPpoPlayerContinuous:
        clip_actions = False
        def __init__(self, env):
            self.env = env
        def get_action(self, obs, is_deterministic=False):
            return torch.full((obs.shape[0], 4), 0.25)
        def run(self):
            self.get_action(torch.zeros(3, 10), True)
            raise SystemExit(0)          # the episode ledger's quota stop
""")
RUN_CHILD = HOOK_CHILD.replace(
    "action = player.get_action(torch.zeros(n, 10), True)",
    "import atexit\natexit.register(lambda: print(json.dumps({'patched': True, 'action': []})))\nplayer.run()")


class ChildMixin:
    """Runs the hook in a child process with a stand-in player module; this process stays clean."""

    def run_child(self, arm, script=HOOK_CHILD, drop=(), expect_rc=0):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            pkg = tmp / "standin" / "aerial_gym" / "rl_training" / "rl_games"
            pkg.mkdir(parents=True)
            for d in (pkg, pkg.parent, pkg.parent.parent):
                (d / "__init__.py").write_text("")
            (pkg / "navrl_players.py").write_text(STANDIN_PLAYER)
            (tmp / "child.py").write_text(script)
            env = {k: v for k, v in os.environ.items() if not k.startswith(("MOTAR_B1R_", "NAVRL_"))}
            env.update(PYTHONNOUSERSITE="1", CUDA_VISIBLE_DEVICES="",
                       PYTHONPATH=os.pathsep.join([str(tmp / "standin"), str(ROOT), str(ROOT / "tools/b1r_site")]),
                       MOTAR_B1R_CONTROLLER="B-PN", MOTAR_B1R_N="4", MOTAR_B1R_VNOM="2.5",
                       NAVRL_BULK_EVAL_JSON=str(tmp / "result.json"))
            if arm:
                env["MOTAR_B1R_ARM"] = arm
            for key in drop:
                env.pop(key)
            done = subprocess.run([sys.executable, "-B", str(tmp / "child.py")], env=env, cwd=str(tmp),
                                  capture_output=True, text=True, timeout=120)
            self.assertEqual(done.returncode, expect_rc, done.stderr[-2000:])
            if expect_rc:
                return None, done.stderr
            out = json.loads(done.stdout.strip().splitlines()[-1])
            sidecar = tmp / "b1r_hook.json"
            return out, (json.loads(sidecar.read_text()) if sidecar.is_file() else None)


class HookTest(ChildMixin, unittest.TestCase):
    def test_inert_without_the_variable(self):
        out, sidecar = self.run_child("")
        self.assertFalse(out["patched"])
        self.assertEqual(out["action"], [[0.25] * 4] * 3)
        self.assertIsNone(sidecar)

    def test_shadow_returns_the_policy_action_bit_for_bit(self):
        out, sidecar = self.run_child("F_SHADOW")
        self.assertTrue(out["patched"])
        self.assertEqual(out["action"], [[0.25] * 4] * 3)
        self.assertEqual((sidecar["arm"], sidecar["get_action_calls"], sidecar["patch_applied"]), ("F_SHADOW", 1, True))
        self.assertEqual(sidecar["counts"]["non_finite"], 0)
        self.assertEqual(sidecar["window"], "PROCESS_WINDOW")
        self.assertEqual(Path(sidecar["sitecustomize"]), (ROOT / "tools/b1r_site/sitecustomize.py").resolve())
        self.assertIsInstance(sidecar["pid"], int)

    def test_sidecar_written_when_player_run_exits(self):
        out, sidecar = self.run_child("B-PN", script=RUN_CHILD)
        self.assertEqual(sidecar["written_by"], "player_run_exit")
        self.assertEqual(sidecar["get_action_calls"], 1)
        out, sidecar = self.run_child("B-PN")
        self.assertEqual(sidecar["written_by"], "atexit")             # fallback only

    def test_baseline_arm_returns_the_baseline(self):
        out, sidecar = self.run_child("B-PN")
        self.assertEqual(out["action"][0], [1.0, 0.0, 0.0, 0.0])
        self.assertAlmostEqual(out["action"][2][3], 0.2, places=6)
        self.assertEqual(sidecar["counts"]["no_belief"], 1)
        self.assertEqual(sidecar["controller"], "B-PN")

    def test_startup_imports_no_torch_or_simulator(self):
        out, sidecar = self.run_child("B-PN", script=STARTUP_CHILD)
        self.assertEqual(out, {"torch": False, "aerial_gym": False, "baseline": False, "hook": True})
        self.assertIsNone(sidecar)      # only a process that patched and acted writes one

    def test_install_error_fails_closed(self):
        _, stderr = self.run_child("B-PN", script=STARTUP_CHILD, drop=("MOTAR_B1R_VNOM",), expect_rc=3)
        self.assertIn("install failed", stderr)

    def test_unknown_arm_fails_closed(self):
        _, stderr = self.run_child("B3", script=STARTUP_CHILD, expect_rc=3)
        self.assertIn("MOTAR_B1R_ARM", stderr)


# Runner/analysis imports use unique names; no simulator package stubs are installed here.
import importlib.util
from unittest import mock


def load_tool(name):
    spec = importlib.util.spec_from_file_location('_b1r_test_' + name, ROOT / 'tools' / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


br = load_tool('b1r_run')
ba = load_tool('b1r_analysis')


class RunnerTest(unittest.TestCase):
    def test_frozen_plan_and_matrices(self):
        p = br.load_plan()
        self.assertEqual(p['_prereg_sha256'], br.PREREG_SHA)
        selected = {'selected': {'B-PN': {'N': 5, 'v_nom': 2.5}, 'B-PP': {'N': 0, 'v_nom': 2.0}}}
        for stage, n in [('A', 28), ('B', 32), ('C-PN', 28), ('C-PP', 28)]:
            cells = br.stage_cells(p, stage, selected)
            self.assertEqual(len(cells), n)
            self.assertEqual(len({c['cell_dir'] for c in cells}), n)
        self.assertIn('tools/b1r_run.py', p['runtime_paths'])
        for key, value in cb.PARAMS.items():
            if key in p['controller']:
                self.assertEqual(value, p['controller'][key])

    def test_real_tuning_template_and_environment(self):
        p = br.load_plan(); legacy = br.load_tool('kroc2027_eval')
        occ = br.read(ROOT / p['contract_source']['plan'])
        protocol = legacy.kp.load_protocol(legacy.PROTOCOL)
        c = br.stage_cells(p, 'B')[0]; old = br.template_dir(p, c)
        if not (old / 'receipt.json').exists():
            self.skipTest('original OCC-P2 receipt unavailable')
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / 'cell'; target.mkdir()
            env = br.contract_env(legacy, legacy.base_env(inherited={}), protocol, occ, p, c, target.parent)
            original = br.read(old / 'receipt.json')['env_snapshot_navrl']
            # Launcher provides its frozen defaults; compare all runner-supplied NAVRL values.
            for key, value in env.items():
                if key.startswith('NAVRL_') and key not in p['contract_source']['env_keys_allowed_to_differ'] and key not in p['contract_source']['tuning_env_extra']:
                    self.assertEqual(value, original.get(key), key)
            actual = dict(original)
            actual.update({k: v for k, v in env.items() if k.startswith('NAVRL_')})
            actual.update(NAVRL_BULK_EVAL_JSON=str(target/'result.json'), NAVRL_SEED=str(c['seed']))
            (target/'receipt.json').write_text(json.dumps({'env_snapshot_navrl': actual}))
            self.assertEqual(br.env_problems(target, old, p, 'B'), [])
            actual['NAVRL_EPISODE_QUOTA_PER_ENV'] = '4'
            (target/'receipt.json').write_text(json.dumps({'env_snapshot_navrl': actual}))
            self.assertTrue(br.env_problems(target, old, p, 'B'))
            self.assertEqual(env['PYTHONPATH'], os.pathsep.join([str(ROOT), str(ROOT/'tools/b1r_site')]))

    def test_g0_real_same_and_different_seed(self):
        p=br.load_plan(); c=br.stage_cells(p,'A')[0]; old=br.template_dir(p,c)
        other=br.template_dir(p,dict(c,seed=4302))
        if not old.exists() or not other.exists():
            self.skipTest('original OCC-P2 cells unavailable')
        fields=br.read(ROOT/'docs/plans/occlusion_pilot_v2.json')['g0_noninterference']['ledger_fields']
        legacy=br.load_tool('kroc2027_eval')
        self.assertEqual(br.g0_compare(legacy,old,old,4,fields)['problems'],[])
        self.assertTrue(br.g0_compare(legacy,old,other,4,fields)['problems'])

    def test_selection_exact_grid_and_ties(self):
        p=br.load_plan(); cells=br.stage_cells(p,'B')
        counts={'capture':100,'crash':10,'timeout':146}
        data=[(c,dict(counts)) for c in cells]
        selected=br.select(data,p)['selected']
        self.assertEqual(selected['B-PN']['N'],3)
        self.assertEqual(selected['B-PN']['v_nom'],1.5)
        self.assertEqual(selected['B-PP']['v_nom'],1.5)
        # A single capture improvement outranks all secondary criteria.
        data[-1][1].update(capture=101,crash=20,timeout=135)
        self.assertEqual(br.select(data,p)['selected']['B-PP']['v_nom'],3)
        with self.assertRaises(ValueError):br.select(data[:-1],p)
        with self.assertRaises(ValueError):br.select(data+[data[0]],p)

    def test_census_complete_large_lists_late_lines_and_symlinks(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp); data=p/'data';data.mkdir();(p/'link').symlink_to(data, target_is_directory=True)
            (data/'cycle').symlink_to(p, target_is_directory=True)
            (data/'a.json').write_text(json.dumps([{}]*5001+[{'seed':4390}]))
            (data/'b.jsonl').write_text('{}\n'*202+json.dumps({'seed':4390})+'\n')
            census=br.seed_census([p],4390)
            self.assertEqual(len(census['hits']),2)
            self.assertEqual(census['parse_errors'],0)
            (data/'bad.json').write_text('{')
            self.assertEqual(br.seed_census([p],4390)['parse_errors'],1)

    def test_g1_signatures_and_missing_rows(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)
            rows=[dict(env_index=i,env_episode_index=0,in_quota=True) for i in range(10)]
            rec=[dict(env_index=i,env_episode_index=0,ever_track_active=False,outcome='timeout',
                      swept_min_distance_m=1.,observations=10,obs_camera_valid=10) for i in range(10)]
            (p/'episode_ledger_summary.json').write_text(json.dumps({'rows_file':'episodes.jsonl','quota':{'counts':{'capture':0}}}))
            (p/'episodes.jsonl').write_text('\n'.join(map(json.dumps,rows)))
            (p/'kroc2027_episodes.jsonl').write_text('\n'.join(map(json.dumps,rec)))
            (p/'b1r_hook.json').write_text(json.dumps({'counts':{'steps_env':10,'no_belief':6}}))
            found,_=br.defect_signatures([p],br.load_plan()['gates'])
            self.assertEqual({x.split(':')[0] for x in found},{'D0','D1','D2','D3'})
            (p/'kroc2027_episodes.jsonl').write_text('\n'.join(map(json.dumps,rec[:-1])))
            with self.assertRaises(ValueError):br.defect_signatures([p],br.load_plan()['gates'])

    def test_guards_and_manifest_tamper(self):
        legacy=mock.Mock();legacy.free_disk_gb.return_value=9;legacy.git.side_effect=lambda *a:'abc' if a[0]=='rev-parse' else ''
        with mock.patch.object(br.time,'monotonic',return_value=10):
            self.assertIsNone(br.guard_reason(legacy,ROOT,'abc',20))
            self.assertEqual(br.guard_reason(legacy,ROOT,'abc',9),'WALL_CLOCK_CAP')
            legacy.free_disk_gb.return_value=8.19
            self.assertEqual(br.guard_reason(legacy,ROOT,'abc',20),'FREE_DISK_BELOW_8P2_GB')
            legacy.free_disk_gb.return_value=9
            self.assertEqual(br.guard_reason(legacy,ROOT,'other',20),'TRACKED_SOURCE_OR_WORKLOG_CHANGED')
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp); (p/'data').write_text('a')
            br.emit(p/'RAW_MANIFEST.json',{'file_count':1,'files':{'data':br.sha(p/'data')}})
            br.verify_manifest(p)
            (p/'data').write_text('b')
            with self.assertRaises(ValueError):br.verify_manifest(p)

    def test_hook_evidence_fails_closed(self):
        p=br.load_plan();c=br.stage_cells(p,'A')[0]
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp)
            self.assertTrue(br.hook_problems(directory,c))
            s=dict(arm=c['arm'],controller=c['controller'],n_gain=c['n_gain'],v_nom=c['v_nom'],patch_applied=True,
                   get_action_calls=1,counts={'non_finite':0,'raw_out_of_bounds':0,'nonzero_z':0})
            (directory/'b1r_hook.json').write_text(json.dumps(s))
            self.assertTrue(br.hook_problems(directory,c))
            s.update(sitecustomize=str((ROOT/'tools/b1r_site/sitecustomize.py').resolve()),
                     import_origins={m:str(ROOT/m/'__init__.py') for m in ('aerial_gym','motar_eval')},
                     source_sha256={f:br.sha(ROOT/f) for f in br.HOOK_FILES[:3]})
            (directory/'b1r_hook.json').write_text(json.dumps(s))
            self.assertEqual(br.hook_problems(directory,c),[])

    def test_stopped_or_incomplete_stage_rejected(self):
        p=br.load_plan()
        with tempfile.TemporaryDirectory() as tmp:
            p=dict(p,output_root=tmp);d=Path(tmp)/'A';d.mkdir()
            br.emit(d/'run_report.json',{'verdict':'STOPPED','stop_reason':'failed'})
            br.emit(d/'RAW_MANIFEST.json',{'file_count':1,'files':{'run_report.json':br.sha(d/'run_report.json')}})
            with self.assertRaises(ValueError):br.complete_report(p,'A')


class RunnerRepairTest(unittest.TestCase):
    def test_census_records_dangling_links_without_failing(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)
            (p / 'gone').symlink_to(p / 'missing_target')
            (p / 'gone_seed4390').symlink_to(p / 'nowhere')
            (p / 'points_at').symlink_to(p / 'run_seed4390' / 'x.json')
            (p / 'ok.json').write_text(json.dumps({'seed': 4301}))
            census = br.seed_census([p], 4390)
            self.assertEqual(census['parse_errors'], 0)
            self.assertEqual(census['dangling_links'], 3)
            self.assertEqual(len(census['hits']), 2)          # by link name and by target text
            (p / 'bad.jsonl').write_text('{\n')
            self.assertEqual(br.seed_census([p], 4390)['parse_errors'], 1)

    def test_scene_hashes_count_layouts_not_environments(self):
        p = br.load_plan()
        cell = br.template_dir(p, br.stage_cells(p, 'A')[0])
        if not (cell / 'occlusion_layouts.npz').is_file():
            self.skipTest('original OCC-P2 layouts unavailable')
        hashes = br.scene_hashes([cell], p['num_envs'])
        self.assertGreater(len(hashes), p['num_envs'])     # one hash per reset, not per environment
        with self.assertRaises(ValueError):
            br.scene_hashes([cell], 10 ** 6)

    def test_record_cpu_parses_the_run_tests_report(self):
        text = 'noise\n{"x": 1}\n.....\n{\n  "profile": "PUBLIC_CPU",\n  "status": "PASS",\n  "ran": 5,\n' \
               '  "failures": 0,\n  "errors": 0,\n  "skipped": 1,\n  "excluded": 0,\n  "nested": {\n    "a": 1\n  }\n}\n'
        self.assertEqual(br.last_json_object(text)['ran'], 5)
        with tempfile.TemporaryDirectory() as tmp:
            receipt = Path(tmp) / 'receipt.txt'
            receipt.write_text(text)
            here = os.getcwd()
            try:
                os.chdir(tmp)                                  # a relative receipt path must work
                with mock.patch.object(br, 'CPU_EVIDENCE', Path(tmp) / 'cpu.json'):
                    self.assertEqual(br.record_cpu(Path('receipt.txt')), 0)
                    value = br.read(Path(tmp) / 'cpu.json')
            finally:
                os.chdir(here)
            self.assertEqual(value['PUBLIC_CPU']['ran'], 5)
            self.assertEqual(set(value['source_sha256']), {*br.HOOK_FILES, 'tests/test_b1r_classical_baseline.py'})
            receipt.write_text(text.replace('"errors": 0', '"errors": 2'))
            with mock.patch.object(br, 'CPU_EVIDENCE', Path(tmp) / 'cpu2.json'):
                self.assertEqual(br.record_cpu(receipt), 1)


class AnalysisTest(unittest.TestCase):
    def test_exact_statistics(self):
        self.assertEqual(ba.sign_flip_p([1]*7),2/128)
        self.assertEqual(ba.sign_flip_p([0]*7),1)
        self.assertEqual(ba.holm([2/128,4/128])[0],[4/128,4/128])
        self.assertEqual(ba.bca_interval([1]*7,20000,20261023),'NOT_ESTIMABLE')
        ci=ba.bca_interval([1,2,3,4,5,6,7],20000,20261023)
        self.assertTrue(1<=ci[0]<4<ci[1]<=7)
        self.assertEqual(ci,ba.bca_interval([1,2,3,4,5,6,7],20000,20261023))
        self.assertEqual(ba.reading(1,.03,[.1,2]),'SUPERIOR')
        self.assertEqual(ba.reading(-1,.03,[-2,-.1]),'INFERIOR')
        for ci in ([-.1,2],'NOT_ESTIMABLE'):
            self.assertEqual(ba.reading(1,.03,ci),'NOT_SHOWN')

    def test_quota_metrics_and_missing_row_rejection(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)
            rows=[dict(env_index=i,env_episode_index=0,in_quota=True,outcome=o) for i,o in enumerate(['capture','crash'])]
            records=[dict(r,actions=10,obs_camera_valid=5,observations=10,obs_track_active=8,
                          track_losses=1,steps_commanded_over_2p5=3) for r in rows]
            (p/'episode_ledger_summary.json').write_text(json.dumps({'rows_file':'episodes.jsonl','quota':{'counts':{'capture':1,'crash':1,'timeout':0}}}))
            (p/'episodes.jsonl').write_text('\n'.join(map(json.dumps,rows)))
            (p/'kroc2027_episodes.jsonl').write_text('\n'.join(map(json.dumps,records)))
            (p/'result.json').write_text('{}')
            (p/'b1r_hook.json').write_text('{"counts":{}}')
            out=ba.cell_metrics(p)
            self.assertEqual(out['capture_rate'],.5)
            self.assertEqual(out['rmtc60_s'],30.5)
            self.assertEqual(out['time_to_capture_mean_s'],1)
            self.assertEqual(out['camera_detection_fraction'],.5)
            (p/'kroc2027_episodes.jsonl').write_text(json.dumps(records[0]))
            with self.assertRaises(ValueError):ba.cell_metrics(p)


class LateSidecarHookTest(ChildMixin, unittest.TestCase):
    def test_probe_does_not_require_result_path(self):
        out,side=self.run_child('B-PN',script=STARTUP_CHILD,drop=('NAVRL_BULK_EVAL_JSON',))
        self.assertTrue(out['hook']);self.assertIsNone(side)

    def test_actual_action_without_result_path_fails_closed(self):
        _,err=self.run_child('B-PN',drop=('NAVRL_BULK_EVAL_JSON',),expect_rc=3)
        self.assertIn('sidecar failed',err)


if __name__ == "__main__":
    unittest.main()
