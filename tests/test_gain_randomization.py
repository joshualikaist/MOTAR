"""Opt-in controller-gain randomization for NavRL (D-2026-11-04-1; docs/plans/prereg_gain_randomization_gr0_v1.md).

Everything runs in child processes: importing `aerial_gym` imports Isaac Gym, so the child replaces the package
`__init__` modules with path-only stubs, and the stubs and NAVRL_* variables never reach the shared PUBLIC_CPU process.
"""
import json
import os
import pathlib
import subprocess
import sys
import textwrap
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]

CHILD = textwrap.dedent(r'''
    import json, sys, types, pathlib
    ROOT = pathlib.Path(sys.argv[1])
    for name, rel in (("aerial_gym", "aerial_gym"), ("aerial_gym.config", "aerial_gym/config"),
                      ("aerial_gym.config.controller_config", "aerial_gym/config/controller_config"),
                      ("aerial_gym.control", "aerial_gym/control"),
                      ("aerial_gym.control.controllers", "aerial_gym/control/controllers"),
                      ("aerial_gym.utils", "aerial_gym/utils")):
        m = types.ModuleType(name)
        m.__path__ = [str(ROOT / rel)]
        sys.modules[name] = m
    import importlib
    out = {}
    try:
        cfg = importlib.import_module("aerial_gym.config.controller_config.lee_controller_config_navrl")
    except ValueError as e:
        print(json.dumps({"config_error": str(e)}))
        raise SystemExit(0)
    out["gain_rand"] = cfg.GAIN_RAND
    out["randomize_params"] = bool(cfg.control.randomize_params)
    out["has_scale_attr"] = hasattr(cfg.control, "gain_rand_scale")
    if sys.argv[2] == "controller":
        import torch
        blc = importlib.import_module("aerial_gym.control.controllers.base_lee_controller")

        def make(c):
            ctl = object.__new__(blc.BaseLeeController)
            ctl.cfg, ctl.device, n = c, "cpu", 6
            for k, lo, hi in (("linvel", c.K_vel_tensor_min, c.K_vel_tensor_max),
                              ("rot", c.K_rot_tensor_min, c.K_rot_tensor_max),
                              ("angvel", c.K_angvel_tensor_min, c.K_angvel_tensor_max),
                              ("pos", c.K_pos_tensor_min, c.K_pos_tensor_max)):
                mn = torch.tensor(lo).expand(n, -1)
                mx = torch.tensor(hi).expand(n, -1)
                setattr(ctl, "K_%s_tensor_min" % k, mn)
                setattr(ctl, "K_%s_tensor_max" % k, mx)
                setattr(ctl, "K_%s_tensor_current" % k, (mx + mn) / 2.0)
            return ctl

        def gains(ctl):
            return [ctl.K_linvel_tensor_current.tolist(), ctl.K_rot_tensor_current.tolist(),
                    ctl.K_angvel_tensor_current.tolist(), ctl.K_pos_tensor_current.tolist()]

        torch.manual_seed(123)
        state = torch.get_rng_state()
        a = make(cfg.control)
        nominal = gains(a)
        a.randomize_params(torch.tensor([1, 3, 4]))
        a.randomize_params(torch.tensor([3]))
        out["global_rng_untouched"] = bool(torch.equal(state, torch.get_rng_state()))
        b = make(cfg.control)
        b.randomize_params(torch.tensor([1, 3, 4]))
        b.randomize_params(torch.tensor([3]))
        out["nominal"], out["after"], out["repeat"] = nominal, gains(a), gains(b)
    print(json.dumps(out))
''')


def child(env_extra, mode="config"):
    env = {k: v for k, v in os.environ.items() if not k.startswith("NAVRL_")}
    env.update(env_extra, PYTHONNOUSERSITE="1", CUDA_VISIBLE_DEVICES="")
    done = subprocess.run([sys.executable, "-B", "-c", CHILD, str(ROOT), mode], env=env, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, universal_newlines=True, timeout=300)
    if done.returncode != 0:
        raise AssertionError(done.stderr[-2000:])
    return json.loads(done.stdout.strip().splitlines()[-1])


ON = {"NAVRL_GAIN_RAND": "1", "NAVRL_GAIN_RAND_SCALE": "0.4", "NAVRL_GAIN_RAND_SEED": "4501"}


class GainRandomizationConfig(unittest.TestCase):
    def test_off_is_the_historical_config(self):
        for extra in ({}, {"NAVRL_GAIN_RAND": "0"}):
            r = child(extra)
            self.assertIsNone(r["gain_rand"])
            self.assertFalse(r["randomize_params"])
            self.assertFalse(r["has_scale_attr"])

    def test_on_parses_scale_and_seed(self):
        r = child(ON)
        self.assertEqual(r["gain_rand"], [0.4, 4501])
        self.assertTrue(r["randomize_params"])
        self.assertTrue(r["has_scale_attr"])

    def test_bad_settings_refuse(self):
        bad = ({"NAVRL_GAIN_RAND": "2"}, {"NAVRL_GAIN_RAND": "1", "NAVRL_GAIN_RAND_SCALE": "0.4"},
               {"NAVRL_GAIN_RAND": "1", "NAVRL_GAIN_RAND_SEED": "1"},
               dict(ON, NAVRL_GAIN_RAND_SCALE="1.0"), dict(ON, NAVRL_GAIN_RAND_SCALE="-0.1"),
               dict(ON, NAVRL_GAIN_RAND_SEED="-1"))
        for extra in bad:
            self.assertIn("config_error", child(extra), extra)


class GainRandomizationController(unittest.TestCase):
    def test_draws_are_bounded_seeded_and_leave_the_global_rng_alone(self):
        r = child(ON, "controller")
        self.assertTrue(r["global_rng_untouched"])
        self.assertEqual(r["after"], r["repeat"])                     # same seed, same calls -> same gains
        nominal, after = r["nominal"], r["after"]
        self.assertEqual(after[3], nominal[3])                         # K_pos (unused by velocity control) untouched
        changed = 0
        for g in range(3):
            for env in range(6):
                for ax in range(3):
                    n, v = nominal[g][env][ax], after[g][env][ax]
                    if env in (1, 3, 4):
                        self.assertGreaterEqual(v, n * 0.6 - 1e-6)
                        self.assertLessEqual(v, n * 1.4 + 1e-6)
                        changed += int(v != n)
                    else:
                        self.assertEqual(v, n)                         # envs that did not reset keep nominal
        self.assertGreater(changed, 20)                                # positive control: draws really happen

    def test_scale_zero_keeps_nominal_bitwise_and_off_changes_nothing(self):
        r0 = child(dict(ON, NAVRL_GAIN_RAND_SCALE="0.0"), "controller")
        self.assertEqual(r0["after"], r0["nominal"])
        off = child({}, "controller")
        self.assertEqual(off["after"], off["nominal"])
        self.assertTrue(off["global_rng_untouched"])


if __name__ == "__main__":
    unittest.main()
