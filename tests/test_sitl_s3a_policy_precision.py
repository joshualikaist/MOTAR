"""CPU tests for the S3a-P1 policy precision control (tools/sitl_s3a_policy_precision.py; motar-z5f).

No GPU is used. The real-input preflight runs in a child process, so the bridge's NAVRL_* setup cannot leak into
the shared PUBLIC_CPU process."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def load(name, rel):
    saved = list(sys.path)
    try:
        spec = importlib.util.spec_from_file_location(name, ROOT / rel)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        sys.path[:] = saved
    return mod


P = load("sitl_s3a_policy_precision_t", "tools/sitl_s3a_policy_precision.py")


class Grid(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="s3a_p1_"))
        rng = np.random.default_rng(0)
        self.calls, self.envs = 3, 4
        self.obs = rng.normal(size=(self.calls, self.envs, P.OBS_DIM)).astype(np.float32)
        self.act = rng.normal(size=(self.calls, self.envs, 4)).astype(np.float32)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def write(self, order, obs=None, act=None):
        obs = self.obs if obs is None else obs
        call = np.array([c for c, e in order])
        env = np.array([e for c, e in order])
        rows = np.stack([obs[min(c, self.calls - 1), e] for c, e in order])   # an outside row reuses a value
        np.savez(self.tmp / "obs.npz", obs=rows, call_index=call, env_id=env)
        np.savez(self.tmp / "state.npz", action=self.act if act is None else act)
        return self.tmp / "obs.npz", self.tmp / "state.npz"

    def test_rows_are_placed_by_call_and_env(self):
        order = [(c, e) for c in range(self.calls) for e in range(self.envs)]
        order = order[::-1]                                          # dump order does not matter
        obs, act = P.batches(*self.write(order), calls=self.calls, envs=self.envs)
        np.testing.assert_array_equal(obs, self.obs)
        np.testing.assert_array_equal(act, self.act)

    def test_incomplete_repeated_or_non_finite_grids_refuse(self):
        full = [(c, e) for c in range(self.calls) for e in range(self.envs)]
        repeated = full[:-1] + [full[0]]
        with self.assertRaisesRegex(ValueError, "repeated"):
            P.batches(*self.write(repeated), calls=self.calls, envs=self.envs)
        outside = full[:-1] + [(self.calls, 0)]
        with self.assertRaisesRegex(ValueError, "outside"):
            P.batches(*self.write(outside), calls=self.calls, envs=self.envs)
        bad = self.obs.copy()
        bad[1, 2, 5] = np.nan
        with self.assertRaisesRegex(ValueError, "non-finite"):
            P.batches(*self.write(full, obs=bad), calls=self.calls, envs=self.envs)
        with self.assertRaisesRegex(ValueError, "shapes"):
            P.batches(*self.write(full[:-1]), calls=self.calls, envs=self.envs)

    def test_input_pins(self):
        (self.tmp / "a.bin").write_bytes(b"a")
        pins = {"a": ("a.bin", P.sha256_file(self.tmp / "a.bin"))}
        self.assertEqual(P.verify_inputs(self.tmp, pins)["a"]["sha256"], pins["a"][1])
        with self.assertRaisesRegex(ValueError, "differ"):
            P.verify_inputs(self.tmp, {"a": ("a.bin", "0" * 64)})
        with self.assertRaisesRegex(ValueError, "differ"):
            P.verify_inputs(self.tmp, {"b": ("missing.bin", "0" * 64)})


class Reading(unittest.TestCase):
    def test_compare_and_controls(self):
        a = np.zeros((5, 2, 4), np.float32)
        b = a.copy()
        b[3, 1, 2] = 2e-4
        c = P.compare(a, b)
        self.assertEqual((c["argmax_call_env_dim"], c["n_above_tol"], c["n"]), ([3, 1, 2], 1, 40))
        self.assertAlmostEqual(c["max"], 2e-4, places=9)
        rec = np.arange(40, dtype=np.float32).reshape(5, 2, 4)        # every call differs from the next
        ok = P.controls(rec + 1e-3, rec)
        self.assertTrue(ok["ok"])
        self.assertFalse(P.controls(rec + 0.5, rec)["ok"])             # true pairing too far: positive fails
        same = np.ones((5, 2, 4), np.float32)
        self.assertFalse(P.controls(same, same)["ok"])                 # shifted pairing identical: negative fails
        nan = rec.copy()
        nan[1, 1, 1] = np.nan
        c = P.compare(nan, rec)
        self.assertEqual((c["finite"], c["max"], c["n_nonfinite"]), (False, None, 1))
        self.assertFalse(P.controls(nan, rec)["ok"])

    def test_fixed_reading(self):
        def m(on=None, off_cpu=None, ok=True, on_off=1e-3, finite=True):
            out = {"controls": {"ok": ok}}
            if on is not None:
                out.update(gpu_on_vs_recorded={"max": on, "finite": finite}, gpu_off_vs_cpu={"max": off_cpu},
                           gpu_on_vs_gpu_off={"max": on_off})
            return out
        cases = [({70: m(1e-7, 5e-5), 205: m(0.0, 1e-4)}, "TF32_EXPLAINS_POLICY_RESIDUAL"),
                 ({70: m(1e-7, 5e-5), 205: m(2e-5, 1e-6)}, "GPU_REPLAY_NOT_FAITHFUL"),
                 ({70: m(1e-5, 5e-5), 205: m(0.0, 1.1e-4)}, "MIXED"),                    # 70 explained only
                 ({70: m(1e-5, 2e-4), 205: m(0.0, 1.1e-4)}, "RESIDUAL_NOT_TF32"),
                 ({70: m(1e-5, 2e-3, on_off=0.0), 205: m(0.0, 1.1e-4, on_off=0.0)}, "TF32_SWITCH_NO_EFFECT"),
                 ({70: m(1e-5, 2e-3, on_off=0.0), 205: m(0.0, 1.1e-4)}, "MIXED"),        # review r2 N2
                 ({70: m(1e-7, 5e-5), 205: m(0.0, 2e-3, on_off=0.0)}, "MIXED"),
                 ({70: m(1e-7, 5e-5, finite=False), 205: m(0.0, 1e-6)}, "NONFINITE_OUTPUT"),
                 ({70: m(1e-7, 5e-5, ok=False), 205: m(0.0, 1e-6)}, "CONTROL_FAILED"),
                 ({70: m(), 205: m()}, "CPU_ONLY")]
        for metrics, want in cases:
            self.assertEqual(P.reading(metrics), want, metrics)
        self.assertEqual(P.density_readings({70: m(1e-7, 5e-5), 205: m(0.0, 2e-3, on_off=0.0)}),
                         {"70": "TF32_EXPLAINS_POLICY_RESIDUAL", "205": "TF32_SWITCH_NO_EFFECT"})

    def test_tf32_switches_are_set_and_read_back(self):
        import torch
        saved = (torch.backends.cuda.matmul.allow_tf32, torch.backends.cudnn.allow_tf32,
                 torch.get_float32_matmul_precision())
        try:
            off = P.set_tf32(False)
            self.assertEqual((off["cuda_matmul_allow_tf32"], off["cudnn_allow_tf32"], off["float32_matmul_precision"]),
                             (False, False, "highest"))
            on = P.set_tf32(True)
            self.assertEqual((on["cuda_matmul_allow_tf32"], on["cudnn_allow_tf32"], on["float32_matmul_precision"]),
                             (True, True, "high"))
        finally:
            torch.backends.cuda.matmul.allow_tf32, torch.backends.cudnn.allow_tf32 = saved[:2]
            torch.set_float32_matmul_precision(saved[2])


class CLI(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="s3a_p1_cli_"))

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_approval_problems(self):
        good = {"approved_by": "u", "approved_at": "t", "scope": P.SCOPE, "commit": "abc"}
        clean = {"head": "abc", "dirty": ""}
        self.assertEqual(P.approval_problems(good, clean), [])                      # positive control
        self.assertTrue(P.approval_problems(dict(good, scope="x"), clean))
        self.assertTrue(P.approval_problems(dict(good, commit="def"), clean))
        self.assertTrue(P.approval_problems(good, dict(clean, dirty=" M x")))
        self.assertTrue(P.approval_problems(dict(good, approved_by=""), clean))
        self.assertTrue(P.approval_problems([], clean))
        self.assertTrue(P.inside_git_tree(ROOT / "tools"))
        self.assertFalse(P.inside_git_tree(self.tmp))

    def test_execute_is_single_use_and_refuses_before_cuda(self):
        approval = self.tmp / "approval.json"
        approval.write_text(json.dumps({"approved_by": "u", "approved_at": "t", "scope": P.SCOPE, "commit": "abc"}))
        clean = {"head": "abc", "dirty": ""}
        never = mock.patch.object(P, "measure", side_effect=AssertionError("measure must not run"))   # review r2 N5
        f = self.tmp / "f.pth"
        f.write_bytes(b"F")
        pins = [mock.patch.object(P, "verify_inputs", lambda: {}), mock.patch.object(P, "_f_checkpoint", lambda: f),
                mock.patch.object(P, "F_SHA", P.sha256_file(f))]             # each refusal is for its own reason
        for pin in pins:
            pin.start()
        self.addCleanup(lambda: [pin.stop() for pin in pins])

        def refused(argv, reason, **kw):
            import contextlib
            import io
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                rc = P.main(argv, git=clean, **kw)
            self.assertEqual(rc, 2, out.getvalue())
            self.assertIn(reason, out.getvalue())
        run_dir = self.tmp / "gpu_run01"
        with never, mock.patch.dict(os.environ, {"CUDA_VISIBLE_DEVICES": "0"}):
            run_dir.mkdir()                                                         # a first attempt existed
            self.assertEqual(P.main(["--execute", "--approval-file", str(approval)], run_dir=run_dir, git=clean,
                                    faults=lambda: 0), 2)
            self.assertEqual(list(run_dir.iterdir()), [])
            run_dir.rmdir()
            with self.assertRaises(SystemExit):                                     # no free output path
                P.main(["--execute", "--approval-file", str(approval), "--out", str(self.tmp / "r.json")],
                       run_dir=run_dir, git=clean)
            self.assertEqual(P.main(["--execute", "--approval-file", str(approval)], run_dir=ROOT / "gpu_run01",
                                    git=clean, faults=lambda: 0), 2)                # inside a Git tree
            self.assertFalse((ROOT / "gpu_run01").exists())
            def unreadable():
                raise RuntimeError("journal")
            self.assertEqual(P.main(["--execute", "--approval-file", str(approval)], run_dir=run_dir, git=clean,
                                    faults=unreadable), 2)                          # refuses before the directory
            big = self.tmp / "big.json"
            big.write_text('{"approved_by": "u", "approved_at": "t", "scope": "%s", "commit": "abc", "x": 1e999}'
                           % P.SCOPE)
            self.assertEqual(P.main(["--execute", "--approval-file", str(big)], run_dir=run_dir, git=clean,
                                    faults=lambda: 0), 2)                           # review r3 N11
            for bad in ("-1", "\u0663", "0,1", " 0"):                        # one ASCII index only (r4 B4-l1)
                with mock.patch.dict(os.environ, {"CUDA_VISIBLE_DEVICES": bad}):
                    refused(["--execute", "--approval-file", str(approval)], "one GPU index", run_dir=run_dir,
                            faults=lambda: 0)
            with mock.patch.object(P, "verify_inputs", side_effect=ValueError("inputs differ")):
                refused(["--execute", "--approval-file", str(approval)], "inputs differ", run_dir=run_dir,
                        faults=lambda: 0)
            with mock.patch.object(P, "F_SHA", "0" * 64):                    # Policy F before the directory
                refused(["--execute", "--approval-file", str(approval)], "Policy F", run_dir=run_dir, faults=lambda: 0)
            run_dir.mkdir()
            refused(["--execute", "--approval-file", str(approval)], "single use", run_dir=run_dir, faults=lambda: 0)
            run_dir.rmdir()
            nan = self.tmp / "nan.json"
            nan.write_text('{"approved_by": "u", "approved_at": "t", "scope": "%s", "commit": "abc", "x": NaN}'
                           % P.SCOPE)
            self.assertEqual(P.main(["--execute", "--approval-file", str(nan)], run_dir=run_dir, git=clean,
                                    faults=lambda: 0), 2)
            self.assertFalse(run_dir.exists())
        with never, mock.patch.dict(os.environ, {"CUDA_VISIBLE_DEVICES": ""}):
            self.assertEqual(P.main(["--execute", "--approval-file", str(approval)], run_dir=run_dir, git=clean,
                                    faults=lambda: 0), 2)                           # CUDA hidden: not a GPU run
            self.assertFalse(run_dir.exists())

    def test_execute_accept_path_records_and_voids(self):
        approval = self.tmp / "approval.json"
        approval.write_text(json.dumps({"approved_by": "u", "approved_at": "t", "scope": P.SCOPE, "commit": "abc"}))
        clean = {"head": "abc", "dirty": ""}
        fake = mock.patch.object(P, "measure", lambda gpu, out_npz: {"reading": "TF32_EXPLAINS_POLICY_RESIDUAL",
                                                                     "density_readings": {"70": "x"}, "gpu": gpu})
        pins = mock.patch.object(P, "verify_inputs", lambda: {})
        watchdog = mock.patch.object(P, "_rss_watchdog", lambda: None)
        with fake, watchdog, pins, mock.patch.dict(os.environ, {"CUDA_VISIBLE_DEVICES": "0"}):
            run_dir = self.tmp / "ok_run"
            self.assertEqual(P.main(["--execute", "--approval-file", str(approval)], run_dir=run_dir, git=clean,
                                    faults=lambda: 0), 0)
            attempt = json.loads((run_dir / "attempt.json").read_text())
            result = json.loads((run_dir / "result.json").read_text())
            self.assertEqual((attempt["kernel_faults_start"], result["reading"], result["gpu"]),
                             (0, "TF32_EXPLAINS_POLICY_RESIDUAL", True))
            self.assertEqual(len(result["approval"]["approval_file"]["sha256"]), 64)
            self.assertEqual(P.main(["--execute", "--approval-file", str(approval)], run_dir=run_dir, git=clean,
                                    faults=lambda: 0), 2)                           # the same approval again: refused
            counts = iter([0, 1])
            rise = self.tmp / "rise_run"
            self.assertEqual(P.main(["--execute", "--approval-file", str(approval)], run_dir=rise, git=clean,
                                    faults=lambda: next(counts)), 0)
            result = json.loads((rise / "result.json").read_text())
            self.assertEqual((result["reading"], result["reading_if_valid"]),
                             ("VOID_HOST_FAULT", "TF32_EXPLAINS_POLICY_RESIDUAL"))
            self.assertEqual((result["density_readings"], result["density_readings_if_valid"]), (None, {"70": "x"}))
            calls = iter([0])

            def then_unreadable():
                try:
                    return next(calls)
                except StopIteration:
                    raise RuntimeError("journal gone") from None
            lost = self.tmp / "lost_run"
            self.assertEqual(P.main(["--execute", "--approval-file", str(approval)], run_dir=lost, git=clean,
                                    faults=then_unreadable), 0)
            result = json.loads((lost / "result.json").read_text())                # still written (review r2 N1)
            self.assertEqual(result["reading"], "VOID_HOST_FAULT")
            self.assertIn("journal gone", result["fault_check_error"])

    def test_rss_watchdog_aborts(self):
        code = ("import importlib.util, sys, time; s = importlib.util.spec_from_file_location('p', sys.argv[1]); "
                "p = importlib.util.module_from_spec(s); s.loader.exec_module(p); p._rss_watchdog(cap=int(sys.argv[2]), "
                "period=0.05); time.sleep(2); print('alive')")
        tool = str(ROOT / "tools/sitl_s3a_policy_precision.py")
        killed = subprocess.run([sys.executable, "-B", "-c", code, tool, "1"], capture_output=True, text=True,
                                timeout=60)
        self.assertEqual(killed.returncode, 3, killed.stderr)
        alive = subprocess.run([sys.executable, "-B", "-c", code, tool, str(8 * 1024 ** 3)], capture_output=True,
                               text=True, timeout=60)
        self.assertEqual((alive.returncode, alive.stdout.strip()), (0, "alive"))

    def test_refusals(self):
        with mock.patch.dict(os.environ, {"CUDA_VISIBLE_DEVICES": "0"}):
            with self.assertRaises(SystemExit):
                P.main(["--cpu-only", "--out", str(self.tmp / "r.json")])
        with self.assertRaises(SystemExit):
            P.main(["--execute", "--out", str(self.tmp / "r.json")])                       # no approval file
        with mock.patch.dict(os.environ, {"CUDA_VISIBLE_DEVICES": ""}):
            with self.assertRaises(SystemExit):
                P.main(["--cpu-only", "--out", str(ROOT / "r_inside_git.json")])           # inside a Git tree
        self.assertFalse((ROOT / "r_inside_git.json").exists())
        (self.tmp / "old.json").write_text("{}")
        with mock.patch.dict(os.environ, {"CUDA_VISIBLE_DEVICES": ""}):
            with self.assertRaises(SystemExit):
                P.main(["--cpu-only", "--out", str(self.tmp / "old.json")])                 # output must be new
        approval = self.tmp / "approval.json"
        approval.write_text(json.dumps({"approved_by": "x", "approved_at": "y", "scope": P.SCOPE, "commit": "0" * 40}))
        self.assertEqual(P.main(["--execute", "--approval-file", str(approval)], run_dir=self.tmp / "g1"), 2)
        approval.write_text(json.dumps({"approved_by": "x", "approved_at": "y", "scope": "other",
                                        "commit": P.git_state()["head"]}))
        self.assertEqual(P.main(["--execute", "--approval-file", str(approval)], run_dir=self.tmp / "g2"), 2)
        self.assertFalse((self.tmp / "g1").exists() or (self.tmp / "g2").exists())

    @unittest.skipUnless(all((P.G8 / rel).is_file() for rel, _ in P.INPUTS.values()), "G8 r2 inputs not present")
    def test_cpu_preflight_on_the_stored_inputs(self):
        out = self.tmp / "result.json"
        env = dict(os.environ, CUDA_VISIBLE_DEVICES="", PYTHONNOUSERSITE="1", OMP_NUM_THREADS="1",
                   MKL_NUM_THREADS="1")
        p = subprocess.run([sys.executable, "-B", str(ROOT / "tools/sitl_s3a_policy_precision.py"), "--cpu-only",
                            "--out", str(out)], capture_output=True, text=True, env=env, timeout=600)
        self.assertEqual(p.returncode, 0, p.stderr[-2000:])
        result = json.loads(out.read_text())
        self.assertEqual(result["reading"], "CPU_ONLY")
        for bars in ("70", "205"):
            m = result["densities"][bars]
            self.assertTrue(m["controls"]["ok"], m["controls"])
            self.assertEqual(m["cpu_vs_recorded"]["n"], 84 * 128 * 4)
            self.assertGreater(m["cpu_vs_recorded"]["max"], 1e-4)        # the known residual is present
            self.assertLess(m["cpu_vs_recorded"]["max"], 1e-2)
            self.assertNotIn("gpu_on_vs_recorded", m)
        self.assertTrue(Path(result["actions_npz"]["path"]).is_file())
        self.assertEqual(result["kernel_faults_start"], result["kernel_faults_end"])
        self.assertTrue(result["loaded_module_files"])

    @unittest.skipUnless(all((P.G8 / rel).is_file() for rel, _ in P.INPUTS.values()), "G8 r2 inputs not present")
    def test_gpu_branch_with_the_cpu_standing_in(self):
        """The GPU code path end to end, with "cpu" as the device: never faithful to the GPU recording, and the
        precision switches have no effect on CPU arithmetic."""
        out = self.tmp / "smoke.json"
        code = ("import importlib.util, json, sys; s = importlib.util.spec_from_file_location('p', sys.argv[1]); "
                "p = importlib.util.module_from_spec(s); s.loader.exec_module(p); "
                "r = p.measure(gpu=True, gpu_device='cpu'); open(sys.argv[2], 'w').write(json.dumps(r))")
        env = dict(os.environ, CUDA_VISIBLE_DEVICES="", PYTHONNOUSERSITE="1", OMP_NUM_THREADS="1",
                   MKL_NUM_THREADS="1")
        p = subprocess.run([sys.executable, "-B", "-c", code, str(ROOT / "tools/sitl_s3a_policy_precision.py"),
                            str(out)], capture_output=True, text=True, env=env, timeout=600)
        self.assertEqual(p.returncode, 0, p.stderr[-2000:])
        result = json.loads(out.read_text())
        self.assertEqual(result["reading"], "GPU_REPLAY_NOT_FAITHFUL")
        self.assertIsNone(result["density_readings"])                         # no per-density cause (r3 N12)
        for bars in ("70", "205"):
            m = result["densities"][bars]
            self.assertEqual(m["gpu_on_vs_gpu_off"]["max"], 0.0)
            self.assertEqual(m["gpu_off_vs_cpu"]["max"], 0.0)                    # same construction, same numbers
            self.assertEqual(m["navrl_env_gpu"], m["navrl_env_cpu"])
            self.assertEqual((m["flags_off"]["float32_matmul_precision"], m["flags_on"]["float32_matmul_precision"]),
                             ("highest", "high"))


if __name__ == "__main__":
    unittest.main()
