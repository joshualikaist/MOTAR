"""Known-bad controls for the C0 tests (tests/test_navrl_sensor_ablation.py): each mutant breaks one property of the
ablation module; the test suite must FAIL on every mutant and PASS on the unmodified module.
usage: tests/camonly_c0_mutants.py TREE OUT_JSON   (not collected by tools/run_tests.py: no test_ prefix)
"""
import importlib.util
import io
import json
import pathlib
import sys
import unittest

TREE, OUT = pathlib.Path(sys.argv[1]).resolve(), pathlib.Path(sys.argv[2])
sys.argv = sys.argv[:1]


def load_tests():
    spec = importlib.util.spec_from_file_location("c0_tests_mutant_run", TREE / "tests/test_navrl_sensor_ablation.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


T = load_tests()
A = T.ABL
ORIG = {k: getattr(A, k) for k in ("install_perception", "governor_scan", "out_of_fov_commands", "removed_bins",
                                   "CONDITIONS")}


def m_no_mask():
    """observe passes the raw LiDAR (perception leak)."""
    orig = ORIG["install_perception"]

    def install_perception(module, condition):
        return orig(module, "full")
    A.install_perception = install_perception


def m_governor_raw():
    """the governor ignores the condition and uses the raw LiDAR (governor leak)."""
    A.governor_scan = lambda perception, condition, scan_m, max_range_m: scan_m


def m_no_k2():
    """no 2.0 m/s cap for commands outside the FOV."""
    A.out_of_fov_commands = lambda perception, command_xy: torch_zeros(command_xy)


def m_fill_zero():
    """removed bins filled with 0 m instead of no return."""
    orig = ORIG["install_perception"]

    def install_perception(module, condition):
        cls = orig(module, "full")          # inner wrapper passes through; the fill below is the only masking
        wrapped = cls.observe

        def observe(self, *args, **kwargs):
            removed = ORIG["removed_bins"](self, condition)
            self._ablation_removed = removed
            if "lidar_m" in kwargs and bool(removed.any()):
                x = kwargs["lidar_m"].reshape(self.num_envs, module.VBEAMS, module.HBEAMS).clone()
                x[:, :, removed] = 0.0
                kwargs["lidar_m"] = x.reshape(kwargs["lidar_m"].shape)
            return wrapped(self, *args, **kwargs)
        cls.observe = observe
        return cls
    A.install_perception = install_perception


def m_fov_shifted():
    """removal mask shifted by one bin (FOV edge wrong)."""
    def removed_bins(perception, condition):
        return ORIG["removed_bins"](perception, condition).roll(1)
    A.removed_bins = removed_bins


def torch_zeros(command_xy):
    import torch
    return torch.zeros(command_xy.shape[0], dtype=torch.bool)


MUTANTS = {"none": None, "no_mask": m_no_mask, "governor_raw": m_governor_raw, "no_k2": m_no_k2,
           "fill_zero": m_fill_zero, "fov_shifted": m_fov_shifted}


def run_suite():
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(T.SensorAblationC0)
    buf = io.StringIO()
    res = unittest.TextTestRunner(stream=buf, verbosity=0).run(suite)
    bad = sorted({t.id().split(".")[-1] for t, _ in res.failures + res.errors})
    return {"ran": res.testsRun, "failures": len(res.failures), "errors": len(res.errors), "failed_tests": bad}


out = {}
for name, mutate in MUTANTS.items():
    for k, v in ORIG.items():
        setattr(A, k, v)
    if mutate:
        mutate()
    out[name] = run_suite()
for k, v in ORIG.items():
    setattr(A, k, v)
ok = (out["none"]["failures"] == 0 and out["none"]["errors"] == 0
      and all(out[m]["failures"] + out[m]["errors"] > 0 for m in MUTANTS if m != "none"))
res = {"tree": str(TREE), "mutants": out, "pass": ok}
OUT.write_text(json.dumps(res, indent=1) + "\n")
print(json.dumps({k: (v["failures"] + v["errors"], v["failed_tests"]) for k, v in out.items()}, indent=1))
print("pass", ok)
raise SystemExit(0 if ok else 1)
