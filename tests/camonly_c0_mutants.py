"""Known-bad controls for the C0 tests (tests/test_navrl_sensor_ablation.py): each mutant breaks one property of the
ablation module; the test suite must FAIL on every mutant and PASS on the unmodified module.
Mutants assoc_raw and no_elevation_fix were added after review r1 (B1, M1); the source mutants replica_on_masked,
raw_branch_no_mask and k2_in_raw_branch after review r2 (M-r2-1).
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
    A.governor_scan = lambda perception, condition, scan_m, max_range_m, *args, **kwargs: scan_m


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


def m_assoc_raw():
    """the LiDAR target association still reads the raw LiDAR (review r1 B1)."""
    orig = ORIG["install_perception"]

    def install_perception(module, condition):
        cls = orig(module, condition)
        assoc = cls._associate_lidar_target

        def associate(self, lidar_m, *args, **kwargs):
            return assoc(self, getattr(self, "_ablation_raw_lidar_m", lidar_m), *args, **kwargs)
        cls._associate_lidar_target = associate
        return cls
    A.install_perception = install_perception


def m_no_elevation_fix():
    """camera values copied into the governor rows without the 1/cos(elevation) correction (review r1 M1)."""
    def governor_scan(perception, condition, scan_m, max_range_m, vertical_fov_deg):
        removed = getattr(perception, "_ablation_removed", None)
        if removed is None or not bool(removed.any()):
            return scan_m
        inside = A.fov_inside(perception)
        out = scan_m.clone()
        rin, rout = removed & inside, removed & ~inside
        out[:, :, rin] = perception._ablation_fused_m[:, :, rin].to(out.dtype)
        out[:, :, rout] = float(max_range_m)
        return out
    A.governor_scan = governor_scan


SOURCE = (TREE / "aerial_gym/task/navrl_task/navrl_sensor_ablation.py").read_text()
SOURCE_MUTANTS = {
    # review r2 M-r2-1: the obs_only governor
    "replica_on_masked": ('        raw = getattr(self, "_ablation_raw_lidar_m", None)\n',
                          '        raw = bound.arguments["lidar_m"]\n'),
    "raw_branch_no_mask": ("            kwargs = dict(kwargs, target_return_mask=perception._ablation_raw_target_like)\n",
                           "            kwargs = dict(kwargs, target_return_mask=None)\n"),
    "k2_in_raw_branch": ("            return orig_dlc(scan_m, bearings_rad, command_xy, **kwargs)\n        scan = governor_scan",
                         "            c = orig_dlc(scan_m, bearings_rad, command_xy, **kwargs)\n"
                         "            return torch.where(out_of_fov_commands(perception, command_xy), torch.zeros_like(c), c)\n"
                         "        scan = governor_scan"),
}


def source_module(name):
    """A copy of the ablation module with one source edit (the edit must match exactly once)."""
    old, new = SOURCE_MUTANTS[name]
    assert SOURCE.count(old) == 1, name
    import types
    mod = types.ModuleType("navrl_sensor_ablation_mutant_" + name)
    exec(compile(SOURCE.replace(old, new), mod.__name__, "exec"), mod.__dict__)
    return mod


def torch_zeros(command_xy):
    import torch
    return torch.zeros(command_xy.shape[0], dtype=torch.bool)


MUTANTS = {"none": None, "no_mask": m_no_mask, "governor_raw": m_governor_raw, "no_k2": m_no_k2,
           "fill_zero": m_fill_zero, "fov_shifted": m_fov_shifted, "assoc_raw": m_assoc_raw,
           "no_elevation_fix": m_no_elevation_fix}


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
for name in SOURCE_MUTANTS:
    T.ABL = source_module(name)
    out[name] = run_suite()
T.ABL = A
ok = (out["none"]["failures"] == 0 and out["none"]["errors"] == 0
      and all(out[m]["failures"] + out[m]["errors"] > 0 for m in out if m != "none"))
res = {"tree": str(TREE), "mutants": out, "pass": ok}
OUT.write_text(json.dumps(res, indent=1) + "\n")
print(json.dumps({k: (v["failures"] + v["errors"], v["failed_tests"]) for k, v in out.items()}, indent=1))
print("pass", ok)
raise SystemExit(0 if ok else 1)
