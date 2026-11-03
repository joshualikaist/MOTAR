"""Bounded CPU regressions for R3 F1-F5; not a full R2 gate. P0R_SRC selects the source under test."""
import json
import inspect
import pathlib
import sys
import unittest
import warnings
from unittest import mock

import numpy as np

from harness import CATALOG, bar, camera, classify, core, header, scan

COUNTS = {"height_cases": 0, "valid_height_certificates": 0, "baseline_false_certificate": 0,
          "callback_positive_controls": 0}
CREATED_CORES = []


def make_core(config):
    c = core.CertifierCore(config, 0, 0)
    CREATED_CORES.append(c)
    return c


def packet(step=1, z=1.0, layout="P0R-72x1", objects=()):
    pose = (0.0, 0.0, z, 0.0)
    r, s, _ = scan(list(objects), pose, layout)
    return r, s, *header(step, pose)


def ready():
    c = make_core("SYNTH-72x1")
    assert c.step(*packet())["accepted"]
    return c


class RepairChecks(unittest.TestCase):
    def test_height_eligibility_and_positive_controls(self):
        p = np.array([[[0.0, 0.0], [0.3, 0.0]]])
        for cfg, layout in [("SYNTH-72x1", "P0R-72x1"), ("SYNTH-72x4", "MOTAR-72x4")]:
            for z in [-0.05, 0.0, 0.05, 1.0, 2.95, 3.0, 3.05, 3.5]:
                with self.subTest(cfg=cfg, z=z):
                    c = make_core(cfg)
                    rep = c.step(*packet(z=z, layout=layout))
                    self.assertTrue(rep["accepted"])
                    out = c.certify(p, 1)[0]
                    COUNTS["height_cases"] += 1
                    if 0.0 < z < 3.0:
                        self.assertGreater(rep["free_rays"], 0)
                        self.assertTrue(out["static_bar_certified"], out)
                        COUNTS["valid_height_certificates"] += 1
                    else:
                        self.assertEqual(rep["free_rays"], 0)
                        self.assertFalse(out["static_bar_certified"], out)
        self.assertGreater(COUNTS["valid_height_certificates"], 0)

    def test_planar_bar_top_false_certificate(self):
        scene = [bar(1.5, 0.0, *CATALOG[0])]
        p = np.array([[[0.0, 0.0], [2.5, 0.0]]])
        for cfg in ["SYNTH-72x1", "SYNTH-72x1-NOASSURE"]:
            with self.subTest(cfg=cfg):
                c = make_core(cfg)
                rep = c.step(*packet(z=3.05, objects=scene))
                self.assertTrue(rep["complete"])
                self.assertEqual(rep["hits"], 0)
                out = c.certify(p, 1)[0]
                cl = classify(out, p[0], scene, z_vehicle=3.05)
                COUNTS["baseline_false_certificate"] += int(cl == "FALSE_CERT")
                self.assertNotEqual(cl, "FALSE_CERT", out)
                self.assertFalse(out["static_bar_certified"], out)

    def test_numeric_hooks_suppressed_and_settings_restored(self):
        c = ready()
        calls = []
        def callback(*args, **kwargs):
            calls.append("callback")
        olderr, oldcall = np.geterr(), np.geterrcall()
        oldshow, oldfilters = warnings.showwarning, warnings.filters[:]
        try:
            np.seterrcall(callback)
            np.seterr(all="call")
            np.array([1e-170]) ** 2
            self.assertGreater(len(calls), 0)
            COUNTS["callback_positive_controls"] += len(calls)
            calls.clear()
            cam = camera(0.2, (0.0, 0.0, 1.0, 0.0), (1.5, 0.0, 1.0))
            cam[12] = 1.7e308
            self.assertTrue(c.step(*packet(step=2), camera=cam)["accepted"])
            paths = np.array([[[0.0, 0.0], [1e-170, 0.0]], [[0.0, 0.5], [0.0, 2.0]]])
            out = c.certify(paths, 2)
            self.assertTrue(all(x["valid_for_step"] == 2 for x in out))
            self.assertTrue(all(x["static_bar_certified"] for x in out), out)
            self.assertEqual(len(c.exceptions), 0)
            self.assertIsNone(c.map_states(np.array([[1e308, 0.0]]), 0.2))
            self.assertEqual(calls, [])
            self.assertEqual(np.geterr(), dict.fromkeys(olderr, "call"))
            self.assertIs(np.geterrcall(), callback)
            np.seterr(all="warn")
            warnings.simplefilter("always")
            warnings.showwarning = callback
            warnings.warn("test callback positive control", RuntimeWarning)
            self.assertGreater(len(calls), 0)
            COUNTS["callback_positive_controls"] += len(calls)
            calls.clear()
            filt = warnings.filters[:]
            cam[0] = 0.3
            self.assertTrue(c.step(*packet(step=3), camera=cam)["accepted"])
            self.assertEqual(calls, [])
            self.assertEqual(warnings.filters, filt)
            self.assertIs(warnings.showwarning, callback)
            np.seterr(all="raise")
            self.assertIsNone(c.map_states(np.array([[1e308, 0.0]]), 0.3))
            self.assertIsNone(c.map_states(np.array([[np.nan, 0.0]]), 0.3))
            self.assertIsNone(c.map_states(np.array([[0.0, 0.0]]), float("nan")))
            self.assertIsInstance(c.map_states(np.array([[0.0, 0.0]]), 0.3), list)
            self.assertEqual(np.geterr(), dict.fromkeys(olderr, "raise"))
        finally:
            np.seterr(**olderr)
            np.seterrcall(oldcall)
            warnings.showwarning = oldshow
            warnings.filters[:] = oldfilters

    def test_result_is_bound_to_captured_packet(self):
        c = ready()
        def internal_state_change(*args):
            # Trusted test instrumentation, not a claim about supporting caller monkeypatching.
            c._current = None
            return None, 0
        with mock.patch.object(c.map, "veto_near", side_effect=internal_state_change):
            out = c.certify(np.array([[[0.0, 0.0], [0.3, 0.0]]]), 1)[0]
        self.assertTrue(out["static_bar_certified"])
        self.assertEqual(out["valid_for_step"], 1)
        self.assertNotEqual(out["evidence_label"], "NONE")

    def test_keyword_binding_does_not_redispatch_caller_protocol(self):
        stacks = []
        class Keyword(str):
            __hash__ = str.__hash__
            def __eq__(self, other):
                stacks.append([f.function for f in inspect.stack()])
                return str.__eq__(self, other)
        self.assertTrue(Keyword("camera") == "camera")
        self.assertGreater(len(stacks), 0)
        stacks.clear()
        c = ready()
        self.assertTrue(c.step(*packet(step=2), **{Keyword("camera"): None})["accepted"])
        out = c.certify(np.array([[[0.0, 0.0], [0.3, 0.0]]]), **{Keyword("for_step"): 2})
        self.assertTrue(out[0]["static_bar_certified"])
        self.assertIsInstance(c.map_states(np.array([[0.0, 0.0]]), **{Keyword("now_t"): 0.2}), list)
        # Initial Python binding occurs in the caller; a second bind inside the numeric core is forbidden.
        forbidden = {"guarded", "step", "certify", "map_states", "_step", "_certify", "_map_states"}
        self.assertFalse(any(forbidden.intersection(stack) for stack in stacks), stacks)

    def test_owned_copy_rechecked_and_private(self):
        b = core.B
        a = np.zeros((1, 72), np.float64)
        out = b.exact_copy(a, b.F64, a.shape)
        self.assertFalse(np.shares_memory(out, a))
        self.assertFalse(out.flags.writeable)
        self.assertTrue(out.flags.c_contiguous)
        paths = np.zeros((1, 2, 2))
        oldarray = np.array
        def shape_change(*args, **kwargs):
            x = oldarray(*args, **kwargs)
            x.shape = (x.size,)
            return x
        with mock.patch.object(b.np, "array", side_effect=shape_change):
            self.assertIsNone(b.exact_copy(a, b.F64, a.shape))
            self.assertIsNone(b.exact_paths(paths, 64))
        def dtype_change(*args, **kwargs):
            return oldarray(*args, **kwargs).astype(np.float32)
        with mock.patch.object(b.np, "array", side_effect=dtype_change):
            self.assertIsNone(b.exact_copy(a, b.F64, a.shape))
            self.assertIsNone(b.exact_paths(paths, 64))

    def test_late_certificate_is_not_emitted(self):
        c = ready()
        p = np.array([[[0.0, 0.0], [0.3, 0.0]]])
        self.assertTrue(c.certify(p, 1)[0]["static_bar_certified"])
        # Deterministic simulated computation overrun: no sleep or wall-time performance claim.
        with mock.patch.object(core.time, "perf_counter", side_effect=[0.0, 0.0, 1.0]):
            out = c.certify(p, 1, deadline_s=0.001)[0]
        self.assertFalse(out["static_bar_certified"])
        self.assertEqual(out["reason"], "DEADLINE_UNKNOWN")


if __name__ == "__main__":
    output = pathlib.Path(sys.argv[1])
    if output.exists():
        raise SystemExit("Refusing to overwrite existing evidence")
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(RepairChecks))
    COUNTS["internal_exceptions"] = sum(len(c.exceptions) for c in CREATED_CORES)
    passed = result.wasSuccessful() and COUNTS["internal_exceptions"] == 0
    output.write_text(json.dumps({"source": str(pathlib.Path(core.__file__).resolve()), "tests": result.testsRun,
                                 "failures": len(result.failures), "errors": len(result.errors),
                                 "pass": passed, "counts": COUNTS}, indent=2) + "\n")
    raise SystemExit(0 if passed else 1)
