"""Phase 6 non-interference checks for the Q02-B/Q03 CPU shadow package (structural and CPU-only).

They complement test_q02b_timing.FirewallTests (import isolation of motar_eval/q02, no action in a record, global RNG
untouched). Frozen F action non-interference is checked structurally here: no production module references the shadow
package and the shadow package cannot mutate its input. A run-time parity check on the simulator is NOT run (GPU).
"""
import ast
import dataclasses
import hashlib
import inspect
import os
from pathlib import Path
import re
import subprocess
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tests"))
import test_q02b_timing as qb
from motar_eval.q02 import obstacle_map as om, pipeline as pl, reacq_planner as rp, validation as va

PRODUCTION = ("aerial_gym", "rl_games", "resources")
GT_TOKENS = re.compile(r"\b(target_position|target_pos_gt|obstacle_positions|env_asset\w*|segmentation_mask|semantic_id|"
                       r"gt_visibility|true_target)\b")


def digest(records):
    rows = [repr(dataclasses.replace(d, runtime_s=None)) for d in records]
    return hashlib.sha256("\n".join(rows).encode()).hexdigest()


class NonInterferenceTests(unittest.TestCase):
    def test_production_code_does_not_reference_the_shadow_package(self):
        hits = []
        for top in PRODUCTION:
            for path in (ROOT / top).rglob("*.py") if (ROOT / top).is_dir() else ():
                text = path.read_text(errors="replace")
                if "motar_eval.q02" in text or "motar_eval/q02" in text or "from motar_eval import q02" in text:
                    hits.append(str(path.relative_to(ROOT)))
        self.assertEqual(hits, [])

    def test_shadow_package_imports_no_simulator_learning_or_rng_module(self):
        for path in sorted((ROOT / "motar_eval/q02").glob("*.py")):
            tree = ast.parse(path.read_text())
            mods = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
            mods |= {(n.module or "").split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.level == 0}
            with self.subTest(path=path.name):
                self.assertFalse(mods & {"torch", "isaacgym", "aerial_gym", "warp", "random", "rl_games", "secrets"})

    def test_input_is_not_mutated_and_cannot_be(self):
        rec = pl.ShadowRecorder(); rec.reset(1)
        inp = qb.frame(0, **qb.SEEN)
        before = {f: np.array(getattr(inp, f)).tobytes() for f in inp.__dataclass_fields__ if isinstance(getattr(inp, f), np.ndarray)}
        self.assertTrue(before)
        rec.observe(1, 0, inp, qb.ORACLE_PROXY)
        for f, b in before.items():
            arr = getattr(inp, f)
            self.assertEqual(arr.tobytes(), b, f); self.assertFalse(arr.flags.writeable, f)

    def test_ground_truth_firewall_in_signatures_and_sources(self):
        allowed = {
            pl.ShadowRecorder.observe: {"self", "episode", "decision", "inp", "provenance", "terminal"},
            om.LocalObstacleMap.integrate: {"self", "inp", "report"},
            rp.plan: {"m", "position", "yaw", "velocity", "belief", "now_s", "camera", "limits", "params"},
        }
        for fn, names in allowed.items():
            self.assertEqual(set(inspect.signature(fn).parameters), names, fn.__qualname__)
        for path in sorted((ROOT / "motar_eval/q02").glob("*.py")):
            with self.subTest(path=path.name):
                self.assertIsNone(GT_TOKENS.search(path.read_text()))
        m = om.LocalObstacleMap()
        self.assertFalse(m.validity(0.0)["ground_truth_used"])

    def test_records_are_identical_across_fresh_processes_and_global_rng_states(self):
        code = ("import sys; sys.path[:0]=[%r,%r]; import numpy as np; np.random.seed(int(sys.argv[1]));"
                "import test_q02b_timing as qb, test_q02b_q03_noninterference as t;"
                "print(t.digest(qb.run([qb.SEEN]*4+[qb.OCC]*10+[qb.SEEN]*3, policy='H3')))") % (str(ROOT), str(ROOT / "tests"))
        outs = []
        for seed, hashseed in ((1, "0"), (987654, "12345")):
            env = dict(os.environ, PYTHONHASHSEED=hashseed, CUDA_VISIBLE_DEVICES="")
            r = subprocess.run([sys.executable, "-B", "-c", code, str(seed)], capture_output=True, text=True, env=env, timeout=300)
            self.assertEqual(r.returncode, 0, r.stderr[-2000:])
            outs.append(r.stdout.strip().splitlines()[-1])
        self.assertEqual(outs[0], outs[1])
        self.assertEqual(outs[0], digest(qb.run([qb.SEEN] * 4 + [qb.OCC] * 10 + [qb.SEEN] * 3, policy="H3")))


if __name__ == "__main__":
    unittest.main()
