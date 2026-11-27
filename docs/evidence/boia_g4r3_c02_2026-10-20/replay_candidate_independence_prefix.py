"""Replay archived pre-fix A-F in an isolated CPU process; write no files.

From the worktree root:
CUDA_VISIBLE_DEVICES='' PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B docs/evidence/boia_g4r3_c02_2026-10-20/replay_candidate_independence_prefix.py

The source snapshot was uncommitted, so a base Git checkout alone cannot replay
it. This loader verifies that snapshot and its unchanged dependencies, executes
it as a separate relative-import module, and swaps only the fixture module's
local binding. Production modules/files are never changed. Fixture functions
are SHA-bound individually, allowing unrelated tests to be added later.
"""
import ast
import hashlib
import importlib
import json
from pathlib import Path
import sys
import types

import torch

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
manifest = json.loads((HERE / "candidate_independence_prefix_source_manifest.json").read_text())
digest = lambda raw: hashlib.sha256(raw).hexdigest()
for path in ("motar_eval/boia/types.py", "motar_eval/boia/geometry.py",
             "motar_eval/boia/dynamics.py", "tests/test_boia_c02_safety.py"):
    if digest((ROOT / path).read_bytes()) != manifest[path]:
        raise RuntimeError("Archived replay dependency SHA mismatch: " + path)
raw = (HERE / "safety_prefix_candidate_fix.py.txt").read_bytes()
if digest(raw) != manifest["motar_eval/boia/safety.py"]:
    raise RuntimeError("Archived safety source SHA mismatch")
fixture_path = ROOT / "tests/test_boia_c02_candidate_independence.py"
fixture_source = fixture_path.read_text()
function_sha = {node.name: digest(ast.get_source_segment(fixture_source, node).encode())
                for node in ast.parse(fixture_source).body if isinstance(node, ast.FunctionDef)}
for name, expected in manifest["fixture_function_sha256"].items():
    if function_sha.get(name) != expected:
        raise RuntimeError("Archived replay fixture changed: " + name)
fixture = importlib.import_module("test_boia_c02_candidate_independence")
archived = types.ModuleType("motar_eval.boia._g4r3_archived_safety")
archived.__package__ = "motar_eval.boia"
archived.__file__ = str(HERE / "safety_prefix_candidate_fix.py.txt")
exec(compile(raw, archived.__file__, "exec"), archived.__dict__)
fixture.safety = archived
receipt = {"phase": "PRE_FIX_EXPECTED_FAILURE", "archived_safety_sha256": digest(raw),
           "status_encoding": {"MODEL_SAFE": 0, "MODEL_UNSAFE": 1, "UNKNOWN": 2, "INVALID": 3},
           "cases": {}}
names = ("status", "prefix_ok", "backup_ok", "legacy_hard_ok", "legacy_stop_ok",
         "prefix_min_clearance_m", "backup_min_clearance_m", "backup_distance_m")
for letter in "ABCDEF":
    data, expected = fixture.case(letter)
    out = fixture.evaluate(data)
    row = {"shape": list(out.status.shape), "status": out.status.tolist(),
           "expected_status": expected.tolist(), "required_membership_pass": bool(torch.equal(out.status, expected)),
           "observed_healthy_counts": (out.status == 0).sum(-1).tolist(),
           "expected_healthy_counts": (expected == 0).sum(-1).tolist()}
    if letter == "E":
        order = torch.tensor(list(range(19, -1, -2)) + list(range(18, -1, -2)))
        eo = torch.tensor([2, 0, 1])
        permuted = fixture.evaluate(fixture.permute(data, order, eo))
        restored = permuted.status[torch.argsort(eo)][:, torch.argsort(order)]
        row["status_permutation_invariance_alone"] = bool(torch.equal(restored, out.status))
        row["interpretation"] = "Pure permutation is invariant, but healthy membership is poisoned; not evidence of a permutation-only defect."
    if letter == "F":
        one = fixture.evaluate(fixture.select(data, 0, 3))
        row["healthy_single_status"] = int(one.status)
        row["same_candidate_batch_status"] = int(out.status[0, 3])
        equal = {}
        for name in names:
            a, b = getattr(out, name)[0, 3], getattr(one, name)[0, 0]
            equal[name] = bool(torch.isclose(a, b, rtol=0, atol=0, equal_nan=True))
        row["single_vs_batch_exact"] = equal
    receipt["cases"][letter] = row
print(json.dumps(receipt, indent=2, allow_nan=False))
