"""Run unchanged R4 assertions against exact archived R3 decision behavior.

Uses the additive current record schema/explicit fixtures; the old selector simply
ignores those new safety records. Neither test assertions nor simulator are patched.
Only the tested decide callable is replaced. Retain its known negative result.
"""
import argparse
import hashlib
import importlib.machinery
import importlib.util
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    evidence = Path(__file__).resolve().parent
    archived = evidence / "pre_r1_decision.py.txt"
    manifest = json.loads((evidence / "pre_r1_source_manifest.json").read_text())
    actual = hashlib.sha256(archived.read_bytes()).hexdigest()
    assert actual == manifest["motar_eval/boia_research/decision.py"]["sha256"]
    name = "motar_eval.boia_research._r4_legacy_decision"
    loader = importlib.machinery.SourceFileLoader(name, str(archived))
    old = importlib.util.module_from_spec(importlib.util.spec_from_loader(name, loader))
    loader.exec_module(old)
    test_path = ROOT / "tests/test_boia_research_safety.py"
    spec = importlib.util.spec_from_file_location("r4_pre_r1_test", test_path)
    test = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(test)
    test.decide = old.decide
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(test.TestIndependentSafety))
    assert result.failures or result.errors, "negative control did not reproduce a defect"
    receipt = dict(schema="motar.g4r4.r1.negative.v1", selector_sha256=actual,
                   test_sha256=hashlib.sha256(test_path.read_bytes()).hexdigest(),
                   ran=result.testsRun, failures=len(result.failures), errors=len(result.errors),
                   failure_details=[dict(id=t.id(), traceback=msg) for t, msg in result.failures],
                   error_details=[dict(id=t.id(), traceback=msg) for t, msg in result.errors],
                   status="PRE_FIX_FAIL_REPRODUCED", additive_schema_fixture_only=True,
                   safety_module_static_test_scope="Checks frozen current helper imports; negative behaviors use exact old selector")
    with open(args.output, "x") as stream:
        json.dump(receipt, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({k: receipt[k] for k in ("status", "ran", "failures", "errors")}))


if __name__ == "__main__":
    main()
