"""R4 independent safety identity/time checks; no old safety function is an oracle."""
import ast
from dataclasses import replace
import hashlib
import importlib.util
from pathlib import Path
import random
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval.boia_research.decision import decide, initial_state
from motar_eval.boia_research.safety import validate_safety

# Existing explicit synthetic fixtures only; expected outcomes below are literals.
_spec = importlib.util.spec_from_file_location("r4_fixture_helpers", Path(__file__).with_name("test_boia_research.py"))
f = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(f)


def choose(c, state=None):
    return decide(c, state or initial_state(c.key[0], c.key[1], c.baseline_id))


class TestIndependentSafety(unittest.TestCase):
    def test_01_current_safety_stale_utility_may_override(self):
        o = f.option(benefit=replace(f.bound(), key=("envA", "episode1", 6)))
        out = choose(f.case((o,), baseline_safe=False))
        self.assertEqual((out.outcome, out.selected_id), ("SAFETY_OVERRIDE", "A"))

    def test_02_stale_safety_current_utility_refused(self):
        o = f.option(safety_evidence=replace(f.safety_evidence("A"), expires_at_s=1.01))
        out = choose(f.case((o,), baseline_safe=False))
        self.assertEqual((out.outcome, out.selected_id), ("NO_CERTIFIED_BACKUP", None))

    def test_03_candidate_binding(self):
        o = f.option(safety_evidence=f.safety_evidence("OTHER"))
        self.assertEqual(choose(f.case((o,), baseline_safe=False)).outcome, "NO_CERTIFIED_BACKUP")

    def test_04_episode_binding(self):
        o = f.option(safety_evidence=f.safety_evidence("A", key=("envA", "previous", 7)))
        self.assertEqual(choose(f.case((o,), baseline_safe=False)).outcome, "NO_CERTIFIED_BACKUP")

    def test_05_decision_binding(self):
        o = f.option(safety_evidence=f.safety_evidence("A", key=("envA", "episode1", 6)))
        self.assertEqual(choose(f.case((o,), baseline_safe=False)).outcome, "NO_CERTIFIED_BACKUP")

    def test_06_missing_safety_is_not_minted_from_boolean(self):
        o = f.option(safety_evidence=None)
        self.assertTrue(o.safety_valid)
        out = choose(f.case((o,), baseline_safe=False))
        self.assertEqual((out.outcome, out.selected_id), ("NO_CERTIFIED_BACKUP", None))

    def test_07_insufficient_F_current_alternative(self):
        out = choose(f.case((f.option(),), baseline_safety_evidence=None))
        self.assertEqual((out.outcome, out.selected_id), ("SAFETY_OVERRIDE", "A"))

    def test_08_insufficient_F_all_alternatives_unverified(self):
        out = choose(f.case((f.option(safety_evidence=None),), baseline_safety_evidence=None))
        self.assertEqual((out.outcome, out.selected_id), ("NO_CERTIFIED_BACKUP", None))

    def test_09_missing_utility_never_authorizes_F_or_blocks_backup(self):
        o = f.option(benefit=None, utility=None)
        self.assertEqual(choose(f.case((o,), baseline_safe=False, evidence=None)).outcome, "SAFETY_OVERRIDE")
        # In the utility path the same missing record refuses intervention, not safety.
        self.assertEqual(choose(f.case((o,))).outcome, "ABSTAIN")
        self.assertEqual(choose(f.case((), baseline_safe=False, evidence=None)).outcome, "NO_CERTIFIED_BACKUP")

    def test_10_normal_wait_and_intervene(self):
        a, b = f.option("A", gain=1.), f.option("B", .2, gain=2.)
        self.assertEqual(choose(f.case((a,))).outcome, "INTERVENE")
        out = choose(f.case((a, b)))
        self.assertEqual((out.outcome, out.reason), ("KEEP_F", "WAIT_FOR_LATER_OPPORTUNITY"))

    def test_11_order_independence(self):
        opts = (f.option("B", benefit=None), f.option("A", benefit=None))
        c = f.case(opts, baseline_safe=False)
        a, b = choose(c), choose(replace(c, options=tuple(reversed(opts))))
        self.assertEqual(a, b)
        self.assertEqual(a.selected_id, "A")

    def test_12_env_state_and_RNG_isolation(self):
        ca = f.case((f.option(),), baseline_safe=False)
        cb = f.case((f.option("B", key=("envB", "ep2", 7)),), key=("envB", "ep2", 7))
        sa, sb = initial_state("envA", "episode1", "F"), initial_state("envB", "ep2", "F")
        fingerprint = hashlib.sha256(repr((ca, cb, sa, sb)).encode()).hexdigest()
        rng = random.getstate()
        before = choose(cb, sb)
        choose(ca, sa)
        self.assertEqual(choose(cb, sb), before)
        self.assertEqual(random.getstate(), rng)
        self.assertEqual(hashlib.sha256(repr((ca, cb, sa, sb)).encode()).hexdigest(), fingerprint)

    def test_13_env_binding_and_version_binding(self):
        for changes in (dict(key=("other", "episode1", 7)), dict(sensor_version="old"),
                        dict(boundary_version="old"), dict(model_id="different"),
                        dict(sensor_timestamp_s=.99), dict(boundary_timestamp_s=.99)):
            o = f.option(safety_evidence=replace(f.safety_evidence("A"), **changes))
            self.assertEqual(choose(f.case((o,), baseline_safe=False)).outcome, "NO_CERTIFIED_BACKUP")

    def test_14_time_nan_future_measurement_and_age(self):
        for changes in (dict(issued_at_s=1.03), dict(expires_at_s=float("nan")),
                        dict(issued_at_s=float("inf")), dict(sensor_timestamp_s=True)):
            o = f.option(safety_evidence=replace(f.safety_evidence("A"), **changes))
            self.assertEqual(choose(f.case((o,), baseline_safe=False)).outcome, "NO_CERTIFIED_BACKUP")
        c = f.case((f.option(),), baseline_safe=False)
        aged = replace(c.safety_context, now_s=1.21)
        self.assertEqual(choose(replace(c, safety_context=aged)).outcome, "NO_CERTIFIED_BACKUP")

    def test_15_prefix_backup_refs_qualification_and_provenance(self):
        for changes in (dict(prefix_safe=False), dict(backup_feasible=False),
                        dict(prefix_horizon_s=.09), dict(prefix_reference=""),
                        dict(backup_reference=""), dict(model_qualification="PHYSICAL_CERTIFICATE"),
                        dict(provenance=None),
                        dict(provenance=replace(f.safety_evidence("A").provenance, supported=False))):
            o = f.option(safety_evidence=replace(f.safety_evidence("A"), **changes))
            self.assertEqual(choose(f.case((o,), baseline_safe=False)).outcome, "NO_CERTIFIED_BACKUP")

    def test_16_baseline_also_requires_fresh_proof_before_keep_wait_endgame(self):
        for changes in (dict(baseline_safety_evidence=None), dict(safety_context=None),
                        dict(baseline_safety_evidence=replace(f.safety_evidence("F"), candidate_id="A"))):
            c = f.case((), endgame=True, **changes)
            out = choose(c)
            self.assertEqual((out.outcome, out.selected_id), ("NO_CERTIFIED_BACKUP", None))

    def test_17_future_proof_covers_wait_and_issuance_age(self):
        a, b = f.option("A"), f.option("B", .2, gain=4.)
        b = replace(b, safety_evidence=replace(b.safety_evidence, prefix_horizon_s=.1))
        self.assertEqual(choose(f.case((a, b))).selected_id, "A")
        # An aged prediction needs its unelapsed wait + action, not original horizon only.
        ctx = f.safety_context()
        cert = replace(f.safety_evidence("A"), issued_at_s=1.01, prefix_horizon_s=.1)
        self.assertFalse(validate_safety(ctx, f.KEY, "A", 0., cert).valid)

    def test_18_boolean_veto_and_future_not_current_backup(self):
        self.assertEqual(choose(f.case((f.option(safe=False),), baseline_safe=False)).outcome, "NO_CERTIFIED_BACKUP")
        self.assertEqual(choose(f.case((f.option(wait_s=.2),), baseline_safe=False)).outcome, "NO_CERTIFIED_BACKUP")

    def test_19_unsafe_or_unknown_candidate_cannot_win_utility(self):
        bad = f.option("A", gain=100., safety_evidence=None)
        good = f.option("B", gain=1.)
        self.assertEqual(choose(f.case((bad, good))).selected_id, "B")

    def test_20_safety_helpers_never_read_utility_or_privileged_records(self):
        root = Path(__file__).resolve().parents[1] / "motar_eval/boia_research"
        for name in ("safety.py",):
            tree = ast.parse((root / name).read_text())
            for node in ast.walk(tree):
                if isinstance(node, ast.Attribute):
                    self.assertNotIn(node.attr, ("benefit", "utility", "target_position", "ground_truth", "task"))
        tree = ast.parse((root / "decision.py").read_text())
        for fn in tree.body:
            if isinstance(fn, ast.FunctionDef) and fn.name in ("_current_safe", "_option_safe", "_baseline_safe"):
                self.assertFalse(any(isinstance(x, ast.Attribute) and x.attr in ("benefit", "utility") for x in ast.walk(fn)))


if __name__ == "__main__":
    unittest.main()
