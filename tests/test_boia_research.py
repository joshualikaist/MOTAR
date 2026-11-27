"""Independent exact-surrogate arithmetic; no existing BOIA function is an oracle."""
import ast
from dataclasses import fields, replace
import hashlib
import math
from pathlib import Path
import random
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval.boia_research.benefit import paired_bound
from motar_eval.boia_research.decision import decide, initial_state, opportunity_window
from motar_eval.boia_research.evidence import build_hypotheses, validate_probability
from motar_eval.boia_research.types import (
    BoundContract, ErrorAllowances, EvidenceSet, LossTable, MeasuredEvidence,
    Option, ProbabilityInterval, Provenance, ResearchCase, ResearchState,
    SwitchPolicy, UtilityValidity,
    SafetyContext, SafetyEvidence, SafetyProvenance,
)


KEY = ("envA", "episode1", 7)
DOMAIN = "EXACT_SYNTHETIC_ROOM"
LABELS = ("FOREGROUND_OCCLUDER", "TARGET_SURFACE")
ZERO = ErrorAllowances(0., 0., 0., 0., 0.)


def safety_context(key=KEY):
    return SafetyContext(key, 1.02, "synthetic-sensor-v1", "synthetic-boundary-v1",
                         1., 1., "exact-planar-test-v1")


def safety_evidence(candidate_id, wait_s=0., key=KEY):
    # Explicit exhaustive synthetic fixture, not inferred from boolean/utility.
    return SafetyEvidence(key, candidate_id, wait_s, "synthetic-sensor-v1",
                          "synthetic-boundary-v1", 1., 1., 1.02, 1.2, True, True,
                          wait_s + .1, "exact-wait-and-prefix", "exact-complete-backup",
                          "exact-planar-test-v1", "NOMINAL_CPU_MODEL_ONLY",
                          SafetyProvenance("EXACT_SYNTHETIC", "finite fixture proof",
                                           "exact-planar-test-v1", True))


def proof(**changes):
    return replace(Provenance("EXACT_SYNTHETIC", "enumerated two-world surrogate fixture",
                              DOMAIN, True, True), **changes)


def measured(**changes):
    return replace(MeasuredEvidence(KEY, (5.8, 5.8), (5.8, 5.8), 1., 1.01, 1.02,
                                   ProbabilityInterval(1., 1., proof()), proof(),
                                   True, True, True), **changes)


def evidence(key=KEY):
    return build_hypotheses(measured(key=key), DOMAIN)


def losses(values, key=KEY, scenarios=("surface_world", "foreground_world"), paths=("atom0",)):
    return LossTable(key, scenarios, paths, values)


def bound(candidate_id="A", wait_s=0., gain=1., key=KEY, switching=0., **changes):
    # Exactly two worlds and one equally weighted atom/world. Exhaustive only in
    # this constructed finite model; zero errors make NO empirical calibration claim.
    contract = BoundContract(proof(), DOMAIN, "ENUMERATED", True, LABELS)
    result = paired_bound(candidate_id, wait_s, losses(((10.,), (1.,)), key),
                          losses(((10. - gain,), (1. - gain,)), key),
                          contract, replace(ZERO, switching=switching))
    return replace(result, **changes)


def option(candidate_id="A", wait_s=0., gain=1., key=KEY, safe=True, **changes):
    return replace(Option(candidate_id, wait_s, safe,
                          UtilityValidity(True, "EXACT_SYNTHETIC_DECLARATION", DOMAIN),
                          bound(candidate_id, wait_s, gain, key),
                          safety_evidence(candidate_id, wait_s, key)), **changes)


def case(options=(), key=KEY, **changes):
    return replace(ResearchCase(key, "F", True, tuple(options), evidence(key),
                                (0., .2, .4), (True, True),
                                SwitchPolicy(.2, .1, 2, 2),
                                safety_context=safety_context(key),
                                baseline_safety_evidence=safety_evidence("F", key=key)), **changes)


def choose(c, state=None):
    return decide(c, state or initial_state(c.key[0], c.key[1], "F"))


def advance(c, decision_id):
    key = (c.key[0], c.key[1], decision_id)
    return replace(c, key=key, evidence=replace(c.evidence, key=key),
                   options=tuple(replace(o, benefit=replace(o.benefit, key=key),
                                         safety_evidence=replace(o.safety_evidence, key=key)) for o in c.options),
                   safety_context=replace(c.safety_context, key=key),
                   baseline_safety_evidence=replace(c.baseline_safety_evidence, key=key))


class TestBoiaResearch(unittest.TestCase):
    def test_01_identical_measurement_different_worlds(self):
        # Hidden evaluation labels live only in test and are never algorithm inputs.
        worlds = ({"actual_world": "sphere"}, {"actual_world": "bar"})
        results = [build_hypotheses(measured(), DOMAIN) for _ in worlds]
        self.assertEqual(results[0], results[1])
        self.assertEqual(results[0].labels, LABELS)
        self.assertEqual(choose(case([option()])), choose(case([option()])))

    def test_02_foreground_target_first_hit_is_preserved(self):
        overlapping = evidence()
        self.assertEqual(overlapping.first_hits_m, ((5.8, 5.8),))
        self.assertEqual(overlapping.labels, LABELS)
        foreground = build_hypotheses(measured(first_hit_m=(5.2, 5.3)), DOMAIN)
        self.assertEqual(foreground.labels, ("FOREGROUND_OCCLUDER",))
        self.assertEqual(foreground.first_hits_m, ((5.2, 5.3),))

    def test_03_incorrect_association_retained_and_uncovered_option_refused(self):
        ambiguous = build_hypotheses(measured(association=ProbabilityInterval(.0, .2, proof())), DOMAIN)
        self.assertIn("FALSE_ASSOCIATION", ambiguous.labels)
        self.assertTrue(ambiguous.valid)
        self.assertEqual(choose(case([option()], evidence=ambiguous)).outcome, "ABSTAIN")
        self.assertFalse(choose(case([option()], evidence=ambiguous)).window.valid_options)

    def test_04_missing_evidence_abstains(self):
        out = choose(case([option()], evidence=None))
        self.assertEqual((out.outcome, out.selected_id), ("ABSTAIN", "F"))

    def test_05_unsupported_calibration_abstains(self):
        unsupported = build_hypotheses(measured(association=ProbabilityInterval(.9, 1.,
                                       proof(kind="UNAVAILABLE", supported=False))), DOMAIN)
        self.assertIn("UNKNOWN_OR_UNOBSERVED", unsupported.labels)
        self.assertEqual(choose(case([option()], evidence=unsupported)).outcome, "ABSTAIN")

    def test_06_positive_robust_paired_benefit(self):
        got = bound(switching=.2)
        self.assertEqual(got.scenario_means, (1., 1.))
        self.assertAlmostEqual(got.lower_bound, .8)
        self.assertEqual(choose(case([option(benefit=got)])).outcome, "INTERVENE")

    def test_07_negative_worst_world_refuses_intervention(self):
        got = paired_bound("A", 0., losses(((10.,), (1.,))),
                           losses(((9.,), (3.,))),
                           BoundContract(proof(), DOMAIN, "ENUMERATED", True, LABELS), ZERO)
        self.assertEqual(got.scenario_means, (1., -2.))
        self.assertEqual(got.lower_bound, -2.)
        self.assertEqual(choose(case([option(benefit=got)])).outcome, "KEEP_F")

    def test_08_switching_cost_reverses_positive_gain(self):
        got = bound(switching=1.2)
        self.assertAlmostEqual(got.lower_bound, -.2)
        self.assertEqual(choose(case([option(benefit=got)])).outcome, "KEEP_F")

    def test_09_early_opportunity_changes_reactive_decision(self):
        c = case([option("A", 0., 3.), option("A", .2, -1.), option("A", .4, -2.)])
        out = choose(c)
        # Independent reactive baseline only acts after loss: visible now => KEEP_F.
        visible_now = True
        reactive = "KEEP_F" if visible_now else "INTERVENE"
        self.assertEqual(reactive, "KEEP_F")
        self.assertEqual(out.outcome, "INTERVENE")
        self.assertEqual(out.window.current_window_s, (0.,))
        self.assertTrue(out.window.closing)

    def test_10_late_opportunity_changes_immediate_threshold(self):
        c = case([option("A", 0., 1.), option("A", .2, 3.), option("A", .4, -1.)])
        # Same safety/info/surrogate: immediate expected-gain rule chooses gain1 now.
        immediate_threshold = "INTERVENE" if 1. > c.policy.tau_on else "KEEP_F"
        out = choose(c)
        self.assertEqual(immediate_threshold, "INTERVENE")
        self.assertEqual((out.outcome, out.window.selected_wait_s), ("KEEP_F", .2))
        self.assertEqual(out.reason, "WAIT_FOR_LATER_OPPORTUNITY")
        empty_now = choose(case([option("A", 0., -1.), option("A", .4, 3.)]))
        self.assertEqual(empty_now.window.current_window_s, ())
        self.assertEqual(empty_now.window.reason, "EMPTY_CURRENT_LATE_ADVISORY")

    def test_11_unsafe_wait_prefix_cannot_be_bridged(self):
        out = choose(case([option("A", 0., 1.), option("B", .4, 5.)],
                          wait_interval_safe=(False, True)))
        self.assertEqual((out.outcome, out.selected_id), ("INTERVENE", "A"))
        self.assertEqual(out.window.supported_times_s, (0.,))

    def test_12_safe_f_unsafe_high_gain_alternative(self):
        self.assertEqual(choose(case([option(gain=100., safe=False)])).outcome, "KEEP_F")

    def test_13_unsafe_f_current_viable_backup_precedes_utility(self):
        backup = option(utility=UtilityValidity(False, "UNCALIBRATED", DOMAIN))
        out = choose(case([backup], baseline_safe=False, evidence=None, backup_id="A"))
        self.assertEqual((out.outcome, out.selected_id), ("SAFETY_OVERRIDE", "A"))
        self.assertFalse(out.physical_qualified)

    def test_14_no_certified_backup_is_failure_not_stop_success(self):
        out = choose(case([option(safe=False)], baseline_safe=False, backup_id="OLD"))
        self.assertEqual(out.outcome, "NO_CERTIFIED_BACKUP")
        self.assertIsNone(out.selected_id)
        # Yesterday's owner/backup alone is not today's declaration.
        old = ResearchState("envA", "episode1", "OLD", 5, 0)
        self.assertIsNone(choose(case([], baseline_safe=False, backup_id="OLD"), old).selected_id)

    def test_15_endgame_and_no_search_do_not_bypass_safety(self):
        self.assertEqual(choose(case([option(gain=100.)], endgame=True)).outcome, "KEEP_F")
        self.assertEqual(choose(case([option()], acquired=False)).reason, "NO_INITIAL_SEARCH")
        self.assertEqual(choose(case([option()], endgame=True, baseline_safe=False)).outcome,
                         "SAFETY_OVERRIDE")

    def test_16_candidate_permutation_and_wait_ties(self):
        opts = (option("B", .2), option("B", 0.), option("A", 0.))
        a, b = choose(case(opts)), choose(case(tuple(reversed(opts))))
        self.assertEqual(a, b)
        self.assertEqual((a.selected_id, a.window.selected_wait_s), ("A", 0.))

    def test_17_environment_reset_isolation_and_terminal_no_loss_read(self):
        ca, cb = case([option()]), case([option("B", key=("envB", "ep2", 7))], key=("envB", "ep2", 7))
        sa, sb = initial_state("envA", "episode1", "F"), initial_state("envB", "ep2", "F")
        before = (choose(ca, sa), choose(cb, sb))
        terminal_a = replace(ca, terminal=True, evidence=None, options=None)
        self.assertEqual(choose(terminal_a, before[0].new_state).reason, "TERMINAL_RESET_NO_ACTION")
        after_b = choose(cb, sb)
        self.assertEqual(before[1], after_b)
        self.assertEqual(tuple(choose(c, s) for c, s in ((cb, sb), (ca, sa))), tuple(reversed(before)))
        self.assertEqual(sa.owner_id, "F")
        new_episode = replace(ca, key=("envA", "episode2", 0))
        self.assertEqual(choose(new_episode, sa).reason, "EXPLICIT_STATE_RESET_REQUIRED")

    def test_18_gt_schema_import_firewall_and_no_actions(self):
        package = Path(__file__).resolve().parents[1] / "motar_eval" / "boia_research"
        permitted = {"math", "dataclasses", "typing"}
        forbidden_names = {"gt", "ground_truth", "task", "simulator", "evaluator", "action", "cmd"}
        for path in package.glob("*.py"):
            tree = ast.parse(path.read_text())
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    self.assertTrue(all(a.name in permitted for a in node.names), path)
                if isinstance(node, ast.ImportFrom):
                    self.assertTrue(node.level == 1 or node.module in permitted, path)
                if isinstance(node, (ast.Name, ast.Attribute)):
                    name = node.id if isinstance(node, ast.Name) else node.attr
                    self.assertNotIn(name.lower(), forbidden_names, path)
        for record in (MeasuredEvidence, ResearchCase, ResearchState):
            self.assertFalse(forbidden_names.intersection(f.name for f in fields(record)))
        out = choose(case([option()]))
        self.assertFalse(any("action" in f.name or "command" in f.name for f in fields(out)))
        self.assertEqual(out.scope, "CPU_RESEARCH_ONLY")
        self.assertFalse(out.physical_qualified)

    def test_19_pairing_is_scenario_path_context_exact_not_shape_only(self):
        base = losses(((10.,), (1.,)))
        c = BoundContract(proof(), DOMAIN, "ENUMERATED", True, LABELS)
        variants = (replace(base, key=("other", "episode1", 7)),
                    replace(base, path_ids=("different_atom",)),
                    replace(base, scenario_ids=tuple(reversed(base.scenario_ids))))
        for other in variants:
            result = paired_bound("A", 0., base, other, c, ZERO)
            self.assertFalse(result.valid)
            self.assertIn("UNPAIRED", result.reason)

    def test_20_paired_regret_not_independent_worst_case(self):
        # Independent max losses: max(F)=10, max(A)=9 -> gain1 here; change second
        # world to F1/A0 still gain1. Comparing min(F)=1 to max(A)=9 is invalid -8.
        self.assertEqual(bound().lower_bound, 1.)
        wrong_unpaired = min(10., 1.) - max(9., 0.)
        self.assertEqual(wrong_unpaired, -8.)

    def test_21_missing_allowances_and_f_continuation_refuse(self):
        base, candidate = losses(((10.,), (1.,))), losses(((9.,), (0.,)))
        c = BoundContract(proof(), DOMAIN, "ENUMERATED", True, LABELS)
        for name in ("mc", "set_coverage", "calibration", "continuation", "switching"):
            result = paired_bound("A", 0., base, candidate, c, replace(ZERO, **{name: None}))
            self.assertFalse(result.valid)
        result = paired_bound("A", 0., base, candidate,
                              replace(c, provenance=proof(continuation_covered=False)), ZERO)
        self.assertEqual(result.reason, "F_CONTINUATION_MISMATCH_UNBOUNDED")

    def test_22_mc_bound_counts_all_options_scenarios_and_independent_paths(self):
        # Hoeffding for |d|<=D: D*sqrt(2*log(2*KJ*B/alpha)/M).
        n = 100
        base = losses((tuple([2.] * n), tuple([2.] * n)), paths=tuple("p%d" % i for i in range(n)))
        other = replace(base, values=(tuple([1.] * n), tuple([1.] * n)))
        p = proof(kind="HELD_OUT_DECLARATION", reference="test-only held-out declaration")
        c = BoundContract(p, DOMAIN, "IID_PATHS", True, LABELS, False, 6, .05, 1.)
        eps = math.sqrt(2. * math.log(2. * 6. * 2. / .05) / 100.)
        good = paired_bound("A", 0., base, other, c, replace(ZERO, mc=eps))
        self.assertTrue(good.valid)
        self.assertAlmostEqual(good.lower_bound, 1. - eps)
        bad = paired_bound("A", 0., base, other, c, replace(ZERO, mc=eps * .99))
        self.assertEqual(bad.reason, "MC_ALLOWANCE_OMITS_SIMULTANEOUS_SELECTION")
        # A bound formed for one tested option cannot be reused across two choices.
        one_c = replace(c, tested_options=1)
        one_eps = math.sqrt(2. * math.log(2. * 1. * 2. / .05) / 100.)
        one = paired_bound("A", 0., base, other, one_c, replace(ZERO, mc=one_eps))
        good_b = replace(good, candidate_id="B")
        out = choose(case([option(benefit=one), option("B", benefit=good_b)]))
        self.assertEqual(out.selected_id, "B")
        refused = paired_bound("A", 0., base, other,
                               replace(c, exhaustive=False, set_coverage_proven=False),
                               replace(ZERO, mc=eps))
        self.assertEqual(refused.reason, "FINITE_SCENARIOS_ONLY_NOT_A_COVERAGE_BOUND")
        huge_count = paired_bound("A", 0., base, other, replace(c, tested_options=10 ** 1000),
                                  replace(ZERO, mc=eps))
        self.assertEqual(huge_count.reason, "INVALID_SELECTION_FAMILY")

    def test_23_nan_huge_int_overflow_are_option_local(self):
        good = option("B", gain=2.)
        for bad_value in (float("nan"), float("inf"), 10 ** 1000):
            invalid = option("A", benefit=replace(bound(), lower_bound=bad_value))
            self.assertEqual(choose(case([invalid, good])).selected_id, "B")
        c = BoundContract(proof(), DOMAIN, "ENUMERATED", True, LABELS)
        huge_losses = losses(((1.e308, 1.e308), (1.e308, 1.e308)), paths=("p0", "p1"))
        zero_losses = replace(huge_losses, values=((0., 0.), (0., 0.)))
        overflow = paired_bound("A", 0., huge_losses, zero_losses, c, ZERO)
        self.assertFalse(overflow.valid)
        self.assertEqual(choose(case([option(benefit=overflow), good])).selected_id, "B")

    def test_24_domain_mismatch_is_option_local(self):
        a = option("A", gain=10., utility=UtilityValidity(True, "WRONG_DOMAIN", "unseen"))
        b = option("B", gain=2.)
        self.assertEqual(choose(case([a, b])).selected_id, "B")
        a = option("A", gain=10., benefit=replace(bound(), domain="unseen"))
        self.assertEqual(choose(case([a, b])).selected_id, "B")

    def test_25_hysteresis_never_retains_harmful_unsafe_or_unsupported_owner(self):
        state = ResearchState("envA", "episode1", "A", 1, 0)
        low = case([option(gain=.05)])
        held = choose(low, state)
        self.assertEqual((held.selected_id, held.new_state.dwell), ("A", 2))
        released = choose(advance(low, 8), held.new_state)
        self.assertEqual(released.selected_id, "F")
        for o in (option(gain=-1.), option(safe=False),
                  option(utility=UtilityValidity(False, "MISSING", DOMAIN))):
            self.assertEqual(choose(case([o]), state).selected_id, "F")

    def test_26_no_rng_mutation_repeatability_and_immutable_inputs(self):
        c = case([option()])
        state = initial_state("envA", "episode1", "F")
        before = random.getstate()
        fingerprint = hashlib.sha256(repr((c, state)).encode()).hexdigest()
        first, second = decide(c, state), decide(c, state)
        self.assertEqual(first, second)
        self.assertEqual(random.getstate(), before)
        self.assertEqual(hashlib.sha256(repr((c, state)).encode()).hexdigest(), fingerprint)

    def test_27_probability_timestamp_grid_flags_fail_closed(self):
        for interval in (ProbabilityInterval(-.1, .5, proof()), ProbabilityInterval(.9, 1.1, proof()),
                         ProbabilityInterval(None, None, proof())):
            self.assertFalse(validate_probability(interval, DOMAIN).valid)
        stale = build_hypotheses(measured(now_s=2.), DOMAIN)
        self.assertFalse(stale.valid)
        self.assertEqual(choose(case([option()], wait_interval_safe=(True,))).outcome, "ABSTAIN")
        self.assertEqual(choose(case([option(safe="UNKNOWN")], baseline_safe="UNKNOWN")).outcome,
                         "NO_CERTIFIED_BACKUP")

    def test_28_unsupported_comparison_abstains_without_poisoning_siblings(self):
        invalid = option(benefit=replace(bound(), valid=False, reason="CALIBRATION_UNAVAILABLE"))
        self.assertEqual(choose(case([invalid])).outcome, "ABSTAIN")
        self.assertEqual(choose(case([invalid])).reason, "NO_UTILITY_SUPPORTED_COMPARISON")
        self.assertEqual(choose(case([invalid, option("B", gain=2.)])).selected_id, "B")
        unsafe = choose(case([replace(invalid, safety_valid=False)]))
        self.assertEqual((unsafe.outcome, unsafe.reason), ("KEEP_F", "NO_SAFE_ALTERNATIVE"))

    def test_29_conflicting_duplicate_backup_ids_cannot_certify_one_choice(self):
        out = choose(case([option(), option(safe=False)], baseline_safe=False, backup_id="A"))
        self.assertEqual(out.outcome, "NO_CERTIFIED_BACKUP")

    def test_30_duplicate_and_stale_ids_do_not_advance_state_or_reuse_proof(self):
        c = case([option()])
        first = choose(c)
        for stale in (c, advance(c, 6)):
            out = choose(stale, first.new_state)
            self.assertEqual((out.outcome, out.selected_id), ("NO_CERTIFIED_BACKUP", None))
            self.assertEqual(out.reason, "STALE_DECISION_NO_CURRENT_CERTIFICATE")
            self.assertEqual(out.new_state, first.new_state)
        self.assertEqual(choose(advance(c, 8), first.new_state).new_state.last_decision_id, 8)
        # A terminal reset can clear this row even if its ID is stale/invalid.
        out = choose(replace(c, terminal=True, key=("envA", "episode1", -1), options=None), first.new_state)
        self.assertIsNone(out.new_state.last_decision_id)
        for bad_step in (-1, True, .5):
            bad = choose(replace(c, key=("envA", "episode1", bad_step)))
            self.assertEqual(bad.reason, "INVALID_DECISION_ID_NO_CURRENT_CERTIFICATE")
            self.assertIsNone(bad.selected_id)


if __name__ == "__main__":
    unittest.main()
