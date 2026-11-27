"""Standalone scalar arithmetic and decision contrasts; no simulator or old BOIA.

Run from repository root with pinned Python -B. Creates a NEW receipt exclusively.
All losses/safety declarations below are EXACT_SYNTHETIC, not mission forecasts.
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from motar_eval.boia_research.benefit import paired_bound
from motar_eval.boia_research.decision import decide, initial_state
from motar_eval.boia_research.types import (
    BoundContract, ErrorAllowances, EvidenceSet, LossTable, Option, Provenance,
    ResearchCase, SwitchPolicy, UtilityValidity,
)

KEY = ("independent-env", "independent-episode", 1)
DOMAIN = "EXACT_SYNTHETIC_INDEPENDENT"
LABELS = ("FOREGROUND_OCCLUDER", "TARGET_SURFACE")
PROOF = Provenance("EXACT_SYNTHETIC", "finite scalar world enumeration", DOMAIN, True, True)
CONTRACT = BoundContract(PROOF, DOMAIN, "ENUMERATED", True, LABELS)
ZERO = ErrorAllowances(0., 0., 0., 0., 0.)


def benefit(candidate_id, wait_s, base_values, option_values, cost=0.):
    scenes = tuple("world%d" % i for i in range(len(base_values)))
    base = LossTable(KEY, scenes, ("exact-atom",), tuple((x,) for x in base_values))
    other = replace(base, values=tuple((x,) for x in option_values))
    return paired_bound(candidate_id, wait_s, base, other, CONTRACT,
                        replace(ZERO, switching=cost))


def option(candidate_id, wait_s, gain, cost=0., safe=True):
    # Nonnegative input losses; independent expected gain is literal subtraction.
    b = benefit(candidate_id, wait_s, (2., 4.), (2. - gain, 4. - gain), cost)
    return Option(candidate_id, wait_s, safe,
                  UtilityValidity(True, "EXACT_SYNTHETIC", DOMAIN), b)


def case(options, **changes):
    return replace(ResearchCase(KEY, "F", True, tuple(options),
                    EvidenceSet(KEY, LABELS, ((5.8, 5.8),), True, (), DOMAIN),
                    (0., .2, .4), (True, True), SwitchPolicy(.1, .05)), **changes)


def choose(c):
    return decide(c, initial_state(KEY[0], KEY[1], "F"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    checks = []

    def check(name, actual, expected):
        ok = abs(actual - expected) < 1.e-12 if isinstance(expected, float) else actual == expected
        checks.append(dict(name=name, actual=actual, expected=expected, passed=ok))
        if not ok:
            raise AssertionError(checks[-1])

    robust = benefit("A", 0., (10., 1.), (9., 0.))
    check("paired_worst_scene_gain", robust.lower_bound, 1.)
    check("unpaired_cross_world_expression_is_different", min(10., 1.) - max(9., 0.), -8.)
    harmful = benefit("A", 0., (10., 1., 0.), (9., 0., 2.))
    check("added_adverse_world", harmful.lower_bound, -2.)
    switch = choose(case((option("A", 0., .4, cost=.5),)))
    check("switch_cost_net", switch.window.valid_options[0].benefit.lower_bound, -.1)
    check("switch_cost_reverses_intervention", switch.outcome, "KEEP_F")
    now, later = option("A", 0., .4), option("B", .2, .9)
    c = case((now, later))
    timed = choose(c)
    immediate = choose(case((now,), wait_grid_s=(0.,), wait_interval_safe=()))
    check("time_options_wait", (timed.outcome, timed.reason, timed.window.selected_wait_s),
          ("KEEP_F", "WAIT_FOR_LATER_OPPORTUNITY", .2))
    check("immediate_only_differs", immediate.outcome, "INTERVENE")
    check("unsafe_wait_cannot_authorize_later", choose(replace(c, wait_interval_safe=(False, True))).selected_id, "A")
    early = choose(case((now,)))
    check("pre_loss_positive_option", early.outcome, "INTERVENE")
    check("actual_loss_only_rule_while_currently_seen", "KEEP_F" if not False else "INTERVENE", "KEEP_F")
    abstain = choose(case((now,), evidence=None))
    override = choose(case((now,), baseline_safe=False, evidence=None, endgame=True))
    nobackup = choose(case((replace(now, safety_valid=False),), baseline_safe=False))
    check("missing_evidence_abstains_safe_F", abstain.outcome, "ABSTAIN")
    check("safety_overrides_utility_and_endgame", override.outcome, "SAFETY_OVERRIDE")
    check("no_backup_not_safe_stop", (nobackup.outcome, nobackup.selected_id), ("NO_CERTIFIED_BACKUP", None))
    stale = decide(c, timed.new_state)
    check("stale_proof_not_reused", (stale.outcome, stale.selected_id), ("NO_CERTIFIED_BACKUP", None))
    check("stale_does_not_advance_state", stale.new_state == timed.new_state, True)
    outcomes = sorted({x.outcome for x in (timed, immediate, abstain, override, nobackup)})
    check("all_five_outcomes", outcomes, sorted(("KEEP_F", "INTERVENE", "ABSTAIN", "SAFETY_OVERRIDE", "NO_CERTIFIED_BACKUP")))
    check("outputs_research_only", all(x.scope == "CPU_RESEARCH_ONLY" and not x.physical_qualified
                                       for x in (timed, immediate, abstain, override, nobackup)), True)

    # Pre-built tiny O=2,J=3 case; no model/safety generation, transfer, log/SHA cost.
    fresh = initial_state(KEY[0], KEY[1], "F")
    samples = []
    for _ in range(1000):
        t0 = time.perf_counter_ns()
        decide(c, fresh)
        samples.append((time.perf_counter_ns() - t0) / 1000.)
    ordered = sorted(samples)
    sources = sorted((ROOT / "motar_eval/boia_research").glob("*.py"))
    sources.append(ROOT / "tests/test_boia_research.py")
    receipt = dict(schema="motar.g4r3.research.independent-oracle.v1",
        scope="CPU_RESEARCH_ONLY", physical_qualified=False, actual_mission="NOT_TESTED",
        checks=checks, status="PASS", synthetic_decision_contrasts=2,
        timing=dict(samples=1000, options=2, wait_grid=3, median_us=statistics.median(samples),
                    p95_us=ordered[949], p99_us=ordered[989], max_observed_us=max(samples),
                    claim="finite prebuilt kernel only, not WCET or full planner latency"),
        source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    with open(args.output, "x") as stream:
        json.dump(receipt, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps(dict(status=receipt["status"], checks=len(checks), timing=receipt["timing"])))


if __name__ == "__main__":
    main()
