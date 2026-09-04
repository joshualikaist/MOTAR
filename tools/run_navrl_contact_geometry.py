"""Contact-corridor forensics: where is the governor's monitored region empty?

Prereg: docs/prereg_2026-09-04_contact_corridor_forensics.md. Reuses the distractor-envelope
launcher's CLOSED evaluation environment builder by import, exactly as the S1 and depth-order
screens do, so the checkpoint pin, camera contract, goal band and gate-0 checks are inherited
rather than re-typed.

The question: the stopcap screen made the stopping margin POSITIVE at contact (+0.395 m) and the
crash rate still ROSE. So at the moment of contact the governor believes it can stop, which means
the corridor it monitors does not contain what the vehicle hits. This decomposes those contacts.

Two arms, off and riskcap, because the composition of the blind spot may differ when the filter
is active.
"""

import importlib.util
import json
import os
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[1]
ENVELOPE = REPO / "tools" / "run_navrl_distractor_envelope.py"
SEED = int(os.environ.get("NAVRL_CG_SEED", "491"))
RESULT_ROOT = REPO / "results" / f"navrl_contact_geometry_seed{SEED}"
ENVELOPE_CELL = "v7_n0"          # zero distractors: this is about bars, not decoys
ARMS = {
    "off": {"NAVRL_SPEED_GOVERNOR": "off"},
    "riskcap": {"NAVRL_SPEED_GOVERNOR": "riskcap"},
}


def _load_envelope():
    spec = importlib.util.spec_from_file_location("distractor_envelope_for_cg", ENVELOPE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _require(cond, msg):
    if not cond:
        raise SystemExit(f"[contact-geom] FAIL: {msg}")


def _arm_env(mod, arm, out_dir):
    env = mod.evaluation_env(ENVELOPE_CELL, preflight=False)
    env["NAVRL_SEED"] = str(SEED)
    env["NAVRL_V2_RESULT_DIR"] = str(out_dir)
    env["NAVRL_V2_SHARED_SOURCE_BUNDLE"] = str(RESULT_ROOT / "source_bundle")
    env["NAVRL_CONTACT_GEOMETRY"] = "1"
    env["NAVRL_SPEED_GOVERNOR_DIAG"] = "1"   # the corridor is computed even when mode is off
    env.update(ARMS[arm])
    return env


def run_evaluate(mod, only=None, episodes=None):
    gate0 = mod.verify_prerequisites()
    _require(mod.gate0_static_passed(gate0),
             "envelope gate-0 failed: " + mod.gate0_failure_report(gate0))
    n = int(episodes or mod.EPISODES)
    for arm in (ARMS if only is None else [only]):
        out_dir = RESULT_ROOT / arm
        if out_dir.exists():
            print(f"[contact-geom] arm {arm}: exists, skipping")
            continue
        env = _arm_env(mod, arm, out_dir)
        print(f"[contact-geom] EVALUATE {arm} | seed {SEED} | {n} episodes")
        code = mod.tee_run(["bash", str(mod.EVALUATOR), str(mod.CHECKPOINT), str(n)],
                           env, out_dir.parent / f"{arm}.eval.log.partial")
        _require(code == 0, f"{arm}: evaluator exited {code}")
        _require(out_dir.is_dir(), f"{arm}: no result directory")
        (out_dir.parent / f"{arm}.eval.log.partial").replace(out_dir / "eval.log")


def _load_arm(arm, checkpoint_sha):
    path = RESULT_ROOT / arm / "70bars.json"
    _require(path.is_file(), f"missing {path}")
    r = json.loads(path.read_text())
    cond = r.get("condition") or {}
    _require(int(cond.get("seed", -1)) == SEED, f"{arm}: wrong seed")
    _require(cond.get("contact_geometry") is True, f"{arm}: forensics flag not recorded")
    _require(r.get("checkpoint_sha256") == checkpoint_sha, f"{arm}: checkpoint drifted")
    cg = r["contact_geometry"]
    # M2: the five categories must cover every contact exactly once.
    cmd = cg["commanded_direction"]
    total = (cg["contacts"] and sum(cmd[k] for k in
             ("vertical_out", "behind", "lateral", "no_return", "in_corridor")))
    _require(total == cg["contacts"],
             f"{arm}: categories sum to {total} but {cg['contacts']} contacts -- VOID")
    return {"arm": arm, "outcome": r["outcome"], "contact_geometry": cg,
            "governor_mode": cond.get("speed_governor_mode"), "path": str(path)}


def run_summarize(mod):
    rows = [_load_arm(a, mod.CHECKPOINT_SHA) for a in ARMS]
    lines = ["# Contact-corridor forensics (seed 491, 0 distractors, 70 bars)", "",
             "| arm | crash | contacts | vertical_out | behind | lateral | no_return | **in_corridor** |",
             "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for r in rows:
        c, cm = r["contact_geometry"], r["contact_geometry"]["commanded_direction"]
        f = lambda x: "-" if x is None else f"{x:.1%}"
        lines.append(
            f"| {r['arm']} | {r['outcome']['crash_rate']:.2%} | {c['contacts']:,} | "
            f"{f(cm['vertical_out_rate'])} | {f(cm['behind_rate'])} | {f(cm['lateral_rate'])} | "
            f"{f(cm['no_return_rate'])} | **{f(cm['in_corridor_rate'])}** |")
    lines += ["", "## 가설 대조", "",
              "| arm | C: 실제속도 기준 in_corridor | C: 재분류 수 | 평균 명령-실제 각차 | D: 회랑 내 막대 2개+ | 평균 막대 수 |",
              "|---|---:|---:|---:|---:|---:|"]
    for r in rows:
        c = r["contact_geometry"]
        a = c["actual_velocity_direction"]
        f = lambda x: "-" if x is None else f"{x:.1%}"
        lines.append(
            f"| {r['arm']} | {f(a['in_corridor_rate'])} | {c['hypothesis_c_reclassified_into_corridor']:+,} | "
            f"{c['mean_cmd_vs_actual_deg']:.1f}° | {f(c['hypothesis_d_rate'])} | "
            f"{c['mean_bars_in_corridor']:.2f} |")
    # A1 replication gate, frozen in docs/prereg_2026-09-05_a1_forensics_replication.md
    GATE = {"off": (0.725, 0.819), "riskcap": (0.728, 0.834)}
    verdict = None
    if SEED != 491:
        passed = []
        lines += ["", "## A1 재현 판정 (seed 491 CI 게이트, 결과 이전 동결)", "",
                  "| arm | lateral+no_return | 허용 구간 | |", "|---|---:|---|---|"]
        for r in rows:
            cm = r["contact_geometry"]["commanded_direction"]
            v = cm["lateral_rate"] + cm["no_return_rate"]
            lo, hi = GATE[r["arm"]]
            ok = lo <= v <= hi
            passed.append(ok)
            lines.append(f"| {r['arm']} | {v:.1%} | [{lo:.1%}, {hi:.1%}] | {'통과' if ok else '벗어남'} |")
        verdict = ("REPLICATED" if all(passed) else
                   "PARTIAL" if any(passed) else "FAILED")
        lines += ["", f"**verdict_replication: {verdict}**", ""]
    lines.append("")
    RESULT_ROOT.mkdir(parents=True, exist_ok=True)
    (RESULT_ROOT / "summary.json").write_text(
        json.dumps({"schema_version": 1, "seed": SEED,
                    "prereg": "docs/prereg_2026-09-04_contact_corridor_forensics.md",
                    "policy_checkpoint_sha256": mod.CHECKPOINT_SHA,
                    "verdict_replication": verdict, "arms": rows},
                   indent=2, sort_keys=True) + "\n")
    (RESULT_ROOT / "summary.md").write_text("\n".join(lines))
    print("\n".join(lines))


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else ""
    mod = _load_envelope()
    if mode == "evaluate":
        run_evaluate(mod, sys.argv[2] if len(sys.argv) > 2 else None,
                     int(sys.argv[3]) if len(sys.argv) > 3 else None)
    elif mode == "summarize":
        run_summarize(mod)
    else:
        raise SystemExit(f"usage: {sys.argv[0]} evaluate [arm] [episodes]|summarize")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
