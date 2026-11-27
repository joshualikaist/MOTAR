"""Deterministic CPU shadow scenario benchmark for F0 / H1 / H2 / H3 / BOIA (Q02-B + Q03 v0). CPU only, read-only.

Usage: python -B docs/evidence/q02b_q03_cpu_package_2026-10-22/shadow_benchmark.py <out.json> [<example_records.json>]
Scenes come from the Q02-A production-faithful synthetic helper (tests/test_q02_evidence.py): static vehicle, scripted
target/bars per decision. Each scenario declares, per decision, whether an intervention would be warranted
(benchmark_metrics.LABEL_SEMANTICS). These labels are fixture declarations, not simulator outcomes. Policies only
PROPOSE in shadow; BOIA has no utility estimate and abstains. The Q01-V2 packets are replayed read-only as
single-decision episodes.
"""
import json, os, sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tests"))
import test_q02_evidence as q2a
from motar_eval.q02 import contracts as c, shadow as sh
from motar_eval.q02.benchmark_metrics import Row, summarize
from motar_eval.q02.history import ORACLE_PROXY
from motar_eval.q02.pipeline import ShadowRecorder

BAR = ((4.0, 0.0, 0.3),)
SEEN, OCC = dict(detected=True), dict(bars=BAR)
RING = tuple((0.9 * np.cos(a), 0.9 * np.sin(a), 0.25) for a in np.linspace(0, 2 * np.pi, 16, endpoint=False))
BOX = dict(target_v=(4.0, 0.2, 0.0), detected=True, depth=q2a.box_target_scene((4.0, 0.2, 0.0)))
DWELL = 5          # decisions (0.5 s) after loss onset before an intervention is declared warranted


def warranted_after(n_seen, n_total, dwell=DWELL):
    return [k >= n_seen + dwell for k in range(n_total)]


def scenarios():
    s = {}
    s["S1_brief_occlusion"] = ([SEEN] * 6 + [OCC] * 3 + [SEEN] * 10, None)
    s["S2_long_occlusion_detour"] = ([SEEN] * 6 + [OCC] * 25, warranted_after(6, 31))
    s["S3_long_occlusion_no_plan"] = ([SEEN] * 6 + [dict(bars=RING + BAR)] * 25, None)
    # Target at 3 m moves left at 3 m/s (0.3 m per decision, within the tracker continuity gate) and leaves the 87 deg
    # FOV (|y| > 3 tan 43.5 deg = 2.85 m) after 10 decisions; the camera flag follows visibility.
    exit_ = [dict(target_v=(3.0, 0.3 * (k + 1), 0.0), detected=0.3 * (k + 1) < 2.8) for k in range(24)]
    first_lost = 6 + next(k for k, f in enumerate(exit_) if not f["detected"])
    s["S4_target_leaves_fov"] = ([dict(target_v=(3.0, 0.0, 0.0), detected=True)] * 6 + exit_, warranted_after(first_lost, 30))
    # Fixture teleport (the first S4 draft, kept as a negative case): the track jumps 6 m in one decision -> identity suspect.
    s["S4b_target_teleport"] = ([SEEN] * 6 + [dict(target_v=(2.0, 6.0, 0.0))] * 20, None)
    s["S5_detector_miss_clear_los"] = ([SEEN] * 6 + [dict(render_target=False)] * 15, None)
    s["S6_sensor_dropout"] = ([SEEN] * 6 + [dict(cam_clock=c.Clock(None, None))] * 8 + [SEEN] * 6, None)
    s["S7_never_acquired"] = ([dict(render_target=False)] * 15, None)
    s["S8_ambiguous_reappearance"] = ([SEEN] * 6 + [OCC] * 8 + [BOX] * 8,
                                      [6 + DWELL <= k < 14 for k in range(22)])   # occlusion phase as S2; reappearance not
    s["S9_occlusion_growing_covariance"] = ([SEEN] * 6 + [dict(bars=BAR, cov_pos=0.01 + 0.08 * k) for k in range(20)],
                                            warranted_after(6, 26))
    return {k: (v[0], v[1] or [False] * len(v[0])) for k, v in s.items()}


def frame(k, **kw):
    clock = c.Clock(0.1 * k, 0.1 * k, "DECLARED_SIM_SCHEDULE")
    kw.setdefault("cam_clock", clock); kw.setdefault("lidar_clock", clock)
    return q2a.make(decision=0.1 * k, **kw)


def run(policy, name, script, labels, keep=None):
    rec = ShadowRecorder(policy); rec.reset(1); rows = []
    for k, kw in enumerate(script):
        d, cands = rec.observe(1, k, frame(k, **kw), ORACLE_PROXY)
        ms = d.local_map_status or {}
        rows.append(Row(name, policy, k, d.decision_s, d.decision, d.reasons, d.evidence_hypothesis, d.identity_verdict,
                        bool(ms.get("accepted")), bool(ms.get("stale")), d.plan_status,
                        int((d.candidate_summary or {}).get("l2_pass_with_view_gain", 0)),
                        d.state_after in (sh.LOSS_SUSPECTED, sh.REACQUIRE_PLANNED), bool(labels[k]), float(d.runtime_s)))
        if keep is not None and len(keep) < 6 and (d.decision == "PROPOSE_REACQUIRE_SHADOW" or k in (5, 12)):
            feats = dict((f.name, f.value) for f in d.features)
            keep.append(dict(scenario=name, policy=policy, time_s=d.decision_s, belief=feats["relative_target_position"],
                             position_uncertainty=feats["position_uncertainty"], evidence_hypothesis=d.evidence_hypothesis,
                             identity_verdict=d.identity_verdict, geometry_support=d.geometry_support,
                             detector_provenance=d.detector_provenance, local_map=d.local_map_status,
                             candidate_summary=d.candidate_summary, state=(d.state_before, d.state_after),
                             decision=d.decision, reasons=d.reasons, owner=d.owner, action=d.action,
                             delta_u=d.delta_u.status, runtime_s=round(d.runtime_s, 4),
                             best_candidate=None if not cands else max(
                                 ((x.candidate_id, x.l1_observed_direction_open, x.l2_kinematic_check, x.l3_dynamic_feasibility,
                                   x.l4_closed_loop_safety, round(x.visibility_score, 3), round(x.visibility_blocked_share, 3),
                                   round(x.travel_time_s, 3)) for x in cands), key=lambda t: (t[2] == "PASS", -t[6]))))
    return rows


def q01_replay():
    root = Path("/home/fair/workspaces/aerial_gym_ws/boia_shadow_runs/BOIA-SHADOW-Q01-V2/S1_ON")
    if not root.is_dir():
        return dict(status="Q01_RAW_NOT_PRESENT")
    before = {p.name: os.stat(p).st_mtime_ns for p in root.iterdir()}
    lines = (root / "sensors.jsonl").read_text().splitlines()
    rows = [json.loads(l) for l in lines]
    ck = lambda t: c.Clock(t["capture_sim_s"], t["delivery_sim_s"], t["provenance"], t["frame"], t["depth_convention"])
    out = []
    for i in range(3):
        z = dict(np.load(root / ("sensor_%03d.npz" % i), allow_pickle=False))
        inp = c.EvidenceInput.from_arrays(z, ck(rows[i]["camera_time"]), ck(rows[i]["lidar_time"]),
                                          rows[i]["camera_time"]["decision_sim_s"], tuple(rows[i]["calibration"]))
        for policy in ("F0", "H1", "H2", "H3", "BOIA"):
            rec = ShadowRecorder(policy); rec.reset(1)
            d, _ = rec.observe(1, 0, inp, ORACLE_PROXY)
            out.append(dict(frame=i, policy=policy, decision=d.decision, reasons=d.reasons, hypothesis=d.evidence_hypothesis,
                            identity_verdict=d.identity_verdict, map_accepted=d.local_map_status["accepted"],
                            runtime_s=round(d.runtime_s, 4)))
    after = {p.name: os.stat(p).st_mtime_ns for p in root.iterdir()}
    return dict(status="REPLAYED_READ_ONLY", raw_unchanged=before == after, records=out)


def main(out, examples=None):
    rows, keep = [], []
    for name, (script, labels) in scenarios().items():
        for policy in ("F0", "H1", "H2", "H3", "BOIA"):
            rows += run(policy, name, script, labels, keep if policy in ("H3", "BOIA") else None)
    summary = summarize(rows)
    per_scenario = {}
    for r in rows:
        k = per_scenario.setdefault(r.scenario, {}).setdefault(r.policy, dict(proposals=0, first_proposal_s=None, abstain=0))
        if r.decision == "PROPOSE_REACQUIRE_SHADOW":
            k["proposals"] += 1; k["first_proposal_s"] = k["first_proposal_s"] if k["first_proposal_s"] is not None else r.time_s
        k["abstain"] += r.decision == "ABSTAIN"
    res = dict(label_semantics=rows[0] and summarize(rows)[rows[0].policy]["label_semantics"], dwell_decisions=DWELL,
               summary=summary, per_scenario=per_scenario, q01_replay=q01_replay())
    Path(out).write_text(json.dumps(res, indent=1, default=str) + "\n")
    if examples:
        Path(examples).write_text(json.dumps(keep, indent=1, default=str) + "\n")
    for p, s in summary.items():
        print(p, {k: s[k] for k in ("trigger_count", "unnecessary_triggers", "missed_trigger_scenarios", "trigger_latency_s",
                                    "abstain_count", "invalid_input_rejections", "loss_decisions", "loss_decisions_with_candidate")},
              "runtime", {k: round(v, 4) for k, v in s["runtime_s"].items()})


if __name__ == "__main__":
    main(*sys.argv[1:3])
