"""Camera-only C2, crash-mechanism telemetry (EXPLORATORY, post hoc; not preregistered). CPU only, reads JSON.

For each C2 condition (A = GR-0 S00 cells; B_in, B_out, C, C_gL, B_in_gL = C2 cells), pooled over the 12 blocks
(70/115/160/205 bars x seeds 4501-4503):
  1. outcomes on the preregistered quota episodes (episodes.jsonl, in_quota) and the crash-step distribution;
  2. the policy's commanded action (result.json `action`, all samples; vision mode = vehicle-frame velocity,
     x forward, y left, per-axis limit 2.5 m/s) and the speed governor's telemetry (result.json `speed_governor`);
  3. crash causes;
  4. paired outcomes: the first episode of each env (env_episode_index 0) starts from the same initial state in every
     condition (checked by kroc2027 `initial_state_hash`), so the A -> X outcome transition is a within-scene contrast.
usage: c2_mechanism_telemetry.py OUT_JSON
"""
import collections
import hashlib
import json
import math
import pathlib
import sys

RES = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results")
GR0 = RES / "gain_randomization_gr0_v1/gr0_main/S00"
C2 = RES / "camonly_c1c2_v1/c2"
NAMES = {"A": None, "B_in": "no_lidar_in_fov", "B_out": "no_lidar_out_fov", "C": "no_lidar",
         "C_gL": "no_lidar_obs_only", "B_in_gL": "no_lidar_in_fov_obs_only"}
DENS = (70, 115, 160, 205)
SEEDS = (4501, 4502, 4503)
DT = 0.1
OUT = pathlib.Path(sys.argv[1])


def cell_dir(cond, d, s):
    cid = "E2_obstacle_aware__%dbars__seed%d" % (d, s)
    return (GR0 / cid) if NAMES[cond] is None else (C2 / NAMES[cond] / cid)


def jsonl(p):
    with open(p) as f:
        return [json.loads(x) for x in f if x.strip()]


def pct(xs, q):
    xs = sorted(xs)
    if not xs:
        return None
    k = (len(xs) - 1) * q
    lo, hi = math.floor(k), math.ceil(k)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def main():
    if OUT.exists():
        raise SystemExit("refusing to overwrite " + str(OUT))
    out = {"analysis": "camonly C2 crash-mechanism telemetry", "status": "EXPLORATORY_POST_HOC",
           "inputs_sha256": {}, "conditions": {}, "by_density": {}, "paired_first_episode": {}}
    first_ep = {}            # cond -> {(d, s, env): (initial_state_hash, outcome, completion_step)}
    for cond in NAMES:
        quota = collections.Counter()
        crash_steps = []
        act_n = 0
        act = collections.defaultdict(float)
        gov = collections.defaultdict(float)
        gov_n = 0
        contact = collections.defaultdict(float)
        contact_n = 0
        causes = collections.Counter()
        ctx = collections.defaultdict(float)
        first_ep[cond] = {}
        for d in DENS:
            dq = collections.Counter()
            for s in SEEDS:
                cd = cell_dir(cond, d, s)
                for fn in ("episodes.jsonl", "result.json", "kroc2027_episodes.jsonl"):
                    out["inputs_sha256"]["%s/%d/%d/%s" % (cond, d, s, fn)] = hashlib.sha256(
                        (cd / fn).read_bytes()).hexdigest()
                eps = jsonl(cd / "episodes.jsonl")
                kro = jsonl(cd / "kroc2027_episodes.jsonl")
                kmap = {(k["env_index"], k["env_episode_index"]): k for k in kro}
                for e in eps:
                    if e["env_episode_index"] == 0:
                        k = kmap.get((e["env_index"], 0))
                        if k is not None and k["actions"] != e["completion_step"]:
                            raise ValueError("first-episode duration mismatch %s" % e["episode_id"])
                        first_ep[cond][(d, s, e["env_index"])] = (
                            k["initial"]["initial_state_hash"] if k else None, e["outcome"], e["completion_step"])
                    if not e["in_quota"]:
                        continue
                    quota[e["outcome"]] += 1
                    dq[e["outcome"]] += 1
                    if e["outcome"] == "crash":
                        # within-episode duration: kroc2027 `actions` (episodes.jsonl completion_step is a global
                        # step counter, equal to the duration only for each env's first episode)
                        crash_steps.append(kmap[(e["env_index"], e["env_episode_index"])]["actions"])
                r = json.loads((cd / "result.json").read_text())
                a = r["action"]
                n = a["samples"]
                act_n += n
                for i, v in enumerate(a["mean_abs"]):
                    act["mean_abs_%d" % i] += v * n
                for i, v in enumerate(a["executed_edge95_rate"]):
                    act["edge95_%d" % i] += v * n
                for key in ("signed_mean_y", "positive_y_rate", "negative_y_rate", "high80_y_rate",
                            "sign_flip_y_rate"):
                    act[key] += a[key] * n
                m = a["motion"]
                act["mean_speed_mps"] += m["mean_speed_mps"] * m["samples"]
                act["mean_command_speed_mps"] += m["mean_command_speed_mps"] * m["samples"]
                act["motion_samples"] += m["samples"]
                for c in ("front_clear", "front_blocked", "target_visible", "target_hidden"):
                    cc = a["context"][c]
                    ctx[c + "_n"] += cc["samples"]
                    ctx[c + "_abs_y"] += cc["mean_abs_y"] * cc["samples"] if cc["mean_abs_y"] is not None else 0.0
                g = r["speed_governor"]
                gn = g["samples"]
                gov_n += gn
                for key in ("intervention_rate", "mean_requested_speed_mps", "mean_executed_speed_mps",
                            "mean_clearance_m", "mean_scale", "mean_requested_ttc_s",
                            "negative_stopping_margin_requested_rate", "negative_stopping_margin_executed_rate"):
                    if g.get(key) is not None:
                        gov[key] += g[key] * gn
                gc = g["contact"]
                if gc["count"]:
                    contact_n += gc["count"]
                    for key in ("mean_actual_speed_mps", "mean_requested_speed_mps", "mean_executed_speed_mps",
                                "mean_clearance_m", "mean_requested_ttc_s", "mean_stopping_margin_requested_m",
                                "mean_stopping_margin_executed_m", "mean_step"):
                        if gc.get(key) is not None:
                            contact[key] += gc[key] * gc["count"]
                cc = r["crash_causes"]
                for key in ("bar_contact", "below", "above", "out_of_bounds"):
                    causes[key] += cc[key]
            nq = sum(dq.values())
            out["by_density"].setdefault(str(d), {})[cond] = {
                "quota_episodes": nq, "crash_pct": 100.0 * dq["crash"] / nq, "capture_pct": 100.0 * dq["capture"] / nq}
        nq = sum(quota.values())
        early = {"within_%.1fs" % t: 100.0 * sum(1 for x in crash_steps if x * DT <= t) / max(1, len(crash_steps))
                 for t in (1.0, 2.0, 3.0, 5.0)}
        out["conditions"][cond] = {
            "quota_episodes": nq,
            "crash_pct": 100.0 * quota["crash"] / nq, "capture_pct": 100.0 * quota["capture"] / nq,
            "timeout_pct": 100.0 * quota["timeout"] / nq,
            "crash_step": {"n": len(crash_steps), "p10": pct(crash_steps, 0.1), "p50": pct(crash_steps, 0.5),
                           "p90": pct(crash_steps, 0.9), "share_of_crashes_pct": early},
            "crash_within_t_as_pct_of_episodes": {k: v * quota["crash"] / nq for k, v in early.items()},
            "action_all_samples": dict({k: v / act_n for k, v in act.items()
                                        if k not in ("mean_speed_mps", "mean_command_speed_mps", "motion_samples")},
                                       samples=act_n,
                                       mean_speed_mps=act["mean_speed_mps"] / act["motion_samples"],
                                       mean_command_speed_mps=act["mean_command_speed_mps"] / act["motion_samples"]),
            "action_context": {
                "front_blocked_fraction": ctx["front_blocked_n"] / (ctx["front_blocked_n"] + ctx["front_clear_n"]),
                "target_visible_fraction": ctx["target_visible_n"] / (ctx["target_visible_n"] + ctx["target_hidden_n"])},
            "speed_governor_all_samples": dict({k: v / gov_n for k, v in gov.items()}, samples=gov_n),
            "speed_governor_at_contact": dict({k: v / contact_n for k, v in contact.items()}, contacts=contact_n),
            "crash_causes_all_episodes": dict(causes),
        }
    # paired first episodes
    base = first_ep["A"]
    for cond in NAMES:
        if cond == "A":
            continue
        trans = collections.Counter()
        mismatch = 0
        both_crash_steps = []
        new_crash_steps = []
        for key, (h, o, st) in base.items():
            x = first_ep[cond].get(key)
            if x is None:
                continue
            if h is None or x[0] != h:
                mismatch += 1
                continue
            trans[(o, x[1])] += 1
            if o != "crash" and x[1] == "crash":
                new_crash_steps.append(x[2])
            if o == "crash" and x[1] == "crash":
                both_crash_steps.append(x[2])
        n = sum(trans.values())
        out["paired_first_episode"][cond] = {
            "pairs": n, "initial_state_mismatch": mismatch,
            "transitions": {"%s->%s" % k: v for k, v in sorted(trans.items())},
            "A_crash_pct": 100.0 * sum(v for k, v in trans.items() if k[0] == "crash") / max(1, n),
            "X_crash_pct": 100.0 * sum(v for k, v in trans.items() if k[1] == "crash") / max(1, n),
            "new_crash_pct_of_pairs": 100.0 * sum(v for k, v in trans.items() if k[0] != "crash" and k[1] == "crash") / max(1, n),
            "resolved_crash_pct_of_pairs": 100.0 * sum(v for k, v in trans.items() if k[0] == "crash" and k[1] != "crash") / max(1, n),
            "new_crash_step_p10_p50_p90": [pct(new_crash_steps, q) for q in (0.1, 0.5, 0.9)],
        }
    OUT.write_text(json.dumps(out, indent=1, sort_keys=True) + "\n")
    print(OUT, hashlib.sha256(OUT.read_bytes()).hexdigest())


if __name__ == "__main__":
    main()
