"""Independent re-analysis of HR3-P01 raw traces. NumPy only; no repository imports."""
import json
import math
import sys
from pathlib import Path

import numpy as np

RUN = Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/h_r3_pilot_v1/HR3-P01_seed4502")
ARMS = ("B0", "B1", "B2", "B3")
W, DT = 20, 0.1
CLASSES = ["NONE", "RANGE_EXIT", "FOV_EXIT", "SCALE_LOSS", "FULL", "CENTRE_ONLY", "PARTIAL", "VISIBLE"]
PRIMARY = {"FULL", "CENTRE_ONLY", "PARTIAL"}

tr, meta = {}, {}
for a in ARMS:
    with np.load(RUN / f"seed4502_{a}" / "h_r3_trace.npz") as z:
        tr[a] = {k: z[k] for k in z.files}
    meta[a] = json.loads((RUN / f"seed4502_{a}" / "h_r3_meta.json").read_text())
    assert meta[a]["el0_classes"] == CLASSES
steps, n = tr["B0"]["camera"].shape
print("steps,n", steps, n, "keys", sorted(tr["B0"].keys()))
print("env config per arm identical:", all(meta[a]["env"] == meta["B0"]["env"] for a in ARMS), meta["B0"]["env"])
print("max_velocity/yaw_rate_max/governor", meta["B0"]["max_velocity"], meta["B0"]["yaw_rate_max"], meta["B0"]["speed_governor"])

# ---- matching
t0 = {a: tr[a]["t0"] for a in ARMS}
dig = {a: tr[a]["digest"] for a in ARMS}
matched, unmatched = [], []
for e in range(n):
    ts = [int(t0[a][e]) for a in ARMS]
    if all(t < 0 for t in ts):
        continue
    same_t0 = len(set(ts)) == 1 and ts[0] > 0
    same_prefix = same_t0 and all(np.array_equal(dig[a][:ts[0], e], dig["B0"][:ts[0], e]) for a in ARMS)
    if same_prefix:
        matched.append((e, ts[0]))
    else:
        unmatched.append((e, ts, "t0" if not same_t0 else "prefix"))
print("triggered", len(matched) + len(unmatched), "matched", len(matched), "unmatched", len(unmatched),
      "never", n - len(matched) - len(unmatched))
print("unmatched reasons", {r: sum(1 for u in unmatched if u[2] == r) for r in ("t0", "prefix")})
um_cls = {}
for e, ts, r in unmatched:
    c = CLASSES[int(tr["B0"]["el0_cls"][ts[0] - 1, e])]
    um_cls[c] = um_cls.get(c, 0) + 1
print("unmatched B0 class at t0-1:", um_cls)
t0s = sorted(t for _, t in matched)
print("matched t0 range", t0s[0], t0s[-1], "hist", {t: t0s.count(t) for t in sorted(set(t0s))})

# coupling onset: first step where an env not yet branched (in B0) differs from B0
def onset(a, b):
    pre = (t0[b] < 0)[None, :] | (np.arange(steps)[:, None] < t0[b][None, :])
    hit = np.nonzero(((dig[a] != dig[b]) & pre).any(axis=1))[0]
    return int(hit[0]) if len(hit) else None
print("coupling onset vs B0:", {a: onset(a, "B0") for a in ARMS[1:]},
      " B2 vs B1:", onset("B2", "B1"), " B3 vs B2:", onset("B3", "B2"))
for a in ARMS[1:]:
    pre = (t0["B0"] < 0)[None, :] | (np.arange(steps)[:, None] < t0["B0"][None, :])
    d = (dig[a] != dig["B0"]) & pre
    first = np.nonzero(d.any(axis=1))[0][0]
    print(f"  {a}: envs differing at onset step {first}:", np.nonzero(d[first])[0].tolist(),
          "| any env differing before step 10 in ANY state (incl. branched):",
          np.nonzero((dig[a] != dig["B0"])[:10].any(axis=0))[0].tolist())
# resets / crashes / captures around the onset
for a in ARMS:
    r = tr[a]["reset"][:12]
    print(f"  {a} resets at steps<12:", [(int(s), np.nonzero(r[s])[0].tolist()) for s in range(12) if r[s].any()])

# ---- outcomes
def outcome(a, e, t):
    x = tr[a]
    reobs = term = None
    for i in range(t, min(t + W, steps)):
        terminal = bool(x["crashed"][i, e] or x["captured"][i, e] or x["reset"][i, e])
        if reobs is None and x["camera"][i, e] and x["el0_visible_px"][i, e] >= 1:
            reobs = i
            if terminal:
                print(f"  NOTE same-step reobs+terminal arm={a} env={e} i={i}")
        if terminal:
            term = i
            break
    # strict rule: a reobservation on the terminal step itself does not count (post-reset obs)
    strict = reobs is not None and (term is None or reobs < term)
    last = (term - 1) if term is not None else min(t + W, steps) - 1
    span = list(range(t, last + 1))
    pos = x["robot_pos"][[t - 1] + span, e]
    dist = lambda i: float(np.linalg.norm(x["target_pos"][i, e] - x["robot_pos"][i, e]))
    later = slice(t, min(t + W + 30, steps))
    return {"reobs": reobs is not None, "strict": strict, "t_reobs": (reobs - t + 1) * DT if reobs is not None else None,
            "crash": bool(x["crashed"][t:t + W, e][: (term - t + 1) if term is not None else W].any()),
            "crash_ctrl": bool((x["crashed"][t:t + W, e] & x["intervening"][t:t + W, e]).any()),
            "capture": bool(x["captured"][t:t + W, e][: (term - t + 1) if term is not None else W].any()),
            "term": term, "reset_any_in_window": bool(x["reset"][t:t + W, e].any()),
            "ddist": dist(last) - dist(t - 1), "trans": float(np.linalg.norm(np.diff(pos, axis=0), axis=1).sum()),
            "fov": float(np.mean(x["el0_in_frustum"][span, e])), "cam_frac": float(np.mean(x["camera"][span, e])),
            "cap5": bool(x["captured"][later, e].any()), "crash5": bool(x["crashed"][later, e].any()),
            "ctrl_steps": int(x["intervening"][t:t + W, e].sum()),
            "false_det": bool((x["camera"][t:t + W, e] & (x["el0_visible_px"][t:t + W, e] < 1)).any()),
            "end_reason": meta[a]["end_reason"][e]}

events = []
for e, t in matched:
    cls = {a: CLASSES[int(tr[a]["el0_cls"][t - 1, e])] for a in ARMS}
    if len(set(cls.values())) != 1:
        print("  CLASS DIFFERS ACROSS ARMS at t0-1", e, cls)
    res = {a: outcome(a, e, t) for a in ARMS}
    events.append({"env": e, "t0": t, "cls": cls["B0"], "res": res,
                   "shadow": float(tr["B0"]["shadow_offset"][t, e]), "clear": bool(tr["B0"]["shadow_clear"][t, e])})
prim = [ev for ev in events if ev["cls"] in PRIMARY]
print("\nPRIMARY n =", len(prim), {c: sum(1 for ev in prim if ev["cls"] == c) for c in PRIMARY})
print("primary envs/t0:", [(ev["env"], ev["t0"]) for ev in prim])
for a in ARMS:
    r = [ev["res"][a] for ev in prim]
    print(f"{a}: reobs={sum(x['reobs'] for x in r)} strict={sum(x['strict'] for x in r)} "
          f"crash={sum(x['crash'] for x in r)} crash_ctrl={sum(x['crash_ctrl'] for x in r)} capture={sum(x['capture'] for x in r)} "
          f"terminal={sum(x['term'] is not None for x in r)} reset_in_W={sum(x['reset_any_in_window'] for x in r)} "
          f"cap5={sum(x['cap5'] for x in r)} crash5={sum(x['crash5'] for x in r)} false_det={sum(x['false_det'] for x in r)} "
          f"median_t={np.median([x['t_reobs'] for x in r if x['reobs']]):.2f} "
          f"mean_t_cens={np.mean([x['t_reobs'] if x['reobs'] else W*DT for x in r]):.3f} "
          f"ddist={np.mean([x['ddist'] for x in r]):.3f} trans={np.mean([x['trans'] for x in r]):.3f} "
          f"fov={np.mean([x['fov'] for x in r]):.3f} cam={np.mean([x['cam_frac'] for x in r]):.3f} "
          f"ctrl_steps_mean={np.mean([x['ctrl_steps'] for x in r]):.2f}")
    print("   end_reason counts:", {k: sum(1 for x in r if x['end_reason'] == k) for k in set(x['end_reason'] for x in r)})

def mcnemar(n10, n01):
    m = n10 + n01
    if m == 0:
        return 1.0
    p = sum(math.comb(m, i) for i in range(min(n10, n01) + 1)) / 2 ** m
    return min(1.0, 2 * p)

for x, y in (("B3", "B2"), ("B3", "B1"), ("B2", "B1"), ("B1", "B0"), ("B2", "B0"), ("B3", "B0")):
    ax = np.array([ev["res"][x]["reobs"] for ev in prim]); ay = np.array([ev["res"][y]["reobs"] for ev in prim])
    n10, n01 = int((ax & ~ay).sum()), int((~ax & ay).sum())
    rng = np.random.default_rng(0)
    idx = rng.integers(0, len(ax), (10000, len(ax)))
    boots = (ax[idx].astype(float) - ay[idx].astype(float)).mean(axis=1)
    print(f"{x}-{y}: diff={(ax.mean()-ay.mean())*100:+.1f}pp n10/n01={n10}/{n01} McNemar p={mcnemar(n10,n01):.4f} "
          f"boot95=[{np.percentile(boots,2.5)*100:+.1f},{np.percentile(boots,97.5)*100:+.1f}]")
pat = {}
for ev in prim:
    k = "".join("1" if ev["res"][a]["reobs"] else "0" for a in ("B1", "B2", "B3"))
    pat.setdefault(k, []).append(ev["env"])
print("patterns B1B2B3:", {k: (len(v), v) for k, v in sorted(pat.items())})
print("B0 reobs among 000:", [ev["env"] for ev in prim if ev["res"]["B0"]["reobs"] and not any(ev["res"][a]["reobs"] for a in ("B1","B2","B3"))])
print("shadow offset at t0 per primary event:", [(ev["env"], ev["shadow"], ev["clear"]) for ev in prim])

# ---- all matched: collisions
print("\nALL MATCHED (107): per arm crash in W / under control / terminal / reset in W / cap5 / crash5")
for a in ARMS:
    r = [ev["res"][a] for ev in events]
    print(f"  {a}: crash={sum(x['crash'] for x in r)} ctrl={sum(x['crash_ctrl'] for x in r)} "
          f"crash_envs={[ev['env'] for ev in events if ev['res'][a]['crash']]} "
          f"ctrl_envs={[(ev['env'],ev['cls']) for ev in events if ev['res'][a]['crash_ctrl']]} "
          f"terminal={sum(x['term'] is not None for x in r)} capture={sum(x['capture'] for x in r)} "
          f"same_step_reobs_term={sum(1 for x in r if x['reobs'] and not x['strict'])}")
by = {}
for ev in events:
    by.setdefault(ev["cls"], []).append(ev)
for c, evs in sorted(by.items()):
    print(f"  class {c} n={len(evs)}:", {a: sum(ev["res"][a]["reobs"] for ev in evs) for a in ARMS},
          "B2-B1 n10/n01", int(sum(ev["res"]["B2"]["reobs"] and not ev["res"]["B1"]["reobs"] for ev in evs)),
          int(sum(ev["res"]["B1"]["reobs"] and not ev["res"]["B2"]["reobs"] for ev in evs)),
          "B3-B2 n10/n01", int(sum(ev["res"]["B3"]["reobs"] and not ev["res"]["B2"]["reobs"] for ev in evs)),
          int(sum(ev["res"]["B2"]["reobs"] and not ev["res"]["B3"]["reobs"] for ev in evs)))

# ---- target independence
mx = 0.0
for ev in events:
    e, t = ev["env"], ev["t0"]
    ends = [t + W] + [t + int(np.nonzero(tr[a]["reset"][t:t + W, e])[0][0]) for a in ARMS if tr[a]["reset"][t:t + W, e].any()]
    end = min(ends)
    for a in ARMS:
        mx = max(mx, float(np.abs(tr[a]["target_pos"][t:end, e] - tr["B0"]["target_pos"][t:end, e]).max(initial=0.0)))
print("\ntarget max diff pre-terminal over matched events:", mx)
# target diff over the FULL window ignoring termination, and over primary events
mxp = max(float(np.abs(tr[a]["target_pos"][ev["t0"]:ev["t0"] + W, ev["env"]] - tr["B0"]["target_pos"][ev["t0"]:ev["t0"] + W, ev["env"]]).max())
          for ev in prim for a in ARMS)
print("target max diff over full window, primary events (B0 env76 crash aside):", mxp)

# ---- B2/B3 divergence diagnostics (post-hoc replication)
print("\nB2 vs B3 divergence per primary event: env, t0, pattern, first action diff, first digest diff, B3 nonzero offset ctrl steps, max cmd speed B2/B3")
maxv = meta["B0"]["max_velocity"]
nb = 0; nd = 0
for ev in prim:
    e, t = ev["env"], ev["t0"]; w = slice(t, t + W)
    ad = np.abs(tr["B3"]["action"][w, e] - tr["B2"]["action"][w, e]).max(axis=1) > 0
    dd = dig["B3"][w, e] != dig["B2"][w, e]
    fa = int(np.nonzero(ad)[0][0]) if ad.any() else None
    fd = int(np.nonzero(dd)[0][0]) if dd.any() else None
    noise_first = fd is not None and (fa is None or fd < fa)
    nb += noise_first
    k = "".join("1" if ev["res"][a]["reobs"] else "0" for a in ("B1", "B2", "B3"))
    if k[1] != k[2]:
        nd += noise_first
    sp = lambda a: float((np.linalg.norm(tr[a]["action"][w, e][:, :2], axis=1) * maxv)[tr[a]["intervening"][w, e]].max(initial=0.0))
    nz = int(((tr["B3"]["shadow_offset"][w, e] != 0) & tr["B3"]["intervening"][w, e]).sum())
    print(f"  {e:4d} t0={t} {k} fa={fa} fd={fd} noise_first={noise_first} nz_off={nz}/{int(tr['B3']['intervening'][w,e].sum())} "
          f"spd B2={sp('B2'):.2f} B3={sp('B3'):.2f} B2trans={ev['res']['B2']['trans']:.2f} B3trans={ev['res']['B3']['trans']:.2f}")
print("noise-first events:", nb, "among discordant:", nd)
print("median max cmd speed B2/B3:", np.median([float((np.linalg.norm(tr[a]["action"][ev["t0"]:ev["t0"]+W, ev["env"]][:, :2], axis=1)*maxv)[tr[a]["intervening"][ev["t0"]:ev["t0"]+W, ev["env"]]].max(initial=0.0)) for ev in prim]) if False else None)
for a in ("B2", "B3"):
    v = [float((np.linalg.norm(tr[a]["action"][ev["t0"]:ev["t0"]+W, ev["env"]][:, :2], axis=1)*maxv)[tr[a]["intervening"][ev["t0"]:ev["t0"]+W, ev["env"]]].max(initial=0.0)) for ev in prim]
    print(f"  median max cmd speed {a}: {np.median(v):.3f}")

# switch check: steps where both B2 and B3 control the same env from the same prior digest
same = np.zeros_like(tr["B2"]["intervening"]); same[1:] = dig["B2"][:-1] == dig["B3"][:-1]
both = tr["B2"]["intervening"] & tr["B3"]["intervening"] & same
eq = np.abs(tr["B3"]["action"] - tr["B2"]["action"]).max(axis=-1) == 0
zero = tr["B3"]["shadow_offset"] == 0
print("switch check: steps", int(both.sum()), "zero", int((both & zero).sum()), "zero&equal", int((both & zero & eq).sum()),
      "nonzero", int((both & ~zero).sum()), "nonzero&equal", int((both & ~zero & eq).sum()))
# also B1 vs B2 yaw switch
same12 = np.zeros_like(tr["B2"]["intervening"]); same12[1:] = dig["B1"][:-1] == dig["B2"][:-1]
both12 = tr["B1"]["intervening"] & tr["B2"]["intervening"] & same12
print("B1/B2 same-state control steps:", int(both12.sum()), "xy action equal:",
      int((both12 & (np.abs(tr["B2"]["action"][..., :2] - tr["B1"]["action"][..., :2]).max(axis=-1) == 0)).sum()),
      "B1 yaw all zero while controlling:", bool((tr["B1"]["action"][..., 3][tr["B1"]["intervening"]] == 0).all()))

# compute
for a in ARMS:
    c = tr[a]["controller_ms"]
    print(f"compute {a}: mean={c.mean():.3f} p95={np.percentile(c,95):.3f} p99={np.percentile(c,99):.3f}")

# ---- belief error at t0-1 and within window (secondary diagnostic)
be = [float(np.linalg.norm(tr["B0"]["belief_pos"][ev["t0"]-1, ev["env"]] - tr["B0"]["target_pos"][ev["t0"]-1, ev["env"]])) for ev in prim]
print("belief error at t0-1 (primary): median %.3f max %.3f" % (np.median(be), max(be)))
for k, envs in sorted(pat.items()):
    print("  pattern", k, "belief err at t0-1:", [round(float(np.linalg.norm(tr['B0']['belief_pos'][ev['t0']-1, ev['env']] - tr['B0']['target_pos'][ev['t0']-1, ev['env']])), 2) for ev in prim if ev['env'] in envs])

# ---- sensitivity: unmatched FULL events using each arm's own t0
print("\nUNMATCHED events with B0 class FULL (sensitivity, own t0 per arm):")
for e, ts, r in unmatched:
    if CLASSES[int(tr["B0"]["el0_cls"][ts[0] - 1, e])] == "FULL":
        print("  env", e, "t0s", ts, r, {a: (outcome(a, e, int(t0[a][e]))["reobs"] if t0[a][e] > 0 else None) for a in ARMS})

# ---- episode step index at t0: the matched events are all at the start of the first episode
print("\nfirst detection step (B0) for primary events:", [(ev["env"], int(np.nonzero(tr["B0"]["camera"][:, ev["env"]])[0][0]), ev["t0"]) for ev in prim])
json.dump({"primary": [{"env": ev["env"], "t0": ev["t0"], "cls": ev["cls"], "res": {a: {k: v for k, v in ev["res"][a].items()} for a in ARMS}} for ev in prim]},
          open(Path(__file__).parent / "recompute_primary.json", "w"), indent=1, default=str)
