"""R2 rule 7: evidence-map invariants over random packet sequences (moving vehicle and target, bars, incomplete and
refused packets, cameras, tracks). usage: map_invariants.py OUT_JSON"""
import json
import math
import random
import sys

import numpy as np

from harness import CATALOG, bar, beam_dirs, camera, core, dump, header, scan, target

K_FREE = 2
viol = {f"I{i}": 0 for i in range(1, 8)}
examples = []
checked = {"sequences": 0, "steps": 0, "cell_checks": 0}


def note(k, info):
    viol[k] += 1
    if len(examples) < 20:
        examples.append({"inv": k, **info})


def hit_cells(r, s, pose, layout, lo, n, width=0.05):
    """Test-side cells of this packet's HIT points (independent of the map); a point within 1e-6 of a cell boundary
    marks both neighbours, so float rounding can only widen the set."""
    D = beam_dirs(layout)
    c, sn = math.cos(pose[3]), math.sin(pose[3])
    out = set()
    for i in range(s.shape[0]):
        for j in range(s.shape[1]):
            if s[i, j] != core.HIT or not (0.2 <= r[i, j] <= 12.0):
                continue
            d = D[i, j]
            p = (pose[0] + (c * d[0] - sn * d[1]) * r[i, j], pose[1] + (sn * d[0] + c * d[1]) * r[i, j])
            cand = []
            for v in p:
                f = (v - lo) / width
                k = math.floor(f)
                cand.append({k, k - 1} if f - k < 1e-6 else ({k, k + 1} if k + 1 - f < 1e-6 else {k}))
            for a in cand[0]:
                for b in cand[1]:
                    if 0 <= a < n and 0 <= b < n:
                        out.add(a * n + b)
    return out


def map_only(states):
    return ["OBSERVED_HIT" if s == "MOVING_TARGET_CANDIDATE" else s for s in states]


for seq in range(int(sys.argv[2]) if len(sys.argv) > 2 else 60):
    rng = random.Random(9000 + seq)
    layout = "P0R-72x1" if seq % 2 == 0 else "MOTAR-72x4"
    cfg = "SYNTH-72x1" if seq % 2 == 0 else "SYNTH-72x4"
    bars = [bar(rng.uniform(-6, 6), rng.uniform(-6, 6), *rng.choice(CATALOG)) for _ in range(rng.randint(2, 8))]
    bars = [b for b in bars if math.hypot(*b["c"]) > 1.5]
    tgt0, tv = (rng.uniform(-4, 4), rng.uniform(-4, 4)), (rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5))
    cam_core = core.CertifierCore(cfg, 0, 0)
    ref_core = core.CertifierCore(cfg, 0, 0)            # identical packets and tracks, no camera
    occupied_at, static_seen, conflict_seen, complete_steps, free_before = {}, set(), set(), [], set()
    pose = [0.0, 0.0, 1.0, 0.0]
    for k in range(1, 31):
        pose[0] += rng.uniform(-0.1, 0.1)
        pose[1] += rng.uniform(-0.1, 0.1)
        pose[3] = math.atan2(math.sin(pose[3] + rng.uniform(-0.2, 0.2)), math.cos(pose[3] + rng.uniform(-0.2, 0.2)))
        tc = (tgt0[0] + tv[0] * 0.1 * k, tgt0[1] + tv[1] * 0.1 * k, 1.0)
        r, s, _ = scan(bars + [target(*tc)], tuple(pose), layout)
        mode = rng.random()
        if mode < 0.2:
            s[rng.randrange(s.shape[0]), rng.randrange(72)] = rng.choice([0, 3, 4, 5, 6])     # incomplete packet
        hi, hf = header(k, tuple(pose))
        if mode > 0.95:
            hf[0] += 0.05                                                                    # refused packet
        trk = np.array([1.0, k * 0.1, tc[0], tc[1], 0.2]) if rng.random() < 0.7 else None
        cam = camera(k * 0.1, tuple(pose), (tc[0] + rng.gauss(0, .5), tc[1] + rng.gauss(0, .5), 1.0),
                     sigma_r=rng.uniform(0.05, 0.7))
        rep = cam_core.step(r, s, hi, hf, camera=cam, track=trk)
        ref = ref_core.step(r, s, hi, hf, camera=None, track=trk)
        if rep["accepted"] and rep["complete"]:
            complete_steps.append(k)
        if rep["accepted"]:
            for cc in hit_cells(r, s, pose, layout, cam_core.map.lo, cam_core.map.n):
                occupied_at[cc] = k                                     # latest occupancy EVIDENCE (test-side)
        m = cam_core.map
        ids = np.flatnonzero((m.occ_last >= 0) | m.free_any)
        far = np.array([m.n * m.n - 1, 0])                     # grid corners: > 12 m from every pose
        sample = np.concatenate([ids, far])
        st = m.derive_ids(sample.tolist())
        st_ref = ref_core.map.derive_ids(sample.tolist())
        checked["cell_checks"] += len(sample)
        pts = np.stack([m.lo + (sample // m.n + 0.5) * 0.05, m.lo + (sample % m.n + 0.5) * 0.05], 1)
        rep_states = cam_core.map_states(pts[:400], k * 0.1)
        rep_later = cam_core.map_states(pts[:400], k * 0.1 + 100.0)
        for c, a, b in zip(sample.tolist(), st, st_ref):
            if a != b:
                note("I5", {"seq": seq, "step": k, "cell": c, "camera": a, "no_camera": b})
            if c in static_seen and a not in ("STATIC_OCCUPANCY", "CONFLICT"):
                note("I1", {"seq": seq, "step": k, "cell": c, "state": a})
            if c in conflict_seen and a != "CONFLICT":
                note("I2", {"seq": seq, "step": k, "cell": c, "state": a})
            if a == "VALIDATED_FREE" and c in occupied_at:
                n_complete = sum(1 for x in complete_steps if x > occupied_at[c])
                if n_complete < K_FREE:
                    note("I3", {"seq": seq, "step": k, "cell": c, "complete_after": n_complete})
            if a == "VALIDATED_FREE" and not (rep["accepted"] and rep["complete"]) and c not in free_before:
                note("I4", {"seq": seq, "step": k, "cell": c})
            if a == "STATIC_OCCUPANCY":
                static_seen.add(c)
            if a == "CONFLICT":
                conflict_seen.add(c)
        for x, y in zip(map_only(rep_states), map_only(rep_later)):
            if x != y:
                note("I6", {"seq": seq, "step": k, "now": x, "later": y})
        if st[-2:] != ["UNKNOWN", "UNKNOWN"]:
            note("I7", {"seq": seq, "step": k, "corners": st[-2:]})
        free_before = {c for c, a in zip(sample.tolist(), st) if a == "VALIDATED_FREE"}
        checked["steps"] += 1
    checked["sequences"] += 1

desc = {"I1": "STATIC never leaves STATIC/CONFLICT", "I2": "CONFLICT is permanent",
        "I3": "occupied -> FREE only after >= K_FREE complete packets", "I4": "an incomplete/refused packet adds no FREE",
        "I5": "camera labels never change a map state", "I6": "time alone never changes a map state (label TTL only)",
        "I7": "never-observed cells are UNKNOWN"}
print(json.dumps({"violations": viol, "checked": checked}, indent=1))
dump(sys.argv[1], {"pass": sum(viol.values()) == 0, "violations": viol, "invariants": desc, "checked": checked,
                   "examples": examples})
