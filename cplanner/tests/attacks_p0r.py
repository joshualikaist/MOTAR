"""R2 rule 4: attacks on the P0-R boundary and regressions of the v1-v7 defects (H1/F2, H3/F1, R-1..R-6, time and
frame mixing, UNKNOWN->FREE, static deletion, blind zone). usage: attacks_p0r.py OUT_JSON"""
import json
import math
import random
import sys

import numpy as np

from harness import bar, camera, core, dump, header, pole, scan, target

ROWS = []
CALLS = {"n": 0}


def case(cid, cls, what, ok, detail):
    ROWS.append({"id": cid, "class": cls, "what": what, "pass": bool(ok), "detail": detail})
    print("%-5s %-8s %-5s %s | %s" % (cid, cls, "PASS" if ok else "FAIL", what, json.dumps(detail, default=str)[:160]),
          flush=True)


def P(r, deg):
    return (r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg)))


POSE = (0.0, 0.0, 1.0, 0.0)


def fresh(cfg="SYNTH-72x1"):
    return core.CertifierCore(cfg, 0, 0)


def run(objs, path, *, cfg="SYNTH-72x1", step=1, camera_=None, track=None, mutate=None, layout="P0R-72x1"):
    r, s, who = scan(objs, POSE, layout)
    if mutate:
        mutate(r, s, who)
    c = fresh(cfg)
    hi, hf = header(step, POSE)
    rep = c.step(r, s, hi, hf, camera=camera_, track=track)
    out = c.certify(np.array([path], float), step)[0]
    return rep, out, c


# a bar across the +x path at 1.5 m, plus open space elsewhere
BAR_SCENE = [bar(1.5, 0.0, 0.25, 0.25)]
PATH_X = ((0.0, 0.0), (2.5, 0.0))
OPEN_PATH = ((0.0, 0.0), (0.0, 2.0))


def counter_classes():
    class Meta(type):
        def __eq__(cls, other):
            CALLS["n"] += 1
            return True
        __hash__ = type.__hash__

    class Sub(np.ndarray, metaclass=Meta):
        def __array_finalize__(self, obj):
            CALLS["n"] += 1

        def __getitem__(self, k):
            CALLS["n"] += 1
            return np.ndarray.__getitem__(self, k)

        def __len__(self):
            CALLS["n"] += 1
            return np.ndarray.__len__(self)

        def __iter__(self):
            CALLS["n"] += 1
            return np.ndarray.__iter__(self)

    class Fake:
        def __array__(self, dtype=None):
            CALLS["n"] += 1
            return np.full((1, 72), 12.0)

        @property
        def __array_interface__(self):
            CALLS["n"] += 1
            return np.full((1, 72), 12.0).__array_interface__

        def __len__(self):
            CALLS["n"] += 1
            return 1

    class SStr(str):
        def __eq__(self, o):
            CALLS["n"] += 1
            return str.__eq__(self, o)

        def __hash__(self):
            CALLS["n"] += 1
            return str.__hash__(self)

    class IInt(int):
        def __eq__(self, o):
            CALLS["n"] += 1
            return True
        __hash__ = int.__hash__

    class FFloat(float):
        def __gt__(self, o):
            CALLS["n"] += 1
            return True
    return Meta, Sub, Fake, SStr, IInt, FFloat


# ================================================================== H1 / F2 family: the boundary
Meta, Sub, Fake, SStr, IInt, FFloat = counter_classes()
r0, s0, _ = scan(BAR_SCENE, POSE)
hi0, hf0 = header(1, POSE)
good = dict(ranges=r0, status=s0, header_i=hi0, header_f=hf0)
bad_variants = {
    "ndarray_subclass": lambda a: a.view(Sub),
    "list": lambda a: a.tolist(),
    "array_protocol_object": lambda a: Fake(),
    "object_dtype": lambda a: a.astype(object),
    "float32_or_int32": lambda a: a.astype(np.float32 if a.dtype.kind == "f" else np.int32),
    "big_endian": lambda a: a.astype(a.dtype.newbyteorder(">")) if a.dtype.itemsize > 1 else 1 / 0,
    "bool_dtype": lambda a: a.astype(bool),
    "structured_dtype": lambda a: np.zeros(a.shape, dtype=[("v", a.dtype)]),
    "wrong_shape_flat": lambda a: a.reshape(-1) if a.ndim > 1 else a.reshape(1, -1),
    "wrong_shape_extra_row": lambda a: np.concatenate([a.reshape(1, -1)] * 2),
    "zero_d": lambda a: np.array(a.reshape(-1)[0]),
}
for name, f in bad_variants.items():
    for arg in good:
        args = dict(good)
        try:
            args[arg] = f(good[arg])
        except Exception as e:     # a variant that cannot be built for this dtype is skipped and recorded
            case("B-" + name[:10], "SAFETY", "%s for %s" % (name, arg), True, {"skipped": type(e).__name__})
            continue
        CALLS["n"] = 0
        c = fresh()
        rep = c.step(**args)
        out = c.certify(np.array([PATH_X], float), 1)[0]
        case("B1", "SAFETY", "boundary refuses %s as %s; no caller code runs" % (name, arg),
             (not rep["accepted"]) and rep["reason"] == "MALFORMED_ARRAY_" + arg and CALLS["n"] == 0
             and out["reason"] == "NO_CURRENT_PACKET" and not c.exceptions,
             {"step": rep, "certify": out["reason"], "caller_calls": CALLS["n"]})

# B2: the caller mutates its own (writeable) arrays after step: the core's snapshot is unaffected
r1, s1, _ = scan(BAR_SCENE, POSE)
c = fresh()
c.step(r1, s1, *header(1, POSE))
r1[:] = 12.0
s1[:] = core.NO_HIT
out = c.certify(np.array([PATH_X], float), 1)[0]
case("B2", "SAFETY", "caller rewrites its arrays after step (bar removed in the caller's copy)",
     out["reason"] == "OBSERVED_SURFACE_INTERSECTION", {"certify": out["reason"]})

# B3: arrays that view a caller buffer later rewritten
buf = bytearray(r0.astype(np.float64).tobytes())
view = np.frombuffer(buf, dtype=np.float64).reshape(1, 72)
c = fresh()
c.step(view, s0.copy(), *header(1, POSE))
buf[:] = np.full((1, 72), 12.0).tobytes()
out = c.certify(np.array([PATH_X], float), 1)[0]
case("B3", "SAFETY", "array over a caller bytearray rewritten after step", out["reason"] == "OBSERVED_SURFACE_INTERSECTION",
     {"certify": out["reason"]})

# B4: non-contiguous view accepted and identical
c1, c2 = fresh(), fresh()
big = np.zeros((2, 144))
big[0, ::2] = r0[0]
c1.step(big[:1, ::2], s0.copy(), *header(1, POSE))
c2.step(np.ascontiguousarray(r0), s0.copy(), *header(1, POSE))
o1 = c1.certify(np.array([PATH_X, OPEN_PATH], float), 1)
o2 = c2.certify(np.array([PATH_X, OPEN_PATH], float), 1)
case("B4", "CONTRACT", "a strided exact float64 view is copied and gives identical results", o1 == o2,
     {"reasons": [o["reason"] for o in o1]})

# B5: paths, for_step, deadline, config name, ids
c = fresh()
c.step(r0, s0, *header(1, POSE))
for name, p in {"paths_subclass": np.array([OPEN_PATH], float).view(Sub), "paths_list": [OPEN_PATH],
                "paths_K0": np.zeros((0, 2, 2)), "paths_K65": np.zeros((65, 2, 2)),
                "paths_float32": np.array([OPEN_PATH], np.float32), "paths_object": np.array([OPEN_PATH], object),
                "paths_nan": np.array([((0.0, 0.0), (np.nan, 1.0))])}.items():
    CALLS["n"] = 0
    out = c.certify(p, 1)
    ok = all(not o["static_bar_certified"] for o in out) and CALLS["n"] == 0
    case("B5", "SAFETY", "certify refuses %s" % name, ok, {"reasons": [o["reason"] for o in out][:2], "calls": CALLS["n"]})
for name, fs in {"np_int64": np.int64(1), "bool": True, "int_subclass": IInt(1), "float": 1.0, "str": "1"}.items():
    CALLS["n"] = 0
    out = c.certify(np.array([OPEN_PATH], float), fs)[0]
    case("B6", "SAFETY", "for_step as %s refused" % name, out["reason"] == "PACKET_NOT_CURRENT_FOR_STEP" and CALLS["n"] == 0,
         {"reason": out["reason"], "calls": CALLS["n"]})
CALLS["n"] = 0
out = c.certify(np.array([OPEN_PATH], float), 1, deadline_s=FFloat(5.0))[0]
case("B7", "SAFETY", "deadline as float subclass refused", out["reason"] == "MALFORMED_DEADLINE" and CALLS["n"] == 0,
     {"reason": out["reason"], "calls": CALLS["n"]})
for name, args in {"config_str_subclass": (SStr("SYNTH-72x1"), 0, 0), "env_bool": ("SYNTH-72x1", True, 0),
                   "env_int_subclass": ("SYNTH-72x1", IInt(0), 0), "unknown_config": ("NOPE", 0, 0)}.items():
    CALLS["n"] = 0
    try:
        core.CertifierCore(*args)
        raised = False
    except ValueError:
        raised = True
    case("B8", "CONTRACT", "constructor refuses %s (raises ValueError, a programming error)" % name,
         raised and CALLS["n"] == 0, {"raised": raised, "calls": CALLS["n"]})

# B9: camera and track as hostile objects: ignored, never called, certify unchanged
base = run(BAR_SCENE, OPEN_PATH)[1]
for name, cam in {"camera_subclass": camera(0.1, POSE, (2.0, 0.0, 1.0)).view(Sub), "camera_list": [0.0] * 14,
                  "camera_fake": Fake(), "camera_wrong_shape": np.zeros(13)}.items():
    CALLS["n"] = 0
    rep, out, c = run(BAR_SCENE, OPEN_PATH, camera_=cam)
    case("B9", "SAFETY", "%s ignored; no caller code; certificate unchanged" % name,
         rep["accepted"] and rep["camera"] == "IGNORED_MALFORMED_ARRAY" and CALLS["n"] == 0 and out == base,
         {"camera": rep["camera"], "calls": CALLS["n"]})
for name, trk in {"track_subclass": np.array([1, .1, 0, 0, .2]).view(Sub), "track_list": [1, .1, 0, 0, .2]}.items():
    CALLS["n"] = 0
    rep, out, c = run(BAR_SCENE, OPEN_PATH, track=trk)
    case("B10", "SAFETY", "%s ignored; no caller code; certificate unchanged" % name,
         rep["accepted"] and rep["track"] == "IGNORED" and CALLS["n"] == 0 and out == base, {"calls": CALLS["n"]})

# B11: results are fresh plain dicts: mutating one changes nothing later
c = fresh()
c.step(r0, s0, *header(1, POSE))
o = c.certify(np.array([OPEN_PATH], float), 1)
o[0]["static_bar_certified"] = True
o[0]["scope"]["dynamic_objects"] = "CERTIFIED"
o2 = c.certify(np.array([OPEN_PATH], float), 1)
case("B11", "CONTRACT", "mutating a returned result does not change the next result", o2 == [base] or o2[0] == base,
     {"scope": o2[0]["scope"]["dynamic_objects"]})

# ================================================================== H3 / F1: the camera cannot lift anything
# v6 H3 geometry: target 3 m at 30 deg, thin pole r 0.04 at 0.9 m on the same bearing; path 1.2 m through the pole
h3 = [target(*P(3.0, 30)), pole(*P(0.9, 30), 0.04)]
h3_path = ((0.0, 0.0), P(1.2, 30))
for lab, cam in {"no camera": None, "truthful camera": camera(0.1, POSE, (*P(3.0, 30), 1.0)),
                 "camera range -1 sigma": camera(0.1, POSE, (*P(3.0, 30), 1.0), sigma_r=0.09, range_err=-0.09),
                 "camera on the pole": camera(0.1, POSE, (*P(0.9, 30), 1.0))}.items():
    rep, out, _ = run(h3, h3_path, camera_=cam)
    case("H3", "SAFETY", "v6 H3 geometry (pole plan clearance -0.490 m), %s" % lab,
         out["reason"] == "OBSERVED_SURFACE_INTERSECTION", {"reason": out["reason"], "labels": rep["labelled_returns"]})
# v7 F1 geometry: target 2.40 m at 30 deg, pole r 0.015 at 1.845 m; camera range 2.40 - 0.09 (-1 sigma); path 2.0 m
f1 = [target(*P(2.40, 30)), pole(*P(1.845, 30), 0.015)]
f1_path = ((0.0, 0.0), P(2.0, 30))
for k_sigma in (0.0, -1.0, -2.0, -3.0, 1.0):
    cam = camera(0.1, POSE, (*P(2.40, 30), 1.0), sigma_r=0.09, sigma_b=0.002, range_err=0.09 * k_sigma)
    rep, out, _ = run(f1, f1_path, camera_=cam)
    case("F1", "SAFETY", "v7 F1 geometry, camera range error %+.0f sigma" % k_sigma,
         out["reason"] == "OBSERVED_SURFACE_INTERSECTION", {"reason": out["reason"], "labels": rep["labelled_returns"]})

# ================================================================== R-1: a missing or failed beam never becomes NO_HIT
bar_beams = None


def fail_bar_beams(code, rng_value):
    def m(r, s, who):
        idx = np.where(who[0] == 0)[0]
        s[0, idx] = code
        r[0, idx] = rng_value
    return m


for name, code, val in (("NOT_DELIVERED", 0, 12.0), ("SENSOR_ERROR", 3, 1.25), ("DROPOUT", 4, 12.0),
                        ("NEAR_CLIPPED", 5, 0.0), ("DISTANCE_ONLY at 12", 6, 12.0), ("unknown code 9", 9, 12.0),
                        ("HIT with NaN", 1, float("nan")), ("HIT with +inf", 1, float("inf")),
                        ("HIT beyond max range", 1, 12.5), ("NO_HIT with 11.9", 2, 11.9),
                        ("NO_HIT with NaN", 2, float("nan")), ("HIT inside blind zone", 1, 0.1)):
    rep, out, _ = run(BAR_SCENE, PATH_X, mutate=fail_bar_beams(code, val))
    case("R1", "SAFETY", "beams on the bar declared %s: no certificate through the bar" % name,
         out["reason"] == "INCOMPLETE_ACQUISITION_PACKET" and not rep["complete"],
         {"reason": out["reason"], "outcomes": rep["beam_outcomes"]})
rep, out, _ = run(BAR_SCENE, OPEN_PATH, mutate=lambda r, s, w: s.__setitem__((0, slice(None)), core.DISTANCE_ONLY))
case("R1m", "SAFETY", "MOTAR-style distance-only packet (no hit bit): nothing certifies",
     out["reason"] == "INCOMPLETE_ACQUISITION_PACKET", {"reason": out["reason"], "outcomes": rep["beam_outcomes"]})

# ================================================================== R-2: no free evidence never certifies
ring = [pole(*P(0.30, a), 0.08) for a in range(0, 360, 5)]
rep, out, _ = run(ring, ((0.0, 0.0), (0.05, 0.0)))
case("R2", "SAFETY", "no usable free evidence (dense ring) is not a certificate",
     not out["static_bar_certified"], {"reason": out["reason"], "free_rays": rep["free_rays"]})

# ================================================================== R-3: labels
c = fresh("SYNTH-72x1-RUNTIME") if "SYNTH-72x1-RUNTIME" in core.REG.CONFIGS else fresh("SYNTH-72x4-RUNTIME")
r4, s4, _ = scan([], POSE, "MOTAR-72x4")
rep = c.step(r4, s4, *header(1, POSE, health=2))
out = c.certify(np.array([OPEN_PATH], float), 1)[0]
case("R3a", "CONTRACT", "synthetic health under a runtime config: every beam AMBIGUOUS, refused",
     rep["beam_outcomes"]["AMBIGUOUS"] == 288 and out["reason"] == "INCOMPLETE_ACQUISITION_PACKET", rep["beam_outcomes"])
c = fresh("SYNTH-72x4-RUNTIME")
rep = c.step(r4, s4, *header(1, POSE, health=1))
out = c.certify(np.array([OPEN_PATH], float), 1)[0]
case("R3b", "CONTRACT", "runtime-declared health: label RUNTIME_DECLARED, basis DECLARED_NOT_AUTHENTICATED, no SAFE flag",
     out["evidence_label"] == "RUNTIME_DECLARED" and out["health_basis"] == "DECLARED_NOT_AUTHENTICATED"
     and "safe" not in json.dumps(out).lower().replace("safe_reject", ""), {"label": out["evidence_label"]})
c = fresh()
rep = c.step(r0, s0, *header(1, POSE, health=0))
out = c.certify(np.array([OPEN_PATH], float), 1)[0]
case("R3c", "CONTRACT", "health UNKNOWN: every beam AMBIGUOUS", rep["beam_outcomes"]["AMBIGUOUS"] == 72
     and out["reason"] == "INCOMPLETE_ACQUISITION_PACKET", rep["beam_outcomes"])

# ================================================================== R-4: no exception escapes (fuzz)
rng = random.Random(4242)
nprng = np.random.default_rng(4242)
escaped, internal, n = 0, 0, 0
pool = [None, 0, 1.5, "x", [], (), {}, object(), np.zeros(3), np.zeros((1, 72)), np.zeros((1, 72), np.int8),
        np.zeros((4, 72)), np.zeros(5, np.int64), np.zeros(5), np.full((1, 72), np.nan), np.zeros((1, 72), object),
        np.zeros((2, 2, 2)), np.zeros((1, 2, 2)), Fake()]
for i in range(3000):
    c = fresh(rng.choice(["SYNTH-72x1", "SYNTH-72x4"]))
    shape = c.shape
    args = [rng.choice(pool + [nprng.normal(size=shape) * 20, nprng.integers(-3, 10, size=shape).astype(np.int8),
                               nprng.integers(-5, 2 ** 40, size=5).astype(np.int64),
                               np.array([rng.choice([0.0, 0.1, 1e300, -1.0, np.inf]), *nprng.normal(size=4)])])
            for _ in range(4)]
    try:
        n += 1
        rep = c.step(*args, camera=rng.choice(pool), track=rng.choice(pool))
        c.certify(rng.choice(pool + [nprng.normal(size=(rng.randint(1, 3), 2, 2))]), rng.choice([0, 1, -1, None, 1.0]),
                  deadline_s=rng.choice([None, 0.0, 1e-9, 1.0, "x"]))
        c.map_states(rng.choice(pool + [nprng.normal(size=(3, 2))]), rng.choice([0.0, 1, None]))
        core.certify_step(c, *args, rng.choice(pool + [nprng.normal(size=(2, 2, 2))]))
    except Exception as e:
        escaped += 1
    internal += len(c.exceptions)
case("R4", "SAFETY", "3,000 malformed calls of step/certify/map_states/certify_step: none raises",
     escaped == 0, {"calls": n, "escaped": escaped, "internal_errors_logged": internal})

# ================================================================== R-5: currency
c = fresh()
r_open, s_open, _ = scan([], POSE)
c.step(r_open, s_open, *header(1, POSE))
ok1 = c.certify(np.array([PATH_X], float), 1)[0]["static_bar_certified"]
bad = c.step(r0, s0.astype(np.int32), *header(2, POSE))          # the step-2 packet (bar present) is malformed
o_old = c.certify(np.array([PATH_X], float), 1)[0]
o_new = c.certify(np.array([PATH_X], float), 2)[0]
case("R5a", "SAFETY", "a refused newer packet leaves NO current packet (the older open-scene certificate is not reissued)",
     ok1 and not bad["accepted"] and o_old["reason"] == "NO_CURRENT_PACKET" and o_new["reason"] == "NO_CURRENT_PACKET",
     {"before": ok1, "after_old_step": o_old["reason"], "after_new_step": o_new["reason"]})
c = fresh()
c.step(r_open, s_open, *header(5, POSE))
rep_replay = c.step(r_open, s_open, *header(5, POSE))
rep_older = c.step(r_open, s_open, *header(4, POSE))
case("R5b", "SAFETY", "replayed and out-of-order steps refused", not rep_replay["accepted"] and not rep_older["accepted"]
     and rep_replay["reason"] == "STEP_NOT_INCREASING", {"replay": rep_replay["reason"], "older": rep_older["reason"]})
c = fresh()
c.step(r_open, s_open, *header(1, POSE))
c.step(r0, s0, *header(2, POSE))
o = c.certify(np.array([PATH_X], float), 1)[0]
case("R5c", "SAFETY", "a certificate request for an older step is refused", o["reason"] == "PACKET_NOT_CURRENT_FOR_STEP",
     {"reason": o["reason"]})

# ================================================================== R-6: range error bound and noise
c = fresh("SYNTH-72x1-ERRBOUND")
c.step(r_open, s_open, *header(1, POSE))
o = c.certify(np.array([OPEN_PATH], float), 1)[0]
case("R6a", "SAFETY", "a positive range error bound is not qualified", o["reason"] == "RANGE_ERROR_BOUND_NOT_QUALIFIED",
     {"reason": o["reason"]})
c = fresh()
rep = c.step(r_open, s_open, *header(1, POSE, noise=1))
o = c.certify(np.array([OPEN_PATH], float), 1)[0]
case("R6b", "SAFETY", "noise applied without a qualified bound: every beam AMBIGUOUS",
     o["reason"] == "INCOMPLETE_ACQUISITION_PACKET" and rep["beam_outcomes"]["AMBIGUOUS"] == 72, rep["beam_outcomes"])

# ================================================================== time / frame mixing
c = fresh()
hi, hf = header(3, POSE)
hf[0] = 0.25
rep = c.step(r_open, s_open, hi, hf)
case("T1", "SAFETY", "timestamp off the step schedule refused", rep["reason"] == "TIMESTAMP_NOT_ON_STEP_SCHEDULE", rep)
rep = fresh().step(r_open, s_open, *header(1, POSE, env=1))
case("T2", "SAFETY", "foreign env id refused", rep["reason"] == "ENV_OR_EPISODE_MISMATCH", rep)
rep = fresh().step(r_open, s_open, *header(1, POSE, episode=7))
case("T2b", "SAFETY", "foreign episode id refused", rep["reason"] == "ENV_OR_EPISODE_MISMATCH", rep)
for nm, pose in {"nan x": (np.nan, 0, 1, 0), "inf yaw": (0, 0, 1, np.inf), "yaw 100 rad": (0, 0, 1, 100.0),
                 "x 1e6": (1e6, 0, 1, 0)}.items():
    rep = fresh().step(r_open, s_open, *header(1, pose))
    case("T3", "SAFETY", "pose %s refused" % nm, not rep["accepted"], rep)
cam_late = camera(0.5, POSE, (2.0, 0.0, 1.0))
rep, out, _ = run(BAR_SCENE, OPEN_PATH, camera_=cam_late)
case("T4", "CONTRACT", "camera 0.4 s off the packet: ignored for labels, certificate unchanged",
     rep["camera"].startswith("IGNORED") and out == base, {"camera": rep["camera"]})

# ================================================================== UNKNOWN -> FREE, static deletion, blind zone
shadow = [bar(2.0, 0.0, 0.3, 0.3), bar(4.0, 0.0, 0.3, 0.3)]           # second bar hidden behind the first
rep, out, c = run(shadow, ((3.5, 0.6), (4.5, 0.6)))
case("U1", "SAFETY", "a segment next to an occluded bar (never observed) is not certified",
     not out["static_bar_certified"], {"reason": out["reason"]})
c = fresh()
c.step(r_open, s_open, *header(1, POSE))
for k in range(2, 30):
    c.step(r_open, np.zeros_like(s_open), *header(k, POSE))         # 28 steps of fully MISSING packets
st = c.map_states(np.array([[3.0, 0.0], [8.0, 3.0]]), 2.9)
case("U2", "SAFETY", "time with incomplete packets never makes a cell FREE", "VALIDATED_FREE" not in st, {"states": st})
POSE_S = (0.0, 0.025, 1.0, 0.0)                                     # beam 0 runs through cell centres (y frac 0.5)
c = fresh()
tr = lambda t: np.array([1.0, t, 9.0, 9.0, 0.2])                                # noqa: E731
for k in range(1, 26):
    rk, sk, _ = scan(BAR_SCENE, POSE_S)
    cam = camera(k * 0.1, POSE_S, (1.25, 0.025, 1.0)) if k > 20 else None      # camera later claims the bar is the target
    rep = c.step(rk, sk, *header(k, POSE_S), camera=cam, track=tr(k * 0.1))
QP = np.array([[1.25, 0.025]])                                       # beam 0's hit point on the bar face
st = c.map_states(QP, 2.5)
case("S1", "SAFETY", "a confirmed static bar stays STATIC after the camera labels it as the target",
     st == ["STATIC_OCCUPANCY"] and rep["labelled_returns"] > 0, {"state": st, "labels": rep["labelled_returns"]})
c2 = c
r_open_s, s_open_s, _ = scan([], POSE_S)
for k in (26, 27, 28):                                              # the bar 'disappears' (free rays through its cell)
    c2.step(r_open_s, s_open_s, *header(k, POSE_S))
st = c2.map_states(QP, 2.8)
o = c2.certify(np.array([((0.0, 0.025), (2.5, 0.025))], float), 28)[0]
o_nomap = fresh()
o_nomap.step(r_open_s, s_open_s, *header(1, POSE_S))
o_nomap = o_nomap.certify(np.array([((0.0, 0.025), (2.5, 0.025))], float), 1)[0]
case("S2", "SAFETY", "free evidence after STATIC gives CONFLICT (veto kept), never FREE",
     st == ["CONFLICT"] and o["reason"] == "MAP_VETO_CONFLICT" and o_nomap["static_bar_certified"],
     {"state": st, "certify": o["reason"], "same_packet_without_history": o_nomap["reason"]})
near = [bar(0.40, 0.0, 0.25, 0.25)]                                 # surface 0.15 m from the sensor
rep, out, _ = run(near, OPEN_PATH)
case("Z1", "SAFETY", "object inside the 0.2 m blind zone: near-clipped beams make the packet incomplete",
     out["reason"] == "INCOMPLETE_ACQUISITION_PACKET" and rep["beam_outcomes"]["AMBIGUOUS"] > 0, rep["beam_outcomes"])
rep, out, _ = run([], OPEN_PATH)
rep2, out2, _ = run([], OPEN_PATH, cfg="SYNTH-72x1-NOASSURE")
case("Z2", "CONTROL", "open scene certifies with and without nearest-hit assurance (normal function)",
     out["static_bar_certified"] and out2["static_bar_certified"], {"assured": out["reason"], "not": out2["reason"]})

# ================================================================== summary
by = {}
for r in ROWS:
    by.setdefault(r["class"], [0, 0])
    by[r["class"]][0] += r["pass"]
    by[r["class"]][1] += 1
summary = {k: "%d/%d" % tuple(v) for k, v in by.items()}
print(json.dumps(summary))
dump(sys.argv[1], {"summary": summary, "pass": all(r["pass"] for r in ROWS), "rows": ROWS})
