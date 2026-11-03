"""M3 development checks of the C-Planner skeleton (cplanner/planner/c_planner.py). Not a gate.

Synthetic 72x4 scans come from the independent harness ray caster (`harness.scan`, MOTAR-72x4 layout) and are turned
into the NORMALISED values the warp sensor delivers (HIT t/12, NO_HIT 1.0, NEAR_CLIPPED -1.0). Every check pairs a
refusal with a positive control.
usage: planner_unit.py OUT_JSON
"""
import json
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "planner"))
from harness import CATALOG, R_PLAN, bar, classify, clearance_by_kind, scan, target   # noqa: E402
import core                                                                             # noqa: E402
import c_planner as CP                                                                  # noqa: E402


def norm_scan(objs, pose):
    r, s, _ = scan(objs, pose, "MOTAR-72x4")
    v = np.where(s == core.HIT, r / 12.0, np.where(s == core.NO_HIT, 1.0, -1.0))
    return v.astype(np.float32)


def segs(path):
    return [((float(path[i, 0]), float(path[i, 1])), (float(path[i + 1, 0]), float(path[i + 1, 1])))
            for i in range(len(path) - 1)]


def executed_path(pos, vel, info, act, yaw):
    """Rebuild the executed polyline from the action (world velocity command) for oracle checks."""
    c, s = math.cos(yaw), math.sin(yaw)
    vx, vy = act[0] * CP.V_MAX, act[1] * CP.V_MAX
    v_cmd = np.array([c * vx - s * vy, s * vx + c * vy])
    return CP.flown_path(np.asarray(pos[:2], float), np.asarray(vel[:2], float), v_cmd)


def main():
    out = pathlib.Path(sys.argv[1])
    if out.exists():
        raise SystemExit("refusing to overwrite " + str(out))
    checks, notes = {}, {}

    # ---- pure functions
    g, tau = CP.intercept((0.0, 0.0), (5.0, 0.0), (0.0, 0.0))
    checks["intercept_stationary"] = abs(tau - 2.0) < 1e-12 and np.allclose(g, (5.0, 0.0))
    g, tau = CP.intercept((0.0, 0.0), (5.0, 0.0), (3.0, 0.0))
    checks["intercept_unreachable_caps_tau"] = tau == CP.TAU_MAX and np.allclose(g, (14.0, 0.0))
    p = CP.flown_path((1.0, 1.0), (2.5, 0.0), np.array([2.5, 0.0]))
    checks["flown_path_steady"] = np.allclose(p[-1], (1.0 + 2.5 * CP.HORIZON_S, 1.0)) and p.shape == (CP.N_POINTS + 1, 2)

    v = np.ones((4, 72), np.float32)
    v[2, 10] = 3.0 / 12.0                 # 0 deg row, azimuth 180 - 50 = 130 deg
    v[0, 20] = 3.0 / 12.0                 # +20 deg row, azimuth 80 deg
    v[3, 30] = -1.0                       # NEAR_CLIPPED, azimuth 30 deg
    v[2, 40] = 2.0 / 12.0                 # masked as target-like
    mask = np.zeros((4, 72), bool)
    mask[2, 40] = True
    yaw, pos = 0.3, (1.0, 2.0, 1.0)
    pts = CP.scan_points(v, pos, yaw, mask)
    exp = []
    for row, col, rng in ((0, 20, 3.0 * math.cos(math.radians(20.0))), (2, 10, 3.0),
                          (3, 30, 0.2 * math.cos(math.radians(-10.0)))):
        a = math.radians(180.0 - 5.0 * col) + yaw
        exp.append((pos[0] + rng * math.cos(a), pos[1] + rng * math.sin(a)))
    got = sorted(map(tuple, np.round(pts, 9)))
    checks["scan_points_geometry"] = len(pts) == 3 and np.allclose(sorted(exp), got, atol=1e-5)
    checks["scan_points_mask_positive_control"] = len(CP.scan_points(v, pos, yaw, None)) == 4

    obst = np.array([[0.3, 0.0]])
    away = CP.flown_path((0.0, 0.0), (0.0, 0.0), np.array([-1.25, 0.0]))
    toward = CP.flown_path((0.0, 0.0), (0.0, 0.0), np.array([1.25, 0.0]))
    checks["escape_admissible"] = CP.admissible(away, obst, 0.3)[0] is True
    checks["escape_toward_refused"] = CP.admissible(toward, obst, 0.3)[0] is False
    far = np.array([[2.8, 0.0]])                                  # steady 2.5 m/s path ends 0.3 m short of it
    short = CP.flown_path((0.0, 0.0), (0.0, 0.0), np.array([1.25, 0.0]))
    checks["outside_keep_r_safe"] = (CP.admissible(short, far, 2.8)[0] is True
                                     and CP.admissible(CP.flown_path((0.0, 0.0), (2.5, 0.0), np.array([2.5, 0.0])),
                                                       far, 2.8)[0] is False)

    # ---- closed scenes through the certifier (MOTAR-72x4-RUNTIME; arena 0-40 m, so the scenes sit at (20, 20))
    X0, Y0 = 20.0, 20.0
    pose = (X0, Y0, 1.0, 0.0)
    tgt = target(X0 + 5.0, Y0, 1.0)
    T = (X0 + 5.0, Y0, 1.0)
    empty = [tgt]
    nv = norm_scan(empty, pose)
    results = {}
    for mode in CP.MODES:
        pl = CP.CPlanner(0, 0, mode)
        act, info = pl.act(np.int64(1), pose[:3], (0.0, 0.0, 0.0), pose[3], T, (0.0, 0.0, 0.0), True, nv)
        results[mode] = info
        checks["empty_%s_certified_direct" % mode] = (info["behaviour"] == "NAVIGATE" and info["certified"] is True
                                                      and info["offset"] == 0.0 and info["speed"] == 2.5
                                                      and act[0] > 0.99 and abs(act[1]) < 1e-9)
    notes["empty"] = {m: {k: results[m][k] for k in ("behaviour", "certified", "offset", "speed")} for m in results}

    hx, hy = CATALOG[0]
    blocked = [bar(X0 + 2.0, Y0, hx, hy), tgt]
    nv = norm_scan(blocked, pose)
    for mode in CP.MODES:
        pl = CP.CPlanner(0, 0, mode)
        act, info = pl.act(1, pose[:3], (0.0, 0.0, 0.0), pose[3], T, (0.0, 0.0, 0.0), True, nv)
        path = executed_path(pose, (0.0, 0.0), info, act, pose[3])
        bar_clear = min(clearance_by_kind(sg, blocked, 1.0)["bar"] for sg in segs(path))
        cls = [classify({"static_bar_certified": True}, sg, blocked, 1.0) for sg in segs(path)]
        notes["blocked_" + mode] = {"behaviour": info["behaviour"], "certified": info["certified"],
                                    "offset": info.get("offset"), "bar_clearance_minus_R": bar_clear,
                                    "filter_tries": info.get("filter_tries")}
        checks["blocked_%s_avoids_bar" % mode] = (info["behaviour"] == "NAVIGATE" and info["offset"] != 0.0
                                                  and bar_clear > 0.0)
        if mode == "FILTER":
            checks["blocked_FILTER_certified_is_valid"] = info["certified"] is True and "FALSE_CERT" not in cls

    near = [bar(X0 + 0.36 + hx, Y0, hx, hy), tgt]                 # face 0.36 m ahead of the vehicle
    nv = norm_scan(near, pose)
    pl = CP.CPlanner(0, 0, "OFF")
    act, info = pl.act(1, pose[:3], (0.0, 0.0, 0.0), pose[3], T, (0.0, 0.0, 0.0), True, nv)
    path = executed_path(pose, (0.0, 0.0), info, act, pose[3])
    checks["inside_r_safe_escapes"] = (info["behaviour"] == "NAVIGATE" and info.get("escape") is True
                                       and info["clearance_now"] < CP.R_SAFE and act[0] < 0.0
                                       and pl.counts["escape"] == 1)
    notes["inside"] = {"clearance_now": info.get("clearance_now"), "offset": info.get("offset"),
                       "end": path[-1].tolist()}

    nv = norm_scan(empty, pose).copy()
    nv[1, 5] = np.nan                                            # SENSOR_ERROR: incomplete packet
    pl_off, pl_f = CP.CPlanner(0, 0, "OFF"), CP.CPlanner(0, 0, "FILTER")
    _, i_off = pl_off.act(1, pose[:3], (0.0, 0.0, 0.0), 0.0, T, (0.0, 0.0, 0.0), True, nv)
    a_f, i_f = pl_f.act(1, pose[:3], (0.0, 0.0, 0.0), 0.0, T, (0.0, 0.0, 0.0), True, nv)
    checks["incomplete_OFF_executes_uncertified"] = (i_off["certified"] is False
                                                     and i_off["reasons"] == ["INCOMPLETE_ACQUISITION_PACKET"])
    checks["incomplete_FILTER_brakes_after_cap"] = (i_f["behaviour"] == "BRAKE" and i_f["filter_tries"] ==
                                                    CP.MAX_FILTER_TRIES and abs(a_f[0]) < 1e-12
                                                    and abs(a_f[1]) < 1e-12 and pl_f.counts["filter_exhausted"] == 1)

    pl = CP.CPlanner(0, 0, "OFF")
    act, info = pl.act(1, pose[:3], (0.0, 0.0, 0.0), 0.0, T, (0.0, 0.0, 0.0), False,
                       norm_scan(empty, pose))
    checks["no_belief_searches"] = info["behaviour"] == "SEARCH" and np.allclose(act, (0, 0, 0, 0.2))
    act, info = pl.act(2, pose[:3], (0.0, 0.0, 0.0), 0.0, (X0 + 0.7, Y0, 1.0), (0.0, 0.0, 0.0), True,
                       norm_scan([target(X0 + 0.7, Y0, 1.0)], pose))
    checks["terminal_uncertified_pursuit"] = (info["behaviour"] == "TERMINAL" and info["certified"] is False
                                              and act[0] > 0.99)

    res = {"suite": "M3 planner development checks (not a gate)", "pass": all(checks.values()), "checks": checks,
           "notes": notes, "params": {k: getattr(CP, k) for k in ("R_SAFE", "EXCLUDE_M", "MAX_FILTER_TRIES",
                                                                    "HORIZON_S", "N_POINTS", "TAU_V")}}
    with out.open("x") as fh:
        json.dump(res, fh, indent=1, default=float)
        fh.write("\n")
    print(json.dumps({"pass": res["pass"], "failed": [k for k, v in checks.items() if not v], "notes": notes},
                     indent=1, default=float))
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
