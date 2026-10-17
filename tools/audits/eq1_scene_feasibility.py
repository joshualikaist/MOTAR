#!/usr/bin/env python3
"""EQ-1B scene — URDF collision versus visual geometry and start-to-capture passability (CPU).

    python -B tools/audits/eq1_scene_feasibility.py [--out docs/evidence/.../eq1b_scene.json] [--workers 12]

Validation only; raw OCC-P2 results are read, never written. Two parts:

1. URDF (independent XML parse; no simulator import). Every visual primitive of the Policy F body
   (`quad_navrl_collide.urdf`) is transformed by its joint and visual origins and its exact AABB is
   compared with the base_link collision box. The 40 bars_h3 assets are checked for visual ==
   collision and against the recorded layout half extents.
2. Kinematic passability on every recorded OCC-P2 quota layout. Bars are full-height (3 m = arena
   height), so 3-D contact with an upright bar equals 2-D contact of the body's horizontal
   projection with the bar footprint. Two bodies are declared before execution:
     conservative: the collision box's circumscribed sphere, r = sqrt(0.14^2 + 0.14^2 + 0.04^2), any attitude;
     optimistic:   r = 0.14 cos 45 deg, contained in the projection at every tilt <= 45 deg (NAVRL_MAX_TILT_DEG);
   plus a reserve tier that adds the riskcap hard margin (0.45 m) to both. The body centre must stay
   within the OOB limit [0, 40] +- 1.0 m; the goal is the capture disc (success_radius 0.5 m) around
   the target's position in the first recorded frame. Cells are squares with exact box distances:
     CONNECTED     a 4-connected chain of cells wholly farther than r_cons from every bar (a certified path);
     DISCONNECTED  no 8-connected chain of cells that are not wholly within r_opt of one bar (certified: no
                   continuous path exists for the optimistic body);
     UNRESOLVED    neither at the finest grid (0.1 -> 0.05 -> 0.025 m). A failed coarse search is never
                   read as impossibility.
   This is path existence for a holonomic body. Dynamically feasible trajectories (thrust, tilt,
   braking, tracking error) are NOT_TESTED here.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
import csv
import gzip
import io
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy import ndimage

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eq1_common as common  # noqa: E402

ROBOT_URDF = "resources/robots/quad/quad_navrl_collide.urdf"
BAR_DIR = "resources/models/environment_assets/bars_h3"
BOX_HALF = np.array([0.14, 0.14, 0.04])
R_CONS = float(np.linalg.norm(BOX_HALF))
R_OPT = 0.14 * math.cos(math.radians(45.0))
RESERVE_M = 0.45
ARENA = (0.0, 40.0)
OOB_MARGIN = 1.0
CAPTURE_R = 0.5
GRIDS = (0.1, 0.05, 0.025)
CONTACT_TOL = 1e-9


# ------------------------------------------------------------------ URDF
def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def origin(el):
    o = el.find("origin") if el is not None else None
    xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0").split() if o is not None else "0 0 0".split())])
    rpy = [float(v) for v in (o.get("rpy", "0 0 0").split() if o is not None else "0 0 0".split())]
    return xyz, rpy_matrix(*rpy)


def primitive_aabb(geom, pos, rot):
    """Exact AABB half extents of a URDF primitive placed at pos with rotation rot."""
    g = list(geom)[0]
    if g.tag == "sphere":
        half = np.full(3, float(g.get("radius")))
    elif g.tag == "box":
        size = np.array([float(v) for v in g.get("size").split()]) / 2
        half = np.abs(rot) @ size
    elif g.tag == "cylinder":
        r, length = float(g.get("radius")), float(g.get("length"))
        axis = rot[:, 2]
        half = np.abs(axis) * length / 2 + r * np.sqrt(np.clip(1 - axis ** 2, 0, 1))
    else:
        raise ValueError("unsupported geometry %s" % g.tag)
    return pos, half, g.tag


def robot_geometry(path):
    root = ET.parse(path).getroot()
    joints = {j.find("child").get("link"): j for j in root.findall("joint")}
    visuals, collisions = [], []
    for link in root.findall("link"):
        name = link.get("name")
        jpos, jrot = (np.zeros(3), np.eye(3))
        if name in joints:
            jpos, jrot = origin(joints[name])
            if joints[name].find("parent").get("link") != "base_link":
                raise ValueError("nested chains are not expected in this URDF")
        for kind, out in (("visual", visuals), ("collision", collisions)):
            for el in link.findall(kind):
                vpos, vrot = origin(el)
                pos, half, tag = primitive_aabb(el.find("geometry"), jpos + jrot @ vpos, jrot @ vrot)
                out.append({"link": name, "type": tag, "centre": pos.tolist(), "half": half.tolist()})
    return visuals, collisions


def case_urdf():
    visuals, collisions = robot_geometry(common.ROOT / ROBOT_URDF)
    assert len(collisions) == 1 and collisions[0]["link"] == "base_link"
    box = np.array(collisions[0]["half"])
    rows = []
    for v in visuals:
        lo = np.array(v["centre"]) - np.array(v["half"])
        hi = np.array(v["centre"]) + np.array(v["half"])
        excess = np.maximum(np.maximum(hi - box, -box - lo), 0.0)
        rows.append({**v, "outside_collision_m": float(excess.max()),
                     "touches_collision_boundary": bool(np.any(np.abs(np.maximum(hi, -lo) - box) < 1e-9))})
    inside = all(r["outside_collision_m"] <= CONTACT_TOL for r in rows)
    for r in rows:
        lo = np.array(r["centre"]) - np.array(r["half"])
        hi = np.array(r["centre"]) + np.array(r["half"])
        r["outside_collision_xy_m"] = float(np.maximum(np.maximum(hi - box, -box - lo), 0.0)[:2].max())
    inside_xy = all(r["outside_collision_xy_m"] <= CONTACT_TOL for r in rows)
    bars = []
    for p in sorted((common.ROOT / BAR_DIR).glob("bar_*.urdf")):
        link = ET.parse(p).getroot().find("link")
        vs = [c.find("geometry/box").get("size") for c in link.findall("visual")]
        cs = [c.find("geometry/box").get("size") for c in link.findall("collision")]
        vo = [origin(c)[0].tolist() for c in link.findall("visual")]
        co = [origin(c)[0].tolist() for c in link.findall("collision")]
        bars.append({"asset": p.name, "visual": vs, "collision": cs, "equal": vs == cs and vo == co,
                     "half": (np.array([float(x) for x in cs[0].split()]) / 2).tolist()})
    return {"name": "B3 URDF collision versus visual geometry",
            "robot": {"urdf": ROBOT_URDF, "collision_half_m": box.tolist(), "visuals": rows,
                      "visual_inside_collision_3d": common.verdict(inside, "base_link visual sphere r 0.05 m exceeds the "
                                                                   "0.08 m-thick box by 0.01 m above and below"),
                      "visual_inside_collision_xy": common.verdict(inside_xy, "horizontal projection: arms and motors inside; "
                                                                   "motor cylinders reach the +-0.14 m faces exactly"),
                      "rotors_modelled": False,
                      "rotor_envelope": {"status": "NOT_TESTED", "note": "no propeller geometry in the URDF; the box is a "
                                         "proxy bounding motors only, not a measured rotor envelope"},
                      "circumscribed_sphere_m": R_CONS, "yaw_circumscribed_xy_m": float(np.hypot(0.14, 0.14)),
                      "inscribed_at_45deg_tilt_m": R_OPT},
            "bars": {"assets": len(bars), "all_visual_equal_collision": common.verdict(all(b["equal"] for b in bars),
                                                                                      "box size and origin identical"),
                     "rows": bars}}


def bar_halves():
    return np.array([b["half"] for b in case_urdf()["bars"]["rows"]], dtype=np.float32)


# ------------------------------------------------------------------ passability
def _grid(h):
    lo, hi = ARENA[0] - OOB_MARGIN, ARENA[1] + OOB_MARGIN
    n = int(math.ceil((hi - lo) / h - 1e-9))
    return lo + (np.arange(n) + 0.5) * h, lo, n


def _masks(centres, halves, h, r_cons, r_opt):
    """blocked_cons: some point of the cell within r_cons of a bar; blocked_opt: every point within r_opt of one bar."""
    xs, lo, n = _grid(h)
    blocked_cons = np.zeros((n, n), dtype=bool)
    blocked_opt = np.zeros((n, n), dtype=bool)
    reach = max(r_cons, r_opt) + h
    for c, hf in zip(centres, halves):
        i0 = max(0, int((c[0] - hf[0] - reach - lo) / h)); i1 = min(n, int((c[0] + hf[0] + reach - lo) / h) + 2)
        j0 = max(0, int((c[1] - hf[1] - reach - lo) / h)); j1 = min(n, int((c[1] + hf[1] + reach - lo) / h) + 2)
        if i0 >= i1 or j0 >= j1:
            continue
        ax = np.abs(xs[i0:i1] - c[0])[:, None]
        ay = np.abs(xs[j0:j1] - c[1])[None, :]
        near = np.maximum(ax - hf[0] - h / 2, 0) ** 2 + np.maximum(ay - hf[1] - h / 2, 0) ** 2
        far = np.maximum(ax + h / 2 - hf[0], 0) ** 2 + np.maximum(ay + h / 2 - hf[1], 0) ** 2
        blocked_cons[i0:i1, j0:j1] |= near <= r_cons ** 2
        blocked_opt[i0:i1, j0:j1] |= far <= r_opt ** 2
    return xs, lo, n, blocked_cons, blocked_opt


def classify(centres, halves, start, target, r_cons, r_opt):
    """CONNECTED / DISCONNECTED / UNRESOLVED (+ the grid that decided) for one start/target pair."""
    for h in GRIDS:
        xs, lo, n, bc, bo = _masks(centres, halves, h, r_cons, r_opt)
        si, sj = int((start[0] - lo) / h), int((start[1] - lo) / h)
        if not (0 <= si < n and 0 <= sj < n):
            return "START_OUTSIDE_REGION", h
        gx = np.maximum(np.abs(xs - target[0]) - h / 2, 0)
        gy = np.maximum(np.abs(xs - target[1]) - h / 2, 0)
        goal = gx[:, None] ** 2 + gy[None, :] ** 2 < CAPTURE_R ** 2
        lab_c, _ = ndimage.label(~bc)
        connected = lab_c[si, sj] != 0 and bool(np.any(lab_c[goal] == lab_c[si, sj]))
        lab_o, _ = ndimage.label(~bo, structure=np.ones((3, 3), dtype=int))
        reachable = lab_o[si, sj] != 0 and bool(np.any(lab_o[goal] == lab_o[si, sj]))
        if connected and not reachable:
            raise AssertionError("certificate contradiction: conservative path but optimistic disconnection")
        if connected:
            return "CONNECTED", h
        if not reachable:
            return "DISCONNECTED", h
    return "UNRESOLVED", GRIDS[-1]


def cell_passability(cell_dir):
    from motar_eval import occlusion_trace as ot

    common.assert_in_tree(ot)
    cell_dir = Path(cell_dir)
    trace = ot.read_trace(cell_dir)
    layouts, frames = trace["layouts"], trace["frames"]
    lookup = ot.reset_lookup(layouts)
    eps = ot.episode_frames(frames)
    rows = [json.loads(l) for l in open(cell_dir / "episodes.jsonl")]
    out = []
    for r in rows:
        if not r["in_quota"]:
            continue
        key = (r["env_index"], r["env_episode_index"])
        ef = eps.get(key)
        if ef is None or key not in lookup:
            out.append({"key": key, "outcome": r["outcome"], "status": "NO_TRACE"})
            continue
        first = ef[np.argmin(ef["frame"])]
        centres, halves, _ = ot.layout_arrays(layouts, lookup[key])
        start, target = first["p_pos"][:2].astype(np.float64), first["t_pos"][:2].astype(np.float64)
        body, hb = classify(centres[:, :2], halves[:, :2], start, target, R_CONS, R_OPT)
        reserve, hr = classify(centres[:, :2], halves[:, :2], start, target, R_CONS + RESERVE_M, R_OPT + RESERVE_M)
        out.append({"key": key, "outcome": r["outcome"], "body": body, "body_grid_m": hb,
                    "reserve": reserve, "reserve_grid_m": hr, "start_target_m": float(np.linalg.norm(start - target))})
    return {"cell": cell_dir.name, "group": cell_dir.parent.parent.name, "bars": int(rows[0]["bars"]),
            "seed": int(rows[0]["seed"]), "layout_halves": np.unique(layouts["half"][:, :2], axis=0).tolist(),
            "episodes": out}


def cell_dirs(root=common.OCC_P2):
    return sorted(p for p in Path(root).glob("*/*/*") if (p / "occlusion_frames.bin").exists())


def summarize(cells):
    table = {}
    contradictions = []
    for c in cells:
        for e in c["episodes"]:
            k = "%s/%d" % (c["group"], c["bars"])
            t = table.setdefault(k, {"episodes": 0, "body": {}, "reserve": {}, "captures_by_body": {}})
            t["episodes"] += 1
            for tier in ("body", "reserve"):
                s = e.get(tier, e.get("status"))
                t[tier][s] = t[tier].get(s, 0) + 1
            s = e.get("body", e.get("status"))
            if e["outcome"] == "capture":
                t["captures_by_body"][s] = t["captures_by_body"].get(s, 0) + 1
                if s == "DISCONNECTED":
                    contradictions.append({"cell": c["cell"], "key": e["key"]})
    return table, contradictions


def case_passability(workers=12, root=common.OCC_P2):
    dirs = cell_dirs(root)
    with ProcessPoolExecutor(max_workers=workers) as pool:
        cells = list(pool.map(cell_passability, dirs))
    table, contradictions = summarize(cells)
    halves = bar_halves()
    recorded = np.unique(np.concatenate([np.array(c["layout_halves"], dtype=np.float32) for c in cells]), axis=0)
    unmatched = [h.tolist() for h in recorded if not np.any(np.all(np.abs(halves[:, :2] - h) < 1e-6, axis=1))]
    total = sum(t["episodes"] for t in table.values())
    certified = sum(t["body"].get("CONNECTED", 0) for t in table.values())
    not_certified = [{"cell": c["cell"], "env": e["key"][0], "episode": e["key"][1], "outcome": e["outcome"],
                      "body": e.get("body"), "reserve": e.get("reserve")}
                     for c in cells for e in c["episodes"] if e.get("body") != "CONNECTED" or e.get("reserve") != "CONNECTED"]
    return {"name": "B4 start-to-capture passability on recorded OCC-P2 layouts (kinematic only)",
            "root": str(root), "cells": len(cells), "quota_episodes": total, "radii_m": {
                "conservative_body": R_CONS, "optimistic_body": R_OPT, "reserve_added": RESERVE_M},
            "grids_m": list(GRIDS), "by_group_density": table,
            "captures_in_certified_disconnected": contradictions,
            "not_certified_at_some_tier": not_certified,
            "recorded_half_extents_not_in_bar_assets": unmatched,
            "all_scenes_certified_connected": common.verdict(certified == total,
                                                             "%d of %d quota scenes certified at the body tier" % (certified, total)),
            "dynamic_feasibility": {"status": "NOT_TESTED", "note": "holonomic path existence only; thrust/tilt/braking "
                                    "and tracking error are not modelled"},
            "per_cell": cells}


def write_episode_table(path, case):
    """Per-episode rows as gzip CSV (the summary JSON keeps only the counts)."""
    with open(path, "wb") as raw, gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as gz, \
            io.TextIOWrapper(gz, newline="") as fh:  # mtime 0: byte-reproducible
        w = csv.writer(fh)
        w.writerow(["group", "bars", "seed", "cell", "env", "episode", "outcome", "body", "body_grid_m",
                    "reserve", "reserve_grid_m", "start_target_m"])
        for c in case["per_cell"]:
            for e in c["episodes"]:
                w.writerow([c["group"], c["bars"], c["seed"], c["cell"], e["key"][0], e["key"][1], e["outcome"],
                            e.get("body", e.get("status")), e.get("body_grid_m", ""), e.get("reserve", e.get("status")),
                            e.get("reserve_grid_m", ""), "%.6f" % e["start_target_m"] if "start_target_m" in e else ""])


def run(workers=12):
    return {"schema": "motar.eq1b-scene.v1", "package": "EQ-1B scene feasibility (CPU, validation only)",
            "source": common.source_binding([ROBOT_URDF, BAR_DIR, "motar_eval/occlusion_trace.py"]),
            "cases": [case_urdf(), case_passability(workers)]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out")
    parser.add_argument("--workers", type=int, default=12)
    args = parser.parse_args(argv)
    record = run(args.workers)
    passability = record["cases"][1]
    if args.out:
        table = Path(args.out).with_name(Path(args.out).stem + "_episodes.csv.gz")
        write_episode_table(table, passability)
        passability["episode_table"] = {"file": table.name, "sha256": common.sha256_file(table)}
    passability.pop("per_cell")
    if args.out:
        common.write_record(args.out, record)
    print(json.dumps(passability["by_group_density"], indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
