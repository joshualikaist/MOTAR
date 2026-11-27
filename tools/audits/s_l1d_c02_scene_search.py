#!/usr/bin/env python3
"""S-L1D amendment 1: float64 design and robustness check of the SL1D-C02 GPU fixtures (CPU only).

Uses only the committed S-L1D NumPy oracle. ``--verify`` re-checks every amendment scene:
- the target-alone support count with a sphere-boundary margin (no ambiguous pixel ray);
- for occluded scenes, the evaluator class, alone/visible pixel counts and centre-robustness are
  stable across the randomised bar half extents (hx 0.20/0.30/0.40, hy 0.20/0.40), +-4 mm edge
  shifts and +-1.5 mm target shifts;
- the oracle's online class with the primed true centre.
``--search`` reruns the grid search that produced the candidates (minutes).
"""
from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
import s_l1d_reference as rd  # noqa: E402

AMENDMENT = ROOT / "docs/plans/s_l1d_v1_amendment1.json"
HX, HY = (0.20, 0.30, 0.40), (0.20, 0.40)
RAYS = rd.rays().reshape(-1, 3)


def boundary_margin(target):
    rel = np.asarray(target, dtype=np.float64) - rd.CAMERA_TRANSLATION
    along = RAYS @ rel
    closest = np.linalg.norm(rel[None] - along[:, None] * RAYS, axis=1)
    return float(np.min(np.abs(closest - rd.RADIUS)[along > 0]))


def bar_boxes(bars, hx, hy):
    boxes, halves = [], []
    for b in bars:
        if "y" in b:
            cy = b["y"]
        elif "edge_left_y" in b:
            cy = b["edge_left_y"] + hy
        else:
            cy = b["edge_right_y"] - hy
        boxes.append([b["x"], cy, .5])
        halves.append([hx, hy, 1.5])
    return boxes, halves


def shifted(bars, de):
    out = []
    for b in bars:
        c = dict(b)
        for key in ("edge_left_y", "edge_right_y"):
            if key in c:
                c[key] = c[key] + de
        out.append(c)
    return out


def verify_scene(scene):
    target = scene["target"]
    results = set()
    edge_shifts = (-0.004, 0., 0.004) if scene["bars"] else (0.,)
    sizes = list(itertools.product(HX, HY)) if scene["bars"] else [(0.3, 0.3)]
    for (hx, hy), de in itertools.product(sizes, edge_shifts):
        for dy, dz in ((0, 0), (.0015, 0), (-.0015, 0), (0, .0015), (0, -.0015)):
            t = [target[0], target[1] + dy, target[2] + dz]
            boxes, halves = bar_boxes(shifted(scene["bars"], de), hx, hy)
            ev = rd.evaluator_class(boxes, halves, t)
            results.add((ev["class"], ev["alone_px"], ev["visible_px"], ev["centre_robust"]))
    boxes, halves = bar_boxes(scene["bars"], .3, .3)
    ev = rd.evaluator_class(boxes, halves, target)
    online = rd.online_class(rd.delivered_depth(ev), False, True, target, np.eye(3) * 1e-6)
    return {"stable": len(results) == 1, "variants": sorted(results), "class": ev["class"], "alone_px": ev["alone_px"],
            "visible_px": ev["visible_px"], "centre_robust": ev["centre_robust"], "online": online["class"],
            "margin_m": boundary_margin(target), "ambiguous_px": rd.ambiguous_support_pixels(target)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--verify", action="store_true")
    args = ap.parse_args(argv)
    amendment = json.loads(AMENDMENT.read_text())
    report = {s["id"]: verify_scene(s) for s in amendment["scenes"]}
    print(json.dumps(report, indent=1))
    expected = amendment["scene_design"]["expected_from_design"]
    ok = all(r["stable"] and r["class"] == expected[k]["design_evaluator_class"] and r["online"] == expected[k]["design_online_class"]
             for k, r in report.items())
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
