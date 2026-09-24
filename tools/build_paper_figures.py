#!/usr/bin/env python3
"""Build the MOTAR paper figure family (Figures 1-6) from committed evidence.

One generator, one visual system (see DESIGN_SYSTEM.md). Every number drawn is read
from a canonical machine-readable source listed in SOURCES, recorded in the output
manifest next to the SHA-256 of that source, and checked by
tests/test_paper_figures_final.py. Schematic panels are labelled "schematic"; no
panel shows fabricated imagery or data.

    python tools/build_paper_figures.py            # write docs/assets/paper/final/
    python tools/build_paper_figures.py --check    # rebuild in memory, compare
    python tools/build_paper_figures.py --out DIR  # draft into another directory

Byte-exact regeneration needs the recorded matplotlib version and the Liberation Sans
font. With another matplotlib version --check verifies the source-bound values only,
and says so.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
import os
from pathlib import Path
import random
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/assets/paper/final"

SOURCES = {
    "registry": "docs/quantitative_positioning_registry.json",
    "status_manifest": "docs/status_manifest.json",
    "ledger": "docs/literature_quantitative_ledger_2026-09-18.json",
    "target_motion": "results/target_motion_e0_e2_2026-09-18/canonical_summary.json",
    "visibility": "docs/results/target_motion_visibility_reacquisition_audit_2026-09-19.json",
    "e3s": "results/eth_ds5_e3s_2026-09-10/run/e3s_result.json",
    "p8": "results/perception_p8_2026-09-09/error_model.json",
    "p9_fit": "results/perception_p9_2026-09-09/goodness_of_fit.json",
    "p10": "results/perception_p10_seed_replication_2026-09-10/summary.json",
    "safety": "results/independent_verification_2026-09-07/recomputed.json",
    "d8b": "results/dynamic_mesh_policy_sensitivity_d8b_2026-09-13/summary.json",
    "coverage": "tools/build_quantitative_positioning_figure.py",
    "relation": "docs/relation_to_published_systems_2026-09-16.md",
}

# ---------------------------------------------------------------- visual system
# Tokens are documented in DESIGN_SYSTEM.md; the three data hues pass the dataviz
# palette validator (light surface, all pairs).
INK = "#1f1f1f"        # primary text and key numbers
INK2 = "#4a4a4f"       # secondary text, arrows
MUTED = "#6e6e73"      # notes, axis text
RULE = "#cfcbc3"       # box borders, axis spines
GRID = "#ebe8e2"       # gridlines
PANEL = "#f7f6f2"      # schematic fill
OBSTACLE = "#b9b5ad"   # schematic obstacles
WHITE = "#ffffff"
BLUE = "#2f6599"       # data slot 1, research blue
BLUE_WASH = "#e8eef6"
ACCENT = "#c2562b"     # data slot 2, restrained accent (attention marks only)
TEAL = "#1c9a7b"       # data slot 3
REF = "#8e8e93"        # reference arm / baseline marks
LIGHT = "#d9d6cf"      # remainder segments
FONT = "Liberation Sans"
FONT_STACK = "'Liberation Sans', Arial, Helvetica, sans-serif"
MONO = "Liberation Mono"
MONO_STACK = "'Liberation Mono', Menlo, Consolas, monospace"
MINUS = "−"
# Figures 1-6 are drawn at their printed width (a full two-column page), so point sizes in the files
# are the printed sizes: labels and ticks >= 8 pt, notes >= 7.5 pt, nothing essential below 7 pt.
PAPER_W = 7.0
WIDTH_IN = 10.0          # the system map is a screen diagram for the README and site
T_LETTER, T_TITLE, T_LABEL, T_NOTE, T_TAG = 10, 9, 8, 7.5, 7.5

ARMS = ["H_historical", "E0_static", "E1_cv", "E2_obstacle_aware"]
SHORT = {"H_historical": "H", "E0_static": "E0", "E1_cv": "E1", "E2_obstacle_aware": "E2"}

# Method families, worded after the Method column of
# docs/relation_to_published_systems_2026-09-16.md (bound by test).
METHOD = {
    "NavRL": "PPO + velocity-obstacle shield",
    "Elastic Tracker": "EKF + visibility-aware optimization",
    "Fast-Tracker": "EKF + kinodynamic search",
    "OPEN": "MAPPO + LSTM evader prediction",
    "YOPO": "guidance-learned motion primitives",
    "MOTAR": "frozen Transformer-PPO + arc-clearance filter",
}
FIG6_ROWS = ["NavRL", "Elastic Tracker", "Fast-Tracker", "OPEN", "YOPO", "MOTAR"]
FIG6_AXIS_NAMES = ["Target\ntracking", "Random\nclutter", "Perception\nuncertainty",
                   "Temporal\nestimation", "Safety\nanalysis", "Observation\ncontract"]


def fmt_pp(value, digits=2):
    return (MINUS if value < 0 else "+") + f"{abs(value):.{digits}f}"


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_json(key):
    return json.loads((ROOT / SOURCES[key]).read_text(encoding="utf-8"))


def coverage_rows():
    """Studied-axis matrix, imported from the generator that owns it (no second copy)."""
    sys.path.insert(0, str(ROOT / "tools"))
    import build_quantitative_positioning_figure as positioning  # noqa: E402
    return positioning.AXES, positioning.COVERAGE


# ---------------------------------------------------------------- source values
def collect_values():
    """Every number a figure draws, read from its canonical source."""
    reg = load_json("registry")
    tm = load_json("target_motion")
    vis = load_json("visibility")
    e3s = load_json("e3s")
    p8 = load_json("p8")
    fit = load_json("p9_fit")
    p10 = load_json("p10")
    safety = load_json("safety")
    d8b = load_json("d8b")
    ledger = load_json("ledger")
    status = load_json("status_manifest")

    seeds = sorted({k.split("|")[1] for k in tm["arm_seed_means"]})
    target_motion = {
        "seeds": seeds,
        "per_seed_capture_pct": {SHORT[a]: [100 * tm["arm_seed_means"][f"{a}|{s}"]["capture_rate"]
                                            for s in seeds] for a in ARMS},
        "arm_mean_capture_pct": {SHORT[a]: 100 * tm["arm_means"][a]["capture_rate"] for a in ARMS},
        "contrasts_vs_H": {},
        "episodes": tm["raw_source"]["raw_episodes"],
        "verdict": tm["retraining_verdict"]["verdict"],
        "mechanism_claim_status": tm["mechanism_summary"]["claim_status"],
        "timeouts": {},
    }
    for row in tm["contrasts_vs_H"]:
        if row["metric"] == "capture_rate":
            seed_pp = [100 * row["seed_diffs"][s] for s in seeds]
            target_motion["contrasts_vs_H"][SHORT[row["arm"]]] = {
                "mean_pp": 100 * row["mean_diff"], "bca95_pp": [100 * x for x in row["bca95"]],
                "seed_pp": seed_pp, "seed_sign_consistent": len({x > 0 for x in seed_pp}) == 1}
    e2_e0 = next(r for r in tm["contrasts_within_E"] if r["metric"] == "capture_rate"
                 and r["baseline"] == "E0_static" and r["arm"] == "E2_obstacle_aware")
    target_motion["E2_minus_E0_pp"] = 100 * e2_e0["mean_diff"]
    for a in ARMS:
        total = sum(vis["per_arm_outcome"][f"{a}|{o}"]["episodes"] for o in ("capture", "crash", "timeout"))
        row = vis["per_arm_outcome"][f"{a}|timeout"]
        target_motion["timeouts"][SHORT[a]] = {
            "episodes_total": total, "timeouts": row["episodes"], "never_acquired": row["never_acquired"],
            "timeout_pct_of_all": 100 * row["episodes"] / total,
            "never_acquired_pct_of_all": 100 * row["never_acquired"] / total,
            "never_acquired_pct_of_timeouts": 100 * row["never_acquired_rate"]}

    g2 = e3s["primary"]["gates"]["G2_central"]
    e3s_vals = {"median_of_block_medians_pct": 100 * g2["median_of_block_medians"],
                "gate_pct": 100 * g2["threshold"], "frames": e3s["data_quality"]["included_frames"],
                "blocks": e3s["primary"]["blocks"], "status": e3s["status"], "per_block": []}
    for name, row in sorted(e3s["primary"]["per_block"].items(), key=lambda kv: int(kv[0])):
        lo, hi = row["range_span_m"]
        e3s_vals["per_block"].append({"block": int(name), "median_abs_rel_pct": 100 * row["median_abs_rel"],
                                      "range_mid_m": (lo + hi) / 2, "frames": row["frames"]})

    p8_bins = [{"label": f"{b['lower_inclusive_px']}–{b['upper_exclusive_px']} px", "frames": b["frames"],
                "p": {k: 100 * v for k, v in b["state_probabilities"].items()}}
               for b in p8["bins"] if b["supported"]]
    fit_points = [[t, g] for b in fit["bins"] for row in b["transition_rows"]
                  for t, g in zip(row["target"], row["generated"])]
    fit_vals = {"passed": fit["passed"], "replicas": fit["replicas"], "points": fit_points,
                "max_abs_error": max(r["max_abs_error"] for b in fit["bins"] for r in b["transition_rows"]),
                "threshold": fit["thresholds"]["transition_max_abs"]}

    seeds_p10 = sorted(p10["per_training_seed"], key=int)
    per = p10["per_training_seed"]
    p10_vals = {
        "seeds": seeds_p10,
        "frozen_cost_pp": per[seeds_p10[0]]["p9_cost_on_source"]["delta_pp"],
        "frozen_cost_ci": per[seeds_p10[0]]["p9_cost_on_source"]["ci95"],
        "frozen_cost_identical": len({round(per[s]["p9_cost_on_source"]["delta_pp"], 9)
                                      for s in seeds_p10}) == 1,
        "readapt_seed_pp": [per[s]["primary_capture"]["delta_pp"] for s in seeds_p10],
        "readapt_mean_pp": p10["R2_seed_level_t"]["mean_pp"],
        "readapt_ci": p10["R2_seed_level_t"]["ci95"],
        "residual_seed_pp": [per[s]["p9_cost_on_adapted"]["delta_pp"] for s in seeds_p10],
        "residual_all_exclude_zero": all(per[s]["p9_cost_on_adapted"]["excludes_zero"] for s in seeds_p10),
        "status": p10["status"],
    }
    # The frozen cost is an episode-level interval over the evaluation seeds of the frozen arms.
    frozen_runs = p10["determinism_of_frozen_arms"][seeds_p10[1]]
    p10_vals["frozen_cost_eval_seeds"] = sorted({int(k.rsplit("_s", 1)[1]) for k in frozen_runs if "_s" in k})
    p9_runs = [r["new"] for k, r in frozen_runs.items() if k.startswith("source_p9_")]
    assert "%d/%d" % (sum(r["captured"] for r in p9_runs), sum(r["episodes"] for r in p9_runs)) == \
        per[seeds_p10[0]]["p9_cost_on_source"]["a"], "frozen-cost episodes do not match the evaluation seeds"

    c = safety["contrasts"]["dwa_arc-riskcap"]
    cells = [{"seed": r["seed"], "density": r["density"], "delta_pp": r["delta"]} for r in c["cell"]]
    safety_vals = {"pooled_pp": c["pooled"]["delta"], "ci": [c["pooled"]["lo"], c["pooled"]["hi"]],
                   "k": c["pooled"]["k"], "cells": cells,
                   "negative_cells": sum(1 for r in cells if r["delta_pp"] < 0),
                   "seed_ci": [c["seed_t"]["lo"], c["seed_t"]["hi"]], "seed_df": c["seed_t"]["df"],
                   "seeds": sorted({r["seed"] for r in c["per_seed"]})}

    d8b_vals = {"verdict": d8b["verdict"], "primary_pp": d8b["primary"]["mean"],
                "checkpoint_sha256": d8b["provenance"]["checkpoint_sha256"],
                "primary_ci": d8b["primary"]["ci95"], "margin_pp": d8b["primary"]["margin_pp"],
                "episodes_per_cell": d8b["design"]["episodes_per_cell"],
                "cells": [{"arm": cell["arm"], "seed": int(cell["cell"].split("_")[0][1:]),
                           "capture_pct": 100 * cell["capture_rate"]} for cell in d8b["cells"]]}

    real = {}
    for work in FIG6_ROWS:
        if work == "MOTAR":
            real[work] = bool(status["real_flight_validated"])
        else:
            real[work] = any(e["work"] == work and e.get("simulation_or_real") in ("real", "both")
                             for e in ledger["entries"])

    keys = ("perception_error_cost_frozen", "p10_policy_readaptation", "safety_filter_geometry",
            "target_motion_generalization", "d8b_mesh_observation", "e3s_size_range")
    registry = {}
    for key in keys:
        e = next(x for x in reg["internal_motar"] if x["id"] == key)
        registry[key] = {"value_pp": e.get("value_pp"), "value_percent": e.get("value_percent"),
                         "ci95_pp": e.get("ci95_pp"), "verdict": e["verdict"], "source_path": e["source_path"]}
    return {"registry": registry, "class_a_count": reg["matched_external"]["class_a_count"],
            "target_motion": target_motion, "e3s": e3s_vals, "p8": p8_bins, "p9_fit": fit_vals,
            "p10": p10_vals, "safety": safety_vals, "d8b": d8b_vals, "real_hardware_reported": real}


# ---------------------------------------------------------------- drawing kit
def setup_matplotlib():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "svg.fonttype": "none", "svg.hashsalt": "motar-paper-figures", "pdf.fonttype": 42,
        "font.family": FONT, "font.size": T_LABEL, "axes.edgecolor": RULE, "axes.linewidth": 0.7,
        "axes.labelcolor": INK2, "axes.labelsize": T_LABEL, "xtick.color": MUTED, "ytick.color": MUTED,
        "xtick.labelsize": T_LABEL, "ytick.labelsize": T_LABEL, "xtick.major.width": 0.7,
        "ytick.major.width": 0.7, "xtick.major.size": 2.5, "ytick.major.size": 2.5,
        "legend.fontsize": T_LABEL,
        "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False,
        "figure.facecolor": WHITE, "axes.facecolor": WHITE, "savefig.facecolor": WHITE,
        "lines.solid_capstyle": "round",
    })
    return plt


def fig_title(fig, x, y, letter, title):
    """Panel label in figure coordinates so every panel title shares one baseline."""
    fig.text(x, y, letter, fontsize=T_LETTER, fontweight="bold", color=INK, ha="left", va="baseline")
    fig.text(x + 0.026, y, title, fontsize=T_TITLE, fontweight="bold", color=INK, ha="left", va="baseline")


def note(ax, text, y=-0.3, x=0.0):
    ax.text(x, y, text, transform=ax.transAxes, fontsize=T_NOTE, color=MUTED, ha="left", va="top",
            linespacing=1.25)


def tag(ax, x, y, text, edge=INK2, size=T_TAG, ha="left", va="center", transform=None):
    kw = {"transform": transform} if transform is not None else {}
    return ax.text(x, y, text, fontsize=size, fontweight="bold", color=INK, ha=ha, va=va,
                   bbox={"boxstyle": "square,pad=0.3", "facecolor": WHITE, "edgecolor": edge,
                         "linewidth": 0.9}, **kw)


def hgrid(ax):
    ax.grid(axis="y", color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)


def vgrid(ax):
    ax.grid(axis="x", color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)


def arrow(ax, p0, p1, color=INK2, lw=1.0, scale=9, zorder=3):
    from matplotlib.patches import FancyArrowPatch
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=scale, color=color,
                                 linewidth=lw, shrinkA=0, shrinkB=0, zorder=zorder))


def box(ax, x, y, w, h, edge=RULE, fill=WHITE, lw=0.8, zorder=1):
    from matplotlib.patches import Rectangle
    patch = Rectangle((x, y), w, h, facecolor=fill, edgecolor=edge, linewidth=lw, zorder=zorder)
    ax.add_patch(patch)
    return patch


def square(ax, x, y, s, color=OBSTACLE, zorder=3):
    from matplotlib.patches import Rectangle
    ax.add_patch(Rectangle((x - s / 2, y - s / 2), s, s, facecolor=color, edgecolor="none", zorder=zorder))


def dot(ax, x, y, r, color, zorder=5):
    from matplotlib.patches import Circle
    ax.add_patch(Circle((x, y), r, facecolor=color, edgecolor=WHITE, linewidth=1.0, zorder=zorder))


def uav(ax, x, y, s=1.0, heading=90.0, color=BLUE, zorder=6):
    """Small isosceles triangle pointing along `heading` degrees (schematic vehicle)."""
    from matplotlib.patches import Polygon
    h = math.radians(heading)
    tip = (x + 1.1 * s * math.cos(h), y + 1.1 * s * math.sin(h))
    left = (x + 0.8 * s * math.cos(h + 2.5), y + 0.8 * s * math.sin(h + 2.5))
    right = (x + 0.8 * s * math.cos(h - 2.5), y + 0.8 * s * math.sin(h - 2.5))
    ax.add_patch(Polygon([tip, left, right], closed=True, facecolor=color, edgecolor=WHITE,
                         linewidth=0.8, zorder=zorder))


def circled(ax, x, y, n, size=T_TAG, zorder=7):
    ax.text(x, y, str(n), fontsize=size, fontweight="bold", color=WHITE, ha="center", va="center",
            zorder=zorder, bbox={"boxstyle": "circle,pad=0.2", "facecolor": INK, "edgecolor": INK,
                                 "linewidth": 0.8})


def canvas(plt, height_in, xlim=100.0, width_in=PAPER_W):
    fig = plt.figure(figsize=(width_in, height_in))
    ax = fig.add_axes([0, 0, 1, 1])
    ylim = xlim * height_in / width_in
    ax.set_xlim(0, xlim)
    ax.set_ylim(0, ylim)
    ax.set_axis_off()
    return fig, ax, ylim


def scatter_field(n, x0, x1, y0, y1, min_gap, seed, keep_clear=()):
    """Deterministic dart throwing for SCHEMATIC obstacle fields only (not a recorded layout)."""
    rng = random.Random(seed)
    pts, tries = [], 0
    while len(pts) < n and tries < 20000:
        tries += 1
        p = (rng.uniform(x0, x1), rng.uniform(y0, y1))
        if any(math.hypot(p[0] - q[0], p[1] - q[1]) < min_gap for q in pts):
            continue
        if any(math.hypot(p[0] - cx, p[1] - cy) < r for cx, cy, r in keep_clear):
            continue
        pts.append(p)
    return pts


# ---------------------------------------------------------------- Figure 1
def figure1(plt, v):
    from matplotlib.patches import Wedge, Rectangle
    fig, ax, H = canvas(plt, 5.35)
    reg = v["registry"]
    tmv = v["target_motion"]["contrasts_vs_H"]

    ax.text(1.5, H - 2.0, "SYSTEM", fontsize=T_LABEL, fontweight="bold", color=MUTED, ha="left", va="center")
    ax.text(11.5, H - 2.0, "schematic of the simulated closed loop · every policy is evaluated with frozen weights",
            fontsize=T_NOTE, color=MUTED, ha="left", va="center")
    top, bottom, w = H - 4.3, H - 27.6, 15.0
    xs = [1.5 + i * 16.4 for i in range(6)]
    names = ["Environment", "Perception", "Temporal state", "Frozen policy", "Safety filter", "Evaluation"]
    subs = ["bar field, target", "camera + LiDAR", "track history", "Transformer-PPO", "arc speed cap",
            "arms × densities"]
    for i, x in enumerate(xs):
        box(ax, x, bottom, w, top - bottom)
        ax.text(x + w / 2, bottom + 5.0, names[i], fontsize=8.5, fontweight="bold", color=INK,
                ha="center", va="center")
        ax.text(x + w / 2, bottom + 2.1, subs[i], fontsize=T_LABEL, color=INK2, ha="center", va="center")
        if i < 5:
            arrow(ax, (x + w + 0.15, bottom + 14.5), (xs[i + 1] - 0.15, bottom + 14.5), lw=0.8, scale=5)
    y0, y1 = bottom + 7.6, top - 1.2          # illustration band inside each stage box
    midy = (y0 + y1) / 2

    # 1 environment (schematic, not a recorded layout)
    ex0, ex1 = xs[0] + 1.0, xs[0] + w - 1.0
    box(ax, ex0, y0, ex1 - ex0, y1 - y0, fill=PANEL, lw=0.5)
    target = (ex1 - 2.3, y1 - 2.2)
    start = (ex0 + 1.8, y0 + 3.4)
    for bx, by in scatter_field(16, ex0 + 0.8, ex1 - 0.8, y0 + 2.6, y1 - 0.8, 1.9, 7,
                                keep_clear=[(start[0], start[1], 2.0), (target[0], target[1], 1.8)]):
        square(ax, bx, by, 0.75)
    trail = [(target[0] - 3.4, target[1] - 2.6), (target[0] - 2.1, target[1] - 1.2), target]
    ax.plot([p[0] for p in trail], [p[1] for p in trail], color=ACCENT, lw=0.9, linestyle=(0, (1.6, 1.4)))
    dot(ax, target[0], target[1], 0.6, ACCENT)
    uav(ax, start[0], start[1], s=1.0, heading=40)
    ax.text(ex1 - 0.4, y0 + 1.1, "schematic", fontsize=T_NOTE, color=MUTED, style="italic", ha="right", va="center")

    # 2 perception
    cx, cy = xs[1] + 4.6, midy - 0.3
    for k in range(18):
        ang = 2 * math.pi * k / 18
        ax.plot([cx, cx + 3.6 * math.cos(ang)], [cy, cy + 3.6 * math.sin(ang)], color="#cdc9c1", lw=0.5, zorder=2)
    ax.add_patch(Wedge((cx, cy), 8.6, -24, 24, facecolor=BLUE_WASH, edgecolor=BLUE, linewidth=0.8, zorder=3))
    uav(ax, cx, cy, s=1.0, heading=0)
    dot(ax, cx + 6.9, cy + 1.0, 0.6, ACCENT)

    # 3 temporal state: three frames of history, then the token row
    tx = xs[2] + 1.6
    for k in range(3):
        fx, fy = tx + k * 1.8, y0 + 4.9 - k * 1.2
        box(ax, fx, fy, 6.2, 5.0, edge=INK2 if k == 2 else RULE, lw=0.7, zorder=3 + k)
    box(ax, tx + 7.4, y0 + 4.2, 1.2, 1.2, edge=ACCENT, lw=0.9, zorder=7)
    for k in range(9):
        box(ax, tx + k * 1.3, y0 + 0.6, 1.0, 1.4, edge=BLUE if k >= 7 else RULE,
            fill=BLUE_WASH if k >= 7 else WHITE, lw=0.6, zorder=3)

    # 4 frozen policy: tokens into an attention actor whose weights do not change
    px = xs[3] + 1.4
    for k in range(6):
        box(ax, px, y0 + 0.6 + k * 1.75, 2.0, 1.2, lw=0.6, zorder=3)
    arrow(ax, (px + 2.4, midy), (px + 4.0, midy), lw=0.8, scale=5)
    box(ax, px + 4.3, y0 + 2.2, 7.6, 7.6, edge=INK2, lw=0.9, zorder=3)
    ax.text(px + 8.1, y0 + 7.4, "actor", fontsize=T_LABEL, color=INK2, ha="center", va="center", zorder=4)
    ax.text(px + 8.1, y0 + 4.6, "frozen", fontsize=T_LABEL, fontweight="bold", color=INK, ha="center",
            va="center", zorder=4)

    # 5 safety filter: the turning path sets the clearance; speed is capped, direction kept
    sx, sy = xs[4] + 2.0, y0 + 1.0
    theta = [math.radians(a) for a in range(0, 61, 2)]
    rad = 7.6
    ax.plot([sx + rad * (1 - math.cos(t)) for t in theta], [sy + rad * math.sin(t) for t in theta],
            color=BLUE, lw=1.0, linestyle=(0, (2.6, 1.8)))
    square(ax, sx + 4.2, sy + 7.6, 1.1)
    uav(ax, sx, sy, s=0.95, heading=90)
    bx0 = xs[4] + w - 4.6
    for k, (height, color) in enumerate([(9.0, LIGHT), (4.6, BLUE)]):
        ax.add_patch(Rectangle((bx0 + k * 1.9, y0 + 1.0), 1.3, height, facecolor=color, edgecolor="none",
                               zorder=3))

    # 6 evaluation grid: four arms (rows) by four densities (columns)
    gx, gy = xs[5] + 4.0, y0 + 1.2
    for r, arm in enumerate(["H", "E0", "E1", "E2"]):
        ax.text(gx - 0.7, gy + 8.0 - r * 2.35, arm, fontsize=7, color=INK2, ha="right", va="center")
        for c in range(4):
            box(ax, gx + c * 2.35, gy + 7.2 - r * 2.35, 2.0, 1.6, fill=BLUE_WASH, lw=0.5)

    # where each measurement intervenes (perception carries two: measured error 1, rendering 5)
    for n, (x, dx) in {4: (xs[0], 0), 1: (xs[1], 0), 5: (xs[1], -2.6), 2: (xs[3], 0), 3: (xs[4], 0)}.items():
        circled(ax, x + w - 1.5 + dx, top - 1.7, n)
    # evidence chain: five separate experiments, one row each
    ey = bottom - 4.6
    ax.text(1.5, ey, "EVIDENCE", fontsize=T_LABEL, fontweight="bold", color=MUTED, ha="left", va="center")
    ax.text(13.0, ey, "five separate experiments, not a causal chain · each value bound to a result record · "
            "pp = percentage points", fontsize=T_NOTE, color=MUTED, ha="left", va="center")
    rows = [
        ("Measured error", fmt_pp(reg["perception_error_cost_frozen"]["value_pp"]) + " pp",
         "capture of frozen policy F under injected measured error",
         f"episode-level 95% CI · {len(v['p10']['frozen_cost_eval_seeds'])} evaluation seeds", None),
        ("Readaptation", fmt_pp(reg["p10_policy_readaptation"]["value_pp"]) + " pp",
         "retrained − frozen, under the same error; 95% CI crosses zero",
         f"{len(v['p10']['seeds'])} training seeds", reg["p10_policy_readaptation"]["verdict"]),
        ("Safety geometry", fmt_pp(reg["safety_filter_geometry"]["value_pp"]) + " pp",
         "crash, arc-clearance − riskcap, frozen policy F",
         f"{v['safety']['negative_cells']}/{v['safety']['k']} seed × density cells lower · "
         f"{len(v['safety']['seeds'])} evaluation seeds", None),
        ("Target motion", f"E0 {fmt_pp(tmv['E0']['mean_pp'])} · E2 {fmt_pp(tmv['E2']['mean_pp'])}",
         "pp capture vs training target H, frozen policy F",
         f"not a monotonic ladder · {len(v['target_motion']['seeds'])} evaluation seeds", None),
        ("Observation contract", fmt_pp(reg["d8b_mesh_observation"]["value_pp"]) + " pp",
         "capture, mesh-shaded vs analytic target; causality NOT TESTED",
         "policy R (not F) · 3 paired evaluation seeds", reg["d8b_mesh_observation"]["verdict"]),
    ]
    row_h, first = 7.0, ey - 2.6
    ax.plot([1.5, 98.5], [first, first], color=RULE, lw=0.7)
    for k, (label, value, line1, line2, verdict) in enumerate(rows):
        yc = first - (k + 0.5) * row_h
        circled(ax, 3.0, yc, k + 1)
        ax.text(5.3, yc, label, fontsize=8.5, fontweight="bold", color=INK, ha="left", va="center")
        ax.text(26.0, yc + (1.3 if verdict else 0), value, fontsize=11, fontweight="bold", color=INK, ha="left",
                va="center")
        if verdict:
            tag(ax, 26.4, yc - 1.75, verdict, edge=ACCENT if verdict == "MATERIAL_LOSS" else INK2)
        ax.text(50.0, yc + 1.35, line1, fontsize=T_LABEL, color=INK2, ha="left", va="center")
        ax.text(50.0, yc - 1.45, line2, fontsize=T_LABEL, color=MUTED, ha="left", va="center")
        ax.plot([1.5, 98.5], [yc - row_h / 2, yc - row_h / 2], color=GRID if k < 4 else RULE, lw=0.6 if k < 4 else 0.7)
    ax.text(1.5, 3.4, "Simulation only · no real-flight, sim-to-real or state-of-the-art claim.", fontsize=T_NOTE,
            color=MUTED, ha="left", va="center")
    ax.text(1.5, 1.3, "Circled numbers mark where each experiment intervenes in the loop.", fontsize=T_NOTE,
            color=MUTED, ha="left", va="center")
    title = "MOTAR overview: simulated closed loop and measured evidence chain"
    desc = ("Schematic of the simulated closed loop: environment with a random bar field and a moving target, "
            "camera and LiDAR perception with measured detector error, temporal state, frozen Transformer-PPO "
            "policy, arc-clearance speed cap and velocity controller, evaluation over target-motion arms "
            "H/E0/E1/E2 and densities 70-205 bars. Five separate experiments, not a causal chain: measured "
            "error changes the capture of frozen policy F "
            f"by {rows[0][1]}; readaptation {rows[1][1]}, INCONCLUSIVE; safety geometry {rows[2][1]} crash; "
            f"target motion {rows[3][1]} pp versus H; observation contract {rows[4][1]}, MATERIAL_LOSS, "
            "causality NOT TESTED, measured on a different frozen checkpoint, policy R. Simulation only.")
    return fig, title, desc


# ---------------------------------------------------------------- Figure 2
def figure2(plt, v):
    fig = plt.figure(figsize=(PAPER_W, 6.6))
    gs = fig.add_gridspec(2, 2, left=0.095, right=0.985, top=0.905, bottom=0.145, wspace=0.62, hspace=0.86,
                          width_ratios=[1, 1.05])
    e3s, p8, fit, p10 = v["e3s"], v["p8"], v["p9_fit"], v["p10"]
    top_y, low_y = 0.955, 0.47

    ax = fig.add_subplot(gs[0, 0])
    ax.axhline(e3s["gate_pct"], color=RULE, lw=0.9)
    ax.text(109, e3s["gate_pct"] + 0.7, f"preregistered gate {e3s['gate_pct']:.0f}%", fontsize=T_NOTE,
            color=MUTED, ha="right", va="bottom")
    ax.axhline(e3s["median_of_block_medians_pct"], color=BLUE, lw=1.2)
    ax.scatter([b["range_mid_m"] for b in e3s["per_block"]], [b["median_abs_rel_pct"] for b in e3s["per_block"]],
               s=38, color=BLUE, edgecolor=WHITE, linewidth=1.2, zorder=4)
    ax.text(31.5, e3s["median_of_block_medians_pct"] + 1.1,
            f"median of {e3s['blocks']} blocks = {e3s['median_of_block_medians_pct']:.1f}%", fontsize=8.5,
            fontweight="bold", color=INK, ha="left", va="bottom")
    ax.set_xlim(30, 110)
    ax.set_ylim(0, 30)
    ax.set_xlabel("block mean range (m)")
    ax.set_ylabel("median abs. rel. error (%)")
    hgrid(ax)
    fig_title(fig, 0.02, top_y, "a", "Measured range proxy on real footage")
    note(ax, f"ETH ds5, one flight · apparent-size proxy\nunit: {e3s['blocks']} blocks of 15 s "
             f"({e3s['frames']:,} frames, not independent)")

    ax = fig.add_subplot(gs[0, 1])
    colors = {"HIT": BLUE, "FALSE_LOCK": ACCENT, "NO_LOCK": TEAL}
    names = {"HIT": "hit", "FALSE_LOCK": "false lock", "NO_LOCK": "no lock"}
    for r, b in enumerate(p8):
        y, left = len(p8) - 1 - r, 0.0
        for state in ("HIT", "FALSE_LOCK", "NO_LOCK"):
            width = b["p"][state]
            ax.barh(y, max(width - 0.8, 0.1), left=left + 0.4, height=0.5, color=colors[state], linewidth=0)
            if width >= 9:
                ax.text(left + width / 2, y, f"{width:.0f}", fontsize=T_LABEL, color=WHITE, ha="center", va="center",
                        fontweight="bold")
            left += width
    ax.set_yticks(range(len(p8)))
    ax.set_yticklabels([f"{b['label']}\nn = {b['frames']}" for b in reversed(p8)], fontsize=T_LABEL)
    ax.set_xlim(0, 100)
    ax.set_ylim(-0.55, 3.0)
    ax.set_xlabel("share of frames (%)")
    ax.spines["left"].set_visible(False)
    ax.tick_params(axis="y", length=0)
    for state in ("HIT", "FALSE_LOCK", "NO_LOCK"):
        ax.scatter([], [], marker="s", s=42, color=colors[state], label=names[state])
    ax.legend(loc="upper left", bbox_to_anchor=(0.0, 1.04), ncol=3, fontsize=T_LABEL, handletextpad=0.3,
              columnspacing=1.0, borderaxespad=0.0)
    fig_title(fig, 0.54, top_y, "b", "Error model by target size (P8)")
    note(ax, "NPS-Drones validation split\nframes with exactly one target")

    ax = fig.add_subplot(gs[1, 0])
    pts = fit["points"]
    ax.plot([0, 1], [0, 1], color=RULE, lw=1.0)
    ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=26, color=BLUE, edgecolor=WHITE, linewidth=1.0, zorder=4)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_xlabel("measured transition probability")
    ax.set_ylabel("injected in simulation")
    hgrid(ax)
    vgrid(ax)
    ax.text(0.04, 0.93, f"{len(pts)} state transitions", transform=ax.transAxes, fontsize=T_LABEL, color=INK, va="top")
    ax.text(0.04, 0.83, f"max |error| {fit['max_abs_error']:.3f} (gate ≤ {fit['threshold']:.2f})",
            transform=ax.transAxes, fontsize=T_LABEL, color=INK2, va="top")
    tag(ax, 0.04, 0.67, "GOODNESS-OF-FIT PASS" if fit["passed"] else "GOODNESS-OF-FIT FAIL",
        transform=ax.transAxes)
    fig_title(fig, 0.02, low_y, "c", "Injection reproduces the model (P9)")
    note(ax, f"hit / false-lock / no-lock transitions\n{fit['replicas']} replicas per size bin")

    ax = fig.add_subplot(gs[1, 1])
    ax.axvline(0, color=INK2, lw=0.9)
    lo, hi = p10["frozen_cost_ci"]
    ax.plot([lo, hi], [2, 2], color=INK, lw=1.8, solid_capstyle="butt")
    ax.scatter([p10["frozen_cost_pp"]], [2], s=62, color=INK, zorder=4, edgecolor=WHITE, linewidth=1.2)
    ax.text(p10["frozen_cost_pp"], 2.3, fmt_pp(p10["frozen_cost_pp"]) + " pp", fontsize=10.5, fontweight="bold",
            color=INK, ha="center", va="bottom")
    lo, hi = p10["readapt_ci"]
    ax.plot([lo, hi], [1, 1], color=MUTED, lw=1.8, solid_capstyle="butt")
    ax.scatter(p10["readapt_seed_pp"], [1] * len(p10["readapt_seed_pp"]), s=26, facecolor=WHITE,
               edgecolor=BLUE, linewidth=1.2, zorder=4)
    ax.scatter([p10["readapt_mean_pp"]], [1], s=52, color=BLUE, zorder=5, edgecolor=WHITE, linewidth=1.2)
    ax.text(hi + 0.35, 1.2, fmt_pp(p10["readapt_mean_pp"]) + " pp", fontsize=9, fontweight="bold",
            color=INK, ha="left", va="center")
    tag(ax, hi + 0.35, 0.72, p10["status"])
    ax.scatter(p10["residual_seed_pp"], [0] * len(p10["residual_seed_pp"]), s=26, facecolor=WHITE,
               edgecolor=INK2, linewidth=1.2, zorder=4)
    if p10["residual_all_exclude_zero"]:
        ax.text(0.4, 0, "each seed's 95% CI\nexcludes zero", fontsize=T_NOTE, color=MUTED, ha="left", va="center")
    ax.set_yticks([2, 1, 0])
    ax.set_yticklabels(["measured error\non policy F", "readaptation\n(adapted − frozen)",
                        "residual cost\nafter readaptation"], fontsize=T_LABEL)
    ax.tick_params(axis="y", length=0)
    ax.set_ylim(-0.55, 2.85)
    ax.set_xlim(-7.5, 9.5)
    ax.set_xticks([-7.5, -5, -2.5, 0, 2.5, 5, 7.5])
    ax.set_xlabel("capture difference (pp)")
    ax.spines["left"].set_visible(False)
    vgrid(ax)
    fig_title(fig, 0.54, low_y, "d", "Effect on frozen policy F (P10)")
    note(ax, f"205 bars · top: episode-level 95% CI, {len(p10['frozen_cost_eval_seeds'])} evaluation seeds pooled\n"
             f"middle: seed-level 95% CI; open dots = training seeds (n = {len(p10['seeds'])})", x=-0.55)
    title = "Perception to policy: measured error, error model, injection and frozen-policy effect"
    desc = (f"(a) ETH ds5 E3-S: median absolute relative range error {e3s['median_of_block_medians_pct']:.1f}% "
            f"over {e3s['blocks']} blocks and {e3s['frames']} frames, below the {e3s['gate_pct']:.0f}% gate. "
            "(b) P8 hit / false-lock / no-lock shares by target size. (c) P9 injector transition probabilities "
            f"against measured ones, max error {fit['max_abs_error']:.3f}. (d) Measured error changes frozen policy F "
            f"capture by {fmt_pp(p10['frozen_cost_pp'])} pp; readaptation {fmt_pp(p10['readapt_mean_pp'])} pp, "
            f"95% CI [{fmt_pp(p10['readapt_ci'][0])}, {fmt_pp(p10['readapt_ci'][1])}], {p10['status']}; a residual "
            "cost remains for every seed.")
    return fig, title, desc


# ---------------------------------------------------------------- Figure 3
def figure3(plt, v):
    from matplotlib.patches import Polygon, Rectangle
    s = v["safety"]
    fig_h = 4.1
    fig = plt.figure(figsize=(PAPER_W, fig_h))
    # Equal aspect: 49 x 32.4 schematic units drawn half the page wide.
    width_in = 0.5 * PAPER_W
    ax = fig.add_axes([0.0, 0.25, 0.5, width_in * 32.4 / 49.0 / fig_h])
    ax.set_xlim(1.5, 50.5)
    ax.set_ylim(1.4, 33.8)
    ax.set_axis_off()
    fig_title(fig, 0.02, 0.935, "a", "Where clearance is measured (schematic)")
    # Obstacle offsets from the vehicle; the same field is drawn in both geometries.
    obstacles = [(-4.2, 11.8), (4.2, 12.0), (-0.6, 19.5), (-5.6, 20.6), (6.8, 21.6), (-2.8, 6.4)]
    half, rad = 1.5, 19.0          # corridor half-width and turning radius (schematic units)
    layouts = [("straight corridor\n(riskcap)", 3.0, "line"), ("turning arc\n(implemented arc filter)", 27.8, "arc")]
    for title, x0, kind in layouts:
        panel = box(ax, x0, 6.2, 21.2, 27.4, fill=PANEL, lw=0.6)
        ux, uy = x0 + 10.6, 8.6
        for ox, oy in obstacles:
            square(ax, ux + ox, uy + oy, 1.6)
        if kind == "line":
            band = Polygon([(ux - half, uy), (ux + half, uy), (ux + half, uy + 24.5), (ux - half, uy + 24.5)],
                           closed=True, facecolor=BLUE_WASH, edgecolor=BLUE, linewidth=0.9, zorder=2)
            inside = [(oy, (ox, oy)) for ox, oy in obstacles if abs(ox) <= half and oy > 0]
        else:
            th = [math.radians(a) for a in range(180, 96, -2)]
            outer = [(ux + rad + (rad + half) * math.cos(t), uy + (rad + half) * math.sin(t)) for t in th]
            inner = [(ux + rad + (rad - half) * math.cos(t), uy + (rad - half) * math.sin(t)) for t in reversed(th)]
            band = Polygon(outer + inner, closed=True, facecolor=BLUE_WASH, edgecolor=BLUE, linewidth=0.9, zorder=2)
            inside = []
            for ox, oy in obstacles:
                r = math.hypot(ox - rad, oy)
                angle = math.degrees(math.atan2(oy, ox - rad))
                if abs(r - rad) <= half and 96 <= angle <= 180:
                    inside.append((math.radians(180 - angle) * rad, (ox, oy)))
        ax.add_patch(band)
        band.set_clip_path(panel)
        hit = min(inside)[1]       # first obstacle along the measured geometry
        ax.add_patch(Rectangle((ux + hit[0] - 1.05, uy + hit[1] - 1.05), 2.1, 2.1, facecolor="none",
                               edgecolor=ACCENT, linewidth=1.3, zorder=5))
        uav(ax, ux, uy - 0.3, s=1.2, heading=90)
        ax.text(x0 + 10.6, 5.6, title, fontsize=T_LABEL, color=INK, ha="center", va="top", linespacing=1.15)
    fig.text(0.035, 0.155, "Accent outline: first obstacle inside each geometry.\n"
             "Both filters cap speed and keep the commanded direction;\nneither is a collision guarantee.",
             fontsize=T_NOTE, color=MUTED, ha="left", va="top", linespacing=1.3)

    ax = fig.add_axes([0.6, 0.35, 0.385, 0.53])
    dens = sorted({c["density"] for c in s["cells"]})
    seeds = sorted({c["seed"] for c in s["cells"]})
    markers = dict(zip(seeds, ["o", "s", "^"]))
    offsets = dict(zip(seeds, [-6, 0, 6]))
    lo, hi = s["ci"]
    ax.axhspan(lo, hi, color=BLUE_WASH, zorder=0, linewidth=0)
    ax.axhline(s["pooled_pp"], color=BLUE, lw=1.4, zorder=1)
    ax.axhline(0, color=INK2, lw=0.9, zorder=1)
    for seed in seeds:
        pts = [c for c in s["cells"] if c["seed"] == seed]
        ax.scatter([c["density"] + offsets[seed] for c in pts], [c["delta_pp"] for c in pts], s=22,
                   marker=markers[seed], facecolor=WHITE, edgecolor=INK2, linewidth=1.1, zorder=4,
                   label=f"seed {seed}")
    ax.set_xticks(dens)
    ax.set_xlim(min(dens) - 22, max(dens) + 22)
    ax.set_ylim(-4.3, 2.2)
    ax.set_yticks([-4, -3, -2, -1, 0, 1, 2])
    ax.set_xlabel("obstacle density (bars per arena)")
    ax.set_ylabel("crash difference (pp)")
    hgrid(ax)
    ax.legend(loc="lower right", fontsize=T_NOTE, ncol=3, handletextpad=0.1, columnspacing=0.6, borderaxespad=0.1)
    ax.text(0.02, 0.9, f"{fmt_pp(s['pooled_pp'], 4)} pp", transform=ax.transAxes, fontsize=11,
            fontweight="bold", color=INK, ha="left", va="center")
    ax.text(0.02, 0.76, "arc − riskcap, pooled over cells", transform=ax.transAxes, fontsize=T_LABEL, color=INK2,
            ha="left", va="center")
    note(ax, f"95% CI [{fmt_pp(lo, 3)}, {fmt_pp(hi, 3)}] pooled over {s['k']} seed × density cells\n"
             f"seed-level 95% CI [{fmt_pp(s['seed_ci'][0], 2)}, {fmt_pp(s['seed_ci'][1], 2)}], "
             f"n = {len(s['seeds'])} evaluation seeds\n"
             f"{s['negative_cells']}/{s['k']} cells below zero; cells share three seeds, not replicates",
         y=-0.36, x=-0.2)
    fig_title(fig, 0.53, 0.935, "b", "Configured contrast on frozen policy F")
    title = "Safety-filter geometry: straight corridor versus implemented arc-clearance filter"
    desc = ("(a) Schematic of where each filter measures clearance before capping speed. (b) Crash-rate "
            f"difference arc-clearance minus riskcap for {s['k']} seed-density cells: pooled "
            f"{fmt_pp(s['pooled_pp'], 4)} pp, 95% CI [{fmt_pp(lo, 4)}, {fmt_pp(hi, 4)}], "
            f"seed-level 95% CI [{fmt_pp(s['seed_ci'][0], 2)}, {fmt_pp(s['seed_ci'][1], 2)}] over "
            f"{len(s['seeds'])} evaluation seeds; {s['negative_cells']}/{s['k']} cells below zero, and the cells "
            "are not independent replicates. Frozen policy F. Configured contrast in simulation; no "
            "collision-safety guarantee.")
    return fig, title, desc


# ---------------------------------------------------------------- Figure 4
def figure4(plt, v):
    tm = v["target_motion"]
    fig = plt.figure(figsize=(PAPER_W, 6.6))
    gs = fig.add_gridspec(2, 2, left=0.095, right=0.985, top=0.905, bottom=0.145, wspace=0.34, hspace=0.86)
    top_y, low_y = 0.955, 0.47

    ax = fig.add_subplot(gs[0, 0])
    ax.set_xlim(0, 40)
    ax.set_ylim(-0.6, 22)
    ax.set_axis_off()
    fig_title(fig, 0.02, top_y, "a", "Target-motion arms (schematic)")
    specs = [("H", "training law"), ("E0", "static"), ("E1", "constant\nvelocity"), ("E2", "obstacle-\naware")]
    for k, (arm, name) in enumerate(specs):
        x0 = k * 10.0
        box(ax, x0 + 0.4, 6.6, 9.2, 15.0, fill=PANEL, lw=0.5)
        ax.text(x0 + 5.0, 5.0, arm, fontsize=9, fontweight="bold", color=INK, ha="center", va="center")
        ax.text(x0 + 5.0, 3.6, name, fontsize=T_LABEL, color=INK2, ha="center", va="top", linespacing=1.1)
    for bx, by in [(4.4, 15.9), (7.1, 10.2), (14.1, 16.0), (17.3, 10.6), (24.1, 16.6), (26.6, 11.0),
                   (33.2, 14.9), (35.6, 11.1)]:
        square(ax, bx, by, 1.0)
    h = [(2.2, 8.6), (3.5, 11.0), (2.5, 13.4), (4.5, 18.9), (6.9, 17.2), (8.4, 19.6)]
    ax.plot([p[0] for p in h], [p[1] for p in h], color=REF, lw=1.3)
    dot(ax, *h[-1], 0.55, ACCENT)
    dot(ax, 15.2, 13.2, 0.55, ACCENT)
    ax.plot([21.8, 28.6], [8.5, 19.8], color=BLUE, lw=1.3)
    dot(ax, 28.6, 19.8, 0.55, ACCENT)
    e2 = [(31.8, 8.5), (33.9, 10.1), (34.6, 12.6), (33.9, 16.4), (35.5, 18.3), (37.9, 19.7)]
    ax.plot([p[0] for p in e2], [p[1] for p in e2], color=BLUE, lw=1.3)
    dot(ax, *e2[-1], 0.55, ACCENT)

    arms = ["H", "E0", "E1", "E2"]
    ax = fig.add_subplot(gs[0, 1])
    for i, arm in enumerate(arms):
        color = REF if arm == "H" else BLUE
        mean = tm["arm_mean_capture_pct"][arm]
        ax.plot([i - 0.22, i + 0.22], [mean, mean], color=color, lw=2.2, solid_capstyle="butt")
        ax.scatter([i - 0.1, i, i + 0.1], tm["per_seed_capture_pct"][arm], s=24, facecolor=WHITE, edgecolor=color,
                   linewidth=1.2, zorder=4)
        ax.text(i, mean + 0.6, f"{mean:.1f}", fontsize=8.5, fontweight="bold", color=INK, ha="center", va="bottom")
    ax.set_xticks(range(4))
    ax.set_xticklabels(["H\ntraining", "E0\nstatic", "E1\nconst. vel.", "E2\nobst.-aware"], fontsize=T_LABEL)
    ax.set_ylim(82, 91)
    ax.set_xlim(-0.6, 3.6)
    ax.set_ylabel("close-approach rate (%)")
    hgrid(ax)
    fig_title(fig, 0.535, top_y, "b", "Capture by arm")
    note(ax, f"bar = mean, dots = seeds · {tm['episodes']:,} episodes\n70–205 bars · y-axis starts at 82%", y=-0.3)

    ax = fig.add_subplot(gs[1, 0])
    ax.axvline(0, color=INK2, lw=0.9)
    for r, arm in enumerate(["E2", "E1", "E0"]):
        c = tm["contrasts_vs_H"][arm]
        lo, hi = c["bca95_pp"]
        ax.plot([lo, hi], [r, r], color=BLUE, lw=2.2, solid_capstyle="butt")
        ax.scatter(c["seed_pp"], [r] * 3, s=22, facecolor=WHITE, edgecolor=BLUE, linewidth=1.1, zorder=4)
        ax.scatter([c["mean_pp"]], [r], s=50, color=BLUE, edgecolor=WHITE, linewidth=1.2, zorder=5)
        ax.text(c["mean_pp"], r + 0.3, fmt_pp(c["mean_pp"]) + " pp", fontsize=9, fontweight="bold",
                color=INK, ha="center", va="bottom")
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(["E2 − H", "E1 − H", "E0 − H"])
    ax.set_ylim(-0.6, 2.85)
    ax.set_xlim(-5, 3)
    ax.set_xlabel("capture difference vs H (pp)")
    ax.spines["left"].set_visible(False)
    ax.tick_params(axis="y", length=0)
    vgrid(ax)
    fig_title(fig, 0.02, low_y, "c", "Seed-paired difference vs H")
    note(ax, f"unit: evaluation seed (n = {len(tm['seeds'])}) · bar = BCa 95% CI · dots = seeds\n"
             "all six arm-level capture contrasts are 3/3 same sign")

    ax = fig.add_subplot(gs[1, 1])
    for i, arm in enumerate(arms):
        row = tm["timeouts"][arm]
        never, total = row["never_acquired_pct_of_all"], row["timeout_pct_of_all"]
        ax.bar(i, never, width=0.46, color=ACCENT if arm == "E0" else INK2, linewidth=0)
        ax.bar(i, total - never, bottom=never, width=0.46, color=LIGHT, linewidth=0)
        ax.text(i, total + 0.18, f"{total:.1f}%", fontsize=T_LABEL, color=INK, ha="center", va="bottom")
    e0 = tm["timeouts"]["E0"]
    ax.text(-0.22, 9.9, f"{e0['never_acquired_pct_of_timeouts']:.2f}% of E0 timeouts\nnever acquired the target",
            fontsize=T_LABEL, fontweight="bold", color=INK, ha="left", va="center")
    ax.set_xticks(range(4))
    ax.set_xticklabels(arms)
    ax.set_ylim(0, 13.2)
    ax.set_yticks([0, 2, 4, 6, 8, 10, 12])
    ax.set_ylabel("timeout episodes (% of all)")
    hgrid(ax)
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(color=INK2, label="target never acquired"), Patch(color=LIGHT, label="acquired, then lost")],
              loc="upper right", fontsize=T_NOTE, handlelength=1.0, borderaxespad=0.1)
    fig_title(fig, 0.535, low_y, "d", "Timeouts and first acquisition")
    note(ax, "consistent with a first-acquisition / visibility\nmechanism (association only)")
    c = tm["contrasts_vs_H"]
    means = tm["arm_mean_capture_pct"]
    title = "Target motion: close-approach rate of frozen policy F under four target-motion arms"
    desc = (f"(a) Schematic of arms H (training law), E0 static, E1 constant velocity, E2 obstacle-aware. "
            f"(b) Capture by arm: E0 {means['E0']:.2f}% < E1 {means['E1']:.2f}% < H {means['H']:.2f}% "
            f"< E2 {means['E2']:.2f}%. (c) Differences vs H: E0 {fmt_pp(c['E0']['mean_pp'])} pp, "
            f"E1 {fmt_pp(c['E1']['mean_pp'])} pp, E2 {fmt_pp(c['E2']['mean_pp'])} pp. (d) Timeouts: "
            f"{e0['timeout_pct_of_all']:.1f}% of E0 episodes, {e0['never_acquired_pct_of_timeouts']:.2f}% of them "
            "never acquired the target; consistent with a first-acquisition / visibility mechanism, association "
            "only and not a causal explanation.")
    return fig, title, desc


# ---------------------------------------------------------------- Figure 5
def figure5(plt, v):
    d = v["d8b"]
    fig_h = 4.0
    fig = plt.figure(figsize=(PAPER_W, fig_h))
    width_in = 0.54 * PAPER_W
    ax = fig.add_axes([0.0, 0.14, 0.54, width_in * 36.0 / 52.0 / fig_h])
    ax.set_xlim(0, 52)
    ax.set_ylim(0, 36.0)
    ax.set_axis_off()
    fig_title(fig, 0.02, 0.935, "a", "One variable changes: the observation contract")
    arms = [("analytic_flat", "analytic target\nflat"), ("mesh_flat", "mesh target\nflat"),
            ("mesh_shaded", "mesh target\nshaded")]
    for k, (_, label) in enumerate(arms):
        y = 27.0 - k * 8.4
        box(ax, 2.0, y, 15.6, 5.6, edge=ACCENT if k == 2 else RULE, lw=1.0 if k == 2 else 0.7)
        ax.text(9.8, y + 2.8, label, fontsize=T_LABEL, color=INK, ha="center", va="center", linespacing=1.15)
        arrow(ax, (17.9, y + 2.8), (23.1, 19.0 + (1 - k) * 1.8), lw=0.9, scale=7)
    box(ax, 23.4, 13.2, 15.4, 11.8, edge=INK2, fill=PANEL, lw=0.9)
    for yy, text, bold in [(21.9, "same detector", True), (19.8, "+ frozen policy R", True),
                           (17.0, "same task, seeds,", False), (15.1, "arena and 70 bars", False)]:
        ax.text(31.1, yy, text, fontsize=8.5 if bold else T_LABEL, fontweight="bold" if bold else "normal",
                color=INK if bold else INK2, ha="center", va="center")
    arrow(ax, (39.1, 19.1), (43.4, 19.1), lw=0.9, scale=7)
    ax.text(47.2, 19.1, "capture", fontsize=T_LABEL, color=INK, ha="center", va="center")
    ax.text(26.0, 7.2, "Arrows show data flow in the experiment, not a causal mechanism.", fontsize=T_NOTE,
            color=INK2, ha="center", va="center")
    ax.text(26.0, 5.0, "No renderer property was isolated as the cause of the loss.", fontsize=T_NOTE,
            color=INK2, ha="center", va="center")
    tag(ax, 26.0, 1.4, "CAUSALITY NOT TESTED", edge=ACCENT, size=8.5, ha="center")

    ax = fig.add_axes([0.65, 0.35, 0.335, 0.53])
    order = [a for a, _ in arms]
    for i, arm in enumerate(order):
        vals = [c["capture_pct"] for c in d["cells"] if c["arm"] == arm]
        mean = sum(vals) / len(vals)
        color = ACCENT if arm == "mesh_shaded" else (REF if arm == "analytic_flat" else BLUE)
        ax.plot([i - 0.22, i + 0.22], [mean, mean], color=color, lw=2.2, solid_capstyle="butt")
        ax.scatter([i - 0.1, i, i + 0.1], vals, s=18, facecolor=WHITE, edgecolor=color, linewidth=1.1, zorder=4)
        ax.text(i, mean + 3.2, f"{mean:.1f}%", fontsize=T_LABEL, color=INK, ha="center", va="bottom")
    ax.set_xticks(range(3))
    ax.set_xticklabels(["analytic\nflat", "mesh\nflat", "mesh\nshaded"], fontsize=T_LABEL)
    ax.set_ylim(0, 100)
    ax.set_xlim(-0.6, 2.6)
    ax.set_ylabel("capture rate (%)")
    hgrid(ax)
    ax.text(0.98, 0.93, f"{fmt_pp(d['primary_pp'], 3)} pp", transform=ax.transAxes, fontsize=11,
            fontweight="bold", color=INK, ha="right", va="center")
    ax.text(0.98, 0.81, "mesh shaded − analytic", transform=ax.transAxes, fontsize=T_LABEL, color=INK2,
            ha="right", va="center")
    tag(ax, 0.98, 0.45, d["verdict"], edge=ACCENT, ha="right", transform=ax.transAxes)
    fig_title(fig, 0.575, 0.935, "b", "Sensitivity of frozen policy R (D8b)")
    note(ax, f"95% CI [{fmt_pp(d['primary_ci'][0], 3)}, {fmt_pp(d['primary_ci'][1], 3)}]\n"
             f"unit: paired evaluation seed (n = 3) · {d['episodes_per_cell']:,} episodes per cell\n"
             "policy R = frozen ref5in D1 checkpoint (ep1900),\n70 bars; not policy F", y=-0.27, x=-0.3)
    title = "Observation contract: frozen policy R under three target-rendering treatments"
    desc = (f"(a) Only the observation contract changes; detector, frozen policy R, task and seeds are fixed. "
            f"(b) Capture by treatment, 3 seeds x {d['episodes_per_cell']} episodes: mesh shaded minus analytic "
            f"{fmt_pp(d['primary_pp'], 3)} pp, 95% CI [{fmt_pp(d['primary_ci'][0], 3)}, "
            f"{fmt_pp(d['primary_ci'][1], 3)}], verdict {d['verdict']}. CAUSALITY NOT TESTED: no renderer "
            "property was isolated as the cause. Checkpoint: policy R, frozen ref5in D1 ep1900 at 70 bars, not the "
            "ep25000 policy F used by the other figures. Unit of inference: paired evaluation seed.")
    return fig, title, desc


# ---------------------------------------------------------------- Figure 6
def figure6(plt, v):
    _, coverage = coverage_rows()
    fig, ax, H = canvas(plt, 4.3)
    ax.text(1.5, H - 2.3, "Studied axes by system", fontsize=T_TITLE, fontweight="bold", color=INK, ha="left",
            va="center")
    ax.text(1.5, H - 5.0, "● studied or reported on this axis  ·  ○ not reported  ·  a mark is not a performance score",
            fontsize=T_NOTE, color=MUTED, ha="left", va="center")
    col0, colw = 34.6, 9.7
    names = FIG6_AXIS_NAMES + ["Real\nhardware"]
    ytop = H - 12.2
    xs = [col0 + j * colw + (1.2 if j == len(names) - 1 else 0) for j in range(len(names))]
    for x, name in zip(xs, names):
        ax.text(x, ytop + 1.0, name, fontsize=T_LABEL, color=INK2, ha="center", va="bottom", linespacing=1.1)
    ax.text(1.8, ytop + 1.0, "System\nmethod family", fontsize=T_LABEL, color=INK2, ha="left", va="bottom",
            linespacing=1.1)
    row_h = 6.2
    ax.plot([(xs[-2] + xs[-1]) / 2] * 2, [ytop - len(FIG6_ROWS) * row_h + 0.4, ytop + 5.0], color=GRID, lw=0.7)
    ax.plot([1.5, 98.5], [ytop, ytop], color=RULE, lw=0.7)
    for i, work in enumerate(FIG6_ROWS):
        y = ytop - (i + 0.5) * row_h
        if work == "MOTAR":
            box(ax, 1.0, y - row_h / 2 + 0.3, 97.8, row_h - 0.6, edge="none", fill=BLUE_WASH, lw=0)
        ax.text(1.8, y + 1.25, work, fontsize=8.5, fontweight="bold" if work == "MOTAR" else "normal", color=INK,
                ha="left", va="center")
        ax.text(1.8, y - 1.35, METHOD[work], fontsize=T_NOTE, color=MUTED, ha="left", va="center")
        marks = list(coverage[work]) + [1 if v["real_hardware_reported"][work] else 0]
        for j, (x, on) in enumerate(zip(xs, marks)):
            if on:
                ax.scatter([x], [y], s=44, color=BLUE if j < len(marks) - 1 else INK2, edgecolor=WHITE,
                           linewidth=1.0, zorder=4)
            else:
                ax.scatter([x], [y], s=32, facecolor=WHITE, edgecolor="#b4b0a8", linewidth=1.0, zorder=4)
        if i < len(FIG6_ROWS) - 1:
            ax.plot([1.5, 98.5], [y - row_h / 2, y - row_h / 2], color=GRID, lw=0.6)
    ybot = ytop - len(FIG6_ROWS) * row_h
    ax.plot([1.5, 98.5], [ybot, ybot], color=RULE, lw=0.7)
    tag(ax, 1.8, ybot - 3.4, f"Class A matched external comparisons = {v['class_a_count']}", size=8.5)
    ax.text(1.5, ybot - 7.3, "No external number is subtracted from, or ranked against, a MOTAR result.",
            fontsize=T_NOTE, color=MUTED, ha="left", va="center")
    ax.text(1.5, ybot - 9.6, "Real hardware = any flight or onboard-compute entry in the positioning ledger; "
            "MOTAR is simulation-only.", fontsize=T_NOTE, color=MUTED, ha="left", va="center")
    title = "Relation to published systems: studied axes, not performance"
    desc = ("Coverage matrix of axes studied by NavRL, Elastic Tracker, Fast-Tracker, OPEN, YOPO and MOTAR: target "
            "tracking, random clutter, perception uncertainty, temporal estimation, safety analysis, observation "
            "contract, and whether any real-hardware result is reported (MOTAR: none, simulation-only). A mark is "
            f"not a performance score. Class A matched external comparisons = {v['class_a_count']}.")
    return fig, title, desc


# ---------------------------------------------------------------- architecture code map
def figure_arch(plt, v):
    fig, ax, H = canvas(plt, 5.3, width_in=WIDTH_IN)
    w, h = 22.4, 9.6
    col = [2.0, 26.5, 51.0, 75.5]

    def block(x, y, title, role, files, edge=RULE, fill=WHITE):
        box(ax, x, y, w, h, edge=edge, fill=fill)
        ax.text(x + 1.0, y + h - 1.7, title, fontsize=9.8, fontweight="bold", color=INK, ha="left", va="center")
        ax.text(x + 1.0, y + h - 3.7, role, fontsize=8.3, color=INK2, ha="left", va="center")
        for k, name in enumerate(files):
            ax.text(x + 1.0, y + h - 5.8 - 1.65 * k, name, fontsize=7.5, family=MONO, color=MUTED, ha="left",
                    va="center")

    ax.text(2.0, H - 1.8, "INPUTS THAT DEFINE A SCENARIO", fontsize=8.6, fontweight="bold", color=MUTED,
            ha="left", va="center")
    ytop = H - 13.0
    block(col[0], ytop, "Target-motion generator", "H legacy · E0 static · E1 CV · E2 local",
          ["target_motion.py", "physical_target.py (route, gated)"], fill=PANEL)
    block(col[1], ytop, "Rendering contract", "analytic proxy or mesh treatment (D8)",
          ["navrl_dynamic_mesh_treatment.py", "tools/renderer_validation/"], fill=PANEL)
    block(col[2], ytop, "Measured error injection", "P8 model driven through P9 (opt-in)",
          ["navrl_empirical_error.py"], fill=PANEL)
    block(col[3], ytop, "Task and checkpoint contract", "success 0.5 m · 60 s · frozen weights",
          ["navrl_task_config.py", "research_task_contract.json"], fill=PANEL)

    ya = ytop - 14.0
    yb = ya - 13.5
    block(col[0], ya, "Obstacle environment", "40 × 40 × 3 m arena, 70–205 bars",
          ["navrl_bars_env.py", "navrl_curriculum.py"])
    block(col[1], ya, "Sensors", "forward camera · LiDAR 72 × 4 @ 12 m",
          ["navrl_detector.py", "navrl_lidar_config.py"])
    block(col[2], ya, "Perception + temporal state", "detection, tracking, 2 s history",
          ["navrl_perception.py"])
    block(col[3], ya, "Policy observation", "898-D actor · 906-D critic · 17 tokens",
          ["navrl_task.py", "navrl_perception.py"])
    block(col[3], yb, "Frozen PPO policy", "Transformer actor-critic (rl_games)",
          ["navrl_transformer_network.py", "runner.py"], edge=INK2)
    block(col[2], yb, "Safety filter", "speed cap from LiDAR clearance",
          ["speed_governor.py"])
    block(col[1], yb, "Low-level control", "Lee velocity controller",
          ["velocity_control.py", "lee_controller_config_navrl.py"])
    block(col[0], yb, "Vehicle dynamics", "quadrotor, Isaac Gym PhysX",
          ["navrl_*_quad_config.py"])
    for i in range(3):
        arrow(ax, (col[i] + w + 0.2, ya + h / 2), (col[i + 1] - 0.2, ya + h / 2), lw=1.0, scale=8)
        arrow(ax, (col[i + 1] - 0.2, yb + h / 2), (col[i] + w + 0.2, yb + h / 2), lw=1.0, scale=8)
    arrow(ax, (col[3] + w / 2, ya - 0.2), (col[3] + w / 2, yb + h + 0.2), lw=1.0, scale=8)
    arrow(ax, (col[0] + w / 2, yb + h + 0.2), (col[0] + w / 2, ya - 0.2), lw=1.0, scale=8)
    ax.text(50.0, (ya + yb + h) / 2, "CLOSED LOOP · simulation · 0.1 s RL step", fontsize=8.6, fontweight="bold",
            color=MUTED, ha="center", va="center")
    for i, target in enumerate([col[0], col[1], col[2], col[3]]):
        arrow(ax, (target + w / 2, ytop - 0.2), (target + w / 2, ya + h + 0.2), color=MUTED, lw=0.9, scale=7)

    ye = 1.6
    box(ax, 2.0, ye, 95.9, 7.6, edge=RULE, fill=BLUE_WASH)
    ax.text(3.0, ye + 5.6, "EVIDENCE LAYER", fontsize=8.6, fontweight="bold", color=INK2, ha="left", va="center")
    steps = [("fail-closed launchers", "eval_navrl_*.sh"),
             ("receipts + raw counts", "results/<run>/"),
             ("canonical registries", "docs/*registry.json"),
             ("docs · site · figures", "tools/build_paper_figures.py"),
             ("guard tests", "tests/")]
    for k, (label, name) in enumerate(steps):
        x = 3.0 + k * 19.1
        ax.text(x, ye + 3.4, label, fontsize=8.6, color=INK, ha="left", va="center")
        ax.text(x, ye + 1.5, name, fontsize=7.5, family=MONO, color=MUTED, ha="left", va="center")
        if k < 4:
            arrow(ax, (x + 15.6, ye + 3.4), (x + 18.4, ye + 3.4), lw=0.9, scale=7)
    title = "MOTAR code map: closed loop, scenario inputs and evidence layer"
    desc = ("Closed simulation loop - obstacle environment, sensors (camera, LiDAR), perception and temporal "
            "state, policy observation (898-D actor, 906-D critic, 17 tokens), frozen Transformer PPO policy, "
            "LiDAR speed-cap safety filter, Lee velocity controller, quadrotor dynamics - with scenario inputs "
            "(target-motion generator, rendering contract, measured error injection, task contract) and the "
            "evidence layer (launchers, receipts, registries, generated docs and figures, guard tests). File "
            "names are under aerial_gym/ unless prefixed.")
    return fig, title, desc


FIGURES = [
    ("fig1-motar-overview", figure1),
    ("fig2-perception-policy", figure2),
    ("fig3-safety", figure3),
    ("fig4-target-motion", figure4),
    ("fig5-observation-contract", figure5),
    ("fig6-positioning", figure6),
    ("arch-motar-code-map", figure_arch),
]


# ---------------------------------------------------------------- output
def finish_svg(raw: str, stem: str, title: str, desc: str) -> str:
    raw = re.sub(r"<metadata>.*?</metadata>\s*", "", raw, flags=re.S)
    raw = raw.replace("'Liberation Sans'", FONT_STACK).replace("'Liberation Mono'", MONO_STACK)

    def esc(text):
        return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")

    raw = raw.replace("<svg xmlns:xlink", f'<svg role="img" aria-labelledby="{stem}-title {stem}-desc" '
                                          "xmlns:xlink", 1)
    head_end = raw.index(">", raw.index("<svg")) + 1
    return (raw[:head_end] + f'\n <title id="{stem}-title">{esc(title)}</title>\n'
            f' <desc id="{stem}-desc">{esc(desc)}</desc>' + raw[head_end:])


def render_all():
    os.environ.setdefault("SOURCE_DATE_EPOCH", "0")
    plt = setup_matplotlib()
    import matplotlib
    values = collect_values()
    files, captions = {}, {}
    for stem, builder in FIGURES:
        fig, title, desc = builder(plt, values)
        captions[stem] = {"title": title, "desc": desc}
        svg = io.StringIO()
        fig.savefig(svg, format="svg", metadata={"Date": None, "Creator": None})
        files[stem + ".svg"] = finish_svg(svg.getvalue(), stem, title, desc).encode("utf-8")
        pdf = io.BytesIO()
        fig.savefig(pdf, format="pdf", metadata={"CreationDate": None, "Creator": None, "Producer": None,
                                                 "Title": title})
        files[stem + ".pdf"] = pdf.getvalue()
        png = io.BytesIO()
        fig.savefig(png, format="png", dpi=220, metadata={"Software": None})
        files[stem + ".png"] = png.getvalue()
        plt.close(fig)
    manifest = {
        "schema_version": 1,
        "generator": "tools/build_paper_figures.py",
        "matplotlib": matplotlib.__version__,
        "font": FONT,
        "sources": {path: sha256_file(ROOT / path) for path in SOURCES.values()},
        "captions": captions,
        "values": values,
        "files": {name: hashlib.sha256(data).hexdigest() for name, data in sorted(files.items())},
    }
    files["manifest.json"] = (json.dumps(manifest, indent=1, sort_keys=True, ensure_ascii=False) + "\n").encode("utf-8")
    return files


def check(files) -> int:
    path = OUT / "manifest.json"
    if not path.is_file():
        print(json.dumps({"status": "FAIL", "problems": ["no recorded manifest; run the builder"]}, indent=2))
        return 1
    recorded = json.loads(path.read_text(encoding="utf-8"))
    fresh = json.loads(files["manifest.json"])
    problems = []
    if recorded["values"] != fresh["values"]:
        problems.append("source-bound values differ from the recorded manifest")
    if recorded["sources"] != fresh["sources"]:
        problems.append("a canonical source changed since the figures were built")
    same_engine = recorded.get("matplotlib") == fresh["matplotlib"]
    if same_engine:
        for name, data in sorted(files.items()):
            if name.endswith((".png", ".pdf")):
                # PNG and PDF are not tracked; their recorded hashes pin the rebuild.
                if hashlib.sha256(data).hexdigest() != recorded["files"].get(name):
                    problems.append(f"{name} differs from its recorded hash")
                continue
            target = OUT / name
            if not target.is_file() or target.read_bytes() != data:
                problems.append(f"{name} differs from a fresh build")
    scope = "byte-exact" if same_engine else (
        f"values only (recorded matplotlib {recorded.get('matplotlib')}, running {fresh['matplotlib']})")
    print(json.dumps({"status": "FAIL" if problems else "PASS", "scope": scope, "problems": problems}, indent=2))
    return 1 if problems else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--check", action="store_true", help="rebuild in memory and compare")
    parser.add_argument("--out", type=Path, default=OUT, help="output directory (default: %(default)s)")
    args = parser.parse_args()
    files = render_all()
    if args.check:
        return check(files)
    args.out.mkdir(parents=True, exist_ok=True)
    for name, data in files.items():
        (args.out / name).write_bytes(data)
    print(f"wrote {len(files)} files to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
