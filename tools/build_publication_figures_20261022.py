#!/usr/bin/env python3
"""Rebuild publication figures from immutable records; no simulator imports or GPU.

Run with the aerialgym CPU Python and PYTHONNOUSERSITE=1. SVG text stays editable;
PDF fonts are embedded and PNG is 300 dpi. Historical figure packages are untouched.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import zipfile

os.environ.setdefault("MPLBACKEND", "Agg")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/motar-publication-mpl-20261022")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Rectangle
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/assets/paper/redesign-2026-10-22"
MEASUREMENT = "b1c0bc612339d2f8dfb86a0ca16de11bdffe09b5"
SOURCES = {
    "motion": "results/target_motion_replication_7seed/analysis.json",
    "cells": "results/target_motion_replication_7seed/cells.csv",
    "perception": "results/perception_p10_seed_replication_2026-09-10/summary.json",
    "safety": "results/independent_verification_2026-09-07/recomputed.json",
    "observation": "results/dynamic_mesh_policy_sensitivity_d8b_2026-09-13/summary.json",
    "progress": "docs/status/data/boia-progress.json",
    "e3s": "results/eth_ds5_e3s_2026-09-10/run/e3s_result.json",
    "component_registry": "docs/research_status_registry.json",
    "status_manifest": "docs/status_manifest.json",
}
INK, MUTED, BLUE, TEAL, ORANGE = "#172b3a", "#546675", "#28648d", "#217f75", "#ae5938"
RULE, GRID, WASH, WHITE = "#bac7cf", "#e8edf0", "#f2f6f8", "#ffffff"
COLORS = ["#72818b", BLUE, ORANGE, TEAL]
ARMS = ["H_historical", "E0_static", "E1_cv", "E2_obstacle_aware"]
NAMES = ["H · training lineage", "E0 · static", "E1 · constant velocity", "E2 · obstacle-aware"]
FIELDS = [
    {"name": "Fused static scan", "shape": [4, 72], "flat": 288, "tokens": 1},
    {"name": "Obstacle history", "shape": [5, 8, 12], "flat": 480, "tokens": 5},
    {"name": "Own-state history", "shape": [5, 10], "flat": 50, "tokens": 5},
    {"name": "Target-track history", "shape": [5, 16], "flat": 80, "tokens": 5},
]
GRAPH = {
    "baseline_edges": [["rgbd", "perception"], ["lidar", "perception"], ["own_state", "perception"],
                       ["perception", "flat898"], ["flat898", "tokens17"], ["tokens17", "actor"],
                       ["actor", "action_transform"], ["action_transform", "xy_governor"],
                       ["lidar", "xy_governor"], ["xy_governor", "lee_controller"],
                       ["lee_controller", "physx"], ["physx", "sensor_models"],
                       ["sensor_models", "rgbd"], ["sensor_models", "lidar"], ["physx", "own_state"]],
    "baseline_governor": "riskcap",
    "alternative_governor": "arc-clearance (experimental alternate; not serial)",
    "vertical_action": "Actor z output overwritten by task PI altitude hold",
    "ground_truth": "Scene generation/rendering and evaluator/critic only; no direct actor target-GT input",
    "boia_applies_action": False,
    "actor_fields": FIELDS,
    "history_slots": 5,
    "nominal_history_interval_s": 0.5,
    "tracker_expiry_s": 5.0,
    "token_embedding_width": 64,
    "source_commit": MEASUREMENT,
    "source_locators": {
        "flat_and_history": "navrl_perception.py::observe, _update_histories, _target_features",
        "tokens": "navrl_transformer_network.py::forward",
        "action": "navrl_task.py::transform_action_to_command (z overwrite)",
        "filter": "navrl_task.py::_apply_sensor_speed_governor; speed_governor.py::apply_speed_governor",
        "control": "velocity_control.py::LeeVelocityController.update",
    },
}


def digest(b):
    return hashlib.sha256(b).hexdigest()


def values():
    d = {k: json.loads((ROOT / p).read_text()) for k, p in SOURCES.items() if k != "cells"}
    with (ROOT / SOURCES["cells"]).open() as f:
        cells = list(csv.DictReader(f))
    q = d["motion"]["estimators"]["quota"]
    seeds = sorted({int(r["seed"]) for r in cells})
    densities = sorted({int(r["bars"]) for r in cells})
    rate = {}
    for arm in ARMS:
        rate[arm] = [[100 * int(next(r for r in cells if r["arm"] == arm and int(r["seed"]) == seed
                                  and int(r["bars"]) == density)["primary_capture"]) /
                     int(next(r for r in cells if r["arm"] == arm and int(r["seed"]) == seed
                              and int(r["bars"]) == density)["primary_episodes"])
                     for density in densities] for seed in seeds]
    p = d["perception"]
    frozen = p["per_training_seed"][sorted(p["per_training_seed"])[0]]["p9_cost_on_source"]
    s = d["safety"]["contrasts"]["dwa_arc-riskcap"]
    return {
        "seeds": seeds, "densities": densities, "density_capture_pct": rate,
        "episodes": sum(int(r["primary_episodes"]) for r in cells),
        "motion_primary": q["primary"],
        "motion_capture_pct": {k: 100 * v for k, v in q["arm_mean_capture"].items()},
        "frozen_cost": frozen, "readaptation": p["R2_seed_level_t"],
        "safety_pooled": s["pooled"], "safety_seed": s["seed_t"], "safety_per_seed": s["per_seed"],
        "safety_cells": s["cell"], "observation": d["observation"]["primary"],
        "progress": d["progress"]["milestones"], "research_snapshot": d["progress"]["research_snapshot"],
        "e3s": {"median_pct":100*d["e3s"]["primary"]["gates"]["G2_central"]["median_of_block_medians"],
                "frames":d["e3s"]["data_quality"]["included_frames"],"blocks":d["e3s"]["primary"]["blocks"],
                "block_seconds":d["e3s"]["block_seconds"]},
    }


def setup():
    plt.rcParams.update({"font.family": "Liberation Sans", "font.size": 8.5,
                        "text.color": INK, "axes.labelcolor": INK, "axes.labelsize": 8.5,
                        "xtick.labelsize": 8, "ytick.labelsize": 8, "axes.spines.top": False,
                        "axes.spines.right": False, "axes.edgecolor": RULE, "axes.linewidth": .7,
                        "xtick.color": MUTED, "ytick.color": MUTED,
                        "svg.fonttype": "none", "svg.hashsalt": "motar-redesign-20261022",
                        "pdf.fonttype": 42, "legend.fontsize": 8, "legend.frameon": False,
                        "figure.facecolor": WHITE, "savefig.facecolor": WHITE,
                        "savefig.dpi": 300})


def canvas(height=4.1):
    fig = plt.figure(figsize=(7.2, height))
    ax = fig.add_axes([.025, .035, .95, .93], xlim=(0, 100), ylim=(60, 0))
    ax.axis("off")
    return fig, ax


def text(ax, x, y, s, size=8.5, bold=False, color=INK, ha="left", **kw):
    return ax.text(x, y, s, fontsize=size, fontweight="bold" if bold else "normal",
                   color=color, ha=ha, va="center", linespacing=1.4, **kw)


def box(ax, x, y, w, h, title, body="", fill=WHITE, edge=RULE, dashed=False, title_size=8.8):
    ax.add_patch(Rectangle((x, y), w, h, facecolor=fill, edgecolor=edge,
                           linewidth=.85, linestyle=(0, (4, 3)) if dashed else "solid"))
    if body:
        text(ax, x+1.5, y+3.4, title, title_size, True)
        text(ax, x+1.5, y+h*.66, body, 8.0, color=MUTED)
    else:
        text(ax, x+w/2, y+h/2, title, title_size, True, ha="center")


def arrow(ax, points, color=MUTED, dashed=False, head=True):
    if len(points) > 2:
        xs, ys = zip(*points[:-1])
        ax.plot(xs, ys, color=color, lw=.9, linestyle="--" if dashed else "-")
    p, q = points[-2:]
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>" if head else "-", mutation_scale=8,
                                color=color, lw=.9, linestyle="--" if dashed else "-"))


def system(v):
    fig, ax = canvas(4.3)
    ax.set_xlim(-5,101)
    text(ax, 0, 1.5, "(a) Sensor-to-action loop", 10, True)
    text(ax, 100, 1.5, "SCHEMATIC · FROZEN POLICY F", 8, color=MUTED, ha="right")
    box(ax, 0, 8, 21, 11, "RGB-D camera", "Synthetic RGB + range", WASH)
    box(ax, 0, 23, 21, 11, "LiDAR", "4 × 72 ray returns", WASH)
    box(ax, 0, 38, 21, 11, "Own vehicle state", "Pose, velocity; simulator", WASH)
    box(ax, 28, 8, 30, 26, "Perception + tracking", "Camera measurement + LiDAR aid\n6-D target state, 6 × 6 covariance\nObstacle features + sampled history")
    box(ax, 65, 8, 35, 26, "Frozen Transformer PPO", "898-D observation → 17 tokens\nActor outputs 4 components\nTask replaces z with altitude hold")
    arrow(ax, [(21, 13), (28, 13)])
    arrow(ax, [(21, 28), (28, 28)])
    arrow(ax, [(21, 43), (24, 43), (24, 32), (28, 32)])
    arrow(ax, [(58, 21), (65, 21)])
    box(ax, 65, 40, 35, 12, "XY speed governor", "Policy F: riskcap magnitude cap")
    box(ax, 28, 40, 30, 12, "Lee velocity control + PhysX", "Velocity/yaw → thrust + torques")
    arrow(ax, [(82, 34), (82, 40)])
    arrow(ax, [(65, 46), (58, 46)])
    arrow(ax, [(12, 34), (12, 36), (61, 36), (61, 44), (65, 44)], color=BLUE)
    arrow(ax, [(43, 52), (43, 56), (-3, 56), (-3, 13)],head=False)
    for y in [13,28,43]:
        arrow(ax,[(-3,y),(0,y)])
    text(ax, 10, 57.8, "Vehicle / scene state → next sensor delivery", 8, color=MUTED)
    text(ax, 65, 57.8, "Arc-clearance: separate experimental mode", 7.8, color=MUTED)
    return fig


def observation(v):
    fig, ax = canvas(4.5)
    text(ax, 0, 1, "(a) Flat actor observation", 10, True)
    text(ax, 100, 1, "898 scalar values · before network embedding", 8.5, ha="right", color=MUTED)
    rows = [("Fused static scan", "RGB-D + LiDAR · 4 × 72 bins", "288", "1 token"),
            ("Obstacle history", "5 steps × 8 slots × 12 features", "480", "5 tokens"),
            ("Own-state history", "5 steps × 10 features", "50", "5 tokens"),
            ("Target-track history", "5 steps × 16 features", "80", "5 tokens")]
    for i, (name, shape, n, token) in enumerate(rows):
        y=7+i*8
        ax.add_patch(Rectangle((0,y), 68, 7, facecolor=WASH if i%2==0 else WHITE, edgecolor=RULE, lw=.6))
        text(ax, 2, y+2.5, name, 8.7, True)
        text(ax, 2, y+5.2, shape, 8, color=MUTED)
        text(ax, 65, y+3.5, n, 10, True, ha="right")
        arrow(ax, [(68,y+3.5),(74,y+3.5)])
        box(ax,75,y,25,7,token,title_size=8.6)
    text(ax, 2, 42, "(b) Network sequence", 10, True)
    text(ax, 2, 47, "[CLS] + scan + obstacle[1..5] + own[1..5] + target[1..5]", 8.5, True)
    text(ax, 100, 47, "17 × 64", 10, True, ha="right")
    ax.plot([0,100],[51,51],color=RULE,lw=.7)
    text(ax, 0, 54, "History: 5 slots at nominal 0.5 s spacing (2 s span). Tracker expiry: 5 s.", 8)
    text(ax, 0, 58, "Target history stores measured/estimated features; no direct target-GT fields enter the actor.", 8, color=MUTED)
    return fig


def motion(v):
    fig, axs = plt.subplots(1,2,figsize=(7.2,3.3),gridspec_kw={"width_ratios":[1,1.2]})
    fig.subplots_adjust(left=.09,right=.98,bottom=.25,top=.76,wspace=.38)
    a,b=axs
    for i, arm in enumerate(ARMS):
        rates=np.mean(np.array(v["density_capture_pct"][arm]),axis=1)
        a.scatter(i+np.linspace(-.13,.13,len(rates)),rates,s=17,facecolor=WHITE,edgecolor=COLORS[i],lw=.9,zorder=3)
        a.plot([i-.23,i+.23],[rates.mean()]*2,color=COLORS[i],lw=2.3,zorder=4)
    a.set_xticks(range(4),["H","E0","E1","E2"])
    a.set_ylim(70,100);a.set_yticks([70,80,90,100]);a.set_ylabel("Capture (%)")
    a.grid(axis="y",color=GRID);a.set_axisbelow(True)
    a.set_title("(a) Equal-weight density means",loc="left",fontsize=9,fontweight="bold",pad=15)
    a.text(0,-.27,"Open marks: evaluation seeds; line: mean.\nAxis begins at 70%; same quota per environment.",transform=a.transAxes,fontsize=8,color=MUTED)
    b.axvline(0,color=RULE,lw=.9)
    for i,r in enumerate(v["motion_primary"]):
        y=2-i; mean=100*r["mean_diff"]; lo,hi=[100*x for x in r["bca95"]]
        color=COLORS[i+1]
        vals=list(r["seed_diffs"].values())
        b.scatter([100*x for x in vals],y+np.linspace(-.12,.12,len(vals)),s=17,facecolor=WHITE,edgecolor=color,lw=.9,zorder=3)
        b.errorbar(mean,y,xerr=[[mean-lo],[hi-mean]],fmt="D",ms=5,color=color,capsize=3,lw=1.4,zorder=4)
        b.text(mean,y+.32,f"{mean:+.2f} pp",ha="center",fontsize=8.5,fontweight="bold")
    b.set_yticks([2,1,0],["E0 − H","E1 − H","E2 − H"]);b.set_ylim(-.5,2.7)
    b.set_xlim(-7,3);b.set_xticks([-6,-4,-2,0,2]);b.set_xlabel("Capture difference (percentage points)")
    b.grid(axis="x",color=GRID);b.set_axisbelow(True)
    b.set_title("(b) Paired contrasts · recorded BCa 95%",loc="left",fontsize=9,fontweight="bold",pad=15)
    fig.text(.09,.98,"Independent target-motion replication",fontsize=11,fontweight="bold",va="top")
    fig.text(.09,.02,f"Policy F · {len(v['seeds'])} fresh evaluation seeds · {v['episodes']:,} primary episodes · inference unit: seed",fontsize=8,color=MUTED)
    return fig


def density(v):
    fig,axs=plt.subplots(1,2,figsize=(7.2,3.5),gridspec_kw={"width_ratios":[1.3,1]})
    fig.subplots_adjust(left=.09,right=.98,bottom=.24,top=.82,wspace=.4)
    a,b=axs;ds=v["densities"]
    for i,arm in enumerate(ARMS):
        rows=np.array(v["density_capture_pct"][arm])
        for row in rows:a.plot(ds,row,color=COLORS[i],alpha=.16,lw=.6)
        a.plot(ds,rows.mean(axis=0),"o-",color=COLORS[i],ms=4,lw=1.8,label=["H","E0","E1","E2"][i])
    a.set_xticks(ds);a.set_ylim(70,100);a.set_yticks([70,80,90,100]);a.set_ylabel("Capture (%)")
    a.set_xlabel("Obstacle density (bars per arena)");a.grid(axis="y",color=GRID);a.set_axisbelow(True)
    a.legend(ncol=4,loc="lower left",bbox_to_anchor=(-.04,1.02),columnspacing=.8,handlelength=1.2)
    diff=np.array(v["density_capture_pct"][ARMS[3]])-np.array(v["density_capture_pct"][ARMS[0]])
    b.axhline(0,color=RULE,lw=.9)
    for j,den in enumerate(ds):
        b.scatter(np.full(len(v["seeds"]),j)+np.linspace(-.13,.13,len(v["seeds"])),diff[:,j],s=19,facecolor=WHITE,edgecolor=TEAL,lw=.9)
        b.plot([j-.25,j+.25],[diff[:,j].mean()]*2,color=TEAL,lw=2)
    b.set_xticks(range(len(ds)),ds);b.set_ylabel("E2 − H capture (pp)");b.set_xlabel("Bars per arena")
    b.set_ylim(-2,5);b.grid(axis="y",color=GRID);b.set_axisbelow(True)
    b.set_title("Paired seed differences",loc="left",fontsize=9,fontweight="bold",pad=15)
    fig.text(.09,.97,"Density response · exploratory display",fontsize=11,fontweight="bold",va="top")
    fig.text(.09,.07,"Thin lines / open dots: seven seeds. Thick lines: means. Left axis begins at 70%.",fontsize=8,color=MUTED)
    fig.text(.09,.02,"No density-specific confidence intervals or confirmatory density claims are introduced.",fontsize=8,color=MUTED)
    return fig


def sensitivities(v):
    fig, axs=plt.subplots(2,2,figsize=(7.2,5.0))
    fig.subplots_adjust(left=.08,right=.97,bottom=.12,top=.77,wspace=.35,hspace=1.3)
    cases=[
        ("(a) Measured-error injection · policy F",v["frozen_cost"]["delta_pp"],v["frozen_cost"]["ci95"],(-8,2),"Capture difference (pp)","Episode-level 95% CI; 2 evaluation seeds."),
        ("(b) Readaptation from policy F",v["readaptation"]["mean_pp"],v["readaptation"]["ci95"],(-4,4),"Capture difference (pp)","3 training seeds · t 95% CI · INCONCLUSIVE."),
        ("(c) Arc vs riskcap · policy F",v["safety_seed"]["mean"],[v["safety_seed"]["lo"],v["safety_seed"]["hi"]],(-3,1),"Crash difference (pp)","Seed mean + t 95% CI; 3 evaluation seeds."),
        ("(d) Shaded mesh vs analytic · policy R",v["observation"]["mean"],v["observation"]["ci95"],(-55,5),"Capture difference (pp)","3 paired seeds · t 95% CI · cause NOT_TESTED."),
    ]
    seed_values=[None,v['readaptation']['per_seed_delta_pp'],
                 [r['delta'] for r in v['safety_per_seed']],v['observation']['values']]
    for ax,(title,mu,ci,lim,label,note),dots in zip(axs.flat,cases,seed_values):
        ax.axvline(0,color=RULE,lw=.9)
        ax.errorbar(mu,0,xerr=[[mu-ci[0]],[ci[1]-mu]],fmt="D",ms=5,color=BLUE,capsize=4,lw=1.6)
        if dots is not None:
            ax.scatter(dots,np.linspace(-.25,-.35,len(dots)),s=15,facecolor=WHITE,edgecolor=BLUE,lw=.8,zorder=3)
        ax.text(mu,.33,f"{mu:+.2f}",ha="center",fontsize=10,fontweight="bold")
        ax.set_xlim(*lim);ax.set_ylim(-.5,.8);ax.set_yticks([]);ax.set_xlabel(label)
        ax.set_title(title,loc="left",fontsize=8.8,fontweight="bold",pad=27)
        ax.grid(axis="x",color=GRID);ax.set_axisbelow(True)
        ax.spines['left'].set_visible(False)
        ax.text(0,1.12,note,transform=ax.transAxes,fontsize=7.8,color=MUTED)
    fig.text(.08,.98,"Separate experiments, separate estimands",fontsize=11,fontweight="bold",va="top")
    fig.text(.08,.92,"Intervals and horizontal scales differ. These effects are neither pooled nor additive.",fontsize=8,color=MUTED)
    return fig


def boia(v):
    fig,ax=canvas(4.3)
    text(ax,0,1,"(a) Passive research branch",10,True)
    text(ax,100,1,"SCHEMATIC · SOURCE SNAPSHOT "+v["research_snapshot"][:7],8,color=MUTED,ha="right")
    box(ax,0,7,100,9,"Immutable sensor-side input", "RGB-D + LiDAR; own state; permitted tracker mean / covariance / age",WASH)
    box(ax,0,23,29,13,"B · Belief / history", "Tracker mean + covariance\nPrior acquisition / loss state")
    box(ax,35.5,23,29,13,"C · Local map", "Timestamped depth + LiDAR\nObserved-space limitations")
    box(ax,71,23,29,13,"A · Evidence", "Detection ≠ identity\nValidity + geometry checks")
    for x in [14.5,50,85.5]:arrow(ax,[(x,16),(x,23)])
    box(ax,0,43,57,11,"D · Candidate screens", "L1/L2 only; review FAIL / HOLD",WASH)
    box(ax,65,43,35,11,"Timing + shadow log", "Utility ABSTAIN\nNo action executor",WASH)
    arrow(ax,[(14.5,36),(14.5,43)])
    arrow(ax,[(50,36),(50,43)])
    arrow(ax,[(85.5,36),(85.5,43)])
    arrow(ax,[(57,48),(65,48)])
    text(ax,0,58,"(b) Applied commands remain owned by Frozen F → XY governor → Lee controller.",8.5,True)
    return fig


FIGURES = [
    ("fig01-system",system,"Verified sensor-to-action architecture","Historical Policy F closed loop. Sensor models and own state feed perception; 898 flat values precede tokenization. PI altitude hold replaces actor z. Baseline riskcap and experimental arc-clearance are alternatives."),
    ("fig02-observation",observation,"Actor observation data structure","288 + 480 + 50 + 80 = 898 flat actor values; 17 embedded tokens include CLS. Five history slots nominally span two seconds; tracker expiry is five seconds. No direct target ground truth enters the actor."),
    ("fig03-motion",motion,"Independent target-motion replication","Recorded seven-seed paired capture contrasts with BCa intervals. Unit of inference is evaluation seed, equal-weight densities and per-environment quota. Historical three-seed results are not pooled."),
    ("fig04-density",density,"Target-motion density response","New exploratory display of immutable primary counts, seven seed trajectories and density-wise E2 minus H contrasts. No new tests or density-specific confidence intervals."),
    ("fig05-sensitivities",sensitivities,"Separate sensitivity experiments","Perception cost and readaptation, seed-level arc crash mean, and rendering sensitivity in distinct panels. Policy F and policy R never pooled. Different inference units and independent horizontal axes are labelled."),
    ("fig06-boia",boia,"BOIA passive evidence and proposal architecture","Separate shadow-only research branch. Input evidence, permitted belief and timestamped local map feed candidate checks and timing records. Q02-B and Q03 remain FAIL/HOLD; no executor or online action path."),
]


def render(out):
    setup(); v=values();out.mkdir(parents=True,exist_ok=True)
    files={}; meta=[]
    for stem,fn,title,desc in FIGURES:
        fig=fn(v)
        for ext in ["svg","pdf","png"]:
            buf=io.BytesIO()
            metadata={"Creator":"MOTAR reproducible publication figure builder"}
            if ext=="pdf":metadata.update({"CreationDate":None,"ModDate":None})
            if ext=="svg":metadata.update({"Date":None,"Title":title,"Description":desc})
            fig.savefig(buf,format=ext,metadata=metadata)
            b=buf.getvalue()
            if ext=="svg":
                s=b.decode();s=s.replace('<svg ', '<svg role="img" aria-labelledby="figure-title figure-desc" ',1)
                end=s.index('>',s.index('<svg '))+1
                from xml.sax.saxutils import escape
                s=s[:end]+'\n<title id="figure-title">'+escape(title)+'</title>\n<desc id="figure-desc">'+escape(desc)+'</desc>'+s[end:]
                # Matplotlib path commands carry trailing spaces; normalize only
                # whitespace, retaining editable text and exact vector geometry.
                b=('\n'.join(line.rstrip() for line in s.splitlines())+'\n').encode()
            name=stem+'.'+ext;(out/name).write_bytes(b);files[name]=digest(b)
        meta.append({"id":stem,"title":title,"caption":desc,"width_in":float(fig.get_size_inches()[0]),
                     "height_in":float(fig.get_size_inches()[1]),"png_dpi":300})
        plt.close(fig)
    payload={"schema":"MOTAR-PUBLICATION-REDESIGN-1","measurement_source":MEASUREMENT,
             "research_snapshot":v["research_snapshot"],"matplotlib_version":matplotlib.__version__,
             "sources":{k:{"path":p,"sha256":digest((ROOT/p).read_bytes())} for k,p in SOURCES.items()},
             "graph":GRAPH,"values":v,"figures":meta,"files":files,
             "density_display":"Exploratory re-display of primary counts. No added CI or hypothesis test.",
             "safety_estimands":"Chart uses seed_t.mean with seed_t CI; pooled estimate preserved separately."}
    (out/'manifest.json').write_text(json.dumps(payload,indent=2)+'\n')
    with zipfile.ZipFile(out/'motar-publication-figures.zip','w',zipfile.ZIP_DEFLATED) as z:
        for p in sorted(out.iterdir()):
            if p.suffix in ('.svg','.pdf','.png','.json'):
                info=zipfile.ZipInfo(p.name,date_time=(2026,10,22,0,0,0));info.compress_type=zipfile.ZIP_DEFLATED
                z.writestr(info,p.read_bytes())
    try:
        name=str(out.relative_to(ROOT))
    except ValueError:
        name=str(out)
    print(json.dumps({"status":"PASS","figures":len(meta),"formats":["SVG","PDF","PNG"],"out":name}))


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=OUT);a=p.parse_args();render(a.out)


if __name__=='__main__':main()
