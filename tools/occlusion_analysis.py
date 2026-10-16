#!/usr/bin/env python3
"""Occlusion analysis of R-P1a-v2 traces (CPU, read-only on its inputs).

    python -B tools/occlusion_analysis.py cell --dir CELL [--out NEW.json]
    python -B tools/occlusion_analysis.py pilot --root RUN [--out NEW.json]
    python -B tools/occlusion_analysis.py sweep --cells CELL... [--paths 8] [--seed 0] [--out NEW.json]
    python -B tools/occlusion_analysis.py sweep --synthetic [--paths 8] [--seed 0]

Indicators are kept apart and never relabelled as one another:

* ``fused_loss``                -- no fused measurement (camera or associated LiDAR); not an occlusion;
* ``center_occlusion_in_view``  -- offline 3D: target centre inside the frustum and range, centre ray
                                   blocked by a box (geometry, not the rendered image);
* ``geometric_full_in_view``    -- the same with every silhouette-rim ray blocked too (a proxy);
* ``out_of_view``               -- target centre outside the frustum or range;
* ``rendered_full_in_view``     -- the R-P1a probe saw zero rendered target pixels with the centre in
                                   view; only when the probe ran and render alignment was verified,
                                   otherwise ``NOT_RECORDED``.

Only primary-quota episodes enter the summaries. Intervals count after the first fused acquisition;
an interval still open when the episode ends is censored with the task outcome as its end cause.
Offline line of sight is compared frame by frame with the recorder's online box test and, when
present, with the Warp mesh probe (``parity``); neither comparison is GPU renderer parity.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motar_eval import occlusion_events as ev  # noqa: E402
from motar_eval import occlusion_geometry as og  # noqa: E402
from motar_eval import occlusion_sweep as osw  # noqa: E402
from motar_eval import occlusion_trace as ot  # noqa: E402

INDICATORS = ("fused_loss", "center_occlusion_in_view", "geometric_full_in_view", "out_of_view",
              "rendered_full_in_view")


def _jsonl(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def analyse_cell(directory, rim_count: int = 8):
    directory = Path(directory)
    trace = ot.read_trace(directory)
    meta = trace["summary"]["metadata"]
    dt, cam = float(meta["dt_s"]), meta["camera"]
    hfov, vfov = np.radians(cam["hfov_deg"]), np.radians(cam["vfov_deg"])
    aligned = meta.get("render_alignment") == "ZERO_LATENCY_RENDER_DELIVERY"
    lookup = ot.reset_lookup(trace["layouts"])
    ledger = {(r["env_index"], r["env_episode_index"]): r for r in _jsonl(directory / "episodes.jsonl")}
    records = {(r["env_index"], r["env_episode_index"]): r for r in _jsonl(directory / "kroc2027_episodes.jsonl")}
    intervals = {k: [] for k in INDICATORS}
    exposures, reacq, problems, per_episode = [], [], [], []
    parity = {"frames": 0, "offline_vs_online_box_disagree": 0, "probe_frames": 0, "offline_vs_probe_disagree": 0}
    episodes = 0
    for key, frames in ot.episode_frames(trace["frames"]).items():
        row = ledger.get(key)
        if row is None or not row.get("in_quota"):
            continue
        record = records.get(key)
        if record is None:
            problems.append("no recorder record for env %d episode %d" % key)
            continue
        if key not in lookup:
            problems.append("no layout for env %d episode %d" % key)
            continue
        episodes += 1
        terminal = int(record["actions"])
        if frames["frame"][-1] != terminal - 1 or frames["frame"][0] != 0:
            problems.append("frames of env %d episode %d do not cover 0..actions-1" % key)
            continue
        center, half, quat = ot.layout_arrays(trace["layouts"], lookup[key])
        eye = og.camera_eye(frames["p_pos"], frames["p_vehicle_quat"], cam["translation_m"])
        target = frames["t_pos"].astype(np.float64)
        angular, ranged = og.in_frustum(eye, frames["p_vehicle_quat"], target, hfov, vfov, cam["max_range_m"])
        in_view = angular & ranged
        los = og.line_of_sight(eye, target, center, half, quat, radius=meta["target_radius_m"], rim_count=rim_count)
        fused = ot.flag(frames, ot.FUSED_VALID)
        acquisition = ev.first_true(fused)
        expected_first = record.get("first_obs_fused_valid")
        if (acquisition + 1 if acquisition >= 0 else None) != expected_first:
            problems.append("acquisition frame of env %d episode %d differs from the recorder" % key)
        parity["frames"] += len(frames)
        parity["offline_vs_online_box_disagree"] += int(np.sum((in_view & ~los["center_blocked"]) !=
                                                               ot.flag(frames, ot.GEO_VISIBLE)))
        probe = frames["probe_center_blocked"]
        known = probe >= 0
        parity["probe_frames"] += int(known.sum())
        parity["offline_vs_probe_disagree"] += int(np.sum(known & ((probe == 1) != los["center_blocked"])))
        pixels = frames["probe_visible_pixels"]
        series = {
            "fused_loss": ~fused,
            "center_occlusion_in_view": in_view & los["center_blocked"],
            "geometric_full_in_view": in_view & los["geometric_full_occlusion"],
            "out_of_view": ~in_view,
            "rendered_full_in_view": (in_view & (pixels == 0)) if (aligned and np.all(pixels >= 0)) else None,
        }
        outcome = record["outcome"]
        for name, values in series.items():
            if values is None:
                continue
            rows = ev.episode_intervals(values, frames["frame"], acquisition, terminal, dt, outcome)
            for r in rows:
                r["episode"] = "%d/%d" % key
            intervals[name] += rows
            if name == "center_occlusion_in_view":
                reacq += [ev.reacquisition(fused, frames["frame"], r, terminal, dt) for r in rows]
        exposures.append(ev.exposure_s(acquisition, terminal, dt))
        per_episode.append({"episode": "%d/%d" % key, "exposure_s": exposures[-1],
                            "counts": {name: sum(1 for r in intervals[name] if r["episode"] == "%d/%d" % key)
                                       for name in INDICATORS}})
    out = {"cell": directory.name, "context": meta["context"], "identity": meta["identity"],
           "render_alignment": meta.get("render_alignment"), "quota_episodes": episodes,
           "problems": problems, "parity": parity, "indicators": {}}
    for name in INDICATORS:
        if name == "rendered_full_in_view" and not intervals[name] and not (aligned and parity["probe_frames"]):
            out["indicators"][name] = "NOT_RECORDED"
            continue
        out["indicators"][name] = ev.summarise(intervals[name], exposures)
    eligible = [r for r in reacq if r.get("eligible")]
    out["reacquisition_after_center_occlusion"] = {
        "restored_intervals": len(eligible),
        "reacquired": sum(1 for r in eligible if not r["censored"]),
        "kaplan_meier_time_s": ev.kaplan_meier([r["time_s"] for r in eligible],
                                               [not r["censored"] for r in eligible], (0.5, 1.0, 2.0))}
    out["_intervals"] = intervals
    out["_exposures"] = exposures
    out["_per_episode"] = per_episode
    return out


def analyse_pilot(root, resamples=1000):
    root = Path(root)
    cells = sorted({p.parent for p in root.rglob(ot.SUMMARY_FILE)})
    groups = {}
    per_cell = {}
    for cell in cells:
        result = analyse_cell(cell)
        ctx = result["context"]
        group = "%s/%dbars" % (result["identity"].get("behavior_level"), ctx["bars"])
        groups.setdefault(group, []).append(result)
        per_cell[str(cell.relative_to(root))] = {k: v for k, v in result.items() if not k.startswith("_")}
    out = {"cells": per_cell, "groups": {}}
    for group, results in sorted(groups.items()):
        entry = {"seeds": sorted(r["context"]["seed"] for r in results), "indicators": {}}
        for name in INDICATORS:
            pooled = [i for r in results for i in r["_intervals"][name]]
            exposures = [e for r in results for e in r["_exposures"]]
            if all(r["indicators"].get(name) == "NOT_RECORDED" for r in results):
                entry["indicators"][name] = "NOT_RECORDED"
                continue
            summary = ev.summarise(pooled, exposures)
            # Clusters are episodes (each resets its own scene); seeds are reported separately.
            clusters = {(r["context"]["seed"], e["episode"]): (e["counts"][name], e["exposure_s"])
                        for r in results for e in r["_per_episode"]}
            summary["episode_cluster_bootstrap_rate_per_exposure_s"] = ev.cluster_bootstrap(
                clusters, lambda picked: (sum(c for c, _ in picked) / sum(x for _, x in picked)
                                          if sum(x for _, x in picked) > 0 else None),
                resamples=resamples, seed=0)
            summary["per_seed_rate_per_exposure_s"] = {
                str(r["context"]["seed"]): r["indicators"][name]["rate_per_exposure_s"] for r in results
                if isinstance(r["indicators"][name], dict)}
            entry["indicators"][name] = summary
        out["groups"][group] = entry
    out["note"] = ("exploratory; two seeds give no confirmatory density inference. Rates are per second "
                   "of post-acquisition observation; the bootstrap counts intervals per resampled episode set")
    return out


def run_sweep(cells, synthetic, paths, seed):
    layouts = {}
    if synthetic:
        rng = np.random.default_rng(seed)
        sizes = [(0.45, 0.6), (0.5, 0.8), (0.42, 0.51), (0.6, 0.6)]
        for density in (70, 115, 160, 205):
            layouts[density] = [osw.synthetic_layout(density, rng, sizes) for _ in range(2)]
    else:
        for cell in cells:
            trace = ot.read_trace(cell)
            bars = trace["summary"]["metadata"]["context"]["bars"]
            for index in range(len(trace["layouts"]["hashes"])):
                center, half, _ = ot.layout_arrays(trace["layouts"], index)
                layouts.setdefault(bars, []).append((center, half))
    result = osw.sweep_families(layouts, paths, seed)
    result["layout_source"] = "SYNTHETIC_TEST_LAYOUTS_NOT_EXPERIMENT_SCENES" if synthetic else [str(c) for c in cells]
    return result


def _emit(obj, out):
    text = json.dumps(obj, indent=1, sort_keys=True, default=float) + "\n"
    if out:
        with open(out, "x") as handle:
            handle.write(text)
    else:
        sys.stdout.write(text)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    c = sub.add_parser("cell")
    c.add_argument("--dir", required=True)
    c.add_argument("--out")
    p = sub.add_parser("pilot")
    p.add_argument("--root", required=True)
    p.add_argument("--out")
    s = sub.add_parser("sweep")
    s.add_argument("--cells", nargs="*", default=[])
    s.add_argument("--synthetic", action="store_true")
    s.add_argument("--paths", type=int, default=8)
    s.add_argument("--seed", type=int, default=0)
    s.add_argument("--out")
    args = parser.parse_args(argv)
    if args.command == "cell":
        result = analyse_cell(args.dir)
        _emit({k: v for k, v in result.items() if not k.startswith("_")}, args.out)
    elif args.command == "pilot":
        _emit(analyse_pilot(args.root), args.out)
    else:
        if not args.synthetic and not args.cells:
            parser.error("sweep needs --cells or --synthetic")
        _emit(run_sweep(args.cells, args.synthetic, args.paths, args.seed), args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
