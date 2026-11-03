"""Focused F6 CPU controls; this is not the historical R2 gate or a qualification.

Usage: check_r1_temporal.py NEW_OUTPUT.json [--layout NAME] [--height Z]
P0R_SRC selects the actual core.  The independent harness casts every packet;
no map evidence, static flag, or certificate result is injected by this test.

A thin noncatalog pole stays still for 22 packets, then moves aside. A fresh
track belongs to a separate, real target. The former pole's HIT cell therefore
can become STATIC with the track, then CONFLICT under later truthful free rays.
Without the track, K_FREE later packets resolve that occupancy episode and
restore a valid certificate through the former pole position. A second path
must retain accepted certificates in both cores. The camera deliberately labels
the pole as a target (a false positive); its labels must actually activate while
all queried certificate dictionaries remain identical to the no-camera core.

The default is six deterministic sequences, two layouts x three altitudes. This
adds altitude and nonzero pose/yaw coverage without changing old R2 rules/data.
All clearance classification calls receive the actual vehicle altitude.
"""
import argparse
import collections
import hashlib
import json
import math
import pathlib
import time

import numpy as np

from harness import CATALOG, DT, SRC, bar, camera, classify, core, header, pole, scan, target
from evidence_map import D_SEP, K_FREE, N_STATIC, T_STATIC, WIDTH


LAYOUTS = ("P0R-72x1", "MOTAR-72x4")
HEIGHTS = (0.05, 1.35, 2.95)
STEPS = 26
LAST_STATIONARY_STEP = 22
# Exercise occupancy before/at/after the static threshold and both free packets.
CERTIFY_STEPS = (1, 20, 21, 22, 23, 24, 26)


def sha256(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def source_hashes():
    paths = list(pathlib.Path(SRC).glob("*.py"))
    paths.extend((pathlib.Path(__file__), pathlib.Path(__file__).with_name("harness.py")))
    return {str(p.resolve()): sha256(p) for p in sorted(paths)}


def sequence(layout, z, case_index):
    """Use public core ingestion/certification/reporting; map arrays are read only diagnostics."""
    yaw = 0.23 * case_index
    cs, sn = math.cos(yaw), math.sin(yaw)
    # Put the known horizontal pole return at a cell centre, even with rotation.
    # This makes the later ray genuinely traverse the cell's central half.
    anchor = (1.475 + 0.15 * case_index, 0.025 + 0.10 * case_index)
    pose = (anchor[0] - 1.45 * cs, anchor[1] - 1.45 * sn, z, yaw)

    def world(x, y):
        return (pose[0] + cs * x - sn * y, pose[1] + sn * x + cs * y)

    initial_pole = world(1.5, 0.0)
    final_pole = world(1.5, 3.0)
    target_xy = world(-3.0, 3.0)
    target_world = (*target_xy, z)
    bar_xy = world(4.0, 4.0)
    fixed = [target(*target_world), bar(*bar_xy, *CATALOG[0])]
    paths = np.array([[world(0.0, 0.0), world(2.0, 0.0)],
                      [world(0.0, 0.0), world(-1.5, 0.0)]], dtype=np.float64)
    cfg = "SYNTH-72x1" if layout == LAYOUTS[0] else "SYNTH-72x4"
    cores = {name: core.CertifierCore(cfg, 0, 0)
             for name in ("baseline", "camera", "track", "track_camera")}
    probe = np.array([anchor], dtype=np.float64)
    stats = dict(steps=0, complete_accepted_packets=0, camera_labelled_returns=0,
                 baseline_accepted_certificates=0, baseline_valid_certificates=0,
                 track_accepted_certificates=0, track_gain=0, track_loss=0,
                 conflict_caused_loss_after_k_free=0, restored_baseline_paths=0,
                 retained_track_control_certificates=0, camera_certificate_differences=0,
                 baseline_static_peak=0, track_static_peak=0, track_conflict_peak=0,
                 internal_exceptions=0)
    reasons = collections.Counter()
    classes = collections.Counter()
    rows = []
    certificates = []
    witness = None
    first_static_step = None
    first_conflict_step = None
    truthful_pole_hit = False
    for step in range(1, STEPS + 1):
        pole_xy = initial_pole if step <= LAST_STATIONARY_STEP else final_pole
        scene = [pole(*pole_xy, 0.05)] + fixed
        ranges, status, owners = scan(scene, pose, layout)
        if step == 1:
            # The 0-degree horizontal beam really strikes the pole at 1.45 m.
            truthful_pole_hit = bool(np.any((owners == 0) & (status == core.HIT)
                                            & np.isclose(ranges, 1.45, rtol=0.0, atol=1e-10)))
        t = float(step * DT)
        cam = camera(t, pose, (*pole_xy, z), sigma_r=0.05, sigma_b=0.005)
        trk = np.array([1.0, t, *target_xy, 0.20], dtype=np.float64)
        reports = {}
        results = {}
        for name, certifier in cores.items():
            reports[name] = certifier.step(ranges, status, *header(step, pose),
                                           camera=cam if "camera" in name else None,
                                           track=trk if name.startswith("track") else None)
            if step in CERTIFY_STEPS:
                results[name] = certifier.certify(paths, step)
        stats["steps"] += 1
        stats["complete_accepted_packets"] += sum(
            int(rep.get("accepted", False) and rep.get("complete", False)) for rep in reports.values())
        stats["camera_labelled_returns"] += sum(
            reports[name].get("labelled_returns", 0) for name in ("camera", "track_camera"))

        baseline, tracked = cores["baseline"], cores["track"]
        # No assignment to any core/map field occurs: these are actual diagnostics.
        static_n = int(np.count_nonzero(tracked.map.static))
        conflict_n = int(np.count_nonzero(tracked.map.conflict))
        baseline_static_n = int(np.count_nonzero(baseline.map.static))
        stats["baseline_static_peak"] = max(stats["baseline_static_peak"], baseline_static_n)
        stats["track_static_peak"] = max(stats["track_static_peak"], static_n)
        stats["track_conflict_peak"] = max(stats["track_conflict_peak"], conflict_n)
        states = {name: certifier.map_states(probe, t)[0] for name, certifier in cores.items()}
        if states["track"] == "STATIC_OCCUPANCY" and first_static_step is None:
            first_static_step = step
            ij = np.floor((probe[0] - tracked.map.lo) / WIDTH).astype(np.int64)
            cell_id = int(ij[0] * tracked.map.n + ij[1])
            hits = tracked.map.pending[cell_id]
            witness = dict(cell_id=cell_id, cell_probe_world=anchor,
                           distinct_hit_packets=len({h[0] for h in hits}),
                           first_hit_step=min(h[0] for h in hits), last_hit_step=max(h[0] for h in hits),
                           hit_span_s=max(h[1] for h in hits) - min(h[1] for h in hits),
                           all_tracks_fresh_active=all(h[2] for h in hits),
                           min_track_separation_after_radius_m=min(h[3] for h in hits))
        if states["track"] == "CONFLICT" and first_conflict_step is None:
            first_conflict_step = step
        rows.append(dict(step=step, t=t, pole_world=pole_xy, pole_cell_states=states,
                         baseline_static_cells=baseline_static_n, track_static_cells=static_n,
                         track_conflict_cells=conflict_n,
                         accepted_complete=all(r.get("accepted") and r.get("complete") for r in reports.values())))

        if step not in CERTIFY_STEPS:
            continue
        stats["camera_certificate_differences"] += int(results["baseline"] != results["camera"])
        stats["camera_certificate_differences"] += int(results["track"] != results["track_camera"])
        for index, (path, base_result, track_result) in enumerate(zip(paths, results["baseline"], results["track"])):
            b, tr = base_result["static_bar_certified"], track_result["static_bar_certified"]
            # The oracle uses the actual z; its default z=1 is never used here.
            classification = classify(base_result, path, scene, z_vehicle=z)
            track_class = classify(track_result, path, scene, z_vehicle=z)
            classes["baseline:" + classification] += 1
            classes["track:" + track_class] += 1
            stats["baseline_accepted_certificates"] += int(b)
            stats["baseline_valid_certificates"] += int(b and classification == "VALID")
            stats["track_accepted_certificates"] += int(tr)
            stats["track_gain"] += int(not b and tr)
            stats["track_loss"] += int(b and not tr)
            reasons["baseline:" + base_result["reason"]] += 1
            reasons["track:" + track_result["reason"]] += 1
            if step >= LAST_STATIONARY_STEP + K_FREE and index == 0:
                stats["restored_baseline_paths"] += int(b and classification == "VALID")
                stats["conflict_caused_loss_after_k_free"] += int(
                    b and not tr and track_result["reason"] == "MAP_VETO_CONFLICT")
            if step >= LAST_STATIONARY_STEP + K_FREE and index == 1:
                stats["retained_track_control_certificates"] += int(b and tr and track_class == "VALID")
        certificates.append(dict(step=step, results=results))

    stats["internal_exceptions"] = sum(len(c.exceptions) for c in cores.values())
    final_states = rows[-1]["pole_cell_states"]
    checks = {
        "schedule_has_at_least_24_packets": STEPS >= 24,
        "all_packets_truthfully_cast_and_complete_accepted": (
            truthful_pole_hit and stats["complete_accepted_packets"] == 4 * STEPS),
        "static_rule_actually_crossed": (
            witness is not None and witness["distinct_hit_packets"] >= N_STATIC
            and witness["hit_span_s"] >= T_STATIC and witness["all_tracks_fresh_active"]
            and witness["min_track_separation_after_radius_m"] >= D_SEP
            and first_static_step <= LAST_STATIONARY_STEP
            and stats["track_static_peak"] > 0 and stats["baseline_static_peak"] == 0),
        "later_free_packet_creates_conflict": (
            first_conflict_step is not None and first_conflict_step > LAST_STATIONARY_STEP
            and stats["track_conflict_peak"] > 0 and final_states["track"] == "CONFLICT"),
        "baseline_resolves_old_occupancy_and_certifies": (
            final_states["baseline"] == "VALIDATED_FREE" and stats["restored_baseline_paths"] > 0
            and stats["baseline_valid_certificates"] > 0),
        "track_rejection_has_positive_control": (
            stats["track_loss"] > 0 and stats["conflict_caused_loss_after_k_free"] > 0
            and stats["retained_track_control_certificates"] > 0),
        "track_never_adds_certificate_in_queried_pairs": stats["track_gain"] == 0 and stats["track_loss"] > 0,
        "active_camera_labels_preserve_certificates": (
            stats["camera_labelled_returns"] > 0 and stats["camera_certificate_differences"] == 0),
        "oracle_uses_vehicle_altitude_and_accepts_valid_cases": (
            stats["baseline_valid_certificates"] > 0
            and classes["baseline:FALSE_CERT"] == 0 and classes["track:FALSE_CERT"] == 0),
        "no_internal_exceptions": stats["internal_exceptions"] == 0,
    }
    return dict(layout=layout, z_vehicle=z, pose=pose, paths=paths.tolist(),
                track_target_world=target_world, initial_pole_world=initial_pole,
                final_pole_world=final_pole, first_static_step=first_static_step,
                first_conflict_step=first_conflict_step, static_witness=witness,
                stats=stats, checks=checks, focused_checks_pass=all(checks.values()),
                classes=dict(classes), reasons=dict(reasons), packet_diagnostics=rows,
                certificate_queries=certificates)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=pathlib.Path)
    parser.add_argument("--layout", choices=LAYOUTS, action="append")
    parser.add_argument("--height", type=float, choices=HEIGHTS, action="append")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists; choose a new evidence path")
    start = time.monotonic()
    before = source_hashes()
    cases = []
    for layout in args.layout or LAYOUTS:
        for z in args.height or HEIGHTS:
            # Keep case geometry stable when selecting a development subset.
            case_index = LAYOUTS.index(layout) * len(HEIGHTS) + HEIGHTS.index(z)
            cases.append(sequence(layout, z, case_index))
    after = source_hashes()
    result = dict(scope="FOCUSED_F6_CPU_CONTROL_ONLY", qualification="NOT_QUALIFIED",
                  historical_r2="UNCHANGED_NOT_RERUN", source_dir=str(pathlib.Path(SRC).resolve()),
                  source_sha256_before=before, source_sha256_after=after,
                  source_unchanged_during_run=before == after,
                  rules=dict(steps=STEPS, dt_s=DT, last_stationary_step=LAST_STATIONARY_STEP,
                             certify_steps=CERTIFY_STEPS, n_static=N_STATIC, t_static_s=T_STATIC,
                             k_free=K_FREE, d_sep_m=D_SEP,
                             camera="FALSE_POSITIVE_TARGET_LABEL_ON_NONCATALOG_POLE"),
                  cases=cases, focused_checks_pass=before == after and all(c["focused_checks_pass"] for c in cases),
                  wall_s=time.monotonic() - start)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as handle:
        json.dump(result, handle, indent=2)
        handle.write("\n")
    print(json.dumps({"scope": result["scope"], "qualification": result["qualification"],
                      "focused_checks_pass": result["focused_checks_pass"],
                      "source_unchanged_during_run": before == after, "wall_s": result["wall_s"],
                      "cases": [{k: c[k] for k in ("layout", "z_vehicle", "stats", "checks")}
                                for c in cases]}, indent=2))
    return 0 if result["focused_checks_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
