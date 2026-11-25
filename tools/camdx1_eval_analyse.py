"""Analysis of CAM-DX1 (docs/plans/prereg_camonly_dx1_capture_diag_2026-11-25.md): start condition x detection resolution,
S2b-FT at 205 bars, seeds 4501-4503 (one block per seed), 1,024 episodes per cell.

Readings (fixed by the preregistration; descriptive effects, not a design gate). Per effect and metric (capture, crash;
timeout reported) the estimate is the mean over the 3 seed blocks of the paired block difference (in pp):
  START       = mean(V312 - R312, V624 - R624)     weights V312 +1/2, V624 +1/2, R312 -1/2, R624 -1/2
  DET         = mean(R624 - R312, V624 - V312)     weights R624 +1/2, V624 +1/2, R312 -1/2, V312 -1/2
  INTERACTION = (V624 - R624) - (V312 - R312)      weights V624 +1, R624 -1, V312 -1, R312 +1 (reported only, no label)
95 % interval: episode bootstrap, 10,000 resamples, numpy default_rng(4511): every cell's in-quota episodes are resampled
with replacement inside the cell, drawn as one multinomial(n, (capture, crash, timeout) shares) per cell and resample
(exactly the law of the outcome counts of an episode resample), cells in the fixed order R312, R624, V312, V624 x seeds;
every effect is computed on the same resamples; percentile interval (2.5 %, 97.5 %).
Label (capture: the difference; crash: its negative, "gain" = fewer crashes): MATERIAL_GAIN if >= +5 pp and the
interval excludes 0; MATERIAL_LOSS if <= -5 pp and the interval excludes 0; SMALL if the interval excludes 0 and
|difference| < 5 pp; NO_DETECTED_CHANGE otherwise.
If the 624 arms read NOT_RUN_OOM, DET and INTERACTION read NOT_RUN_OOM and START is V312 - R312 only (flagged).
Use of the result (fixed by the preregistration): START capture MATERIAL_GAIN -> the next training targets the
visible-start case; DET capture MATERIAL_GAIN -> 624 x 468 detection; both NO_DETECTED_CHANGE -> the policy.
Per-cell and pooled measures (episode-weighted over the in-quota episodes): capture / crash / timeout; crash classes
(navrl_cams2diag: contact = bar contact, target_contact_swept, out_of_bounds, below, above); never detected (recorder
visible_steps = 0); detected at step 0 (navrl_inview_start start_detected); minimum target distance (ledger
min_relative_distance_m) in bins < 1, 1-2, 2-5, 5-10, >= 10 m by outcome; capture time (episode_steps x rl_step_dt_s)
median and p90 (numpy linear percentile); mean executed command speed and mean vehicle speed (frame-weighted); mean
rendered target pixel area per detected frame, overall and by the frame's horizontal distance (2 m bins), with the
624 / 312 ratios per bin; step-0 bearing shares within +-28.35 and +-33.35 deg; the user's bar (crash <= 20 %, capture
>= 70 %) per arm, not a flight decision.
Analysis controls (run before the eval is read; a miss reads CONTROL_FAIL and no label is given): synthetic ledgers
(identical arms -> START and DET NO_DETECTED_CHANGE; +8 pp capture / -8 pp crash in every V block -> START capture and
crash MATERIAL_GAIN; +2 pp capture in every V block at 20,000 episodes per cell (tight interval) -> SMALL; -8 pp ->
MATERIAL_LOSS) and the recorded CAM-S2a-E pair YC12k - S12k at 205 bars x 4501-4503 (~/motar_tmp/cams2a_eval_20261124/
raw_a1/eval_01/, ledgers bound by SHA-256) -> capture MATERIAL_LOSS and crash MATERIAL_LOSS.
usage: tools/camdx1_eval_analyse.py controls OUT_JSON
       tools/camdx1_eval_analyse.py eval SMOKE_DIR IDENTITY_DIR EVAL_DIR OUT_JSON
"""
import hashlib
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import kroc2027_eval as K                                    # noqa: E402  (read_jsonl)
import camdx1_eval_run as X                                  # noqa: E402  (arms, plan, file names)

BOOT_N = 10000
BOOT_SEED = 4511
MATERIAL_PP = 5.0
OUTCOMES = ("capture", "crash", "timeout")
ARM_ORDER = ("R312", "R624", "V312", "V624")
EFFECTS = {"START": {"V312": 0.5, "V624": 0.5, "R312": -0.5, "R624": -0.5},
           "DET": {"R624": 0.5, "V624": 0.5, "R312": -0.5, "V312": -0.5},
           "INTERACTION": {"V624": 1.0, "R624": -1.0, "V312": -1.0, "R312": 1.0}}
START_312_ONLY = {"V312": 1.0, "R312": -1.0}
RESTRICTED_SUFFIX = "_6_12"
START_6_12 = {"V312": 0.5, "V624": 0.5, "R312_6_12": -0.5, "R624_6_12": -0.5}
START_6_12_312_ONLY = {"V312": 1.0, "R312_6_12": -1.0}
NEAR_MAX_M = 12.0 + 1e-3               # a 6-12 m start (the V range; starts are >= 6 m by construction)
DET_PIXEL_BINS = (3, 4, 5)             # recorder 2 m bins [6, 8), [8, 10), [10, 12)
DET_PIXEL_RATIO_MIN = 3.0              # review r1 P1-2: DET is labelled only if the 624 / 312 pixel area there is >= 3
START_CLASSES = ("in_view_6_12", "out_of_view_6_12", "beyond_12")
LABELLED = ("START", "DET")
MIN_DIST_BINS = ((0.0, 1.0, "<1"), (1.0, 2.0, "1-2"), (2.0, 5.0, "2-5"), (5.0, 10.0, "5-10"), (10.0, float("inf"), ">=10"))
CRASH_CLASSES = ("contact", "target_contact_swept", "out_of_bounds", "below", "above", "uncoded", "unknown")
USER_BAR = {"crash_max": 0.20, "capture_min": 0.70}
S2AE_EVAL = pathlib.Path.home() / "motar_tmp" / "cams2a_eval_20261124" / "raw_a1" / "eval_01"
S2AE_CELL = "S2A_trained__205bars__seed%d"
S2AE_SEEDS = (4501, 4502, 4503)
SYN_N, SYN_N_TIGHT = 1024, 20000


def sha(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


# ---------------------------------------------------------------- the readings (pure)
def counts_from_ledger(rows):
    """{n, capture, crash, timeout} over the in-quota ledger rows."""
    q = [r for r in rows if r.get("in_quota")]
    out = {o: sum(1 for r in q if r.get("outcome") == o) for o in OUTCOMES}
    out["n"] = len(q)
    if sum(out[o] for o in OUTCOMES) != out["n"]:
        raise ValueError("ledger rows outside capture / crash / timeout")
    return out


def rate(c, o):
    return c[o] / c["n"] if c["n"] else float("nan")


def bootstrap(cells, arms, blocks, n_boot=BOOT_N, seed=BOOT_SEED):
    """{(arm, block): array [n_boot, 3] of resampled (capture, crash, timeout) rates}: one multinomial per present cell
    (n > 0) and resample, cells drawn in the fixed order arms x blocks from one numpy default_rng(seed)."""
    rng = np.random.default_rng(seed)
    out = {}
    for a in arms:
        for b in blocks:
            c = (cells.get(a) or {}).get(b)
            if not c or not c["n"]:
                continue
            p = np.array([c[o] for o in OUTCOMES], dtype=float) / c["n"]
            out[(a, b)] = rng.multinomial(c["n"], p, size=n_boot) / float(c["n"])
    return out


def label(diff_pp, ci_pp, metric):
    """The preregistered label: capture on the difference, crash on its negative (a reduction is a gain)."""
    g = diff_pp if metric == "capture" else -diff_pp
    lo, hi = ci_pp
    if not (lo > 0.0 or hi < 0.0):
        return "NO_DETECTED_CHANGE"
    if g >= MATERIAL_PP:
        return "MATERIAL_GAIN"
    if g <= -MATERIAL_PP:
        return "MATERIAL_LOSS"
    return "SMALL"


def complete_blocks(cells, weights, blocks):
    """Review r1 P2-4: the seed blocks in which every arm of the effect has a cell with episodes."""
    return [b for b in blocks if all(((cells.get(a) or {}).get(b) or {}).get("n") for a in weights)]


def effect(cells, weights, blocks, boot, labelled=True):
    """Per metric: the paired block differences, their mean (pp), the bootstrap interval and the label, over the
    complete blocks only; NOT_RUN when no block is complete."""
    blocks = complete_blocks(cells, weights, blocks)
    if not blocks:
        return "NOT_RUN"
    out = {"blocks": list(blocks)}
    for k, o in enumerate(OUTCOMES):
        per_block = [100.0 * sum(w * rate(cells[a][b], o) for a, w in weights.items()) for b in blocks]
        mean = sum(per_block) / len(per_block)
        draws = sum(w * boot[(a, b)][:, k] for a, w in weights.items() for b in blocks) * (100.0 / len(blocks))
        lo, hi = (float(x) for x in np.percentile(draws, [2.5, 97.5]))
        out[o] = {"block_diff_pp": per_block, "mean_pp": mean, "ci95_pp": [lo, hi],
                  "label": label(mean, (lo, hi), o) if labelled and o in ("capture", "crash") else None}
    return out


def _with(name, eff, **extra):
    return dict(eff, **extra) if isinstance(eff, dict) else eff


def readings(cells, blocks, arms_present, restricted=None):
    """START, DET, INTERACTION (and, with `restricted` = {R arm: {block: counts of its 6-12 m starts}}, START_6_12, the
    distance-matched visible-start contrast) on one joint bootstrap: the four arms first, then the restricted R cells."""
    arms = [a for a in ARM_ORDER if a in arms_present]
    allcells = dict(cells)
    extra = []
    for a, by in (restricted or {}).items():
        allcells[a + RESTRICTED_SUFFIX] = by
        extra.append(a + RESTRICTED_SUFFIX)
    boot = bootstrap(allcells, arms + extra, blocks)
    full = set(ARM_ORDER) <= set(arms_present)
    if full:
        out = {name: _with(name, effect(cells, w, blocks, boot, labelled=name in LABELLED), weights=w,
                           reported_only=name not in LABELLED) for name, w in EFFECTS.items()}
        w612 = START_6_12
    else:
        out = {"START": _with("START", effect(cells, START_312_ONLY, blocks, boot), weights=START_312_ONLY,
                              flag="START_312_ONLY"), "DET": "NOT_RUN_OOM", "INTERACTION": "NOT_RUN_OOM"}
        w612 = START_6_12_312_ONLY
    if restricted:
        out["START_6_12"] = _with("START_6_12", effect(allcells, w612, blocks, boot), weights=w612,
                                  note="V arms against the R arms' episodes that started 6-12 m away (distance-matched "
                                       "yaw contrast); primary evidence for 'visible start'")
    if isinstance(out.get("START"), dict):
        out["START"]["reading"] = "visible start + shorter range (6-12 m instead of 6-28 m)"
    return out


def use_of_result(r):
    """The preregistered pointer to the next design (not a gate). Review r1 P1-3: 'visible start' rests on START_6_12;
    the full START effect is 'visible start + shorter range'. DET points only when interpretable (r1 P1-2)."""
    def lab(name):
        e = r.get(name)
        return e.get("capture", {}).get("label") if isinstance(e, dict) else e

    start, start_full, det = lab("START_6_12"), lab("START"), lab("DET")
    out = []
    if start == "MATERIAL_GAIN":
        out.append("START_6_12 capture MATERIAL_GAIN: the next training's evaluation and design target the visible-start "
                   "case; search is reported separately")
    if det == "MATERIAL_GAIN":
        out.append("DET capture MATERIAL_GAIN: the next training uses 624 x 468 detection (memory permitting)")
    if start == "NO_DETECTED_CHANGE" and det == "NO_DETECTED_CHANGE":
        out.append("START_6_12 and DET both NO_DETECTED_CHANGE: the capture loss is not explained by acquisition; the "
                   "next design looks at the policy (curriculum, initialisation)")
    return out or ["no preregistered pointer applies (START_6_12 %s / DET %s; START, visible start + shorter range, %s)"
                   % (start, det, start_full)]


# ---------------------------------------------------------------- controls
def synthetic_rows(n, capture, crash):
    """A synthetic in-quota ledger with round(n x share) captures and crashes, the rest timeouts."""
    nc, nk = int(round(n * capture)), int(round(n * crash))
    outs = ["capture"] * nc + ["crash"] * nk + ["timeout"] * (n - nc - nk)
    return [{"in_quota": True, "outcome": o} for o in outs]


def synthetic_cells(n, v_capture_pp, v_crash_pp, base=(0.50, 0.36), shifted=("V312", "V624")):
    """Every cell at `base`; the `shifted` arms (default both V arms) moved by (+capture, +crash) pp in every block."""
    cells = {}
    for a in ARM_ORDER:
        cells[a] = {}
        for s in X.SEEDS:
            cap, cra = base
            if a in shifted:
                cap, cra = cap + v_capture_pp / 100.0, cra + v_crash_pp / 100.0
            cells[a][s] = counts_from_ledger(synthetic_rows(n, cap, cra))
    return cells


def s2ae_manifest():
    return {str(S2AE_EVAL / arm / (S2AE_CELL % s) / "episodes.jsonl"): None for arm in ("S12k", "YC12k") for s in S2AE_SEEDS}


def controls():
    """The preregistered analysis controls. Returns (all_ok, results)."""
    res = {}
    plan = (("synthetic_identical", SYN_N, 0.0, 0.0, {"START": ("NO_DETECTED_CHANGE", "NO_DETECTED_CHANGE"),
                                                      "DET": ("NO_DETECTED_CHANGE", "NO_DETECTED_CHANGE")}),
            ("synthetic_plus8pp", SYN_N, 8.0, -8.0, {"START": ("MATERIAL_GAIN", "MATERIAL_GAIN"),
                                                     "DET": ("NO_DETECTED_CHANGE", "NO_DETECTED_CHANGE")}),
            ("synthetic_plus2pp_tight", SYN_N_TIGHT, 2.0, -2.0, {"START": ("SMALL", "SMALL")}),
            ("synthetic_minus8pp", SYN_N, -8.0, 8.0, {"START": ("MATERIAL_LOSS", "MATERIAL_LOSS")}),
            ("synthetic_det_plus8pp", SYN_N, 8.0, -8.0, {"DET": ("MATERIAL_GAIN", "MATERIAL_GAIN"),          # r1 P2-6
                                                         "START": ("NO_DETECTED_CHANGE", "NO_DETECTED_CHANGE")}))
    for name, n, dcap, dcra, want in plan:
        cells = synthetic_cells(n, dcap, dcra, shifted=("R624", "V624") if name == "synthetic_det_plus8pp"
                                else ("V312", "V624"))
        r = readings(cells, list(X.SEEDS), ARM_ORDER)
        got = {e: (r[e]["capture"]["label"], r[e]["crash"]["label"]) for e in want}
        res[name] = {"episodes_per_cell": n, "v_shift_pp": {"capture": dcap, "crash": dcra}, "required": want,
                     "labels": got, "start_capture": r["START"]["capture"], "ok": all(got[e] == want[e] for e in want)}
    # the recorded CAM-S2a-E pair YC12k - S12k at 205 bars
    man = s2ae_manifest()
    try:
        cells = {}
        for arm in ("S12k", "YC12k"):
            cells[arm] = {}
            for s in S2AE_SEEDS:
                path = S2AE_EVAL / arm / (S2AE_CELL % s) / "episodes.jsonl"
                man[str(path)] = sha(path)
                cells[arm][s] = counts_from_ledger(K.read_jsonl(path))
        boot = bootstrap(cells, ("S12k", "YC12k"), list(S2AE_SEEDS))
        e = effect(cells, {"YC12k": 1.0, "S12k": -1.0}, list(S2AE_SEEDS), boot)
        got = (e["capture"]["label"], e["crash"]["label"])
        res["cams2a_e_yc12k_minus_s12k_205"] = {"required": ("MATERIAL_LOSS", "MATERIAL_LOSS"), "labels": got,
                                                "effect": e, "manifest": man,
                                                "episodes_per_cell": {a: {s: cells[a][s]["n"] for s in S2AE_SEEDS}
                                                                      for a in cells},
                                                "ok": got == ("MATERIAL_LOSS", "MATERIAL_LOSS")}
    except (OSError, ValueError) as exc:
        res["cams2a_e_yc12k_minus_s12k_205"] = {"ok": False, "error": str(exc), "manifest": man}
    return all(v["ok"] for v in res.values()), res


# ---------------------------------------------------------------- per-cell measures
def _join(rows):
    return {(r.get("env_index"), r.get("env_episode_index")): r for r in rows}


def start_class(bearing_deg, dist_xy_m):
    """in_view_6_12: 6-12 m and |bearing| <= 28.35 deg; out_of_view_6_12: 6-12 m, outside; beyond_12: > 12 m."""
    if dist_xy_m > NEAR_MAX_M:
        return "beyond_12"
    return "in_view_6_12" if abs(bearing_deg) <= float(X.HALF_DEG) + X.BEARING_TOL_DEG else "out_of_view_6_12"


def near_counts(m):
    """{n, capture, crash, timeout} of a cell's 6-12 m starts (both start classes below 12 m)."""
    out = {"n": 0, "capture": 0, "crash": 0, "timeout": 0}
    for c in ("in_view_6_12", "out_of_view_6_12"):
        for k in out:
            out[k] += m["start_classes"][c][k]
    return out


def det_pixel_gate(pooled):
    """Review r1 P1-2: DET is interpretable only if the pooled 624 / 312 mean pixel area per detected frame over the
    6-12 m own-distance bins is >= 3 in both pairs (R624/R312 and V624/V312)."""
    def area(m):
        f = sum(m["bin_frames"][k] for k in DET_PIXEL_BINS)
        return (sum(m["bin_pixels"][k] for k in DET_PIXEL_BINS) / f) if f else None

    out = {}
    for pair, (hi, lo) in (("R", ("R624", "R312")), ("V", ("V624", "V312"))):
        a, b = (area(pooled[x]) if x in pooled else None for x in (hi, lo))
        out[pair] = (a / b) if a and b else None
    out["interpretable"] = all(v is not None and v >= DET_PIXEL_RATIO_MIN for v in (out["R"], out["V"]))
    out["min_ratio"] = DET_PIXEL_RATIO_MIN
    return out


def cell_measures(cell_dir):
    """Every per-cell measure from the ledger, both recorders and result.json (in-quota episodes only). Raw sums are
    kept so that cells pool episode- or frame-weighted."""
    cell_dir = pathlib.Path(cell_dir)
    ledger = K.read_jsonl(cell_dir / "episodes.jsonl")
    diag = _join(K.read_jsonl(cell_dir / X.DIAG_FILE))
    inv = _join(K.read_jsonl(cell_dir / X.INVIEW_FILE))
    dt = float((json.loads((cell_dir / "result.json").read_text()).get("condition") or {}).get("rl_step_dt_s"))
    q = [r for r in ledger if r.get("in_quota")]
    m = {"n": len(q), "counts": {o: 0 for o in OUTCOMES}, "crash_classes": {c: 0 for c in CRASH_CLASSES},
         "never_detected": 0, "detected_at_step0": 0, "step0_recorded": 0,
         "min_dist_bins": {o: {b[2]: 0 for b in MIN_DIST_BINS} for o in OUTCOMES}, "capture_steps": [],
         "exec_speed_sum": 0.0, "robot_speed_sum": 0.0, "frames": 0, "robot_frames": 0,
         "detected_frames": 0, "detected_pixel_sum": 0.0, "bin_frames": [0] * 16, "bin_pixels": [0.0] * 16,
         "step0_within_28_35": 0, "step0_within_33_35": 0, "rl_step_dt_s": dt, "missing_recorder_rows": 0,
         "start_classes": {c: {"n": 0, "capture": 0, "crash": 0, "timeout": 0, "never_detected": 0} for c in START_CLASSES}}
    for r in q:
        key = (r.get("env_index"), r.get("env_episode_index"))
        o = r["outcome"]
        m["counts"][o] += 1
        d = float(r.get("min_relative_distance_m"))
        m["min_dist_bins"][o][next(b[2] for b in MIN_DIST_BINS if b[0] <= d < b[1])] += 1
        dr, ir = diag.get(key), inv.get(key)
        if dr is None or ir is None or dr.get("outcome") != o or ir.get("outcome") != o:
            m["missing_recorder_rows"] += 1
            continue
        if o == "crash":
            c = dr.get("crash_class")
            m["crash_classes"][c if c in m["crash_classes"] else "unknown"] += 1
        if int(dr.get("visible_steps") or 0) == 0:
            m["never_detected"] += 1
        if o == "capture":
            m["capture_steps"].append(int(dr.get("episode_steps")))
        if ir.get("start_detected") is not None:
            m["step0_recorded"] += 1
            m["detected_at_step0"] += int(bool(ir["start_detected"]))
        b, d0 = ir.get(X.RENDER_BEARING), ir.get(X.RENDER_DIST)       # the rendered step-0 pose (review r1 P1-1)
        if b is not None:
            m["step0_within_28_35"] += int(abs(b) <= float(X.HALF_DEG) + X.BEARING_TOL_DEG)
            m["step0_within_33_35"] += int(abs(b) <= X.R_OUTSIDE_DEG)
        if b is not None and d0 is not None:                              # review r1 P1-3: start classes
            sc = m["start_classes"][start_class(b, d0)]
            sc["n"] += 1
            sc[o] += 1
            sc["never_detected"] += int(int(dr.get("visible_steps") or 0) == 0)
        f = int(ir.get("frames") or 0)
        m["frames"] += f
        m["robot_frames"] += f
        m["exec_speed_sum"] += float(ir.get("exec_speed_mean_mps") or 0.0) * f
        m["robot_speed_sum"] += float(ir.get("robot_speed_mean_mps") or 0.0) * f
        m["detected_frames"] += int(ir.get("detected_frames") or 0)
        m["detected_pixel_sum"] += float(ir.get("detected_pixel_sum") or 0.0)
        for k, v in enumerate(ir.get("det_frames_by_dist_bin") or []):
            m["bin_frames"][k] += int(v)
        for k, v in enumerate(ir.get("det_pixels_by_dist_bin") or []):
            m["bin_pixels"][k] += float(v)
    return m


def pool(measures):
    """Sum a list of cell measures (episode- and frame-weighted pooling)."""
    out = json.loads(json.dumps(measures[0]))
    for m in measures[1:]:
        for k in ("n", "never_detected", "detected_at_step0", "step0_recorded", "exec_speed_sum", "robot_speed_sum",
                  "frames", "robot_frames", "detected_frames", "detected_pixel_sum", "step0_within_28_35",
                  "step0_within_33_35", "missing_recorder_rows"):
            out[k] += m[k]
        for k in ("counts", "crash_classes"):
            for c in out[k]:
                out[k][c] += m[k][c]
        for o in OUTCOMES:
            for b in out["min_dist_bins"][o]:
                out["min_dist_bins"][o][b] += m["min_dist_bins"][o][b]
        out["capture_steps"] += m["capture_steps"]
        for c in START_CLASSES:
            for k in out["start_classes"][c]:
                out["start_classes"][c][k] += m["start_classes"][c][k]
        out["bin_frames"] = [a + b for a, b in zip(out["bin_frames"], m["bin_frames"])]
        out["bin_pixels"] = [a + b for a, b in zip(out["bin_pixels"], m["bin_pixels"])]
    return out


def summarise(m):
    """Shares and means of pooled or single-cell measures."""
    n = m["n"]
    steps = np.array(m["capture_steps"], dtype=float) * m["rl_step_dt_s"]
    crashes = m["counts"]["crash"]
    return {"n": n, "rates": {o: m["counts"][o] / n for o in OUTCOMES},
            "crash_class_share_of_crashes": {c: (v / crashes if crashes else None) for c, v in m["crash_classes"].items()},
            "crash_class_share_of_episodes": {c: v / n for c, v in m["crash_classes"].items()},
            "never_detected_share": m["never_detected"] / n,
            "detected_at_step0_share": (m["detected_at_step0"] / m["step0_recorded"]) if m["step0_recorded"] else None,
            "step0_bearing_within_28_35_share": m["step0_within_28_35"] / n,
            "step0_bearing_within_33_35_share": m["step0_within_33_35"] / n,
            "min_dist_bins_by_outcome": m["min_dist_bins"],
            "capture_time_s": {"n": int(steps.size), "median": float(np.percentile(steps, 50)) if steps.size else None,
                               "p90": float(np.percentile(steps, 90)) if steps.size else None},
            "mean_exec_command_speed_mps": m["exec_speed_sum"] / m["frames"] if m["frames"] else None,
            "mean_vehicle_speed_mps": m["robot_speed_sum"] / m["robot_frames"] if m["robot_frames"] else None,
            "mean_target_pixel_area_per_detected_frame": (m["detected_pixel_sum"] / m["detected_frames"]
                                                          if m["detected_frames"] else None),
            "detected_frame_share": m["detected_frames"] / m["frames"] if m["frames"] else None,
            "pixel_area_by_dist_bin": [(p / f if f else None) for f, p in zip(m["bin_frames"], m["bin_pixels"])],
            "detected_frames_by_dist_bin": m["bin_frames"], "missing_recorder_rows": m["missing_recorder_rows"],
            "start_classes": {c: dict(v, **({"rates": {o: v[o] / v["n"] for o in OUTCOMES},
                                               "never_detected_share": v["never_detected"] / v["n"]} if v["n"] else {}))
                              for c, v in m["start_classes"].items()},
            "user_bar": {"crash_le_20pct": m["counts"]["crash"] / n <= USER_BAR["crash_max"],
                         "capture_ge_70pct": m["counts"]["capture"] / n >= USER_BAR["capture_min"]}}


def pixel_ratios(hi, lo):
    """624 / 312 mean pixel area per detected frame per 2 m distance bin (bins with frames in both)."""
    out = []
    for k, (a, b) in enumerate(zip(hi["pixel_area_by_dist_bin"], lo["pixel_area_by_dist_bin"])):
        out.append({"bin_m": [k * 2.0, (k + 1) * 2.0 if k < 15 else None], "ratio": (a / b) if a and b else None})
    return out


# ---------------------------------------------------------------- bindings and the eval
def load_reports(smoke_dir, identity_dir, eval_dir):
    """The three stage reports and their binding problems."""
    p = []
    reps = {}
    for name, d, f, want in (("smoke", smoke_dir, "smoke_report.json", ("SMOKE_OK",)),
                             ("identity", identity_dir, "identity_report.json", ("IDENTITY_PASS",)),
                             ("eval", eval_dir, "eval_run_report.json",
                              ("COMPLETE", "COMPLETE_624_NOT_RUN_OOM", "INCOMPLETE"))):
        try:
            reps[name] = json.loads((pathlib.Path(d) / f).read_text())
        except (OSError, ValueError) as e:
            p.append("%s report unreadable: %s" % (name, e))
            continue
        if reps[name].get("unit") != X.UNIT or reps[name].get("verdict") not in want:
            p.append("%s report is not %s %s" % (name, X.UNIT, "/".join(want)))
    commits = {r.get("commit") for r in reps.values()}
    if len(commits) != 1:
        p.append("stage reports name different commits %s" % sorted(map(str, commits)))
    ckpts = {r.get("ft_checkpoint_sha256") for r in reps.values()}
    if ckpts != {X.FT_CKPT_SHA}:
        p.append("stage reports name checkpoints %s, not %s" % (sorted(map(str, ckpts)), X.FT_CKPT_SHA[:12]))
    return reps, p


def eval_cells(ev):
    """{arm: {seed: cell_dir}} of the final OK record of every planned cell (624 in the final memory mode)."""
    mem = ev.get("mem_624", "E128")
    final_envs = X.FALLBACK_NUM_ENVS if mem == "E64" else X.NUM_ENVS
    incomplete = ev.get("verdict") == "INCOMPLETE"      # r1 P2-4: readings from complete seed blocks only
    out, p = {}, []
    for arm, d, s in X.eval_plan():
        if X.res(arm) == 624 and mem == "NOT_RUN_OOM":
            continue
        recs = [r for r in ev.get("records") or [] if (r.get("arm"), r.get("density"), r.get("seed")) == (arm, d, s)
                and r.get("verdict") == "OK" and (X.res(arm) == 312 or r.get("num_envs") == final_envs)]
        if len(recs) != 1:
            if not (incomplete and not recs):
                p.append("%s/%d: %d OK records in the final memory mode" % (arm, s, len(recs)))
            continue
        r = recs[0]
        cell = pathlib.Path(r["cell_dir"])
        want = (r.get("stats") or {}).get("result_sha256")
        if r.get("commit") != ev.get("commit") or not want or not (cell / "result.json").is_file() \
                or sha(cell / "result.json") != want:
            p.append("%s/%d: record commit or result.json SHA-256 does not bind" % (arm, s))
            continue
        out.setdefault(arm, {})[s] = cell
    return out, p, mem


def main_eval(smoke_dir, identity_dir, eval_dir, out):
    ok, ctrl = controls()
    reps, p = load_reports(smoke_dir, identity_dir, eval_dir)
    res = {"unit": X.UNIT, "prereg": X.PREREG, "analysis_sha256": sha(__file__), "controls": ctrl, "controls_ok": ok,
           "binding_problems": p}
    if "eval" not in reps:
        res["verdict"] = "EVAL_BINDING_FAIL"
        return write(out, res)
    cells_dir, cp, mem = eval_cells(reps["eval"])
    p += cp
    res["mem_624"] = mem
    res["smoke_controls"] = {k: reps.get("smoke", {}).get(k) for k in ("visible_start_control", "smoke_624_control")}
    measures = {a: {s: cell_measures(cells_dir[a][s]) for s in cells_dir[a]} for a in cells_dir}
    for a in measures:
        for s, m in measures[a].items():
            if m["n"] != 1024 or m["missing_recorder_rows"]:
                p.append("%s/%d: %d in-quota episodes, %d without recorder rows" % (a, s, m["n"], m["missing_recorder_rows"]))
    res["cells"] = {a: {str(s): summarise(m) for s, m in measures[a].items()} for a in measures}
    pooled = {a: pool([measures[a][s] for s in sorted(measures[a])]) for a in measures}
    res["pooled"] = {a: summarise(m) for a, m in pooled.items()}
    if "R624" in res["pooled"]:
        res["pixel_ratio_624_over_312"] = {"R": pixel_ratios(res["pooled"]["R624"], res["pooled"]["R312"]),
                                           "V": pixel_ratios(res["pooled"]["V624"], res["pooled"]["V312"])}
    best = max(res["pooled"], key=lambda a: (res["pooled"][a]["rates"]["capture"], -res["pooled"][a]["rates"]["crash"]))
    res["best_cell_vs_user_bar"] = {"arm": best, "rates": res["pooled"][best]["rates"], "bar": USER_BAR,
                                    "meets": res["pooled"][best]["user_bar"],
                                    "note": "not a flight decision: one policy, one density, no memory grid, no backstop"}
    if p:
        res["verdict"] = "EVAL_BINDING_FAIL"
        return write(out, res)
    counts = {a: {s: {o: measures[a][s]["counts"][o] for o in OUTCOMES} for s in measures[a]} for a in measures}
    for a in counts:
        for s in counts[a]:
            counts[a][s]["n"] = measures[a][s]["n"]
    restricted = {a: {s: near_counts(measures[a][s]) for s in measures[a]} for a in ("R312", "R624") if a in measures}
    r = readings(counts, list(X.SEEDS), set(counts), restricted)
    gate = det_pixel_gate(pooled)
    res["det_pixel_gate"] = gate
    if isinstance(r.get("DET"), dict) and not gate["interpretable"]:
        for o in ("capture", "crash"):
            r["DET"][o]["label"] = "NOT_INTERPRETABLE"
    if not ok:
        for e in r.values():
            if isinstance(e, dict):
                for o in ("capture", "crash"):
                    e[o]["label"] = "CONTROL_FAIL"
    res["readings"] = r
    res["eval_verdict"] = reps["eval"].get("verdict")
    res["use_of_result"] = use_of_result(r) if ok else ["CONTROL_FAIL: no label is given"]
    res["verdict"] = "ANALYSED" if ok else "CONTROL_FAIL"
    return write(out, res)


def write(out, res):
    pathlib.Path(out).write_text(json.dumps(res, indent=1, default=str) + "\n")
    print(json.dumps({k: res.get(k) for k in ("verdict", "controls_ok", "mem_624")}, indent=1))
    return 0 if res.get("verdict") == "ANALYSED" else 1


def main_controls(out):
    ok, ctrl = controls()
    pathlib.Path(out).write_text(json.dumps({"controls_ok": ok, "controls": ctrl, "analysis_sha256": sha(__file__)},
                                            indent=1, default=str) + "\n")
    print(json.dumps({k: (v["labels"], v["ok"]) if "labels" in v else v.get("error") for k, v in ctrl.items()}, indent=1,
                     default=str))
    return 0 if ok else 1


def main(argv=None):
    argv = list(argv if argv is not None else sys.argv[1:])
    if len(argv) == 2 and argv[0] == "controls":
        return main_controls(argv[1])
    if len(argv) == 5 and argv[0] == "eval":
        return main_eval(*argv[1:])
    print(__doc__.strip().splitlines()[-2:])
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
