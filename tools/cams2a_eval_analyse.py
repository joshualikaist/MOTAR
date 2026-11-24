"""Analysis of CAM-S2a-E (DRAFT r2 for docs/plans/prereg_camonly_s2a_eval_2026-11-24.md; to live at tools/cams2a_eval_analyse.py;
design review r1 of the draft applied).

Blocks are (density, seed) pairs: 3 densities x 3 seeds = 9 blocks per gated arm; rates are per-block quota rates
(gr_analyse.rates: the first QUOTA episodes of each env). Intervals as CAM-K2-E (tools/camk2e_analyse.py), with the
degrees of freedom of this grid: a stratified percentile bootstrap (10,000 resamples, seed 20,261,124, seeds resampled
within density; camk1e_analyse.strat_ci) and a stratified t-interval with df = sum_d (n_d - 1) = 6. Contrasts in pp; a
label needs both intervals to agree. With 3 seeds per stratum the bootstrap has at most 10 distinct resample means per
stratum, so the t-interval is the operative guard (review P3-3).

Crash contrasts use CAM-K2-E's gate-A rule with the labels of CAM-K2-E3 (MATERIAL_REDUCTION both upper ends < -5 pp;
SMALL_REDUCTION both upper ends < 0 and mean <= -2; WORSE both lower ends > 0 and mean >= +2; NEGLIGIBLE both intervals
inside (-2, +2); UNRESOLVED when the intervals disagree on the side; otherwise NO_DETECTED_CHANGE; a reduction with
timeouts up >= 2 pp reads CRASH_TO_TIMEOUT). Capture and timeout contrasts use C2's scheme (camonly_c2_analyse.label;
capture band 3 pp, timeout band 2 pp: LOWER / SMALL_LOWER / HIGHER / SMALL_HIGHER / NEGLIGIBLE / NO_DETECTED_CHANGE /
UNRESOLVED).

Gated (stage A arms only: S1k, S12k, YC12k on the 9 blocks):
  L  (the final checkpoint against the earliest kept one, in the final training environment)  S12k - S1k:
        capture HIGHER and crash not in {WORSE, UNRESOLVED} -> LEARNING_SUPPORTED; capture LOWER or crash WORSE ->
        LEARNING_REFUTED; otherwise LEARNING_NOT_DETECTED. (S1k trained only at 70 bars with the FOV curriculum a third
        through, so LEARNING_SUPPORTED is the expected pipeline-sanity outcome, not a discovery; review P2-4.)
  Y  (yaw clip at 1.047 rad/s)  YC12k - S12k. Preconditions (review P1-2): every YC12k cell has [yawclip] telemetry
        (else YAWCLIP_INACTIVE), and the clip's BINDING SHARE, the share of control steps whose requested yaw rate
        exceeds 1.047 rad/s pooled over the YC12k cells' recorder rows (exact, whole cell), is >= MIN_BINDING_SHARE
        (1 %), else YAWCLIP_WEAKLY_BINDING (no tolerance label). Then crash WORSE or capture LOWER -> YAWCLIP_SENSITIVE;
        crash in {NEGLIGIBLE, SMALL_REDUCTION, MATERIAL_REDUCTION} and capture in {NEGLIGIBLE, SMALL_HIGHER, HIGHER,
        SMALL_LOWER} -> YAWCLIP_TOLERATED (NOTE: TOLERATED admits a capture loss whose intervals exclude 0 but whose
        mean is under 3 pp, the SMALL_LOWER label); otherwise YAWCLIP_UNRESOLVED. The binding share is reported for the
        S12k cells too (the same quantity where the clip is absent), with the whole-cell [yawclip] clipped share.
Reported, not gated: S3k - S1k and S6k - S3k on the 175-bar blocks only (stage B; 3 blocks, no interval) and, when
stage C ran, on every block they share; pooled and per-density rates per arm; crash split (recorder rows:
target_contact_swept / contact / below / above / out_of_bounds; result.json's crash_causes are reported as "excludes
target contacts", training review F6); crash per 1,000 steps from the ledger's episode lengths (completion_step
differences per env, in-quota episodes); capture kinematics (relative speed, closing, lateral: mean, p50, p90, share
above 1.0 m/s; training review F7); blind behaviour; [camsafe] and [yawclip] telemetry (last line = running total through
its call count); the speed-governor summary; the delay distribution.
Controls (computed before the run on CAM-K1-E's cells at 115/160/205 bars x 3 seeds; any miss reads CONTROL_FAIL and no
gate label is given): on the full 2,048-episode cells the crash rule must read WORSE on F66 - K1s1, not MATERIAL_REDUCTION
or WORSE on K1s1 - K1s2 and MATERIAL_REDUCTION on K1s1 - F66, and the capture scheme HIGHER on K1s1 - F66; at THIS unit's
scale (review P2-5) the same cells truncated to the first 8 episodes per env (1,024 per cell): the K1s1 - K1s2 null must
not read MATERIAL_REDUCTION or WORSE (its half-widths are this unit's resolution); the same null with +5 pp crash (-5 pp
capture) added to K1s1 in every block must read WORSE (LOWER): the labels are reachable at this resolution; reported, not
required (review r2 P1-6): the +3 pp and +2 pp CONTRASTS (K1s1 shifted by pp minus the quota-8 null's mean) with their labels;
the cells must be at CAM-K1-E's manifest.
Bindings (EVAL_BINDING_FAIL otherwise): identity IDENTITY_PASS on the eval commit and the report the eval named; eval
COMPLETE with stages A and B in the planned order; same runner environment and training receipt; every cell passes the
runner's checks again with its checkpoint SHA; complete 9-block grids for S1k, S12k, YC12k and complete 175-bar blocks for
S3k, S6k; in every block S12k differs from S1k and YC12k from S12k; the identity cells S12k_identity, S1k_smoke and
YC12k_negative equal the eval's S12k, S1k and YC12k cells at 205 / 4501; the recorder's episode lengths equal the ledger's;
in every YC12k cell the recorder's executed-yaw-over-limit count is 0; the training receipt at the recorded SHA; the
checkpoints at their pins; this script's bytes equal to the eval commit's.
usage: tools/cams2a_eval_analyse.py controls CAMK1E_RAW OUT_JSON
       tools/cams2a_eval_analyse.py eval CAMK1E_RAW IDENTITY_DIR EVAL_DIR OUT_JSON
"""
import json
import math
import pathlib
import random
import statistics
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import camk1e_analyse as E1                                  # noqa: E402  (strat_ci, manifest_ok, file_matches_commit)
import camonly_c2_analyse as C2A                             # noqa: E402  (side, label, governor)
import gr_analyse as A                                       # noqa: E402  (rates, BOOT)
import kroc2027_eval as K                                    # noqa: E402  (read_jsonl, comparable)
import cams2a_eval_run as V                                  # noqa: E402  (arms, plan, cell checks, checkpoints)

ROOT = pathlib.Path(__file__).resolve().parents[1]
BOOT_SEED = 20261124
T_CRIT = {4: 2.776445105, 6: 2.446911851, 8: 2.306004135}   # t 0.975 quantiles
A_BAND, FLOOR, CAP_BAND, TO_BAND = 5.0, 2.0, 3.0, 2.0
MIN_BINDING_SHARE = 0.01                                     # review P1-2: the clip must bind on >= 1 % of control steps
SHIFT_PP = 5.0                                               # review P2-5: the required shifted-null control adds +5 pp per block
CONTRAST_REPORTED_PP = (3.0, 2.0)                            # review r2 P1-6: reported contrasts of +3 pp (claim scale) and +2 pp (the
                                                             # floor): the shift is pp minus the quota-8 null's mean
METRICS = ("crash", "capture", "timeout")
CELL = V.CELL
K1E_CELL = "E2_obstacle_aware__%dbars__seed%d"
K1E_DENSITIES = (115, 160, 205)
CAMK1E_MANIFEST = ROOT / "docs/evidence/camonly_2026-11-04/camk1e/SHA256_MANIFEST.json"
GATED_L = "S12k_minus_S1k"
GATED_Y = "YC12k_minus_S12k"
CONTRASTS = {GATED_L: {"S12k": 1, "S1k": -1}, GATED_Y: {"YC12k": 1, "S12k": -1}}
DESCRIPTIVE = {"S3k_minus_S1k": {"S3k": 1, "S1k": -1}, "S6k_minus_S3k": {"S6k": 1, "S3k": -1}, "S12k_minus_S6k": {"S12k": 1, "S6k": -1}}


def both(boot, t, test):
    return test(boot) and test(t)


def crash_label(boot, t, mean, timeout_mean=0.0):
    """CAM-K2-E's gate-A rule (tools/camk2e_analyse.gate_a) with CAM-K2-E3's labels."""
    if C2A.side(boot) != C2A.side(t):
        return "UNRESOLVED"
    if both(boot, t, lambda ci: ci[1] < -A_BAND):
        return "CRASH_TO_TIMEOUT" if timeout_mean >= FLOOR else "MATERIAL_REDUCTION"
    if both(boot, t, lambda ci: ci[1] < 0) and mean <= -FLOOR:
        return "CRASH_TO_TIMEOUT" if timeout_mean >= FLOOR else "SMALL_REDUCTION"
    if both(boot, t, lambda ci: ci[0] > 0) and mean >= FLOOR:
        return "WORSE"
    if both(boot, t, lambda ci: -FLOOR < ci[0] and ci[1] < FLOOR):
        return "NEGLIGIBLE"
    return "NO_DETECTED_CHANGE"


def gate_l(crash, capture):
    if capture == "HIGHER" and crash not in ("WORSE", "UNRESOLVED"):
        return "LEARNING_SUPPORTED"
    if capture == "LOWER" or crash == "WORSE":
        return "LEARNING_REFUTED"
    return "LEARNING_NOT_DETECTED"


def gate_y(crash, capture, telemetry_present, binding_share):
    if not telemetry_present:
        return "YAWCLIP_INACTIVE"
    if binding_share is None or binding_share < MIN_BINDING_SHARE:
        return "YAWCLIP_WEAKLY_BINDING"
    if crash == "WORSE" or capture == "LOWER":
        return "YAWCLIP_SENSITIVE"
    if crash in ("NEGLIGIBLE", "SMALL_REDUCTION", "MATERIAL_REDUCTION") and \
            capture in ("NEGLIGIBLE", "SMALL_HIGHER", "HIGHER", "SMALL_LOWER"):
        return "YAWCLIP_TOLERATED"
    return "YAWCLIP_UNRESOLVED"


def strat_t(xs, blocks):
    """Stratified t-interval over equal-size strata, df = sum_d (n_d - 1) (6 for 9 blocks)."""
    by = {}
    for x, (d, _s) in zip(xs, blocks):
        by.setdefault(d, []).append(x)
    groups = [by[d] for d in sorted(by)]
    df = sum(len(g) - 1 for g in groups)
    m = sum(xs) / len(xs)
    se = math.sqrt(sum(statistics.variance(g) / len(g) for g in groups) / len(groups) ** 2)
    return [m - T_CRIT[df] * se, m + T_CRIT[df] * se], df


def contrast(cells, weights, blocks, metric, rng):
    xs = [100.0 * sum(w * cells[k][b][metric] for k, w in weights.items()) for b in blocks]
    t, df = strat_t(xs, blocks)
    return {"mean_pp": sum(xs) / len(xs), "ci95_pp": E1.strat_ci(xs, blocks, rng), "t95_pp": t, "t_df": df,
            "blocks_pp": xs, "blocks": [list(b) for b in blocks]}


def labelled(cells, weights, blocks, rng):
    c = {m: contrast(cells, weights, blocks, m, rng) for m in METRICS}
    c["crash"]["label"] = crash_label(c["crash"]["ci95_pp"], c["crash"]["t95_pp"], c["crash"]["mean_pp"],
                                      c["timeout"]["mean_pp"])
    c["capture"]["label"] = C2A.label(c["capture"]["ci95_pp"], c["capture"]["t95_pp"], c["capture"]["mean_pp"], CAP_BAND)
    c["timeout"]["label"] = C2A.label(c["timeout"]["ci95_pp"], c["timeout"]["t95_pp"], c["timeout"]["mean_pp"], TO_BAND)
    return c


def block_means(cells, weights, blocks):
    """Descriptive per-block differences in pp (no interval; stage B's three 175-bar blocks)."""
    xs = {m: [100.0 * sum(w * cells[k][b][m] for k, w in weights.items()) for b in blocks] for m in METRICS}
    return {m: {"mean_pp": sum(v) / len(v), "blocks_pp": v, "blocks": [list(b) for b in blocks]} for m, v in xs.items()}


# ---------------------------------------------------------------- controls (CAM-K1-E, 9 blocks)
def rates_truncated(cell_dir, quota):
    """gr_analyse.rates restricted to the first `quota` episodes of each env (review P2-5: this unit's 1,024-episode scale)."""
    rows = [r for r in K.read_jsonl(cell_dir / "episodes.jsonl") if r.get("in_quota") and r.get("env_episode_index", 0) < quota]
    n = len(rows)
    return {"n": n, **{o: sum(1 for r in rows if r.get("outcome") == o) / n for o in ("capture", "crash", "timeout")}}


def load_camk1e(k1e, blocks, quota=None):
    cells = {}
    for arm in ("F66", "K1s1", "K1s2"):
        cells[arm] = {}
        for b in blocks:
            d = k1e / "eval_01" / arm / (K1E_CELL % b)
            cells[arm][b] = {"dir": d, **(A.rates(d) if quota is None else rates_truncated(d, quota))}
    return cells


def camk1e_manifest_ok(k1e, blocks):
    rels = ["raw/eval_01/%s/%s/%s" % (arm, K1E_CELL % b, f) for arm in ("F66", "K1s1", "K1s2") for b in blocks
            for f in ("episodes.jsonl", "result.json", "receipt.json")]
    return E1.manifest_ok(CAMK1E_MANIFEST, k1e.parent, rels, "files")


def shifted(cells, arm, pp):
    """A copy of `cells` with arm's crash rate raised by pp percentage points in every block (and capture lowered)."""
    out = {k: {b: dict(v) for b, v in c.items()} for k, c in cells.items()}
    for b in out[arm]:
        out[arm][b]["crash"] += pp / 100.0
        out[arm][b]["capture"] -= pp / 100.0
    return out


def controls(k1e):
    """Fixed order, one generator. Known-bad and known-good contrasts of CAM-K1-E at 2,048 episodes per cell, then the
    null and a +3 pp shifted null at this unit's 1,024-episode scale (review P2-5)."""
    blocks = [(d, s) for d in K1E_DENSITIES for s in V.SEEDS]
    full = load_camk1e(k1e, blocks)
    q8 = load_camk1e(k1e, blocks, quota=V.QUOTA)
    q8_shift = shifted(q8, "K1s1", SHIFT_PP)
    null_mean = 100.0 * sum(q8["K1s1"][b]["crash"] - q8["K1s2"][b]["crash"] for b in blocks) / len(blocks)
    q8_contrast = {pp: shifted(q8, "K1s1", pp - null_mean) for pp in CONTRAST_REPORTED_PP}
    rng = random.Random(BOOT_SEED)
    out = {}
    plan = (("crash_on_K1E_F66_minus_K1s1", full, {"F66": 1, "K1s1": -1}, "crash", "WORSE"),
            ("crash_on_K1E_K1s1_minus_K1s2", full, {"K1s1": 1, "K1s2": -1}, "crash", "not MATERIAL_REDUCTION or WORSE"),
            ("crash_on_K1E_K1s1_minus_F66", full, {"K1s1": 1, "F66": -1}, "crash", "MATERIAL_REDUCTION"),
            ("capture_on_K1E_K1s1_minus_F66", full, {"K1s1": 1, "F66": -1}, "capture", "HIGHER"),
            ("crash_on_K1E_K1s1_minus_K1s2_quota8", q8, {"K1s1": 1, "K1s2": -1}, "crash", "not MATERIAL_REDUCTION or WORSE"),
            ("crash_on_K1E_K1s1_plus5pp_minus_K1s2_quota8", q8_shift, {"K1s1": 1, "K1s2": -1}, "crash", "WORSE"),
            ("capture_on_K1E_K1s1_minus5pp_minus_K1s2_quota8", q8_shift, {"K1s1": 1, "K1s2": -1}, "capture", "LOWER"),
            ("crash_contrast_plus3pp_K1s1_minus_K1s2_quota8", q8_contrast[3.0], {"K1s1": 1, "K1s2": -1}, "crash", "REPORTED"),
            ("crash_contrast_plus2pp_K1s1_minus_K1s2_quota8", q8_contrast[2.0], {"K1s1": 1, "K1s2": -1}, "crash", "REPORTED"))
    for name, cells, w, metric, required in plan:
        c = labelled(cells, w, blocks, rng)
        label = c[metric]["label"]
        ok = (True if required == "REPORTED" else
              label not in ("MATERIAL_REDUCTION", "WORSE") if required.startswith("not ") else label == required)
        out[name] = {"metric": metric, "label": label, "required": required, "ok": ok, "mean_pp": c[metric]["mean_pp"],
                     "ci95_pp": c[metric]["ci95_pp"], "t95_pp": c[metric]["t95_pp"], "t_df": c[metric]["t_df"],
                     "episodes_per_cell": cells["K1s1"][blocks[0]]["n"]}
    for name in ("crash_on_K1E_K1s1_minus_K1s2", "crash_on_K1E_K1s1_minus_K1s2_quota8"):
        cr = out[name]
        cr["half_width_pp"] = {"bootstrap": (cr["ci95_pp"][1] - cr["ci95_pp"][0]) / 2, "t": (cr["t95_pp"][1] - cr["t95_pp"][0]) / 2}
    out["quota8_null_mean_pp"] = null_mean
    out["contrast_shift_pp"] = {str(pp): pp - null_mean for pp in CONTRAST_REPORTED_PP}
    out["camk1e_cells_match_manifest"] = camk1e_manifest_ok(k1e, blocks)
    out["ok"] = all(v["ok"] for v in out.values() if isinstance(v, dict) and "ok" in v) and out["camk1e_cells_match_manifest"]
    return out


# ---------------------------------------------------------------- diagnostics from the ledger and the recorder
def episode_lengths(ledger_rows):
    """Per in-quota episode: steps = completion_step - the env's previous completion_step (0 before the first)."""
    last, out, quota_keys = {}, {}, set()
    for r in sorted(ledger_rows, key=lambda r: (r["env_index"], r["env_episode_index"])):
        env = r["env_index"]
        out[(env, r["env_episode_index"])] = r["completion_step"] - last.get(env, 0)
        last[env] = r["completion_step"]
        if r.get("in_quota"):
            quota_keys.add((env, r["env_episode_index"]))
    return {k: v for k, v in out.items() if k in quota_keys}


def cell_diagnostics(cell_dir):
    ledger = K.read_jsonl(cell_dir / "episodes.jsonl")
    rows = {(r["env_index"], r["env_episode_index"]): r for r in K.read_jsonl(cell_dir / V.DIAG_FILE)}
    lengths = episode_lengths(ledger)
    quota = [r for r in ledger if r.get("in_quota")]
    steps = sum(lengths.values())
    crashes = sum(1 for r in quota if r["outcome"] == "crash")
    classes = {}
    lengths_match = True
    kin = {"rel_speed": [], "closing": [], "lateral": [], "dist": []}
    blind = {"hidden_steps": 0, "visible_steps": 0, "over_cap": 0, "hid_vx": 0.0, "hid_abs_vy": 0.0, "hid_speed": 0.0,
             "hid_abs_yaw": 0.0, "hid_yaw": 0.0, "hid_exec_speed": 0.0, "hid_exec_abs_yaw": 0.0, "vis_speed": 0.0,
             "vis_abs_yaw": 0.0, "vis_exec_speed": 0.0, "vis_exec_abs_yaw": 0.0}
    yaw = {"request_over_ref": 0, "exec_over_ref": 0, "steps": 0}
    delays = {}
    for r in quota:
        key = (r["env_index"], r["env_episode_index"])
        d = rows.get(key)
        if d is None:
            lengths_match = False
            continue
        lengths_match &= d.get("episode_steps") == lengths.get(key)
        if r["outcome"] == "crash":
            classes[d.get("crash_class")] = classes.get(d.get("crash_class"), 0) + 1
        if r["outcome"] == "capture" and d.get("capture_rel_speed_mps") is not None:
            kin["rel_speed"].append(d["capture_rel_speed_mps"])
            kin["closing"].append(d["capture_closing_mps"])
            kin["lateral"].append(d["capture_lateral_mps"])
            kin["dist"].append(d["capture_dist_m"])
        h, v = d.get("hidden_steps") or 0, d.get("visible_steps") or 0
        blind["hidden_steps"] += h
        blind["visible_steps"] += v
        blind["over_cap"] += d.get("hidden_exec_over_blind_cap") or 0
        for key_sum, field, n in (("hid_vx", "hidden_raw_vx_mps", h), ("hid_abs_vy", "hidden_raw_abs_vy_mps", h),
                                  ("hid_speed", "hidden_raw_speed_mps", h), ("hid_abs_yaw", "hidden_raw_abs_yaw_radps", h),
                                  ("hid_yaw", "hidden_raw_yaw_radps", h), ("hid_exec_speed", "hidden_exec_speed_mps", h),
                                  ("hid_exec_abs_yaw", "hidden_exec_abs_yaw_radps", h),
                                  ("vis_speed", "visible_raw_speed_mps", v), ("vis_abs_yaw", "visible_raw_abs_yaw_radps", v),
                                  ("vis_exec_speed", "visible_exec_speed_mps", v), ("vis_exec_abs_yaw", "visible_exec_abs_yaw_radps", v)):
            if d.get(field) is not None:
                blind[key_sum] += d[field] * n
        yaw["request_over_ref"] += d.get("yaw_request_over_ref_steps") or 0
        yaw["exec_over_ref"] += d.get("yaw_exec_over_ref_steps") or 0
        yaw["steps"] += d.get("episode_steps") or 0
        delays[str(d.get("delay_k"))] = delays.get(str(d.get("delay_k")), 0) + 1

    def q(xs, p):
        return sorted(xs)[min(len(xs) - 1, int(p * len(xs)))] if xs else None

    hs, vs = max(1, blind["hidden_steps"]), max(1, blind["visible_steps"])
    res = json.loads((cell_dir / "result.json").read_text())
    return {"quota_episodes": len(quota), "steps_in_quota": steps, "crashes": crashes,
            "crash_per_1000_steps": 1000.0 * crashes / steps if steps else None,
            "recorder_lengths_equal_ledger": lengths_match, "crash_classes": classes,
            "crash_causes_result_json_excludes_target_contacts": res.get("crash_causes"),
            "yaw": {"request_over_ref_steps": yaw["request_over_ref"], "exec_over_ref_steps": yaw["exec_over_ref"],
                    "steps": yaw["steps"], "request_over_ref_share": yaw["request_over_ref"] / max(1, yaw["steps"])},
            "capture_kinematics": {"n": len(kin["rel_speed"]),
                                   **{k + "_mean": (sum(x) / len(x) if x else None) for k, x in kin.items()},
                                   **{k + "_p50": q(x, 0.5) for k, x in kin.items()}, **{k + "_p90": q(x, 0.9) for k, x in kin.items()},
                                   "share_rel_speed_over_1mps": (sum(1 for x in kin["rel_speed"] if x > 1.0) / len(kin["rel_speed"])
                                                                 if kin["rel_speed"] else None)},
            "blind": {"hidden_step_share": blind["hidden_steps"] / max(1, blind["hidden_steps"] + blind["visible_steps"]),
                      "hidden_steps": blind["hidden_steps"], "visible_steps": blind["visible_steps"],
                      "hidden_exec_over_blind_cap_share": blind["over_cap"] / hs,
                      "hidden_raw_vx_mps": blind["hid_vx"] / hs, "hidden_raw_abs_vy_mps": blind["hid_abs_vy"] / hs,
                      "hidden_raw_speed_mps": blind["hid_speed"] / hs, "hidden_raw_abs_yaw_radps": blind["hid_abs_yaw"] / hs,
                      "hidden_raw_yaw_radps": blind["hid_yaw"] / hs, "hidden_exec_speed_mps": blind["hid_exec_speed"] / hs,
                      "hidden_exec_abs_yaw_radps": blind["hid_exec_abs_yaw"] / hs,
                      "visible_raw_speed_mps": blind["vis_speed"] / vs, "visible_raw_abs_yaw_radps": blind["vis_abs_yaw"] / vs,
                      "visible_exec_speed_mps": blind["vis_exec_speed"] / vs, "visible_exec_abs_yaw_radps": blind["vis_exec_abs_yaw"] / vs},
            "delay_k_episodes": delays}


def pooled_diagnostics(per_cell):
    """Episode-weighted pooling of cell_diagnostics over cells (descriptive)."""
    eps = sum(c["quota_episodes"] for c in per_cell)
    steps = sum(c["steps_in_quota"] for c in per_cell)
    crashes = sum(c["crashes"] for c in per_cell)
    classes = {}
    for c in per_cell:
        for k, v in c["crash_classes"].items():
            classes[k] = classes.get(k, 0) + v
    hidden = sum(c["blind"]["hidden_steps"] for c in per_cell)
    visible = sum(c["blind"]["visible_steps"] for c in per_cell)
    ysteps = sum(c["yaw"]["steps"] for c in per_cell)
    return {"episodes": eps, "steps": steps, "crashes": crashes, "crash_per_1000_steps": 1000.0 * crashes / steps if steps else None,
            "crash_classes": classes, "crash_class_shares": {k: v / max(1, crashes) for k, v in classes.items()},
            "hidden_step_share": hidden / max(1, hidden + visible),
            "hidden_exec_over_blind_cap_share": (sum(c["blind"]["hidden_exec_over_blind_cap_share"] * c["blind"]["hidden_steps"]
                                                     for c in per_cell) / max(1, hidden)),
            "yaw_request_over_ref_share": sum(c["yaw"]["request_over_ref_steps"] for c in per_cell) / max(1, ysteps),
            "yaw_exec_over_ref_steps": sum(c["yaw"]["exec_over_ref_steps"] for c in per_cell),
            "capture_rel_speed_mean": (sum((c["capture_kinematics"]["rel_speed_mean"] or 0) * c["capture_kinematics"]["n"]
                                           for c in per_cell) / max(1, sum(c["capture_kinematics"]["n"] for c in per_cell)))}


# ---------------------------------------------------------------- bindings and the eval
def report_bindings(ident, ev, ident_path, blocks_present):
    """The report-level bindings (pure; review P2-6). `blocks_present` maps arm -> sorted blocks with an OK cell."""
    plan = V.eval_plan()
    required = [(a, d, s) for a, d, s, st in plan if st in ("A", "B")]
    ab = V.ab_records(ev.get("records", []))
    order = [(r.get("arm"), r.get("density"), r.get("seed")) for r in ev.get("records", [])]
    b = {"identity_pass": ident.get("verdict") == "IDENTITY_PASS",
         "identity_same_commit": ident.get("commit") == ev.get("commit"),
         "identity_is_the_one_eval_named": str(pathlib.Path(ident_path).resolve())
         == str(pathlib.Path(ev.get("identity_report") or "/nonexistent").resolve()),
         "eval_complete": ev.get("verdict") == "COMPLETE" and ev.get("cells_required") == len(required) == 33,
         "env_allowlisted_same": bool(ident.get("env_allowlisted")) and ident.get("env_allowlisted") == ev.get("env_allowlisted"),
         "train_receipt_same_and_present": bool(ev.get("train_receipt_sha256"))
         and ident.get("train_receipt_sha256") == ev.get("train_receipt_sha256"),
         "order_as_planned": order[:len(required)] == required and order == [(a, d, s) for a, d, s, _ in plan][:len(order)],
         "all_ab_records_ok": len(ab) == len(required) and all(r.get("verdict") == "OK" for r in ab),   # stage C never judged (P0-3)
         "changed_since_training_allowlisted": ev.get("changed_since_training") is not None
         and not V.diff_allowlist_problems(ev.get("changed_since_training") or [])}
    grid = [(d, s) for d in V.DENSITIES for s in V.SEEDS]
    for arm in V.GATED_ARMS:
        b["complete_grid_" + arm] = blocks_present.get(arm, []) == grid
    for arm in V.DESCRIPTIVE_ARMS:
        b["complete_175_" + arm] = [blk for blk in blocks_present.get(arm, []) if blk[0] == 175] == [(175, s) for s in V.SEEDS]
    return b


def load_eval(ident_dir, eval_dir):
    ident = json.loads((ident_dir / "identity_report.json").read_text())
    ev = json.loads((eval_dir / "eval_run_report.json").read_text())
    committed = ev.get("committed_sha256") or {}
    ck = ev.get("checkpoints") or {}
    train_receipt = json.loads(V.TRAIN_RECEIPT.read_text()) if V.TRAIN_RECEIPT.is_file() else {}
    cells = {a: {} for a in V.EVAL_ARMS}
    receipts_ok, lengths_ok, exec_yaw_ok = True, True, True
    diag = {}
    for r in ev.get("records", []):
        arm = r.get("arm")
        if r.get("stage") == "C" and r.get("verdict") != "OK":
            continue                                        # a cut stage-C cell is reported by the runner, never judged (P0-3)
        if arm not in cells or r.get("verdict") != "OK":
            receipts_ok = False
            continue
        d = eval_dir / arm / r["cell_id"]
        blk = (r["density"], r["seed"])
        try:
            cells[arm][blk] = {"dir": d, **A.rates(d)}
            want_ck = V.arm_ckpt(arm, {k: tuple(v) for k, v in ck.items()})
            receipt = json.loads((d / "receipt.json").read_text())
            receipts_ok &= (not V.receipt_problems(d, arm, r["density"], r["seed"], train_receipt, committed, want_ck[1])
                            and receipt.get("checkpoint_sha256") == want_ck[1] == r.get("checkpoint_sha256"))
            g = cell_diagnostics(d)
            diag["%s/%d/%d" % (arm, blk[0], blk[1])] = g
            lengths_ok &= g["recorder_lengths_equal_ledger"]
            if arm == "YC12k":
                exec_yaw_ok &= g["yaw"]["exec_over_ref_steps"] == 0
        except (OSError, ValueError, KeyError, TypeError):
            receipts_ok = False
    binding = report_bindings(ident, ev, ident_dir / "identity_report.json",
                              {a: sorted(c) for a, c in cells.items()})
    binding["cells_pass_the_runner_checks_again"] = receipts_ok
    binding["recorder_lengths_equal_ledger"] = lengths_ok
    binding["yc_executed_yaw_never_over_limit"] = exec_yaw_ok
    binding["train_receipt_at_recorded_sha"] = V.TRAIN_RECEIPT.is_file() and V.sha(V.TRAIN_RECEIPT) == ev.get("train_receipt_sha256")
    for ident_arm, eval_arm in (("S12k_identity", "S12k"), ("S1k_smoke", "S1k"), ("YC12k_negative", "YC12k")):
        a = ident_dir / ident_arm / (CELL % (205, 4501)) / "episodes.jsonl"
        b = eval_dir / eval_arm / (CELL % (205, 4501)) / "episodes.jsonl"
        binding["identity_%s_equals_eval_%s" % (ident_arm, eval_arm)] = a.is_file() and b.is_file() and K.read_jsonl(a) == K.read_jsonl(b)
    return binding, cells, ev, diag


def differ(cells, a, b, blocks):
    return all(blk in cells[a] and blk in cells[b] and K.read_jsonl(cells[a][blk]["dir"] / "episodes.jsonl")
               != K.read_jsonl(cells[b][blk]["dir"] / "episodes.jsonl") for blk in blocks)


def binding_share(diag, arm):
    """The clip's binding share pooled over an arm's cells: requested-yaw-over-limit steps / steps (exact, whole cells)."""
    cells = [v for k, v in diag.items() if k.startswith(arm + "/")]
    steps = sum(c["yaw"]["steps"] for c in cells)
    return (sum(c["yaw"]["request_over_ref_steps"] for c in cells) / steps) if steps else None


def write(out, res):
    with out.open("x") as fh:
        json.dump(res, fh, indent=1, default=str)
        fh.write("\n")


def main_controls(k1e, out):
    res = {"analysis": "CAM-S2a-E controls", "controls": controls(k1e), "bootstrap": {"resamples": A.BOOT, "seed": BOOT_SEED,
           "scheme": "seeds within density", "t_df": 6}, "script_sha256": E1.sha(__file__)}
    write(out, res)
    print(json.dumps({k: (v["label"] if isinstance(v, dict) else v) for k, v in res["controls"].items()}, indent=1))
    return 0 if res["controls"]["ok"] else 1


def main_eval(k1e, ident_dir, eval_dir, out):
    blocks = [(d, s) for d in V.DENSITIES for s in V.SEEDS]
    blocks175 = [(175, s) for s in V.SEEDS]
    binding, cells, ev, diag = load_eval(ident_dir, eval_dir)
    _ck, ck_problems = V.checkpoints()
    binding["checkpoints_at_pins"] = not ck_problems
    binding["script_equals_eval_commit"] = E1.file_matches_commit(__file__, "tools/cams2a_eval_analyse.py", ev.get("commit"))
    diffs = {"S12k_vs_S1k": differ(cells, "S12k", "S1k", blocks), "YC12k_vs_S12k": differ(cells, "YC12k", "S12k", blocks)}
    binding["cells_differ"] = all(diffs.values())
    binding["cells_differ_detail"] = diffs
    if not all(v for k, v in binding.items() if k != "cells_differ_detail"):
        res = {"analysis": "CAM-S2a-E", "reading": "EVAL_BINDING_FAIL", "binding": binding}
        write(out, res)
        print(json.dumps(res, indent=1, default=str))
        return 1
    ctrl = controls(k1e)
    rng = random.Random(BOOT_SEED)
    contrasts = {name: labelled(cells, w, blocks, rng) for name, w in CONTRASTS.items()}
    labels = {n: {m: contrasts[n][m]["label"] for m in METRICS} for n in CONTRASTS}
    yc_records = [r for r in ev.get("records", []) if r.get("arm") == "YC12k"]
    telemetry_present = bool(yc_records) and all(isinstance(r.get("yawclip_telemetry_last_line"), dict) for r in yc_records)
    share_yc, share_s12k = binding_share(diag, "YC12k"), binding_share(diag, "S12k")
    clipped_last_line = {"%s/%d/%d" % (r["arm"], r["density"], r["seed"]): (r.get("yawclip_telemetry_last_line") or {}).get("clipped_frac")
                         for r in yc_records}
    descriptive = {}
    for name, w in DESCRIPTIVE.items():
        arms = list(w)
        shared = [b for b in blocks if all(b in cells[a] for a in arms)]
        if shared == blocks:
            descriptive[name] = {"blocks": 9, **labelled(cells, w, blocks, rng)}
        elif all(b in shared for b in blocks175):
            descriptive[name] = {"blocks": len(shared), "no_interval": "fewer than 9 blocks", **block_means(cells, w, shared)}
    if not ctrl["ok"]:
        reading = {"reading": "CONTROL_FAIL", "gate_L": None, "gate_Y": None}
    else:
        reading = {"reading": "REPORTED",
                   "gate_L": gate_l(labels[GATED_L]["crash"], labels[GATED_L]["capture"]),
                   "gate_Y": gate_y(labels[GATED_Y]["crash"], labels[GATED_Y]["capture"], telemetry_present, share_yc)}
    pooled = {k: {m: 100.0 * sum(v[m] for v in c.values()) / len(c) for m in METRICS} for k, c in cells.items() if c}
    per_density = {k: {str(d): {m: 100.0 * sum(c[b][m] for b in c if b[0] == d) / sum(1 for b in c if b[0] == d)
                                for m in METRICS} for d in V.DENSITIES if any(b[0] == d for b in c)} for k, c in cells.items() if c}
    pooled_diag = {arm: pooled_diagnostics([v for k, v in diag.items() if k.startswith(arm + "/")])
                   for arm in V.EVAL_ARMS if any(k.startswith(arm + "/") for k in diag)}
    res = {"analysis": "CAM-S2a-E", **reading, "labels": labels,
           "yawclip": {"telemetry_present_in_every_YC12k_cell": telemetry_present, "min_binding_share": MIN_BINDING_SHARE,
                       "binding_share_YC12k_requested_over_limit": share_yc, "binding_share_S12k_requested_over_limit": share_s12k,
                       "clipped_share_last_telemetry_line_per_YC12k_cell": clipped_last_line,
                       "note": "TOLERATED admits a capture loss with intervals excluding 0 but a mean under 3 pp (SMALL_LOWER)"},
           "controls": ctrl, "binding": binding, "contrasts": contrasts, "descriptive_contrasts": descriptive,
           "pooled_rate_pct": pooled, "per_density_rate_pct": per_density, "diagnostics_per_cell": diag,
           "diagnostics_pooled_per_arm": pooled_diag,
           "speed_governor": {k: C2A.governor(c) for k, c in cells.items() if c},
           "yawclip_telemetry_last_line": {"%s/%d/%d" % (r["arm"], r["density"], r["seed"]): r.get("yawclip_telemetry_last_line")
                                           for r in ev.get("records", []) if r.get("yawclip")},
           "camsafe_telemetry_last_line": {"%s/%d/%d" % (r["arm"], r["density"], r["seed"]): r.get("camsafe_telemetry_last_line")
                                           for r in ev.get("records", [])},
           "mismatch_lines": {"%s/%d/%d" % (r["arm"], r["density"], r["seed"]): r.get("mismatch_lines") for r in ev.get("records", [])},
           "stage_c_cells_done": ev.get("stage_c_cells_done"), "blocks": [list(b) for b in blocks],
           "bootstrap": {"resamples": A.BOOT, "seed": BOOT_SEED, "scheme": "seeds within density", "t_df": 6,
                         "note": "3 seeds per stratum: the bootstrap has at most 10 distinct means per stratum; the t-interval is the operative guard"},
           "bands_pp": {"material": A_BAND, "floor": FLOOR, "capture": CAP_BAND, "timeout": TO_BAND},
           "inputs": [str(k1e), str(ident_dir), str(eval_dir)], "commit": ev.get("commit"), "script_sha256": E1.sha(__file__)}
    write(out, res)
    print(json.dumps({"reading": res["reading"], "gate_L": res["gate_L"], "gate_Y": res["gate_Y"], "labels": labels,
                      "binding_share_YC12k": share_yc, "controls": {k: v["label"] for k, v in ctrl.items() if isinstance(v, dict)}},
                     indent=1))
    return 0


def main(argv=None):
    argv = list(argv if argv is not None else sys.argv[1:])
    if argv[:1] == ["controls"] and len(argv) == 3:
        k1e, out = pathlib.Path(argv[1]), pathlib.Path(argv[2])
        if out.exists():
            raise SystemExit("refusing to overwrite " + str(out))
        return main_controls(k1e, out)
    if argv[:1] == ["eval"] and len(argv) == 5:
        k1e, ident_dir, eval_dir, out = (pathlib.Path(a) for a in argv[1:5])
        if out.exists():
            raise SystemExit("refusing to overwrite " + str(out))
        return main_eval(k1e, ident_dir, eval_dir, out)
    raise SystemExit("\n".join(__doc__.strip().splitlines()[-2:]).strip())


if __name__ == "__main__":
    raise SystemExit(main())
