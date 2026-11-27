"""Analysis of CAM-REF stage 1 (docs/plans/prereg_camonly_ref_matched_protocol_2026-11-26.md): Policy F, CAM-K1 (seed 1)
and CAM-S2b-FT on one protocol ladder at 205 bars, seeds 4501-4503 (one block per seed), the first 8 episodes per env.

Per cell (fixed by the preregistration): capture, crash (all / bar contact / target contact / out of bounds; below and
above as "other"), timeout, as per-seed rates, the episode-weighted pooled rate, the seed-block interval (mean of the
seed blocks +- t(0.975, blocks - 1) x sd / sqrt(blocks)) and an episode bootstrap interval. Crash classes come from the
per-episode recorders (navrl_camref_rules' recorder: crash_class; navrl_cams2diag: crash_class): contact = bar contact
(the task's cause 0; includes a physical contact with the target body), target_contact_swept = target contact (the
0.40 m swept rule), out_of_bounds; a cell without a per-episode recorder (GR-0, CAM-K1-E) reads NOT_RECORDED for them.
Episode bootstrap: 10,000 resamples, numpy default_rng(4611); every cell's counted episodes are resampled with replacement
inside the cell, drawn as one multinomial(n, category shares) per cell and resample (the categories: capture, the crash
classes, timeout; capture / crash / timeout for a cell without classes), cells in the fixed order CELL_ORDER x seeds; every
reading uses the same resamples; percentile interval (2.5 %, 97.5 %).
Differences (pp) are the mean over the complete seed blocks of the paired block difference, with the bootstrap interval
and the seed-block interval of the block differences.
Readings:
  R1  protocol share of S2b-FT: S0 - S6 (S0 = CAM-DX1 V312, whose seed-4501 cell S0ID reproduced), split S0 - S3
      (P_S -> P_T) and S3 - S6 (P_T -> P_K). Labels on capture and crash (all): MATERIAL (|difference| >= 5 pp and the
      bootstrap interval excludes 0), SMALL (interval excludes 0, |difference| < 5 pp), NO_DETECTED_CHANGE (otherwise);
      the sign is reported with the label (positive = S0 higher). Bar / target / out-of-bounds crash: reported.
  R2  policy gap at the matched points: F - S2b-FT and K1 - S2b-FT at P_K (F0 = GR-0 S00 and K1 = CAM-K1-E K1s1, whose
      seed-4501 cells F0ID and K1ID reproduced; S6), P_T (F4, K1T, S3) and P_S (F8, K1S, S0). Reported. The crash classes
      of GR-0 and CAM-K1-E read NOT_RECORDED; F0ID's and K1ID's (seed 4501) are reported separately.
  R3  F8 and F4 (all three seeds pooled, n episodes): one-sided 95 % Wilson upper bound (z = 1.6448536) and lower bound of
      (i) all crashes and (ii) bar-contact crashes against 20 %: BELOW_20 (upper <= 20 %), ABOVE_20 (lower > 20 %),
      UNRESOLVED otherwise. The same bounds are reported for S0 and S3 (the use-of-result rule reads S0, S2b-FT at P_S).
  Use of the result (fixed): R3(ii) BELOW_20 at F8 and S0 bar-contact crash ABOVE_20 -> sensing / representation;
  R3(ii) ABOVE_20 at F8 -> 205 bars under CAM-S2 rules not reached zero-shot even with 360 deg ranging; R1 crash MATERIAL
  -> the protocol factors that carry it are reported for the design. No flight claim.
Analysis controls (run before the eval is read; a miss reads CONTROL_FAIL and no label is given): synthetic ledgers
(identical cells -> R1 NO_DETECTED_CHANGE; S0 crash +8 pp / capture -8 pp in every block -> MATERIAL; 18 % crash with
n = 3,072 -> BELOW_20; 23 % -> ABOVE_20) and the recorded CAM-DX1 pair R312 - V312 at 205 bars x 4501-4503 (ledgers bound
by SHA-256) -> crash MATERIAL.
usage: tools/camref_eval_analyse.py controls OUT_JSON
       tools/camref_eval_analyse.py eval IDENTITY_DIR EVAL_DIR OUT_JSON
"""
import hashlib
import json
import math
import pathlib
import subprocess
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import kroc2027_eval as K                                    # noqa: E402  (read_jsonl)
import camref_eval_run as X                                  # noqa: E402  (cells, references, pins)

ROOT = pathlib.Path(__file__).resolve().parents[1]
BOOT_N = 10000
BOOT_SEED = 4611
MATERIAL_PP = 5.0
Z_ONE_SIDED_95 = 1.6448536269514722
BAR = 0.20
FIRST_N = X.FIRST_N
SEEDS = X.SEEDS
T975 = {1: 12.706204736, 2: 4.302652730}               # t(0.975, df)
CLASS_CATS = ("capture", "crash_bar", "crash_target", "crash_oob", "crash_other", "timeout")
PLAIN_CATS = ("capture", "crash", "timeout")
CLASS_OF = {"contact": "crash_bar", "target_contact_swept": "crash_target", "out_of_bounds": "crash_oob",
            "below": "crash_other", "above": "crash_other", "uncoded": "crash_other", "unknown": "crash_other"}
METRICS = ("capture", "crash", "timeout", "crash_bar", "crash_target", "crash_oob")
LABELLED = ("capture", "crash")
CELL_ORDER = ("S0", "S3", "S6", "F0", "F3", "F4", "F8", "K1PK", "K1T", "K1S")
R1 = {"S0-S6": {"S0": 1.0, "S6": -1.0}, "S0-S3": {"S0": 1.0, "S3": -1.0}, "S3-S6": {"S3": 1.0, "S6": -1.0}}
R2 = {"P_K": {"F-S2B": {"F0": 1.0, "S6": -1.0}, "K1-S2B": {"K1PK": 1.0, "S6": -1.0}},
      "P_T": {"F-S2B": {"F4": 1.0, "S3": -1.0}, "K1-S2B": {"K1T": 1.0, "S3": -1.0}},
      "P_S": {"F-S2B": {"F8": 1.0, "S0": -1.0}, "K1-S2B": {"K1S": 1.0, "S0": -1.0}}}
R3_CELLS = ("F8", "F4")
R3_REPORTED = ("S0", "S3")
SYN_N, SYN_N_POOLED = 1024, 3072
DX1_R312 = pathlib.Path.home() / "motar_tmp/camdx1_20261125/a1/eval/R312/DX1_R312__205bars__seed%d"
DX1_R312_SHA = {4501: "295b580ed296d9af59a28f7f338dd6237d2330b68bbffabf651846ae410093fc",
                4502: "d5f3be5fba0ec3701470e90f5c96aa49acf8489a8669445bd4aa3e55b035783d",
                4503: "7dc0e4fd6d60958f0740695568881662c2074246dfaaa62ebce0b29362f07e1a"}


def sha(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


# ---------------------------------------------------------------- counts (pure)
def counts(ledger_rows, class_rows=None, n_first=FIRST_N):
    """The category counts of the first n_first episodes per env. With `class_rows` (a recorder's per-episode rows), the
    crash classes too ("split": True); every counted crash must have a recorder row with the same outcome."""
    sel = [r for r in ledger_rows if int(r["env_episode_index"]) < n_first]
    out = {o: sum(1 for r in sel if r.get("outcome") == o) for o in PLAIN_CATS}
    out["n"] = len(sel)
    if out["capture"] + out["crash"] + out["timeout"] != out["n"]:
        raise ValueError("ledger rows outside capture / crash / timeout")
    out["split"] = class_rows is not None
    if class_rows is not None:
        by = {(r["env_index"], r["env_episode_index"]): r for r in class_rows}
        for c in CLASS_CATS[1:-1]:
            out[c] = 0
        for r in sel:
            if r["outcome"] != "crash":
                continue
            row = by.get((r["env_index"], r["env_episode_index"]))
            if row is None or row.get("outcome") != "crash":
                raise ValueError("crash %s has no recorder row with the same outcome" % ((r["env_index"],
                                                                                          r["env_episode_index"]),))
            out[CLASS_OF.get(row.get("crash_class"), "crash_other")] += 1
    return out


def rate(c, m):
    if m in ("crash_bar", "crash_target", "crash_oob") and not c.get("split"):
        return None
    return c[m] / c["n"] if c["n"] else float("nan")


def synthetic_counts(n, capture, crash, bar_share=1.0, split=True):
    nc, nk = int(round(n * capture)), int(round(n * crash))
    out = {"n": n, "capture": nc, "crash": nk, "timeout": n - nc - nk, "split": split}
    if split:
        nb = int(round(nk * bar_share))
        out.update(crash_bar=nb, crash_target=nk - nb, crash_oob=0, crash_other=0)
    return out


# ---------------------------------------------------------------- bootstrap and readings (pure)
def bootstrap(cells, n_boot=BOOT_N, seed=BOOT_SEED, order=CELL_ORDER):
    """{(cell, block): {metric: array [n_boot] of resampled rates}}, one multinomial per present cell and resample, in the
    fixed order `order` x blocks from one numpy default_rng(seed)."""
    rng = np.random.default_rng(seed)
    out = {}
    names = [a for a in order if a in cells] + sorted(a for a in cells if a not in order)
    for a in names:
        for b in sorted(cells[a]):
            c = cells[a][b]
            if not c or not c["n"]:
                continue
            cats = CLASS_CATS if c.get("split") else PLAIN_CATS
            p = np.array([c[k] for k in cats], dtype=float) / c["n"]
            draw = rng.multinomial(c["n"], p, size=n_boot) / float(c["n"])
            d = {k: draw[:, i] for i, k in enumerate(cats)}
            if c.get("split"):
                d["crash"] = draw[:, 1:5].sum(axis=1)
            out[(a, b)] = d
    return out


def label(diff_pp, ci_pp):
    lo, hi = ci_pp
    if not (lo > 0.0 or hi < 0.0):
        return "NO_DETECTED_CHANGE"
    return "MATERIAL" if abs(diff_pp) >= MATERIAL_PP else "SMALL"


def seed_block_interval(xs):
    if len(xs) < 2:
        return None
    m = sum(xs) / len(xs)
    sd = math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))
    h = T975[len(xs) - 1] * sd / math.sqrt(len(xs)) if len(xs) - 1 in T975 else None
    return None if h is None else [m - h, m + h]


def complete_blocks(cells, weights, blocks):
    return [b for b in blocks if all(((cells.get(a) or {}).get(b) or {}).get("n") for a in weights)]


def effect(cells, weights, blocks, boot, labelled=LABELLED):
    """Per metric: the paired block differences (pp), their mean, the bootstrap and seed-block intervals and, for the
    labelled metrics, the label; NOT_RECORDED for a crash class some cell lacks; NOT_RUN when no block is complete."""
    blocks = complete_blocks(cells, weights, blocks)
    if not blocks:
        return "NOT_RUN"
    out = {"blocks": list(blocks), "weights": dict(weights)}
    for m in METRICS:
        if any(rate(cells[a][b], m) is None for a in weights for b in blocks):
            out[m] = "NOT_RECORDED"
            continue
        per_block = [100.0 * sum(w * rate(cells[a][b], m) for a, w in weights.items()) for b in blocks]
        mean = sum(per_block) / len(per_block)
        draws = sum(w * boot[(a, b)][m] for a, w in weights.items() for b in blocks) * (100.0 / len(blocks))
        lo, hi = (float(x) for x in np.percentile(draws, [2.5, 97.5]))
        out[m] = {"block_diff_pp": per_block, "mean_pp": mean, "ci95_pp": [lo, hi],
                  "seed_block_ci95_pp": seed_block_interval(per_block),
                  "label": label(mean, (lo, hi)) if m in labelled else None}
    return out


def wilson(k, n, z=Z_ONE_SIDED_95):
    """One-sided 95 % Wilson (lower, upper) bounds of k / n."""
    if n <= 0:
        return None, None
    p = k / n
    den = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return centre - half, centre + half


def bar_label(k, n, bar=BAR):
    lo, hi = wilson(k, n)
    if hi is None:
        return "NOT_RUN"
    return "BELOW_20" if hi <= bar else ("ABOVE_20" if lo > bar else "UNRESOLVED")


def r3_reading(cells, name, blocks):
    by = cells.get(name) or {}
    present = [b for b in blocks if (by.get(b) or {}).get("n")]
    if not present:
        return "NOT_RUN"
    n = sum(by[b]["n"] for b in present)
    out = {"blocks": present, "n": n}
    for metric, key in (("all_crashes", "crash"), ("bar_contact_crashes", "crash_bar")):
        if any(rate(by[b], key) is None for b in present):
            out[metric] = "NOT_RECORDED"
            continue
        k = sum(by[b][key] for b in present)
        lo, hi = wilson(k, n)
        out[metric] = {"k": k, "rate": k / n, "wilson_lower_one_sided_95": lo, "wilson_upper_one_sided_95": hi,
                       "label": bar_label(k, n)}
    return out


def cell_summary(by, boot, name):
    out = {}
    blocks = sorted(by)
    for m in METRICS:
        rates = {str(b): rate(by[b], m) for b in blocks}
        if any(v is None for v in rates.values()):
            out[m] = "NOT_RECORDED"
            continue
        xs = [100.0 * rates[str(b)] for b in blocks]
        n = sum(by[b]["n"] for b in blocks)
        pooled = 100.0 * sum(by[b][m] for b in blocks) / n if n else None
        draws = sum(boot[(name, b)][m] for b in blocks) * (100.0 / len(blocks))
        out[m] = {"per_seed_pct": {str(b): 100.0 * rates[str(b)] for b in blocks}, "pooled_pct": pooled,
                  "block_mean_pct": sum(xs) / len(xs), "seed_block_ci95_pct": seed_block_interval(xs),
                  "bootstrap_ci95_pct": [float(x) for x in np.percentile(draws, [2.5, 97.5])]}
    out["episodes"] = {str(b): by[b]["n"] for b in blocks}
    return out


def readings(cells, blocks=SEEDS):
    boot = bootstrap(cells)
    r1 = {k: effect(cells, w, blocks, boot) for k, w in R1.items()}
    r2 = {pt: {k: effect(cells, w, blocks, boot, labelled=()) for k, w in d.items()} for pt, d in R2.items()}
    r3 = {name: r3_reading(cells, name, blocks) for name in R3_CELLS + R3_REPORTED}
    per_cell = {name: cell_summary(by, boot, name) for name, by in cells.items() if by}
    return {"R1": r1, "R2": r2, "R3": r3, "cells": per_cell}


def use_of_result(r):
    def lab(cell, metric):
        e = r["R3"].get(cell)
        v = e.get(metric) if isinstance(e, dict) else None
        return v.get("label") if isinstance(v, dict) else v

    f8_bar, s0_bar = lab("F8", "bar_contact_crashes"), lab("S0", "bar_contact_crashes")
    e = r["R1"].get("S0-S6")
    r1_crash = e.get("crash", {}).get("label") if isinstance(e, dict) and isinstance(e.get("crash"), dict) else None
    out = []
    if f8_bar == "BELOW_20" and s0_bar == "ABOVE_20":
        out.append("R3(ii) BELOW_20 at F8 and S2b-FT (S0) bar-contact crash ABOVE_20: the gap is sensing / representation; "
                   "the next design gives the camera policy a 360 deg range representation (map / memory, NavRL-style) "
                   "and/or a privileged teacher")
    if f8_bar == "ABOVE_20":
        out.append("R3(ii) ABOVE_20 at F8: 205 bars under CAM-S2 rules is not reached even with 360 deg ranging zero-shot; "
                   "the next design revisits the task rules, the obstacle field or trains a 360 deg reference under P_S "
                   "first")
    if r1_crash == "MATERIAL":
        out.append("R1 crash MATERIAL: the protocol factors that carry it (S0-S3, S3-S6) are reported for the design; no "
                   "flight claim")
    return out or ["no preregistered pointer applies (F8 bar %s, S0 bar %s, R1 crash %s)" % (f8_bar, s0_bar, r1_crash)]


# ---------------------------------------------------------------- controls
def synthetic_cells(n, s0_capture_pp=0.0, s0_crash_pp=0.0, base=(0.62, 0.30)):
    cells = {}
    for a in ("S0", "S3", "S6"):
        cells[a] = {}
        for s in SEEDS:
            cap, cra = base
            if a == "S0":
                cap, cra = cap + s0_capture_pp / 100.0, cra + s0_crash_pp / 100.0
            cells[a][s] = synthetic_counts(n, cap, cra)
    return cells


def controls():
    """The preregistered analysis controls. Returns (all_ok, results)."""
    res = {}
    for name, dcap, dcra, want in (("synthetic_identical", 0.0, 0.0, ("NO_DETECTED_CHANGE", "NO_DETECTED_CHANGE")),
                                   ("synthetic_plus8pp", -8.0, 8.0, ("MATERIAL", "MATERIAL"))):
        cells = synthetic_cells(SYN_N, dcap, dcra)
        e = readings(cells)["R1"]["S0-S6"]
        got = (e["capture"]["label"], e["crash"]["label"])
        res[name] = {"episodes_per_cell": SYN_N, "s0_shift_pp": {"capture": dcap, "crash": dcra}, "required": want,
                     "labels": got, "crash": e["crash"], "ok": got == want}
    for name, share, want in (("synthetic_18pct_n3072", 0.18, "BELOW_20"), ("synthetic_23pct_n3072", 0.23, "ABOVE_20")):
        k = int(round(SYN_N_POOLED * share))
        lo, hi = wilson(k, SYN_N_POOLED)
        got = bar_label(k, SYN_N_POOLED)
        res[name] = {"k": k, "n": SYN_N_POOLED, "wilson_lower": lo, "wilson_upper": hi, "required": want, "label": got,
                     "ok": got == want}
    man = {}
    try:
        cells = {"R312": {}, "V312": {}}
        for s in SEEDS:
            for arm, path in (("R312", pathlib.Path(str(DX1_R312) % s) / "episodes.jsonl"),
                              ("V312", X.reference_dir("DX1_V312", s) / "episodes.jsonl")):
                man[str(path)] = sha(path)
                cells[arm][s] = counts(K.read_jsonl(path))
        pins_ok = all(man[str(pathlib.Path(str(DX1_R312) % s) / "episodes.jsonl")] == DX1_R312_SHA[s]
                      and man[str(X.reference_dir("DX1_V312", s) / "episodes.jsonl")]
                      == X.REF_SHA[("DX1_V312", s)]["episodes.jsonl"] for s in SEEDS)
        boot = bootstrap(cells, order=("R312", "V312"))
        e = effect(cells, {"R312": 1.0, "V312": -1.0}, list(SEEDS), boot)
        got = e["crash"]["label"]
        res["dx1_r312_minus_v312_205"] = {"required": "MATERIAL", "label": got, "crash": e["crash"], "manifest": man,
                                          "pins_ok": pins_ok, "ok": got == "MATERIAL" and pins_ok}
    except (OSError, ValueError, KeyError) as exc:
        res["dx1_r312_minus_v312_205"] = {"ok": False, "error": str(exc), "manifest": man}
    return all(v["ok"] for v in res.values()), res


# ---------------------------------------------------------------- loading the cells
def class_rows_of(cell_dir):
    """The per-episode crash-class rows of a cell: the CAM-REF recorder, else navrl_cams2diag, else None."""
    for name in (X.RECORD_FILE, X.DIAG_FILE):
        path = pathlib.Path(cell_dir) / name
        if path.is_file():
            return K.read_jsonl(path), name
    return None, None


def load_cell(cell_dir, n_first=FIRST_N, num_envs=X.NUM_ENVS):
    rows = K.read_jsonl(pathlib.Path(cell_dir) / "episodes.jsonl")
    cls, source = class_rows_of(cell_dir)
    c = counts(rows, cls, n_first)
    per_env = {}
    for r in rows:
        if int(r["env_episode_index"]) < n_first:
            per_env[r["env_index"]] = per_env.get(r["env_index"], 0) + 1
    if len(per_env) != num_envs or any(v != n_first for v in per_env.values()):
        raise ValueError("%s does not hold %d episodes in each of %d envs" % (cell_dir, n_first, num_envs))
    c["class_source"] = source
    return c


def file_matches_commit(path, rel, commit, root=ROOT):
    done = subprocess.run(["git", "-C", str(root), "show", "%s:%s" % (commit, rel)], capture_output=True)
    return done.returncode == 0 and hashlib.sha256(done.stdout).hexdigest() == sha(path)


def load_eval(ident_dir, eval_dir):
    """(binding, cells): the new cells by rung arm and seed plus the reference cells, with every binding checked."""
    ident = json.loads((pathlib.Path(ident_dir) / "identity_report.json").read_text())
    ev = json.loads((pathlib.Path(eval_dir) / "eval_run_report.json").read_text())
    b = {"identity_verdict": ident.get("verdict"), "eval_verdict": ev.get("verdict"),
         "identity_ok": ident.get("verdict") == "IDENTITY_PASS",
         "same_commit": ident.get("commit") == ev.get("commit"),
         "eval_names_identity": str(pathlib.Path(ident_dir).resolve() / "identity_report.json")
         in (ev.get("linked_reports") or {}).values(),
         "references_at_pins": not X.reference_problems(),
         "script_matches_eval_commit": file_matches_commit(pathlib.Path(__file__).resolve(), "tools/camref_eval_analyse.py",
                                                           ev.get("commit") or "")}
    cells = {}
    for r in ev.get("records") or []:
        if r.get("verdict") != "OK":
            continue
        cells.setdefault(r["arm"], {})[r["seed"]] = load_cell(r["cell_dir"])
    cells["S0"] = {s: load_cell(X.reference_dir("DX1_V312", s)) for s in SEEDS}
    cells["K1PK"] = {s: load_cell(X.reference_dir("K1E_K1s1", s)) for s in SEEDS}
    cells["F0"] = {s: load_cell(X.reference_dir("GR0_S00", s)) for s in SEEDS}
    for arm in ("F0ID", "K1ID", "S0ID"):                     # the identity cells, reported with their crash classes
        rec = next((r for r in ident.get("records") or [] if r.get("arm") == arm and r.get("verdict") == "OK"), None)
        b[arm.lower() + "_cell_reported"] = load_cell(rec["cell_dir"]) if rec else None
    b["complete_blocks"] = sorted(s for s in SEEDS if all(s in (cells.get(a) or {}) for a in X.EVAL_ARMS))
    b["ok"] = all(b[k] for k in ("identity_ok", "same_commit", "eval_names_identity", "references_at_pins",
                                 "script_matches_eval_commit"))
    return b, cells


def main(argv=None):
    argv = list(argv if argv is not None else sys.argv[1:])
    if len(argv) == 2 and argv[0] == "controls":
        ok, res = controls()
        pathlib.Path(argv[1]).write_text(json.dumps({"controls_ok": ok, "controls": res}, indent=1, default=float) + "\n")
        print(json.dumps({"controls_ok": ok, **{k: v.get("label", v.get("labels")) for k, v in res.items()}}, indent=1))
        return 0 if ok else 1
    if len(argv) != 4 or argv[0] != "eval":
        raise SystemExit(__doc__.split("usage:")[1])
    ident_dir, eval_dir, out = argv[1], argv[2], pathlib.Path(argv[3])
    ok, res = controls()
    binding, cells = load_eval(ident_dir, eval_dir)
    report = {"unit": X.UNIT, "prereg": X.PREREG, "controls_ok": ok, "controls": res, "binding": binding}
    if not ok:
        report["verdict"] = "CONTROL_FAIL"
    elif not binding["ok"]:
        report["verdict"] = "EVAL_BINDING_FAIL"
    elif not binding["complete_blocks"]:
        report["verdict"] = "NOT_RUN"          # review r1 P1-2: prereg reads complete seed blocks only; never fall back
    else:
        r = readings(cells, blocks=binding["complete_blocks"])
        report.update(r, use_of_result=use_of_result(r), verdict="READ")
    out.write_text(json.dumps(report, indent=1, default=float) + "\n")
    print(json.dumps({"verdict": report["verdict"], "use_of_result": report.get("use_of_result")}, indent=1))
    return 0 if report["verdict"] == "READ" else 1


if __name__ == "__main__":
    raise SystemExit(main())
