"""Runner of CAM-K2-E, the matched evaluation of CAM-K2 v2 (docs/plans/prereg_camonly_k2_eval_v1_2026-11-08.md; draft
~/motar_tmp/camk2e_20261108/PREREG_DRAFT.md). It evaluates CAM-K2 v2 only (tools/camk2v2_train.py, the real vehicle's
T/W 4.0 as a raised per-motor limit, opt-in NAVRL_VEH_RAND_LIMIT_MODE=raise). CAM-K2 v1 (tools/camk2_train.py, the T/W 2.25 cap) is WITHDRAWN_BEFORE_RUN:
nothing here imports its constants or reads its unit folder, and a v1 checkpoint or manifest is refused.

Built on CAM-K1-E's runner (tools/camk1e_run.py: approval, CAM-K1 checkpoints, camera values) and tools/gr_run.py's
cell environment, checks and launcher call (GR-0's S00 grid condition: E2, A3, quota ledger, motion recorder,
direct_l2_v1, gains OFF), with the launcher copy aerial_gym/rl_training/rl_games/eval_navrl_camk2e_arms.sh (SHA-pinned).
Every cell runs `no_lidar` at the 66 x 54 deg camera through tools/realveh_python (CAM-K2's entry point):
  treatments ON   vehicle randomization with the per-motor limit raised to T/W 4.0 (4.0 x m x g / 4, 2.4525 N
                  at 0.25 kg), thrust +-15 % per env, imbalance +-5 % per motor, tau 0.02-0.04 s, and the OAK-D Pro
                  depth model, all as CAM-K2 v2 trained (tools/camk2v2_train.py), with
                  the module seeds fixed per cell from the cell seed (10 x seed + 1, 10 x seed + 2): the same in every
                  arm of a block (matched);
  treatments OFF  both switches off: the camtrain path (ablation and overrides) through the realveh entry.
A cell FAILs unless its realveh receipt names exactly the arm's settings, the committed SHA-256 of every hashed file,
task robot navrl_quad and the CAM-K1 overrides; its NAVRL snapshot names the condition, the camera values and exactly
the treatment keys of its mode; the recorder and the cell log show the 66 x 54 deg camera; and, with treatments ON,
the modules' telemetry and the vehicle-randomization record show plausible values (as CAM-K2's post-run check), and
with treatments OFF no telemetry line appears.

  identity  K1s1_off_identity: CAM-K1 seed 1, treatments OFF, through this launcher and entry, must equal CAM-K1-E's
            eval K1s1 cell (205 bars, seed 4501) exactly; K1s1_on_negative: the same with the treatments ON must
            differ from it; K1s1_on_repeat: the same ON cell again must equal K1s1_on_negative exactly (determinism
            under the treatments, review r1 E-L2); K2s1_smoke: CAM-K2 seed 1, treatments ON, must be valid and
            differ from K1s1_on_negative. 4 cells; their wall times feed the eval's rate gate.
  eval      on GR-0's S00 grid (4 densities x 3 seeds, quota 16), order density, seed, arm: the main arms K1s1, K1s2,
            K2s1, K2s2 under the treatments (48 cells), then in the 70 and 205 bar blocks the nominal arms K2s1_nom,
            K2s2_nom (12 cells); 60 cells. Needs exactly one identity report in the same parent folder, an
            IDENTITY_PASS on this commit, no earlier eval there, the same allowlisted runner environment and the same
            CAM-K1-E folder, and the rate gate: RATE_CELLS (60) x CAM-K1-E's mean eval cell x (identity / CAM-K1-E
            identity cell time) x RATE_MARGIN (1.10) <= MAX_HOURS (review r2 R2-1). --max-hours other than 3.5 refuses.
Fail-closed preflight: an execution approval naming SCOPE and HEAD; clean tree; branch camonly/dev; the preregistration,
launcher copy, modules, entry point, wrapper, this runner and the analysis committed; the launcher at its pinned SHA;
the CAM-K1 checkpoints from their TRAINED train reports (pinned in camk1e_run); the CAM-K2 checkpoints from their
TRAINED train reports and TRAINED sidecar manifests at the SHA-256 pinned below (K2_SHA; placeholders refuse); one
RTX 3070; no other GPU compute process; import origins inside the tree; free disk >= 1 GB; the output folder new; a
COMPLETE CAM-K1-E eval folder. A rise in the host fault count voids the run.
usage: tools/camk2e_run.py {identity,eval} --approved-commit SHA --camk1e-raw DIR --out DIR [--max-hours H]
"""
import argparse
import json
import os
import pathlib
import re
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import gr_run as G                                          # noqa: E402
import kroc2027_eval as K                                   # noqa: E402
import camk1e_run as R1                                     # noqa: E402  (approval, CAM-K1 checkpoints, camera values)
import camk2v2_train as T2                                  # noqa: E402  (CAM-K2 v2's settings, telemetry, record, entry)

PREREG = "docs/plans/prereg_camonly_k2_eval_v1_2026-11-08.md"
SCOPE = ("CAM-K2-E v1: matched GPU evaluation of CAM-K2 v2 seeds 1 and 2 against CAM-K1 seeds 1 and 2 under CAM-K2 "
         "v2's treatments (no_lidar, 66 x 54 deg, per-motor limit raised to T/W 4.0, thrust +-15 %, motor imbalance "
         "+-5 %, tau 0.02-0.04 s, OAK-D Pro depth model) on GR-0's S00 grid (4 densities x 3 seeds, quota 16), plus "
         "CAM-K2 v2 with the treatments off (F's vehicle: T/W 3.26, tau 0.04 s, exact depth) at 70 and 205 bars, "
         "after a 4-cell identity control; 64 cells, cap 3.5 h per stage with a rate gate, idle host; "
         "tools/camk2e_run.py")
UNIT_DIR = pathlib.Path.home() / "motar_tmp" / "camk2e_20261108"
CAMK2_DIR = pathlib.Path.home() / "motar_tmp" / "camk2v2_20261108"     # CAM-K2 v2's unit folder (v1's is withdrawn)
CAMK2_UNIT = "CAM-K2 v2"                                                 # the sidecar manifest's unit
ENV_ALLOW = R1.ENV_ALLOW
LAUNCHER = "aerial_gym/rl_training/rl_games/eval_navrl_camk2e_arms.sh"
LAUNCHER_SHA = "e5923d8ee1490e7602d703b544a275a0bde35aa9154c5f99461ddbe55c89985e"
ENTRY = "tools/realveh_entry.py"
WRAPPER = ROOT / "tools/realveh_python"
MODULES = ("aerial_gym/task/navrl_task/navrl_sensor_ablation.py", "aerial_gym/task/navrl_task/navrl_vehicle_rand.py",
           "aerial_gym/task/navrl_task/navrl_depth_model.py")
COMMITTED_FILES = (PREREG, LAUNCHER, ENTRY, "tools/realveh_python", "tools/camk2e_run.py", "tools/camk2e_analyse.py",
                   "tools/camk2v2_train.py") + MODULES
BRANCH = "camonly/dev"
CONDITION = "no_lidar"
QUOTA = 16
MIN_FREE_GB = 1.0
MAX_HOURS = 3.5
ROBOT = "navrl_quad"
# CAM-K2 v2 final checkpoints. PLACEHOLDERS until both seeds read TRAINED: fill each from
# ~/motar_tmp/camk2v2_20261108/train_s{seed}/train_report.json ("checkpoint": "sha256") after checking its sidecar
# manifest (verdict TRAINED), before the commit (review R3-1). checkpoints() and tests/test_camk2e.py refuse these.
PLACEHOLDER = "FILL_AFTER_CAMK2_TRAINING"
K2_SHA = {1: "769a26f48fa202061adb1bae4024258e6e5b8d058a0b80ba39da02ab7de04e8a",      # CAM-K2 v2 s1, 04f341c
          2: "e76eb04878d1cb7ede425fe0c3fe0c70c68e5ca2801e6acf9c9ec1bd88321db6"}      # CAM-K2 v2 s2, 04f341c
MAIN_ARMS = ("K1s1", "K1s2", "K2s1", "K2s2")
NOMINAL_ARMS = ("K2s1_nom", "K2s2_nom")
NOMINAL_DENSITIES = (70, 205)
IDENTITY_PLAN = (("K1s1_off_identity", 205, 4501), ("K1s1_on_negative", 205, 4501), ("K1s1_on_repeat", 205, 4501),
                 ("K2s1_smoke", 205, 4501))
RATE_CELLS, RATE_MARGIN = 60, 1.10          # review r2 R2-1: 60 x CAM-K1-E eval mean cell x (identity / CAM-K1-E identity) x 1.1 <= cap
TREATED_ARMS = MAIN_ARMS + ("K1s1_on_negative", "K1s1_on_repeat", "K2s1_smoke")
CELL = "E2_obstacle_aware__%dbars__seed%d"
IDENTITY_REFERENCE = "eval_01/K1s1/" + CELL % (205, 4501)        # CAM-K1-E's eval K1s1 cell, inside --camk1e-raw
VEH_KEYS = ("NAVRL_VEH_RAND", "NAVRL_VEH_RAND_SEED", "NAVRL_VEH_RAND_LIMIT_MODE") + tuple(T2.VEH_RAND)
DEPTH_KEYS = ("NAVRL_DEPTH_MODEL", "NAVRL_DEPTH_SEED") + tuple(T2.DEPTH_MODEL)
TREATMENT_KEYS = VEH_KEYS + DEPTH_KEYS
# The keys a cell's NAVRL snapshot may differ in from CAM-K1-E's K1s cell of the same block (the run's nonce, file
# paths and run id, the checkpoint SHA, the receipt variable of the entry point, and the treatment keys).
SNAPSHOT_EXPECTED = {"NAVRL_BULK_EVAL_JSON", "NAVRL_EVAL_RUN_NONCE", "NAVRL_KROC2027_RUN_ID", "NAVRL_EVAL_CHECKPOINT",
                     "NAVRL_SENSOR_ABLATION_RECEIPT", "NAVRL_CAMTRAIN_RECEIPT", "NAVRL_REALVEH_RECEIPT"} | set(TREATMENT_KEYS)


def treated(arm):
    return arm in TREATED_ARMS


def module_seeds(seed):
    """The vehicle-randomization and depth-model seeds of every arm's cell at evaluation seed `seed` (matched)."""
    return 10 * int(seed) + 1, 10 * int(seed) + 2


def expected_settings(seed, on):
    """The realveh receipt's settings for a cell (the CAM-K2 values, the per-cell module seeds)."""
    veh_seed, depth_seed = module_seeds(seed)
    return {"vehicle_rand": dict(T2.VEH_RAND, NAVRL_VEH_RAND_SEED=veh_seed,
                                 NAVRL_VEH_RAND_LIMIT_MODE=T2.VEH_RAND_LIMIT_MODE) if on else None,
            "depth_model": dict(T2.DEPTH_MODEL, NAVRL_DEPTH_SEED=depth_seed) if on else None,
            "camtrain": dict(T2.EXPECTED_CAMTRAIN)}


def treatment_env(seed):
    """The NAVRL_* variables of the treatments for a cell at evaluation seed `seed` (as the launcher expects)."""
    veh_seed, depth_seed = module_seeds(seed)
    env = {"NAVRL_VEH_RAND": "1", "NAVRL_VEH_RAND_SEED": str(veh_seed), "NAVRL_DEPTH_MODEL": "oakd",
           "NAVRL_DEPTH_SEED": str(depth_seed), "NAVRL_VEH_RAND_LIMIT_MODE": T2.VEH_RAND_LIMIT_MODE}
    env.update({k: repr(float(v)) for k, v in T2.VEH_RAND.items()})
    env.update({k: repr(float(v)) for k, v in T2.DEPTH_MODEL.items()})
    return env


def arm_ckpt(arm, ckpts):
    base = {"K1s1_off_identity": "K1s1", "K1s1_on_negative": "K1s1", "K1s1_on_repeat": "K1s1", "K2s1_smoke": "K2s1",
            "K2s1_nom": "K2s1",
            "K2s2_nom": "K2s2"}.get(arm, arm)
    return ckpts[base]


def checkpoints(camk1_dir=R1.CAMK1_DIR, camk2_dir=CAMK2_DIR):
    """The CAM-K1 checkpoints (camk1e_run's pinned TRAINED reports) and the CAM-K2 checkpoints from their TRAINED train
    reports and sidecar manifests at the K2_SHA pins."""
    out, problems = R1.checkpoints(camk1_dir)
    problems = [p for p in problems if not p.startswith("Policy F")]
    out = {k: v for k, v in out.items() if k != "F"}
    for seed in (1, 2):
        pin = K2_SHA[seed]
        if not re.fullmatch(r"[0-9a-f]{64}", str(pin)):
            problems.append("CAM-K2 seed %d checkpoint SHA pin is a placeholder (%r): fill it from the TRAINED train "
                            "report before the commit" % (seed, pin))
        rp = camk2_dir / ("train_s%d" % seed) / "train_report.json"
        try:
            tr = json.loads(rp.read_text())
            ck = tr["checkpoint"]
        except (OSError, ValueError, KeyError, TypeError) as e:
            problems.append("CAM-K2 seed %d train report unreadable: %s" % (seed, e))
            continue
        if tr.get("verdict") != "TRAINED" or tr.get("seed") != seed or tr.get("development_smoke"):
            problems.append("CAM-K2 seed %d is not a TRAINED non-smoke run" % seed)
        if tr.get("prereg") != T2.PREREG or tr.get("scope") != T2.SCOPE:
            problems.append("CAM-K2 seed %d train report is not CAM-K2 v2's (prereg %r)" % (seed, tr.get("prereg")))
        if ck.get("sha256") != pin:
            problems.append("CAM-K2 seed %d checkpoint SHA %s is not the pinned %s" % (seed, ck.get("sha256"), pin))
        copy = pathlib.Path(str(ck.get("copy") or ""))
        if ck.get("sha256") != ck.get("copy_sha256") or not copy.is_file() or G.sha(copy) != ck.get("copy_sha256"):
            problems.append("CAM-K2 seed %d checkpoint copy missing or SHA mismatch" % seed)
        man = copy.parent / (copy.name + ".manifest.json")
        try:
            m = json.loads(man.read_text())
            want = (ck.get("manifest") or {}).get("sha256")
            if want != G.sha(man):
                problems.append("CAM-K2 seed %d manifest SHA differs from the train report's" % seed)
            if (m.get("verdict") != "TRAINED" or m.get("checkpoint_sha256") != pin or m.get("robot") != ROBOT
                    or m.get("treatments") != T2.expected_settings(seed) or m.get("seed") != seed
                    or m.get("unit") != CAMK2_UNIT or m.get("prereg") != T2.PREREG):
                problems.append("CAM-K2 seed %d manifest is not a TRAINED manifest of this checkpoint and the CAM-K2 "
                                "treatments" % seed)
        except (OSError, ValueError) as e:
            problems.append("CAM-K2 seed %d sidecar manifest unreadable: %s" % (seed, e))
        out["K2s%d" % seed] = (str(copy), ck.get("copy_sha256"))
    return out, problems


def telemetry_problems(text, expected):
    """CAM-K2 v2's post-run telemetry rules (tools/camk2v2_train.telemetry_problems) for a cell's expected settings."""
    p = []
    veh = T2.telemetry_lines([text], T2.VR_MOD.LOG_PREFIX)
    dep = T2.telemetry_lines([text], T2.DM_MOD.LOG_PREFIX)
    vr, dm = expected["vehicle_rand"], expected["depth_model"]
    if vr is None and dm is None:
        if veh or dep:
            p.append("treatments OFF, but %d telemetry lines appear" % (len(veh) + len(dep)))
        return p
    lo = (1.0 - vr["NAVRL_VEH_RAND_TW_SCALE"]) * (1.0 - vr["NAVRL_VEH_RAND_MOTOR_IMBAL"])
    hi = (1.0 + vr["NAVRL_VEH_RAND_TW_SCALE"]) * (1.0 + vr["NAVRL_VEH_RAND_MOTOR_IMBAL"])
    t_lo, t_hi = vr["NAVRL_VEH_RAND_TAU_MIN"], vr["NAVRL_VEH_RAND_TAU_MAX"]
    by = {e: [x for x in veh if x.get("event") == e] for e in ("install", "init", "reset", "update")}
    if len(by["install"]) != 1 or by["install"][0].get("settings") != vr:
        p.append("veh-rand: expected one install line with the cell's settings, got %r" % by["install"])
    if not by["init"]:
        p.append("veh-rand: no init line")
    for i in by["init"]:
        if i.get("mode") != "raise" or i.get("cap_applied") is not True \
                or not (lo - 1e-6 <= (i.get("factor_min") or 0) < (i.get("factor_max") or 0)
                                                     <= hi + 1e-6) or i["factor_max"] - i["factor_min"] < 0.05:
            p.append("veh-rand: init line implausible: %r" % i)
    if not by["reset"]:
        p.append("veh-rand: no reset line")
    for r in by["reset"]:
        if not (r.get("envs", 0) > 0 and lo - 1e-6 <= r.get("factor_min", -1) <= r.get("factor_max", -1) <= hi + 1e-6
                and t_lo - 1e-6 <= r.get("tau_min", -1) <= r.get("tau_max", -1) <= t_hi + 1e-6):
            p.append("veh-rand: reset line implausible: %r" % r)
            break
    if max([int(x.get("motor_updates") or 0) for x in by["reset"] + by["update"]] or [0]) <= 0:
        p.append("veh-rand: no motor-update count > 0")
    p += T2.raise_telemetry_problems(by["init"], by["reset"] + by["update"], {})   # review r1 M-1
    dby = {e: [x for x in dep if x.get("event") == e] for e in ("install", "first", "summary")}
    if len(dby["install"]) != 1 or dby["install"][0].get("settings") != dm:
        p.append("depth-model: expected one install line with the cell's settings, got %r" % dby["install"])
    if not dby["first"]:
        p.append("depth-model: no first-frame line")
    if not dby["summary"]:
        p.append("depth-model: no summary line")
    k = dm["NAVRL_DEPTH_SIGMA_D_PX"] / dm["NAVRL_DEPTH_FB_PXM"]
    for s in dby["summary"]:
        med = s.get("median_valid_z_m")
        hole = s.get("invalid_hole")
        ok = (s.get("calls") == T2.DM_MOD.STATS_CALLS and (s.get("eligible_pixels") or 0) > 0
              and hole is not None and dm["NAVRL_DEPTH_HOLE_P"] * (1 - T2.HOLE_SHARE_TOLERANCE) <= hole
              <= dm["NAVRL_DEPTH_HOLE_P"] * (1 + T2.HOLE_SHARE_TOLERANCE)
              and 0.0 <= (s.get("invalid_minz") if s.get("invalid_minz") is not None else -1) < 0.5
              and 0.0 <= (s.get("invalid_maxz") if s.get("invalid_maxz") is not None else -1) < 0.5
              and 0.0 < (s.get("mean_abs_noise_m") or 0) < 1.0
              and med is not None and 0.0 < med <= dm["NAVRL_DEPTH_MAXZ_M"]
              and s.get("sigma_z_at_median_m") is not None
              and abs(s["sigma_z_at_median_m"] - k * med * med) <= 1e-6 + 1e-6 * k * med * med)
        if not ok:
            p.append("depth-model: summary implausible: %r" % s)
    return p


def record_problems(record, expected):
    """CAM-K2 v2's vehicle-randomization record rules (tools/camk2v2_train.veh_rand_record_problems) for a cell: the
    limit raised to TW x m x g / 4 above navrl_quad's own 4 x 2 N."""
    if not isinstance(record, dict):
        return ["vehicle-randomization record missing or unreadable"]
    vr = expected["vehicle_rand"]
    p = []
    if record.get("settings") != vr:
        p.append("record settings %r differ from the cell's" % record.get("settings"))
    tw = vr["NAVRL_VEH_RAND_TW_NOMINAL"]
    if record.get("tw_nominal") != tw or record.get("cap_applied") is not True or record.get("num_motors") != 4:
        p.append("record tw_nominal / cap_applied / num_motors implausible")
    try:
        if not all(T2.MASS_RANGE_KG[0] <= m <= T2.MASS_RANGE_KG[1] for m in record["mass_kg"]):
            p.append("record mass outside %r" % (T2.MASS_RANGE_KG,))
        mass, grav, cap = record["mass_kg"], record["gravity_mps2"], record["cap_per_motor_n"]
        for c, m, g in zip(cap, mass, grav):
            if abs(c - tw * m * g / 4.0) > 1e-6 * c or abs(g - T2.G_NOMINAL) > 1e-4:
                p.append("record cap %r != %g x %r x %r / 4" % (c, tw, m, g))
        own_lo, own_hi = 8.0 / (max(mass) * max(grav)), 8.0 / (min(mass) * min(grav))    # 4 x 2 N over m g
        if (record.get("mode") != "raise" or record["thrust_limit_per_motor_n"] != cap
                or record["max_thrust_per_motor_n"] != [2.0, 2.0] or not all(c > 2.0 for c in cap)
                or not all(own_lo * (1 - 1e-6) <= o <= own_hi * (1 + 1e-6) for o in record["own_thrust_to_weight"])):
            p.append("record mode %r / limit / own T/W: not navrl_quad raised to the limit" % (record.get("mode"),))
        live = record["live_max_thrust_per_motor_n"]   # review r1 M-1: read back from the live tensor
        if not (len(live) == 2 and all(abs(v - c) <= T2.LIVE_RTOL * c for v, c in zip(live, cap))):
            p.append("record live max_thrust %r is not the limit %r" % (live, cap))
    except (KeyError, TypeError, ValueError) as e:
        p.append("record incomplete: %r" % (e,))
    return p


def receipt_problems(cell_dir, arm, seed, committed):
    """The realveh receipt, the cell receipt's NAVRL snapshot, the camera and (treatments ON) the telemetry and the
    vehicle-randomization record of one cell."""
    on = treated(arm)
    exp = expected_settings(seed, on)
    try:
        rec = json.loads((cell_dir / "realveh_receipt.json").read_text())
        receipt = json.loads((cell_dir / "receipt.json").read_text())
    except (OSError, ValueError) as e:
        return ["realveh receipt or cell receipt missing or unreadable: %s" % e]
    p = []
    s = rec.get("settings") or {}
    for key, want in exp.items():
        if s.get(key) != want:
            p.append("realveh setting %s is %r, expected %r" % (key, s.get(key), want))
    if on and not (isinstance(s.get("worst_case_thrust_to_weight"), (int, float))
                   and s["worst_case_thrust_to_weight"] >= T2.E.MIN_WORST_CASE_THRUST_TO_WEIGHT):
        p.append("worst-case thrust-to-weight %r" % s.get("worst_case_thrust_to_weight"))
    if rec.get("task_robot_name") != ROBOT:
        p.append("task robot %r is not %s" % (rec.get("task_robot_name"), ROBOT))
    abl = rec.get("ablation") or {}
    if abl.get("condition") != CONDITION or abl.get("hbeams") != 72 or abl.get("vbeams") != 4:
        p.append("ablation %r is not no_lidar on a 72 x 4 LiDAR" % ({k: abl.get(k) for k in ("condition", "hbeams",
                                                                                            "vbeams")},))
    for k in ("task", "perception"):
        v = abl.get(k)
        if not isinstance(v, str) or not v.startswith("/") or not K.is_inside(v, ROOT):
            p.append("ablation %s missing, relative or outside the tree: %r" % (k, v))
    for k in ("entry", "script"):
        if not K.is_inside(rec.get(k, ""), ROOT):
            p.append("realveh receipt %s outside the tree: %r" % (k, rec.get(k)))
    if (rec.get("overrides_after") or {}) != T2.EXPECTED_AFTER:
        p.append("overrides %r are not CAM-K1's" % rec.get("overrides_after"))
    shas = rec.get("sha256") or {}
    if set(shas) != set(committed) or any(want is None or shas.get(n) != want for n, want in committed.items()):
        p.append("receipt SHA-256 differ from the committed files")
    if on != ("vehicle_rand" in rec and "depth_model" in rec):
        p.append("receipt install records do not match treatments %s" % ("ON" if on else "OFF"))
    snap = receipt.get("env_snapshot_navrl", {})
    if snap.get("NAVRL_SENSOR_ABLATION") != CONDITION:
        p.append("snapshot NAVRL_SENSOR_ABLATION %r" % snap.get("NAVRL_SENSOR_ABLATION"))
    for k, v in R1.CAM_VALUES.items():
        if snap.get(k) != v:
            p.append("snapshot %s %r != %r" % (k, snap.get(k), v))
    want_t = treatment_env(seed) if on else {}
    got_t = {k: snap[k] for k in TREATMENT_KEYS if k in snap}
    if got_t != want_t:
        p.append("snapshot treatment keys %r != %r" % (got_t, want_t))
    if snap.get("NAVRL_REALVEH_RECEIPT") != str(cell_dir / "realveh_receipt.json") or "NAVRL_CAMTRAIN_RECEIPT" in snap:
        p.append("snapshot receipt variables are not the realveh entry's")
    if snap.get("NAVRL_ROBOT", ROBOT) != ROBOT or any(k.startswith("NAVRL_REALVEH_") and k != "NAVRL_REALVEH_RECEIPT"
                                                     for k in snap):
        p.append("snapshot names another robot or a retired NAVRL_REALVEH_* variable")
    try:
        cam = json.loads((cell_dir / K.RECORDER_SUMMARY).read_text()).get("camera", {})
        for key, env_key in (("hfov_deg", "NAVRL_CAMTRAIN_HFOV_DEG"), ("vfov_deg", "NAVRL_CAMTRAIN_VFOV_DEG")):
            if not isinstance(cam.get(key), (int, float)) or abs(cam[key] - float(R1.CAM_VALUES[env_key])) > 1e-6:
                p.append("recorder camera %s = %r" % (key, cam.get(key)))
    except (OSError, ValueError) as e:
        p.append("recorder summary unreadable: %s" % e)
    log = cell_dir / "cell.log"
    text = log.read_text(errors="replace") if log.is_file() else ""
    if "hfov=66deg" not in text:
        p.append("cell.log lacks the built camera's FOV line 'hfov=66deg'")
    p += telemetry_problems(text, exp)
    record_path = cell_dir / ("realveh_receipt.json" + T2.E.VEH_RAND_RECORD_SUFFIX)
    if on:
        p += record_problems(json.loads(record_path.read_text()) if record_path.is_file() else None, exp)
    elif record_path.exists():
        p.append("treatments OFF, but a vehicle-randomization record exists")
    return p


def run_cell(protocol, out_root, arm, density, seed, run_id, head, start, records, ckpts, committed):
    ckpt, ckpt_sha = arm_ckpt(arm, ckpts)
    on = treated(arm)
    extra = {"NAVRL_SENSOR_ABLATION": CONDITION, "CAMEVAL_PY": str(WRAPPER), "CAMK2E_CKPT": ckpt,
             "CAMK2E_CKPT_SHA": ckpt_sha, "CAMK2E_TREATMENTS": "on" if on else "off", **R1.CAM_VALUES,
             **(treatment_env(seed) if on else {})}
    rec = G.run_cell(ROOT, protocol, out_root, arm, density, seed, QUOTA, run_id, None, head, start, records,
                     G.CELL_TIMEOUT_S, extra, LAUNCHER, ckpt_sha)
    cell_dir = out_root / arm / rec["cell_id"]
    extra_problems = receipt_problems(cell_dir, arm, seed, committed) if rec["verdict"] == "OK" else []
    if extra_problems:
        rec["problems"].extend(extra_problems)
        rec["verdict"] = "FAIL"
    rec.update(arm=arm, checkpoint=ckpt, checkpoint_sha256=ckpt_sha, treatments="on" if on else "off",
               module_seeds=list(module_seeds(seed)) if on else None)
    print("   realveh receipt", arm, "OK" if not extra_problems else extra_problems[:2], flush=True)
    return rec


def rate_problems(identity_report, camk1e_raw):
    """Review r1 E-m2, repaired after review r2 R2-1: the identity cells all run at 205 bars, the slowest density, so
    their absolute time overstates an eval cell. The overhead of this unit's path (the realveh entry, the depth model)
    is measured as the ratio of the mean identity cell time to CAM-K1-E's identity cells (the same 205-bar, seed-4501
    cells), and the eval refuses unless RATE_CELLS x CAM-K1-E's mean eval cell time x that ratio x RATE_MARGIN fits
    MAX_HOURS."""
    walls = [r.get("wall_s") for r in identity_report.get("records") or []]
    if len(walls) != len(IDENTITY_PLAN) or not all(isinstance(w, (int, float)) and w > 0 for w in walls):
        return ["the identity report has no wall time for each of its %d cells" % len(IDENTITY_PLAN)]
    try:
        k1i = json.loads((pathlib.Path(camk1e_raw) / "identity_01" / "identity_report.json").read_text())
        k1e = json.loads((pathlib.Path(camk1e_raw) / "eval_01" / "eval_run_report.json").read_text())
        ref_i = [r["wall_s"] for r in k1i["records"]]
        ref_e = [r["wall_s"] for r in k1e["records"]]
        assert ref_i and ref_e
    except (OSError, ValueError, KeyError, AssertionError) as e:
        return ["rate gate: CAM-K1-E's identity or eval cell times are unreadable: %s" % e]
    ratio = (sum(walls) / len(walls)) / (sum(ref_i) / len(ref_i))
    projected = RATE_CELLS * (sum(ref_e) / len(ref_e)) * ratio * RATE_MARGIN
    if projected > MAX_HOURS * 3600:
        return ["rate gate: %d x %.0f s x ratio %.2f x %.2f = %.0f s exceeds the %.1f h cap" % (
            RATE_CELLS, sum(ref_e) / len(ref_e), ratio, RATE_MARGIN, projected, MAX_HOURS)]
    return []


def eval_plan():
    """The 60 eval cells in order: density, seed, then the main arms and, at 70 and 205 bars, the nominal arms."""
    plan = []
    for density in G.DENSITIES:
        for seed in G.SEEDS:
            for arm in MAIN_ARMS + (NOMINAL_ARMS if density in NOMINAL_DENSITIES else ()):
                plan.append((arm, density, seed))
    return plan


def preflight(protocol, approved, out_root, camk1e_raw, unit_dir=UNIT_DIR):
    head, problems = G.tree_problems(ROOT, approved)
    if len(approved or "") < 7:
        problems.append("--approved-commit must name at least 7 hex digits")
    import shutil
    free_gb = shutil.disk_usage(str(pathlib.Path.home())).free / 1e9
    if free_gb < MIN_FREE_GB:
        problems.append("free disk %.2f GB < %.1f GB" % (free_gb, MIN_FREE_GB))
    if K.git("rev-parse", "--abbrev-ref", "HEAD", root=ROOT).strip() != BRANCH:
        problems.append("branch is not " + BRANCH)
    ap = unit_dir / "execution_approval.json"
    if not ap.is_file():
        problems.append("no approval file %s" % ap)
    else:
        try:
            problems += R1.approval_problems(json.loads(ap.read_text()), head, scope=SCOPE)
        except ValueError as e:
            problems.append("approval unreadable: %s" % e)
    for rel in COMMITTED_FILES:
        if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", rel], capture_output=True).returncode:
            problems.append("%s is not committed" % rel)
    if not (ROOT / LAUNCHER).is_file() or G.sha(ROOT / LAUNCHER) != LAUNCHER_SHA:
        problems.append("CAM-K2-E launcher missing or not the pinned SHA %s" % LAUNCHER_SHA[:12])
    if not os.access(str(WRAPPER), os.X_OK):
        problems.append("tools/realveh_python is not executable")
    ev = camk1e_raw / "eval_01" / "eval_run_report.json"
    if not ev.is_file() or json.loads(ev.read_text()).get("verdict") != "COMPLETE":
        problems.append("--camk1e-raw has no COMPLETE eval_01/eval_run_report.json")
    if not (camk1e_raw / IDENTITY_REFERENCE / "episodes.jsonl").is_file():
        problems.append("--camk1e-raw lacks %s" % IDENTITY_REFERENCE)
    problems += K.static_problems(protocol, ROOT)
    if K.gpu_names() != [G.GPU]:
        problems.append("expected exactly one %s, found %s" % (G.GPU, K.gpu_names()))
    if G.gpu_apps():
        problems.append("other GPU compute processes: %s" % G.gpu_apps())
    imp, origins = K.import_problems(K.base_env(root=ROOT, inherited={k: v for k, v in os.environ.items()
                                                                       if not k.startswith("NAVRL_")}),
                                     K.launcher_path(protocol, ROOT).parent, ROOT)
    problems += imp
    if out_root.exists():
        problems.append("output already exists: %s" % out_root)
    return head, problems, origins


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=("identity", "eval"))
    ap.add_argument("--approved-commit", required=True)
    ap.add_argument("--camk1e-raw", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-hours", type=float, default=MAX_HOURS)
    args = ap.parse_args(argv)
    if args.max_hours != MAX_HOURS:                 # review r1 E-m2
        raise SystemExit("--max-hours is fixed at %g by the preregistration" % MAX_HOURS)
    out_root = pathlib.Path(args.out).resolve()
    k1e = pathlib.Path(args.camk1e_raw).resolve()
    protocol = json.loads(K.PROTOCOL.read_text())
    head, problems, origins = preflight(protocol, args.approved_commit, out_root, k1e)
    ckpts, ck_problems = checkpoints()
    problems += ck_problems
    committed = T2.committed_sha256(ROOT)
    start = G.host()
    report = {"stage": args.stage, "tree": str(ROOT), "commit": head, "prereg": PREREG, "scope": SCOPE,
              "started": G.now(), "host_start": start, "import_origins": origins, "launcher": LAUNCHER,
              "launcher_sha256": LAUNCHER_SHA, "camk1e_raw": str(k1e), "checkpoints": ckpts,
              "committed_sha256": committed, "preflight_problems": problems, "max_hours": args.max_hours,
              "env_allowlisted": {k: os.environ[k] for k in sorted(os.environ) if ENV_ALLOW.match(k)}}
    if args.stage == "eval":
        idents = sorted(out_root.parent.glob("*/identity_report.json"))
        ident = None
        if len(idents) != 1:
            problems.append("eval needs exactly one identity_report.json in %s, found %d" % (out_root.parent,
                                                                                            len(idents)))
        else:
            r = json.loads(idents[0].read_text())
            if r.get("verdict") == "IDENTITY_PASS" and r.get("commit") == head:
                ident = idents[0]
            else:
                problems.append("the identity report is not an IDENTITY_PASS on this commit")
        if list(out_root.parent.glob("*/eval_run_report.json")):
            problems.append("an eval_run_report.json already exists in %s" % out_root.parent)
        if ident is not None:
            irep = json.loads(ident.read_text())
            if irep.get("env_allowlisted") != report["env_allowlisted"]:
                problems.append("runner environment differs from the identity stage's")
            if irep.get("camk1e_raw") != str(k1e):
                problems.append("--camk1e-raw differs from the identity stage's")
            problems += rate_problems(irep, k1e)
        report["identity_report"] = str(ident)
    if problems:
        report["verdict"] = "PREFLIGHT_FAIL"
        print(json.dumps(report, indent=1, default=str))
        return 2
    out_root.mkdir(parents=True)
    budget = K.GpuBudget(args.max_hours)
    records, void = [], False
    run_id = "CAMK2E-" + args.stage.upper()
    if args.stage == "identity":
        comp = {}
        for arm, density, seed in IDENTITY_PLAN:
            rec = run_cell(protocol, out_root, arm, density, seed, run_id, head, start, records, ckpts, committed)
            if rec["verdict"] == "VOID_HOST_FAULT":
                void = True
                break
            if rec["verdict"] == "OK":
                comp[arm] = G.identity_compare(k1e / IDENTITY_REFERENCE, out_root / arm / rec["cell_id"])
        cid = CELL % (205, 4501)
        if "K1s1_on_repeat" in comp and "K1s1_on_negative" in comp:
            comp["K1s1_on_repeat"]["equals_K1s1_on_negative"] = (
                K.read_jsonl(out_root / "K1s1_on_repeat" / cid / "episodes.jsonl")
                == K.read_jsonl(out_root / "K1s1_on_negative" / cid / "episodes.jsonl"))
        if "K2s1_smoke" in comp and "K1s1_on_negative" in comp:
            comp["K2s1_smoke"]["differs_from_K1s1_on_negative"] = (
                K.read_jsonl(out_root / "K2s1_smoke" / cid / "episodes.jsonl")
                != K.read_jsonl(out_root / "K1s1_on_negative" / cid / "episodes.jsonl"))
        ok = (set(comp) == {a for a, _, _ in IDENTITY_PLAN} and comp["K1s1_off_identity"]["result_equal"]
              and comp["K1s1_off_identity"]["episodes_equal"] and not comp["K1s1_on_negative"]["episodes_equal"]
              and comp["K1s1_on_repeat"].get("equals_K1s1_on_negative") is True
              and comp["K2s1_smoke"].get("differs_from_K1s1_on_negative") is True)
        report["comparison"] = comp
        report["verdict"] = "VOID_HOST_FAULT" if void else ("IDENTITY_PASS" if ok else "IDENTITY_FAIL")
        name = "identity_report.json"
    else:
        plan = eval_plan()
        for arm, density, seed in plan:
            if budget.exhausted():
                report["budget_exhausted"] = True
                break
            rec = run_cell(protocol, out_root, arm, density, seed, run_id, head, start, records, ckpts, committed)
            if rec["verdict"] != "OK":
                void = rec["verdict"] == "VOID_HOST_FAULT"
                break
        complete = len(records) == len(plan) and all(r["verdict"] == "OK" for r in records)
        report["verdict"] = "VOID_HOST_FAULT" if void else ("COMPLETE" if complete else "INCOMPLETE")
        report["cells_expected"] = len(plan)
        name = "eval_run_report.json"
    report["records"] = records
    report["host_end"] = G.host()
    report["finished"] = G.now()
    (out_root / name).write_text(json.dumps(report, indent=1, default=str) + "\n")
    print(json.dumps({k: report[k] for k in ("stage", "verdict", "commit")}, indent=1))
    return 0 if report["verdict"] in ("IDENTITY_PASS", "COMPLETE") else 1


if __name__ == "__main__":
    raise SystemExit(main())
