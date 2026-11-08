"""CAM-K2 v2 training wrapper (docs/plans/prereg_camonly_k2_train_v2_2026-11-08.md; draft
~/motar_tmp/camk2_20261108/PREREG_V2_DRAFT.md): one seed of the 1,000-epoch camera-only continuation of CAM-K1 v1 with
the real vehicle's thrust-to-weight modelled at run time on navrl_quad (the user, 2026-11-08: 1.1 kg, 5x4.3x3 props,
Velox 2550 kV motors; T/W 4.0 nominal, INFERRED from LIGPOWER V2306 V2 2400KV / T5143S / 4S bench data, review r1
m-2): the motor model's per-motor limit is raised (NAVRL_VEH_RAND_LIMIT_MODE=raise, opt-in) to 4.0 x m x g / 4, with
vehicle (mass / motor) randomization around it and the OAK-D Pro depth model, via
train_navrl_camk2v2.sh and tools/realveh_python. A copy of tools/camk2_train.py (CAM-K2 v1, WITHDRAWN_BEFORE_RUN)
with the v2 constants; v1's file is unchanged. The robot identity stays navrl_quad, so the checkpoint lineage of F and
CAM-K1 is unchanged and NavRLTask's robot-lineage guard is never bypassed.

Seed s warm-starts from CAM-K1 v1 seed s's final checkpoint (ep28000, SHA-256 pinned below). The checkpoint is
copied into the output folder first: runner.py writes a `*_rlnorm.pth` sibling next to the checkpoint it is given
(its critic keys carry `_orig_mod.`), which must not land in CAM-K1's evidence folder.

Fail-closed preflight, before anything is created:
  - UNIT_DIR/execution_approval.json names this file's SCOPE and the commit, which is HEAD; branch camonly/dev;
    clean tracked and untracked tree; the preregistration committed;
  - exactly one RTX 3070, no GPU compute process except the user's remote desktop, no NavRL runner, no PX4 SITL or
    Gazebo process; the source checkpoint's SHA-256; at least MIN_FREE_GB free;
  - the seed's output directory does not exist (single use).
During the run: a wall-clock cap (timeout -k 120, MAX_HOURS). Afterwards, from files only: kernel faults, boot id and
online CPUs unchanged (else VOID_HOST_FAULT); exactly one new run folder whose tag matches; its
last_gen_ppo_ep_29000_*.pth (28002 for the smoke) exists; the realveh receipt holds every preregistered setting and
the committed SHA-256 of every hashed file; the vehicle-randomization record shows the cap computed from the
controller's mass and raised (TW 4.0 x m x g / 4, 2.4525 N per motor at 0.25 kg, above navrl_quad's own 2 N), with the
live max_thrust read back equal to it; the telemetry carries that read-back and the raise counters (review r1 M-1);
the trained checkpoint's recorded robot
is navrl_quad with the committed navrl_quad config / URDF SHA-256 (read from the checkpoint's pickle without torch);
the source copy is unchanged; the task's FOV line is in the logs; the tracked tree is unchanged.
usage: camk2v2_train.py {1,2} [--smoke] [--max-hours 2.5]
"""
import argparse
import collections
import hashlib
import importlib.util
import json
import os
import pathlib
import pickle
import re
import shutil
import subprocess
import sys
import time
import zipfile

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import gr_run as R                                          # noqa: E402  (host, gpu, tree helpers)

PREREG = "docs/plans/prereg_camonly_k2_train_v2_2026-11-08.md"
SCOPE = ("CAM-K2 v2: camera-only (no_lidar) continuation of CAM-K1 v1 seed s ep28000 for 1,000 epochs "
         "(28001..29000) on navrl_quad (robot identity and lineage unchanged) with the real 1.1 kg vehicle's "
         "thrust-to-weight modelled as a raised per-motor thrust limit at T/W 4.0 (the thrust-to-weight of a 1.1 kg "
         "vehicle at T/W 4.0: maximum thrust acceleration and hover fraction; rotational dynamics and drag "
         "navrl_quad's), "
         "vehicle randomization (TW_SCALE 0.15, MOTOR_IMBAL 0.05, tau 0.02-0.04 s) and the OAK-D Pro depth model "
         "(800P extended disparity + subpixel: MinZ 0.4 m, sigma_d 0.15 px, f*B 66.1875 px m, MaxZ 20 m, hole "
         "probability 0.02, invalid = no return), "
         "fixed 205 bars, LR 5e-6, camera FOV 66x54 deg, yaw_align_weight 0.6, visibility_bonus 0.03; a 2-epoch "
         "development smoke (seed 1, not a result) first, then seed 1 then seed 2, each once")
APPROVAL_KEYS = ("approved_by", "approved_at", "scope", "commit")
BRANCH = "camonly/dev"                 # the unit is committed and run from MOTAR-camonly
UNIT_DIR = pathlib.Path.home() / "motar_tmp" / "camk2v2_20261108"
CAMK1_DIR = pathlib.Path.home() / "motar_tmp" / "camk1_20261106"
RL = ROOT / "aerial_gym" / "rl_training" / "rl_games"
LAUNCHER = RL / "train_navrl_camk2v2.sh"
SOURCES = {1: {"path": CAMK1_DIR / "train_s1/checkpoint/last_gen_ppo_ep_28000_rew_-17.796482.pth",
               "sha256": "d7bf51a89b0e8124687e618cf8d95735713d36af2eb34fc5b8819efb14715beb"},
           2: {"path": CAMK1_DIR / "train_s2/checkpoint/last_gen_ppo_ep_28000_rew_-10.653644.pth",
               "sha256": "5128bf2e0461fb45c6b8c4463d4d0a950bb88c0d28923c41d02d99da1102a4b7"}}
SOURCE_EPOCH = 28000
TAG = "v2-camk2v2-ep28000-205bars-main-riskcap-camonly-tw40-s%d"
FINAL_EPOCH = 29000
SMOKE_EPOCH = 28002
MAX_HOURS = 2.5
FOV_LOG_LINE = "hfov=66deg"            # navrl_task.py prints the built camera's FOV at task construction
MIN_FREE_GB = 1.0                      # each 1,000-epoch seed writes about 0.18 GB
ENV_STRIP = ("NAVRL_", "CAMK1_", "CAMK2_", "CAMK2V2_", "CAMTRAIN_", "REALVEH_", "S3B_")   # the launcher sets every one it needs
ROBOT = "navrl_quad"                   # Policy F's and CAM-K1's vehicle identity (the checkpoint lineage)
ROBOT_CONFIG_REL = "aerial_gym/config/robot_config/navrl_quad_config.py"
ROBOT_URDF_REL = "resources/robots/quad/quad_navrl_collide.urdf"
ROBOT_URDF_FILE = "quad_navrl_collide.urdf"
EXPECTED_AFTER = {"detector_hfov_deg": 66.0, "detector_vfov_deg": 54.0, "yaw_align_weight": 0.6,
                  "visibility_bonus": 0.03}
EXPECTED_CAMTRAIN = {"NAVRL_CAMTRAIN_HFOV_DEG": 66.0, "NAVRL_CAMTRAIN_VFOV_DEG": 54.0,
                     "NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT": 0.6, "NAVRL_CAMTRAIN_VISIBILITY_BONUS": 0.03}
# CAM-K2 v2 (2026-11-08): the real vehicle's T/W (4.0 nominal, INFERRED; review r1 m-2) by raising the limit, opt-in
# (review r1 m-1); tau 0.02-0.04 s.
VEH_RAND_LIMIT_MODE = "raise"
VEH_RAND = {"NAVRL_VEH_RAND_TW_NOMINAL": 4.0, "NAVRL_VEH_RAND_TW_SCALE": 0.15, "NAVRL_VEH_RAND_MOTOR_IMBAL": 0.05,
            "NAVRL_VEH_RAND_TAU_MIN": 0.02, "NAVRL_VEH_RAND_TAU_MAX": 0.04}
# OAK-D Pro 800P, extended disparity + subpixel: f*B = 882.5 px x 0.075 m = 66.1875 px m (preregistration, m-4).
DEPTH_MODEL = {"NAVRL_DEPTH_MINZ_M": 0.4, "NAVRL_DEPTH_MAXZ_M": 20.0, "NAVRL_DEPTH_SIGMA_D_PX": 0.15,
               "NAVRL_DEPTH_FB_PXM": 882.5 * 0.075, "NAVRL_DEPTH_HOLE_P": 0.02}
VEH_RAND_SEED = {1: 4911, 2: 4912}     # the smoke runs seed 1's values
DEPTH_SEED = {1: 4921, 2: 4922}
G_NOMINAL = 9.81
SMOKE_RATE_MARGIN = 1.25               # seeds refuse if 1,000 epochs x the smoke's last epoch time x this > MAX_HOURS
SEED_EPOCHS = FINAL_EPOCH - SOURCE_EPOCH
HOLE_SHARE_TOLERANCE = 0.5             # the depth telemetry's hole share must lie within HOLE_P x (1 +- this)
MASS_RANGE_KG = (0.249, 0.251)         # navrl_quad's PhysX total mass as the controller reads it (URDF sum 0.25 kg)


def _load_private(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


E = _load_private("camk2_realveh_entry", ROOT / "tools" / "realveh_entry.py")     # stdlib only at import
# The two modules' telemetry constants (stdlib only at import; torch is imported inside their functions).
VR_MOD = _load_private("camk2_vehicle_rand", ROOT / "aerial_gym/task/navrl_task/navrl_vehicle_rand.py")
DM_MOD = _load_private("camk2_depth_model", ROOT / "aerial_gym/task/navrl_task/navrl_depth_model.py")


def expected_settings(seed):
    return {"vehicle_rand": dict(VEH_RAND, NAVRL_VEH_RAND_SEED=VEH_RAND_SEED[seed],
                                 NAVRL_VEH_RAND_LIMIT_MODE=VEH_RAND_LIMIT_MODE),
            "depth_model": dict(DEPTH_MODEL, NAVRL_DEPTH_SEED=DEPTH_SEED[seed]),
            "camtrain": dict(EXPECTED_CAMTRAIN)}


def hashed_relpaths():
    """The receipt's SHA-256 names and their tracked paths (the entry's own list)."""
    return {k: pathlib.Path(v).resolve().relative_to(ROOT.resolve()).as_posix() for k, v in E.HASHED.items()}


def committed_sha256(root=ROOT):
    """SHA-256 of each hashed file's bytes at HEAD (git show), not of the working copy."""
    out = {}
    for name, rel in hashed_relpaths().items():
        done = subprocess.run(["git", "-C", str(root), "show", "HEAD:%s" % rel], capture_output=True)
        out[name] = hashlib.sha256(done.stdout).hexdigest() if done.returncode == 0 else None
    return out


def approval_problems(approval, head, scope=SCOPE):
    p = []
    if not isinstance(approval, dict):
        return ["approval is not a JSON object"]
    for k in APPROVAL_KEYS:
        if not isinstance(approval.get(k), str) or not approval[k].strip():
            p.append("approval key %s missing or empty" % k)
    if approval.get("scope") != scope:
        p.append("approval scope differs from the code's SCOPE")
    if not head or approval.get("commit") not in (head, head[:7]):
        p.append("approval commit %r is not HEAD %s" % (approval.get("commit"), head))
    return p


def receipt_problems(receipt, seed, committed):
    """The realveh receipt against the preregistered settings and the committed SHA-256 of every hashed file."""
    if not isinstance(receipt, dict):
        return ["realveh receipt missing or unreadable"]
    p = []
    s = receipt.get("settings") or {}
    for key, want in expected_settings(seed).items():
        if s.get(key) != want:
            p.append("setting %s is %r, preregistered %r" % (key, s.get(key), want))
    worst = s.get("worst_case_thrust_to_weight")
    if not isinstance(worst, (int, float)) or worst < E.MIN_WORST_CASE_THRUST_TO_WEIGHT:
        p.append("worst-case thrust-to-weight %r below %g" % (worst, E.MIN_WORST_CASE_THRUST_TO_WEIGHT))
    if receipt.get("task_robot_name") != ROBOT:
        p.append("task robot is %r, expected %s" % (receipt.get("task_robot_name"), ROBOT))
    if (receipt.get("vehicle_rand") or {}).get("settings") != expected_settings(seed)["vehicle_rand"]:
        p.append("vehicle randomization not installed with the preregistered settings")
    if (receipt.get("depth_model") or {}).get("settings") != expected_settings(seed)["depth_model"]:
        p.append("depth model not installed with the preregistered settings")
    if (receipt.get("ablation") or {}).get("condition") != "no_lidar":
        p.append("ablation condition is not no_lidar: %r" % (receipt.get("ablation") or {}).get("condition"))
    after = receipt.get("overrides_after") or {}
    for k, v in EXPECTED_AFTER.items():
        if after.get(k) != v:
            p.append("override %s is %r, preregistered %r" % (k, after.get(k), v))
    shas = receipt.get("sha256") or {}
    if set(shas) != set(committed):
        p.append("receipt hashes %s, expected %s" % (sorted(shas), sorted(committed)))
    for name, want in committed.items():
        if want is None or shas.get(name) != want:
            p.append("SHA-256 of %s is %r, committed %r" % (name, shas.get(name), want))
    env = receipt.get("env_navrl") or {}
    for k, v in (("NAVRL_SENSOR_ABLATION", "no_lidar"), ("NAVRL_VEH_RAND", "1"), ("NAVRL_DEPTH_MODEL", "oakd"),
                 ("NAVRL_PERCEPTION_PERTURB", "0")):
        if env.get(k) != v:
            p.append("environment %s=%r, expected %r" % (k, env.get(k), v))
    if env.get("NAVRL_ROBOT", ROBOT) != ROBOT:
        p.append("environment NAVRL_ROBOT=%r, expected unset or %s" % (env.get("NAVRL_ROBOT"), ROBOT))
    stale = sorted(k for k in env if k.startswith("NAVRL_REALVEH_") and k != "NAVRL_REALVEH_RECEIPT")
    if stale:
        p.append("retired settings in the environment: %s" % stale)
    return p


def veh_rand_record_problems(record, seed):
    """The cap the vehicle randomization computed at init_tensors: TW_NOMINAL x m x g / 4 from the controller's own
    mass and gravity, applied, with navrl_quad's mass and T/W."""
    if not isinstance(record, dict):
        return ["vehicle-randomization record missing or unreadable"]
    p = []
    tw = VEH_RAND["NAVRL_VEH_RAND_TW_NOMINAL"]
    if record.get("settings") != expected_settings(seed)["vehicle_rand"]:
        p.append("record settings %r differ from the preregistered ones" % record.get("settings"))
    if record.get("tw_nominal") != tw or record.get("cap_applied") is not True or record.get("num_motors") != 4:
        p.append("record tw_nominal / cap_applied / num_motors: %r / %r / %r"
                 % (record.get("tw_nominal"), record.get("cap_applied"), record.get("num_motors")))
    try:
        mass, g, cap = record["mass_kg"], record["gravity_mps2"], record["cap_per_motor_n"]
        own, max_thrust = record["own_thrust_to_weight"], record["max_thrust_per_motor_n"]
        if not all(MASS_RANGE_KG[0] <= m <= MASS_RANGE_KG[1] for m in mass):
            p.append("controller mass %r outside %r" % (mass, MASS_RANGE_KG))
        if not all(abs(x - G_NOMINAL) < 1e-4 for x in g):
            p.append("gravity %r is not %g" % (g, G_NOMINAL))
        for c, m, x in zip(cap, mass, g):
            if abs(c - tw * m * x / 4.0) > 1e-6 * c:
                p.append("cap %r != %g x %r x %r / 4" % (c, tw, m, x))
        limit = record["thrust_limit_per_motor_n"]
        own_lo, own_hi = 8.0 / (max(mass) * max(g)), 8.0 / (min(mass) * min(g))   # 4 x 2 N over m g
        if record.get("mode") != "raise" or not all(own_lo * (1 - 1e-6) <= o <= own_hi * (1 + 1e-6) for o in own) \
                or max_thrust != [2.0, 2.0] or limit != cap or not all(c > 2.0 for c in cap):
            p.append("record mode %r, own T/W %r, max thrust %r or limit %r: not navrl_quad raised to the limit"
                     % (record.get("mode"), own, max_thrust, limit))
        live = record["live_max_thrust_per_motor_n"]    # read back from the motor model's tensor (review r1 M-1)
        if not (len(live) == 2 and all(abs(v - c) <= LIVE_RTOL * c for v, c in zip(live, cap))):
            p.append("record live max_thrust %r is not the limit %r: the raise is not shown to have taken effect"
                     % (live, cap))
    except (KeyError, TypeError, ValueError) as exc:
        p.append("record incomplete: %r" % (exc,))
    return p


LIVE_RTOL = 1e-6                       # the live float32 max_thrust against the float64 limit


def raise_telemetry_problems(inits, lines, details):
    """Review r1 M-1: every reset / update line carries the raise counters and a live read-back of max_thrust equal to
    the init line's limit; at least one comes after motor updates; the counts are consistent. The shares of commands
    above navrl_quad's own 2 N and at the limit are descriptive (details["raise"])."""
    p = []
    limit = (inits[-1].get("cap_per_motor_n") if inits else None) or []
    keys = ("cmd_elements", "cmd_above_own_max", "cmd_at_limit", "cmd_max_n", "live_max_thrust_n")
    if not lines or any(not all(k in x for k in keys) for x in lines):
        return ["veh-rand: reset / update lines without the raise counters and the live max_thrust read-back"]
    for x in lines:
        live = x["live_max_thrust_n"]
        if len(limit) != 2 or not isinstance(live, list) or len(live) != 2 \
                or not all(abs(v - c) <= LIVE_RTOL * c for v, c in zip(live, limit)):
            p.append("veh-rand: live max_thrust %r is not the limit %r" % (live, limit))
            break
        n, above, at = x["cmd_elements"], x["cmd_above_own_max"], x["cmd_at_limit"]
        if not (isinstance(n, int) and isinstance(above, int) and isinstance(at, int) and 0 <= at <= above <= n):
            p.append("veh-rand: raise counters inconsistent: %r" % ({k: x[k] for k in keys},))
            break
    last = max(lines, key=lambda x: x["cmd_elements"])
    if last["cmd_elements"] <= 0:
        p.append("veh-rand: no raise-counter line after a motor update")
    else:
        details["raise"] = {"cmd_elements": last["cmd_elements"], "cmd_above_own_max": last["cmd_above_own_max"],
                            "cmd_at_limit": last["cmd_at_limit"], "cmd_max_n": last["cmd_max_n"],
                            "share_above_own_max": last["cmd_above_own_max"] / last["cmd_elements"],
                            "share_at_limit": last["cmd_at_limit"] / last["cmd_elements"],
                            "live_max_thrust_n": last["live_max_thrust_n"]}
    return p


def robot_problems(state, committed, what):
    p = []
    for k, want in (("cfg_robot_name", ROBOT), ("cfg_robot_config_path", ROBOT_CONFIG_REL),
                    ("cfg_robot_asset_path", ROBOT_URDF_REL), ("cfg_robot_asset_file", ROBOT_URDF_FILE),
                    ("cfg_robot_config_sha256", committed.get("robot_config")),
                    ("cfg_robot_asset_sha256", committed.get("robot_urdf"))):
        if want is None or (state or {}).get(k) != want:
            p.append("%s %s is %r, expected %r" % (what, k, (state or {}).get(k), want))
    return p


class _Opaque:
    """Stands in for every non-builtin object of a checkpoint pickle (tensors, storages, numpy scalars)."""

    def __init__(self, *args, **kwargs):
        pass

    def __setstate__(self, state):
        pass


class _MetadataUnpickler(pickle.Unpickler):
    _SAFE = {("collections", "OrderedDict"): collections.OrderedDict}

    def find_class(self, module, name):
        return self._SAFE.get((module, name), _Opaque)

    def persistent_load(self, pid):
        return None


def checkpoint_metadata(path):
    """`epoch` and `env_state` of a torch zip checkpoint, read from its data.pkl without torch: every tensor and
    every class becomes an inert placeholder, so no code from the file runs."""
    with zipfile.ZipFile(str(path)) as z:
        names = [n for n in z.namelist() if n.endswith("/data.pkl")]
        if len(names) != 1:
            raise ValueError("%s: expected one data.pkl, found %s" % (path, names))
        obj = _MetadataUnpickler(z.open(names[0])).load()
    if not isinstance(obj, dict):
        raise ValueError("%s: checkpoint root is not a dict" % path)
    env_state = obj.get("env_state")
    return {"epoch": obj.get("epoch"),
            "env_state": {k: v for k, v in (env_state or {}).items() if isinstance(v, (str, int, float, bool))}}


def checkpoint_problems(meta, final_epoch, committed):
    if not isinstance(meta, dict):
        return ["checkpoint metadata unreadable"]
    p = [] if meta.get("epoch") == final_epoch else ["checkpoint epoch %r, expected %d" % (meta.get("epoch"),
                                                                                         final_epoch)]
    return p + robot_problems(meta.get("env_state"), committed, "trained checkpoint")


def telemetry_lines(texts, prefix):
    """Every JSON object logged with `prefix` in the given log texts, in order."""
    out = []
    for text in texts:
        for line in text.splitlines():
            i = line.find(prefix)
            if i < 0:
                continue
            try:
                out.append(json.loads(line[i + len(prefix):]))
            except ValueError:
                continue
    return out


def telemetry_problems(texts, seed):
    """The modules' own lines must show that both treatments ran, with plausible values (module docstrings)."""
    p, details = [], {}
    exp = expected_settings(seed)
    lo = (1.0 - VEH_RAND["NAVRL_VEH_RAND_TW_SCALE"]) * (1.0 - VEH_RAND["NAVRL_VEH_RAND_MOTOR_IMBAL"])
    hi = (1.0 + VEH_RAND["NAVRL_VEH_RAND_TW_SCALE"]) * (1.0 + VEH_RAND["NAVRL_VEH_RAND_MOTOR_IMBAL"])
    t_lo, t_hi = VEH_RAND["NAVRL_VEH_RAND_TAU_MIN"], VEH_RAND["NAVRL_VEH_RAND_TAU_MAX"]
    veh = telemetry_lines(texts, VR_MOD.LOG_PREFIX)
    by = {e: [x for x in veh if x.get("event") == e] for e in ("install", "init", "reset")}
    details["veh_rand"] = {"install": by["install"][-1:], "init": by["init"][-1:], "reset_lines": len(by["reset"]),
                           "last_reset": by["reset"][-1:]}
    if len(by["install"]) != 1 or by["install"][0].get("settings") != exp["vehicle_rand"]:
        p.append("veh-rand: expected one install line with the preregistered settings, got %r" % by["install"])
    if not by["init"]:
        p.append("veh-rand: no init line")
    for i in by["init"]:                       # one per robot instance (one expected)
        if i.get("mode") != "raise" or i.get("cap_applied") is not True or not (lo - 1e-6 <= (i.get("factor_min") or 0) < (i.get("factor_max") or 0)
                                                     <= hi + 1e-6):
            p.append("veh-rand: init line implausible: %r" % i)
        elif i["factor_max"] - i["factor_min"] < 0.05:
            p.append("veh-rand: initial factors span only %.4f" % (i["factor_max"] - i["factor_min"]))
    if not by["reset"]:
        p.append("veh-rand: no reset line: the reset redraws are not shown to have run")
    updates = [x for x in veh if x.get("event") == "update"]
    details["veh_rand"]["motor_updates_max"] = max([int(x.get("motor_updates") or 0) for x in by["reset"] + updates]
                                                   or [0])
    if details["veh_rand"]["motor_updates_max"] <= 0:
        p.append("veh-rand: no motor-update count > 0: the cap and factors are not shown to have been applied")
    details["veh_rand"]["raise"] = None
    p += raise_telemetry_problems(by["init"], by["reset"] + updates, details["veh_rand"])
    for r in by["reset"]:
        if not (r.get("envs", 0) > 0 and r.get("envs_total", 0) >= r.get("envs", 0)
                and lo - 1e-6 <= r.get("factor_min", -1) <= r.get("factor_max", -1) <= hi + 1e-6
                and t_lo - 1e-6 <= r.get("tau_min", -1) <= r.get("tau_max", -1) <= t_hi + 1e-6):
            p.append("veh-rand: reset line implausible: %r" % r)
            break
    dep = telemetry_lines(texts, DM_MOD.LOG_PREFIX)
    dby = {e: [x for x in dep if x.get("event") == e] for e in ("install", "first", "summary", "heartbeat")}
    details["depth_model"] = {"install": dby["install"][-1:], "first": dby["first"][-1:], "summary": dby["summary"][-1:],
                              "heartbeats": len(dby["heartbeat"]),
                              "max_calls": max([x.get("calls", 0) for x in dep] or [0])}
    if len(dby["install"]) != 1 or dby["install"][0].get("settings") != exp["depth_model"]:
        p.append("depth-model: expected one install line with the preregistered settings, got %r" % dby["install"])
    if not dby["first"]:
        p.append("depth-model: no first-frame line")
    if not dby["summary"]:
        p.append("depth-model: no summary line (written after %d frames)" % DM_MOD.STATS_CALLS)
    for s in dby["summary"]:                   # one per perception instance (one expected)
        hole_p = DEPTH_MODEL["NAVRL_DEPTH_HOLE_P"]
        k = DEPTH_MODEL["NAVRL_DEPTH_SIGMA_D_PX"] / DEPTH_MODEL["NAVRL_DEPTH_FB_PXM"]
        med = s.get("median_valid_z_m")
        checks = (
            ("calls", s.get("calls") == DM_MOD.STATS_CALLS),
            ("eligible pixels", (s.get("eligible_pixels") or 0) > 0),
            ("hole share", hole_p * (1 - HOLE_SHARE_TOLERANCE) <= (s.get("invalid_hole") or -1)
             <= hole_p * (1 + HOLE_SHARE_TOLERANCE)),
            ("MinZ share", 0.0 <= (s.get("invalid_minz") if s.get("invalid_minz") is not None else -1) < 0.5),
            ("MaxZ share", 0.0 <= (s.get("invalid_maxz") if s.get("invalid_maxz") is not None else -1) < 0.5),
            ("mean |noise|", 0.0 < (s.get("mean_abs_noise_m") or 0) < 1.0),
            ("median z", med is not None and 0.0 < med <= DEPTH_MODEL["NAVRL_DEPTH_MAXZ_M"]),
            ("sigma at median z", med is not None and s.get("sigma_z_at_median_m") is not None
             and abs(s["sigma_z_at_median_m"] - k * med * med) <= 1e-6 + 1e-6 * k * med * med),
        )
        bad = [name for name, ok in checks if not ok]
        if bad:
            p.append("depth-model: summary implausible (%s): %r" % (", ".join(bad), s))
    return p, details


_CRASHDIAG = re.compile(r"NavRL crashdiag \| bar_contact=([0-9.]+).*? below=([0-9.]+).*? above=([0-9.]+) "
                        r"oob=([0-9.]+).*?\(n_crash=(\d+)\)")
_STEP_TIME = re.compile(r"step time \(s\)\s*:\s*([0-9.]+)")


def crash_causes(texts):
    """Per-cause shares of the crash terminations from the task's `NavRL crashdiag` lines (descriptive, not a gate)."""
    rows = []
    for text in texts[:1]:                     # the launcher log holds every line once (the session log repeats it)
        for m in _CRASHDIAG.finditer(text):
            rows.append({"bar_contact": float(m.group(1)), "below": float(m.group(2)), "above": float(m.group(3)),
                         "oob": float(m.group(4)), "n_crash": int(m.group(5))})
    if not rows:
        return "NOT_RECORDED"
    last10 = rows[-10:]
    mean = {k: sum(r[k] for r in last10) / len(last10) for k in ("bar_contact", "below", "above", "oob")}
    return {"windows": len(rows), "first": rows[0], "last": rows[-1], "mean_last_10": mean}


def epoch_step_times(texts):
    return [float(x) for x in _STEP_TIME.findall(texts[0])] if texts else []


def smoke_rate_problems(smoke_report, max_hours=MAX_HOURS):
    """Seeds refuse if SEED_EPOCHS x the smoke's last epoch time x SMOKE_RATE_MARGIN exceeds the wall cap."""
    times = (smoke_report or {}).get("epoch_step_times_s") or []
    if not times:
        return ["the smoke recorded no epoch step time"]
    projected = SEED_EPOCHS * float(times[-1]) * SMOKE_RATE_MARGIN
    if projected > max_hours * 3600.0:
        return ["projected %d epochs x %.3f s x %.2f = %.0f s exceeds the %.1f h cap"
                % (SEED_EPOCHS, times[-1], SMOKE_RATE_MARGIN, projected, max_hours)]
    return []


def write_manifest(dst, seed, head, details, out, verdict):
    """Sidecar next to the checkpoint copy: the checkpoint's env_state cannot tell the CAM-K2 treatments apart."""
    receipt, record = out / "realveh_receipt.json", pathlib.Path(str(out / "realveh_receipt.json") + E.VEH_RAND_RECORD_SUFFIX)
    manifest = {"unit": "CAM-K2 v2", "seed": seed, "commit": head, "prereg": PREREG, "scope": SCOPE, "verdict": verdict,
                "checkpoint": dst.name, "checkpoint_sha256": R.sha(dst),
                "source": {"path": str(SOURCES[seed]["path"]), "sha256": SOURCES[seed]["sha256"]},
                "robot": ROBOT, "treatments": expected_settings(seed),
                "realveh_receipt_sha256": R.sha(receipt) if receipt.is_file() else None,
                "veh_rand_record_sha256": R.sha(record) if record.is_file() else None,
                "veh_rand_cap_per_motor_n": ((details.get("veh_rand_record") or {}).get("cap_per_motor_n")),
                "note": "env_state records robot navrl_quad like CAM-K1; bind this manifest in CAM-K2-E"}
    path = dst.parent / (dst.name + ".manifest.json")
    with open(str(path), "x") as fh:
        json.dump(manifest, fh, indent=1, sort_keys=True)
        fh.write("\n")
    return {"file": path.name, "sha256": R.sha(path)}


def smoke_ok(smoke_dir, head=None):
    """Seeds 1 and 2 need the development smoke to have read SMOKE_OK at this same HEAD."""
    f = pathlib.Path(smoke_dir) / "train_report.json"
    if not f.is_file():
        return False
    r = json.loads(f.read_text())
    head = head or R.K.git("rev-parse", "HEAD", root=ROOT).strip()
    return r.get("verdict") == "SMOKE_OK" and r.get("development_smoke") is True and r.get("commit") == head


def sim_processes():
    out = subprocess.run(["pgrep", "-a", "-f", "gzserver|px4_sitl_default/bin/px4"], capture_output=True,
                         text=True).stdout
    return [ln for ln in out.splitlines() if ln.strip() and "pgrep" not in ln]


def source_problems(seed):
    src = SOURCES[seed]
    if not src["path"].is_file():
        return ["source checkpoint missing: %s" % src["path"]]
    if R.sha(src["path"]) != src["sha256"]:
        return ["source checkpoint SHA mismatch: %s" % src["path"]]
    return []


def preflight(seed, out):
    head, problems = R.tree_problems(ROOT, R.K.git("rev-parse", "HEAD", root=ROOT).strip())
    if R.K.git("rev-parse", "--abbrev-ref", "HEAD", root=ROOT).strip() != BRANCH:
        problems.append("branch is not %s" % BRANCH)
    ap = UNIT_DIR / "execution_approval.json"
    if not ap.is_file():
        problems.append("no approval file %s" % ap)
    else:
        problems += approval_problems(json.loads(ap.read_text()), head)
    if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", PREREG], capture_output=True).returncode:
        problems.append("preregistration %s is not committed" % PREREG)
    if R.K.gpu_names() != [R.GPU]:
        problems.append("expected exactly one %s" % R.GPU)
    if R.gpu_apps():
        problems.append("other GPU compute processes: %s" % R.gpu_apps())
    if subprocess.run(["pgrep", "-f", "runner.py .*--task navrl_task"], capture_output=True).stdout.strip():
        problems.append("a NavRL runner process is already active")
    if sim_processes():
        problems.append("a PX4 SITL or Gazebo process is running: %s" % sim_processes()[:2])
    problems += source_problems(seed)
    if out.exists():
        problems.append("output exists (single use): %s" % out)
    free_gb = shutil.disk_usage(str(ROOT)).free / 1e9
    if free_gb < MIN_FREE_GB:
        problems.append("only %.2f GB free on disk (need %.1f)" % (free_gb, MIN_FREE_GB))
    return head, problems


def clean_env():
    """The caller's environment without any NAVRL_/CAMK1_/CAMK2_/CAMTRAIN_/REALVEH_/S3B_ variable."""
    return {k: v for k, v in os.environ.items() if not k.startswith(ENV_STRIP)}


def run_dirs():
    runs = RL / "runs"
    return {p.name for p in runs.iterdir()} if runs.is_dir() else set()


def post_run_problems(out, seed, smoke, before_dirs, committed, rc, start, end, head=None):
    """Every post-run check, from files only. Returns (problems, host_ok, details)."""
    tag = TAG % seed + ("-smoke" if smoke else "")
    final_epoch = SMOKE_EPOCH if smoke else FINAL_EPOCH
    details, problems = {}, []
    host_ok = all(end.get(k) == start.get(k) for k in ("fault_count", "boot_id", "online"))
    if not host_ok:
        problems.append("HOST state changed during training")
    new = sorted(p for p in run_dirs() if p not in before_dirs)
    details["new_run_dirs"] = new
    if len(new) != 1 or not new[0].endswith(tag):
        problems.append("expected exactly one new run folder ending in %s" % tag)
    ckpts = sorted((RL / "runs" / new[0] / "nn").glob("last_gen_ppo_ep_%d_*.pth" % final_epoch)) if len(new) == 1 else []
    if len(ckpts) != 1:
        problems.append("missing last_gen_ppo_ep_%d_*.pth" % final_epoch)
    if rc != 0:
        problems.append("launcher exit %d" % rc)
    receipt_path = out / "realveh_receipt.json"
    receipt = json.loads(receipt_path.read_text()) if receipt_path.is_file() else None
    details["realveh_receipt"] = receipt
    problems += receipt_problems(receipt, seed, committed)
    record_path = pathlib.Path(str(receipt_path) + E.VEH_RAND_RECORD_SUFFIX)
    record = json.loads(record_path.read_text()) if record_path.is_file() else None
    details["veh_rand_record"] = record
    problems += veh_rand_record_problems(record, seed)
    copy = out / "source_checkpoint" / SOURCES[seed]["path"].name
    if not copy.is_file() or R.sha(copy) != SOURCES[seed]["sha256"]:
        problems.append("the source checkpoint copy changed or is missing: %s" % copy)
    if ckpts:
        meta = checkpoint_metadata(ckpts[0])
        details["checkpoint_robot"] = {k: v for k, v in meta["env_state"].items() if k.startswith("cfg_robot")}
        details["checkpoint_epoch"] = meta["epoch"]
        problems += checkpoint_problems(meta, final_epoch, committed)
        dst = out / "checkpoint" / ckpts[0].name
        dst.parent.mkdir(exist_ok=True)
        shutil.copy2(ckpts[0], dst)
        details["checkpoint"] = {"run_dir": str(RL / "runs" / new[0]), "file": ckpts[0].name,
                                 "sha256": R.sha(ckpts[0]), "copy": str(dst), "copy_sha256": R.sha(dst)}
        logs = sorted((RL / "train_session_logs").glob("camk2v2_s%d%s_*.log" % (seed, "-smoke" if smoke else "")))
        if logs:
            shutil.copy2(logs[-1], out / logs[-1].name)
            details["session_log"] = {"file": logs[-1].name, "sha256": R.sha(logs[-1])}
    texts = [(out / "launcher.log").read_text(errors="replace") if (out / "launcher.log").is_file() else ""]
    if details.get("session_log"):
        texts.append((out / details["session_log"]["file"]).read_text(errors="replace"))
    details["fov_log_line_found"] = any(FOV_LOG_LINE in x for x in texts)
    if not details["fov_log_line_found"]:
        problems.append("the task's startup line %r is in neither log: the FOV override is not shown to have reached "
                        "the task" % FOV_LOG_LINE)
    tele_problems, details["telemetry"] = telemetry_problems(texts[:1], seed)
    problems += tele_problems
    details["crash_causes"] = crash_causes(texts)
    details["epoch_step_times_s"] = epoch_step_times(texts)
    if len(details["epoch_step_times_s"]) < (2 if smoke else 1):
        problems.append("the log holds %d epoch step-time lines" % len(details["epoch_step_times_s"]))
    return problems, host_ok, details


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("seed", type=int, choices=(1, 2))
    ap.add_argument("--max-hours", type=float, default=MAX_HOURS)
    ap.add_argument("--smoke", action="store_true", help="the 2-epoch development smoke (seed 1 only; not a result)")
    args = ap.parse_args(argv)
    if args.smoke and args.seed != 1:
        raise SystemExit("the smoke runs seed 1 only")
    if args.max_hours != MAX_HOURS:
        raise SystemExit("--max-hours is fixed at %g by the preregistration" % MAX_HOURS)
    out = UNIT_DIR / ("train_smoke" if args.smoke else "train_s%d" % args.seed)
    if not args.smoke and not smoke_ok(UNIT_DIR / "train_smoke"):
        print(json.dumps({"seed": args.seed, "verdict": "PREFLIGHT_FAIL",
                          "preflight_problems": ["the development smoke did not read SMOKE_OK"]}, indent=1))
        return 2
    if not args.smoke:
        rate = smoke_rate_problems(json.loads((UNIT_DIR / "train_smoke" / "train_report.json").read_text()))
        if rate:
            print(json.dumps({"seed": args.seed, "verdict": "PREFLIGHT_FAIL", "preflight_problems": rate}, indent=1))
            return 2
    head, problems = preflight(args.seed, out)
    start = R.host()
    report = {"seed": args.seed, "commit": head, "prereg": PREREG, "scope": SCOPE, "host_start": start,
              "preflight_problems": problems, "started": R.now(), "source": {k: str(v) for k, v in
                                                                             SOURCES[args.seed].items()}}
    if problems:
        report["verdict"] = "PREFLIGHT_FAIL"
        print(json.dumps(report, indent=1, default=str))
        return 2
    committed = committed_sha256()
    out.mkdir(parents=True)
    copy = out / "source_checkpoint" / SOURCES[args.seed]["path"].name
    copy.parent.mkdir()
    shutil.copy2(SOURCES[args.seed]["path"], copy)
    if R.sha(copy) != SOURCES[args.seed]["sha256"]:
        report.update(verdict="PREFLIGHT_FAIL", preflight_problems=["source checkpoint copy SHA mismatch"])
        (out / "train_report.json").write_text(json.dumps(report, indent=1, default=str) + "\n")
        return 2
    before = run_dirs()
    t0 = time.time()
    env = dict(clean_env(), NAVRL_REALVEH_RECEIPT=str(out / "realveh_receipt.json"), CAMK2V2_SOURCE_CKPT=str(copy),
               CUDA_VISIBLE_DEVICES="0", CAMK2V2_SMOKE="1" if args.smoke else "0")
    with (out / "launcher.log").open("w") as fh:
        rc = subprocess.run(["timeout", "-k", "120", str(int(args.max_hours * 3600)), "bash", str(LAUNCHER),
                             str(args.seed)], stdout=fh, stderr=subprocess.STDOUT, cwd=str(RL), env=env).returncode
    report["exit"], report["wall_s"] = rc, time.time() - t0
    end = R.host()
    report["host_end"] = end
    problems, host_ok, details = post_run_problems(out, args.seed, args.smoke, before, committed, rc, start, end,
                                                   head=head)
    report.update(details)
    if R.K.git("status", "--porcelain", "--untracked-files=no", root=ROOT).strip():
        problems.append("tree became dirty")
    report["problems"] = problems
    ok_label = "SMOKE_OK" if args.smoke else "TRAINED"
    report["development_smoke"] = bool(args.smoke)
    report["verdict"] = "VOID_HOST_FAULT" if not host_ok else (ok_label if not problems else "TRAINING_FAIL")
    if report.get("checkpoint"):               # written after the verdict, with it (review r2 N-6)
        report["checkpoint"]["manifest"] = write_manifest(pathlib.Path(report["checkpoint"]["copy"]), args.seed, head,
                                                          report, out, report["verdict"])
    report["finished"] = R.now()
    (out / "train_report.json").write_text(json.dumps(report, indent=1, default=str) + "\n")
    print(json.dumps({k: report.get(k) for k in ("seed", "verdict", "exit", "wall_s", "checkpoint", "problems")},
                     indent=1, default=str))
    return 0 if report["verdict"] == ok_label else 1


if __name__ == "__main__":
    raise SystemExit(main())
