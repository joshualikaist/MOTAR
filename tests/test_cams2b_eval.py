"""CPU tests of tools/cams2b_eval_run.py and tools/cams2b_eval_analyse.py (CAM-S2b-FT-E,
docs/plans/prereg_camonly_s2b_ft_eval_2026-11-24.md): the A60 arm's environment is byte-for-byte CAM-S2a-E's YC12k; the FT
environment check accepts the S2b training driver's own expected environment and refuses a deviation; the FT / FT45 cell
environments and the snapshot/receipt checks accept a faithful cell and refuse deviating ones (yaw scale, clip limit,
recorder reference, executed yaw over the clip); the reuse checks accept real CAM-S2a-E YC12k cells and refuse a real
unclipped cell; the S2b training run report is read for the final checkpoint; plans, verdicts, the reuse decision, the
rate gate and the stage links; the G1 / G2 rules; the report bindings; the controls on CAM-K1-E's real cells at this
unit's 6-block geometry. Tests that need files absent on this host are skipped with a reason (a skip is never a pass).
Run from tests/: python -B -m unittest test_cams2b_eval"""
import hashlib
import importlib.util
import json
import pathlib
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
K1E_RAW = pathlib.Path.home() / "motar_tmp" / "camk1e_20261108" / "raw"


def _load_tool(name):
    """tools/<name>.py under its production module name (the analysis imports the runner), tools/ importable."""
    if str(ROOT / "tools") not in sys.path:
        sys.path.insert(0, str(ROOT / "tools"))
    spec = importlib.util.spec_from_file_location(name, str(ROOT / "tools" / (name + ".py")))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


W = _load_tool("cams2b_eval_run")
AN = _load_tool("cams2b_eval_analyse")
S2A = W.S2A
HAVE_S2A = W.S2A_TRAIN_RECEIPT.is_file() and W.S2A_TRAIN_REPORT.is_file()
S2A_RECEIPT = json.loads(W.S2A_TRAIN_RECEIPT.read_text()) if HAVE_S2A else None
S2A_ENV = S2A_RECEIPT["env_navrl"] if S2A_RECEIPT else None
needs_s2a = unittest.skipUnless(HAVE_S2A, "CAM-S2a training receipt %s is required (skipped, not passed)" % W.S2A_TRAIN_RECEIPT)
TRAIN_DRIVER = ROOT / "tools" / "cams2b_train.py"


def _reroot(obj, old, new):
    """A copy of a JSON document with every string path under `old` moved under `new`."""
    if isinstance(obj, dict):
        return {k: _reroot(v, old, new) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_reroot(v, old, new) for v in obj]
    if isinstance(obj, str) and obj.startswith(str(old) + "/"):
        return str(new) + obj[len(str(old)):]
    return obj


def _ft_env():
    """The FT training environment as the S2b training driver expects it (its own expected_env when the driver is
    present; else the same differences written out)."""
    receipt_path = W.FT_TRAIN_RECEIPT
    if TRAIN_DRIVER.is_file():
        spec = importlib.util.spec_from_file_location("cams2b_train_for_test", str(TRAIN_DRIVER))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod.expected_env(S2A_ENV, "ft", ROOT, receipt_path)
    env = dict(S2A_ENV)
    env.update(NAVRL_YAW_RATE_MAX="1.0472", NAVRL_DENSITY_CHECK_EPS="16", NAVRL_REALVEH_RECEIPT=str(receipt_path),
               NAVRL_P9_ERROR_MODEL=str(ROOT / W.P9_MODEL_REL), NAVRL_V2_ALLOW_RESUME="1", NAVRL_LEARNING_RATE="7.5e-06",
               NAVRL_RESET_DENSITY_WINDOW="1", NAVRL_DENSITY_RESUME_WARMUP="1000")
    return env


def _ft_receipt():
    """A faithful S2b-FT training receipt: CAM-S2a's, written in this worktree, with the FT environment."""
    rec = _reroot(json.loads(json.dumps(S2A_RECEIPT)), W.tree_of(S2A_RECEIPT), ROOT)
    rec["env_navrl"] = _ft_env()
    rec["settings"]["receipt"] = str(W.FT_TRAIN_RECEIPT)
    return rec


def _ft_report(path, pin, **changes):
    rep = {"unit": "CAM-S2b", "run": "ft", "mode": "run", "verdict": "TRAINED", "preregistration": W.FT_TRAIN_PREREG,
           "preflight": {"head": "a" * 40, "branch": "camonly/dev"}, "faults_before": 1, "faults_after": 1,
           "max_epochs": 15000, "run_folder": str(path.parent.parent),
           "source_checkpoint": {"sha256": W.S2A_CKPT_SHA},
           "final_checkpoint": {"epoch": 15000, "file": path.name, "sha256": pin}}
    rep.update(changes)
    return rep


class Plans(unittest.TestCase):
    def test_plans_grid_and_caps(self):
        self.assertEqual(W.DENSITIES, (175, 205))
        self.assertTrue(all(d % 4 for d in W.DENSITIES))                                   # TF32 cdist note
        self.assertEqual(W.SEEDS, (4501, 4502, 4503))
        self.assertEqual(W.QUOTA, 8)
        plan = W.eval_plan()
        self.assertEqual(len(plan), 12)
        self.assertEqual(plan[:4], [("FT", 175, 4501), ("FT45", 175, 4501), ("FT", 175, 4502), ("FT45", 175, 4502)])
        self.assertEqual(len(W.a60_plan()), 6)
        self.assertEqual([a for a, _, _ in W.IDENTITY_PLAN], ["FT_identity", "A60_repro"])
        self.assertEqual(W.IDENTITY_PLAN[1][1:], W.REPRO_BLOCK)
        self.assertIn(W.REPRO_BLOCK, W.reuse_blocks())
        self.assertEqual(W.SMOKE_QUOTA, 1)
        self.assertAlmostEqual(sum(W.MAX_HOURS[s] for s in ("smoke", "identity", "eval")), 3.15)
        self.assertEqual(W.cell_id("A60_repro", 175, 4501), S2A.CELL % (175, 4501))          # same cell id as the reused cell
        self.assertEqual(W.cell_id("FT_identity", 205, 4501), "S2B_FT__205bars__seed4501")

    def test_arm_configuration(self):
        self.assertEqual(W.ARMS["A60"]["yawclip"], S2A.YAWCLIP_RADPS)                        # "1.047" as YC12k
        self.assertEqual(W.ARMS["A60"]["diag_ref"], S2A.YAWCLIP_RADPS)
        self.assertEqual((W.ARMS["FT"]["yawclip"], W.ARMS["FT"]["diag_ref"]), (None, "0.7854"))
        self.assertEqual((W.ARMS["FT45"]["yawclip"], W.ARMS["FT45"]["diag_ref"]), ("0.7854", "0.7854"))
        self.assertLess(float(W.FT45_RADPS), W.FT_YAW_RATE_MAX)                             # navrl_yawclip refuses limit >= scale

    def test_scope_and_launcher(self):
        self.assertEqual(W.scope_digest(), hashlib.sha256(W.SCOPE.encode()).hexdigest())
        self.assertEqual(W.LAUNCHER, "aerial_gym/rl_training/rl_games/eval_navrl_cams2a_arms.sh")
        self.assertEqual(W.sha(ROOT / W.LAUNCHER), W.LAUNCHER_SHA)                         # the committed CAM-S2a-E copy

    def test_allowlists(self):
        ok = ["docs/plans/prereg_camonly_s2b_ft_eval_2026-11-24.md", "WORKLOG.md", "tests/test_cams2b_eval.py",
              "tools/cams2b_train.py", "tools/cams2b_python", "tools/camera_policy_runtime.py",
              "aerial_gym/rl_training/rl_games/train_navrl_cams2b.sh", "tools/cams2b_eval_run.py"]
        self.assertEqual(W.allowlist_problems(ok, W.S2A_CODE_ALLOWLIST, W.S2A_E_COMMIT), [])
        for bad in ("aerial_gym/task/navrl_task/navrl_task.py", "tools/realveh_entry.py", "tools/cams2a_eval_run.py",
                    "aerial_gym/rl_training/rl_games/play_navrl.sh"):
            self.assertEqual(len(W.allowlist_problems([bad], W.S2A_CODE_ALLOWLIST, W.S2A_E_COMMIT)), 1, bad)
        self.assertEqual(len(W.allowlist_problems(["tools/cams2b_train.py"], W.FT_CODE_ALLOWLIST, "a" * 40)), 1)
        self.assertEqual(W.allowlist_problems(["tools/cams2b_eval_analyse.py", "docs/x.md"], W.FT_CODE_ALLOWLIST, "a" * 40), [])
        self.assertTrue(W.allowlist_problems(None, W.FT_CODE_ALLOWLIST, "a" * 40))


@needs_s2a
class Environments(unittest.TestCase):
    def test_a60_environment_is_cams2a_e_yc12k(self):
        for seed in W.SEEDS:
            self.assertEqual(W.cell_navrl_env(S2A_ENV, seed, "A60"), S2A.cell_navrl_env(S2A_ENV, seed, "YC12k"))
            self.assertEqual(W.cell_navrl_env(S2A_ENV, seed, "A60_repro"), S2A.cell_navrl_env(S2A_ENV, seed, "YC12k"))
        d = pathlib.Path("/tmp/cell")
        self.assertEqual(W.expected_snapshot_values("A60", 175, 4501, d, "c" * 64),
                         S2A.expected_snapshot_values("YC12k", 175, 4501, d, "c" * 64))

    def test_ft_environment_check(self):
        ft = _ft_env()
        self.assertEqual(W.ft_env_problems(ft, S2A_ENV, ROOT, W.tree_of(S2A_RECEIPT)), [])          # positive control
        bad = dict(ft, NAVRL_SPEED_GOVERNOR_FREE_MPS="3.0")
        self.assertTrue(any("outside the allowlist" in p for p in W.ft_env_problems(bad, S2A_ENV, ROOT, W.tree_of(S2A_RECEIPT))))
        self.assertTrue(W.ft_env_problems(dict(ft, NAVRL_YAW_RATE_MAX="3.0"), S2A_ENV, ROOT, W.tree_of(S2A_RECEIPT)))
        moved = dict(ft, NAVRL_P9_ERROR_MODEL=str(ROOT / "results/other/p9_error_injector.json"))
        self.assertTrue(W.ft_env_problems(moved, S2A_ENV, ROOT, W.tree_of(S2A_RECEIPT)))
        self.assertTrue(W.ft_env_problems(dict(ft, NAVRL_P9_EXPECTED_SHA256="0" * 64), S2A_ENV, ROOT, W.tree_of(S2A_RECEIPT)))
        self.assertTrue(W.ft_env_problems({k: v for k, v in ft.items() if k != "NAVRL_CAMS2"}, S2A_ENV, ROOT,
                                          W.tree_of(S2A_RECEIPT)))                         # F2

    def test_ft_cell_environments(self):
        ft = _ft_env()
        env = W.cell_navrl_env(ft, 4502, "FT")
        self.assertEqual(env["NAVRL_YAW_RATE_MAX"], ft["NAVRL_YAW_RATE_MAX"])
        self.assertEqual(float(env["NAVRL_YAW_RATE_MAX"]), 1.0472)
        self.assertNotIn("NAVRL_YAWCLIP", env)
        self.assertEqual((env["NAVRL_CAMS2DIAG"], env["NAVRL_CAMS2DIAG_YAW_REF_RADPS"], env["NAVRL_CAMS2_CONTACTCODE"]),
                         ("1", "0.7854", "1"))
        self.assertEqual(env["NAVRL_DENSITY_CURRICULUM"], "0")
        for k in S2A.REMOVED_FROM_RECEIPT:
            self.assertNotIn(k, env)
        self.assertEqual(env["NAVRL_VEH_RAND_SEED"], "45021")
        y = W.cell_navrl_env(ft, 4502, "FT45")
        self.assertEqual((y["NAVRL_YAWCLIP"], y["NAVRL_YAWCLIP_RADPS"]), ("1", "0.7854"))
        self.assertEqual({k: v for k, v in y.items() if not k.startswith("NAVRL_YAWCLIP")}, env)
        with self.assertRaises(ValueError):
            W.cell_navrl_env({k: v for k, v in ft.items() if k != "NAVRL_CAMSAFE"}, 4501, "FT")


def _snapshot(cell_dir, arm, density, seed, ckpt_sha, train_env, **changes):
    snap = dict(W.cell_navrl_env(train_env, seed, arm))
    snap.update({"NAVRL_BULK_EVAL": "1", "NAVRL_BULK_EVAL_JSON": str(cell_dir / "result.json"), "NAVRL_EVAL_CHECKPOINT": ckpt_sha,
                 "NAVRL_EVAL_RUN_NONCE": "n", "NAVRL_NUM_BARS": str(density), "NAVRL_DENSITY_START": str(density),
                 "NAVRL_DENSITY_FINAL": str(density), "NAVRL_SEED": str(seed),
                 "NAVRL_REALVEH_RECEIPT": str(cell_dir / "realveh_receipt.json"),
                 "NAVRL_CAMS2DIAG_JSONL": str(cell_dir / W.DIAG_FILE)})
    snap.update(changes)
    return snap


def _write_cell(cell_dir, arm, density, seed, train_receipt, train_sha, committed, ckpt_sha, n_env=4, quota=2, **tw):
    """A synthetic, faithful cell of `arm` against `train_receipt` (tw: rec_base = the receipt the realveh record is
    derived from, when it was written in another tree; shift = outcome rotation; commit = the launcher receipt's)."""
    cell_dir.mkdir(parents=True)
    c = W.cfg(arm)
    tenv = train_receipt["env_navrl"]
    seeds = S2A.module_seeds(seed)
    rec = json.loads(json.dumps(tw.get("rec_base", train_receipt)))
    rec["settings"]["cams2"]["NAVRL_CAMS2_DELAY_SEED"] = int(seeds["NAVRL_CAMS2_DELAY_SEED"])
    rec["settings"]["vehicle_rand"]["NAVRL_VEH_RAND_SEED"] = int(seeds["NAVRL_VEH_RAND_SEED"])
    rec["settings"]["depth_model"]["NAVRL_DEPTH_SEED"] = int(seeds["NAVRL_DEPTH_SEED"])
    rec["sha256"]["realveh_entry"] = committed["realveh_entry"]
    rec["settings"]["cams2_contactcode"] = {}
    rec["cams2_contactcode"] = {"contact_code": 0}
    rec["cams2_contactcode_sha256"] = committed["contactcode"]
    ref = float(tw.get("diag_ref", c["diag_ref"]))
    rec["settings"]["cams2diag"] = {"NAVRL_CAMS2DIAG_JSONL": str(cell_dir / W.DIAG_FILE), "NAVRL_CAMS2DIAG_YAW_REF_RADPS": ref}
    rec["cams2diag"] = {"jsonl": str(cell_dir / W.DIAG_FILE), "yaw_ref_radps": ref}
    rec["cams2diag_sha256"] = committed["cams2diag"]
    scale = float(tenv["NAVRL_YAW_RATE_MAX"])
    if c["yawclip"]:
        rec["settings"]["yawclip"] = {"NAVRL_YAWCLIP_RADPS": float(c["yawclip"])}
        rec["yawclip"] = {"limit_radps": float(tw.get("clip_limit", c["yawclip"])), "scale_radps": tw.get("clip_scale", scale)}
        rec["yawclip_sha256"] = committed["yawclip"]
    rec["env_navrl"] = _snapshot(cell_dir, arm, density, seed, ckpt_sha, tenv, **tw.get("entry_env", {}))
    (cell_dir / "realveh_receipt.json").write_text(json.dumps(rec))
    (cell_dir / "realveh_receipt.json.veh_rand_init.json").write_text("{}")
    (cell_dir / "receipt.json").write_text(json.dumps({"status": "OK", "train_receipt_sha256": train_sha,
                                                        "git_commit": tw.get("commit"), "git_dirty_files": 0,
                                                        "checkpoint_sha256": ckpt_sha,
                                                        "env_snapshot_navrl": _snapshot(cell_dir, arm, density, seed, ckpt_sha, tenv),
                                                        "duration_s": 300, "sim_name": "base_sim", "python": S2A.REAL_PYTHON,
                                                        "gpu4gb": ""}))
    ledger, rows, step = [], [], 0
    outcomes = ["capture", "crash", "crash", "timeout"]
    for ep in range(quota):
        for env in range(n_env):
            step += 7
            o = outcomes[(env + ep + tw.get("shift", 0)) % 4] if env != 0 or ep != 0 else tw.get("first", outcomes[0])
            ledger.append({"env_index": env, "env_episode_index": ep, "completion_step": step, "outcome": o, "in_quota": True,
                           "episode_id": "%d-%d" % (env, ep)})
            rows.append({"env_index": env, "env_episode_index": ep, "completion_step": step, "outcome": o,
                         "crash_class": ("target_contact_swept" if env == 1 else "contact") if o == "crash" else "none",
                         "delay_k": 1 + env % 2, "episode_steps": None, "hidden_steps": 3, "visible_steps": 4,
                         "yaw_request_over_ref_steps": 2, "yaw_exec_over_ref_steps": tw.get("exec_over", 0 if c["yawclip"] else 2),
                         "yaw_ref_radps": ref, "capture_dist_m": 0.8 if o == "capture" else None,
                         "capture_closing_mps": 0.4 if o == "capture" else None,
                         "capture_rel_speed_mps": 1.3 if o == "capture" else None,
                         "capture_lateral_mps": 1.2 if o == "capture" else None})
    lengths = AN.SA.episode_lengths(ledger)
    for r in rows:
        r["episode_steps"] = lengths[(r["env_index"], r["env_episode_index"])]
    (cell_dir / "episodes.jsonl").write_text("".join(json.dumps(r) + "\n" for r in ledger))
    (cell_dir / W.DIAG_FILE).write_text("".join(json.dumps(r) + "\n" for r in rows))
    (cell_dir / "episode_ledger_summary.json").write_text(json.dumps({"status": "COMPLETE", "partition_violations": 0}))
    (cell_dir / "result.json").write_text(json.dumps({
        "checkpoint": ckpt_sha, "crash_causes": {"count": 2}, "outcome": {"n": len(ledger)},
        "speed_governor": {"samples": 10, "intervention_rate": 0.1, "mean_executed_speed_mps": 2.0, "mean_clearance_m": 5.0,
                           "mean_scale": 0.9},
        "condition": dict(S2A.CONDITION_EXPECTED, p9_empirical_error_sha256=tenv["NAVRL_P9_EXPECTED_SHA256"],
                          d8_dynamic_mesh_contract={"mode": S2A.D8_MODE}, evaluation_nonce="n")}))
    log = "WARNING NavRL TRAINING SEED MISMATCH | checkpoint=1 running=%d. x\n" % seed
    log += "[camsafe] calls=320 capped_frac=0.0100 penalised_frac=0.0200 mean_penalty=0.00100 blind_mps=1 weight=0.5\n"
    log += "[cams2contact] {\"event\": \"summary\", \"steps\": 320, \"contacts\": 4}\n"
    log += "[cams2diag] {\"event\": \"summary\", \"rows\": %d}\n" % len(rows)
    if c["yawclip"]:
        log += ("[yawclip] calls=320 commands=40960 clipped_frac=0.1234 mean_abs_yaw_before=0.9 mean_abs_yaw_after=0.6 "
                "limit_radps=%g scale_radps=%g\n" % (float(c["yawclip"]), tw.get("line_scale", scale)))
    log += tw.get("extra_log", "")
    (cell_dir / "cell.log").write_text(log)
    return cell_dir


@needs_s2a
class ReceiptChecks(unittest.TestCase):
    COMMITTED = {"realveh_entry": "e" * 64, "yawclip": "y" * 64, "cams2diag": "d" * 64, "contactcode": "k" * 64}
    CK, TSHA = "c" * 64, "t" * 64

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.base = pathlib.Path(self.tmp.name)
        self.ft = _ft_receipt()

    def tearDown(self):
        self.tmp.cleanup()

    def _check(self, arm, tag="", **tw):
        d = _write_cell(self.base / (arm + tag) / W.cell_id(arm, 175, 4502), arm, 175, 4502, self.ft, self.TSHA,
                        self.COMMITTED, self.CK, **tw)
        return W.receipt_problems(d, arm, 175, 4502, self.ft, self.TSHA, self.COMMITTED, self.CK)

    def test_faithful_ft_and_ft45_cells_pass(self):
        self.assertEqual(self._check("FT"), [])
        self.assertEqual(self._check("FT45"), [])

    def test_deviations_fail(self):
        self.assertTrue(any("yaw-clip receipt" in p for p in self._check("FT45", "s", clip_scale=3.0)))     # wrong action scale
        self.assertTrue(any("yaw-clip receipt" in p for p in self._check("FT45", "l", clip_limit="1.047")))  # wrong clip
        self.assertTrue(any("[yawclip] line" in p for p in self._check("FT45", "t", line_scale=3.0)))
        self.assertTrue(any("cams2diag install" in p for p in self._check("FT", "r", diag_ref="1.047")))     # recorder reference
        self.assertTrue(any("executed steps over the limit" in p for p in self._check("FT45", "x", exec_over=1)))
        self.assertTrue(any("NAVRL_YAW_RATE_MAX" in p for p in self._check("FT", "y", entry_env={"NAVRL_YAW_RATE_MAX": "3.0"})))
        self.assertTrue(any("checkpoint-contract" in p for p in self._check(
            "FT", "m", extra_log="NavRL CONFIG MISMATCH | NAVRL_YAW_RATE_MAX: checkpoint trained with 3.000, running with 1.047\n")))
        d = _write_cell(self.base / "other" / W.cell_id("FT", 175, 4502), "FT", 175, 4502, self.ft, self.TSHA, self.COMMITTED, self.CK)
        self.assertTrue(W.receipt_problems(d, "FT", 175, 4502, self.ft, "u" * 64, self.COMMITTED, self.CK))   # other receipt SHA
        self.assertTrue(W.receipt_problems(d, "FT45", 175, 4502, self.ft, self.TSHA, self.COMMITTED, self.CK))  # clip missing

    def test_ablation_paths_are_tree_relative(self):
        rec = json.loads(json.dumps(self.ft))
        self.assertEqual(W.ablation_problems(rec, S2A_RECEIPT, ROOT), [])         # another tree, same modules
        rec["ablation"]["task"] = str(ROOT / "aerial_gym/task/navrl_task/other.py")
        self.assertTrue(W.ablation_problems(rec, S2A_RECEIPT, ROOT))
        self.assertTrue(W.ablation_problems(json.loads(json.dumps(self.ft)), S2A_RECEIPT, pathlib.Path("/nonexistent")))


@needs_s2a
class TrainingRun(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        nn = pathlib.Path(self.tmp.name) / "runs" / "ppo_x" / "nn"
        nn.mkdir(parents=True)
        self.ck = nn / "last_gen_ppo_ep_15000_rew_99.5.pth"
        self.ck.write_bytes(b"checkpoint")
        self.pin = hashlib.sha256(b"checkpoint").hexdigest()

    def tearDown(self):
        self.tmp.cleanup()

    def test_final_checkpoint_from_the_run_report(self):
        (path, pin), p = W.ft_checkpoint(_ft_report(self.ck, self.pin))
        self.assertEqual((path, pin, p), (str(self.ck), self.pin, []))
        self.assertTrue(W.ft_checkpoint(_ft_report(self.ck, "0" * 64))[1])                                   # SHA mismatch
        self.assertTrue(W.ft_checkpoint(_ft_report(self.ck, self.pin, max_epochs=14000))[1])                  # not the final epoch
        self.assertTrue(W.ft_checkpoint(_ft_report(self.ck, self.pin, final_checkpoint=None))[1])
        g = self.ck.parent / "gen_ppo.pth"
        g.write_bytes(b"checkpoint")
        self.assertTrue(W.ft_checkpoint(_ft_report(g, self.pin))[1])                                          # never gen_ppo.pth

    def test_training_run_accepted_and_altered_refused(self):
        rep, ft = _ft_report(self.ck, self.pin), _ft_receipt()
        self.assertEqual(W.ft_training_problems(ft, rep, S2A_RECEIPT), [])
        for change in ({"verdict": "WALL_CAP"}, {"faults_after": 2}, {"preregistration": "docs/plans/x.md"},
                       {"source_checkpoint": {"sha256": "0" * 64}}, {"preflight": {"head": "a" * 40, "branch": "main"}},
                       {"run": "scr"}):
            self.assertTrue(W.ft_training_problems(ft, dict(rep, **change), S2A_RECEIPT), change)
        bad = json.loads(json.dumps(ft))
        bad["sha256"]["navrl_task"] = "0" * 64
        self.assertTrue(W.ft_training_problems(bad, rep, S2A_RECEIPT))
        bad = json.loads(json.dumps(ft))
        bad["settings"]["camsafe"]["NAVRL_CAMSAFE_BLIND_MPS"] = 2.0
        self.assertTrue(W.ft_training_problems(bad, rep, S2A_RECEIPT))
        bad = json.loads(json.dumps(ft))
        bad["settings"]["receipt"] = "/elsewhere.json"
        self.assertTrue(W.ft_training_problems(bad, rep, S2A_RECEIPT))


REUSE_CELL = W.reused_cell_dir(*W.REPRO_BLOCK)


@needs_s2a
@unittest.skipUnless((REUSE_CELL / "receipt.json").is_file(), "the CAM-S2a-E YC12k cell %s is required (skipped, not passed)" % REUSE_CELL)
class ReuseOnRealCells(unittest.TestCase):
    def test_real_yc12k_cells_pass_and_an_unclipped_cell_fails(self):
        committed = W.committed_sha256_at(W.S2A_E_COMMIT)
        self.assertTrue(all(committed.values()))
        n = 0
        for d, s in W.reuse_blocks():
            cell = W.reused_cell_dir(d, s)
            if (cell / "receipt.json").is_file():
                self.assertEqual(W.reused_cell_problems(cell, d, s, S2A_RECEIPT, committed), [], (d, s))
                n += 1
        self.assertGreater(n, 0)                                                         # non-vacuous
        unclipped = W.S2A_E_RAW / "eval_01" / "S12k" / (S2A.CELL % W.REPRO_BLOCK)
        if (unclipped / "receipt.json").is_file():
            self.assertTrue(W.reused_cell_problems(unclipped, *W.REPRO_BLOCK, S2A_RECEIPT, committed))
        other_seed = W.reused_cell_problems(REUSE_CELL, W.REPRO_BLOCK[0], 4503, S2A_RECEIPT, committed)
        self.assertTrue(other_seed)                                                      # the block is bound

    def test_manifest(self):
        with tempfile.TemporaryDirectory() as t:
            f = pathlib.Path(t) / "a.json"
            f.write_text("x")
            m = {str(f): W.sha(f)}
            self.assertEqual([p for p in W.manifest_problems(m) if "changed" in p], [])
            f.write_text("y")
            self.assertTrue([p for p in W.manifest_problems(m) if "changed" in p])
        self.assertTrue(W.manifest_problems({}))


class Verdicts(unittest.TestCase):
    def test_s2a_e_report_problems(self):
        recs = [{"arm": "YC12k", "density": d, "seed": s, "verdict": "OK", "stage": "A", "checkpoint_sha256": W.S2A_CKPT_SHA,
                 "commit": W.S2A_E_COMMIT} for d, s in W.reuse_blocks()]
        host = {"fault_count": 1, "boot_id": "b", "online": "0-23"}
        ev = {"unit": "CAM-S2a-E", "commit": W.S2A_E_COMMIT, "tree": str(W.S2A_E_TREE), "verdict": "COMPLETE",
              "host_start": host, "host_end": dict(host), "records": recs}
        ident = {"verdict": "IDENTITY_PASS", "commit": W.S2A_E_COMMIT}
        self.assertEqual(W.s2a_e_report_problems(ev, ident), [])
        self.assertEqual(W.s2a_e_report_problems(dict(ev, verdict="INCOMPLETE"), ident), [])
        self.assertTrue(W.s2a_e_report_problems(dict(ev, verdict="VOID_HOST_FAULT"), ident))
        self.assertTrue(W.s2a_e_report_problems(dict(ev, host_end=dict(host, fault_count=2)), ident))
        self.assertTrue(W.s2a_e_report_problems(dict(ev, records=recs[:-1]), ident))
        self.assertTrue(W.s2a_e_report_problems(ev, dict(ident, verdict="IDENTITY_FAIL")))
        self.assertTrue(W.s2a_e_report_problems(dict(ev, records=[dict(recs[0], verdict="FAIL")] + recs[1:]), ident))

    def test_reuse_decision(self):
        eq = {"result_equal": True, "episodes_equal": True, "diag_equal": True}
        self.assertEqual(W.reuse_decision([], eq), "REUSE_OK")
        self.assertEqual(W.reuse_decision(["x"], None), "RERUN_REQUIRED")
        for k in eq:
            self.assertEqual(W.reuse_decision([], dict(eq, **{k: False})), "A60_NOT_REPRODUCED", k)
        self.assertEqual(W.reuse_decision([], None), "A60_NOT_REPRODUCED")
        self.assertEqual(W.reuse_decision([], eq, a60_repro_verdict="FAIL"), "A60_PATH_FAIL")
        self.assertEqual(W.reuse_decision(["x"], None, a60_repro_verdict="FAIL"), "A60_PATH_FAIL")
        self.assertEqual(W.reuse_decision(["x"], None, a60_repro_verdict=None), "A60_NOT_RUN")

    def test_identity_smoke_and_plan_verdicts(self):
        ok = [{"arm": a, "verdict": "OK"} for a, _, _ in W.IDENTITY_PLAN]
        self.assertEqual(W.identity_verdict(ok), "IDENTITY_PASS")
        self.assertEqual(W.identity_verdict([ok[0], dict(ok[1], verdict="FAIL")]), "IDENTITY_PASS")   # G2 never waits on A60
        self.assertEqual(W.identity_verdict(ok[:1]), "IDENTITY_FAIL")
        self.assertEqual(W.identity_verdict([dict(ok[0], verdict="FAIL")]), "IDENTITY_FAIL")
        sm = [{"arm": a, "verdict": "OK"} for a, _, _ in W.SMOKE_PLAN]
        self.assertEqual(W.smoke_verdict(sm, True), "SMOKE_OK")
        self.assertEqual(W.smoke_verdict(sm, False), "SMOKE_FAIL")
        self.assertEqual(W.smoke_verdict(sm[:1] + [{"arm": "FT45_smoke", "verdict": "VOID_HOST_FAULT"}], True), "VOID_HOST_FAULT")
        recs = [{"arm": a, "density": d, "seed": s, "verdict": "OK"} for a, d, s in W.eval_plan()]
        self.assertEqual(W.plan_verdict(recs, W.eval_plan()), "COMPLETE")
        self.assertEqual(W.plan_verdict(recs[:-1], W.eval_plan()), "INCOMPLETE")
        self.assertEqual(W.plan_verdict(recs[:-1] + [dict(recs[-1], verdict="FAIL")], W.eval_plan()), "INCOMPLETE")

    def test_rate_gate(self):
        ident = {"records": [{"arm": "A60_repro", "wall_s": 400.0}, {"arm": "FT_identity", "wall_s": 579.0}]}
        p = W.rate_projection(ident, "eval")
        self.assertAlmostEqual(p["projected_s"], (6 * 0.90 + 6) * 579.0 * 1.10)
        self.assertEqual(W.rate_problems(ident, "eval"), [])                           # 7,261 s <= 9,000 s
        slow = {"records": [dict(r, wall_s=720.0) for r in ident["records"]]}
        self.assertEqual(len(W.rate_problems(slow, "eval")), 1)                         # 9,029 s > 9,000 s
        self.assertAlmostEqual(W.rate_projection(ident, "a60")["projected_s"], (3 + 3 * 1.67) * 400.0 * 1.10)
        self.assertTrue(W.rate_problems({"records": []}, "eval"))

    def test_stage_links(self):
        with tempfile.TemporaryDirectory() as t:
            parent, head, pin = pathlib.Path(t), "h" * 40, "p" * 64
            self.assertTrue(W.stage_link_problems("identity", parent / "identity_01", head, pin)[0])   # no smoke
            (parent / "smoke_01").mkdir()
            (parent / "smoke_01" / "smoke_report.json").write_text(json.dumps(
                {"verdict": "SMOKE_OK", "commit": head, "ft_checkpoint_sha256": pin}))
            self.assertEqual(W.stage_link_problems("identity", parent / "identity_01", head, pin)[0], [])
            self.assertTrue(W.stage_link_problems("identity", parent / "identity_01", "g" * 40, pin)[0])  # other commit
            self.assertTrue(W.stage_link_problems("identity", parent / "identity_01", head, "q" * 64)[0])  # other FT checkpoint
            self.assertTrue(W.stage_link_problems("eval", parent / "eval_01", head, pin)[0])              # no identity
            (parent / "identity_01").mkdir()
            (parent / "identity_01" / "identity_report.json").write_text(json.dumps(
                {"verdict": "IDENTITY_PASS", "commit": head, "ft_checkpoint_sha256": pin, "a60_source": "REUSE_OK"}))
            self.assertEqual(W.stage_link_problems("eval", parent / "eval_01", head, pin)[0], [])
            self.assertTrue(W.stage_link_problems("a60", parent / "a60_01", head, pin)[0])               # no eval, reuse OK
            (parent / "eval_01").mkdir()
            (parent / "eval_01" / "eval_run_report.json").write_text(json.dumps(
                {"verdict": "COMPLETE", "commit": head, "ft_checkpoint_sha256": pin}))
            self.assertTrue(any("RERUN_REQUIRED" in p for p in W.stage_link_problems("a60", parent / "a60_01", head, pin)[0]))
            (parent / "identity_01" / "identity_report.json").write_text(json.dumps(
                {"verdict": "IDENTITY_PASS", "commit": head, "ft_checkpoint_sha256": pin, "a60_source": "RERUN_REQUIRED"}))
            self.assertEqual(W.stage_link_problems("a60", parent / "a60_01", head, pin)[0], [])


class Rules(unittest.TestCase):
    def test_gate_g1(self):
        g = AN.gate_g1
        self.assertEqual(g("MATERIAL_REDUCTION", "NEGLIGIBLE"), "FT_BETTER")
        self.assertEqual(g("SMALL_REDUCTION", "SMALL_HIGHER"), "FT_BETTER")
        self.assertEqual(g("NEGLIGIBLE", "HIGHER"), "FT_BETTER")
        self.assertEqual(g("CRASH_TO_TIMEOUT", "NEGLIGIBLE"), "FT_BETTER")
        self.assertEqual(g("MATERIAL_REDUCTION", "LOWER"), "FT_TRADEOFF")
        self.assertEqual(g("WORSE", "HIGHER"), "FT_TRADEOFF")
        self.assertEqual(g("WORSE", "NEGLIGIBLE"), "FT_WORSE")
        self.assertEqual(g("NEGLIGIBLE", "LOWER"), "FT_WORSE")
        self.assertEqual(g("NEGLIGIBLE", "NEGLIGIBLE"), "FT_NO_MATERIAL_DIFFERENCE")
        self.assertEqual(g("NEGLIGIBLE", "SMALL_HIGHER"), "FT_NO_MATERIAL_DIFFERENCE")
        for crash, cap in (("MATERIAL_REDUCTION", "SMALL_LOWER"), ("MATERIAL_REDUCTION", "NO_DETECTED_CHANGE"),
                           ("UNRESOLVED", "HIGHER"), ("NO_DETECTED_CHANGE", "NEGLIGIBLE"), ("NEGLIGIBLE", "SMALL_LOWER"),
                           ("NEGLIGIBLE", "UNRESOLVED"), ("NO_DETECTED_CHANGE", "HIGHER")):
            self.assertEqual(g(crash, cap), "FT_UNRESOLVED", (crash, cap))

    def test_gate_g2_is_gate_y_with_the_45_prefix(self):
        g = AN.gate_g2
        self.assertEqual(g("NEGLIGIBLE", "SMALL_LOWER", True, 0.05), "YAWCLIP45_TOLERATED")
        self.assertEqual(g("WORSE", "NEGLIGIBLE", True, 0.05), "YAWCLIP45_SENSITIVE")
        self.assertEqual(g("NEGLIGIBLE", "LOWER", True, 0.05), "YAWCLIP45_SENSITIVE")
        self.assertEqual(g("NO_DETECTED_CHANGE", "NEGLIGIBLE", True, 0.05), "YAWCLIP45_UNRESOLVED")
        self.assertEqual(g("NEGLIGIBLE", "NEGLIGIBLE", True, 0.009), "YAWCLIP45_WEAKLY_BINDING")
        self.assertEqual(g("NEGLIGIBLE", "NEGLIGIBLE", True, None), "YAWCLIP45_WEAKLY_BINDING")
        self.assertEqual(g("NEGLIGIBLE", "NEGLIGIBLE", False, 0.05), "YAWCLIP45_INACTIVE")

    def test_strat_t_df4(self):
        xs = [1.0, 2.0, 3.0, 4.0, 6.0, 8.0]
        blks = [(175, 4501), (175, 4502), (175, 4503), (205, 4501), (205, 4502), (205, 4503)]
        ci, df = AN.SA.strat_t(xs, blks)
        self.assertEqual(df, 4)
        se = ((1.0 / 3 + 4.0 / 3) / 4) ** 0.5                                            # variances 1 and 4, 3 seeds, 2 strata
        self.assertAlmostEqual(ci[1] - 4.0, 2.776445105 * se)

    def test_report_bindings(self):
        plan = W.eval_plan()
        recs = [{"arm": a, "density": d, "seed": s, "verdict": "OK"} for a, d, s in plan]
        keys = {"s2a_train_receipt_sha256": "s", "ft_train_receipt_sha256": "f", "ft_train_report_sha256": "r",
                "ft_checkpoint_sha256": "c", "env_allowlisted": {"PATH": "x"}}
        ident = dict(keys, verdict="IDENTITY_PASS", commit="h")
        with tempfile.TemporaryDirectory() as t:
            ip = pathlib.Path(t) / "identity_01" / "identity_report.json"
            ev = dict(keys, verdict="COMPLETE", commit="h", cells_planned=12, records=recs, changed_since_s2a_e=["docs/a.md"],
                      changed_since_ft_training=[], ft_train_head="a" * 40, linked_reports={"identity_report.json": str(ip)})
            present = {a: AN.blocks() for a in W.EVAL_ARMS}
            b = AN.report_bindings(ident, ev, ip, present)
            self.assertTrue(all(b.values()), b)
            self.assertFalse(all(AN.report_bindings(ident, dict(ev, records=recs[::-1]), ip, present).values()))
            self.assertFalse(all(AN.report_bindings(ident, dict(ev, changed_since_s2a_e=["tools/realveh_entry.py"]), ip, present).values()))
            self.assertFalse(all(AN.report_bindings(dict(ident, ft_checkpoint_sha256="d"), ev, ip, present).values()))
            self.assertFalse(all(AN.report_bindings(ident, ev, ip, dict(present, FT45=AN.blocks()[:5])).values()))
            self.assertFalse(all(AN.report_bindings(ident, ev, pathlib.Path(t) / "other.json", present).values()))

    def test_a60_bindings(self):
        present = {"A60": AN.blocks()}
        ident = {"a60_source": "REUSE_OK", "reuse_problems": [], "reuse_manifest": {},
                 "a60_reproduction": {"result_equal": True, "episodes_equal": True, "diag_equal": True}}
        src, b = AN.a60_bindings(ident, None, {}, "/x/eval_01/eval_run_report.json", present, True)
        self.assertEqual(src, "REUSED")
        self.assertFalse(b["reuse_manifest_unchanged"])                                  # an empty manifest never binds
        src, b = AN.a60_bindings(dict(ident, a60_source="RERUN_REQUIRED"), None, {}, "/x", present, True)
        self.assertIsNone(src)                                                           # G1 not answerable without stage a60
        a60 = {"verdict": "COMPLETE", "commit": "h", "records": [{"arm": a, "density": d, "seed": s, "verdict": "OK"}
                                                                  for a, d, s in W.a60_plan()],
               "linked_reports": {"eval_run_report.json": "/x/eval_01/eval_run_report.json"}, "s2a_train_receipt_sha256": "s",
               "ft_checkpoint_sha256": "c", "env_allowlisted": {}}
        ev = {"commit": "h", "s2a_train_receipt_sha256": "s", "ft_checkpoint_sha256": "c", "env_allowlisted": {}}
        src, b = AN.a60_bindings(dict(ident, a60_source="RERUN_REQUIRED"), a60, ev, "/x/eval_01/eval_run_report.json", present, True)
        self.assertEqual(src, "RERUN")
        self.assertTrue(all(b.values()), b)
        src, b = AN.a60_bindings(dict(ident, a60_source="RERUN_REQUIRED"), dict(a60, commit="g"), ev,
                                 "/x/eval_01/eval_run_report.json", present, True)
        self.assertFalse(all(b.values()))


@needs_s2a
@unittest.skipUnless(K1E_RAW.is_dir(), "CAM-K1-E raw cells are required for the analysis controls (skipped, not passed)")
class AnalysisEndToEnd(unittest.TestCase):
    """main_eval on synthetic trees of 4-env, quota-2 cells, in both A60 modes: rerun (identity, eval, a60) and reuse
    (identity, eval, CAM-S2a-E-shaped reused cells and reports; review r1 P1-2). The bindings hold and both gates are
    labelled; reuse_problems accepts the faithful reused tree and refuses a changed file; a changed A60 cell breaks its
    binding; without A60 cells G1 is not answered and G2 still is."""
    HEAD = "h" * 40

    def _tree(self, base, mode):
        from unittest import mock
        committed = {"realveh_entry": "e" * 64, "yawclip": "y" * 64, "cams2diag": "d" * 64, "contactcode": "k" * 64}
        ft = _ft_receipt()
        ft_path = base / "train_ft_receipt.json"
        ft_path.write_text(json.dumps(ft))
        ft_sha, s2a_sha = W.sha(ft_path), W.sha(W.S2A_TRAIN_RECEIPT)
        ck = base / "nn" / "last_gen_ppo_ep_15000_rew_1.0.pth"
        ck.parent.mkdir()
        ck.write_bytes(b"ft")
        pin = W.sha(ck)
        s2a_here = _reroot(json.loads(json.dumps(S2A_RECEIPT)), W.tree_of(S2A_RECEIPT), ROOT)
        keys = {"s2a_train_receipt_sha256": s2a_sha, "ft_train_receipt_sha256": ft_sha, "ft_train_report_sha256": "r" * 64,
                "ft_checkpoint_sha256": pin, "ft_checkpoint": str(ck), "env_allowlisted": {"PATH": "x"}, "tree": str(ROOT)}
        ident_dir, eval_dir, a60_dir, raw = base / "identity_01", base / "eval_01", base / "a60_01", base / "raw_a1"
        shift = {"FT": 1, "FT45": 2, "A60": 3}

        def cell(cell_dir, arm, d, s, reused=False):
            if W.cfg(arm)["policy"] == "FT":
                return _write_cell(cell_dir, arm, d, s, ft, ft_sha, committed, pin, rec_base=ft, commit=self.HEAD,
                                   shift=shift["FT" if arm in ("FT", "FT_identity") else "FT45"],
                                   first=["capture", "crash", "timeout"][(d + s) % 3])
            return _write_cell(cell_dir, arm, d, s, S2A_RECEIPT, s2a_sha, W.committed_sha256_at(W.S2A_E_COMMIT) if reused else committed,
                               W.S2A_CKPT_SHA, rec_base=S2A_RECEIPT if reused else s2a_here,
                               commit=W.S2A_E_COMMIT if reused else self.HEAD, shift=3, first=["capture", "crash", "timeout"][(d + s) % 3])

        recs = []
        for arm, d, s in W.eval_plan():
            c = cell(eval_dir / arm / W.cell_id(arm, d, s), arm, d, s)
            recs.append({"arm": arm, "density": d, "seed": s, "cell_id": c.name, "verdict": "OK", "checkpoint_sha256": pin,
                         "yawclip_telemetry_last_line": {"clipped_frac": 0.12} if arm == "FT45" else None})
        for arm, d, s in W.IDENTITY_PLAN:
            cell(ident_dir / arm / W.cell_id(arm, d, s), arm, d, s)
        common = dict(keys, commit=self.HEAD, committed_sha256=committed)
        patches = [mock.patch.object(W, "FT_TRAIN_RECEIPT", ft_path), mock.patch.object(W, "NUM_ENVS", 4),
                   mock.patch.object(W, "S2A_E_RAW", raw), mock.patch.object(AN.E1, "file_matches_commit", lambda *a, **k: True)]
        orig = W.output_problems                                   # the synthetic cells hold 4 envs x 2 episodes
        patches.append(mock.patch.object(W, "output_problems", lambda *a, **k: orig(*a, quota=2)))
        ident = dict(common, verdict="IDENTITY_PASS")
        if mode == "rerun":
            a60_recs = []
            for arm, d, s in W.a60_plan():
                c = cell(a60_dir / arm / W.cell_id(arm, d, s), arm, d, s)
                a60_recs.append({"arm": arm, "density": d, "seed": s, "cell_id": c.name, "verdict": "OK"})
            (a60_dir / "a60_run_report.json").write_text(json.dumps(dict(
                common, verdict="COMPLETE", records=a60_recs,
                linked_reports={"eval_run_report.json": str(eval_dir / "eval_run_report.json")})))
            ident["a60_source"] = "RERUN_REQUIRED"
        else:
            host = {"fault_count": 1, "boot_id": "b", "online": "0-23"}
            yc = []
            for d, s in W.reuse_blocks():
                c = cell(W.reused_cell_dir(d, s, raw), "A60", d, s, reused=True)
                yc.append({"arm": "YC12k", "density": d, "seed": s, "verdict": "OK", "stage": "A", "commit": W.S2A_E_COMMIT,
                           "checkpoint_sha256": W.S2A_CKPT_SHA, "stats": {"result_sha256": W.sha(c / "result.json")}})
            (raw / "identity_01").mkdir(parents=True)
            (raw / "identity_01" / "identity_report.json").write_text(json.dumps({"verdict": "IDENTITY_PASS", "commit": W.S2A_E_COMMIT}))
            (raw / "eval_01" / "eval_run_report.json").write_text(json.dumps(
                {"unit": "CAM-S2a-E", "commit": W.S2A_E_COMMIT, "tree": str(W.S2A_E_TREE), "verdict": "COMPLETE",
                 "host_start": host, "host_end": dict(host), "records": yc}))
            ident.update(a60_source="REUSE_OK", reuse_problems=[], reuse_manifest=W.reuse_manifest(raw),
                         a60_reproduction={"result_equal": True, "episodes_equal": True, "diag_equal": True})
        (ident_dir / "identity_report.json").write_text(json.dumps(ident))
        (eval_dir / "eval_run_report.json").write_text(json.dumps(dict(
            common, verdict="COMPLETE", cells_planned=12, records=recs, changed_since_s2a_e=[], changed_since_ft_training=[],
            ft_train_head="a" * 40, linked_reports={"identity_report.json": str(ident_dir / "identity_report.json")})))
        return ident_dir, eval_dir, a60_dir, raw, patches

    def _run(self, mode, body):
        with tempfile.TemporaryDirectory() as t:
            base = pathlib.Path(t)
            ident_dir, eval_dir, a60_dir, raw, patches = self._tree(base, mode)
            try:
                for p in patches:
                    p.start()
                body(base, ident_dir, eval_dir, a60_dir, raw)
            finally:
                for p in patches:
                    p.stop()

    def test_main_eval_rerun_mode(self):
        def body(base, ident_dir, eval_dir, a60_dir, raw):
            binding, ab, source, cells, ev, diag, problems = AN.load_eval(ident_dir, eval_dir, a60_dir)
            self.assertEqual(problems, {})
            self.assertTrue(all(binding.values()), binding)
            self.assertEqual(source, "RERUN")
            self.assertTrue(all(ab.values()), ab)
            self.assertEqual(sorted(cells["A60"]), AN.blocks())
            out = base / "analysis.json"
            self.assertEqual(AN.main_eval(K1E_RAW, ident_dir, eval_dir, a60_dir, out), 0)
            res = json.loads(out.read_text())
            self.assertEqual(res["reading"], "REPORTED", res.get("binding"))
            self.assertTrue(res["gate_G1"].startswith("FT_"), res["gate_G1"])
            self.assertTrue(res["gate_G2"].startswith("YAWCLIP45_"), res["gate_G2"])
            self.assertEqual(set(res["labels"]), {AN.G1, AN.G2})
            self.assertEqual(len(res["diagnostics_per_cell"]), 18)
            self.assertEqual(res["yaw"]["reference_radps"], {"FT": "0.7854", "FT45": "0.7854", "A60": "1.047"})
            f = a60_dir / "A60" / W.cell_id("A60", *W.REPRO_BLOCK) / "episodes.jsonl"
            rows = K_read(f)
            rows[0]["outcome"] = "timeout" if rows[0]["outcome"] != "timeout" else "crash"
            f.write_text("".join(json.dumps(r) + "\n" for r in rows))
            self.assertFalse(AN.load_eval(ident_dir, eval_dir, a60_dir)[1]["identity_A60_repro_equals_a60_cell"])
            out2 = base / "analysis2.json"
            self.assertEqual(AN.main_eval(K1E_RAW, ident_dir, eval_dir, None, out2), 0)
            res2 = json.loads(out2.read_text())
            self.assertEqual(res2["gate_G1"], "G1_NOT_ANSWERED")
            self.assertTrue(res2["gate_G2"].startswith("YAWCLIP45_"))
            out3 = base / "analysis3.json"                           # the analysis rechecks against the eval's tree
            ev_path = eval_dir / "eval_run_report.json"
            ev = json.loads(ev_path.read_text())
            ev_path.write_text(json.dumps(dict(ev, tree="/elsewhere")))
            self.assertEqual(AN.main_eval(K1E_RAW, ident_dir, eval_dir, None, out3), 1)
            self.assertEqual(json.loads(out3.read_text())["reading"], "EVAL_BINDING_FAIL")
        self._run("rerun", body)

    def test_main_eval_reuse_mode(self):
        def body(base, ident_dir, eval_dir, a60_dir, raw):
            self.assertEqual(W.reuse_problems(S2A_RECEIPT, raw), [])                       # positive control
            binding, ab, source, cells, ev, diag, problems = AN.load_eval(ident_dir, eval_dir, None)
            self.assertEqual(problems, {})
            self.assertEqual(source, "REUSED")
            self.assertTrue(all(binding.values()), binding)
            self.assertTrue(all(ab.values()), ab)
            out = base / "analysis.json"
            self.assertEqual(AN.main_eval(K1E_RAW, ident_dir, eval_dir, None, out), 0)
            res = json.loads(out.read_text())
            self.assertEqual(res["reading"], "REPORTED")
            self.assertTrue(res["gate_G1"].startswith("FT_"), res["gate_G1"])
            self.assertEqual(res["a60_source"], "REUSED")
            # a changed reused file: the manifest binding and reuse_problems both refuse it
            f = W.reused_cell_dir(*W.REPRO_BLOCK, raw) / "result.json"
            f.write_text(f.read_text().replace('"count": 2', '"count": 3'))
            self.assertFalse(AN.load_eval(ident_dir, eval_dir, None)[1]["reuse_manifest_unchanged"])
            self.assertTrue(any("result_sha256" in p for p in W.reuse_problems(S2A_RECEIPT, raw)))
            out2 = base / "analysis2.json"
            AN.main_eval(K1E_RAW, ident_dir, eval_dir, None, out2)
            self.assertEqual(json.loads(out2.read_text())["gate_G1"], "A60_BINDING_FAIL")
        self._run("reuse", body)


def K_read(path):
    return [json.loads(line) for line in pathlib.Path(path).read_text().splitlines() if line.strip()]


@unittest.skipUnless(K1E_RAW.is_dir(), "CAM-K1-E raw cells are required for the controls (skipped, not passed)")
class Controls(unittest.TestCase):
    def test_controls_on_camk1e_six_blocks(self):
        c = AN.controls(K1E_RAW)
        self.assertTrue(c["camk1e_cells_match_manifest"])
        self.assertTrue(c["ok"], {k: v.get("label") for k, v in c.items() if isinstance(v, dict) and "label" in v})
        self.assertEqual(c["crash_on_K1E_F66_minus_K1s1"]["label"], "WORSE")
        self.assertEqual(c["crash_on_K1E_K1s1_plus5pp_minus_K1s2_quota8"]["label"], "WORSE")
        self.assertEqual(c["capture_on_K1E_K1s1_minus5pp_minus_K1s2_quota8"]["label"], "LOWER")
        self.assertNotIn(c["crash_on_K1E_K1s1_minus_K1s2_quota8"]["label"], ("MATERIAL_REDUCTION", "WORSE"))
        self.assertEqual(c["crash_on_K1E_K1s1_minus_K1s2_quota8"]["t_df"], 4)
        self.assertEqual(c["crash_on_K1E_K1s1_minus_K1s2_quota8"]["episodes_per_cell"], 1024)
        self.assertEqual(c["blocks"], [[d, s] for d in (160, 205) for s in (4501, 4502, 4503)])


if __name__ == "__main__":
    unittest.main()
