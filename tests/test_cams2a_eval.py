"""CPU tests of the drafted tools/cams2a_eval_run.py and tools/cams2a_eval_analyse.py: the cell environment is the
training environment (F2 refusal with a positive control), the snapshot and receipt checks accept a faithful cell and
reject a deviating one, the recorder/ledger alignment check, the MISMATCH scan (P1-4), the diff allowlist (P1-3), the
result-condition check (P2-3), the identity and eval verdicts (P2-6), the rate projection (P1-5), the interval and label
rules, the ledger-derived episode lengths, the checkpoint pins, and the preregistered controls on CAM-K1-E's real cells
(9 blocks, full and at quota 8). Tests that need the training receipt, the checkpoints or the CAM-K1-E raw cells are
skipped with a reason when those files are absent (review P2-9); a skip is never a pass.
Run from tests/: python -B -m unittest test_cams2a_eval"""
import importlib.util
import json
import pathlib
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
K1E_RAW = pathlib.Path.home() / "motar_tmp" / "camk1e_20261108" / "raw"


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_tool(name):
    """tools/<name>.py under its production module name (the analysis imports cams2a_eval_run), tools/ importable."""
    import sys
    if str(ROOT / "tools") not in sys.path:
        sys.path.insert(0, str(ROOT / "tools"))
    spec = importlib.util.spec_from_file_location(name, str(ROOT / "tools" / (name + ".py")))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


V = _load_tool("cams2a_eval_run")
AN = _load_tool("cams2a_eval_analyse")
HAVE_TRAIN = V.TRAIN_RECEIPT.is_file() and V.TRAIN_REPORT.is_file()
TRAIN = json.loads(V.TRAIN_RECEIPT.read_text()) if HAVE_TRAIN else None
TENV = TRAIN["env_navrl"] if TRAIN else None
needs_train = unittest.skipUnless(HAVE_TRAIN, "training receipt %s is required (skipped, not passed)" % V.TRAIN_RECEIPT)


@needs_train
class CellEnvironment(unittest.TestCase):
    def test_training_environment_is_reproduced_except_the_replacements(self):
        self.assertEqual(len(TENV), 161)                                                  # review P2-8
        env = V.cell_navrl_env(TENV, 4502, "S12k")
        for k, v in TENV.items():
            if k in V.REMOVED_FROM_RECEIPT:
                self.assertNotIn(k, env, k)
            elif k in V.EVAL_OVERRIDES or k in V.MODULE_SEED_KEYS:
                self.assertNotEqual(env[k], v, k)
            else:
                self.assertEqual(env[k], v, k)
        self.assertEqual(env["NAVRL_DENSITY_CURRICULUM"], "0")
        self.assertEqual(env["NAVRL_VEH_RAND_SEED"], "45021")
        self.assertEqual(env["NAVRL_CAMS2_DELAY_SEED"], "45024")
        self.assertNotIn("NAVRL_YAWCLIP", env)
        self.assertEqual((env["NAVRL_EPISODE_LEDGER"], env["NAVRL_EPISODE_QUOTA_PER_ENV"], env["NAVRL_CAMS2DIAG"],
                          env["NAVRL_CAMS2DIAG_YAW_REF_RADPS"], env["NAVRL_CAMS2_CONTACTCODE"]), ("1", "8", "1", "1.047", "1"))
        self.assertEqual(V.cell_navrl_env(TENV, 4501, "S12k_nodiag")["NAVRL_CAMS2_CONTACTCODE"], "1")   # amendment 1: every cell
        self.assertEqual(V.cell_navrl_env(TENV, 4501, "S12k_smoke", quota=1)["NAVRL_EPISODE_QUOTA_PER_ENV"], "1")
        self.assertEqual(env["NAVRL_CAMS2_CONTACT_RADIUS_M"], "0.40")
        self.assertEqual(env["NAVRL_SPEED_GOVERNOR_FREE_MPS"], "2.5")
        self.assertEqual(env["NAVRL_GAIN_RAND"], "1")
        self.assertEqual(env["NAVRL_TARGET_PATTERN"], "mixed")
        yc = V.cell_navrl_env(TENV, 4502, "YC12k")
        self.assertEqual((yc["NAVRL_YAWCLIP"], yc["NAVRL_YAWCLIP_RADPS"], yc["NAVRL_YAW_RATE_MAX"]), ("1", "1.047", "3.0"))
        nodiag = V.cell_navrl_env(TENV, 4501, "S12k_nodiag")                              # review P1-1
        self.assertNotIn("NAVRL_CAMS2DIAG", nodiag)
        self.assertNotIn("NAVRL_CAMS2DIAG_YAW_REF_RADPS", nodiag)
        self.assertEqual(nodiag, {k: v for k, v in V.cell_navrl_env(TENV, 4501, "S12k").items() if not k.startswith("NAVRL_CAMS2DIAG")})

    def test_module_seeds_are_distinct_from_training(self):
        for seed in V.SEEDS:
            for k, v in V.module_seeds(seed).items():
                self.assertNotEqual(v, TENV[k], k)
        self.assertNotIn("1", [str(s) for s in V.SEEDS])

    def test_refuses_an_environment_without_the_cams2_rules(self):                        # training review F2
        for drop in ("NAVRL_CAMS2", "NAVRL_CAMS2_CONTACT_RADIUS_M", "NAVRL_CAMSAFE", "NAVRL_DYNAMIC_MESH_TREATMENT",
                     "NAVRL_CAMS2_DELAY_SEED"):
            with self.subTest(drop=drop), self.assertRaises(ValueError):
                V.cell_navrl_env({k: v for k, v in TENV.items() if k != drop}, 4501, "S12k")
        with self.assertRaises(ValueError):
            V.cell_navrl_env(dict(TENV, NAVRL_CAMS2="0"), 4501, "S12k")
        self.assertTrue(V.cell_navrl_env(TENV, 4501, "S12k"))                            # positive control

    def test_cell_env_pins_runtime_and_drops_gpu4gb(self):                                  # review P2-2, r2 P3-6
        import os
        from unittest import mock
        protocol = {"evaluation": {"player_games_cap": 10 ** 9}}
        with mock.patch.dict(os.environ, {"GPU4GB": "1", "AERIAL_GYM_SIM_NAME": "base_sim_4gb", "PYTHON": "/usr/bin/python3"}):
            env = V.cell_env(ROOT, pathlib.Path("/tmp/out"), "S12k", 175, 4502, "/x.pth", "f" * 64, TENV, "t" * 64, protocol)
        self.assertNotIn("GPU4GB", env)
        self.assertEqual((env["AERIAL_GYM_SIM_NAME"], env["PYTHON"]), (V.SIM_NAME, V.REAL_PYTHON))
        self.assertEqual((env["ARMS"], env["GAMES"], env["NUM_ENVS"], env["CAMS2A_CKPT_SHA"]), ("S2A_trained", "1024", "128", "f" * 64))


class Plan(unittest.TestCase):
    def test_densities_avoid_multiples_of_four_and_the_plan_is_block_major(self):
        self.assertTrue(all(d % 4 for d in V.DENSITIES))
        self.assertIn(175, V.DENSITIES)
        self.assertEqual(V.SEEDS, (4501, 4502, 4503))
        plan = V.eval_plan()
        self.assertEqual(len(plan), 45)
        self.assertEqual(len(V.required_cells()), 33)
        self.assertEqual([x[3] for x in plan], ["A"] * 27 + ["B"] * 6 + ["C"] * 12)
        self.assertEqual(plan[:3], [("S1k", 115, 4501, "A"), ("S12k", 115, 4501, "A"), ("YC12k", 115, 4501, "A")])
        self.assertEqual(plan[27], ("S3k", 175, 4501, "B"))
        self.assertEqual(len(V.IDENTITY_PLAN), 5)
        self.assertIn(("S12k_nodiag", 205, 4501), V.IDENTITY_PLAN)

    def test_diff_allowlist(self):                                                           # review P1-3
        ok = ["docs/plans/prereg_camonly_s2a_eval_2026-11-24.md", "WORKLOG.md", "tests/test_yawclip.py", "tools/realveh_entry.py",
              "tools/cams2a_eval_run.py", "aerial_gym/task/navrl_task/navrl_yawclip.py", "docs/evidence/x/REVIEW_r1.md"]
        self.assertEqual(V.diff_allowlist_problems(ok), [])
        for bad in ("aerial_gym/rl_training/rl_games/runner.py", "aerial_gym/task/navrl_task/target_motion.py",
                    "aerial_gym/task/navrl_task/navrl_cams2.py", "tools/gr_run.py", "aerial_gym/rl_training/rl_games/ppo_navrl_perception_transformer.yaml"):
            self.assertEqual(len(V.diff_allowlist_problems([bad])), 1, bad)
        self.assertEqual(len(V.diff_allowlist_problems(ok + ["aerial_gym/rl_training/rl_games/runner.py"])), 1)

    def test_mismatch_scan(self):                                                            # review P1-4
        text = ("x\nWARNING NavRL TRAINING SEED MISMATCH | checkpoint=1 running=4501. y\n"
                "NavRL CURRICULUM CONFIG MISMATCH | NAVRL_DENSITY_START: checkpoint used 70.000, running 205.000\n")
        self.assertEqual(V.mismatch_problems(text), [])
        self.assertEqual(len(V.mismatch_lines(text)), 2)
        bad = text + "NavRL CONFIG MISMATCH | cfg_max_velocity: checkpoint trained with 2.500, running with 3.500.\n"
        self.assertEqual(len(V.mismatch_problems(bad)), 1)
        self.assertEqual(len(V.mismatch_problems(text + "NavRL ACTION STD MISMATCH | checkpoint=a running=b.\n")), 1)
        self.assertEqual(V.mismatch_problems("no warnings here\n"), [])
        # amendment 1: the per-cell P9 module seed line, exactly as identity_01 logged it (with the colour codes)
        real = ("\x1b[33;20m[13006 ms][navrl_task] - WARNING : NavRL CONFIG MISMATCH | NAVRL_P9_SEED: checkpoint trained with "
                "4952.000, running with 45013.000. This changes policy inputs -- results are NOT comparable unless intentional. "
                "(navrl_task.py:4498)\x1b[0m\n")
        self.assertEqual(V.mismatch_problems(real), [])
        self.assertEqual(len(V.mismatch_problems(real.replace("NAVRL_P9_SEED", "NAVRL_DEPTH_SEED"))), 1)   # not allow-listed

    def test_smoke_plan_and_quota_parameter(self):                                           # amendment 1
        self.assertEqual((V.SMOKE["arm"], V.SMOKE["density"], V.SMOKE["seed"], V.SMOKE["quota"]), ("S12k_smoke", 205, 4501, 1))
        self.assertEqual(V.ARM_BASE["S12k_smoke"], "S12k")
        self.assertNotIn("S12k_smoke", [a for a, _, _ in V.IDENTITY_PLAN])
        self.assertNotIn("S12k_smoke", [a for a, _, _, _ in V.eval_plan()])
        want = V.expected_snapshot_values("S12k_smoke", 205, 4501, pathlib.Path("/tmp/c"), "c" * 64, quota=1)
        self.assertEqual((want["NAVRL_EPISODE_QUOTA_PER_ENV"], want["NAVRL_CAMS2_CONTACTCODE"]), ("1", "1"))
        self.assertIn("NAVRL_CAMS2_CONTACTCODE", V.SNAPSHOT_EXPECTED)
        self.assertIn(V.CONTACTCODE_REL, V.DIFF_ALLOWLIST)
        self.assertIn(V.AMENDMENT, V.COMMITTED_FILES)

    REAL_RESULT = pathlib.Path.home() / "motar_tmp/camk2v3e_20261110/raw/eval_01/V3s1/E2_obstacle_aware__115bars__seed4501/result.json"

    @unittest.skipUnless(REAL_RESULT.is_file(), "a real CAM-K2-E3 result.json is required (skipped, not passed)")
    def test_condition_check_against_a_real_result(self):                                    # review P2-3, r2 P0-1
        real = json.loads(self.REAL_RESULT.read_text())["condition"]
        self.assertIs(type(real["d8_dynamic_mesh_treatment"]), bool)                          # the task writes a bool
        self.assertIsNone(real["d8_dynamic_mesh_contract"])                                   # F's cells: no D8 treatment
        tenv = {"NAVRL_P9_EXPECTED_SHA256": "p" * 64}
        # the real condition of a Policy-F-contract cell must be refused (FREE 3.5355, P9 off, no D8 treatment)
        self.assertTrue(V.condition_problems(real, tenv))
        # the same real document with ONLY the CAM-S2a values substituted, in the task's own types, must pass
        cond = dict(real, speed_governor_free_mps=2.5, p9_empirical_error_enabled=True, p9_empirical_error_sha256="p" * 64,
                    d8_dynamic_mesh_treatment=True, d8_dynamic_mesh_contract={"mode": "mesh_flat", "asset_sha256": "x"})
        self.assertEqual(V.condition_problems(cond, tenv), [])
        for k, v in V.CONDITION_EXPECTED.items():
            self.assertIs(type(cond[k]), type(v), k)                                         # expectation types = real types
        self.assertTrue(V.condition_problems(dict(cond, d8_dynamic_mesh_treatment="mesh_flat"), tenv))    # the r2 defect
        self.assertTrue(V.condition_problems(dict(cond, d8_dynamic_mesh_contract={"mode": "mesh_shaded"}), tenv))
        self.assertTrue(V.condition_problems(dict(cond, d8_dynamic_mesh_contract=None), tenv))
        self.assertTrue(V.condition_problems(dict(cond, full_goal_distribution=False), tenv))
        self.assertTrue(V.condition_problems(dict(cond, speed_governor_free_mps=3.53553390593), tenv))
        self.assertTrue(V.condition_problems(dict(cond, p9_empirical_error_sha256="q" * 64), tenv))
        self.assertTrue(V.condition_problems(dict(cond, goal_dist_max_m=20.0), tenv))

    def test_rate_projection_and_gate(self):                                                 # review P1-5
        def report(s1k, ep12k):
            return {"records": [{"arm": a, "wall_s": (s1k if a == "S1k_smoke" else ep12k)} for a, _, _ in V.IDENTITY_PLAN]}
        proj = V.rate_projection(report(300.0, 400.0))
        self.assertEqual((proj["cells_required"], proj["s1k_cells"]), (33, 9))
        self.assertAlmostEqual(proj["projected_s"], (9 * 300.0 + 24 * 400.0) * 1.10)
        self.assertEqual(V.rate_problems(report(300.0, 400.0)), [])                        # 13,260 s < 18,000 s
        self.assertTrue(V.rate_problems(report(400.0, 560.0)))                              # 18,744 s > cap
        self.assertTrue(V.rate_problems({"records": [{"arm": "S12k_identity", "wall_s": 100.0}]}))   # incomplete
        self.assertTrue(V.rate_projection(report(300.0, 360.0))["stage_c_fits"])           # 12,474 + 4,752 s <= 18,000 s
        self.assertFalse(V.rate_projection(report(300.0, 480.0))["stage_c_fits"])          # A+B fit (15,642 s), C would not
        self.assertEqual(V.rate_problems(report(300.0, 480.0)), [])

    def test_identity_verdict(self):                                                         # review P2-6
        recs = [{"arm": a, "verdict": "OK"} for a, _, _ in V.IDENTITY_PLAN]
        eq, ne = {"result_equal": True, "episodes_equal": True}, {"result_equal": False, "episodes_equal": False}
        comp = {"S12k_repeat": eq, "S12k_nodiag": eq, "S1k_smoke": ne, "YC12k_negative": ne, "YC12k_negative_clipped_frac": 0.12}
        self.assertEqual(V.identity_verdict(recs, comp), "IDENTITY_PASS")
        self.assertEqual(V.identity_verdict(recs, dict(comp, S12k_repeat=ne)), "IDENTITY_FAIL")
        self.assertEqual(V.identity_verdict(recs, dict(comp, S12k_nodiag=ne)), "IDENTITY_FAIL")   # recorder not passive
        self.assertEqual(V.identity_verdict(recs, dict(comp, S1k_smoke=eq)), "IDENTITY_FAIL")
        self.assertEqual(V.identity_verdict(recs, dict(comp, YC12k_negative=eq)), "IDENTITY_FAIL")
        self.assertEqual(V.identity_verdict(recs, dict(comp, YC12k_negative_clipped_frac=0.0)), "IDENTITY_FAIL")
        self.assertEqual(V.identity_verdict(recs, dict(comp, YC12k_negative_clipped_frac=None)), "IDENTITY_FAIL")
        self.assertEqual(V.identity_verdict(recs[:-1], comp), "IDENTITY_FAIL")             # a missing cell
        self.assertEqual(V.identity_verdict(recs[:-1] + [{"arm": "YC12k_negative", "verdict": "FAIL"}], comp), "IDENTITY_FAIL")

    def test_eval_verdict(self):
        plan = V.eval_plan()
        ok = [{"arm": a, "density": d, "seed": s, "stage": st, "verdict": "OK"} for a, d, s, st in plan]
        self.assertEqual(V.eval_verdict(ok, plan), "COMPLETE")
        self.assertEqual(V.eval_verdict(ok[:33], plan), "COMPLETE")                         # stage C not run
        self.assertEqual(V.eval_verdict(ok[:40], plan), "COMPLETE")                         # stage C partial
        self.assertEqual(V.eval_verdict(ok[:40] + [dict(ok[40], verdict="FAIL", exit=124)], plan), "COMPLETE")   # r2 P0-3: cut C cell
        self.assertEqual(V.eval_verdict(ok[:32], plan), "INCOMPLETE")                       # stage B incomplete
        self.assertEqual(V.eval_verdict(ok[:32] + [dict(ok[32], verdict="FAIL")], plan), "INCOMPLETE")
        self.assertEqual(V.eval_verdict(ok[:26], plan), "INCOMPLETE")
        self.assertEqual(len(V.ab_records(ok)), 33)

    def test_stage_c_start_rule(self):                                                       # review r2 P0-3
        self.assertTrue(V.stage_c_may_start(500.0, 400.0))                                  # 500 >= 1.10 x 400
        self.assertFalse(V.stage_c_may_start(430.0, 400.0))
        self.assertFalse(V.stage_c_may_start(None, 400.0))
        self.assertFalse(V.stage_c_may_start(500.0, None))

    def test_scope_digest_matches_scope(self):
        import hashlib
        self.assertEqual(V.scope_digest(), hashlib.sha256(V.SCOPE.encode()).hexdigest())
        self.assertIn("stage A", V.SCOPE)


def _snapshot(cell_dir, arm, density, seed, ckpt_sha, **changes):
    snap = dict(V.cell_navrl_env(TENV, seed, arm))
    snap.update({"NAVRL_BULK_EVAL": "1", "NAVRL_BULK_EVAL_JSON": str(cell_dir / "result.json"), "NAVRL_EVAL_CHECKPOINT": ckpt_sha,
                 "NAVRL_EVAL_RUN_NONCE": "n", "NAVRL_NUM_BARS": str(density), "NAVRL_DENSITY_START": str(density),
                 "NAVRL_DENSITY_FINAL": str(density), "NAVRL_SEED": str(seed),
                 "NAVRL_REALVEH_RECEIPT": str(cell_dir / "realveh_receipt.json")})
    if V.diag(arm):
        snap["NAVRL_CAMS2DIAG_JSONL"] = str(cell_dir / V.DIAG_FILE)
    snap.update(changes)
    return snap


@needs_train
class Snapshot(unittest.TestCase):
    CK = "c" * 64

    def test_faithful_snapshot_passes_and_deviations_fail(self):
        d = pathlib.Path("/tmp/cell")
        for arm in ("S12k", "YC12k", "S12k_nodiag"):
            self.assertEqual(V.snapshot_problems(_snapshot(d, arm, 175, 4502, self.CK), TENV, arm, 175, 4502, d, self.CK), [], arm)
        bad = V.snapshot_problems(_snapshot(d, "S12k", 175, 4502, self.CK, NAVRL_SPEED_GOVERNOR_FREE_MPS="3.53553390593"), TENV, "S12k", 175, 4502, d, self.CK)
        self.assertTrue(any("NAVRL_SPEED_GOVERNOR_FREE_MPS" in p for p in bad))
        self.assertTrue(V.snapshot_problems(_snapshot(d, "YC12k", 175, 4502, self.CK), TENV, "S12k", 175, 4502, d, self.CK))    # clip on a plain arm
        self.assertTrue(V.snapshot_problems(_snapshot(d, "S12k", 175, 4502, self.CK), TENV, "S12k", 205, 4502, d, self.CK))     # wrong density
        self.assertTrue(V.snapshot_problems(_snapshot(d, "S12k", 175, 4502, self.CK, NAVRL_DENSITY_CURRICULUM="1"), TENV, "S12k", 175, 4502, d, self.CK))
        self.assertTrue(V.snapshot_problems(_snapshot(d, "S12k", 175, 4502, self.CK, NAVRL_KROC2027_INITIAL_VIEW="visible"), TENV, "S12k", 175, 4502, d, self.CK))
        self.assertTrue(V.snapshot_problems(_snapshot(d, "S12k", 175, 4502, self.CK), TENV, "S12k", 175, 4502, d, "d" * 64))    # other checkpoint (P2-1)
        self.assertTrue(V.snapshot_problems(_snapshot(d, "S12k", 175, 4502, self.CK, NAVRL_PLAYER_GAMES_CAP="5"), TENV, "S12k", 175, 4502, d, self.CK))
        self.assertTrue(V.snapshot_problems(_snapshot(d, "S12k", 175, 4502, self.CK), TENV, "S12k_nodiag", 175, 4502, d, self.CK))   # recorder on a nodiag arm


def _write_cell(cell_dir, arm, density, seed, committed, ckpt_sha, n_env=4, quota=2, **tweaks):
    """A synthetic, faithful cell: realveh receipt derived from the training receipt, launcher receipt, ledger, recorder
    rows, result.json and a cell.log with the telemetry lines."""
    cell_dir.mkdir(parents=True)
    seeds = V.module_seeds(seed)
    rec = json.loads(json.dumps(TRAIN))
    rec["settings"]["cams2"]["NAVRL_CAMS2_DELAY_SEED"] = int(seeds["NAVRL_CAMS2_DELAY_SEED"])
    rec["settings"]["vehicle_rand"]["NAVRL_VEH_RAND_SEED"] = int(seeds["NAVRL_VEH_RAND_SEED"])
    rec["settings"]["depth_model"]["NAVRL_DEPTH_SEED"] = int(seeds["NAVRL_DEPTH_SEED"])
    rec["sha256"]["realveh_entry"] = committed["realveh_entry"]
    rec["settings"]["cams2_contactcode"] = {}                                                   # amendment 1: every cell
    rec["cams2_contactcode"] = {"contact_code": 0}
    rec["cams2_contactcode_sha256"] = committed["contactcode"]
    if V.diag(arm):
        rec["settings"]["cams2diag"] = {"NAVRL_CAMS2DIAG_JSONL": str(cell_dir / V.DIAG_FILE), "NAVRL_CAMS2DIAG_YAW_REF_RADPS": 1.047}
        rec["cams2diag"] = {"jsonl": str(cell_dir / V.DIAG_FILE), "yaw_ref_radps": 1.047}
        rec["cams2diag_sha256"] = committed["cams2diag"]
    if V.yawclip(arm):
        rec["settings"]["yawclip"] = {"NAVRL_YAWCLIP_RADPS": 1.047}
        rec["yawclip"] = {"limit_radps": 1.047, "scale_radps": 3.0}
        rec["yawclip_sha256"] = committed["yawclip"]
    rec["env_navrl"] = _snapshot(cell_dir, arm, density, seed, ckpt_sha)                      # the entry's process env (P2-1)
    for k, v in tweaks.get("receipt", {}).items():
        rec[k] = v
    (cell_dir / "realveh_receipt.json").write_text(json.dumps(rec))
    (cell_dir / "receipt.json").write_text(json.dumps({"status": "OK", "train_receipt_sha256": V.sha(V.TRAIN_RECEIPT),
                                                        "env_snapshot_navrl": _snapshot(cell_dir, arm, density, seed, ckpt_sha),
                                                        "duration_s": 300, "sim_name": "base_sim", "python": V.REAL_PYTHON, "gpu4gb": ""}))
    ledger, rows, step = [], [], 0
    outcomes = ["capture", "crash", "crash", "timeout"]
    for ep in range(quota):
        for env in range(n_env):
            step += 7
            o = outcomes[(env + ep) % 4]
            ledger.append({"env_index": env, "env_episode_index": ep, "completion_step": step, "outcome": o, "in_quota": True,
                           "capture": o == "capture", "crash": o == "crash", "timeout": o == "timeout"})
            rows.append({"env_index": env, "env_episode_index": ep, "completion_step": step, "outcome": o,
                         "crash_class": ("target_contact_swept" if env == 1 else "contact") if o == "crash" else "none",
                         "delay_k": 1 + env % 2, "episode_steps": None,
                         "hidden_steps": 3, "visible_steps": 4, "hidden_raw_vx_mps": 1.0, "hidden_raw_abs_vy_mps": 0.5,
                         "hidden_raw_speed_mps": 1.2, "hidden_raw_abs_yaw_radps": 2.0, "hidden_raw_yaw_radps": -1.0,
                         "hidden_exec_speed_mps": 0.9, "hidden_exec_abs_yaw_radps": 1.0, "hidden_exec_over_blind_cap": 1,
                         "visible_raw_speed_mps": 2.0, "visible_raw_abs_yaw_radps": 0.5, "visible_exec_speed_mps": 1.8,
                         "visible_exec_abs_yaw_radps": 0.4, "yaw_request_over_ref_steps": 2,
                         "yaw_exec_over_ref_steps": 0 if V.yawclip(arm) else 2, "yaw_ref_radps": 1.047,
                         "capture_dist_m": 0.8 if o == "capture" else None, "capture_closing_mps": 0.4 if o == "capture" else None,
                         "capture_rel_speed_mps": 1.3 if o == "capture" else None, "capture_lateral_mps": 1.2 if o == "capture" else None})
    lengths = AN.episode_lengths(ledger)
    for r in rows:
        r["episode_steps"] = lengths[(r["env_index"], r["env_episode_index"])]
    if tweaks.get("flip_row_outcome"):
        rows[0]["outcome"] = "timeout" if rows[0]["outcome"] != "timeout" else "crash"
    (cell_dir / "episodes.jsonl").write_text("".join(json.dumps(r) + "\n" for r in ledger))
    if V.diag(arm):
        (cell_dir / V.DIAG_FILE).write_text("".join(json.dumps(r) + "\n" for r in rows))
    (cell_dir / "result.json").write_text(json.dumps({"crash_causes": {"count": 2, "bar_contact": 2}, "speed_governor": {"samples": 10,
                                                      "intervention_rate": 0.1, "mean_executed_speed_mps": 2.0, "mean_clearance_m": 5.0, "mean_scale": 0.9},
                                                      "condition": dict(V.CONDITION_EXPECTED, p9_empirical_error_sha256=TENV["NAVRL_P9_EXPECTED_SHA256"])}))
    log = "WARNING NavRL TRAINING SEED MISMATCH | checkpoint=1 running=%d. x\n" % seed
    log += "[camsafe] calls=320 capped_frac=0.0100 penalised_frac=0.0200 mean_penalty=0.00100 mean_r_ss_clip=-0.5 blind_mps=1 weight=0.5 rss_clip_m=4\n"
    if not tweaks.get("drop_contact_line"):
        log += "[cams2contact] {\"event\": \"summary\", \"steps\": 320, \"contacts\": 4, \"contact_steps\": 4}\n"
    if V.diag(arm):
        log += "[cams2diag] {\"event\": \"summary\", \"rows\": %d}\n" % len(rows)
    if V.yawclip(arm) and not tweaks.get("drop_yawclip_line"):
        log += "[yawclip] calls=320 commands=40960 clipped_frac=0.1234 mean_abs_yaw_before=0.9 mean_abs_yaw_after=0.6 limit_radps=1.047 scale_radps=3\n"
    log += tweaks.get("extra_log", "")
    (cell_dir / "cell.log").write_text(log)
    return cell_dir


@needs_train
class ReceiptChecks(unittest.TestCase):
    COMMITTED = {"realveh_entry": "e" * 64, "yawclip": "y" * 64, "cams2diag": "d" * 64, "contactcode": "k" * 64}
    CK = "c" * 64

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.base = pathlib.Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def _cell(self, arm, tag="", **tweaks):
        return _write_cell(self.base / (arm + tag) / (V.CELL % (175, 4502)), arm, 175, 4502, self.COMMITTED, self.CK, **tweaks)

    def test_faithful_cells_pass_plain_yawclip_and_nodiag(self):
        for arm in ("S12k", "YC12k", "S12k_nodiag"):
            d = self._cell(arm)
            self.assertEqual(V.receipt_problems(d, arm, 175, 4502, TRAIN, self.COMMITTED, self.CK), [], arm)
        self.assertEqual(V.diag_rows_problems(self.base / "S12k" / (V.CELL % (175, 4502))), [])

    def test_cell_without_cams2_install_fails(self):                                         # training review F2
        d = self._cell("S12k")
        rec = json.loads((d / "realveh_receipt.json").read_text())
        del rec["cams2"]
        del rec["settings"]["cams2"]
        (d / "realveh_receipt.json").write_text(json.dumps(rec))
        p = V.receipt_problems(d, "S12k", 175, 4502, TRAIN, self.COMMITTED, self.CK)
        self.assertTrue(any("F2" in x for x in p), p)

    def test_deviations_fail(self):
        d = self._cell("S12k")
        rec = json.loads((d / "realveh_receipt.json").read_text())
        rec["cams2"]["config_after"]["flight_altitude"] = 1.0                                     # not as trained
        (d / "realveh_receipt.json").write_text(json.dumps(rec))
        self.assertTrue(any("config_after" in x for x in V.receipt_problems(d, "S12k", 175, 4502, TRAIN, self.COMMITTED, self.CK)))
        d2 = self._cell("YC12k", drop_yawclip_line=True)
        self.assertTrue(any("[yawclip]" in x for x in V.receipt_problems(d2, "YC12k", 175, 4502, TRAIN, self.COMMITTED, self.CK)))
        d3 = self._cell("S1k", flip_row_outcome=True)
        self.assertTrue(any("same outcome" in x for x in V.diag_rows_problems(d3)))
        d4 = self._cell("S3k", receipt={"cams2_sha256": "0" * 64})                                 # another cams2 module
        self.assertTrue(any("cams2_sha256" in x for x in V.receipt_problems(d4, "S3k", 175, 4502, TRAIN, self.COMMITTED, self.CK)))
        d10 = self._cell("S12k", tag="_nocontactcode")
        rec = json.loads((d10 / "realveh_receipt.json").read_text())
        del rec["cams2_contactcode"]                                                             # amendment 1: record required
        (d10 / "realveh_receipt.json").write_text(json.dumps(rec))
        self.assertTrue(any("contact-code" in x for x in V.receipt_problems(d10, "S12k", 175, 4502, TRAIN, self.COMMITTED, self.CK)))
        d11 = self._cell("S12k_nodiag", tag="_nocontactline", drop_contact_line=True)
        self.assertTrue(any("[cams2contact]" in x for x in V.receipt_problems(d11, "S12k_nodiag", 175, 4502, TRAIN, self.COMMITTED, self.CK)))
        d8 = self._cell("YC12k", tag="_execover")
        rows = [json.loads(l) for l in (d8 / V.DIAG_FILE).read_text().splitlines()]
        rows[0]["yaw_exec_over_ref_steps"] = 1                                                   # the r2 P0-2 defect, as the runner sees it
        (d8 / V.DIAG_FILE).write_text("".join(json.dumps(r) + "\n" for r in rows))
        self.assertTrue(any("executed steps over the limit" in x for x in V.receipt_problems(d8, "YC12k", 175, 4502, TRAIN, self.COMMITTED, self.CK)))
        d9 = self._cell("YC12k", tag="_nobind")
        rows = [dict(json.loads(l), yaw_request_over_ref_steps=0) for l in (d9 / V.DIAG_FILE).read_text().splitlines()]
        (d9 / V.DIAG_FILE).write_text("".join(json.dumps(r) + "\n" for r in rows))
        self.assertTrue(any("never bound" in x for x in V.receipt_problems(d9, "YC12k", 175, 4502, TRAIN, self.COMMITTED, self.CK)))
        d5 = self._cell("S6k", extra_log="NavRL CONFIG MISMATCH | cfg_max_velocity: checkpoint trained with 2.500, running with 3.500.\n")
        self.assertTrue(any("checkpoint-contract" in x for x in V.receipt_problems(d5, "S6k", 175, 4502, TRAIN, self.COMMITTED, self.CK)))  # P1-4
        d6 = self._cell("S12k_nodiag")
        (d6 / V.DIAG_FILE).write_text("")                                                           # recorder file on the OFF control
        self.assertTrue(any("recorder OFF" in x for x in V.receipt_problems(d6, "S12k_nodiag", 175, 4502, TRAIN, self.COMMITTED, self.CK)))
        d7 = self._cell("S12k", tag="_entryenv")
        rec = json.loads((d7 / "realveh_receipt.json").read_text())
        rec["env_navrl"]["NAVRL_CAMTRAIN_HFOV_DEG"] = "87.0"                                        # entry env differs (P2-1)
        (d7 / "realveh_receipt.json").write_text(json.dumps(rec))
        self.assertTrue(any("entry env_navrl" in x for x in V.receipt_problems(d7, "S12k", 175, 4502, TRAIN, self.COMMITTED, self.CK)))

    def test_cell_diagnostics_from_ledger_and_recorder(self):
        d = self._cell("S12k")
        g = AN.cell_diagnostics(d)
        self.assertEqual(g["quota_episodes"], 8)
        self.assertEqual(g["steps_in_quota"], sum(AN.episode_lengths(V.K.read_jsonl(d / "episodes.jsonl")).values()))
        self.assertTrue(g["recorder_lengths_equal_ledger"])
        self.assertEqual(g["crashes"], 4)
        self.assertAlmostEqual(g["crash_per_1000_steps"], 1000.0 * 4 / g["steps_in_quota"])
        self.assertEqual(g["crash_classes"], {"target_contact_swept": 2, "contact": 2})
        self.assertEqual(g["capture_kinematics"]["n"], 2)
        self.assertAlmostEqual(g["capture_kinematics"]["lateral_mean"], 1.2)
        self.assertEqual(g["capture_kinematics"]["share_rel_speed_over_1mps"], 1.0)
        self.assertAlmostEqual(g["blind"]["hidden_step_share"], 3 / 7)
        self.assertAlmostEqual(g["blind"]["hidden_raw_abs_yaw_radps"], 2.0)
        self.assertEqual(g["delay_k_episodes"], {"1": 4, "2": 4})
        self.assertEqual(g["yaw"]["request_over_ref_steps"], 16)
        self.assertAlmostEqual(g["yaw"]["request_over_ref_share"], 16 / g["yaw"]["steps"])
        self.assertEqual(AN.binding_share({"S12k/175/4502": g}, "S12k"), 16 / g["yaw"]["steps"])
        self.assertIsNone(AN.binding_share({}, "YC12k"))

    def test_report_bindings(self):                                                              # review P2-6
        plan = V.eval_plan()
        recs = [{"arm": a, "density": d, "seed": s, "stage": st, "verdict": "OK"} for a, d, s, st in plan[:33]]
        ident = {"verdict": "IDENTITY_PASS", "commit": "abc", "env_allowlisted": {"HOME": "/h"}, "train_receipt_sha256": "t" * 64}
        ev = {"verdict": "COMPLETE", "commit": "abc", "identity_report": "/tmp/idrep.json", "cells_required": 33,
              "env_allowlisted": {"HOME": "/h"}, "train_receipt_sha256": "t" * 64, "records": recs, "changed_since_training": ["WORKLOG.md"]}
        grid = [(d, s) for d in V.DENSITIES for s in V.SEEDS]
        present = {"S1k": grid, "S12k": grid, "YC12k": grid, "S3k": [(175, s) for s in V.SEEDS], "S6k": [(175, s) for s in V.SEEDS]}
        b = AN.report_bindings(ident, ev, "/tmp/idrep.json", present)
        self.assertTrue(all(b.values()), b)
        self.assertFalse(AN.report_bindings(dict(ident, verdict="IDENTITY_FAIL"), ev, "/tmp/idrep.json", present)["identity_pass"])
        self.assertFalse(AN.report_bindings(ident, dict(ev, changed_since_training=["aerial_gym/rl_training/rl_games/runner.py"]),
                                            "/tmp/idrep.json", present)["changed_since_training_allowlisted"])
        self.assertFalse(AN.report_bindings(ident, ev, "/tmp/idrep.json", dict(present, S12k=grid[:-1]))["complete_grid_S12k"])
        self.assertFalse(AN.report_bindings(ident, dict(ev, records=recs[1:] + recs[:1]), "/tmp/idrep.json", present)["order_as_planned"])
        self.assertFalse(AN.report_bindings(ident, dict(ev, records=recs[:-1] + [dict(recs[-1], verdict="FAIL")]), "/tmp/idrep.json", present)["all_ab_records_ok"])
        cut_c = recs + [{"arm": "S3k", "density": 115, "seed": 4501, "stage": "C", "verdict": "FAIL", "exit": 124}]   # r2 P0-3
        b2 = AN.report_bindings(ident, dict(ev, records=cut_c), "/tmp/idrep.json", present)
        self.assertTrue(all(b2.values()), b2)

    def test_episode_lengths_are_differences_per_env(self):
        rows = [{"env_index": 0, "env_episode_index": 0, "completion_step": 12, "in_quota": True},
                {"env_index": 0, "env_episode_index": 1, "completion_step": 40, "in_quota": True},
                {"env_index": 1, "env_episode_index": 0, "completion_step": 600, "in_quota": True},
                {"env_index": 0, "env_episode_index": 2, "completion_step": 50, "in_quota": False}]
        self.assertEqual(AN.episode_lengths(rows), {(0, 0): 12, (0, 1): 28, (1, 0): 600})


class Rules(unittest.TestCase):
    def test_crash_label_rule(self):
        self.assertEqual(AN.crash_label([-9, -6], [-10, -5.5], -7.5), "MATERIAL_REDUCTION")
        self.assertEqual(AN.crash_label([-9, -6], [-10, -5.5], -7.5, timeout_mean=2.5), "CRASH_TO_TIMEOUT")
        self.assertEqual(AN.crash_label([-4, -1], [-4.5, -0.5], -2.5), "SMALL_REDUCTION")
        self.assertEqual(AN.crash_label([1, 5], [0.5, 5.5], 3.0), "WORSE")
        self.assertEqual(AN.crash_label([-1, 1], [-1.5, 1.5], 0.0), "NEGLIGIBLE")
        self.assertEqual(AN.crash_label([-1, 3], [-1.5, 3.5], 1.0), "NO_DETECTED_CHANGE")
        self.assertEqual(AN.crash_label([-3, -1], [-3.5, 0.5], -2.0), "UNRESOLVED")

    def test_gates(self):
        self.assertEqual(AN.gate_l("MATERIAL_REDUCTION", "HIGHER"), "LEARNING_SUPPORTED")
        self.assertEqual(AN.gate_l("NEGLIGIBLE", "HIGHER"), "LEARNING_SUPPORTED")
        self.assertEqual(AN.gate_l("WORSE", "HIGHER"), "LEARNING_REFUTED")
        self.assertEqual(AN.gate_l("NEGLIGIBLE", "NO_DETECTED_CHANGE"), "LEARNING_NOT_DETECTED")
        self.assertEqual(AN.gate_y("NEGLIGIBLE", "NEGLIGIBLE", True, 0.05), "YAWCLIP_TOLERATED")
        self.assertEqual(AN.gate_y("NEGLIGIBLE", "SMALL_LOWER", True, 0.05), "YAWCLIP_TOLERATED")   # admits a < 3 pp loss
        self.assertEqual(AN.gate_y("WORSE", "NEGLIGIBLE", True, 0.05), "YAWCLIP_SENSITIVE")
        self.assertEqual(AN.gate_y("NEGLIGIBLE", "LOWER", True, 0.05), "YAWCLIP_SENSITIVE")
        self.assertEqual(AN.gate_y("NO_DETECTED_CHANGE", "NEGLIGIBLE", True, 0.05), "YAWCLIP_UNRESOLVED")
        self.assertEqual(AN.gate_y("NEGLIGIBLE", "NEGLIGIBLE", False, 0.05), "YAWCLIP_INACTIVE")
        self.assertEqual(AN.gate_y("NEGLIGIBLE", "NEGLIGIBLE", True, 0.005), "YAWCLIP_WEAKLY_BINDING")   # review P1-2
        self.assertEqual(AN.gate_y("NEGLIGIBLE", "NEGLIGIBLE", True, None), "YAWCLIP_WEAKLY_BINDING")
        self.assertEqual(AN.gate_y("WORSE", "LOWER", True, 0.01), "YAWCLIP_SENSITIVE")               # exactly at the minimum

    def test_strat_t_df6_by_hand(self):
        blocks = [(d, s) for d in (115, 175, 205) for s in (1, 2, 3)]
        xs = [1.0, 2.0, 3.0, 2.0, 2.0, 2.0, 0.0, 4.0, 2.0]
        (lo, hi), df = AN.strat_t(xs, blocks)
        self.assertEqual(df, 6)
        import math
        se = math.sqrt((1.0 / 3 + 0.0 / 3 + 4.0 / 3) / 9)
        self.assertAlmostEqual(hi - lo, 2 * 2.446911851 * se, places=9)
        self.assertAlmostEqual((lo + hi) / 2, 2.0)

    def test_shifted_copy(self):
        cells = {"A": {(1, 1): {"crash": 0.1, "capture": 0.8, "timeout": 0.1}}, "B": {(1, 1): {"crash": 0.1, "capture": 0.8, "timeout": 0.1}}}
        s = AN.shifted(cells, "A", 3.0)
        self.assertAlmostEqual(s["A"][(1, 1)]["crash"], 0.13)
        self.assertAlmostEqual(s["A"][(1, 1)]["capture"], 0.77)
        self.assertEqual(s["B"], cells["B"])
        self.assertEqual(cells["A"][(1, 1)]["crash"], 0.1)                                  # the original is untouched


@needs_train
class Checkpoints(unittest.TestCase):
    def test_pins_match_the_training_hash_list_and_files(self):
        out, problems = V.checkpoints()
        if any("missing" in p for p in problems):
            self.skipTest("checkpoint files absent on this host (skipped, not passed): %s" % problems[:1])
        self.assertEqual(problems, [])
        self.assertEqual(sorted(out), ["S12k", "S1k", "S3k", "S6k"])
        for arm, (path, pin) in out.items():
            self.assertTrue(pathlib.Path(path).name.startswith("last_gen_ppo_ep_"), arm)
        bad, problems = V.checkpoints(hashes_path=pathlib.Path("/nonexistent"))
        self.assertTrue(problems)                                                          # negative control

    def test_training_run_accepted_and_altered_refused(self):
        report = json.loads(V.TRAIN_REPORT.read_text())
        self.assertEqual(V.training_problems(TRAIN, report), [])
        self.assertTrue(V.training_problems(TRAIN, dict(report, verdict="WALL_CAP")))
        self.assertTrue(V.training_problems(TRAIN, dict(report, faults_after=report["faults_before"] + 1)))


@unittest.skipUnless(K1E_RAW.is_dir(), "CAM-K1-E raw cells are required for the controls (skipped, not passed)")
class Controls(unittest.TestCase):
    def test_controls_on_camk1e_nine_blocks_full_and_quota8(self):
        c = AN.controls(K1E_RAW)
        self.assertTrue(c["camk1e_cells_match_manifest"])
        self.assertEqual(c["crash_on_K1E_F66_minus_K1s1"]["label"], "WORSE")
        self.assertEqual(c["crash_on_K1E_K1s1_minus_F66"]["label"], "MATERIAL_REDUCTION")
        self.assertNotIn(c["crash_on_K1E_K1s1_minus_K1s2"]["label"], ("MATERIAL_REDUCTION", "WORSE"))
        self.assertEqual(c["capture_on_K1E_K1s1_minus_F66"]["label"], "HIGHER")
        q8 = c["crash_on_K1E_K1s1_minus_K1s2_quota8"]
        self.assertEqual(q8["episodes_per_cell"], 1024)                                   # review P2-5: this unit's scale
        self.assertNotIn(q8["label"], ("MATERIAL_REDUCTION", "WORSE"))
        self.assertEqual(c["crash_on_K1E_K1s1_plus5pp_minus_K1s2_quota8"]["label"], "WORSE")
        self.assertEqual(c["capture_on_K1E_K1s1_minus5pp_minus_K1s2_quota8"]["label"], "LOWER")
        c3 = c["crash_contrast_plus3pp_K1s1_minus_K1s2_quota8"]                              # reported, not required (r2 P1-6)
        self.assertTrue(c3["ok"])
        self.assertAlmostEqual(c3["mean_pp"], 3.0, places=6)                                 # the shift centres the contrast at +3 pp
        self.assertEqual(c3["label"], "WORSE")
        self.assertAlmostEqual(c["crash_contrast_plus2pp_K1s1_minus_K1s2_quota8"]["mean_pp"], 2.0, places=6)
        self.assertAlmostEqual(c["quota8_null_mean_pp"] + c["contrast_shift_pp"]["3.0"], 3.0, places=9)
        self.assertTrue(c["ok"])
        self.assertEqual(c["crash_on_K1E_F66_minus_K1s1"]["t_df"], 6)
        self.assertEqual(c["crash_on_K1E_F66_minus_K1s1"]["episodes_per_cell"], 2048)


if __name__ == "__main__":
    unittest.main()
