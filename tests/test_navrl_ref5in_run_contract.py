"""CPU-only guards for the ref5in closed-run and provenance contracts."""

from pathlib import Path
import importlib.util
import os
import re
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
RL = ROOT / "aerial_gym/rl_training/rl_games"
LAUNCHER = RL / "train_navrl_v2_ref5in_smoke.sh"
CORRECTIVE_LAUNCHER = RL / "train_navrl_v2_ref5in_smoke_b.sh"
FINAL_CORRECTIVE_LAUNCHER = RL / "train_navrl_v2_ref5in_smoke_c.sh"
D1_LAUNCHER = RL / "train_navrl_v2_ref5in_d1_adapt.sh"
D1_EVALUATOR = ROOT / "tools/run_navrl_ref5in_d1_eval.py"
CV_HEADING_EVALUATOR = ROOT / "tools/run_navrl_ref5in_cv_heading_diagnostic.py"


class Ref5inSmokeLauncherContract(unittest.TestCase):
    def preflight(self, **updates):
        env = {
            **os.environ,
            "REF5IN_PREFLIGHT_ONLY": "1",
            "SEED": "999",
            "MAX_EPOCHS": "1",
            "NAVRL_ROBOT": "navrl_quad",
            "NAVRL_DENSITY_FINAL": "300",
            **updates,
        }
        return subprocess.run(
            [str(LAUNCHER)],
            cwd=RL,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )

    def test_hostile_environment_cannot_change_the_closed_contract(self):
        completed = self.preflight()
        self.assertEqual(completed.returncode, 0, completed.stdout)
        self.assertIn("robot=navrl_ref5in_quad", completed.stdout)
        self.assertIn("seed=197", completed.stdout)
        self.assertIn("epochs=500", completed.stdout)
        self.assertIn("density 70->205 bars", completed.stdout)
        self.assertIn("yaw=3.0 tilt=45.0", completed.stdout)
        self.assertIn("PREFLIGHT PASS", completed.stdout)
        self.assertNotIn("seed=999", completed.stdout)
        self.assertNotIn("70->300 bars", completed.stdout)

    def test_cli_and_checkpoint_resume_are_rejected(self):
        cli = subprocess.run(
            [str(LAUNCHER), "--checkpoint", "/tmp/hostile.pth"],
            cwd=RL,
            env={**os.environ, "REF5IN_PREFLIGHT_ONLY": "1"},
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        self.assertEqual(cli.returncode, 2, cli.stdout)
        self.assertIn("no CLI arguments", cli.stdout)
        inherited = self.preflight(CKPT="/tmp/hostile.pth")
        self.assertEqual(inherited.returncode, 2, inherited.stdout)
        self.assertIn("refusing inherited CKPT", inherited.stdout)

    def test_launcher_pins_source_receipt_and_storage_policy(self):
        source = LAUNCHER.read_text(encoding="utf-8")
        for literal in (
            "NAVRL_REQUIRE_TRAINING_SOURCE_RECEIPT=1",
            "NAVRL_REQUIRE_CLEAN_TRAINING_SOURCE",
            "NAVRL_SAVE_FREQUENCY=250",
            "NAVRL_SPEED_GOVERNOR=off",
            "NAVRL_PERCEPTION_PERTURB=0",
        ):
            self.assertIn(literal, source)

    def test_corrective_launcher_changes_only_preregistered_run_coordinates(self):
        env = {
            **os.environ,
            "REF5IN_PREFLIGHT_ONLY": "1",
            "SEED": "999",
            "MAX_EPOCHS": "1",
            "NAVRL_LEARNING_RATE": "1e-2",
            "NAVRL_ROBOT": "navrl_quad",
        }
        completed = subprocess.run(
            [str(CORRECTIVE_LAUNCHER)],
            cwd=RL,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stdout)
        self.assertIn("robot=navrl_ref5in_quad", completed.stdout)
        self.assertIn("seed=197", completed.stdout)
        self.assertIn("epochs=750", completed.stdout)
        self.assertIn("lr=1.5e-5", completed.stdout)
        self.assertIn("density 70->205 bars", completed.stdout)
        self.assertIn("PREFLIGHT PASS", completed.stdout)
        self.assertNotIn("seed=999", completed.stdout)
        self.assertNotIn("lr=0.01", completed.stdout)

        source = CORRECTIVE_LAUNCHER.read_text(encoding="utf-8")
        for literal in (
            "NAVRL_REQUIRE_TRAINING_SOURCE_RECEIPT=1",
            "NAVRL_REQUIRE_CLEAN_TRAINING_SOURCE",
            "NAVRL_SAVE_FREQUENCY=250",
            "NAVRL_SPEED_GOVERNOR=off",
            "NAVRL_PERCEPTION_PERTURB=0",
        ):
            self.assertIn(literal, source)

    def test_final_corrective_launcher_changes_only_the_budget(self):
        env = {
            **os.environ,
            "REF5IN_PREFLIGHT_ONLY": "1",
            "SEED": "999",
            "MAX_EPOCHS": "1",
            "NAVRL_LEARNING_RATE": "1e-2",
            "NAVRL_ROBOT": "navrl_quad",
        }
        completed = subprocess.run(
            [str(FINAL_CORRECTIVE_LAUNCHER)],
            cwd=RL,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stdout)
        self.assertIn("robot=navrl_ref5in_quad", completed.stdout)
        self.assertIn("seed=197", completed.stdout)
        self.assertIn("epochs=900", completed.stdout)
        self.assertIn("lr=1.5e-5", completed.stdout)
        self.assertIn("density 70->205 bars", completed.stdout)
        self.assertIn("PREFLIGHT PASS", completed.stdout)
        self.assertNotIn("seed=999", completed.stdout)
        self.assertNotIn("lr=0.01", completed.stdout)

        source = FINAL_CORRECTIVE_LAUNCHER.read_text(encoding="utf-8")
        self.assertNotIn("--checkpoint", source)
        for literal in (
            "NAVRL_REQUIRE_TRAINING_SOURCE_RECEIPT=1",
            "NAVRL_REQUIRE_CLEAN_TRAINING_SOURCE",
            "NAVRL_SAVE_FREQUENCY=250",
            "NAVRL_SPEED_GOVERNOR=off",
            "NAVRL_PERCEPTION_PERTURB=0",
        ):
            self.assertIn(literal, source)

    def test_d1_is_a_closed_q3_continuation_not_p3(self):
        completed = subprocess.run(
            [str(D1_LAUNCHER)],
            cwd=RL,
            env={
                **os.environ,
                "REF5IN_D1_PREFLIGHT_ONLY": "1",
                "SEED": "999",
                "MAX_EPOCHS": "2",
                "NAVRL_GENERAL_GOAL_DIST_MIN": "6",
                "NAVRL_DENSITY_CURRICULUM": "1",
            },
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stdout)
        self.assertIn("source=ep900 -> terminal=1900", completed.stdout)
        self.assertIn("applied_goal_range=22.5..28m", completed.stdout)
        self.assertIn("CONTINUATION", completed.stdout)
        self.assertIn("density 70->70 bars", completed.stdout)
        self.assertNotIn("seed=999", completed.stdout)
        source = D1_LAUNCHER.read_text(encoding="utf-8")
        for literal in (
            "EXPECTED_CKPT_SHA=f1670a1d",
            "NAVRL_DENSITY_CURRICULUM=0",
            "NAVRL_NUM_BARS=70",
            "NAVRL_GENERAL_GOAL_DIST_MIN=22.5",
            "NAVRL_SPEED_GOVERNOR=off",
            "NAVRL_REQUIRE_TRAINING_SOURCE_RECEIPT=1",
            "PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True",
        ):
            self.assertIn(literal, source)

    def test_d1_heldout_decision_is_fixed_before_training(self):
        source = D1_EVALUATOR.read_text(encoding="utf-8")
        for literal in (
            "SEED = 331",
            "EPISODES = 8193",
            '"global_crash_lte_27pct"',
            '"q3_crash_lte_30pct"',
            '"q3_cv_timeout_lte_12pct"',
            '"p2_verdict_changed": False',
            '"p3_automatically_unlocked": False',
            'mismatch_lines == [expected]',
            'env["NAVRL_V2_FORCE"] = "1"',
            '"editable aerial_gym VCS commit metadata only"',
            'pattern.sub(placeholder, train_text) == pattern.sub(placeholder, eval_text)',
        ):
            self.assertIn(literal, source)

    def test_post_d1_heading_diagnostic_is_frozen_and_non_decisional(self):
        source = CV_HEADING_EVALUATOR.read_text(encoding="utf-8")
        for literal in (
            "SEED = 337",
            "EPISODES = 2049",
            '("toward", "tangent_left", "tangent_right", "away")',
            '"NAVRL_V2_GOAL_DIST_MIN": "22.5"',
            '"NAVRL_V2_TARGET_PATTERN": "cv"',
            '"p2_verdict_changed": False',
            '"d1_verdict_changed": False',
            '"p3_unlocked": False',
            'mismatch_lines == [expected]',
            # Renamed from "path_length_support" when the screen was rewritten around the radial
            # heading channel; the literal list was not updated, so this test had been failing
            # against a key that no longer exists.
            '"radial_heading_channel_support"',
            '"chirality_sensitive"',
        ):
            self.assertIn(literal, source)

        task = (ROOT / "aerial_gym/task/navrl_task/navrl_task.py").read_text(
            encoding="utf-8"
        )
        for literal in (
            "NAVRL_EVAL_CV_INITIAL_HEADING is evaluation-only",
            'not os.environ.get("NAVRL_EVAL_CHECKPOINT", "").strip()',
            "NAVRL_EVAL_CV_INITIAL_HEADING requires NAVRL_TARGET_PATTERN=cv",
            '"initial_heading_max_contract_error"',
        ):
            self.assertIn(literal, task)


class SourceReceiptContract(unittest.TestCase):
    def test_runtime_snapshot_includes_robot_urdf(self):
        source = (ROOT / "tools/create_navrl_source_bundle.py").read_text(encoding="utf-8")
        self.assertIn('RUNTIME_ROOTS = ("aerial_gym", "resources/robots")', source)
        self.assertIn('".urdf"', source)
        self.assertIn("runtime_status", source)

    def test_create_and_verify_roundtrip_on_the_current_runtime(self):
        # This is a functional hash/snapshot check. It deliberately permits a dirty runtime here;
        # the launcher separately requires clean committed runtime sources for a real run.
        python = Path("/home/fair/miniconda3/envs/aerialgym/bin/python")
        if not python.is_file():
            self.skipTest("aerialgym Python unavailable")
        with tempfile.TemporaryDirectory() as directory:
            created = subprocess.run(
                [
                    str(python),
                    str(ROOT / "tools/create_navrl_source_bundle.py"),
                    "create",
                    "--output",
                    directory,
                ],
                cwd=ROOT,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )
            self.assertEqual(created.returncode, 0, created.stdout)
            manifest = Path(directory) / "source_manifest.json"
            verified = subprocess.run(
                [
                    str(python),
                    str(ROOT / "tools/create_navrl_source_bundle.py"),
                    "verify",
                    "--manifest",
                    str(manifest),
                ],
                cwd=ROOT,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )
            self.assertEqual(verified.returncode, 0, verified.stdout)
            self.assertIn('"verified": true', verified.stdout)


class RuntimeCheckpointContract(unittest.TestCase):
    def test_task_checkpoint_binds_robot_and_training_source(self):
        source = (ROOT / "aerial_gym/task/navrl_task/navrl_task.py").read_text(
            encoding="utf-8"
        )
        for literal in (
            '"cfg_robot_contract_version": 1',
            '"cfg_robot_config_path"',
            '"cfg_robot_config_sha256"',
            '"cfg_robot_asset_path"',
            '"cfg_robot_asset_sha256"',
            '"cfg_training_source_manifest_sha256"',
            "_verify_training_source_receipt()",
            "duplicate runtime path",
            "raise RuntimeError(\n                        \"NavRL ROBOT LINEAGE MISMATCH",
        ):
            self.assertIn(literal, source)

    def test_evaluator_restores_and_preflights_robot_source(self):
        source = (RL / "eval_navrl_v2_density_sweep.sh").read_text(encoding="utf-8")
        for literal in (
            'export NAVRL_ROBOT="${CHECKPOINT_ROBOT}"',
            "robot config source drift",
            "robot URDF source drift",
            '"resources/robots"',
            '".urdf"',
        ):
            self.assertIn(literal, source)

class FirstAcquisitionContract(unittest.TestCase):
    """RESEARCH_PLAN 8.28 first-acquisition diagnostic (seed 359).

    The failure this guards against is specific: an episode that never acquires the target must
    never be averaged in as if it acquired at step 0, because that would report the strongest
    failures as the fastest acquisitions and invert the finding the experiment exists to produce.
    """

    ORCHESTRATOR = ROOT / "tools/run_navrl_ref5in_cv_first_acquisition.py"
    TASK = ROOT / "aerial_gym/task/navrl_task/navrl_task.py"

    def test_preregistered_contract_literals_are_pinned(self):
        source = self.ORCHESTRATOR.read_text(encoding="utf-8")
        for literal in (
            "SEED = 359",
            'MODES = ("toward", "away")',
            "NEVER_ACQUIRED_GAP_THRESHOLD = 0.30",
            "DELAYED_ACQUISITION_STEP_THRESHOLD = 100",
            '"p2_verdict_changed": False',
            '"d1_verdict_changed": False',
            '"p3_unlocked": False',
            '"decision_authority": "none",',
            '"primary_metric": "fused_never_acquired_rate"',
        ):
            self.assertIn(literal, source, f"missing preregistered literal: {literal}")

    def test_thresholds_are_not_recomputed_from_results(self):
        """Both screens must compare against the module constants, never against a value derived
        from the cells."""
        source = self.ORCHESTRATOR.read_text(encoding="utf-8")
        self.assertIn("never_gap >= NEVER_ACQUIRED_GAP_THRESHOLD", source)
        self.assertIn("delay_gap >= DELAYED_ACQUISITION_STEP_THRESHOLD", source)

    def test_secondary_screen_only_applies_when_primary_fails(self):
        source = self.ORCHESTRATOR.read_text(encoding="utf-8")
        self.assertIn("(not primary_support)", source)

    def test_empty_cohorts_must_report_null_not_zero(self):
        source = self.ORCHESTRATOR.read_text(encoding="utf-8")
        self.assertIn('acquired==0 must report null first-visible statistics', source)
        task = self.TASK.read_text(encoding="utf-8")
        # The payload builder must guard every mean/median behind the acquired count.
        payload = task[task.index("def first_acquisition_payload"):]
        payload = payload[:payload.index("def target_motion_outcome_payload")]
        for guarded in ("if acquired else None", "if cam_acquired else None"):
            self.assertIn(guarded, payload)
        self.assertIn("median = None", payload)

    def test_never_acquired_sentinel_is_negative(self):
        task = self.TASK.read_text(encoding="utf-8")
        self.assertIn("self._fa_ep_first_fused[env_ids] = -1", task)
        self.assertIn("self._fa_ep_first_camera[env_ids] = -1", task)
        self.assertIn("acquired = first >= 0", task)

    def test_telemetry_is_bulk_eval_only_and_non_interfering(self):
        task = self.TASK.read_text(encoding="utf-8")
        block = task[task.index("self._fa_ep_obs_steps += valid_y_now"):]
        self.assertLess(
            task.index("if self._bulk_eval_mode:"),
            task.index("self._fa_ep_obs_steps += valid_y_now"),
            "first-acquisition counters must sit inside the bulk-eval guard",
        )
        # No observation/reward/termination consumer may read the counters back.
        for sink in ("task_obs", "self.rewards", "self.terminations", "self.truncations"):
            self.assertNotIn(sink, block[:block.index("def ")])

    def test_export_is_fail_closed_on_outcome_sums(self):
        task = self.TASK.read_text(encoding="utf-8")
        for guard in (
            "NavRL first-acquisition outcome mismatch",
            "NavRL first-acquisition histogram mismatch",
            "NavRL first-acquisition never-count exceeds episode count",
            "first-acquisition observation chronology diverged",
            "camera acquisition precedes fused acquisition",
        ):
            self.assertIn(guard, task, f"missing fail-closed guard: {guard}")

    def test_outcome_sum_guard_compares_like_with_like(self):
        """The guard is compared against a tuple; building a list here makes it fire on every run
        no matter what the counts are, which is exactly how it failed on first use."""
        task = self.TASK.read_text(encoding="utf-8")
        block = task[task.index("fa_outcomes = "):task.index("NavRL first-acquisition outcome")]
        self.assertIn("tuple(", block)
        self.assertNotIn("fa_outcomes = [", block)


class OOBExitForensicsContract(unittest.TestCase):
    """The seed-367 exit telemetry must stay diagnostic-only and cause-aligned."""

    TASK = ROOT / "aerial_gym/task/navrl_task/navrl_task.py"

    def test_recorder_consumes_the_cause_attributed_oob_mask(self):
        task = self.TASK.read_text(encoding="utf-8")
        attribution = "d_oob = oob & ~d_contact & ~below & ~above & crashed_out"
    ORCHESTRATOR = ROOT / "tools/run_navrl_ref5in_oob_exit_forensics.py"

    def test_recorder_consumes_the_cause_attributed_oob_mask(self):
        task = self.TASK.read_text(encoding="utf-8")
        attribution = "d_oob = oob & ~d_contact & ~below & ~above & crashed_out"
        call = "self._record_oob_exit(d_oob, pos, b_min, b_max, m_oob, steps)"
        self.assertIn(attribution, task)
        self.assertIn(call, task)
        self.assertLess(task.index(attribution), task.index(call))

    def test_recorder_is_bulk_eval_only_and_non_interfering(self):
        task = self.TASK.read_text(encoding="utf-8")
        call = task.index("self._record_oob_exit(d_oob, pos, b_min, b_max, m_oob, steps)")
        guard = task.rfind("if self._bulk_eval_mode:", 0, call)
        self.assertGreater(guard, task.rfind("if bool(d_oob.any()):", 0, call))
        block = task[task.index("def _record_oob_exit"):task.index("def _record_first_acquisition")]
        for sink in ("task_obs", "self.rewards", "self.terminations", "self.truncations"):
            self.assertNotIn(sink, block)

    def test_export_fails_closed_against_crash_cause_count(self):
        task = self.TASK.read_text(encoding="utf-8")
        payload = task[task.index("def oob_exit_payload"):task.index("def first_acquisition_payload")]
        self.assertIn('if n != int(self._diag["oob"]):', payload)
        self.assertIn("NavRL OOB forensics disagree with the crash-cause counter", payload)
        self.assertIn("NavRL OOB acquisition strata disagree with the exit counter", payload)
        self.assertIn('"by_acquisition"', payload)
        self.assertIn('("never_acquired", "acquired")', payload)
        for field in (
            '"never_acquired_share"',
            '"goal_closing_speed_mean_mps"',
            '"outward_radial_speed_mean_mps"',
            '"speed_mean_mps"',
            '"goal_distance_mean_m"',
            '"step_median"',
        ):
            self.assertIn(field, payload)

    def test_missing_source_telemetry_fails_closed(self):
        task = self.TASK.read_text(encoding="utf-8")
        recorder = task[task.index("def _record_oob_exit"):task.index("def _record_first_acquisition")]
        self.assertIn("NavRL OOB forensics require first-acquisition telemetry", recorder)
        self.assertIn("NavRL OOB forensics require robot_linvel", recorder)

    def test_edge_buckets_are_explicitly_nonexclusive(self):
        task = self.TASK.read_text(encoding="utf-8")
        recorder = task[task.index("def _record_oob_exit"):task.index("def _record_first_acquisition")]
        payload = task[task.index("def oob_exit_payload"):task.index("def first_acquisition_payload")]
        self.assertIn("a diagonal corner exit lands", recorder)
        self.assertIn("A corner crossing increments two edges", payload)
    def test_rerun_reuses_frozen_seed367_contract(self):
        source = self.ORCHESTRATOR.read_text(encoding="utf-8")
        for literal in (
            'OUTPUT = ROOT / "results/navrl_ref5in_oob_exit_forensics_seed367"',
            "BASE.INCLUDE_OOB_FORENSICS = True",
            'BASE.SUMMARY_SCOPE = "frozen_seed367_oob_exit_forensics_20m_28m"',
            "CHECKPOINT_ROBOT_CONFIG_SHA256 = (",
            "refusing robot config bytes that differ from the frozen checkpoint",
            'env["PYTHONPATH"] = str(ROOT)',
            "refusing aerial_gym imported outside clean worktree",
            "return BASE.main()",
        ):
            self.assertIn(literal, source)
        # The wrapper must not define a second set of experimental knobs that could drift from
        # the audited base contract.
        for forbidden in ("SEED =", "ARMS =", "EPISODES =", "TIMEOUT_DROP_THRESHOLD ="):
            self.assertNotIn(forbidden, source)


class ReflectionAuditContract(unittest.TestCase):
    """Prereg 2026-08-21 N1 real-frame reflection audit (seed 373).

    Two failures this guards against.  First, a preregistered constant silently drifting: seed,
    density, episode budget and the minimum frame count are the whole comparability of the cell.
    Second, and worse, the launcher quietly acquiring authority it was never granted -- the three
    ``*_changed`` / ``p3_unlocked`` literals must stay false in the emitted summary, because this
    experiment cannot revise P2 or D1 and cannot unlock P3.
    """

    ORCHESTRATOR = ROOT / "tools/run_navrl_ref5in_reflection_audit.py"
    OFFLINE_AUDIT = ROOT / "tools/navrl_reflection_offline_audit.py"
    PREREG = ROOT / "docs/prereg_2026-08-21_n1_real_frame_reflection_audit.md"
    FROZEN_AUDIT = (
        ROOT / "results/navrl_ref5in_reflection_audit_seed373/cells/reflection_audit"
        "/reflection_audit.json"
    )

    def source(self):
        return self.ORCHESTRATOR.read_text(encoding="utf-8")

    def test_preregistered_contract_literals_are_pinned(self):
        source = self.source()
        for literal in (
            "SEED = 373",
            "BARS = 70",
            "EPISODES = 1024",
            "MIN_FRAMES = 4096",
            "OBS_DUMP_STRIDE = 1",
            "OBS_DUMP_MAX = 16384",
            'CELL = "reflection_audit"',
            '"navrl_ref5in_reflection_audit_seed373"',
            "CHECKPOINT_SHA = "
            '"197ea26999d6bb9cf23c4e5a55acbe945f89985e2384687d60ab1dbae66a278e"',
            '"decision_authority": "none"',
            '"p2_verdict_changed": False',
            '"d1_verdict_changed": False',
            '"p3_unlocked": False',
            'PREREGISTRATION = "docs/prereg_2026-08-21_n1_real_frame_reflection_audit.md"',
        ):
            self.assertIn(literal, source, f"missing preregistered literal: {literal}")

    def test_gates_compare_against_constants_and_do_not_recompute(self):
        """Every gate must be a comparison against a module constant, never a value re-derived
        from the cell it is judging."""
        source = self.source()
        self.assertIn("frames_valid >= MIN_FRAMES", source)
        self.assertIn('"requested_episodes": EPISODES,', source)
        self.assertIn("== EPISODES and actual >= EPISODES", source)
        self.assertIn("P2.sha256_file(CHECKPOINT) == CHECKPOINT_SHA", source)
        self.assertIn('P2.sha256_file(paths["snapshot"]) == CHECKPOINT_SHA', source)
        self.assertIn('receipt.get("source_checkpoint_sha256") == CHECKPOINT_SHA', source)

    def test_verdict_thresholds_are_not_duplicated_in_the_launcher(self):
        """The prereg's decision thresholds live in exactly one place.  A second copy here could
        drift from the tool that actually applies them."""
        source = self.source()
        for threshold in ("0.30", "0.60", "0.10", "0.90"):
            self.assertNotIn(
                threshold, source, f"verdict threshold {threshold} duplicated in the launcher"
            )
        self.assertIn("classify_verdict", self.OFFLINE_AUDIT.read_text(encoding="utf-8"))

    def test_measurements_are_passed_through_verbatim(self):
        source = self.source()
        self.assertIn('audit.get("measurements_raw_normaliser")', source)
        self.assertIn('"verdict": verdict,', source)
        self.assertIn('"quality_gates": audit.get("quality_gates")', source)

    def test_episode_contract_is_at_least_not_exactly(self):
        """The evaluator drains whole 128-env batches, so a cell finishes at or just past the
        request.  Asserting exact equality here has already broken one arm of an earlier run."""
        source = self.source()
        self.assertIn("actual >= EPISODES", source)
        self.assertNotIn("actual == EPISODES", source)

    def test_pythonpath_is_reinjected_after_canonical_env(self):
        """P2.canonical_env deletes PYTHONPATH.  Setting it before that call is a no-op, and
        without it the run executes the PRIMARY worktree while hashing this one's bytes."""
        source = self.source()
        canonical = source[source.index("def canonical_env("):source.index("def offline_env(")]
        self.assertLess(
            canonical.index("P2.canonical_env(cell_dir()"),
            canonical.index('"PYTHONPATH": str(ROOT)'),
            "PYTHONPATH must be re-injected AFTER P2.canonical_env deletes it",
        )
        self.assertIn('"NAVRL_REQUIRE_SOURCE_ROOT": str(ROOT)', canonical)

    def test_import_origin_gate_fails_closed_on_a_missing_line(self):
        """A missing [origin] line means the guard never ran; that must fail, not pass."""
        source = self.source()
        self.assertIn(r"^\[origin\] aerial_gym ", source)
        self.assertIn(r"sha256=(?P<sha256>[0-9a-f]{64}) \(enforced\)$", source)
        self.assertIn("the import-origin guard did not run", source)
        self.assertIn("entry[0] == origin_sha", source)

    def test_no_provenance_override_is_ever_set(self):
        """This cell applies no target-pattern or CV intervention, so it must need no override."""
        source = self.source()
        self.assertNotIn('"NAVRL_V2_FORCE": "1"', source)
        self.assertNotIn('env["NAVRL_V2_FORCE"]', source)
        self.assertIn('"generic_provenance_override_used": False', source)

    def test_frames_npz_is_hash_bound_to_the_offline_stage(self):
        source = self.source()
        self.assertIn('audit.get("frames_sha256") == frames_sha', source)
        self.assertIn('"--frames-sha256", frames_sha', source)

    def test_run_refuses_to_overwrite_and_gates_on_a_clean_runtime(self):
        source = self.source()
        run_block = source[source.index('if mode == "run":'):source.index("cell = verify_cell()\n    expected")]
        self.assertIn("require(not OUTPUT.exists()", run_block)
        self.assertIn("verify_prerequisites(require_clean=True)", run_block)

    def test_preregistration_document_is_present_and_frozen_on_these_values(self):
        prereg = self.PREREG.read_text(encoding="utf-8")
        for literal in ("**373**", "**4,096**", "`NAVRL_OBS_DUMP_STRIDE = 1`", "16384", "1,024"):
            self.assertIn(literal, prereg, f"preregistration lost: {literal}")

    # ------------------------------------------------------------------------------------------
    # Behavioural guards.  The text assertions above cannot see whether a gate actually fires, so
    # the checks below import the launcher and call it.
    # ------------------------------------------------------------------------------------------

    FAKE_ORIGIN_ROOT = "/nonexistent/produced/in/another/worktree"

    @classmethod
    def launcher(cls):
        """Import the launcher once for behavioural assertions (no GPU, no subprocess)."""
        module = getattr(cls, "_launcher_module", None)
        if module is None:
            spec = importlib.util.spec_from_file_location(
                "reflection_audit_launcher_under_test", cls.ORCHESTRATOR
            )
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            cls._launcher_module = module
        return module

    def temp_dir(self):
        holder = tempfile.TemporaryDirectory()
        self.addCleanup(holder.cleanup)
        return Path(holder.name)

    def fake_cell(self, module, log_lines):
        """Repoint the launcher at a throwaway cell whose run log is exactly ``log_lines``."""
        output = self.temp_dir() / "results/cell_under_test"
        directory = output / "cells" / module.CELL
        directory.mkdir(parents=True)
        (directory / ("%dbars.log" % module.BARS)).write_text(
            "".join(line + "\n" for line in log_lines), encoding="utf-8"
        )
        self.addCleanup(setattr, module, "OUTPUT", module.OUTPUT)
        module.OUTPUT = output
        return directory

    def origin_log_line(self, root, digest):
        return "[origin] aerial_gym %s/aerial_gym/__init__.py sha256=%s (enforced)" % (root, digest)

    def launcher_cell(self, module, verdict, measurements, gates=None, failed_gates=None, **overrides):
        """A minimal verify_cell() return value, good enough to drive build_summary()."""
        table = gates if gates is not None else {
            "Q1_involution": {"passed": True},
            "Q6_import_origin": {"owner": "launcher", "passed": None},
            "Q7_checkpoint_sha_and_manifest": {"passed": True},
        }
        cell = {
            "result": {},
            "receipt": {},
            "audit": {
                "verdict": verdict,
                "measurements_raw_normaliser": measurements,
                "quality_gates": table,
                "failed_gates": failed_gates,
                "frames": {"n_frames_total": 15488, "n_frames_valid": 15488},
            },
            "frames_sha256": "0" * 64,
            "import_origin": {
                "enforced": True,
                "manifest_entry": "aerial_gym/__init__.py",
                "origin_sha256": "b" * 64,
                "manifest_sha256": "b" * 64,
                "log_line_occurrences": 1,
            },
            "manifest_provenance": {
                "checked_by_launcher": True,
                "receipt_schema_version": 2,
                "manifest_schema_version": 2,
                "runtime_clean_verified": True,
            },
            "actual_episodes": 1024,
            "condition": {},
        }
        cell.update(overrides)
        return cell

    # -- G1: Q6 belongs to the artifact, not to whoever happens to be verifying ------------------

    def test_import_origin_is_bound_to_the_artifact_not_to_the_verifiers_own_tree(self):
        """Q6 asserts one relation internal to the artifact: the aerial_gym that EXECUTED is the
        tree whose bytes the manifest hashed.  Pinning it to the verifier's ROOT adds a second,
        different claim and makes a worktree-produced result unverifiable from anywhere else."""
        module = self.launcher()
        digest = "a" * 64
        self.fake_cell(
            module,
            ["prelude", self.origin_log_line(self.FAKE_ORIGIN_ROOT, digest), "epilogue"],
        )
        record = module.verify_import_origin(
            {"aerial_gym/__init__.py": (digest, 271)},
            {"repository_root": self.FAKE_ORIGIN_ROOT},
        )
        self.assertEqual(record["required_source_root"], self.FAKE_ORIGIN_ROOT)
        self.assertEqual(record["origin"], self.FAKE_ORIGIN_ROOT + "/aerial_gym/__init__.py")
        self.assertEqual(record["origin_sha256"], digest)
        self.assertEqual(record["manifest_sha256"], digest)
        self.assertEqual(record["enforced"], True)
        self.assertNotIn(str(ROOT), repr(record), "Q6 must not depend on the verifier's own tree")

    def test_import_origin_still_fails_closed_after_being_rebound_to_the_manifest(self):
        """Deriving the expected tree from the artifact must not make the gate vacuous."""
        module = self.launcher()
        digest = "a" * 64
        mapping = {"aerial_gym/__init__.py": (digest, 271)}
        metadata = {"repository_root": self.FAKE_ORIGIN_ROOT}
        other = "/nonexistent/some/other/tree"

        cases = {
            "no [origin] line at all": ([], mapping, metadata),
            "guard line not marked enforced": (
                [self.origin_log_line(self.FAKE_ORIGIN_ROOT, digest)[: -len(" (enforced)")]],
                mapping,
                metadata,
            ),
            "line names a different tree": (
                [self.origin_log_line(other, digest)], mapping, metadata,
            ),
            "conflicting digests": (
                [
                    self.origin_log_line(self.FAKE_ORIGIN_ROOT, digest),
                    self.origin_log_line(self.FAKE_ORIGIN_ROOT, "c" * 64),
                ],
                mapping,
                metadata,
            ),
            "executed bytes are not the hashed bytes": (
                [self.origin_log_line(self.FAKE_ORIGIN_ROOT, digest)],
                {"aerial_gym/__init__.py": ("d" * 64, 271)},
                metadata,
            ),
            "manifest has no entry for the origin": (
                [self.origin_log_line(self.FAKE_ORIGIN_ROOT, digest)], {"aerial_gym/utils.py": (digest, 1)}, metadata,
            ),
            "manifest records no repository_root": (
                [self.origin_log_line(self.FAKE_ORIGIN_ROOT, digest)], mapping, {},
            ),
            "repository_root is not absolute": (
                [self.origin_log_line("relative/tree", digest)], mapping, {"repository_root": "relative/tree"},
            ),
        }
        for label, (lines, entries, meta) in cases.items():
            with self.subTest(case=label):
                self.fake_cell(module, lines)
                with self.assertRaises(module.ContractError):
                    module.verify_import_origin(entries, meta)

    def test_import_origin_pattern_is_not_compiled_against_the_verifiers_root(self):
        source = self.source()
        self.assertNotIn("ORIGIN_LINE = re.compile", source)
        self.assertNotIn('re.escape(str(ROOT / "aerial_gym" / "__init__.py"))', source)
        block = source[source.index("def verify_import_origin("):source.index("def require_import_origin_evidence(")]
        self.assertIn('metadata.get("repository_root")', block)
        self.assertIsNone(
            re.search(r"\bROOT\b", block), "Q6 must not read the verifier's own ROOT"
        )
        self.assertIn("verify_import_origin(mapping, metadata)", source)

    # -- G2: a receipt-recorded absolute path must not pin the artifact to one machine layout ----

    def test_manifest_lookup_prefers_the_copy_that_travels_with_the_cell(self):
        module = self.launcher()
        directory = self.fake_cell(module, [])
        elsewhere = self.temp_dir() / "source_manifest.json"
        elsewhere.write_text("recorded", encoding="utf-8")
        local = directory / "source_manifest.json"
        local.write_text("cell-local", encoding="utf-8")

        resolved = module.resolve_recorded_path(str(elsewhere), "runtime source manifest")
        self.assertEqual(resolved.read_text(encoding="utf-8"), "cell-local")

        local.unlink()
        resolved = module.resolve_recorded_path(str(elsewhere), "runtime source manifest")
        self.assertEqual(resolved.read_text(encoding="utf-8"), "recorded")

    def test_manifest_lookup_fails_closed_and_names_both_candidates(self):
        module = self.launcher()
        directory = self.fake_cell(module, [])
        missing = self.temp_dir() / "source_manifest.json"
        with self.assertRaises(module.ContractError) as caught:
            module.resolve_recorded_path(str(missing), "runtime source manifest")
        message = str(caught.exception)
        self.assertIn(str(directory / "source_manifest.json"), message)
        self.assertIn(str(missing), message)
        with self.assertRaises(module.ContractError):
            module.resolve_recorded_path("", "runtime source manifest")

    def test_both_recorded_manifests_are_located_by_bytes_not_by_path(self):
        source = self.source()
        block = source[source.index("def verify_cell("):source.index("def build_summary(")]
        self.assertNotIn('Path(str(receipt.get("runtime_source_manifest", ""))).resolve()', block)
        self.assertIn('resolve_recorded_path(\n        receipt.get("runtime_source_manifest")', block)
        self.assertIn('receipt.get("python_environment_manifest")', block)
        self.assertIn(
            'P2.sha256_file(manifest) == receipt.get("runtime_source_manifest_sha256")', block
        )
        self.assertIn(
            "P2.sha256_file(python_environment) == "
            'receipt.get("python_environment_manifest_sha256")',
            block,
        )

    # -- G3: the goal band is forced by the checkpoint contract, so it must be verified ----------

    def test_goal_band_is_pinned_and_verified_in_receipt_and_result(self):
        source = self.source()
        self.assertIn("GOAL_DIST_MIN_M = 22.5", source)
        self.assertIn("GOAL_DIST_MAX_M = 28.0", source)
        verify = source[source.index("def verify_cell("):source.index("def build_summary(")]
        receipt_block = verify[verify.index("pinned = {"):verify.index("receipt_mismatch = {")]
        condition_block = verify[
            verify.index("condition_mismatch = {"):verify.index("require(not condition_mismatch")
        ]
        for name, block in (("receipt", receipt_block), ("result condition", condition_block)):
            self.assertIn('"goal_dist_min_m": GOAL_DIST_MIN_M', block, f"{name} goal band unpinned")
            self.assertIn('"goal_dist_max_m": GOAL_DIST_MAX_M', block, f"{name} goal band unpinned")

    def test_exported_goal_band_cannot_drift_from_the_pinned_constants(self):
        """The band that is exported to the run and the band that is verified afterwards must be
        one value, or the receipt is checked against a number the run never used."""
        module = self.launcher()
        env = module.canonical_env(preflight=True)
        self.assertEqual(float(env["NAVRL_V2_GOAL_DIST_MIN"]), module.GOAL_DIST_MIN_M)
        self.assertEqual(float(env["NAVRL_V2_GOAL_DIST_MAX"]), module.GOAL_DIST_MAX_M)
        self.addCleanup(setattr, module, "GOAL_DIST_MAX_M", module.GOAL_DIST_MAX_M)
        module.GOAL_DIST_MAX_M = 30.0
        with self.assertRaises(module.ContractError):
            module.canonical_env(preflight=True)

    # -- G4: the fail-closed invariant is a biconditional, and it is code, not prose -------------

    def test_fail_closed_verdict_may_not_carry_measurements(self):
        module = self.launcher()
        ok = module.build_summary(self.launcher_cell(module, module.VERDICT_FAIL_CLOSED, None))
        self.assertEqual(ok["verdict"], module.VERDICT_FAIL_CLOSED)
        self.assertIsNone(ok["measurements"])
        with self.assertRaises(module.ContractError):
            module.build_summary(
                self.launcher_cell(module, module.VERDICT_FAIL_CLOSED, {"overall": {}})
            )

    def test_null_measurements_may_only_mean_fail_closed(self):
        module = self.launcher()
        ok = module.build_summary(
            self.launcher_cell(module, "CHIRALITY_CONFIRMED_REAL_FRAME", {"overall": {}})
        )
        self.assertEqual(ok["measurements"], {"overall": {}})
        with self.assertRaises(module.ContractError):
            module.build_summary(self.launcher_cell(module, "CHIRALITY_CONFIRMED_REAL_FRAME", None))
        with self.assertRaises(module.ContractError):
            module.build_summary(self.launcher_cell(module, None, {"overall": {}}))

    def test_fail_closed_constant_is_used_not_merely_defined(self):
        source = self.source()
        self.assertIn('VERDICT_FAIL_CLOSED = "FAIL_CLOSED_TRANSFORM_QUALITY"', source)
        block = source[source.index("def build_summary("):source.index("def _fmt(")]
        self.assertIn("(verdict == VERDICT_FAIL_CLOSED) == (measurements is None)", block)

    # -- G5: the gate tally must not be a tautology ----------------------------------------------

    def test_gate_tally_counts_verdicts_not_dictionary_keys(self):
        module = self.launcher()
        gates = {
            "Q1_involution": {"passed": True},
            "Q6_import_origin": {"owner": "launcher", "passed": None},
            "Q7_checkpoint_sha_and_manifest": {"passed": True},
        }
        evaluated, delegated = module.classify_gates(gates)
        self.assertEqual(delegated, ["Q6_import_origin"])
        self.assertNotIn("Q6_import_origin", evaluated)
        self.assertNotEqual(len(evaluated), len(gates), "the tally must not be len(quality_gates)")
        source = self.source()
        self.assertNotIn("{len(gates)}개 평가", source)
        self.assertIn("gate_tally(payload)", source)

    def test_summary_cannot_report_zero_failures_if_the_launcher_gate_is_gone(self):
        """If verify_import_origin() were deleted -- or simply never called -- Q6 would be judged
        by nobody, and the summary would still print a clean tally.  It must fail instead."""
        module = self.launcher()
        for label, cell in (
            ("evidence missing", self.launcher_cell(module, "CHIRALITY_CONFIRMED_REAL_FRAME", {"overall": {}}, import_origin=None)),
            ("evidence empty", self.launcher_cell(module, "CHIRALITY_CONFIRMED_REAL_FRAME", {"overall": {}}, import_origin={})),
            ("evidence not enforced", self.launcher_cell(module, "CHIRALITY_CONFIRMED_REAL_FRAME", {"overall": {}}, import_origin={"enforced": False, "manifest_entry": "aerial_gym/__init__.py", "origin_sha256": "b" * 64, "manifest_sha256": "b" * 64, "log_line_occurrences": 1})),
            ("executed bytes are not the hashed bytes", self.launcher_cell(module, "CHIRALITY_CONFIRMED_REAL_FRAME", {"overall": {}}, import_origin={"enforced": True, "manifest_entry": "aerial_gym/__init__.py", "origin_sha256": "b" * 64, "manifest_sha256": "e" * 64, "log_line_occurrences": 1})),
            ("no manifest evidence", self.launcher_cell(module, "CHIRALITY_CONFIRMED_REAL_FRAME", {"overall": {}}, manifest_provenance={})),
        ):
            with self.subTest(case=label):
                with self.assertRaises(module.ContractError):
                    module.build_summary(cell)
        payload = module.build_summary(
            self.launcher_cell(module, "CHIRALITY_CONFIRMED_REAL_FRAME", {"overall": {}})
        )
        payload["import_origin"] = {}
        with self.assertRaises(module.ContractError):
            module.gate_tally(payload)

    def test_every_delegated_gate_must_name_an_owner_the_launcher_recognises(self):
        module = self.launcher()
        self.assertEqual(module.LAUNCHER_OWNED_GATES["Q6_import_origin"], "import_origin")
        # A gate this launcher does not know about must never be waved through, whether it is
        # delegated by a null verdict, by an explicit status, or by naming this launcher as owner.
        for label, unclaimed in (
            ("no verdict", {"passed": None}),
            ("delegated status", {"passed": True, "status": "delegated"}),
            ("owner is this launcher", {"passed": True, "owner": module.PRODUCER}),
            ("delegation block names this launcher",
             {"passed": True, "delegation": {"owner": module.PRODUCER}}),
        ):
            with self.subTest(case=label):
                gates = {
                    "Q1_involution": {"passed": True},
                    "Q6_import_origin": {"owner": "launcher", "passed": None},
                    "Q7_checkpoint_sha_and_manifest": {"passed": True},
                    "Q10_unclaimed": unclaimed,
                }
                with self.assertRaises(module.ContractError):
                    module.build_summary(
                        self.launcher_cell(
                            module, "CHIRALITY_CONFIRMED_REAL_FRAME", {"overall": {}}, gates=gates
                        )
                    )
        missing_q6 = {"Q1_involution": {"passed": True}, "Q7_checkpoint_sha_and_manifest": {"passed": True}}
        with self.assertRaises(module.ContractError):
            module.build_summary(
                self.launcher_cell(module, "CHIRALITY_CONFIRMED_REAL_FRAME", {"overall": {}}, gates=missing_q6)
            )

    def test_launcher_owned_gates_track_the_offline_audits_delegation_contract(self):
        """The offline audit owns the spelling of its gate table.  Every gate it hands to this
        launcher must map to evidence this launcher actually produces, under either name."""
        module = self.launcher()
        offline = self.OFFLINE_AUDIT.read_text(encoding="utf-8")
        frozen = (self.FROZEN_AUDIT.read_text(encoding="utf-8") if self.FROZEN_AUDIT.is_file() else "")
        for name in module.LAUNCHER_OWNED_GATES:
            self.assertTrue(
                name in offline or name in frozen,
                f"{name} is claimed by the launcher but named by neither the offline audit tool "
                "nor the frozen seed-373 report",
            )
        delegated = offline[offline.index("DELEGATED_GATES = {"):offline.index("# ----", offline.index("DELEGATED_GATES = {"))]
        for name in re.findall(r'"(Q\d+[A-Za-z_]*)":', delegated):
            self.assertIn(
                name,
                module.LAUNCHER_OWNED_GATES,
                f"the offline audit delegates {name} to this launcher, which does not claim it",
            )
        # Both spellings of the Q7 half resolve to the one piece of evidence verify_cell builds.
        gates = {
            "Q1_involution": {"passed": True},
            "Q6_import_origin": {"passed": None, "status": "delegated"},
            "Q7_manifest_schema_version": {"passed": None, "status": "delegated"},
        }
        payload = module.build_summary(
            self.launcher_cell(module, "CHIRALITY_CONFIRMED_REAL_FRAME", {"overall": {}}, gates=gates)
        )
        evaluated, handed_off, failed = module.gate_tally(payload)
        self.assertEqual(evaluated, ["Q1_involution"])
        self.assertEqual(handed_off, ["Q6_import_origin", "Q7_manifest_schema_version"])
        self.assertEqual(failed, [])

    def test_failed_gate_list_must_agree_with_the_per_gate_verdicts(self):
        module = self.launcher()
        gates = {
            "Q1_involution": {"passed": False},
            "Q6_import_origin": {"owner": "launcher", "passed": None},
            "Q7_checkpoint_sha_and_manifest": {"passed": True},
        }
        with self.assertRaises(module.ContractError):
            module.build_summary(
                self.launcher_cell(module, "CHIRALITY_CONFIRMED_REAL_FRAME", {"overall": {}}, gates=gates)
            )
        payload = module.build_summary(
            self.launcher_cell(
                module, "CHIRALITY_CONFIRMED_REAL_FRAME", {"overall": {}},
                gates=gates, failed_gates=["Q1_involution"],
            )
        )
        self.assertEqual(payload["failed_gates"], ["Q1_involution"])
        for label, failed in (
            ("unknown gate name", ["Q99_invented"]),
            ("gate that reports passed=true", ["Q1_involution", "Q7_checkpoint_sha_and_manifest"]),
        ):
            with self.subTest(case=label):
                with self.assertRaises(module.ContractError):
                    module.build_summary(
                        self.launcher_cell(
                            module, "CHIRALITY_CONFIRMED_REAL_FRAME", {"overall": {}},
                            gates=gates, failed_gates=failed,
                        )
                    )


class SensorFidelityContract(unittest.TestCase):
    """Prereg 2026-08-22 sensor-model fidelity evaluation (seed 421).

    Four failures this guards against.  First, a preregistered constant drifting: seed, density,
    episode budget, goal band and the two arm definitions are the whole comparability of the pair.
    Second, the launcher acquiring authority it was never granted -- the three
    ``*_changed`` / ``p3_unlocked`` literals must stay false, because this experiment cannot revise
    P2 or D1 and cannot unlock P3.  Third, the seed-367 confound returning: ``detector_max_range``
    must never become a per-arm value, because moving it also renormalises the actor's target
    token.  Fourth, and most subtly, capture/crash/timeout leaking into the verdict -- the frozen
    policy was trained against the dishonest sensor, so its outcome rates are a lineage fact and
    the preregistration reports them raw and judges on never-acquired alone.
    """

    ORCHESTRATOR = ROOT / "tools/run_navrl_ref5in_sensor_fidelity.py"
    PREREG = ROOT / "docs/prereg_2026-08-22_sensor_fidelity.md"

    @classmethod
    def launcher(cls):
        """Import the launcher once for behavioural assertions (no GPU, no subprocess)."""
        module = getattr(cls, "_launcher_module", None)
        if module is None:
            spec = importlib.util.spec_from_file_location(
                "sensor_fidelity_launcher_under_test", cls.ORCHESTRATOR
            )
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            cls._launcher_module = module
        return module

    def source(self):
        return self.ORCHESTRATOR.read_text(encoding="utf-8")

    def test_preregistered_contract_literals_are_pinned(self):
        source = self.source()
        for literal in (
            "SEED = 421",
            "BARS = 70",
            "EPISODES = 2049",
            "GOAL_DIST_MIN_M = 22.5",
            "GOAL_DIST_MAX_M = 28.0",
            'ARMS = (("baseline", 160, 90, 2), ("fidelity", 1920, 1200, 50))',
            "NEVER_ACQUIRED_COST_THRESHOLD_PP = 10.00",
            "NEVER_ACQUIRED_NEUTRAL_BAND_PP = 3.00",
            "DETECTOR_MAX_RANGE_M = 20.0",
            "CAMERA_WIDTH = 160",
            "CAMERA_HEIGHT = 90",
            '"navrl_ref5in_sensor_fidelity_seed421"',
            "CHECKPOINT_SHA = "
            '"197ea26999d6bb9cf23c4e5a55acbe945f89985e2384687d60ab1dbae66a278e"',
            'PREREGISTRATION = "docs/prereg_2026-08-22_sensor_fidelity.md"',
            '"decision_authority": "none"',
            '"p2_verdict_changed": False',
            '"d1_verdict_changed": False',
            '"p3_unlocked": False',
            '"schema_version": 1,',
            'VERDICT_COST_CONFIRMED = "FIDELITY_COST_CONFIRMED"',
            'VERDICT_NEUTRAL = "FIDELITY_NEUTRAL"',
            'VERDICT_INCONCLUSIVE = "INCONCLUSIVE_SENSOR_FIDELITY"',
        ):
            self.assertIn(literal, source, f"missing preregistered literal: {literal}")

    def test_the_preregistration_it_names_exists_and_is_the_one_it_implements(self):
        self.assertTrue(self.PREREG.is_file(), "the named preregistration is missing")
        prereg = self.PREREG.read_text(encoding="utf-8")
        for literal in ("421", "2,049", "1920×1200", "20.0", "22.5–28 m"):
            self.assertIn(literal, prereg, f"preregistration does not contain {literal}")

    def test_arm_definitions_match_the_preregistration(self):
        module = self.launcher()
        self.assertEqual(
            module.ARMS, (("baseline", 160, 90, 2), ("fidelity", 1920, 1200, 50))
        )
        self.assertEqual(module.SEED, 421)
        self.assertEqual(module.BARS, 70)
        self.assertEqual(module.EPISODES, 2049)
        self.assertEqual(module.NEVER_ACQUIRED_COST_THRESHOLD_PP, 10.00)
        self.assertEqual(module.NEVER_ACQUIRED_NEUTRAL_BAND_PP, 3.00)
        self.assertEqual(module.DETECTOR_MAX_RANGE_M, 20.0)

    def test_thresholds_are_constants_referenced_by_the_comparison(self):
        """Both thresholds must be compared by NAME.  A literal 10.0 or 3.0 inside the verdict
        would be a second copy that can drift from the preregistered value."""
        source = self.source()
        verdict = source[source.index("def classify_verdict("):source.index("# ------", source.index("def classify_verdict("))]
        self.assertIn("delta_pp >= NEVER_ACQUIRED_COST_THRESHOLD_PP", verdict)
        self.assertIn("abs(delta_pp) <= NEVER_ACQUIRED_NEUTRAL_BAND_PP", verdict)
        for literal in ("10.0", "3.0", "0.10", "0.03"):
            self.assertNotIn(
                literal, verdict, f"verdict threshold {literal} duplicated inside the comparison"
            )

    def test_verdict_boundaries_are_exactly_the_preregistered_ones(self):
        module = self.launcher()
        for delta, expected in (
            (10.00, "FIDELITY_COST_CONFIRMED"),
            (37.42, "FIDELITY_COST_CONFIRMED"),
            (9.99, "INCONCLUSIVE_SENSOR_FIDELITY"),
            (3.01, "INCONCLUSIVE_SENSOR_FIDELITY"),
            (3.00, "FIDELITY_NEUTRAL"),
            (0.0, "FIDELITY_NEUTRAL"),
            (-3.00, "FIDELITY_NEUTRAL"),
            (-3.01, "INCONCLUSIVE_SENSOR_FIDELITY"),
            (-50.0, "INCONCLUSIVE_SENSOR_FIDELITY"),
        ):
            with self.subTest(delta_pp=delta):
                self.assertEqual(module.classify_verdict(delta), expected)

    def test_capture_crash_and_timeout_cannot_enter_the_verdict(self):
        """Prereg section 6 reports them raw and excludes them from the decision.  The verdict
        function takes one number, so the exclusion is structural rather than a promise."""
        module = self.launcher()
        source = self.source()
        verdict = source[source.index("def classify_verdict("):source.index("# ------", source.index("def classify_verdict("))]
        # The docstring is allowed to NAME the excluded quantities; the executable body is not.
        body = verdict[verdict.index('"""', verdict.index('"""') + 3) + 3:]
        for forbidden in ("capture", "crash", "timeout", "outcome_raw"):
            self.assertNotIn(
                forbidden, body, f"{forbidden} appears inside the verdict function body"
            )
        self.assertEqual(
            module.classify_verdict.__code__.co_argcount,
            1,
            "classify_verdict must take exactly the never-acquired delta",
        )
        # Nothing the function reads may name an outcome count: co_names is every global and
        # attribute it touches, so this closes the "read it from a module global" route too.
        for name in module.classify_verdict.__code__.co_names:
            self.assertNotIn("capture", name.lower())
            self.assertNotIn("crash", name.lower())
            self.assertNotIn("timeout", name.lower())

    def test_detector_max_range_is_never_set_per_arm(self):
        """Seed 367 moved this value and renormalised the actor's target token with it.  The
        preregistration turns on the range being untouched, so the launcher must not export it at
        all -- and must assert its own environment does not."""
        source = self.source()
        self.assertNotIn('"NAVRL_DETECTOR_MAX_RANGE": ', source)
        self.assertIn('"NAVRL_DETECTOR_MAX_RANGE" not in env', source)
        self.assertIn('"target_camera_max_range_m": DETECTOR_MAX_RANGE_M', source)
        for _, detect_w, detect_h, min_pixels in self.launcher().ARMS:
            self.assertNotEqual((detect_w, detect_h, min_pixels), (0, 0, 0))

    def test_arm_environments_differ_in_exactly_the_manipulated_variables(self):
        module = self.launcher()
        diff = module.arm_env_diff()
        self.assertEqual(
            sorted(diff),
            ["NAVRL_DETECTOR_MIN_PIXELS", "NAVRL_DETECT_HEIGHT", "NAVRL_DETECT_WIDTH"],
        )
        self.assertEqual(
            diff["NAVRL_DETECTOR_MIN_PIXELS"], {"baseline": "2", "fidelity": "50"}
        )
        self.assertEqual(diff["NAVRL_DETECT_WIDTH"], {"baseline": "160", "fidelity": "1920"})
        self.assertEqual(diff["NAVRL_DETECT_HEIGHT"], {"baseline": "90", "fidelity": "1200"})

    def test_min_pixels_override_lands_after_the_closed_environment(self):
        """P2.canonical_env already contains NAVRL_DETECTOR_MIN_PIXELS=2.  Updating before that
        call is silently overwritten and both arms would run the baseline threshold -- a two-arm
        experiment with one arm."""
        module = self.launcher()
        source = self.source()
        body = source[source.index("def canonical_env("):source.index("def arm_env_diff(")]
        self.assertLess(
            body.index("P2.canonical_env(cell_dir(arm)"),
            body.index('"NAVRL_DETECTOR_MIN_PIXELS": str(min_pixels)'),
            "the per-arm override must be applied AFTER P2.canonical_env",
        )
        self.assertLess(
            body.index("P2.canonical_env(cell_dir(arm)"),
            body.index('"PYTHONPATH": str(ROOT)'),
            "PYTHONPATH must be re-injected AFTER P2.canonical_env, which deletes it",
        )
        for arm, detect_w, detect_h, min_pixels in module.ARMS:
            env = module.canonical_env(arm, detect_w, detect_h, min_pixels, preflight=True)
            self.assertEqual(env["NAVRL_DETECTOR_MIN_PIXELS"], str(min_pixels))
            self.assertEqual(env["NAVRL_DETECT_WIDTH"], str(detect_w))
            self.assertEqual(env["NAVRL_DETECT_HEIGHT"], str(detect_h))
            self.assertEqual(env["NAVRL_CAMERA_WIDTH"], "160")
            self.assertEqual(env["NAVRL_CAMERA_HEIGHT"], "90")
            self.assertNotIn("NAVRL_DETECTOR_MAX_RANGE", env)
            # The narrow override sits on the fidelity arm alone (prereg section 5-b).
            self.assertEqual(
                env.get("NAVRL_V2_FORCE"), "1" if arm == module.FORCE_ARM else None
            )
            self.assertEqual(env["PYTHONPATH"], str(ROOT))
            self.assertEqual(env["NAVRL_REQUIRE_SOURCE_ROOT"], str(ROOT))

    def test_episode_contract_is_at_least_not_exactly(self):
        """The evaluator drains whole 128-env batches, so a cell finishes at or just past the
        request.  Asserting exact equality has already broken one arm of an earlier run."""
        source = self.source()
        self.assertIn("actual >= EPISODES", source)
        self.assertNotIn("actual == EPISODES", source)
        self.assertIn("== EPISODES and actual >= EPISODES", source)

    def test_expected_mismatch_is_a_module_constant_compared_by_name(self):
        """Prereg section 5-b.  The one authorised mismatch is pinned as a constant and the
        comparison references it; a literal string inside the check could drift from the value the
        preregistration froze."""
        module = self.launcher()
        source = self.source()
        self.assertEqual(
            module.EXPECTED_MISMATCH, "cfg_detector_min_pixels: checkpoint=2 expected=50.0"
        )
        self.assertEqual(module.FORCE_ARM, "fidelity")
        self.assertIn(
            'EXPECTED_MISMATCH = "cfg_detector_min_pixels: checkpoint=2 expected=50.0"', source
        )
        self.assertIn("lines == [EXPECTED_MISMATCH]", source)
        self.assertIn("unforced.returncode == 2", source)

    def test_only_the_fidelity_arm_may_ever_force(self):
        """Arm A must not use the override at all, and must not be able to acquire one even if a
        caller asks for it."""
        module = self.launcher()
        self.assertTrue(module.arm_requires_force("fidelity"))
        self.assertFalse(module.arm_requires_force("baseline"))
        baseline = module.canonical_env("baseline", 160, 90, 2, preflight=True)
        self.assertNotIn("NAVRL_V2_FORCE", baseline)
        fidelity = module.canonical_env("fidelity", 1920, 1200, 50, preflight=True)
        self.assertEqual(fidelity["NAVRL_V2_FORCE"], "1")
        # The unforced fidelity environment is what the proof step needs, and it must be reachable.
        unforced = module.canonical_env("fidelity", 1920, 1200, 50, preflight=True, force=False)
        self.assertNotIn("NAVRL_V2_FORCE", unforced)
        # An explicit force on the baseline arm is refused rather than honoured.
        with self.assertRaises(module.ContractError):
            module.canonical_env("baseline", 160, 90, 2, preflight=True, force=True)

    def test_summary_records_the_override_per_arm(self):
        source = self.source()
        self.assertIn('"narrow_provenance_override"', source)
        self.assertIn('"sole_verified_mismatch": EXPECTED_MISMATCH', source)
        self.assertIn('else {"used": False}', source)
        self.assertIn("narrow_provenance_override", self.launcher().SUMMARY_VERIFY_KEYS)

    def test_override_verification_runs_in_both_preflight_and_run(self):
        """A cell must never be produced under an override this process did not verify, so the
        proof cannot live in the `preflight` subcommand alone."""
        source = self.source()
        main = source[source.index("def main() -> int:"):]
        preflight_block = main[main.index('if mode == "preflight":'):main.index('if mode == "run":')]
        run_block = main[main.index('if mode == "run":'):main.index("verified = verify_all()")]
        self.assertIn("preflight_evaluator()", preflight_block)
        self.assertIn("preflight_evaluator()", run_block)
        self.assertLess(
            run_block.index("preflight_evaluator()"),
            run_block.index("run_arm("),
            "the override must be verified BEFORE any arm is executed",
        )
        self.assertIn("verify_narrow_override()", source)

    def test_a_mismatch_set_that_is_not_exactly_one_field_stops_the_run(self):
        """Two mismatch lines, a different field, or an evaluator that stops refusing at all must
        each abort -- that is what makes this stricter than a blanket force."""
        module = self.launcher()

        class FakeCompleted:
            def __init__(self, returncode, stdout):
                self.returncode = returncode
                self.stdout = stdout

        good = "  cfg_detector_min_pixels: checkpoint=2 expected=50.0"
        other = "  cfg_detector_threshold: checkpoint=0.55 expected=0.7"
        passing = "[eval_v2] PREFLIGHT PASS (evaluation not started)"
        cases = {
            "two mismatch lines": (2, "\n".join([good, other])),
            "a different field": (2, other),
            "no mismatch line at all": (2, "[eval_v2] REFUSING: something else"),
            "evaluator stopped refusing": (0, passing),
        }
        for label, (code, stdout) in cases.items():
            with self.subTest(case=label):
                calls = []

                def fake_run(arm, w, h, px, *, force, _code=code, _out=stdout):
                    calls.append(force)
                    return FakeCompleted(_code, _out)

                original = module.run_preflight
                module.run_preflight = fake_run
                try:
                    with self.assertRaises(module.ContractError):
                        module.verify_narrow_override()
                finally:
                    module.run_preflight = original
                self.assertEqual(
                    calls, [False], "the unforced proof must run first and force must not follow"
                )

    def test_the_verified_single_field_override_is_accepted(self):
        """The happy path: refused unforced with exactly one line, then passing under force."""
        module = self.launcher()

        class FakeCompleted:
            def __init__(self, returncode, stdout):
                self.returncode = returncode
                self.stdout = stdout

        calls = []

        def fake_run(arm, w, h, px, *, force):
            calls.append(force)
            if force:
                return FakeCompleted(0, "[eval_v2] PREFLIGHT PASS (evaluation not started)")
            return FakeCompleted(
                2,
                "[eval_v2] REFUSING: v2 contract mismatch:\n"
                "  cfg_detector_min_pixels: checkpoint=2 expected=50.0",
            )

        original = module.run_preflight
        module.run_preflight = fake_run
        try:
            self.assertEqual(module.verify_narrow_override(), module.EXPECTED_MISMATCH)
        finally:
            module.run_preflight = original
        self.assertEqual(calls, [False, True], "the unforced proof must precede the forced run")

    def test_never_acquired_is_read_from_the_evaluator_not_invented(self):
        """The primary measurand must come from the telemetry the evaluator already emits."""
        source = self.source()
        self.assertIn('(result.get("target_motion") or {}).get("first_acquisition")', source)
        self.assertIn('int(rows[label]["never_acquired"])', source)
        self.assertIn('result["action"]["context"]["target_hidden"]["fraction"]', source)
        self.assertIn("NEVER_ACQUIRED_SOURCE", source)

    def test_fail_closed_is_a_biconditional(self):
        source = self.source()
        self.assertIn(
            "(verdict == VERDICT_FAIL_CLOSED) == (published is None)",
            source,
            "the fail-closed contract must be enforced in both directions",
        )

    def test_limitations_are_transcribed_from_the_preregistration(self):
        module = self.launcher()
        self.assertEqual(len(module.LIMITATIONS), 5)
        for index, item in enumerate(module.LIMITATIONS, start=1):
            self.assertTrue(
                item.startswith(f"L{index}:"), f"limitation {index} is not labelled L{index}"
            )

    def test_summary_authority_literals_stay_false(self):
        source = self.source()
        authority = source[source.index('"schema_version": 1,'):source.index('"limitations": list(LIMITATIONS)')]
        for literal in (
            '"decision_authority": "none",',
            '"p2_verdict_changed": False,',
            '"d1_verdict_changed": False,',
            '"p3_unlocked": False,',
        ):
            self.assertIn(literal, authority, f"summary lost the authority literal {literal}")


class DetectionRangeStage1Contract(unittest.TestCase):
    """Prereg 2026-08-22 detection-range stage 1 (screening): TRAIN two arms, then evaluate each.

    This is the repository's first PREREGISTERED TRAINING launcher, so the failures it guards
    against are not the evaluation ones.

    First, a held-fixed condition that the canonical trainer hard-codes.  ``NAVRL_DETECTOR_MIN_PIXELS``
    was exported literally as 2 by train_navrl_v2_search.sh: a child launcher asking for the
    honest-sensor 50 would have trained at 2, both arms would still have trained normally, and the
    resulting checkpoints would have carried the dishonest threshold.  The launcher therefore reads
    back the environment the trainer actually produces instead of trusting what it passed in, and
    the trainer must keep honouring the override.

    Second, the budget being a delta instead of an absolute.  ``--max_epochs`` overwrites the yaml
    value and the agent resumes at the checkpoint's epoch, so 1,000 epochs of adaptation from
    ep1900 is 2900, not 1000; getting that wrong ends training instantly and silently.

    Third, a blanket provenance override creeping in.  Each arm is evaluated at the clip it TRAINED
    at, which is exactly the configuration that needs no override -- so NAVRL_V2_FORCE must be
    unreachable here, unlike in the sensor fidelity launcher where one narrow override was earned.

    Fourth, capture/crash/timeout leaking into the verdict.  They are measured under two different
    sensor definitions, so the preregistration reports them raw and judges on never-acquired alone.
    """

    ORCHESTRATOR = ROOT / "tools/run_navrl_ref5in_detection_range_stage1.py"
    PREREG = ROOT / "docs/prereg_2026-08-22_detection_range_2stage.md"
    TRAINER = RL / "train_navrl_v2_search.sh"

    @classmethod
    def launcher(cls):
        """Import the launcher once for behavioural assertions (no GPU, no training)."""
        module = getattr(cls, "_launcher_module", None)
        if module is None:
            spec = importlib.util.spec_from_file_location(
                "detection_range_stage1_launcher_under_test", cls.ORCHESTRATOR
            )
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            cls._launcher_module = module
        return module

    def source(self):
        return self.ORCHESTRATOR.read_text(encoding="utf-8")

    def test_preregistered_contract_literals_are_pinned(self):
        source = self.source()
        for literal in (
            "TRAIN_SEED = 457",
            "EVAL_SEED = 461",
            "BARS = 70",
            "EPISODES = 2049",
            "WARM_START_EPOCH = 1900",
            "ADAPT_EPOCHS = 1000",
            "NUM_ENVS = 128",
            "PPO_HORIZON = 32",
            'ARMS = (("clip20", 20.0), ("clip28", 28.0))',
            'TREATMENT_ARM = "clip28"',
            'CONTROL_ARM = "clip20"',
            "NEVER_ACQUIRED_HELPS_THRESHOLD_PP = -15.00",
            "DETECT_WIDTH = 1920",
            "DETECT_HEIGHT = 1200",
            "DETECTOR_MIN_PIXELS = 50",
            "CAMERA_WIDTH = 160",
            "CAMERA_HEIGHT = 90",
            "GOAL_DIST_MIN_M = 22.5",
            "GOAL_DIST_MAX_M = 28.0",
            '"navrl_ref5in_detection_range_stage1_s457"',
            "CHECKPOINT_SHA = "
            '"197ea26999d6bb9cf23c4e5a55acbe945f89985e2384687d60ab1dbae66a278e"',
            'PREREGISTRATION = "docs/prereg_2026-08-22_detection_range_2stage.md"',
            '"decision_authority": "none"',
            '"p2_verdict_changed": False',
            '"d1_verdict_changed": False',
            '"p3_unlocked": False',
            '"schema_version": 1,',
            'VERDICT_HELPS = "RANGE_HELPS"',
            'VERDICT_INCONCLUSIVE = "RANGE_INCONCLUSIVE_AT_THIS_BUDGET"',
            'VERDICT_VOID = "STAGE1_VOID"',
        ):
            self.assertIn(literal, source, f"missing preregistered literal: {literal}")

    def test_the_preregistration_it_names_exists_and_is_the_one_it_implements(self):
        self.assertTrue(self.PREREG.is_file(), "the named preregistration is missing")
        prereg = self.PREREG.read_text(encoding="utf-8")
        for literal in ("457", "461", "1,000 epoch", "4.096M", "−15.00 pp", "NAVRL_DETECTOR_MAX_RANGE"):
            self.assertIn(literal, prereg, f"preregistration does not contain {literal}")

    def test_arm_and_budget_constants_match_the_preregistration(self):
        module = self.launcher()
        self.assertEqual(module.ARMS, (("clip20", 20.0), ("clip28", 28.0)))
        self.assertEqual(module.TRAIN_SEED, 457)
        self.assertEqual(module.EVAL_SEED, 461)
        self.assertEqual(module.BARS, 70)
        self.assertEqual(module.EPISODES, 2049)
        self.assertEqual(module.ADAPT_EPOCHS, 1000)
        self.assertEqual(module.NEVER_ACQUIRED_HELPS_THRESHOLD_PP, -15.00)

    def test_the_budget_is_an_absolute_terminal_epoch_not_a_delta(self):
        """runner.py:850 overwrites the yaml max_epochs and the agent resumes at the checkpoint's
        epoch, so 1,000 adaptation epochs from ep1900 must be requested as 2900.  4.096 M samples
        is the same statement in frames: 128 envs x 32 horizon x 1,000 epochs."""
        module = self.launcher()
        self.assertEqual(module.TERMINAL_EPOCH, module.WARM_START_EPOCH + module.ADAPT_EPOCHS)
        self.assertEqual(module.TERMINAL_EPOCH, 2900)
        self.assertEqual(module.SAMPLES_PER_EPOCH, module.NUM_ENVS * module.PPO_HORIZON)
        self.assertEqual(module.ADAPT_SAMPLES, 4_096_000)
        self.assertEqual(module.TERMINAL_FRAME, module.TERMINAL_EPOCH * module.SAMPLES_PER_EPOCH)
        env = module.training_env("clip28", preflight=True)
        self.assertEqual(env["MAX_EPOCHS"], "2900")

    def test_threshold_is_a_constant_referenced_by_the_comparison(self):
        """A literal -15.0 inside the verdict would be a second copy that can drift from the
        preregistered value."""
        source = self.source()
        start = source.index("def classify_verdict(")
        verdict = source[start:source.index("# ------", start)]
        self.assertIn("delta_pp <= NEVER_ACQUIRED_HELPS_THRESHOLD_PP", verdict)
        for literal in ("15.0", "-15", "0.15"):
            self.assertNotIn(
                literal, verdict, f"verdict threshold {literal} duplicated inside the comparison"
            )

    def test_verdict_boundaries_are_exactly_the_preregistered_ones(self):
        module = self.launcher()
        for delta, expected in (
            (-15.00, "RANGE_HELPS"),
            (-15.01, "RANGE_HELPS"),
            (-37.65, "RANGE_HELPS"),
            (-14.99, "RANGE_INCONCLUSIVE_AT_THIS_BUDGET"),
            (0.0, "RANGE_INCONCLUSIVE_AT_THIS_BUDGET"),
            (+20.0, "RANGE_INCONCLUSIVE_AT_THIS_BUDGET"),
        ):
            with self.subTest(delta_pp=delta):
                self.assertEqual(module.classify_verdict(delta), expected)

    def test_capture_crash_and_timeout_cannot_enter_the_verdict(self):
        """Prereg section 5 reports them raw and excludes them from the decision.  The verdict
        function takes one number, so the exclusion is structural rather than a promise."""
        module = self.launcher()
        source = self.source()
        start = source.index("def classify_verdict(")
        verdict = source[start:source.index("# ------", start)]
        # The docstring is allowed to NAME the excluded quantities; the executable body is not.
        body = verdict[verdict.index('"""', verdict.index('"""') + 3) + 3:]
        for forbidden in ("capture", "crash", "timeout", "outcome_raw"):
            self.assertNotIn(
                forbidden, body, f"{forbidden} appears inside the verdict function body"
            )
        self.assertEqual(
            module.classify_verdict.__code__.co_argcount,
            1,
            "classify_verdict must take exactly the never-acquired delta",
        )
        # co_names is every global and attribute the function touches, so this closes the
        # "read it from a module global" route too.
        for name in module.classify_verdict.__code__.co_names:
            self.assertNotIn("capture", name.lower())
            self.assertNotIn("crash", name.lower())
            self.assertNotIn("timeout", name.lower())

    def test_the_manipulated_variable_is_per_arm_in_both_halves(self):
        module = self.launcher()
        for arm, clip in module.ARMS:
            with self.subTest(arm=arm):
                training = module.training_env(arm, preflight=True)
                evaluation = module.evaluation_env(arm, preflight=True)
                self.assertEqual(training["NAVRL_DETECTOR_MAX_RANGE"], f"{clip:.1f}")
                self.assertEqual(evaluation["NAVRL_DETECTOR_MAX_RANGE"], f"{clip:.1f}")

    def test_evaluation_environments_differ_in_exactly_the_manipulated_variable(self):
        module = self.launcher()
        diff = module.evaluation_env_diff()
        self.assertEqual(sorted(diff), ["NAVRL_DETECTOR_MAX_RANGE"])
        self.assertEqual(
            diff["NAVRL_DETECTOR_MAX_RANGE"], {"clip20": "20.0", "clip28": "28.0"}
        )

    def test_the_honest_sensor_is_identical_in_both_arms_and_both_halves(self):
        module = self.launcher()
        for arm, _ in module.ARMS:
            for label, env in (
                ("training", module.training_env(arm, preflight=True)),
                ("evaluation", module.evaluation_env(arm, preflight=True)),
            ):
                with self.subTest(arm=arm, half=label):
                    self.assertEqual(env["NAVRL_DETECT_WIDTH"], "1920")
                    self.assertEqual(env["NAVRL_DETECT_HEIGHT"], "1200")
                    self.assertEqual(env["NAVRL_DETECTOR_MIN_PIXELS"], "50")
                    self.assertEqual(env["NAVRL_CAMERA_WIDTH"], "160")
                    self.assertEqual(env["NAVRL_CAMERA_HEIGHT"], "90")
                    for key in module.ZERO_PERTURBATION_KEYS:
                        self.assertEqual(float(env[key]), 0.0)

    def test_min_pixels_override_lands_after_the_closed_evaluation_environment(self):
        """P2.canonical_env already contains NAVRL_DETECTOR_MIN_PIXELS=2.  Updating before that
        call is silently overwritten and both arms would evaluate at the dishonest threshold."""
        module = self.launcher()
        source = self.source()
        body = source[source.index("def evaluation_env("):source.index("def evaluation_env_diff(")]
        self.assertLess(
            body.index("P2.canonical_env(cell_dir(arm)"),
            body.index('"NAVRL_DETECTOR_MIN_PIXELS": str(DETECTOR_MIN_PIXELS)'),
            "the honest-sensor threshold must be applied AFTER P2.canonical_env",
        )
        self.assertLess(
            body.index("P2.canonical_env(cell_dir(arm)"),
            body.index('"PYTHONPATH": str(ROOT)'),
            "PYTHONPATH must be re-injected AFTER P2.canonical_env, which deletes it",
        )

    def test_the_canonical_trainer_honours_the_detection_threshold_as_an_override(self):
        """The launcher cannot set a condition the trainer hard-codes.  Reverting this line would
        train BOTH arms at the dishonest 2 px threshold while every log still looked normal."""
        module = self.launcher()
        trainer = self.TRAINER.read_text(encoding="utf-8")
        self.assertIn('export NAVRL_DETECTOR_MIN_PIXELS="${NAVRL_DETECTOR_MIN_PIXELS:-2}"', trainer)
        self.assertNotIn("export NAVRL_DETECTOR_MIN_PIXELS=2\n", trainer)
        for literal in module.TRAINER_OVERRIDABLE_LITERALS:
            self.assertIn(literal, trainer)

    def test_reflection_and_lateral_bias_are_zero_by_being_unset(self):
        """Prereg section 4 lists both as 0.  train_navrl_v2_search.sh UNSETS them and
        navrl_task.py reads an unset value as 0, so exporting a literal 0 would be erased anyway --
        absence is the assertion, and the launcher checks the trainer's own environment for it."""
        module = self.launcher()
        self.assertEqual(
            module.UNSET_MEANS_ZERO_KEYS, ("NAVRL_REFLECTION_COEF", "NAVRL_LATERAL_BIAS_COEF")
        )
        trainer = self.TRAINER.read_text(encoding="utf-8")
        self.assertIn("unset NAVRL_LATERAL_BIAS_COEF NAVRL_REFLECTION_COEF", trainer)
        for key in module.UNSET_MEANS_ZERO_KEYS:
            self.assertNotIn(key, module.training_env("clip20", preflight=True))
        # A synthetic "effective" environment: what the launcher passes in, plus the values the
        # canonical trainer adds.  It must verify clean, and adding a surviving coefficient must be
        # what breaks it -- otherwise the assertion is only catching the missing keys.
        effective = dict(module.training_env("clip20", preflight=True))
        effective.update(
            {
                "NAVRL_SEED": str(module.TRAIN_SEED),
                "NUM_ENVS": str(module.NUM_ENVS),
                "FILE": "ppo_navrl_perception_transformer.yaml",
                "TASK": "navrl_task",
                "AERIAL_GYM_SIM_NAME": "base_sim",
            }
        )
        module.verify_effective_training_env("clip20", effective)
        for key in module.UNSET_MEANS_ZERO_KEYS:
            with self.subTest(key=key):
                polluted = dict(effective)
                polluted[key] = "0.5"
                with self.assertRaises(module.ContractError):
                    module.verify_effective_training_env("clip20", polluted)

    def test_no_arm_may_ever_carry_a_provenance_override(self):
        """Unlike the sensor fidelity experiment, this one earns no override: each arm is evaluated
        at the clip it was trained at, and the evaluator's provenance gate has no range field."""
        module = self.launcher()
        source = self.source()
        for arm, _ in module.ARMS:
            self.assertNotIn("NAVRL_V2_FORCE", module.evaluation_env(arm, preflight=True))
        self.assertNotIn('env["NAVRL_V2_FORCE"] = "1"', source)
        self.assertNotIn('"NAVRL_V2_FORCE": "1"', source)
        self.assertIn('"NAVRL_V2_FORCE" not in env', source)
        proof = module.verify_evaluator_needs_no_range_override()
        self.assertIs(proof["override_required"], False)
        self.assertIs(proof["clip_not_recorded_in_checkpoint_provenance"], False)

    def test_a_refused_evaluator_preflight_stops_instead_of_forcing(self):
        module = self.launcher()

        class FakeCompleted:
            def __init__(self, returncode, stdout):
                self.returncode = returncode
                self.stdout = stdout

        passing = "[eval_v2] PREFLIGHT PASS (evaluation not started)"
        cases = {
            "a provenance mismatch": (
                2,
                "[eval_v2] REFUSING: v2 contract mismatch:\n"
                "  cfg_detector_min_pixels: checkpoint=2 expected=50.0",
            ),
            "a silent zero without the marker": (0, "[eval_v2] something else"),
            "a pass that still lists mismatches": (
                0,
                passing + "\n  cfg_max_tilt_deg: checkpoint=45.0 expected=50.0",
            ),
        }
        for label, (code, stdout) in cases.items():
            with self.subTest(case=label):
                original = module.run_eval_preflight
                module.run_eval_preflight = lambda arm, ckpt, _c=code, _o=stdout: FakeCompleted(
                    _c, _o
                )
                try:
                    with self.assertRaises(module.ContractError):
                        module.verify_no_override_needed("clip20", Path("/nonexistent.pth"))
                finally:
                    module.run_eval_preflight = original

    def test_gate_zero_covers_the_three_things_the_preregistration_demands(self):
        """Prereg section 5 Gate 0: max_epochs reached normally, no KL-driven rollback, terminal
        SHA recorded.  A failing arm is VOID, and VOID must be a verdict, not a footnote."""
        module = self.launcher()
        self.assertIn("T2_normal_max_epochs_exit", module.TRAINING_GATES)
        self.assertIn("T3_no_kl_rollback", module.TRAINING_GATES)
        self.assertIn("T4_terminal_sha_recorded", module.TRAINING_GATES)
        source = self.source()
        self.assertIn('summary.get("exit_reason") == MAX_EPOCHS_EXIT_REASON', source)
        self.assertIn("rollback_total == 0 and rollback_streak == 0", source)
        self.assertIn("ROLLBACK_LOG_MARKER", source)
        self.assertIn('"void_arms"', source)
        self.assertIn("VERDICT_VOID", source)
        self.assertIn("void_arms", module.SUMMARY_VERIFY_KEYS)

    def test_fail_closed_is_a_biconditional(self):
        source = self.source()
        self.assertIn(
            "(verdict == VERDICT_VOID) == (published is None)",
            source,
            "the fail-closed contract must be enforced in both directions",
        )

    def test_episode_contract_is_at_least_not_exactly(self):
        """The evaluator drains whole 128-env batches, so a cell finishes at or just past the
        request.  Asserting exact equality has already broken one arm of an earlier run."""
        source = self.source()
        self.assertIn("actual >= EPISODES", source)
        self.assertNotIn("actual == EPISODES", source)

    def test_train_refuses_to_overwrite_an_existing_arm(self):
        source = self.source()
        train_block = source[source.index("def train_arm("):source.index("def verify_training(")]
        self.assertIn("not train_dir(arm).exists()", train_block)
        self.assertIn("refusing overwrite", train_block)
        self.assertLess(
            train_block.index("not train_dir(arm).exists()"),
            train_block.index("tee_run("),
            "the refusal must precede any training",
        )

    def test_stage_two_is_not_implemented_here(self):
        """Prereg section 8 forbids running stage 2 on a negative stage 1.  Stage 2's seeds and
        budget must not exist in this file at all, so it cannot be started by editing one number."""
        source = self.source()
        for literal in ("10000", "10_000", "463", "467"):
            self.assertNotIn(literal, source, f"stage-2 constant {literal} leaked into stage 1")
        self.assertIn('"stage": 1', source)
        self.assertIn('"stage2_authorised"', source)

    def test_limitations_are_transcribed_from_the_preregistration(self):
        module = self.launcher()
        self.assertEqual(len(module.LIMITATIONS), 5)
        for index, item in enumerate(module.LIMITATIONS, start=1):
            self.assertTrue(
                item.startswith(f"L{index}:"), f"limitation {index} is not labelled L{index}"
            )

    def test_summary_authority_literals_stay_false(self):
        source = self.source()
        authority = source[
            source.index('"stage": 1,'):source.index('"limitations": list(LIMITATIONS)')
        ]
        for literal in (
            '"decision_authority": "none",',
            '"p2_verdict_changed": False,',
            '"d1_verdict_changed": False,',
            '"p3_unlocked": False,',
        ):
            self.assertIn(literal, authority, f"summary lost the authority literal {literal}")

    def test_the_script_chain_audit_declares_exactly_the_known_exceptions(self):
        """Reading what the launcher exports proves nothing: the chain runs afterwards and can
        ``export`` over a pinned variable or ``unset`` it.  The audit is mechanical so the next
        experiment does not depend on somebody repeating it by hand, and its exception table is
        pinned here so a NEW clobber cannot be waved through by adding a declaration."""
        module = self.launcher()
        audit = module.verify_pinned_variables_survive_the_script_chain()
        self.assertEqual(audit["variables_audited"], len(module.PINNED_TRAINING_VARIABLES))
        exceptions = sorted(
            name
            for name, notes in audit["findings"].items()
            if any("pass-through" not in note for note in notes)
        )
        self.assertEqual(
            exceptions,
            [
                "NAVRL_LATERAL_BIAS_COEF",
                "NAVRL_NUM_BARS",
                "NAVRL_PERCEPTION_PERTURB",
                "NAVRL_REFLECTION_COEF",
            ],
            "the script chain now touches a pinned variable that is not declared",
        )
        self.assertEqual(module.CHAIN_CLOBBERS_TO_PINNED_VALUE, {"NAVRL_PERCEPTION_PERTURB": "0"})
        for variable in ("NAVRL_DETECTOR_MAX_RANGE", "NAVRL_DETECTOR_MIN_PIXELS",
                         "NAVRL_DETECT_WIDTH", "NAVRL_CAMERA_WIDTH"):
            with self.subTest(variable=variable):
                self.assertIn(variable, module.PINNED_TRAINING_VARIABLES)
                self.assertNotIn(variable, exceptions)

    def test_density_curriculum_is_pinned_off_so_bar_count_cannot_become_a_second_axis(self):
        """train_navrl_v2_search.sh unsets NAVRL_NUM_BARS while the density curriculum owns it.
        clip28 acquires more easily, so it would promote sooner and the arms would have differed in
        DENSITY as well as clip -- two axes, with both runs looking perfectly healthy."""
        module = self.launcher()
        for arm, _ in module.ARMS:
            env = module.training_env(arm, preflight=True)
            self.assertEqual(env["NAVRL_DENSITY_CURRICULUM"], "0")
            self.assertEqual(env["NAVRL_NUM_BARS"], str(module.BARS))
            self.assertEqual(env["NAVRL_DENSITY_START"], str(module.BARS))
            self.assertEqual(env["NAVRL_DENSITY_FINAL"], str(module.BARS))
        # The effective-environment check is the behavioural half and must assert both.
        effective = dict(module.training_env("clip20", preflight=True))
        effective.update(
            {
                "NAVRL_SEED": str(module.TRAIN_SEED),
                "NUM_ENVS": str(module.NUM_ENVS),
                "FILE": "ppo_navrl_perception_transformer.yaml",
                "TASK": "navrl_task",
                "AERIAL_GYM_SIM_NAME": "base_sim",
            }
        )
        module.verify_effective_training_env("clip20", effective)
        for key, bad in (("NAVRL_DENSITY_CURRICULUM", "1"), ("NAVRL_NUM_BARS", "205")):
            with self.subTest(key=key):
                polluted = dict(effective)
                polluted[key] = bad
                with self.assertRaises(module.ContractError):
                    module.verify_effective_training_env("clip20", polluted)

    def test_a_gate_zero_failure_is_reported_as_VOID_not_raised(self):
        """Prereg section 5: a failing arm is VOID.  A launcher that raised would report an
        exception instead of the outcome, and the run would look like a tooling error."""
        module = self.launcher()
        good = {key: {"checked_by_launcher": True, "passed": True}
                for key in module.TRAINING_GATES.values()}
        self.assertTrue(module.training_gates_passed(good))
        broken = {key: dict(value) for key, value in good.items()}
        broken["exit"] = {
            "checked_by_launcher": True,
            "passed": False,
            "exit_reason": "early_stop_collapse",
        }
        self.assertFalse(module.training_gates_passed(broken))
        verified = {
            "trainings": {"clip20": good, "clip28": broken},
            "cells": {},
            "order": ("clip20", "clip28"),
            "void_arms": ["clip28"],
            "held_fixed": {},
            "runtime_map_identity": None,
            "shared_training_receipt": None,
            "single_axis": None,
        }
        payload = module.build_summary(verified)
        self.assertEqual(payload["verdict"], "STAGE1_VOID")
        self.assertEqual(payload["void_arms"], ["clip28"])
        self.assertIsNone(payload["arms"])
        self.assertIsNone(payload["never_acquired_delta_pp"])
        self.assertIn("T2_normal_max_epochs_exit", payload["failed_gates"])
        self.assertFalse(payload["stage2_authorised"])
        # The evaluation gates were never run on a VOID arm, and "not judged" must not be
        # published as "judged and passed".
        for gate in module.PER_ARM_GATES:
            self.assertNotIn(gate, payload["quality_gates"])

    def test_evaluate_refuses_to_spend_gpu_time_on_a_void_arm(self):
        source = self.source()
        block = source[source.index("def evaluate_arm("):source.index("# ------", source.index("def evaluate_arm("))]
        self.assertIn("training_gates_passed(trained)", block)
        self.assertLess(
            block.index("training_gates_passed(trained)"),
            block.index("tee_run("),
            "the VOID refusal must precede the 2,049-episode evaluation",
        )

    def test_an_externally_trained_run_can_be_adopted_for_evaluation(self):
        """`train` is not the only legal way to produce an arm -- the trainer can be driven
        directly -- and the verification half must not be unusable just because this launcher did
        not press the button.  The adoption path names the run explicitly rather than guessing."""
        module = self.launcher()
        source = self.source()
        self.assertEqual(module.ADOPT_RUN_ROOT_ENV % "CLIP20", "DETRANGE_STAGE1_RUN_ROOT_CLIP20")
        self.assertEqual(module.ADOPT_TRAIN_LOG_ENV % "CLIP28", "DETRANGE_STAGE1_TRAIN_LOG_CLIP28")
        block = source[source.index("def evaluate_arm("):source.index("# ------", source.index("def evaluate_arm("))]
        self.assertIn("adopt_training_run(arm)", block)
        # Both variables are required: the log is Gate 0's second, independent rollback witness.
        for present in ({}, {"DETRANGE_STAGE1_RUN_ROOT_CLIP20": "/nonexistent"}):
            with self.subTest(env=sorted(present)):
                saved = {k: os.environ.get(k) for k in
                         ("DETRANGE_STAGE1_RUN_ROOT_CLIP20", "DETRANGE_STAGE1_TRAIN_LOG_CLIP20")}
                for key in saved:
                    os.environ.pop(key, None)
                os.environ.update(present)
                try:
                    with self.assertRaises(module.ContractError):
                        module.adopt_training_run("clip20")
                finally:
                    for key, value in saved.items():
                        os.environ.pop(key, None)
                        if value is not None:
                            os.environ[key] = value

    def test_an_adopted_run_must_claim_only_checkpoint_attested_geometry(self):
        """An operator-selected folder is not evidence of its sensor contract; env_state is."""
        module = self.launcher()
        self.assertNotIn("operator_assertion", module.CLIP_EVIDENCE_ADOPTED)
        self.assertIn("checkpoint_env_state", module.CLIP_EVIDENCE_ADOPTED)
        self.assertNotIn("operator_assertion", module.CLIP_EVIDENCE_LAUNCHER)
        source = self.source()
        for key in ("cfg_detector_max_range", "cfg_detect_width", "cfg_detect_height"):
            self.assertIn(key, source)
        self.assertIn('"detector_max_range_evidence"', self.source())


if __name__ == "__main__":
    unittest.main()
