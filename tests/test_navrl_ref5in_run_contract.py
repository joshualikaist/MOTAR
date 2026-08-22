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


if __name__ == "__main__":
    unittest.main()
