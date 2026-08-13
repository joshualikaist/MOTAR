"""CPU-only guards for the ref5in closed-run and provenance contracts."""

from pathlib import Path
import os
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
            '"path_length_support"',
            '"chirality_sensitive"',
        ):
            self.assertIn(literal, source)

        task = (ROOT / "aerial_gym/task/navrl_task/navrl_task.py").read_text(
            encoding="utf-8"
        )
        for literal in (
            "NAVRL_EVAL_CV_INITIAL_HEADING is evaluation-only",
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


if __name__ == "__main__":
    unittest.main()
