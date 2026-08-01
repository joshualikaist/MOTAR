import importlib.util
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest import mock

import torch
from torch.utils.tensorboard import SummaryWriter


_MODULE_PATH = Path(__file__).resolve().parents[1] / "tools/update_status_snapshot.py"
_SPEC = importlib.util.spec_from_file_location("update_status_snapshot", _MODULE_PATH)
_STATUS = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_STATUS)

_ATTESTATION_MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "tools/navrl_v2_recovery_attestation.py"
)
_ATTESTATION_SPEC = importlib.util.spec_from_file_location(
    "navrl_v2_recovery_attestation_for_status_test", _ATTESTATION_MODULE_PATH
)
_ATTESTATION = importlib.util.module_from_spec(_ATTESTATION_SPEC)
_ATTESTATION_SPEC.loader.exec_module(_ATTESTATION)


class StatusSnapshotTest(unittest.TestCase):
    @staticmethod
    def _write_json(path, payload):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

    @staticmethod
    def _digest(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    @staticmethod
    def _write_training_audit(run_dir):
        writer = SummaryWriter(log_dir=str(run_dir / "summaries"))
        for step in range(9501, 9601):
            writer.add_scalar("ppo/behavior_kl_audit_max", 0.01, step)
            writer.add_scalar("ppo/epoch_rollback", 0.0, step)
            writer.add_scalar("ppo/epoch_rollback_total", 0.0, step)
            writer.add_scalar("ppo/epoch_rollback_streak", 0.0, step)
            for axis in ("x", "y", "z", "yaw"):
                writer.add_scalar(f"policy_action/raw_oob_{axis}", 0.0, step)
        writer.close()

    def _make_recovery_artifacts(self, root, *, canonical=True):
        runs_root = root / "runs"
        run_name = "ppo_260801_1200_navrl_v2-recover-smoke-130bars-s1"
        run_dir = runs_root / run_name
        checkpoint_path = run_dir / "nn/last_gen_ppo_ep_9600_rew_1.pth"
        checkpoint_path.parent.mkdir(parents=True)

        env_state = dict(_STATUS._RECOVERY_CHECKPOINT_CONTRACT)
        env_state.update(
            {
                "cfg_recovery_stage": "smoke",
                "cfg_recovery_source_sha256": _STATUS._RECOVERY_SOURCE_SHA256,
                "cfg_recovery_source_epoch": 9500,
                "cfg_recovery_smoke_required_epochs": 100,
                "cfg_recovery_smoke_bars": 130,
                "n_bars_active": 130,
            }
        )
        torch.save(
            {"epoch": 9600, "frame": 39321600, "env_state": env_state},
            checkpoint_path,
        )
        (run_dir / ".aerial_training_finished").write_text(
            "epoch=9600\n", encoding="utf-8"
        )

        actual_episodes = 2176
        captured, crash, timeout = 1500, 600, 76
        result_path = root / "eval/130bars.json"
        result_path.parent.mkdir(parents=True, exist_ok=True)
        checkpoint_snapshot = result_path.parent / "checkpoint_snapshot.pth"
        shutil.copyfile(checkpoint_path, checkpoint_snapshot)
        receipt_path = result_path.with_suffix(".receipt.json")
        log_path = result_path.with_suffix(".log")
        log_path.write_text("synthetic evaluator log\n", encoding="utf-8")
        nonce = "b" * 64
        evaluator_path = _ATTESTATION.EVALUATOR_SCRIPT.resolve()
        result = {
            "schema_version": 1,
            "requested_episodes": 2049,
            "actual_episodes": actual_episodes,
            "checkpoint": str(checkpoint_path.resolve()),
            "condition": {
                "seed": 42,
                "bars": 130,
                "target_pattern": "mixed",
                "target_speed_mode": "uniform",
                "target_speed_mps": None,
                "target_speed_min_mps": 0.3,
                "target_speed_max_mps": 1.5,
                "pursuer_max_speed_mps": 2.5,
                "oob_margin_m": 1.0,
                "episode_len_steps": 600,
                "num_envs": 128,
                "goal_dist_min_m": 6.0,
                "goal_dist_max_m": 28.0,
                "full_goal_distribution": True,
                "fov_curriculum_saturated": True,
                "evaluation_nonce": nonce,
                "runtime_sim_config_class": "BaseSimConfig",
                "physics_dt_s": 0.01,
                "physics_substeps": 1,
                "physics_steps_per_rl_step": 10,
                "rl_step_dt_s": 0.1,
            },
            "outcome": {
                "captured": captured,
                "crash": crash,
                "timeout": timeout,
                "capture_rate": captured / actual_episodes,
                "crash_rate": crash / actual_episodes,
                "timeout_rate": timeout / actual_episodes,
            },
            "action": {
                "policy": "squashed_gaussian",
                "samples": 25000,
                "task_input_oob_rate": [0.0, 0.0, 0.0, 0.0],
            },
            "v2_evaluation_contract": dict(_STATUS._RECOVERY_RESULT_CONTRACT),
            "checkpoint_sha256": self._digest(checkpoint_path),
            "evaluated_checkpoint_snapshot": str(checkpoint_snapshot.resolve()),
            "evaluated_checkpoint_snapshot_sha256": self._digest(
                checkpoint_snapshot
            ),
            "evaluator_script": str(evaluator_path),
            "evaluator_script_sha256": self._digest(evaluator_path),
            "evaluation_receipt": str(receipt_path.resolve()),
        }
        self._write_json(result_path, result)
        receipt = {
            "schema_version": 1,
            "producer": "eval_navrl_v2_density_sweep.sh",
            "started_at_utc": "2026-08-01T00:00:00+00:00",
            "completed_at_utc": "2026-08-01T00:01:00+00:00",
            "evaluation_nonce": nonce,
            "source_checkpoint": str(checkpoint_path.resolve()),
            "source_checkpoint_sha256": self._digest(checkpoint_path),
            "evaluated_checkpoint_snapshot": str(checkpoint_snapshot.resolve()),
            "evaluated_checkpoint_snapshot_sha256": self._digest(
                checkpoint_snapshot
            ),
            "result_json": str(result_path.resolve()),
            "result_sha256": self._digest(result_path),
            "log_file": str(log_path.resolve()),
            "log_sha256": self._digest(log_path),
            "evaluator_script": str(evaluator_path),
            "evaluator_script_sha256": self._digest(evaluator_path),
            "bars": 130,
            "seed": 42,
            "requested_episodes": 2049,
            "actual_episodes": actual_episodes,
        }
        self._write_json(receipt_path, receipt)

        attestation_path = run_dir / ".navrl_v2_recovery_eval_pass.json"
        if canonical:
            self._write_training_audit(run_dir)
            attestation_path = _ATTESTATION.create_attestation(
                checkpoint_path, result_path
            )
        else:
            # A plausible, hand-written PASS must not substitute for the missing 100-epoch
            # TensorBoard evidence. This mirrors the historical dashboard bypass exactly.
            attestation = {
                "schema_version": 1,
                "verdict": "PASS",
                "created_at_utc": "2026-08-01T00:00:00+00:00",
                "checkpoint": str(checkpoint_path.resolve()),
                "checkpoint_sha256": self._digest(checkpoint_path),
                "checkpoint_epoch": 9600,
                "checkpoint_frame": 39321600,
                "source_checkpoint_sha256": _STATUS._RECOVERY_SOURCE_SHA256,
                "source_epoch": 9500,
                "smoke_epochs": 100,
                "bars": 130,
                "seed": 42,
                "requested_episodes": 2049,
                "episodes": actual_episodes,
                "captured": captured,
                "crash": crash,
                "timeout": timeout,
                "capture_rate": captured / actual_episodes,
                "crash_rate": crash / actual_episodes,
                "timeout_rate": timeout / actual_episodes,
                "training_max_kl": 0.01,
                "training_max_task_input_oob_rate": 0.0,
                "evaluation_max_task_input_oob_rate": 0.0,
                "max_task_input_oob_rate": 0.0,
                "max_rollback_streak": 0.0,
                "final_rollback_streak": 0.0,
                "heldout_result_json": str(result_path.resolve()),
                "heldout_result_sha256": self._digest(result_path),
                "thresholds": dict(_STATUS._RECOVERY_ATTESTATION_THRESHOLDS),
                "evaluation_contract": dict(_STATUS._RECOVERY_RESULT_CONTRACT),
            }
            self._write_json(attestation_path, attestation)
        return {
            "runs_root": runs_root,
            "run_name": run_name,
            "run_dir": run_dir,
            "checkpoint_path": checkpoint_path,
            "result_path": result_path,
            "attestation_path": attestation_path,
        }

    def test_short_transaction_runs_do_not_replace_latest(self):
        self.assertTrue(
            _STATUS._is_smoke_run(
                "ppo_260801_0528_navrl_v2-full-transaction-forced-final-s1"
            )
        )
        self.assertTrue(
            _STATUS._is_smoke_run(
                "ppo_260801_0455_navrl_v2-central-transaction-integration-smoke-s1"
            )
        )

    def test_real_recovery_smoke_remains_a_research_stage(self):
        self.assertFalse(
            _STATUS._is_smoke_run("ppo_260801_1200_navrl_v2-recover-smoke-130bars-s1")
        )

    def test_ttc_run_is_rendered_as_fixed_density_ablation(self):
        update = _STATUS._v2_search_update(
            {
                "run": "ppo_260801_1200_navrl_v2-ttc-ttc-s1",
                "last_epoch": 5250,
                "last_n_bars_active": 70,
                "last_captured_rate": 0.80,
                "last_crash_rate": 0.20,
            },
            is_live=False,
        )
        experiment = update["active_experiment"]
        self.assertEqual(experiment["selector"], "ttc_sector")
        self.assertEqual(experiment["bars"], 70)
        self.assertFalse(experiment["density_curriculum"])
        self.assertIn("selector A/B", update["subtitle"])

    def test_recovery_attestation_requires_real_matching_artifacts(self):
        with tempfile.TemporaryDirectory() as tmp:
            artifacts = self._make_recovery_artifacts(Path(tmp))
            self.assertEqual(
                _STATUS._validate_recovery_attestation(artifacts["run_dir"]), []
            )
            with mock.patch.object(_STATUS, "RUNS_ROOT", artifacts["runs_root"]):
                update = _STATUS._v2_search_update(
                    {
                        "run": artifacts["run_name"],
                        "last_epoch": 9600,
                        "last_n_bars_active": 130,
                    },
                    is_live=False,
                )
            self.assertTrue(update["active_experiment"]["recovery_attestation_valid"])
            self.assertIn("passed this snapshot's verification", update["headline"])
            gate = next(
                item
                for item in update["gates"]
                if item["label"] == "held-out 130-bar attestation"
            )
            self.assertEqual(gate["value"], "PASS")

    def test_recovery_attestation_rejects_handwritten_pass_without_summaries(self):
        with tempfile.TemporaryDirectory() as tmp:
            artifacts = self._make_recovery_artifacts(
                Path(tmp), canonical=False
            )
            errors = _STATUS._validate_recovery_attestation(artifacts["run_dir"])
            self.assertTrue(errors)
            self.assertTrue(
                any(error.startswith("canonical verifier:") for error in errors),
                errors,
            )
            with mock.patch.object(_STATUS, "RUNS_ROOT", artifacts["runs_root"]):
                update = _STATUS._v2_search_update(
                    {
                        "run": artifacts["run_name"],
                        "last_epoch": 9600,
                        "last_n_bars_active": 130,
                    },
                    is_live=False,
                )
            self.assertFalse(
                update["active_experiment"]["recovery_attestation_valid"]
            )
            self.assertIn("curriculum remains blocked", update["headline"])

    def test_recovery_attestation_rejects_mutated_artifact_contract_and_rollback(self):
        mutations = ("result", "checkpoint", "rollback")
        for mutation in mutations:
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as tmp:
                artifacts = self._make_recovery_artifacts(Path(tmp))
                attestation = json.loads(
                    artifacts["attestation_path"].read_text(encoding="utf-8")
                )
                if mutation == "result":
                    result = json.loads(
                        artifacts["result_path"].read_text(encoding="utf-8")
                    )
                    result["condition"]["seed"] = 1
                    self._write_json(artifacts["result_path"], result)
                    # Even if a tamperer updates the quoted digest, the canonical seed contract fails.
                    attestation["heldout_result_sha256"] = self._digest(
                        artifacts["result_path"]
                    )
                elif mutation == "checkpoint":
                    checkpoint = torch.load(
                        artifacts["checkpoint_path"],
                        map_location="cpu",
                        weights_only=False,
                    )
                    checkpoint["env_state"]["cfg_oob_margin"] = 0.5
                    torch.save(checkpoint, artifacts["checkpoint_path"])
                    # A rewritten attestation digest must not hide changed checkpoint semantics.
                    attestation["checkpoint_sha256"] = self._digest(
                        artifacts["checkpoint_path"]
                    )
                else:
                    attestation["max_rollback_streak"] = 1.0
                self._write_json(artifacts["attestation_path"], attestation)

                errors = _STATUS._validate_recovery_attestation(artifacts["run_dir"])
                self.assertTrue(errors)
                with mock.patch.object(_STATUS, "RUNS_ROOT", artifacts["runs_root"]):
                    update = _STATUS._v2_search_update(
                        {
                            "run": artifacts["run_name"],
                            "last_epoch": 9600,
                            "last_n_bars_active": 130,
                        },
                        is_live=False,
                    )
                self.assertFalse(
                    update["active_experiment"]["recovery_attestation_valid"]
                )
                self.assertIn("curriculum remains blocked", update["headline"])


if __name__ == "__main__":
    unittest.main()
