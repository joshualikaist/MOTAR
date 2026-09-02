import os
import subprocess
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = (
    ROOT / "aerial_gym/rl_training/rl_games/eval_navrl_corrected_nonoverlap_physical_off_heldout.sh"
)
EVALUATOR = ROOT / "aerial_gym/rl_training/rl_games/eval_navrl_v2_density_sweep.sh"
PREREG = ROOT / "docs/preregistration_corrected_nonoverlap_physical_off_heldout_eval_2026-09-02.md"


class CorrectedNonoverlapHeldoutContractTest(unittest.TestCase):
    def test_preflight_pins_seed313_trained_densities_and_last_gen(self):
        env = dict(os.environ)
        env["CORRECTED_NONOVERLAP_HELDOUT_PREFLIGHT_ONLY"] = "1"
        result = subprocess.run(
            [str(LAUNCHER)],
            cwd=ROOT,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("seed=313", result.stdout)
        self.assertIn("ckpt=last_gen_ppo_ep_21750", result.stdout)
        self.assertIn("densities=70 85 100 115 130 145", result.stdout)
        self.assertIn("episodes=2049", result.stdout)
        self.assertIn("speed_final=1.25 ramp=1", result.stdout)
        self.assertIn("ood_205=0", result.stdout)
        self.assertIn("gen_ppo=forbidden", result.stdout)
        self.assertNotIn("150 210 280", result.stdout)

    def test_cli_is_rejected(self):
        env = dict(os.environ)
        env["CORRECTED_NONOVERLAP_HELDOUT_PREFLIGHT_ONLY"] = "1"
        result = subprocess.run(
            [str(LAUNCHER), "extra"],
            cwd=ROOT,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        self.assertNotEqual(result.returncode, 0, result.stdout)

    def test_wrapper_and_evaluator_close_the_wrong_default_contract(self):
        wrapper = LAUNCHER.read_text(encoding="utf-8")
        evaluator = EVALUATOR.read_text(encoding="utf-8")
        prereg = PREREG.read_text(encoding="utf-8")
        for literal in (
            "541b36bdcabacf8bb14c6fbb0ad07054dd9735ad24777a3222655ba8ca9c8132",
            "last_gen_ppo_ep_21750_rew_83.1572.pth",
            "NAVRL_SEED=\"${SEED}\"",
            "SEED=313",
            'DENSITIES="70 85 100 115 130 145"',
            "NAVRL_V2_EVAL_CONTRACT=corrected_nonoverlap_physical_off",
            "gen_ppo.pth is forbidden",
        ):
            self.assertIn(literal, wrapper)
        self.assertIn("NAVRL_V2_EVAL_CONTRACT=corrected_nonoverlap_physical_off", evaluator)
        self.assertIn("70 85 100 115 130 145", evaluator)
        self.assertIn("navrl_ref5in_v2_quad", evaluator)
        self.assertIn("NAVRL_PHYSICAL_GEOMETRY_VERSION=v2", evaluator)
        self.assertIn("physx_ref5in_6dof_motor_wrench_v2_same_substep", evaluator)
        self.assertIn('float(os.environ["NAVRL_TARGET_SPEED_FINAL"])', evaluator)
        self.assertNotIn(
            '"target_speed_max_mps": fixed_speed if fixed_speed is not None else 1.5',
            evaluator,
        )
        self.assertIn("untrained OOD", evaluator)
        self.assertIn("untrained OOD and is not this evaluation", prereg)
        self.assertIn("313", prereg)
        self.assertNotIn("70 85 100 115 130 145 205", wrapper)


if __name__ == "__main__":
    unittest.main()
