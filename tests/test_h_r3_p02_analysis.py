"""Synthetic six-arm P02 traces: scoring, integrity, exposure, safety and seed inference."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import h_r3_p02_analysis as analysis


def fixture_trace(n=3, steps=60):
    zeros = lambda dtype=bool: np.zeros((steps, n), dtype=dtype)
    tr = {key: zeros() for key in ("camera", "reset", "crashed", "captured", "timeout", "intervening", "shadow_clear")}
    for key in ("el0_cls", "el0_visible_px", "contact_substeps"):
        tr[key] = zeros(np.int64)
    for key in ("raw_nearest_m", "contact_peak_n", "shadow_offset", "sham_sign", "governor_clearance_m",
                "governor_requested_speed_mps", "governor_executed_speed_mps", "governor_scale"):
        tr[key] = np.ones((steps, n), np.float32)
    for key in ("actual_pos", "before_pos", "actual_vel", "target_pos"):
        tr[key] = np.zeros((steps, n, 3), np.float32)
    tr["actual_pos"][:, :, 0] = np.arange(1, steps + 1)[:, None] * .1
    tr["before_pos"][:, :, 0] = np.arange(steps)[:, None] * .1
    tr["actual_vel"][:, :, 0] = 1
    tr["target_pos"][:, :, 0] = 6
    for key in ("action", "command", "rng_probe"):
        tr[key] = np.zeros((steps, n, 4), np.float32)
    tr["command"][:, :, 0] = 1
    tr["action"][:, :, 0] = .4
    for key in ("nominal_xy", "post_point_xy", "post_cap_xy"):
        tr[key] = np.zeros((steps, n, 2), np.float32)
        tr[key][:, :, 0] = 1
        for arm in ("B2", "B3", "B4", "B5"):
            tr["shadow_%s_%s" % (arm, key)] = tr[key].copy()
    tr["budget_error"] = np.zeros((steps, n, 2), np.float32)
    for key in ("digest", "sensor_digest"):
        tr[key] = np.arange(steps * n * 32, dtype=np.uint8).reshape(steps, n, 32)
        tr["initial_" + key] = np.zeros((n, 32), np.uint8)
    for key in analysis.IDENTITY:
        tr[key] = zeros() if key in ("id_camera_identity_correct", "id_camera_identity_correct_025",
                                    "id_camera_identity_correct_075", "id_centroid_near_visible", "id_camera_delivered") else zeros(np.float32)
    tr["id_detector_bbox"] = np.zeros((steps, n, 4), np.int64)
    tr["id_reported_centroid_uv"] = np.zeros((steps, n, 2), np.float32)
    tr["t0"] = np.full(n, -1, np.int64)
    tr["t0"][:2] = 10
    tr["intervening"][10:30, :2] = True
    tr["camera"][:9] = True
    tr["camera"][:, 2:] = True
    tr["id_camera_delivered"] = tr["camera"].copy()
    tr["el0_cls"][:] = 7
    tr["el0_cls"][9, 0], tr["el0_cls"][9, 1] = 4, 2
    tr["el0_visible_px"][:] = 1
    tr["controller_ms"] = np.ones(steps, np.float32)
    return tr


def detect(tr, step, env=0, spatial=True):
    tr["camera"][step, env] = tr["id_camera_delivered"][step, env] = True
    tr["el0_visible_px"][step, env] = 1
    for key in ("id_camera_identity_correct", "id_camera_identity_correct_025", "id_camera_identity_correct_075"):
        tr[key][step, env] = spatial


def write_process(root, name, tr, seed, arm, identity=True, forced=False):
    path = Path(root) / name
    path.mkdir()
    np.savez(path / "h_r3_p02_trace.npz", **tr)
    meta = {"arm": arm, "seed": seed, "steps": len(tr["camera"]), "num_envs": len(tr["t0"]),
            "window_steps": 20, "step_dt": .1, "nan_fields": {}, "model_training_flag": False,
            "vision_mode": True, "identity_enabled": identity, "forced_reset": forced, "sensor_lineage": "off",
            "rms_digest_start": "frozen", "rms_digest_end": "frozen"}
    (path / "h_r3_p02_meta.json").write_text(json.dumps(meta))
    files = [{"file": file.name, "bytes": file.stat().st_size, "sha256": analysis.sha(file)}
             for file in (path / "h_r3_p02_trace.npz", path / "h_r3_p02_meta.json")]
    (path / "process_done.json").write_text(json.dumps({"process": name, "actor_obs_dim": 898, "nan_fields": {}, "files": files}))


class AnalysisTest(unittest.TestCase):
    def run_fixture(self, edit=None, smoke=False, extra_edit=None):
        plan = copy.deepcopy(analysis.PLAN)
        n, steps = (16, 60) if smoke else (3, 60)
        plan["run"].update(num_envs=n, steps=steps)
        plan["run"]["smoke"].update(num_envs=n, steps=steps)
        seeds = [plan["run"]["smoke"]["seed"]] if smoke else plan["run"]["seeds"]
        with tempfile.TemporaryDirectory() as root:
            names = []
            baseline = None
            for seed in seeds:
                for arm in analysis.ARMS:
                    tr = fixture_trace(n, steps)
                    if arm in ("B0", "B3", "B4"):
                        detect(tr, 12)
                    detect(tr, 11, env=1)
                    if edit:
                        edit(seed, arm, tr)
                    name = "seed%d_%s" % (seed, arm)
                    write_process(root, name, tr, seed, arm)
                    names.append(name)
                    if arm == "B0":
                        baseline = copy.deepcopy(tr)
            if smoke:
                for suffix in ("forced", "noeval"):
                    tr = copy.deepcopy(baseline)
                    if suffix == "forced":
                        tr["reset"][40, 15] = True
                        tr["digest"][40:, 15] ^= np.uint8(1)
                        tr["sensor_digest"][40:, 15] ^= np.uint8(1)
                        tr["rng_probe"][40:, 15] = 2
                    if extra_edit:
                        extra_edit(suffix, tr)
                    name = "seed%d_B0_%s" % (seeds[0], suffix)
                    write_process(root, name, tr, seeds[0], "B0", identity=suffix != "noeval", forced=suffix == "forced")
                    names.append(name)
            report = {"status": "COMPLETED", "stop_reason": None, "source_commit": "committed-source",
                      "plan_sha256": analysis.sha(analysis.PLAN_PATH),
                      "preflight": {"hr3_input_sha256": {"tools/h_r3_p02_analysis.py": analysis.sha(analysis.__file__)}},
                      "processes": [{"id": name, "returncode": 0} for name in names]}
            (Path(root) / "run_report.json").write_text(json.dumps(report))
            with patch.object(analysis, "PLAN", plan):
                return analysis.analyse(root, smoke=smoke)

    def test_primary_denominator_zero_classes_and_two_seed_summary(self):
        result = self.run_fixture()
        self.assertEqual(result["integrity"]["status"], "PASS", result["integrity"])
        self.assertEqual(result["counts"]["expected_processes"], 12)
        self.assertEqual(result["counts"]["primary_events"], 2)
        self.assertEqual(result["counts"]["by_class"]["PARTIAL"], 0)
        self.assertEqual(result["separate_classes"]["FOV_EXIT"]["n"], 2)
        self.assertEqual(result["primary"]["equal_seed"]["contrasts"]["B3-B2"]["mean_rate_diff"], 1.)
        pair = result["primary"]["conditional_descriptive"]["contrasts"]["B3-B2"]
        self.assertEqual((pair["n10"], pair["n01"]), (2, 0))
        self.assertNotIn("pvalue", pair)
        self.assertEqual(result["primary"]["equal_seed"]["two_seed_minimum_two_sided_signflip_p"], .5)

    def test_camera_and_any_target_pixel_does_not_establish_spatial_identity(self):
        def edit(seed, arm, tr):
            if arm == "B3":
                detect(tr, 12, spatial=False)
        result = self.run_fixture(edit)
        arm = result["primary"]["conditional_descriptive"]["arms"]["B3"]
        self.assertEqual(arm["reobserved"], 0)
        self.assertEqual(arm["variants"]["p01_terminal_corrected"]["reobserved"], 2)

    def test_terminal_first_scoring_and_no_new_episode_metrics(self):
        def edit(seed, arm, tr):
            if arm == "B3":
                tr["crashed"][12, 0] = tr["reset"][12, 0] = True
                tr["intervening"][13:30, 0] = False
                tr["captured"][40, 0] = True
        result = self.run_fixture(edit)
        event = result["events"][0]["results"]["B3"]
        self.assertFalse(event["reobserved"])
        self.assertTrue(event["variants"]["p01_literal"]["reobserved"])
        self.assertFalse(event["variants"]["p01_terminal_corrected"]["reobserved"])
        self.assertFalse(event["capture_by_w_plus_3s"])
        self.assertEqual(event["control_only"]["steps"], 3)
        self.assertAlmostEqual(event["control_only"]["translation_m"], .3, places=5)
        self.assertEqual(result["counts"]["primary_events"], 2)

    def test_prefix_initial_and_no_trigger_coupling_fail_without_cohort_discard(self):
        edits = (lambda tr: tr["digest"].__setitem__((5, 0, 0), 11),
                 lambda tr: tr["initial_sensor_digest"].__setitem__((0, 0), 11),
                 lambda tr: tr["sensor_digest"].__setitem__((45, 2, 0), 11),
                 lambda tr: tr["t0"].__setitem__(0, 11))
        for change in edits:
            def edit(seed, arm, tr):
                if arm == "B5":
                    change(tr)
            result = self.run_fixture(edit)
            self.assertEqual(result["integrity"]["status"], "FAIL")
            self.assertEqual(result["counts"]["reference_trigger_events"], 4)
            self.assertFalse(result["primary"]["integrity_valid"])

    def test_fixed20_does_not_end_when_camera_returns(self):
        result = self.run_fixture()
        self.assertEqual(result["events"][0]["results"]["B3"]["control_only"]["steps"], 20)
        def edit(seed, arm, tr):
            if arm == "B3":
                tr["intervening"][13:30, 0] = False
        result = self.run_fixture(edit)
        self.assertEqual(result["integrity"]["status"], "FAIL")
        self.assertTrue(any("fixed20" in reason for reason in result["integrity"]["failures"]))

    def test_seed_effects_remain_separate(self):
        def edit(seed, arm, tr):
            if seed == 4602 and arm == "B3":
                tr["id_camera_identity_correct"][12, 0] = False
        result = self.run_fixture(edit)
        equal = result["primary"]["equal_seed"]["contrasts"]["B3-B2"]
        self.assertEqual(equal["mean_rate_diff"], .5)
        self.assertEqual(equal["observed_seed_range"], [0., 1.])

    def test_preloss_identity_diagnostic_does_not_select_primary(self):
        def edit(seed, arm, tr):
            tr["id_camera_identity_correct"][8, 0] = seed == 4601
        result = self.run_fixture(edit)
        self.assertEqual(result["primary"]["n"], 2)
        self.assertEqual(result["confirmed_preloss_primary_diagnostic_only"]["n"], 1)
        self.assertTrue(result["events"][0]["preloss_online_camera"])

    def test_target_or_rng_divergence_is_integrity_failure(self):
        for key in ("target_pos", "rng_probe"):
            def edit(seed, arm, tr):
                if arm == "B4":
                    tr[key][15, 0, 0] += 1
            result = self.run_fixture(edit)
            self.assertEqual(result["integrity"]["status"], "FAIL")

    def test_smoke_pass_forced_reset_and_evaluator_noninterference(self):
        result = self.run_fixture(smoke=True)
        self.assertEqual(result["smoke_status"], "PASS", result["integrity"])
        self.assertEqual(result["counts"]["loaded_processes"], 8)
        self.assertEqual(result["smoke_noninterference"], {"forced": True, "noeval": True})
        for suffix in ("forced", "noeval"):
            def change(name, tr):
                if name == suffix:
                    tr["digest"][45, 0, 0] ^= np.uint8(1)
            result = self.run_fixture(smoke=True, extra_edit=change)
            self.assertEqual(result["smoke_status"], "FAIL")

    def test_smoke_own_trigger_safety_and_budget_gate(self):
        def crash(seed, arm, tr):
            if arm == "B4":
                tr["crashed"][10, 0] = tr["reset"][10, 0] = True
                tr["intervening"][11:30, 0] = False
        result = self.run_fixture(crash, smoke=True)
        self.assertEqual(result["smoke_status"], "FAIL")
        self.assertEqual(result["safety_own_triggered_windows"]["B4"]["collision_fraction"], .5)
        def budget(seed, arm, tr):
            tr["budget_error"][15, 0, 0] = 2e-6
        self.assertEqual(self.run_fixture(budget, smoke=True)["smoke_status"], "FAIL")

    def test_smoke_without_nonzero_lateral_is_inconclusive(self):
        def edit(seed, arm, tr):
            tr["shadow_offset"][:] = 0
        result = self.run_fixture(edit, smoke=True)
        self.assertEqual(result["smoke_status"], "INCONCLUSIVE")

    def test_archived_shadow_norms_verify_budget_independently(self):
        def edit(seed, arm, tr):
            tr["shadow_B5_nominal_xy"][15, 0, 0] = 1.1
        result = self.run_fixture(edit, smoke=True)
        self.assertEqual(result["smoke_status"], "FAIL")
        self.assertGreater(result["same_state_budget_max_error_mps"], .09)


if __name__ == "__main__":
    unittest.main()
