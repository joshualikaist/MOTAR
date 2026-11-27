"""Endpoint and evidence controls for the additive, non-authorizing audit."""
import copy
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("_formal_readiness_test", ROOT / "tools/sitl_formal_readiness.py")
M = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M)


def wire(env, k, episode=0, values=None):
    return M.FRAME.pack(env, k, episode, *(values or [0.0] * 20), 0, 0, 0)


def valid_s3a(bars=70):
    return {"cell": "E2_obstacle_aware__%dbars__seed4501" % bars, "reading": "EQUIVALENT",
            "tolerances": {"obs": 1e-3, "action": 1e-4}, "alignment_mismatches": 0,
            "envs": {str(i): {"steps": 10, "obs_err": {k: 0.0 for k in ("scan", "obstacles", "robot", "target")},
                             "act_err_bridge": 0.0, "act_err_dumpobs": 0.0, "gov_err": 0.0}
                     for i in range(8)}}


def valid_trace_contract(bars=70):
    cid = 'E2_obstacle_aware__%dbars__seed4501' % bars
    receipt = {'status': 'OK', 'seed': 4501, 'density_bars': bars, 'cell_id': cid,
               'arm': 'E2_obstacle_aware', 'checkpoint_sha256': M.F_SHA,
               'git_commit': 'a' * 40, 'git_dirty_files': 0}
    summary = {'schema': 'motar.occlusion-trace.v1', 'frame_itemsize': M.FRAME.size,
               'frame_dtype': copy.deepcopy(M.FRAME_DTYPE), 'frames': 6010,
               'files': {}, 'bytes': {},
               'metadata': {'frame_contract': M.FRAME_CONTRACT, 'dt_s': .1,
                            'context': {'arm': 'A3', 'bars': bars, 'seed': 4501, 'cell_id': cid,
                                        'checkpoint_sha256': M.F_SHA, 'target_behavior_level': 'e2_obstacle_aware'},
                            'identity': copy.deepcopy(M.TARGET_IDENTITY),
                            'geometry': {'bar_pool': 'bars_h3', 'obstacle_slots': [0, bars]},
                            'source_commit': 'a' * 40, 'source_dirty': False}}
    bindings = {'occlusion_frames.bin': {'sha256': 'b' * 64, 'bytes': 6010 * M.FRAME.size},
                'occlusion_layouts.npz': {'sha256': 'c' * 64, 'bytes': 100}}
    for name, rec in bindings.items():
        summary['files'][name], summary['bytes'][name] = rec['sha256'], rec['bytes']
    return summary, receipt, bindings


class TargetWindows(unittest.TestCase):
    def test_601_states_accept_but_600_and_208_remain_blocked(self):
        for count in (601, 600, 208):
            with self.subTest(count=count):
                payload = b"".join(wire(0, k) for k in range(count))
                row = M.target_windows(io.BytesIO(payload), count)[0]
                self.assertEqual(row["recorded_states"], count)
                self.assertEqual(row["missing_frame_indices"], list(range(count, 601)))
                self.assertEqual(row["status"], "INPUTS_COMPLETE" if count == 601 else "BLOCKED_INCOMPLETE_INPUTS")

    def test_other_episode_cannot_fill_first_episode_endpoint(self):
        payload = b"".join(wire(0, k) for k in range(600)) + wire(0, 600, episode=1)
        row = M.target_windows(io.BytesIO(payload), 601)[0]
        self.assertEqual(row["missing_frame_indices"], [600])

    def test_all_ten_envs_must_be_complete(self):
        payload = b"".join(wire(env, k) for env in range(10) for k in range(601))
        result = M.target_windows(io.BytesIO(payload), 6010)
        self.assertEqual(sum(r["status"] == "INPUTS_COMPLETE" for r in result.values()), 10)

    def test_gap_is_visible_even_with_late_endpoint(self):
        ks = [k for k in range(601) if k != 7]
        row = M.target_windows(io.BytesIO(b"".join(wire(0, k) for k in ks)), 600)[0]
        self.assertEqual(row["missing_frame_indices"], [7])
        self.assertFalse(row["contiguous_from_zero"])

    def test_malformed_streams_refuse(self):
        for payload, count in ((wire(0, 0) * 2, 2), (wire(0, 601), 1),
                               (wire(0, 0, values=[float("nan")] + [0.0] * 19), 1),
                               (wire(0, 0)[:-1], 1), (wire(0, 0) + b"x", 1)):
            with self.subTest(count=count, bytes=len(payload)), self.assertRaises(ValueError):
                M.target_windows(io.BytesIO(payload), count)

    def test_trace_schema_context_and_internal_binding_must_agree(self):
        summary, receipt, bindings = valid_trace_contract()
        M.validate_trace_contract(summary, receipt, bindings, 70)  # Positive control.
        mutations = [(['metadata', 'context', 'seed'], 4502),
                     (['metadata', 'context', 'bars'], 205),
                     (['metadata', 'context', 'cell_id'], 'wrong'),
                     (['metadata', 'identity', 'behavior_level'], 'e0'),
                     (['metadata', 'source_commit'], 'd' * 40),
                     (['metadata', 'source_dirty'], True),
                     (['metadata', 'geometry', 'bar_pool'], 'wrong'),
                     (['frame_dtype'], []), (['frames'], 6000),
                     (['files', 'occlusion_frames.bin'], 'f' * 64),
                     (['bytes', 'occlusion_layouts.npz'], 99)]
        for keys, value in mutations:
            bad = copy.deepcopy(summary)
            node = bad
            for key in keys[:-1]:
                node = node[key]
            node[keys[-1]] = value
            with self.subTest(keys=keys), self.assertRaises(ValueError):
                M.validate_trace_contract(bad, receipt, bindings, 70)
        for key, value in (('cell_id', 'wrong'), ('arm', 'wrong'), ('git_dirty_files', 1)):
            bad = dict(receipt, **{key: value})
            with self.subTest(receipt=key), self.assertRaises(ValueError):
                M.validate_trace_contract(summary, bad, bindings, 70)


class S3aReadiness(unittest.TestCase):
    def check_report(self, report, bars=70):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "s3a.json"
            path.write_text(json.dumps(report))
            return M.audit_s3a(path, bars)

    def test_nonempty_fixed_tolerance_positive_control_still_requires_review(self):
        row = self.check_report(valid_s3a())
        self.assertEqual(row["status"], "S3A_REPORTED_EQUIVALENT")
        self.assertEqual(row["independent_source_bound_review"], "REQUIRED_SEPARATELY")
        self.assertEqual(row["positive_and_one_degree_yaw_negative_controls"], "REQUIRED_SEPARATELY")

    def test_reading_label_cannot_hide_bad_numeric_or_empty_coverage(self):
        cases = []
        for field in ("act_err_bridge", "act_err_dumpobs", "gov_err"):
            report = valid_s3a(); report["envs"]["3"][field] = 1.1e-4; cases.append(report)
        report = valid_s3a(); report["envs"]["3"]["obs_err"]["target"] = 0.002; cases.append(report)
        report = valid_s3a(); report["envs"]["3"]["steps"] = 0; cases.append(report)
        report = valid_s3a(); report["envs"] = {}; cases.append(report)
        report = valid_s3a(); report["alignment_mismatches"] = 1; cases.append(report)
        report = valid_s3a(); report["tolerances"]["action"] = 0.01; cases.append(report)
        report = valid_s3a(); report["envs"]["3"]["gov_err"] = float("nan"); cases.append(report)
        for report in cases:
            with self.subTest(report=report):
                self.assertEqual(self.check_report(report)["status"], "BLOCKED_S3A")

    def test_both_cells_and_fixed_identities_required(self):
        with self.assertRaises(ValueError):
            M.audit("unused", {70: "unused"})
        self.assertEqual(self.check_report(valid_s3a(), bars=205)["status"], "BLOCKED_S3A")

    def test_frozen_negative_reports_stay_blocked(self):
        for bars in (70, 205):
            path = ROOT / "docs/evidence/sitl_2026-11-05/s3a" / ("s3a_%dbars_9882889.json" % bars)
            report = M.audit_s3a(path, bars)
            self.assertEqual(report["status"], "BLOCKED_S3A")
            self.assertTrue(report["problems"])

    def test_complete_cohort_and_reports_never_grant_gate_authority(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            inventory = {"cells": {}}
            s3a = {}
            for bars in (70, 205):
                directory = base / str(bars)
                directory.mkdir()
                summary, receipt, _ = valid_trace_contract(bars)
                (directory / "receipt.json").write_text(json.dumps(receipt))
                (directory / "occlusion_frames.bin").write_bytes(
                    b"".join(wire(env, k) for env in range(10) for k in range(601)))
                (directory / 'occlusion_layouts.npz').write_bytes(b'synthetic layout binding; geometry qualification separate')
                for name in ('occlusion_frames.bin', 'occlusion_layouts.npz'):
                    summary['files'][name] = M.digest(directory / name)
                    summary['bytes'][name] = (directory / name).stat().st_size
                (directory / "occlusion_trace_summary.json").write_text(json.dumps(summary))
                inventory["cells"][str(bars)] = {"zero": {"directory": str(directory), "files": {
                    name: {"bytes": (directory / name).stat().st_size, "sha256": M.digest(directory / name)}
                    for name in ("receipt.json", "occlusion_trace_summary.json", "occlusion_frames.bin", "occlusion_layouts.npz")}}}
                s3a[bars] = base / ("s3a_%d.json" % bars)
                s3a[bars].write_text(json.dumps(valid_s3a(bars)))
            path = base / "inventory.json"
            path.write_text(json.dumps(inventory))
            report = M.audit(path, s3a)
            self.assertEqual(report["status"], "STRUCTURAL_CHECKS_PASS_REVIEW_REQUIRED")
            self.assertFalse(report["formal_gate_authorized"])
            self.assertEqual(report["formal_gate_verdict"], "NOT_TESTED")
            self.assertEqual(sum(r["status"] == "INPUTS_COMPLETE" for r in report["inputs"]["windows"]), 20)


if __name__ == "__main__":
    unittest.main()
