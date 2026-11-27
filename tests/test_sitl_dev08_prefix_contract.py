"""Synthetic positive/rejection controls for dev08 input and censoring contracts.

No raw data, simulator, numpy, checkpoint or network dependency. Real input
verification is a separately recorded read-only preflight, not a fixture here.
"""
import contextlib
import copy
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("dev08_prefix", ROOT / "tools/sitl_dev08_prefix_contract.py")
D = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(D)


def wire_row(env, frame=0, episode=0, floats=None):
    return D.FRAME.pack(env, frame, episode, *(floats or [0.0] * 20), 0, -1, -1)


def fixture_windows():
    return {(b, e): D.frame_window(208 if (b, e) == (70, 5) else 600)
            for b in (70, 205) for e in range(10)}


def fixture_records():
    return [{"phase": "main", "protocol": "observed-prefix-v1", "operational_status": "VALID",
             "bars": b, "env": e, "arm": a, "seed": 4501, "env_episode_index": 0, "outcome": "capture",
             "final_frame_index": 3 if a == "native" else 4,
             "observation_checks": 4 if a == "native" else 5,
             "commands_issued": 3 if a == "native" else 4,
             "start_sim_time_s": 10.0, "final_sim_time_s": 10.3 if a == "native" else 10.4,
             "scheduled_elapsed_s": 0.3 if a == "native" else 0.4}
            for b in (70, 205) for e in range(10) for a in D.ARMS]


class PrefixInputTest(unittest.TestCase):
    def test_recorded_checkpoint_horizon_is_not_action_count(self):
        for n, k, t in ((1, 0, 0.0), (208, 207, 20.7), (600, 599, 59.9)):
            window = D.frame_window(n)
            self.assertEqual(window["last_frame_index"], k)
            self.assertEqual(window["last_scheduled_elapsed_s"], t)
            self.assertEqual(window["max_commands_before_last_check"], k)
            self.assertFalse(window["post_action600_state_available"])
        for n in (0, -1, 601, 208.0, True):
            with self.assertRaises(ValueError):
                D.frame_window(n)

    def test_terminal_precedence_and_no_inferred_timeout(self):
        self.assertEqual(D.checkpoint_decision(0, 208), "CONTINUE")
        self.assertEqual(D.checkpoint_decision(206, 208), "CONTINUE")
        self.assertEqual(D.checkpoint_decision(207, 208), D.CENSORED)
        self.assertEqual(D.checkpoint_decision(599, 600), D.CENSORED)
        self.assertEqual(D.checkpoint_decision(207, 208, capture=True, crash=True), "capture")
        self.assertEqual(D.checkpoint_decision(207, 208, crash=True), "crash")
        for k in (-1, 208, True):
            with self.assertRaises(ValueError):
                D.checkpoint_decision(k, 208)
        with self.assertRaises(ValueError):
            D.checkpoint_decision(0, 208, capture=1)

    def test_all_exact_envs_and_frame_indices_are_required(self):
        raw = b"".join(wire_row(env, k) for env in range(10) for k in (0, 1))
        frames = D.selected_frames(io.BytesIO(raw), 20)
        self.assertEqual(set(frames), set(range(10)))
        self.assertTrue(all(sorted(v) == [0, 1] for v in frames.values()))
        for bad, count in ((raw[:-1], 20), (raw + b"x", 20), (raw[:-2 * D.FRAME.size], 18),
                           (raw + wire_row(0, 1), 21),
                           (b"".join(wire_row(env, k) for env in range(10) for k in (0, 2)), 20)):
            with self.assertRaises(ValueError):
                D.selected_frames(io.BytesIO(bad), count)

    def test_nonfinite_selected_state_rejected_other_episode_cannot_replace_missing_env(self):
        raw = b"".join(wire_row(env) for env in range(10))
        self.assertEqual(len(D.selected_frames(io.BytesIO(raw), 10)), 10)
        invalid = [0.0] * 20
        invalid[17] = float("nan")
        bad = wire_row(0, floats=invalid) + raw[D.FRAME.size:]
        with self.assertRaises(ValueError):
            D.selected_frames(io.BytesIO(bad), 10)
        wrong_episode = wire_row(0, episode=1) + raw[D.FRAME.size:]
        with self.assertRaises(ValueError):
            D.selected_frames(io.BytesIO(wrong_episode), 10)

    def test_size_hash_guard_has_valid_control_and_rejects_drift_or_overlarge_claim(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "trace"
            path.write_bytes(b"valid")
            record = {"bytes": 5, "sha256": hashlib.sha256(b"valid").hexdigest()}
            D.verify_file(path, record)
            path.write_bytes(b"drift")
            with self.assertRaises(ValueError):
                D.verify_file(path, record)
            path.write_bytes(b"valid")
            D.verify_file(path, record)
            for size in (True, -1, D.MAX_FILE_BYTES + 1):
                with self.assertRaises(ValueError):
                    D.verify_file(path, dict(record, bytes=size))

    def test_cli_defaults_strict_and_requires_explicit_prefix_without_environment_mutation(self):
        before_env, before_path = dict(os.environ), list(sys.path)
        for args, expected in (([], "strict600"), (["--protocol", "observed-prefix-v1"], "observed-prefix-v1")):
            output = io.StringIO()
            with mock.patch.object(D, "prepare", return_value={"status": "fixture"}) as prepare:
                with contextlib.redirect_stdout(output):
                    self.assertEqual(D.main(args), 0)
                prepare.assert_called_once_with(protocol=expected)
            self.assertEqual(json.loads(output.getvalue()), {"status": "fixture"})
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
            D.main(["--execute"])
        self.assertEqual(raised.exception.code, 2)
        self.assertEqual(dict(os.environ), before_env)
        self.assertEqual(sys.path, before_path)


class PrefixLedgerTest(unittest.TestCase):
    def test_complete_positive_pairs_and_capture_checkpoint_difference(self):
        records, windows = fixture_records(), fixture_windows()
        saved = copy.deepcopy(records)
        result = D.summarize_complete_pairs(records, windows)
        self.assertEqual(records, saved)
        self.assertEqual(result["arms"]["native"]["counts"]["capture"], 20)
        self.assertEqual(result["paired_transitions"], {"capture->capture": 20})
        self.assertEqual(result["candidate_minus_native_capture_checkpoint_s"],
                         {"n": 20, "median": 0.1, "min": 0.1, "max": 0.1})
        self.assertFalse(result["full_60s_outcome_identified"])

    def test_censored_case_stays_in_twenty_member_denominator(self):
        records = fixture_records()
        row = next(r for r in records if (r["bars"], r["env"], r["arm"]) == (70, 5, "candidate"))
        row.update(outcome=D.CENSORED, final_frame_index=207, observation_checks=208, commands_issued=207,
                   final_sim_time_s=30.7, scheduled_elapsed_s=20.7)
        result = D.summarize_complete_pairs(records, fixture_windows())
        counts = result["arms"]["candidate"]
        self.assertEqual(counts["logical_count_bounds"]["capture"], {"lower": 19, "upper": 20, "denominator": 20})
        self.assertEqual(counts["logical_count_bounds"]["crash"], {"lower": 0, "upper": 1, "denominator": 20})
        self.assertEqual(result["by_density"]["70"]["candidate"]["planned"], 10)
        self.assertEqual(result["pairs_with_censoring"], 1)
        self.assertEqual(result["candidate_minus_native_capture_checkpoint_s"]["n"], 19)
        self.assertFalse(result["bounds_are_confidence_intervals"])

    def test_all_censored_is_unknown_not_success_failure_or_timeout(self):
        records, windows = fixture_records(), fixture_windows()
        for row in records:
            k = windows[(row["bars"], row["env"])]["last_frame_index"]
            row.update(outcome=D.CENSORED, final_frame_index=k, observation_checks=k + 1, commands_issued=k,
                       final_sim_time_s=10 + k * D.DT, scheduled_elapsed_s=round(k * D.DT, 10))
        result = D.summarize_complete_pairs(records, windows)
        self.assertEqual(result["arms"]["native"]["counts"], {"capture": 0, "crash": 0, D.CENSORED: 20})
        self.assertEqual(result["candidate_minus_native_capture_checkpoint_s"]["n"], 0)
        self.assertIsNone(result["candidate_minus_native_capture_checkpoint_s"]["median"])
        self.assertEqual(result["pairs_with_censoring"], 20)

    def test_missing_duplicate_foreign_or_smoke_records_rejected(self):
        records = fixture_records()
        self.assertEqual(len(D.summarize_complete_pairs(records, fixture_windows())["arms"]), 2)
        mutants = [records[:-1], records + [records[0]]]
        for change in ({"phase": "smoke"}, {"env": 10}, {"env": False}, {"bars": "70"},
                       {"seed": 4502}, {"env_episode_index": 1}, {"env_episode_index": False}):
            bad = copy.deepcopy(records)
            bad[0].update(change)
            mutants.append(bad)
        for bad in mutants:
            with self.assertRaises(ValueError):
                D.summarize_complete_pairs(bad, fixture_windows())

    def test_no_early_censor_future_endpoint_or_unobserved_command(self):
        for change in ({"outcome": D.CENSORED}, {"outcome": "timeout"}, {"outcome": "ERROR"},
                       {"final_frame_index": 600, "observation_checks": 601, "commands_issued": 600},
                       {"observation_checks": 3}, {"commands_issued": 4}, {"final_frame_index": True}):
            records = fixture_records()
            records[0].update(change)
            with self.assertRaises(ValueError):
                D.summarize_complete_pairs(records, fixture_windows())
        windows = fixture_windows()
        windows[(70, 0)]["last_frame_index"] = 600
        with self.assertRaises(ValueError):
            D.summarize_complete_pairs(fixture_records(), windows)

    def test_windows_cannot_be_shortened_to_hide_a_failed_episode(self):
        for mode in ("shorten", "missing", "extra"):
            windows = fixture_windows()
            if mode == "shorten":
                windows[(70, 0)] = D.frame_window(10)
            elif mode == "missing":
                del windows[(70, 0)]
            else:
                windows[(70, 10)] = D.frame_window(600)
            with self.assertRaises(ValueError):
                D.summarize_complete_pairs(fixture_records(), windows)

    def test_operational_failure_and_missing_or_forged_time_not_censoring(self):
        for change in ({"operational_status": "ERROR"}, {"protocol": "strict600"},
                       {"final_sim_time_s": float("nan")}, {"final_sim_time_s": 9.0},
                       {"start_sim_time_s": True}, {"scheduled_elapsed_s": 0.4}):
            records = fixture_records()
            records[0].update(change)
            with self.assertRaises(ValueError):
                D.summarize_complete_pairs(records, fixture_windows())


if __name__ == "__main__":
    unittest.main()
