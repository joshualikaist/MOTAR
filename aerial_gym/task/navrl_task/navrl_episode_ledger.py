"""Evaluation-only per-episode ledger with an exact per-environment quota.

Written for the independent seven-seed target-motion replication
(docs/prereg_2026-09-24_target_motion_replication_7seed.md). It addresses two gaps in the aggregate
bulk-evaluation export used on 2026-09-18:

1. The export was aggregate only, so `min_relative_distance_m` and an exact-N view were NOT_RECORDED.
2. The count-based stop ("stop when pooled completions reach N") drops each environment's episode
   still in flight. Those are disproportionately long, mostly timeouts, so the rates carry an
   arm-dependent length bias.

The quota rule counts only the first `quota` episodes each environment completes. Every environment
keeps stepping until all have reached the quota, so reaching a quota changes nothing in the
simulation. Later episodes are still written, with `in_quota = false`.

The legacy view is kept beside the quota view so the two estimators can be compared on the same
episodes. It holds every episode that finished at or before the first step on which the pooled count
reached `legacy_export_at`, which is the window the aggregate `result.json` summarises.

Like the episode forensics, this recorder reads detached tensors, draws no randomness, writes into
no task buffer, and none of its fields is an input to anything.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path

import torch

SCHEMA = "motar.episode-ledger.v1"
OUTCOMES = ("capture", "crash", "timeout")
ON_VALUES = ("1", "true", "yes", "on")
ROWS_NAME = "episodes.jsonl"
SUMMARY_NAME = "episode_ledger_summary.json"
# The rl_games player stops on a pooled episode count. Under the quota rule that count must never
# bind first, so the launcher raises the player cap far above the legacy window.
MIN_PLAYER_CAP_FACTOR = 100


class EpisodeLedger:
    def __init__(self, num_envs, quota_per_env, rows_path, summary_path, context, legacy_export_at):
        if int(quota_per_env) < 1:
            raise ValueError("quota_per_env must be at least 1")
        self.num_envs = int(num_envs)
        self.quota = int(quota_per_env)
        self.rows_path = Path(rows_path)
        self.summary_path = Path(summary_path)
        self.context = dict(context)
        self.legacy_export_at = int(legacy_export_at)
        self.complete = False
        self._episode_index = [0] * self.num_envs
        self._step = 0
        self._rank = 0
        self._quota_counts = dict.fromkeys(OUTCOMES, 0)
        self._quota_rows = 0
        self._quota_distance_sum = 0.0
        self._legacy_counts = dict.fromkeys(OUTCOMES, 0)
        self._legacy_rows = 0
        self._legacy_cutoff_step = None
        self._rows = 0
        self._partition_violations = 0
        self.rows_path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = open(self.rows_path, "w", encoding="utf-8")

    def record(self, finished, successes, crashes, timeouts, min_distance):
        """Write one row per finished episode. Call once per task step, before any reset."""
        self._step += 1
        finished = finished.detach().to(torch.bool).cpu()
        if not bool(finished.any()):
            return
        capture = successes.detach().to(torch.bool).cpu()
        crash = (crashes.detach() if crashes.dtype == torch.bool else crashes.detach() > 0).cpu()
        timeout = timeouts.detach().to(torch.bool).cpu()
        distance = min_distance.detach().to(torch.float64).cpu()
        in_legacy = self._legacy_cutoff_step is None
        for env in torch.nonzero(finished, as_tuple=False).flatten().tolist():
            flags = (bool(capture[env]), bool(crash[env]), bool(timeout[env]))
            valid = sum(flags) == 1
            outcome = OUTCOMES[flags.index(True)] if valid else "invalid"
            if not valid:
                self._partition_violations += 1
            k = self._episode_index[env]
            self._episode_index[env] = k + 1
            self._rank += 1
            in_quota = k < self.quota
            value = float(distance[env])
            row = {
                "cell_id": self.context.get("cell_id"),
                "arm": self.context.get("arm"),
                "bars": self.context.get("bars"),
                "seed": self.context.get("seed"),
                "env_index": env,
                "env_episode_index": k,
                "episode_id": "%s/env%03d/ep%03d" % (self.context.get("cell_id"), env, k),
                "completion_step": self._step,
                "completion_rank": self._rank,
                "outcome": outcome,
                "capture": flags[0],
                "crash": flags[1],
                "timeout": flags[2],
                "min_relative_distance_m": value,
                "in_quota": in_quota,
                "in_legacy_window": in_legacy,
            }
            self._handle.write(json.dumps(row, sort_keys=True) + "\n")
            self._rows += 1
            if in_quota and valid:
                self._quota_counts[outcome] += 1
                self._quota_rows += 1
                self._quota_distance_sum += value
            if in_legacy and valid:
                self._legacy_counts[outcome] += 1
                self._legacy_rows += 1
        if in_legacy and self._legacy_rows >= self.legacy_export_at:
            self._legacy_cutoff_step = self._step
        self.complete = min(self._episode_index) >= self.quota

    def summary(self, legacy_result_written):
        expected = self.num_envs * self.quota
        if self._partition_violations:
            status = "INVALID_OUTCOME_PARTITION"
        elif not self.complete or self._quota_rows != expected:
            status = "INCOMPLETE"
        elif not legacy_result_written:
            status = "INVALID_LEGACY_RESULT_MISSING"
        else:
            status = "COMPLETE"
        rates = {k: v / self._quota_rows for k, v in self._quota_counts.items()} if self._quota_rows else {}
        legacy_rates = ({k: v / self._legacy_rows for k, v in self._legacy_counts.items()}
                        if self._legacy_rows else {})
        return {
            "schema": SCHEMA,
            "status": status,
            "context": self.context,
            "num_envs": self.num_envs,
            "quota_per_env": self.quota,
            "quota": {
                "expected_episodes": expected,
                "episodes": self._quota_rows,
                "counts": self._quota_counts,
                "rates": rates,
                "min_relative_distance_m_mean": (self._quota_distance_sum / self._quota_rows
                                                 if self._quota_rows else None),
                "rule": "first quota_per_env completed episodes of every environment",
            },
            "legacy_window": {
                "export_at": self.legacy_export_at,
                "cutoff_step": self._legacy_cutoff_step,
                "episodes": self._legacy_rows,
                "counts": self._legacy_counts,
                "rates": legacy_rates,
                "rule": "every episode finished at or before the first step on which the pooled count "
                        "reached export_at (the window of the aggregate result.json)",
                "aggregate_result_written": bool(legacy_result_written),
            },
            "rows_written": self._rows,
            "task_steps": self._step,
            "episodes_per_env_min": min(self._episode_index),
            "episodes_per_env_max": max(self._episode_index),
            "partition_violations": self._partition_violations,
            "rows_file": self.rows_path.name,
        }

    def finalize(self, legacy_result_written):
        """Close the rows file, then write the summary with the rows file's hash, atomically."""
        self._handle.flush()
        os.fsync(self._handle.fileno())
        self._handle.close()
        report = self.summary(legacy_result_written)
        report["rows_sha256"] = hashlib.sha256(self.rows_path.read_bytes()).hexdigest()
        tmp = self.summary_path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(report, indent=1, sort_keys=True) + "\n", encoding="utf-8")
        os.replace(tmp, self.summary_path)
        return report


def build_episode_ledger(num_envs, bulk_eval_mode, bulk_eval_output, legacy_export_at, environ=None):
    """The ledger requested by NAVRL_EPISODE_LEDGER, or None. Refuses any incomplete configuration."""
    environ = os.environ if environ is None else environ
    if environ.get("NAVRL_EPISODE_LEDGER", "0").strip().lower() not in ON_VALUES:
        return None
    problems = []
    if not bulk_eval_mode or not bulk_eval_output:
        problems.append("needs NAVRL_BULK_EVAL and NAVRL_BULK_EVAL_JSON")
    if not environ.get("NAVRL_EVAL_CHECKPOINT", "").strip():
        problems.append("needs NAVRL_EVAL_CHECKPOINT")
    try:
        quota = int(environ.get("NAVRL_EPISODE_QUOTA_PER_ENV", ""))
    except ValueError:
        quota = 0
    if quota < 1:
        problems.append("needs NAVRL_EPISODE_QUOTA_PER_ENV >= 1")
    elif quota * int(num_envs) != int(legacy_export_at):
        problems.append("NAVRL_EPISODE_QUOTA_PER_ENV x num_envs must equal PLAY_GAMES_NUM (%d x %d != %d)"
                        % (quota, int(num_envs), int(legacy_export_at)))
    try:
        cap = int(environ.get("NAVRL_PLAYER_GAMES_CAP", ""))
    except ValueError:
        cap = 0
    if cap < MIN_PLAYER_CAP_FACTOR * int(legacy_export_at):
        problems.append("needs NAVRL_PLAYER_GAMES_CAP >= %d x PLAY_GAMES_NUM so the player never stops "
                        "before the quota" % MIN_PLAYER_CAP_FACTOR)
    if problems:
        raise RuntimeError("NAVRL_EPISODE_LEDGER is evaluation-only and " + "; ".join(problems))
    cell_dir = Path(bulk_eval_output).parent

    def number(key):
        try:
            return int(environ.get(key, ""))
        except ValueError:
            return None
    context = {"cell_id": cell_dir.name, "arm": environ.get("NAVRL_TARGET_BEHAVIOR_LEVEL", "").strip() or None,
               "bars": number("NAVRL_NUM_BARS"), "seed": number("NAVRL_SEED")}
    return EpisodeLedger(num_envs, quota, cell_dir / ROWS_NAME, cell_dir / SUMMARY_NAME, context,
                         legacy_export_at)
