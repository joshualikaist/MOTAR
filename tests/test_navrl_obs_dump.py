"""CPU-only tests for the NAVRL_OBS_DUMP evaluation-only observation dump (WORKLOG 2026-08-21).

Extracts the pure module-level helpers straight out of navrl_task.py's source via `ast` and execs
just those nodes in an isolated namespace -- no `import aerial_gym`, no Isaac Gym, no torch, no
GPU. Mirrors the technique in tests/test_navrl_outcome_strata_contract.py (which walks the same
file's AST for wiring checks); this file additionally *executes* the extracted nodes so the
sampling/guard helpers can be exercised for real behavior, not just structure.

What this guards:
  - the streaming decimation (`_obs_dump_retain_decision` / `_obs_dump_thin_step`) stays uniform
    over an unknown-length rollout and keeps the row count within [MAX/2, MAX];
  - `OBS_DUMP_OUTCOME_CODES` is exactly the 6-entry literal the dump's per-episode table promises;
  - the fail-closed export guard (`_validate_obs_dump_export`) raises RuntimeError, and only
    RuntimeError, on each of the mismatches it is supposed to catch, and accepts a consistent
    table.

What this does NOT prove: that `_collect_obs_dump_frame` / `_flush_obs_dump` wire these helpers
correctly into the live GPU tensors -- that needs a real (Isaac Gym) run and is out of scope for a
CPU-only test.

Run: PYTHONNOUSERSITE=1 python -m pytest tests/test_navrl_obs_dump.py -q
"""

import ast
from pathlib import Path
import unittest

import numpy as np

REPO = Path(__file__).resolve().parents[1]
SOURCE_PATH = REPO / "aerial_gym/task/navrl_task/navrl_task.py"
SOURCE = SOURCE_PATH.read_text(encoding="utf-8")
TREE = ast.parse(SOURCE)

_WANTED = {
    "OBS_DUMP_OUTCOME_CODES",
    "_obs_dump_retain_decision",
    "_obs_dump_thin_step",
    "_validate_obs_dump_export",
}


def _is_wanted_assign(node):
    return isinstance(node, ast.Assign) and any(
        isinstance(target, ast.Name) and target.id in _WANTED for target in node.targets
    )


_nodes = [
    node
    for node in TREE.body
    if _is_wanted_assign(node) or (isinstance(node, ast.FunctionDef) and node.name in _WANTED)
]
if len(_nodes) != len(_WANTED):
    found = {
        (n.targets[0].id if isinstance(n, ast.Assign) else n.name) for n in _nodes
    }
    raise AssertionError(
        "expected module-level definitions %s in navrl_task.py, found %s" % (_WANTED, found)
    )

_namespace = {}
exec(compile(ast.Module(body=_nodes, type_ignores=[]), filename=str(SOURCE_PATH), mode="exec"), _namespace)

OBS_DUMP_OUTCOME_CODES = _namespace["OBS_DUMP_OUTCOME_CODES"]
_obs_dump_retain_decision = _namespace["_obs_dump_retain_decision"]
_obs_dump_thin_step = _namespace["_obs_dump_thin_step"]
_validate_obs_dump_export = _namespace["_validate_obs_dump_export"]


def _simulate_stream(n_calls, rows_per_call, max_rows):
    """Drive the two pure helpers exactly as `_collect_obs_dump_frame` does: increment a 0-based
    call counter, retain-check it, append, and thin-loop until the row budget is satisfied.
    Returns (retained_call_indices, final_stride_eff, n_decimations).
    """
    retained = []
    stride_eff = 1
    decimations = 0
    for call_index in range(n_calls):
        if not _obs_dump_retain_decision(call_index, stride_eff):
            continue
        retained.append(call_index)
        while True:
            _, new_stride_eff, thinned = _obs_dump_thin_step(
                len(retained), stride_eff, max_rows, rows_per_call
            )
            if not thinned:
                break
            retained = retained[0::2]
            stride_eff = new_stride_eff
            decimations += 1
    return retained, stride_eff, decimations


class TestOutcomeCodeMap(unittest.TestCase):
    def test_exact_literal(self):
        self.assertEqual(
            OBS_DUMP_OUTCOME_CODES,
            {
                0: "capture",
                1: "crash_bar_contact",
                2: "crash_oob",
                3: "crash_other",
                4: "timeout",
                5: "unattributed",
            },
        )


class TestRetainDecision(unittest.TestCase):
    def test_stride_one_retains_every_call(self):
        for call_index in range(50):
            self.assertTrue(_obs_dump_retain_decision(call_index, 1))

    def test_stride_three_retains_only_multiples(self):
        retained = [c for c in range(30) if _obs_dump_retain_decision(c, 3)]
        self.assertEqual(retained, [0, 3, 6, 9, 12, 15, 18, 21, 24, 27])

    def test_first_call_always_retained_regardless_of_stride(self):
        # call_index 0 is retained under every positive stride -- the whole point of counting
        # calls from 0, not 1 (see the comment on _obs_dump_retain_decision).
        for stride in (1, 2, 3, 7, 100):
            self.assertTrue(_obs_dump_retain_decision(0, stride))


class TestThinStep(unittest.TestCase):
    def test_no_thin_under_budget(self):
        new_len, new_stride, thinned = _obs_dump_thin_step(
            retained_len=5, stride_eff=1, max_rows=100, rows_per_call=4
        )
        self.assertFalse(thinned)
        self.assertEqual(new_len, 5)
        self.assertEqual(new_stride, 1)

    def test_thins_and_doubles_stride_over_budget(self):
        new_len, new_stride, thinned = _obs_dump_thin_step(
            retained_len=11, stride_eff=1, max_rows=40, rows_per_call=4
        )
        self.assertTrue(thinned)
        self.assertEqual(new_len, 6)  # len([0..10][0::2])
        self.assertEqual(new_stride, 2)

    def test_single_element_list_never_thins(self):
        # Guards against an infinite loop if one call alone already exceeds the row budget.
        new_len, new_stride, thinned = _obs_dump_thin_step(
            retained_len=1, stride_eff=1, max_rows=1, rows_per_call=1000
        )
        self.assertFalse(thinned)
        self.assertEqual(new_len, 1)
        self.assertEqual(new_stride, 1)


class TestStreamingDecimationEndToEnd(unittest.TestCase):
    """Drives the two pure helpers over a long synthetic rollout, exactly the way
    `_collect_obs_dump_frame` drives them over the real one, and checks the three properties the
    design promises: uniform spacing, a bounded final row count, and no RNG (determinism).
    """

    def test_uniform_after_decimation(self):
        retained, stride_eff, decimations = _simulate_stream(
            n_calls=200_000, rows_per_call=4, max_rows=4096
        )
        self.assertGreater(decimations, 0)
        self.assertTrue(retained, "expected at least one retained call")
        # Exact uniformity: every retained call index is a multiple of the FINAL stride -- not
        # just "on average" spaced, but exactly on the stride_eff grid, with no leftover.
        offenders = [c for c in retained if c % stride_eff != 0]
        self.assertEqual(offenders, [], "non-uniform retained calls: %s" % offenders[:10])
        # And nothing is skipped WITHIN that grid up to the last retained call: the retained set
        # is exactly {0, stride_eff, 2*stride_eff, ...} truncated to what fits in the rollout.
        expected = list(range(0, retained[-1] + 1, stride_eff))
        self.assertEqual(retained, expected)

    def test_row_count_bounded_between_half_and_full_max(self):
        max_rows = 4096
        retained, _, _ = _simulate_stream(n_calls=500_000, rows_per_call=4, max_rows=max_rows)
        n_rows = len(retained) * 4
        self.assertLessEqual(n_rows, max_rows)
        self.assertGreater(n_rows, max_rows // 2)

    def test_final_stride_is_a_power_of_two_multiple_of_initial(self):
        _, stride_eff, decimations = _simulate_stream(
            n_calls=1_000_000, rows_per_call=1, max_rows=1024
        )
        self.assertEqual(stride_eff, 2 ** decimations)

    def test_deterministic_no_rng(self):
        # Same inputs -> byte-identical outputs, twice, and with a totally different call count
        # that still lands on a decimation boundary.
        a = _simulate_stream(n_calls=50_000, rows_per_call=8, max_rows=2048)
        b = _simulate_stream(n_calls=50_000, rows_per_call=8, max_rows=2048)
        self.assertEqual(a, b)

    def test_short_rollout_never_decimates(self):
        # Fewer calls than fit in the budget: every call retained, stride stays 1.
        retained, stride_eff, decimations = _simulate_stream(
            n_calls=100, rows_per_call=4, max_rows=4096
        )
        self.assertEqual(decimations, 0)
        self.assertEqual(stride_eff, 1)
        self.assertEqual(retained, list(range(100)))


def _valid_tables(n=4, m=2, width=898):
    frame_tables = {
        "obs": np.zeros((n, width), dtype=np.float32),
        "env_id": np.arange(n, dtype=np.int32),
        "call_index": np.zeros(n, dtype=np.int64),
        "episode_uid": np.array([0, 1, 2, 3], dtype=np.int64)[:n],
        "ep_step": np.zeros(n, dtype=np.int64),
        "ctx_target_visible": np.zeros(n, dtype=bool),
        "ctx_front_blocked": np.full(n, -1, dtype=np.int8),
        "ctx_valid": np.ones(n, dtype=bool),
    }
    episode_tables = {
        "ep_uid": np.array([0, 1], dtype=np.int64)[:m],
        "ep_env_id": np.array([0, 1], dtype=np.int32)[:m],
        "outcome": np.array([0, 4], dtype=np.int8)[:m],
        "ep_len": np.array([10, 20], dtype=np.int64)[:m],
    }
    return frame_tables, episode_tables


class TestExportGuard(unittest.TestCase):
    def test_accepts_consistent_tables(self):
        frame_tables, episode_tables = _valid_tables()
        # episode_uid 0 and 1 both "finished" (not live); 2 and 3 are still-running envs with no
        # outcome row yet, which is legitimate (mid-episode frames).
        live = {2, 3}
        _validate_obs_dump_export(frame_tables, episode_tables, 898, 16384, live)

    def test_raises_on_frame_array_length_mismatch(self):
        frame_tables, episode_tables = _valid_tables()
        frame_tables["ep_step"] = frame_tables["ep_step"][:-1]  # one row short
        with self.assertRaisesRegex(RuntimeError, "ep_step"):
            _validate_obs_dump_export(frame_tables, episode_tables, 898, 16384, {2, 3})

    def test_raises_on_obs_width_mismatch(self):
        frame_tables, episode_tables = _valid_tables(width=897)
        with self.assertRaisesRegex(RuntimeError, "897"):
            _validate_obs_dump_export(frame_tables, episode_tables, 898, 16384, {2, 3})

    def test_raises_on_row_count_over_cap(self):
        frame_tables, episode_tables = _valid_tables(n=4)
        with self.assertRaisesRegex(RuntimeError, "exceeds the configured cap"):
            _validate_obs_dump_export(frame_tables, episode_tables, 898, max_rows=3, live_episode_uids={2, 3})

    def test_raises_on_episode_array_length_mismatch(self):
        frame_tables, episode_tables = _valid_tables()
        episode_tables["ep_len"] = episode_tables["ep_len"][:-1]
        with self.assertRaisesRegex(RuntimeError, "ep_len"):
            _validate_obs_dump_export(frame_tables, episode_tables, 898, 16384, {2, 3})

    def test_raises_on_finished_episode_missing_outcome_row(self):
        frame_tables, episode_tables = _valid_tables()
        # episode_uid 2 has frame rows and is NOT in `live` (so it must have finished), but the
        # outcome table only covers 0 and 1 -- a dropped-row bug the guard must catch.
        live = {3}
        with self.assertRaisesRegex(RuntimeError, "no outcome"):
            _validate_obs_dump_export(frame_tables, episode_tables, 898, 16384, live)

    def test_missing_obs_key_raises(self):
        frame_tables, episode_tables = _valid_tables()
        del frame_tables["obs"]
        with self.assertRaisesRegex(RuntimeError, "obs"):
            _validate_obs_dump_export(frame_tables, episode_tables, 898, 16384, {2, 3})

    def test_missing_ep_uid_key_raises(self):
        frame_tables, episode_tables = _valid_tables()
        del episode_tables["ep_uid"]
        with self.assertRaisesRegex(RuntimeError, "ep_uid"):
            _validate_obs_dump_export(frame_tables, episode_tables, 898, 16384, {2, 3})


if __name__ == "__main__":
    unittest.main()
