# S-L1A session recovery and CPU validation — 2026-10-18

The previous session stopped at its usage limit after the oracle (`3bab6e3`) and amendment
(`41cc044`) commits. This session resumed at 04:12 KST on branch `codex/mt-p1`, HEAD `41cc044`.
Before any edit it copied every uncommitted S-L1A path to session scratch, with SHA-256 sums.
No file was regenerated. Runtime source bytes are unchanged since recovery.

## Sources present at recovery

| Path | State | Recovery SHA-256 (prefix) | Changed in this session |
| --- | --- | --- | --- |
| `aerial_gym/task/navrl_task/navrl_detector.py` | modified, +25/−1 | `8f634f22` | no |
| `aerial_gym/task/navrl_task/navrl_perception.py` | modified, +51/−12 | `d97a0f9b` | no |
| `aerial_gym/task/navrl_task/navrl_sensor_lineage.py` | new, 79 lines | `8f5f0fc3` | no |
| `tools/s_l1a_launcher.py` | new, 193 lines | `adda693a` | no |
| `tools/s_l1a_run.py` | new, 401 lines | `883efe07` | no |
| `tools/s_l1a_analysis.py` | new, 242 lines | `87b45267` | no |
| `tests/test_s_l1a_run.py` | new, 253 lines | `1a229efd` | no |
| `tests/test_s_l1a_analysis.py` | new, 163 lines | `cb884aec` | no |
| `tests/test_s_l1a_runtime.py` | new | `b52b2aed` | yes: test harness only, plus one new test |
| `tests/test_eq1_geometry_reference.py` | modified | `9ee428b2` | yes: change split out, see below |

The protected untracked raw paths `results/mt_p1_precision_v1` and `results/occlusion_pilot_v1`
were not staged, moved or deleted.

The OFF contract was confirmed by reading the code. An unset or `off` switch skips the extra
buffer, the extra query and the carve helper, and the historical `else` branch is verbatim. Any
other value fails closed. ON refuses detection latency, target-mask backfill, a non-off
`latency_obstacle_fix` and empirical latency.

## EQ-1A test change and its separation

The recovered diff is preserved byte for byte in
[`eq1_test_change_recovered.patch`](eq1_test_change_recovered.patch) (SHA-256 `4c584a03…`).

- **Original assertion.** `test_source_is_the_measured_source` read `geo.run()["source"]`. That
  value runs `git diff --quiet 5ff285a HEAD` over the four paths EQ-1A measured: perception,
  detector, LiDAR config and `occlusion_geometry.py`.
- **Why it breaks.** Committing the approved S-L1A detector and perception edits makes that
  assertion false by construction. It checks live identity, not an EQ-1 criterion.
- **What the recovered edit did.** It rewrote that test in place. The new version bound the frozen
  `eq1a_geometry.json` source record to the immutable `5ff285a` blobs. No numerical criterion,
  oracle or counterexample test changed.
- **Separation applied.** The EQ-1 file now equals its committed bytes except for one change: the
  obsolete live-identity method is replaced by a four-line comment pointing to the new test. All
  of its counterexample tests still run unchanged on the default-OFF path, and they still assert
  the preserved production FAILs.
- **New test file.** [`tests/test_s_l1a_eq1_provenance.py`](../../../tests/test_s_l1a_eq1_provenance.py)
  makes three checks:
  - the frozen record matches the `5ff285a` blobs;
  - the four measured paths reached the S-L1A base `289fd80` unchanged;
  - the live LiDAR config and `occlusion_geometry.py` still equal the recorded bytes.

  Only the two approved runtime paths may differ.

## Runtime test harness fix

`DetectorCompositionTest` failed only when ON. The detector imports
`aerial_gym.task.navrl_task.navrl_sensor_lineage`, and outside `load_production` that import
pulled in the real package `__init__`, which loads Isaac Gym. Inside the restoring
`eq1_common.production()` context, the test now registers stub packages and the path-loaded
helper. Production code is unchanged.

One test was added. It loads the detector's own 160×90 and 40×24 ray tables and computes float64
first hits along them, which stand in for the Warp query (GPU accuracy is still the canary gate).
It runs the actual `render_raw_rgbd` composition and compares the result with the independent
oracle render on robust pixels.

The detector's 160×90 rays differ from the oracle's rays by at most 1.19e−7 (float32).

| Fixture | Path | Max error (m) | False-clear px | False-obstacle px |
| --- | --- | --- | --- | --- |
| two-depth | S-L1A 160×90 | 8.4e−7 | 0 | 0 |
| two-depth | legacy 40×24 bilinear | 1.746 | 180 | 0 |
| edge | S-L1A 160×90 | 4.2e−7 | 0 | 0 |
| edge | legacy 40×24 bilinear | 2.626 | 180 | 180 |
| thin bar | S-L1A 160×90 | 4.7e−7 | 0 | 0 |
| thin bar | legacy 40×24 bilinear | 2.000 | 20 | 0 |

These are constructed CPU fixtures, not population rates.

## CPU validation before the GPU canary

- **S-L1A tests.** The S-L1A suites pass:
  - `test_s_l1a_reference` 17;
  - `test_s_l1a_runtime` 13;
  - `test_s_l1a_run` 14;
  - `test_s_l1a_analysis` 12;
  - `test_s_l1a_eq1_provenance` 3.

  Run together with `test_eq1_geometry_reference` (8) and the frozen adapter contract test, all 81
  are OK. The runtime suite includes the S14 foreground retention check, the no-camera refusal and
  the OFF bitwise-equality checks against `183a333`, for both perception fusion and the detector.
- **ON guard check.** Under each process environment (3 sensor and 8 closed-loop), the effective
  perception constructs on CPU. The ON flag is set only where intended, with latency 0,
  backfill False and the fix off. The fresh-measurement guard therefore refuses no planned ON arm.
- **Full PUBLIC_CPU before commit.**
  - Result: 2,477 run, 10 skipped, 0 errors and 1 failure.
  - The failure: `test_recovery_observability_contract…test_no_new_runtime_readback_or_frozen_source_edits`
    compares working-tree bytes with `HEAD`, so it fails exactly while the approved runtime edits
    are uncommitted.
  - It is re-run after the source commit, and the result is recorded in WORKLOG.
- **Gate audit.** `tools/s_l1a_analysis.py` implements all six verdict gates, each at the
  preregistered threshold or stricter:
  - A-DEPTH: zero pixels above 1e−4 m and zero false-clear or false-positive pixels at 0.05 m;
  - A-BOUNDARY: every `A_*` scene pass individually;
  - A-CARVE: the S14 and `A_FOREGROUND` bar returns, uncertain carves, and independent
    mask mismatches;
  - A-OFF and A-EL0: digest equality over 14 keys;
  - A-INTEGRITY.

  A-RESOURCE is characterisation only. Amendment 1 seeds (static 4501, moving 4502) are applied to
  closed-loop processes only.
