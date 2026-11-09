# CAM-K2 v3 pre-launch design review r1

- Reviewer: Claude Code subagent, model Opus 5.5 (claude-opus-5-5), fresh context, read-only on the worktree.
- Date: 2026-11-09 (about 12:20-12:45 KST).
- Target: `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `d2a018b`, with the
  STAGED unit (`git diff --cached`, 8 files). Staged SHA-256 values match the probe's "as run" hashes:
  `navrl_depth_validity.py` 8be04135..., `realveh_entry.py` 49d1db92..., `train_navrl_camk2v3.sh` 0f9badb5...
  `git diff ceab4a0 HEAD` is empty for `aerial_gym/task/navrl_task`, `tools/realveh_entry.py` and `tools/camk2v2_train.py`,
  so the probe's base and HEAD agree on every file the rule touches.
- What I ran (CPU, cores 8,9, nice 10): `test_depth_validity` 17 OK; `test_camk2v3_train` 18 OK (in isolation);
  `test_camk2v2_train` 38 OK; `test_experiment_index` 9 OK; `test_realveh` 48 OK; `test_sitl_agq_geometry_diag` 2 errors
  without `OMP_NUM_THREADS/MKL_NUM_THREADS=1`, 25 OK with them. I read the coordinator's PUBLIC_CPU log
  `~/motar_tmp/camk2v3_20261108/public_cpu_precommit.log` (finished during this review: EXIT 1, faults 0 -> 0).
- I reproduced the hit_token references from the raw v2 and CAM-K1 launcher logs with the wrapper's own regex:
  v2 s1 33 windows, last-5 mean 0.1668; v2 s2 28, 0.1916; CAM-K1 s1 115, 0.6768; s2 115, 0.7222. All four match.

## Findings

### BLOCKER

**B-1. PUBLIC_CPU fails on the unit's own test.** `tests/test_camk2v3_train.py:349-350`
`test_no_lasting_path_change` compares `sys.path` at *run* time with the snapshot taken at import (`:37`). In the
single-process suite, test modules imported after this one add entries to `sys.path`, so it fails:
`FAIL: test_no_lasting_path_change (test_camk2v3_train.TestIsolation)`, "First list contains 120 additional elements"
(public_cpu_precommit.log). v2 avoided this with `_PATH_RESTORED = sys.path == _saved_path` computed right after the
restore (`tests/test_camk2v2_train.py:46`). AGENTS.md requires PUBLIC_CPU to end with 0 failures, and the prereg itself
(`prereg_v3:136`) requires the worktree run before the commit.
*Fix:* compute `_PATH_RESTORED = sys.path == _saved_path` at `:43` (after the `finally`) and assert it in the test.
Then rerun PUBLIC_CPU with `OMP_NUM_THREADS=1 MKL_NUM_THREADS=1` exported (see L-1), and record the result in the
prereg's *Tests* section (it can still be edited, because it is not committed) or in the WORKLOG.

### MAJOR

**M-1. The TRAINED check for the new treatment is vacuous about its effect.** `tools/camk2v3_train.py:429-450`
`validity_telemetry_problems` accepts `near_pixels == 0` (it checks `0 <= near <= invalid <= pixels`). If the rule were
installed but never converted a pixel (for example a regression in `dense_mask`, or the depth model's mask always
empty), the run would still read TRAINED. v2 required the vehicle randomization to have acted ("no motor-update count
> 0", `:396-398`); the rule needs the same. The fixtures already assume it (`tests/test_camk2v3_train.py:239`
asserts `near_pixels > 0`), but no test fails a line with `near_pixels: 0`. A normal run would not be refused: the v2
depth-model summaries after 50 calls show `invalid_minz` 0.0370 (smoke), 0.0377 (s1), 0.0375 (s2) of about 46 M
eligible pixels, that is about 1.7 M sub-MinZ pixels, almost all inside dense blobs (the probe turned 301,445 of 301,468
into near). The first-frame line has `invalid_minz` 0.0 in all three v2 runs, so the check belongs on the summary line only.
*Fix:* require `near_pixels > 0` and `invalid_pixels > 0` on the summary line, add a negative test (a summary
line with `near_pixels: 0` fails) next to the existing positive control, and add the clause to the prereg's TRAINED
list (`prereg_v3:109-110`).

**M-2. The disk floor will probably refuse seed 2 of a normal chain.** `tools/camk2v3_train.py:73,563-565`
(`MIN_FREE_GB = 1.0`, checked at each preflight). Free now: 1.324 GB (`shutil.disk_usage`). Measured from v2:
smoke 17.8 MB (run) + 26.7 MB (unit) = 0.045 GB. Each seed: run folder 21.6 MB + 19 intermediate checkpoints x
8.88 MB (they are kept until a deletion is approved) + unit folder 28.7 MB + session log, about 0.22 GB. So seed 2's
preflight sees about 1.324 - 0.045 - 0.22 = 1.06 GB, if nothing else writes. That leaves about 60 MB of margin, and
other sessions, caches and logs write to the same full disk. A refusal is fail-closed (nothing is created), but the
chain stops and needs a new user decision in the middle of the frozen window. The v3 prereg does not state the floor
value. It inherits it as "v2's list" (`prereg_v3:96`). v2 stated it (`prereg_v2:196`).
*Fix (any one, before the commit):* (a) get the user's approval now to delete the seed-1 v3 intermediate checkpoints
(not ep29000, not `gen_ppo`) between seed 1 and seed 2, and add that step to the chain; or (b) free about 0.3 GB
before launch with an approved deletion; or (c) preregister a lower floor with its derivation (for example 0.5 GB,
about 2x one seed's 0.22 GB). In every case write the floor value and the expected use into the prereg.

### MINOR

**m-1. The experiment index entry has broken nesting.** `docs/EXPERIMENTS.md:773-778`: the new CAM-K2 v3 bullet sits
between the CAM-K2-E v1 bullet and CAM-K2-E's own sub-bullets ("Gate A: crash, -5 pp. Gate N ... Gate I", "Before
launch, a CPU depth probe ..."). Those sub-bullets now read as v3's gates, but v3 has no gates and claims no
performance result. *Fix:* move the v3 bullet below the two CAM-K2-E sub-bullets.

**m-2. A sim-to-real limit is missing: other real invalid patterns.** `prereg_v3:140-144` discloses the too-far,
textureless and overexposed cases (good, NOT_TESTED). It omits other dense invalid patterns of real stereo depth:
occlusion shadows beside foreground edges, left-right-check failures, and specular or reflective surfaces. With a 5 x 5
window and density 0.5, any invalid band at least 3 px wide (15/25) becomes a phantom surface at MinZ. *Fix:* add
these to the limit. A policy trained with "dense invalid = obstacle at 0.4 m" may react to them on hardware (NOT_TESTED).

**m-3. The rule also changes the detector's input.** It is not disclosed. Near-blob pixels now pass the detector
mask `depth < max_camera_range` (`navrl_perception.py:1442,1601`), and v2 excluded them. So a target inside MinZ can now
be detected at about 0.4 m, and those pixels are then blanked from the obstacle fusion as target pixels (`:1909-1911`).
This uses only sensor data (no GT; the observation contract holds), and it is rare (target within 0.4 m). *Fix:* one
sentence in *Disclosed limits* or *The rule*: the rule's output feeds both the obstacle fusion and the colour-rule
detector.

**m-4. The DIAG_LOW reading is over-strong.** `prereg_v3:115-117`: "DIAG_LOW would mean that the rule does not restore
hit-bar tokens". hit_token is a training-rollout share that also moves with the policy's crash geometry (v3's policy
diverges from v2's and from CAM-K1's), with the 240 deg sector, and with association against a bar reported at 0.4 m
instead of its true distance. *Fix:* "DIAG_LOW would be consistent with the rule not restoring hit-bar tokens ...;
DIAG_OK does not show restored perception or any crash effect". The references' window spans also differ (CAM-K1 115
windows over its run, v2 28-33 over 1,000 epochs). Say so.

**m-5. The seed order is enforced only outside the repository.** "Seed 2 starts only if seed 1 reads TRAINED"
(`prereg_v3:94`) is enforced by `~/motar_tmp/camk2v3_20261108/run_camk2v3_chain.sh`, not by `tools/camk2v3_train.py`
(`main`, `:655-668`, checks only the smoke). This is the same as v2. *Fix (recommended):* seed 2 refuses unless
`UNIT_DIR/train_s1/train_report.json` reads TRAINED at the same HEAD, with a test. Or, if that is not done, the prereg
should name the chain script and its SHA-256 as the enforcement.

### LOW

- **L-1.** The other 5 PUBLIC_CPU errors are environmental, not the unit. 2 are `test_boia_planner` (the 3 GB disk
  floor; they also failed in v2's pre-commit log). 3 are `test_sitl_agq_geometry_diag` / `test_sitl_s3b_dev08_prefix`
  ("OMP_NUM_THREADS must be 1; MKL_NUM_THREADS must be 1": `run_public_cpu.sh` does not export them; with them set,
  `test_sitl_agq_geometry_diag` passed 25/25). Export both in the runner script and record the 2 boia errors as the
  known disk-floor case.
- **L-2.** The `dense_mask` docstring says `avg_pool2d(count_include_pad=False)` (`navrl_depth_validity.py:21-22`). The
  code uses `count_include_pad=True` with an explicit in-image count (`:90-92`). The two are equivalent, and the corner
  and edge tests confirm it (`tests/test_depth_validity.py:400-401`). Align the text.
- **L-3.** `prereg_v3:6-9` cites the tree copy `~/motar_tmp/camk2v3_20261108/tree`, which no longer exists. Say it was
  removed and bind `wip/SHA256SUMS.txt`. The committed prereg also differs from the wip copy (wip
  6412914..., staged de1a368...), which is fine before the commit; the probe's code hashes equal the staged code.
- **L-4.** The wrapper and launcher headers still say "draft docs/plans/prereg_..." (`camk2v3_train.py:1`,
  `train_navrl_camk2v3.sh:2`). This is cosmetic.
- **L-5.** "OFF is v2" holds for settings, receipt keys, install order and `HASHED` (`realveh_entry.py:147-160,210-215`;
  tests `test_depth_validity.py:457-474,498-507`). Two expected differences: the receipt's `sha256.realveh_entry` value
  changes (the file changed; CAM-K2-E and v2 compare with HEAD, so no pin breaks), and a stray
  `NAVRL_DEPTH_VALIDITY_*` with the rule off is now refused (v2 ignored it). Mention both in the prereg's entry
  line (`:16-17`).

## Checks that passed (with evidence)

1. **Rule against the text.**
   - Mask source: `perception._depth_model_invalid` (`navrl_depth_validity.py:110`), set by the depth model on the same
     call (`navrl_depth_model.py:165`), = (z<MinZ | z>MaxZ | hole) & ~no_hit (`:154-162`).
   - Freshness: the depth model's call counter must equal last+1 (`validity:103-109`). The counter is incremented
     inside `apply` before the inner observe runs (`depth_model:182`).
   - Density: box count over the in-image area, stride 1, same padding, threshold `count >= density x inside - 1e-4`
     (`validity:87-93`). 13/25 is dense and 12/25 is not; corners need 5/9 (tests `:391-402`).
   - Range: `MinZ / perception._depth_model_cos` (`validity:121`), the depth model's own cosine (`depth_model:95-108`).
     Optical z = range x cos, consistent.
   - Pass-through: `torch.where(near, surface, depth)` (`:122`), so every other pixel is bit-identical (tests
     `:371,378,385-389`).
   - Order: the rule is installed before the depth model and the ablation, and refused after either or twice
     (`:158-164`). The entry order is validity -> depth model -> ablation (`realveh_entry.py:210-225`). Wrapping keeps the
     signature through `functools.wraps`/`__wrapped__` (test `:420-425`).
   - MinZ 0 is refused (`:118-119`).
   - The near blob reaches the fused scan through the real `NavRLPerceptionModule` (test `:427-433`: v2 12 m no return
     -> rule about 0.40 m; a valid 0.5 m bar is unchanged).
2. **No simulator-only or target information.** Inputs are the depth-model mask (from rendered depth and the
   depth-model RNG), the intrinsics' cosine and MinZ. There is no segmentation, target state or RNG draw (test
   `:435-441`). Renderer no-hit pixels are excluded by the depth model. Nothing about the target enters the actor
   observation (but see m-3 for the detector input).
3. **Wrapper and launcher.** Against v2, the diff is limited to the v3 constants, `DV_MOD`, the receipt checks
   (`:188-192,207-208`), the validity telemetry (`:429-450`), the diagnostic (`:453-467`), the tag and log glob, and
   `CAMK2V3_*`. The launcher adds three exports (`train_navrl_camk2v3.sh:140-143`).
   - Source SHAs and seeds are unchanged from v2 (4911/4912, 4921/4922).
   - The run tag matches `prereg_v3:85`.
   - The approval binds SCOPE + HEAD (`:154-165`).
   - The smoke gate requires SMOKE_OK at the same HEAD (`:521-529`).
   - Timing: the v2 smoke's last step was 4.18 s, so 1,000 x 4.18 x 1.25 = 5,230 s, under the 9,000 s cap. The v2
     seeds ran 4,816 s and 4,639 s against the 2.5 h cap, so no normal-run refusal (the rule adds two `avg_pool2d` per
     step).
   - The receipt env values ("5", "0.5") equal the launcher's strings. The receipt settings and install-line settings
     equal `DEPTH_VALIDITY` after the JSON round trip.
   - The barprobe regex matches the runner's format (test `:276-284`) and the real v2 lines (33 and 28 windows, so a
     seed will not read DIAG_NOT_RECORDED). The diagnostic does not enter the verdict (`:678`, test `:286-292`).
4. **Tests.** Every refusal is paired with a positive control (for example `test_depth_validity.py:404-418,476-484`,
   `test_camk2v3_train.py:243-259`). The import guard shows that only the entry imports the rule (`:523-525`). The one gap
   is M-1.
5. **Prereg text.**
   - It claims no performance result (`:10`).
   - The probe numbers match `probe/REPORT.md`: 50/50 vs 26/50; 49 frames with about 6,1xx holes, 0 near, 0 token
     differences; minimum token range 0.40 vs 0.02 m.
   - The CAM-K2-E numbers match the evidence README: −3.80 / −4.74 pp, interaction −0.15 / −0.69 pp.
   - The too-far / textureless sim-to-real limit is disclosed as NOT_TESTED (`:140-144`).
   - The labels DIAG_OK / DIAG_LOW / DIAG_NOT_RECORDED and the 4 dp references are exact.

## Verdict

The rule matches its text. Its inputs are sensor-only, and OFF reproduces v2. Before the commit: fix B-1 (one-line test
fix plus a clean PUBLIC_CPU rerun), M-1 (a non-vacuous summary check plus a test) and M-2 (a disk plan), and amend the
text for m-1 to m-4. These are small changes. A focused re-review of the diff is enough.

VERDICT: AMEND_FIRST
