# Independent results review r1: camera-only study, C2 (zero-shot sensor-removal conditions)

- **Reviewer:** Claude Code subagent, fresh context. **Model:** Opus 5.5 (`claude-opus-5-5`).
- **Date:** 2026-11-04, about 22:33-23:10 KST.
- **Brief:** `review_results/REQUEST.md`.
- **What I audited:** worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, run commit
  `5522be0`.
  - Criteria: `prereg_camonly_c1c2_v1.md`, its amendment 1, the study plan and the C0/C1C2 evidence.
  - Code: the ablation module, entry point, wrapper, launcher copy, `camonly_run.py`, `camonly_c2_analyse.py`, plus
    `gr_run.py`, `gr_analyse.py` and `kroc2027_eval.py`.
  - Raw: `results/camonly_c1c2_v1/{c1,c1_r2,c2}` and `results/gain_randomization_gr0_v1/gr0_main`.
- **Mode:** read-only on every repository and every `results/` folder.
  - I wrote only in `review_results/` (this file and `scratch/`).
  - All Python ran on the CPU with the brief's prefix (`taskset -c 6,7,12,13 nice -n 10`, `-B -I`), and no GPU.
  - My scripts import no repository module and read the raw files directly.
  - I did not read any `*env*.txt` file.
- **Tree state:** clean (0 lines) at the start and at the end.
  - HEAD moved during the review from `5522be0` to `a7d368b` (22:35, the author's sensor-check docs commit).
  - That commit touches no file under `docs/plans`, `tools` or `aerial_gym` (`git diff --stat 5522be0 a7d368b`).

## Verdict: **PASS_WITH_LIMITS**

- Every binding the preregistration and amendment 1 require holds, checked from the raw files. None is vacuous.
- My independent recomputation reproduces the author's output exactly, to the last digit (t-intervals to 5e-7 pp,
  from the rounded t constant):
  - all 72 per-block quota rates;
  - all 8 contrasts x 2 metrics: means, bootstrap CIs, t-intervals and labels;
  - the pooled rates, crash causes and governor telemetry.
- The labels follow the amendment's rule.
- No host fault, wrong commit, environment drift, dead condition, pairing error or OOD mislabel was found.
- The limits are listed under the findings: one MAJOR (protocol deviation, disclosure required) and six MINOR
  (wording and record). The readings may be recorded as preregistered, REPORTED, with the wording in
  "Claim scope".

## 1. Bindings, checked from raw (`scratch/rederive.py` -> `scratch/rederive_out.json`)

| Binding (source) | Raw check | Result | Vacuous? |
| --- | --- | --- | --- |
| C1_IDENTITY_PASS on the C2 commit (prereg :75) | recomputed: `c1_r2/full` result.json (minus nonce and checkpoint) equals GR-0 205/4501; `episodes.jsonl` equal **byte for byte**; `no_lidar` differs; both receipts on `5522be0`, dirty 0 | holds | no: `no_lidar` fails equality |
| C2 COMPLETE (:76) | 60 records, all OK, exit 0, no problems; 60 unique cells; order density -> seed -> condition; every cell 2,048 quota rows = 128 env x 16 | holds | no |
| Quota rows are the first 16 per env (my addition) | `in_quota` iff `env_episode_index` < 16 in all 72 cells (`scratch/quota_index.py`) | holds | no outcome-dependent selection |
| Receipt and snapshot name the condition and module SHA (:77; runner :51-70) | 60/60: receipt condition = folder; module SHA = git blob at `842fcf5` **and** `5522be0` (`ea9ccacb…`); module, perception, task and script paths inside the tree; 72 x 4; snapshot `NAVRL_SENSOR_ABLATION` = folder; `NAVRL_SENSOR_ABLATION_RECEIPT` and `NAVRL_BULK_EVAL_JSON` point to the cell's own files; `cell.log` carries the `[camonly] {"condition": …}` install line | holds | no |
| Grid complete (:78) | 12 blocks x 5 conditions; GR-0 S00 = the 12 preregistered blocks, all OK | holds | no |
| Each C2 cell differs from `full` (:79) | 60/60; all 15 condition pairs differ in 12/12 blocks | holds | no: episodes carry no nonce (C1 `full` is byte-identical) |
| L1: GR-0 COMPLETE and the manifest | GR-0 COMPLETE (`c56be07`); **85/85** S00 files plus the run report match the manifest; the disk list equals the manifest list; the camonly and gainrand copies of the manifest are identical (`3581940e…`) | holds | no |
| L1: same `--gr0-dir` | C1 r2 = C2 = `gr0_main` | holds | no |
| M3: recorded condition equals GR-0's | 60/60 `result.json` conditions minus the nonce are equal; this also checks the pairing (bars and seed are in the dict) | holds | no |
| M3: the snapshot differs only in the 5 keys | 60/60 differ in **exactly** the 5 expected keys; no `NAVRL_GAIN_RAND*`; backend `direct_l2_v1`; run id `CAMONLY-C2` | holds | no |
| M3: same allowlisted environment in C1 and C2 | identical and non-empty (15 keys); C2 started 0.15 s after C1 r2 ended | holds | weak by construction (one parent shell); C1's identity bridges it |
| M2: governor engaged | B_in != B_in_gL and C != C_gL in 12/12 blocks. Telemetry agrees: B_out/C intervention 0.85, clearance 1.66 m, executed speed 2.17 m/s (K2 floor) against C_gL 0.41 / 6.91 m / 2.81 m/s and A 0.28 / 7.99 m / 2.95 m/s | holds | no |
| L9: C1 `no_lidar` equals C2 | equal (and C1 at `1908fd8` equals C1 r2 for both cells) | holds | no |
| Host | fault count 0 at every check (C1 and C2 start and end, after each of the 62 cells, and `journalctl` now); one boot id; online `0-3,6-23` throughout | holds | |

## 2. Re-derivation (12 blocks, pp; my bootstrap replica / my independent numpy bootstrap with 100,000 resamples / t, df 11)

| Contrast | Capture mean [boot] | Capture t | Crash mean [boot] | Labels (capture / crash) | Match |
| --- | --- | --- | --- | --- | --- |
| total (OOD) | −39.95 [−45.68, −34.13] | [−46.77, −33.14] | +39.95 [33.95, 45.52] | LOWER / HIGHER | exact |
| in_fov | +0.69 [0.35, 1.06] | [0.26, 1.11] | −0.69 [−1.07, −0.33] | SMALL_HIGHER / SMALL_LOWER | exact |
| out_fov (OOD) | −39.26 [−44.59, −33.74] | [−45.79, −32.73] | +39.25 [33.65, 44.71] | LOWER / HIGHER | exact |
| interaction (OOD) | −1.38 [−2.51, −0.27] | [−2.70, −0.07] | +1.39 [0.28, 2.52] | SMALL_LOWER / SMALL_HIGHER | exact |
| obs_in | +0.16 [−0.12, 0.46] | [−0.18, 0.50] | −0.16 [−0.46, 0.11] | NEGLIGIBLE / NEGLIGIBLE | exact |
| gov_in | +0.53 [0.15, 0.96] | [0.06, 1.00] | −0.53 [−0.96, −0.16] | SMALL_HIGHER / SMALL_LOWER | exact |
| gov_all (OOD) | −5.05 [−5.83, −4.16] | [−6.04, −4.06] | +5.05 [4.15, 5.83] | LOWER / HIGHER | exact |
| obs_all (OOD) | −34.90 [−40.66, −29.00] | [−41.80, −28.01] | +34.90 [28.88, 40.62] | LOWER / HIGHER | exact |

- Pooled capture / crash, % (exact match):
  - A 95.54 / 4.46; B_in 96.23 / 3.76; B_out 56.28 / 43.71;
  - C 55.59 / 44.40; C_gL 60.64 / 39.36; B_in_gL 95.70 / 4.30.
- Identities: total = obs_all + gov_all, and in_fov = obs_in + gov_in, both hold exactly.
- The independent bootstrap (other RNG, 100,000 resamples) gives the same 16 labels.
- Crash causes (aggregate window) and governor telemetry match the author's output exactly.

## 3. Labels

`label()` (`tools/camonly_c2_analyse.py:74-82`) implements the amendment's table (amendment :24-30):
- the bootstrap/t side agreement comes first;
- then |mean| >= band;
- then both intervals inside the open band.

The bands are 3 pp for capture and 2 pp for crash. My own implementation gives the same 16 labels. No UNRESOLVED or
NO_DETECTED_CHANGE occurs. The `ood_terms` match the prereg's OOD set {B_out, C, C_gL}.

## 4. Findings

### BLOCKER

None.

### MAJOR

**M-1. Concurrent CPU work ran during C2, against AGENTS.md:194 and docs/OPERATIONS.md:126.**
- What ran: the author's Warp CPU sensor check, from about 20:35:48 to 20:37:46 KST.
  - Evidence: the mtimes in `~/motar_tmp/camonly_sensor_20261104/`.
  - WORKLOG at `a7d368b`: "run on the CPU while C2 held the GPU".
- Cells it overlapped (`c2.log` lines 171 and 181):
  - C 115/4501, 20:33:46-20:36:37;
  - C_gL 115/4501, 20:36:37-20:39:44. At 186 s this is the slowest C_gL cell; the others took 104-161 s.
- What is not affected:
  - No check voids this: it is not a host fault.
  - The tree freeze held. The files were written outside the tree and copied in at 22:33, after C2 ended at 22:30:58.
  - The CPU work was not disclosed as a deviation.
- Evidence against an effect:
  - The simulation is deterministic: C1 is byte-identical on two commits, and the L9 identity holds late in the run.
  - Both cells sit inside their seed siblings: C captures 1,220 / 1,239 / 1,217; C_gL 1,373 / 1,321 / 1,318.
  - Dropping block (115, 4501) keeps the labels of total, out_fov, in_fov, obs_in, gov_all and obs_all
    (`scratch/sens_drop_block.out`).
- **Required:** record it in the results commit as a protocol deviation, with these times and this sensitivity.

### MINOR

- **m-1. `gov_all` also contains the K2 rule, and the amendment does not say so.**
  - Amendment L3 (:71) names `out_fov`, `total` and `interaction`.
  - `gov_all` = C − C_gL, and C carries K2 while C_gL does not. So `gov_all` measures "the raw-LiDAR governor
    replaced by the camera governor with K2". It is not a governor-agnostic "value of the governor's LiDAR".
  - The telemetry shows this: K2 conditions run slower (2.17 vs 2.81 m/s) yet crash more. The mechanism is not
    identified.
- **m-2. The SMALL labels of `interaction` and `gov_in` are fragile.**
  - Their t-interval ends lie 0.06-0.07 pp from 0.
  - Leaving one block out changes them to UNRESOLVED in 7 of 12 and 4 of 12 cases (`scratch/loo.out`).
  - A Bonferroni correction over 8 contrasts gives p_adj of 0.33 and 0.25. `in_fov` gives 0.035 and holds in 12 of
    12 leave-outs.
  - The labels stand as preregistered (unadjusted; L7), but no conclusion may rest on them.
- **m-3. Two of the plan's reported items were dropped without a record.** "Crashes with the nearest bar inside vs
  outside the FOV" and "time to capture" (plan :169; design review L6) are neither computed nor declared. Record
  them as NOT_REPORTED.
- **m-4. The amendment's runner and analysis changes ran without an independent review.**
  - They went to the GPU 6 minutes after their commit (19:51:34 to 19:57:26), checked only by the author's
    `smoke_02` (14/14, on the committed SHAs).
  - This is allowed by the letter of prereg :144-146.
  - This review read them at `5522be0` and found no defect.
- **m-5. Provenance of the analysis output.**
  - `c2_analysis.json` exists only in `~/motar_tmp`, and it does not record the analysis script's SHA.
  - In the results commit, bind the analysis script (`f9d709ff…`, identical to `5522be0`) and the output
    (`f145389c…`).
- **m-6. The magnitudes are specific to this grid.**
  - The estimand is the equal-weight mean over 4 densities x 3 seeds.
  - The block SD of `total` is 10.7 pp, driven by density: −24.5 pp at 70 bars to −51.4 pp at 205.
  - Per-density values are descriptive only (not preregistered).

### Information

- `ablation_receipts_match` compares the receipt with the runner's self-reported SHA, not with git. I checked it
  against the git blobs: 60/60.
- Condition A comes from a different commit and tree (GR-0, `c56be07`).
  - In `navrl_task.py`, the only runtime difference from `c56be07` to `5522be0` is the removal of the
    `_gain_rand_condition()` recorder, which returns `{}` when OFF.
  - The identity is shown on 1 block of 12, and the M3 condition equality holds 60/60.
- The bootstrap shares one RNG stream across the contrasts, so the draws depend on their order. The independent
  bootstrap shows this does not matter.
- `torch.cdist` does not occur in the ablation, perception or governor files. TF32 effects are shared by every
  condition, and the 160-bar blocks show no anomaly.
- `REVIEW_design_r1.md` says it ran "19:27-20:10", but it was committed at 19:51:34. This is cosmetic.

## 5. Claim scope

Template (prereg :111-112): "With Policy F as trained, removing the LiDAR [where] changes capture by X pp [CI] and
crash by Y pp [CI] on this grid." It always carries "zero-shot, Frozen Policy F, simulation, E2 moving target, A3,
70-205 bars, seeds 4501-4503". OOD contrasts also carry "out of distribution".

**May be said:**
- `total` (zero-shot, OOD; includes K2): capture −39.95 pp [−45.68, −34.13], crash +39.95 pp [33.95, 45.52].
- `out_fov` (zero-shot, OOD; includes K2): −39.26 pp [−44.59, −33.74] / +39.25 pp.
- `obs_all` (zero-shot, OOD): −34.90 pp [−40.66, −29.00] / +34.90 pp. Disclose L1 (the previous action and the ego
  velocity carry the raw-LiDAR governor's effect) and L2.
- `gov_all` (zero-shot, OOD): "replacing the raw-LiDAR governor by the camera governor with K2, under camera-only
  perception": −5.05 pp [−5.83, −4.16] / +5.05 pp.
- `in_fov`: "a detected change below the 3 pp band, +0.69 pp [0.35, 1.06]". It contains the camera's misses and
  range, the in-FOV association, the governor's phantom immunity and the column offset.
- `obs_in`: NEGLIGIBLE, "no change larger than ±3 pp (capture) or ±2 pp (crash); CI [−0.12, 0.46]".
- The loss is "consistent with" F reading the K1 fill as free space. This is not tested.

**May not be said:**
- that the LiDAR, or the camera, is "necessary", "unnecessary" or "sufficient", or that "the camera alone suffices
  inside the FOV";
- that "removing the in-FOV LiDAR improves F", or anything resting on the `gov_in` or `interaction` labels (m-2);
- that `in_fov` or `obs_in` is "camera quality";
- "no effect" for anything except `obs_in`'s band statement;
- any share of the LiDAR's value split between perception and governor ("87 % perception"). The split total =
  obs_all + gov_all is one ordering, the fused-perception x camera-governor cell is not run (L4), and that
  interaction is not estimated;
- causal mechanisms ("crashes are caused by …"), or "the governor's LiDAR prevents 5 pp of crashes" without K2;
- anything about a camera-only policy trained for its sensor (C3-C4), real cameras, real flight, or other grids
  and densities;
- capture and crash as two independent pieces of evidence (timeouts are 0.004-0.012 %).

## Scratch files (`review_results/scratch/`)

| File | Content |
| --- | --- |
| `reports.py` | summary of the C1, C1 r2 and C2 run reports |
| `rederive.py`, `rederive_out.json` | bindings, rates, contrasts and the comparison with the author's output (SHA `85c1fb14…`, `d2cde590…`) |
| `quota_index.py` | the quota is the first 16 episodes per env, in all 72 cells |
| `sens_drop_block.py`, `.out` | M-1 sensitivity |
| `loo.py`, `loo.out` | leave-one-block-out robustness of the labels |
