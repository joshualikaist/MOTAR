# CAM-K2-E v1 — matched evaluation of CAM-K2 v2 under its treatments and nominally (preregistration, 2026-11-08)

**State:** preregistered 2026-11-08 at about 23:40 KST after design review r1 (AMEND_FIRST, repaired) and the round-2
check named in "Review"; NOT_RUN at commit. Throughout this file, "CAM-K2" means CAM-K2 v2 (`aa0e691`, trained
`04f341c`).

**CAM-K2 v1 is WITHDRAWN_BEFORE_RUN.** The user's real-vehicle specs arrived after v1's commit (`62e2699`) and before
any run. The user, relaying their senior, 2026-11-08 at about 16:10 KST: "드론은 무게 1.1kg, 프로펠러 5x4.3x3, 모터
velox 2550kV" (the drone weighs 1.1 kg, with 5x4.3x3 propellers and Velox 2550 kV motors).
- v1 modelled a 0.5 kg placeholder at T/W 2.25.
- CAM-K2 v2 models the real vehicle's T/W, 4.0 nominal (INFERRED; design review r1 m-2), as a raised per-motor limit on navrl_quad (draft
  `~/motar_tmp/camk2_20261108/PREREG_V2_DRAFT.md`; repository path
  `docs/plans/prereg_camonly_k2_train_v2_2026-11-08.md`).
- This evaluation reads v2 only:
  - the runner imports `tools/camk2v2_train.py`, not v1's wrapper;
  - it reads `~/motar_tmp/camk2v2_20261108/`;
  - it refuses a train report, manifest or treatment set of v1.

**Vehicle facts since the draft** (user, 2026-11-08 22:10 and 19:40 KST):
- 4S battery, and the 1.1 kg includes it. There is no thrust-stand data, so thrust stays predicted: about 3.8–4.0 in
  flight, inside v2's 3.4–4.6.
- The wheelbase is still unknown (about 225 mm assumed).

**Done before the commit:**
1. CAM-K2 v2 is committed and both seeds read TRAINED (`04f341c`).
2. The pins are filled from the TRAINED reports: `769a26f4…de04e8a`, `e76eb048…88321db6`.
3. `tests/test_camk2e.py` passes (27).
4. The controls were computed: CONTROLS_OK (table below).
5. The PUBLIC_CPU suite and the round-2 check are in the commit message and the WORKLOG.

**Found before launch** (CPU, exploratory, 2026-11-08 22:35–23:45; it is not CAM-K2-E data):
- **The depth probe** (`~/motar_tmp/camk2_depthprobe_20261108/probe_r1/`, by Codex). The independent review is
  `review_claude_r1/REVIEW.md` (Claude Opus 5.5), which read PASS_WITH_LIMITS.
  - v2's MinZ of 0.4 m and its "invalid → 20 m no return" convention erase face-on bars whose near face is closer
    than 0.4 m (the range corrected in review r2 R2-3). This happens when the centre distance is below half-width + 0.5 m, that is about 0.70–0.89 m for these
    bars.
  - The policy and, under `no_lidar`, the governor are therefore blind for the last ≈0.26 m of approach.
- **The contract audit** (`contract_r1/`) corrected three log readings:
  - `hit_fov` is 240° token-sector inclusion, not camera visibility;
  - `hit_dist` is the distance to the nearest bar centre;
  - the contact clearance is the previous step's requested-corridor clearance.
- **Consequences for this unit:**
  - It characterises the convention v2 was trained with, as preregistered; it is not a repaired sensor.
  - The training logs show no higher crash fraction in the last training window for v2 than for CAM-K1 (0.71 against
    0.73; unpaired). Whether the
    convention changes crashes is NOT_TESTED, and this evaluation measures it only as part of the bundled treatments.
  - The hit-token share is not used as a perception metric across K1 and K2.
  - `camk2e_analyse.py` has no parser for the runner's diagnostic lines (`barprobe v2`, `speedgov`). Any reading of
    them from `cell.log` is post hoc and descriptive, and is labelled so; otherwise they are NOT_RECORDED (review r2
    R2-3).
- **Gate A's label names** contain "ADAPTATION" (review r2 R2-2).
  - A gate A label alone is read as "lower crash under the treatments", never as adaptation.
  - The word "adaptation" is used only when `adaptation_supported` holds: gate A MATERIAL/SMALL and gate I
    INTERACTION_NEGATIVE, for both seeds.
- **Gate I** has only a synthetic test. Its interval width on real data is unmeasured, and it has no floor (review r2
  R2-5).
- **Launcher comment:** the copy's header comment (`eval_navrl_camk2e_arms.sh:7`) says T/W 4.2. The values are passed
  by the runner (4.0, receipts checked); the pinned file is left unchanged.

This is evaluation only. It claims nothing until the analysis has run.

- Training record: CAM-K2 v2 ([preregistration](prereg_camonly_k2_train_v2_2026-11-08.md), `aa0e691`; training record `04f341c`). CAM-K2 v1 ([preregistration](prereg_camonly_k2_train_v1_2026-11-08.md),
  `62e2699`) is WITHDRAWN_BEFORE_RUN and is not evaluated.
- Reference evaluation: CAM-K1-E v1 ([preregistration](prereg_camonly_k1_eval_v1_2026-11-08.md); results
  `ab2eeb5`, `docs/evidence/camonly_2026-11-04/camk1e/`, results review r1 PASS_WITH_LIMITS).
- **Approval** (review r1 E-m4). The runner refuses without `~/motar_tmp/camk2e_20261108/execution_approval.json`
  naming the runner's SCOPE and HEAD. A new approval for this scope is required. The file will cite:
  - the user's 12:07 KST approval, "예, 오늘 전부 (권장)", which named "실기체 모델 학습";
  - the user's 16:10 KST instruction after relaying the specs, "이에 맞춰서 데이터 다시 수집하면 될거같아".

  - and the scoped approval of this unit: the user's 2026-11-08 ~23:30 KST AskUserQuestion answer "v2 평가 먼저
    (권장)" (option text: "준비된 CAM-K2-E로 v2의 실제 평가 충돌률을 … 결재하시는 즉시 이 평가의 GPU 실행도
    승인하시는 것입니다"), and the message "오케이 v2평가부터 먼저 돌리자".
- **Commit order** (review r1 E-m5).
  1. CAM-K2 v2 is committed and trained first.
  2. CAM-K2-E's files (runner, analysis, launcher copy, test) stay out of the `MOTAR-camonly` tree until the K2 pins are
     filled after that training. The by-design failing pin test and the clean-tree preflight would otherwise block
     CAM-K2 v2's PUBLIC_CPU suite and run.
  3. Then CAM-K2-E is committed with the pins, its preregistration and its tests.

## Question

CAM-K2 continued the camera-only lineage (CAM-K1) for 1,000 epochs under three treatments:
- the real vehicle's T/W 4.0 (INFERRED), as navrl_quad's per-motor thrust limit raised to 4.0 × m × g / 4 = 2.4525 N
  (opt-in `NAVRL_VEH_RAND_LIMIT_MODE=raise`). That is the thrust-to-weight of a 1.1 kg vehicle at T/W 4.0; the
  rotational dynamics and drag stay navrl_quad's;
- motor randomization: thrust ±15 % per env, ±5 % per motor, τ 0.02–0.04 s;
- the OAK-D Pro depth model.

Three questions:
1. Under those treatments, does CAM-K2 crash less than CAM-K1? This is gate A, **lower crash under the treatments**.
   K2 is also 1,000 epochs further trained, so gate A alone does not identify adaptation (review r1 E-M1).
2. With the treatments off, does CAM-K2 crash materially more than CAM-K1? This is gate N, reported with its label.
   "Nominal" means **F's vehicle** (review r1 E-m1): navrl_quad's own T/W 3.26, τ 0.04 s and exact depth. That is a
   weaker vehicle than the trained one (T/W 4.0) and the slow end of the trained τ, so gate N mixes nominal loss with
   the vehicle change.
3. Is K2's gain larger under the treatments than nominally? This is gate I, the **interaction**. The word
   "adaptation" needs gate A and gate I together.

## Arms

All arms run Policy F's evaluation contract on GR-0's S00 grid, the condition of C2 and CAM-K1-E:
- E2 moving target, A3 placement;
- 70/115/160/205 bars × seeds 4501–4503;
- quota 16 episodes per environment, 128 environments, nominal gains;
- `no_lidar` at the 66° × 54° camera, with CAM-K1's reward weights 0.6 and 0.03 (they enter the reward only).

| Key | Policy | Treatments | Blocks | Source |
| --- | --- | --- | --- | --- |
| K1s1, K1s2 | CAM-K1 seeds 1, 2 (`d7bf51a8…`, `5128bf2e…`, pinned in `camk1e_run.py`) | **ON** | 12 | new, 24 cells |
| K2s1, K2s2 | CAM-K2 seeds 1, 2 (SHA pinned from their TRAINED train reports and sidecar manifests at commit time) | **ON** | 12 | new, 24 cells |
| K2s1_nom, K2s2_nom | CAM-K2 seeds 1, 2 | OFF | 6 (70 and 205 bars × 3 seeds) | new, 12 cells |
| K1E_K1s1, K1E_K1s2 | CAM-K1 seeds 1, 2 | none (CAM-K1-E) | 12 | CAM-K1-E's eval cells (existing; admitted through the identity stage) |
| K1E_F66 | camera-only Policy F at 66° | none | 12 | CAM-K1-E's eval cells, controls only |

- **"Treatments ON"** is exactly CAM-K2's training treatment, through `tools/realveh_python` (CAM-K2's committed entry
  point):
  - `NAVRL_VEH_RAND=1` with TW_NOMINAL 4.0 (`NAVRL_VEH_RAND_LIMIT_MODE=raise`), TW_SCALE 0.15, MOTOR_IMBAL 0.05, τ 0.02–0.04 s;
  - `NAVRL_DEPTH_MODEL=oakd` with MinZ 0.4 m, MaxZ 20 m, σ_d 0.15 px, f·B 66.1875 px·m and holes 0.02.
- **"Treatments OFF"** runs the same entry with both switches off: only the camera-only ablation and CAM-K1's
  overrides, which is the camtrain path of CAM-K1-E.
- **Module seeds, matched:** for evaluation seed e, vehicle randomization uses seed 10e + 1 and the depth model
  10e + 2 (45011/45012, 45021/45022, 45031/45032). They are identical in every arm of a block, and they differ from
  CAM-K2 v2's training seeds (4911/4912, 4921/4922).
- **Common random numbers hold only at the start of a cell.** Each module draws again at every reset and every frame,
  and the reset sequence depends on the policy's own terminations. So the realised draws diverge between arms after
  the first terminations. The matching removes seed-choice differences between arms, not draw-order differences.

## Runner and launcher

- **Runner:** `tools/camk2e_run.py`. It reuses `tools/camk1e_run.py` (approval, CAM-K1 checkpoints, camera values),
  `tools/gr_run.py`'s cell environment, checks and launcher call, and `tools/camk2v2_train.py` (CAM-K2 v2's values, the
  telemetry parser, the realveh entry's hashed-file list). It does not import v1's `tools/camk2_train.py`.
- **Launcher:** `aerial_gym/rl_training/rl_games/eval_navrl_camk2e_arms.sh`, SHA-pinned in the runner. It is CAM-K1-E's
  launcher copy with five changes, all marked `[camk2e]`:
  1. `CAMEVAL_PY` is `tools/realveh_python`, and the real interpreter goes to `REALVEH_REAL_PYTHON`;
  2. `CAMK2E_TREATMENTS` (on/off) is required and checked against the switches;
  3. the checkpoint and its SHA come as `CAMK2E_CKPT` / `CAMK2E_CKPT_SHA`;
  4. each cell gets its own `NAVRL_REALVEH_RECEIPT`;
  5. no per-cell `NAVRL_CAMTRAIN_RECEIPT`.

  A test reconstructs CAM-K1-E's launcher byte-exactly from it.
- **Cell checks.** A cell is valid only if it passes all of these:
  - GR-0's checks: receipt OK on this commit with a clean tree, the arm's checkpoint SHA, no `NAVRL_GAIN_RAND*`, the
    distance backend, the ledger quota, import origins inside the tree, no recorder violations, host unchanged;
  - **the realveh receipt:**
    - the settings are exactly the cell's: CAM-K1's four values, and with the treatments ON CAM-K2's values with the
      cell's module seeds, or none when OFF;
    - worst-case T/W ≥ 1.8 when ON;
    - task robot navrl_quad;
    - `no_lidar` on a 72 × 4 LiDAR, with absolute in-tree task and perception paths;
    - CAM-K1's overrides;
    - the committed SHA-256 of the entry's 15 hashed files;
    - install records matching the treatment mode;
  - **the cell receipt's NAVRL snapshot** names:
    - the condition and the camera values;
    - exactly the treatment keys of its mode, with the cell's values;
    - the cell's realveh receipt;
    - no camtrain receipt, no other robot and no retired `NAVRL_REALVEH_*` variable;
  - the recorder camera is 66° × 54°, and the cell log has `hfov=66deg`;
  - **treatments ON:**
    - the modules' telemetry passes CAM-K2's post-run rules: install lines with the cell's settings, an init line
      with the cap applied and factors spanning more than 0.05 inside 0.8075–1.2075, reset lines inside the bounds,
      a motor-update count > 0, and the depth model's first and 50-frame summary lines with a hole share in
      0.02 × (1 ± 0.5), MinZ and MaxZ shares in [0, 0.5), mean |noise| in (0, 1) m and σ = k·z²;
    - the init line has mode "raise", and its reset lines have τ inside [0.02, 0.04] s;
    - the vehicle-randomization record shows all of these:
      - mode "raise";
      - the limit 4.0 × m × g / 4 from the controller's mass, equal to `thrust_limit_per_motor_n` and above 2 N;
      - the live read-back of `max_thrust` equal to the limit (review r1 M-1);
    - every reset and update line carries the raise counters and a live `max_thrust` equal to the limit, as in CAM-K2
      v2's post-run check. The shares of commands above 2 N and at the limit are reported per treated cell
      (descriptive);
      - navrl_quad's own 4 × 2 N, with its own T/W equal to 8 / (m g);
  - **treatments OFF:** no telemetry line and no record.
- **Preflight refusals:**
  - the approval does not name SCOPE and HEAD, or the commit argument is shorter than 7 digits;
  - less than 1 GB free disk;
  - not branch `camonly/dev`, or a dirty tree;
  - not committed: this file, the launcher, the three modules, the entry, the wrapper, the runner, the analysis or
    `camk2v2_train.py`;
  - the launcher not at its pinned SHA;
  - a CAM-K1 checkpoint not from its pinned TRAINED report;
  - a CAM-K2 checkpoint not from a TRAINED train report and a TRAINED sidecar manifest at its pinned SHA, or a pin
    still a placeholder;
  - a train report whose preregistration or scope is not CAM-K2 v2's, or a manifest whose unit is not "CAM-K2 v2",
    whose preregistration is not v2's, or whose treatments are not v2's;
  - not exactly one RTX 3070, or another GPU compute process except the remote desktop;
  - import origins outside the tree;
  - no COMPLETE CAM-K1-E eval folder;
  - an existing output folder.
- **Host faults:** a rise in the kernel fault count voids the run.
- **Outputs:** `~/motar_tmp/camk2e_20261108/raw/identity_01/` and `.../raw/eval_01/`.

## Stage 1: identity, negative control, ON repeat and a CAM-K2 smoke (4 cells, 205 bars, seed 4501)

| Cell | Requirement |
| --- | --- |
| `K1s1_off_identity`: CAM-K1 seed 1, treatments OFF, through this launcher and the realveh entry | `result.json` (minus the per-run nonce and the checkpoint path) and `episodes.jsonl` equal CAM-K1-E's eval K1s1 cell exactly |
| `K1s1_on_negative`: CAM-K1 seed 1, treatments ON | `episodes.jsonl` differs from that cell: the treatments take effect in the simulator |
| `K1s1_on_repeat`: the same ON cell again (review r1 E-L2) | `episodes.jsonl` equals `K1s1_on_negative`'s exactly: the treatments are deterministic |
| `K2s1_smoke`: CAM-K2 seed 1, treatments ON | valid, and its `episodes.jsonl` differs from `K1s1_on_negative`'s: the CAM-K2 checkpoint path takes effect |

- **Reading:** IDENTITY_PASS if all four hold; IDENTITY_FAIL otherwise, and stage 2 does not run.
- **What a pass admits:** CAM-K1-E's K1 cells as the nominal reference for gate N and the reported cost, without
  rerunning them. That the realveh entry with both switches off equals the camtrain entry is shown here.
- This stage is the GPU smoke of every changed runner path: the launcher copy, the entry in both modes, and a CAM-K1
  and a CAM-K2 checkpoint.

## Stage 2: eval (60 cells)

- **Order:** density, then seed, then arm: K1s1, K1s2, K2s1, K2s2, and at 70 and 205 bars also K2s1_nom and
  K2s2_nom.
- **Needs:**
  - exactly one identity report in the same parent folder, an IDENTITY_PASS on the same commit, and no earlier eval
    there;
  - the same allowlisted runner environment and the same CAM-K1-E folder.
- **Budget, per stage** (review r2 R2-1 / r3): identity 4 cells × about 3 min; eval 60 cells, projected 60 × 158 s ×
  the identity ratio (about 2.6–3.2 h).
  - The cap is fixed at 3.5 h per stage; any other `--max-hours` refuses. It is checked between cells, so a stage may
    overrun by up to one cell (a 30-minute timeout).
  - **Rate gate** (review r1 E-m2, repaired after review r2 R2-1): the identity cells all run at 205 bars, the slowest
    density. So the overhead is measured as the ratio of their mean time to CAM-K1-E's identity cells (the same cells).
    The eval refuses unless 60 × CAM-K1-E's mean eval cell time (158 s) × that ratio × 1.10 ≤ 3.5 h. The test runs
    realistic 175 s cells (pass) and 300 s cells (refuse). The depth model's cost on evaluation is first measured
    there.
- **Free disk:** at least 1 GB at preflight. CAM-K1-E wrote 294 MB for 39 cells, so about 0.5 GB is expected.
- **Reruns:** a failed identity, or an INCOMPLETE or VOID eval, stays recorded in its folder. A rerun needs a new
  parent folder and a dated amendment.
- **The tree stays frozen** from the commit until the analysis has run.

## Analysis (fixed): `tools/camk2e_analyse.py`

**Bindings first.** Any failure reads EVAL_BINDING_FAIL:
- IDENTITY_PASS on the eval commit, and it is the identity report the eval named;
- eval COMPLETE with 60 cells, in the planned order;
- one runner environment and CAM-K1-E folder in both stages;
- every eval cell passes the runner's cell checks again (`camk2e_run.receipt_problems`, with the run report's
  committed SHA-256), and its checkpoint SHA is the run report's;
- every eval cell's result condition equals CAM-K1-E's K1s cell of the same block;
- its NAVRL snapshot differs from that cell's only in the expected keys: the nonce, paths and run id, the checkpoint
  SHA, the entry's receipt variable, and the `NAVRL_VEH_RAND*` / `NAVRL_DEPTH_*` treatment keys;
- complete grids: 12 blocks for each main arm, 6 for each nominal arm;
- **in every block:**
  - K2s differs from K1s;
  - K1s under the treatments differs from CAM-K1-E's K1s;
  - K2s_nom differs from CAM-K1-E's K1s;
  - K2s1 differs from K2s2;
- the identity stage's `K1s1_on_negative` and `K2s1_smoke` cells equal the eval's K1s1 and K2s1 cells (205 bars,
  seed 4501);
- CAM-K1-E's cells match its committed manifest's SHA-256;
- the CAM-K2 checkpoints come from TRAINED train reports and TRAINED sidecar manifests at the pinned SHA;
- the analysis script's bytes equal its bytes at the eval commit.

**Quantities.**
- Rates are quota rates per block. The blocks are (density, seed) pairs: 12 for the main contrasts, and 6 for gate N
  (70 and 205 bars × 3 seeds).
- **Intervals:** stratified over seeds within density, as CAM-K1-E:
  - a percentile bootstrap (10,000 resamples, seed 20,261,108);
  - a t-interval with df = Σ(seeds − 1): **8 df over 12 blocks, 4 df over gate N's 6 blocks**.
  - Gate N's intervals rest on 2 densities and are wider for that reason.
  - The intervals describe these blocks; they do not generalise to other densities.
- Contrasts are in pp.

**Gates,** on the crash rate, per seed pair s (s = 1, 2). A label needs both intervals to agree.

| Gate | Contrast | Labels, in order |
| --- | --- | --- |
| **A** (adaptation, gated) | K2s − K1s, both under the treatments, 12 blocks | **UNRESOLVED**: the intervals disagree on whether 0 is excluded, or on the side. **MATERIAL_ADAPTATION**: both upper ends < −5 pp. **SMALL_ADAPTATION**: both upper ends < 0 and the mean ≤ −2 pp. **WORSE**: both lower ends > 0 and the mean ≥ +2 pp. **NEGLIGIBLE**: both intervals inside (−2, +2) pp. **NO_DETECTED_ADAPTATION**: otherwise. |
| **N** (no material loss on F's vehicle, reported with its label) | K2s_nom − CAM-K1-E's K1s, 6 blocks, df 4 | **NO_MATERIAL_LOSS**: both upper ends < +3 pp. **MATERIAL_LOSS**: both lower ends > +3 pp. **LOSS_INCONCLUSIVE**: otherwise. |

| **I** (interaction, reported with its label; review r1 E-M1) | (K2s − K1s)_ON − (K2s_nom − CAM-K1-E's K1s)_OFF on the 6 nominal blocks, df 4; weights {K2s +1, K1s −1, K2s_nom −1, K1E_K1s +1} | **INTERACTION_UNRESOLVED**: the intervals disagree. **INTERACTION_NEGATIVE**: both upper ends < 0. **INTERACTION_POSITIVE**: both lower ends > 0. **INTERACTION_NOT_DETECTED**: otherwise. |

- **Timeouts.** A crash reduction with timeouts up by 2 pp or more reads **CRASH_TO_TIMEOUT** in place of
  MATERIAL_ADAPTATION or SMALL_ADAPTATION, as in CAM-K1-E.
- **The word "adaptation"** is allowed for seed s only when gate A reads MATERIAL_ADAPTATION or SMALL_ADAPTATION
  **and** gate I reads INTERACTION_NEGATIVE (`adaptation_supported`). Otherwise gate A reads "lower crash under the
  treatments", which continued training alone could explain. The interaction's OFF arm is F's vehicle, so it compares
  "under the treatments" with "on F's vehicle", not with a neutral nominal.
- **Lineage reading per gate:** the label both seeds share, else **SEEDS_DISAGREE**.
- **Why these bands.**
  - Gate A's 5 pp is half of CAM-K1-E's recovery band. The treatments' cost to CAM-K1 is not known before this run,
    so a smaller adaptation band is meaningful.
  - The 2 pp floor keeps negligible differences out of SMALL_ADAPTATION and WORSE.
  - Gate N's 3 pp is CAM-K1-E's gate P band.
- **Controls on existing data (CAM-K1-E's cells).** Each control uses its own generator seeded 20,261,108, so the
  `controls` mode and the full analysis give the same numbers. Every control must read as required; otherwise the
  reading is CONTROL_FAIL and no gate label is reported.

  | Control | Required | Computed with the script before the commit |
  | --- | --- | --- |
  | Control | Required | Computed 2026-11-08 (`controls_r1_20261108.json`) |
  | --- | --- | --- |
  | gate A on CAM-K1-E's F66 − K1s1 (a known-bad "adaptation") | WORSE | WORSE: +37.67 pp, bootstrap [36.78, 38.45], t [36.45, 38.89] |
  | gate A on CAM-K1-E's K1s1 − K1s2 (two CAM-K1 seeds) | not MATERIAL_ADAPTATION | NEGLIGIBLE: −0.92 pp, bootstrap [−1.51, −0.35], t [−1.78, −0.07] |
  | gate A on CAM-K1-E's K1s1 − F66 (review r1 E-M2: MATERIAL_ADAPTATION reachable) | MATERIAL_ADAPTATION | MATERIAL_ADAPTATION: −37.67 pp, bootstrap [−38.45, −36.78], t [−38.89, −36.45] |
  | gate N on CAM-K1-E's K1s2 − K1s1, 6 nominal blocks (NO_MATERIAL_LOSS reachable at df 4) | NO_MATERIAL_LOSS | NO_MATERIAL_LOSS: +0.88 pp, bootstrap [0.11, 1.73], t [−0.56, 2.32] |
  | gate N on CAM-K1-E's F66 − K1s1, 6 nominal blocks | MATERIAL_LOSS | MATERIAL_LOSS: +35.53 pp, bootstrap [34.62, 36.39], t [33.96, 37.11] |

  The reading is **CONTROLS_OK**. These controls use no CAM-K2-E data. They were computed by
  `tools/camk2e_analyse.py controls`, under the required CPU command form, after S4F-1 ended.
  - **Resolution.** The null controls' half-widths are about 0.58 pp (bootstrap) and 0.86 pp (t) at 12 blocks, and
    0.81 pp and 1.44 pp at 6 blocks.
  - **Disclosed gap:** no claim-scale (about 5 pp) known-bad contrast exists in the current data.
  - **Disclosed coarseness** (review r1 E-L3): with 3 seeds per stratum, the stratified bootstrap has at most 10 distinct
    resample means per stratum, so its interval ends are coarse. The t-interval carries gate N.

**Reported, not gated:**
- the treatments' cost to CAM-K1: K1s under the treatments − CAM-K1-E's K1s, 12 blocks;
- the seed spreads: K2s1 − K2s2 and K1s1 − K1s2 under the treatments; K2s1_nom − K2s2_nom on 6 blocks;
- the same contrasts for capture and timeout (capture labelled with C2's scheme, band 3 pp);
- pooled and per-density rates;
- crash causes, from `result.json`'s legacy window: all actual episodes, not the quota rows;
- the speed governor's telemetry.

## What may be said, and what may not

- **May be said** (if bound and the controls pass): "On this grid, in simulation, under CAM-K2 v2's treatments
  (navrl_quad's per-motor thrust limit raised to T/W 4.0, the thrust-to-weight of a 1.1 kg vehicle at an INFERRED
  T/W 4.0 with navrl_quad's rotational dynamics and drag; ±15 % / ±5 % thrust randomization, τ 0.02–0.04 s; and the
  OAK-D Pro depth model as configured), CAM-K2 v2 seed s changes the crash rate by X pp [CI; t CI] against CAM-K1 seed
  s (gate A: label). On F's vehicle (T/W 3.26, τ 0.04 s, exact depth), it changes the crash rate by Y pp [CI; t CI,
  4 df, 70 and 205 bars] against CAM-K1 seed s (gate N: label). The interaction is Z pp (gate I: label)."
  - "Adaptation" is said only where `adaptation_supported` holds for both seeds (gate A MATERIAL or SMALL, and gate I
    INTERACTION_NEGATIVE), and always as adaptation to these simulated treatments. Otherwise: "lower crash under the
    treatments".
  - Gate N's NO_MATERIAL_LOSS reads "within 3 pp of CAM-K1 on F's vehicle, at 70 and 205 bars on this grid".
  - Both seeds' labels are reported separately. A shared label may be said to be shared.
- **May not be said:**
  - anything about the real vehicle, the real OAK-D Pro, real images or real flight. The treatments are placeholders;
  - that CAM-K2 is robust to mass, motor or depth errors in general. Only these ranges and models were evaluated;
  - that one treatment caused the result. The three are bundled, and gate A measures CAM-K2's whole recipe under
    them: 1,000 more epochs, trained with all three on;
  - that more training would or would not help (not tested);
  - an effect from one seed alone, or a pooled interval for "the CAM-K2 recipe". The intervals are over evaluation
    blocks for one checkpoint each;
  - any comparison with training-rollout rates. CAM-K2's training rollouts ran under PPO exploration noise and the
    training target, without the A3 placement;
  - that CAM-K2 approaches, matches or is equivalent to F with camera + LiDAR. This unit has no such arm, and
    CAM-K1-E's gate P read MATERIAL_LOSS for the source lineage;
  - generalisation beyond these four densities, or for gate N beyond 70 and 205 bars;
  - derived ratios such as a "share of the treatments' cost recovered", unless from a machine-readable record and
    labelled exploratory.

## Disclosed limits

- **Reference lineage.** CAM-K1 is the reference. CAM-K1-E found it at +11.8 / +12.7 pp crash against F with camera
  + LiDAR at 87° (gate P MATERIAL_LOSS), and −37.7 / −36.7 pp against camera-only F at 66° (gate R
  MATERIAL_RECOVERY). An adaptation here is relative to that lineage.
- **The treatments are placeholders.**
  - The vehicle's T/W 4.0 is INFERRED from the user's specs and one bench point (battery, wheelbase and thrust data
    are open questions). Only its thrust-to-weight is matched. navrl_quad's roll acceleration at the limit
    (755 rad/s²) is about 1.1–2.6× the estimate for a 1.1 kg 5-inch frame (286–687 rad/s²), its drag per unit mass is
    navrl_quad's, and the 45° tilt limit bounds lateral force.
    See CAM-K2 v2's "Equivalence statement".
  - Thrust scaling stands in for mass.
  - The attitude loop is Lee, not PX4.
  - The depth model's noise is iid, its invalid-depth encoding is optimistic and its constants are nominal.
  - The detector is F's colour rule.
  - See CAM-K2's preregistration, "Disclosed limits", which all apply here, including the altitude sag at episode
    start in weak envs: CAM-K2-E reads `below` crashes in that light.
- **Seeds.** Two training seeds. The intervals are over evaluation blocks within fixed densities and exclude
  training-seed variation; CAM-K1-E's K1s1 − K1s2 was −0.92 pp with an interval excluding 0.
- **Matched module seeds** remove seed-choice differences only. The draw sequences diverge with each policy's resets.
- **Gate N's scope:** 6 blocks at 2 densities, df 4. Its nominal reference cells come from CAM-K1-E (`971ae67`, the
  camtrain entry), admitted through stage 1's identity.
- **Camera model** (CAM-K1-E l6):
  - one camera for RGB and depth at 66° × 54°;
  - non-square pixels (fx ≈ 123 px, fy ≈ 88 px at 160 × 90);
  - with the treatments on, the depth model's z² noise reaches about 0.91 m at 20 m.
- **Training against evaluation:** CAM-K2 trained at a fixed 205 bars; the grid also has 70, 115 and 160.
- **Idle host** (review r1 E-m3). CAM-K2-E keeps an idle host: no concurrent CPU work during either stage. T-LOAD
  covered the camtrain path, not the realveh modules' CPU-side draws and depth model. The rate gate reads wall times,
  which also need an idle host.
- **Identity coverage** (review r1 E-L1). The identity stage tests one CAM-K1-E cell, K1s1 at 205 bars and seed 4501.
  The K1s2 cells and the other blocks are admitted by extension, with the snapshot and condition bindings.
- **Tests:** `tests/test_camk2e.py` passes (27 tests) with the pins filled and the rate gate repaired.
- **v2 only (`TestV2Only`):**
  - the runner binds `camk2v2_train`, v2's unit folder and the unit "CAM-K2 v2". It commits `camk2v2_train.py`, not
    v1's;
  - the SCOPE and the launcher carry no v1 value;
  - the treated cells carry TW 4.0, `NAVRL_VEH_RAND_LIMIT_MODE=raise` and τ 0.02–0.04 s, with the worst case 3.23;
  - a v2 record and v2 telemetry pass, while these fail:
    - a v1 record (mode "lower", T/W 2.25);
    - mode "none";
    - a limit left at 2 N;
    - a wrong own T/W;
    - a wrong motor maximum;
    - a missing mode or limit;
    - an init line in mode "lower" or "none";
    - reset τ at v1's 0.03–0.06 s.
- **Launcher copy:** reconstructed into CAM-K1-E's launcher exactly, and the runner pins its SHA. The launcher refuses
  a missing, wrong or inconsistent treatment switch, and a valid switch passes to the next guard.
- **Approval:** a valid approval passes. CAM-K1-E's scope, a wrong commit or an empty field fails.
- **Design:**
  - the plan has 60 cells in the fixed order: 48 main, and 12 nominal at 70 and 205 bars;
  - the module seeds are a function of the cell seed only (matched);
  - the treatment values equal CAM-K2's;
  - the real entry parses the runner's environment in both modes.
- **Receipt:**
  - valid ON and OFF cells pass;
  - these ON deviations each fail: wrong vehicle seed, wrong depth value, wrong robot, ablation `full`, wrong hashed
    SHA, wrong FOV, missing telemetry, no motor updates, a wrong hole share, a missing or uncapped record, a camtrain
    receipt variable, v1-mode telemetry, v1 τ, a v1 ("lower") record, a limit not raised;
  - these OFF deviations each fail: a treatment key, telemetry lines;
  - a cell read with another seed or mode fails.
- **Checkpoints:**
  - TRAINED reports with TRAINED manifests at the pins pass;
  - each of these fails:
    - a placeholder or wrong pin;
    - a failed training;
    - a smoke;
    - a non-TRAINED or mismatched manifest;
    - a missing manifest;
    - a v1 unit, preregistration or treatment set;
  - `test_camk2_pins_are_filled` fails until the pins are filled, by design.
- **Gates:**
  - labels on fixed intervals, including the floor, CRASH_TO_TIMEOUT and a null contrast;
  - gate N's bands;
  - df 4 against df 8, checked against a hand computation;
  - the stratified intervals are not widened by a fixed density effect.
- **Runner end to end, with the simulator call patched, from CAM-K1-E's real cells:**
  - identity, then eval COMPLETE with 60 cells in the fixed order;
  - these are refused or fail:
    - eval without an identity pass (PREFLIGHT_FAIL);
    - eval after a failed identity in the same parent (PREFLIGHT_FAIL);
    - an OFF identity cell that differs (IDENTITY_FAIL);
    - an ON negative cell that equals CAM-K1-E's (IDENTITY_FAIL);
    - a wrong FOV in a K2s2 cell, which stops the eval.
- **Analysis end to end, on synthetic eval folders built from CAM-K1-E's real cells:**
  - a known adaptation reads MATERIAL_ADAPTATION, with all five controls as required;
  - one unadapted seed gives SEEDS_DISAGREE;
  - EVAL_BINDING_FAIL for each of: a K2 arm identical to K1, a wrong module seed in a receipt, an unexpected snapshot
    key, a changed result condition, and missing telemetry.
- **Controls on CAM-K1-E's real data:** WORSE on F66 − K1s1; not MATERIAL_ADAPTATION on K1s1 − K1s2; the manifest
  binds; the output refuses to overwrite.

## Review

- A results review follows the run. Until it ends, the readings are CONDITIONAL.

## Design review r1 (Claude Opus 5.5, fresh context, 2026-11-08 16:47–17:10 KST): AMEND_FIRST, repaired as follows

The review is `~/motar_tmp/camk2_20261108/review_v2_r1/REVIEW.md`.

| Finding | Repair |
| --- | --- |
| **E-M1 MAJOR** gate A confounds adaptation with 1,000 more epochs | Gate I, the interaction on the 6 nominal blocks (df 4), reported with a label. "Adaptation" only with gate A MATERIAL/SMALL and gate I INTERACTION_NEGATIVE (`adaptation_supported`). Gate A otherwise reads "lower crash under the treatments". Synthetic test: a uniform improvement, equal ON and OFF, reads gate A MATERIAL_ADAPTATION but not adaptation |
| **E-M2 MAJOR** no control reaches a passing label; gate N has none | Three controls on CAM-K1-E's data: A on K1s1 − F66 (MATERIAL_ADAPTATION), N on K1s2 − K1s1 (NO_MATERIAL_LOSS), N on F66 − K1s1 (MATERIAL_LOSS). Computed: CONTROLS_OK (table above). Resolution recorded |
| E-m1 OFF is no longer easier | "Nominal" = F's vehicle (T/W 3.26, τ 0.04 s, exact depth), in the question, gate N and the wording |
| E-m2 no rate gate; `--max-hours` free | `--max-hours` fixed at 3.5. Eval refuses unless 63 × the mean identity cell wall × 1.25 ≤ 3.5 h (tested both ways); replaced after review r2 R2-1 by the CAM-K1-E-ratio rule above |
| E-m3 T-LOAD coverage | Idle host for both stages |
| E-m4 approval | A new scoped approval, citing the 12:07 and 16:10 answers (Approval paragraph) |
| E-m5 commit order | Written above and in IMPLEMENTATION.md |
| E-L1 | Identity coverage disclosed |
| E-L2 | `K1s1_on_repeat` must equal `K1s1_on_negative` (tested, with a failing case) |
| E-L3 | Bootstrap coarseness disclosed |
| (v2 M-1, shared) | Per-cell raise counters and live read-back required; shares reported per treated cell |

The results go to `docs/evidence/camonly_2026-11-04/camk2e/`.

## Decisions taken before the commit

1. **Approval:** the user's ~23:30 KST answer and message (Approval paragraph).
2. **Gate bands:** gate A's band (5 pp) and gate N's 2-density design are kept as reviewed.
3. **Unbundling** stays open: this unit cannot attribute an effect to one treatment.
4. **Vehicle data:** 4S and the battery in the 1.1 kg are confirmed. They do not change v2's TW_NOMINAL or τ. The
   wheelbase is still open (it affects only the disclosed rotational estimate).
