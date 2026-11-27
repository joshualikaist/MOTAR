# CAM-K2-E v1: round-2 design check (2026-11-08)

- **Reviewer:** Claude Opus 5.5 (fresh context), read-only. This file is the only one written.
- **Worktree:** `MOTAR-camonly`, HEAD `ceab4a0`. Five files are uncommitted: the prereg, `tools/camk2e_run.py`,
  `tools/camk2e_analyse.py`, `eval_navrl_camk2e_arms.sh` and `tests/test_camk2e.py`.
- **Run on CPU:** `tests/test_camk2e.py` once, on cores 12 and 13 with the required command form: **Ran 27, OK**.
  I also called `camk2e_run.checkpoints()` once against the real files (no GPU).
- **Scope:** the five checks the brief asked for. No GPU smoke was possible, so play-mode telemetry is checked by
  reading the code only.

## Verdict: **AMEND_FIRST**

One finding fails a valid run almost by construction: the rate gate, R2-1. The other findings are wording and
consistency. Every r1 finding is otherwise resolved.

## (1) The r1 CAM-K2-E findings

| r1 | Resolved? | Where |
| --- | --- | --- |
| E-M1 | Yes | `gate_i`, `adaptation_supported` and the I_s weights (`camk2e_analyse.py:100-115, 307-308, 319-322`); prereg gate table and wording rule; `test_uniform_improvement_is_not_adaptation`. The "adaptation" word needs both seeds (`all(supported.values())`), as the prereg says. See R2-2 for the label names. |
| E-M2 | Yes | 5 controls (`:155-183`). `controls_r1_20261108.json` reads CONTROLS_OK with the manifest bound. Every number in the prereg table matches the JSON, as do the half-widths 0.58 / 0.86 and 0.81 / 1.44 pp. The gap is disclosed. |
| E-m1 | Yes | Question 2, gate N and "may be said" define nominal as F's vehicle (T/W 3.26, τ 0.04 s, exact depth). |
| E-m2 | **Implemented, but miscalibrated** | `--max-hours` is fixed at 3.5 (`camk2e_run.py:446`). The rate gate is the problem: see R2-1. |
| E-m3 | Yes | Both stages run on an idle host (Disclosed limits). |
| E-m4 | Yes | New scoped approval; the runner requires SCOPE and HEAD (`preflight`). |
| E-m5 | Yes | The commit order is in the prereg. |
| E-L1, E-L2, E-L3 | Yes | L2: `K1s1_on_repeat` must equal `K1s1_on_negative` (`camk2e_run.py:491-505`). L1 and L3 are disclosed. |

## (2) The pins and how the runner accepts the real files

- `K2_SHA` matches both TRAINED reports exactly:
  - s1 `769a26f4…de04e8a`;
  - s2 `e76eb048…88321db6`.
- The train reports the runner reads (`~/motar_tmp/camk2v2_20261108/train_s{1,2}/train_report.json`) are
  byte-identical to the committed copies `train_report_v2_s{1,2}.json` (SHA-256 `25fa1550…` and `3ff8c6d7…`).
- Each report has verdict TRAINED, the right seed, `development_smoke` false and v2's prereg and scope.
- Each sidecar manifest has verdict TRAINED, the pinned `checkpoint_sha256`, robot navrl_quad, unit "CAM-K2 v2",
  v2's prereg and v2's treatments (seeds 4911/4921 and 4912/4922).
- `checkpoints()` on the real files returns **no problems** and binds:
  - K1s1 `d7bf51a8`, K1s2 `5128bf2e`;
  - K2s1 `769a26f4`, K2s2 `e76eb048`.

  So the copy SHA, the manifest SHA against the report, and the `treatments == T2.expected_settings(seed)` check all
  pass on the real records.
- The launcher's SHA equals `LAUNCHER_SHA` (`e5923d8e…`).

## (3) Gates I / A / N, the controls and the "adaptation" rule

- The gate rules are implemented as the prereg's table states:
  - I: disagreement first, then NEGATIVE, POSITIVE, NOT_DETECTED;
  - A: unchanged from r1, with CRASH_TO_TIMEOUT;
  - N: `E1.gate_p` with an assert on its 3 pp band.
- Gate N has df 4 over 6 blocks (T_CRIT 2.776). Its K1s1/K1s2 reference mapping, including the `_nom` arms, is
  correct.
- Under CONTROL_FAIL, every gate reads null and so does `adaptation_supported`.

## (4) Could a valid run fail by construction, or an invalid one pass?

**Valid runs, path by path (reading only):**
- **Treatment env:**
  - ON passes all 15 keys, with the per-cell seeds and `LIMIT_MODE=raise`; I checked the actual `treatment_env(4501)`
    output.
  - OFF passes none. `gr_run.cell_env` strips the inherited `NAVRL_*` (`gr_run.py:102`).
  - The launcher's `case` on `CAMK2E_TREATMENTS` checks both switches. The entry refuses module parameters while
    their switch is off.
  - The snapshot check requires the exact treatment-key set (`camk2e_run.py:304-307`).
  - This is correct for both modes.
- **Play-mode telemetry:**
  - `install()` binds the stream to `sys.stderr` at install time, outside rl_games' quiet construction.
  - The launcher sends `play_navrl.sh` stdout and stderr into `cell.log` (`eval_navrl_camk2e_arms.sh:205`).
  - Reset lines are logged for calls 1–3 after init. Quota-16 cells reset often, so these appear.
  - The update line comes at the first motor update. In raise mode, `raise_counters()` returns its keys even before
    the first update, with zeros and the live read-back. So every reset and update line carries the 5 keys that
    `raise_telemetry_problems` requires, and the first update line has `cmd_elements > 0`.
  - The depth model emits its summary at STATS_CALLS frames.
  - I found nothing that is emitted in training but not in play. This is NOT_TESTED on the GPU. Stage 1's ON cells are
    the first execution, and they fail closed at a cost of at most 4 cells.
- **Identity:**
  - `identity_compare` drops only the nonce and the checkpoint path.
  - CAM-K1-E's `result.json` condition holds no commit, receipt or run-path field: of its 79 keys, the only hashes are
    those of the robot, assets, P9 and the D8 mesh.
  - So the OFF identity is not blocked by construction.

**R2-1 (MAJOR): the rate gate refuses a normal eval** (`camk2e_run.py:93` and `rate_problems` at `:367`; prereg
`:211-212`).
- The gate requires a mean identity cell wall ≤ 3.5 h / (63 × 1.25) = 160 s.
- All 4 identity cells run at **205 bars**, the slowest density. In CAM-K1-E's own records
  (`~/motar_tmp/camk1e_20261108/raw/`):
  - its identity cells, also 205 bars and seed 4501, had a mean of **169.6 s**;
  - its eval cells had a mean of 145 / 154 / 164 / **168 s** at 70 / 115 / 160 / 205 bars;
  - the K1s1 205/4501 cell, which `K1s1_off_identity` must reproduce exactly, took **181 s**.
- So the identity mean will be about 170–180 s before any depth-model overhead, and the gate refuses
  (63 × 170 × 1.25 = 3.72 h > 3.5 h). Meanwhile the real projection is about 60 × 158 s ≈ 2.6 h.
- The tests pass only because they use 150 s (`test_camk2e.py:530, 554`).
- **Fix:** measure overhead against the same cells. Take r = mean(identity walls) / mean(CAM-K1-E's identity and
  205-bar/4501 walls), read from `--camk1e-raw`, which is already required. Refuse unless
  60 × (CAM-K1-E eval mean wall) × r × 1.1 ≤ 3.5 h, that is r ≲ 1.21. Or, as a simpler alternative, keep 63 × mean but
  scale by CAM-K1-E's grid mean over its 205-bar mean (158/168).
- Add a test at realistic walls that must pass: about 175 s ON and 170 s OFF. Keep one that must fail, for example
  r ≥ 1.3.
- Update the prereg's Budget text ("a mean of at most 160 s"; "about 64 cells × 2.6 min ≈ 2.8 h" mixes the two
  stages, and the cap is per stage).

**Invalid runs:** I found no path that lets one pass. Missing telemetry, a wrong mode or seed, a wrong limit, OFF
telemetry and a wrong snapshot are each checked again in the analysis (`receipts_ok`), with positive controls in the
tests.

## (5) The "Found before launch" paragraph

**R2-3 (MINOR): one wrong number, one loose description and one missing parser** (prereg `:32-50`).
- **The number:** "centre distance below half-width + 0.5 m, that is 0.61–0.79 m" is wrong.
  - The `bars_h3` half-widths are 0.20–0.40 m (0.4029–0.7902 m boxes).
  - 0.5 m is MinZ 0.4 m plus the camera's 0.10 m x-offset.
  - So the range is about **0.70–0.89 m**.
  - The error comes from my own depth-probe review r1, item 3. That review needs the same erratum.
  - The "≈0.26 m blind approach" (0.5 − r_v) is unaffected.
- **The description:** "0.71 against 0.73" is the crash fraction of episode ends in the last training progress
  window (n ≈ 2050), unpaired and confounded with T/W 4.0. Say "crash fraction in the last training window", not
  "crash rate".
- **The parser:** "`barprobe v2` … reported descriptively" has no parser in `camk2e_analyse.py`. Only `speed_governor`
  is read, through `C2A.governor`. Either add a parser before the commit, or state that `barprobe` lines are
  NOT_RECORDED by the fixed analysis.
- **The rest is accurate:** PASS_WITH_LIMITS, the three contract corrections, "characterises the convention trained
  with", NOT_TESTED, and hit-token not used as a perception metric.

## Other findings

**R2-2 (MINOR): gate A's labels still contain "ADAPTATION"** (`camk2e_analyse.py:84-97`; prereg `:257` "A
(adaptation, gated)" and the "(gate A: label)" in the "may be said" template).
- Reporting "gate A: MATERIAL_ADAPTATION" uses the word while `adaptation_supported` is false.
- **Fix:** rename the labels to CRASH_REDUCTION_MATERIAL / CRASH_REDUCTION_SMALL / NO_DETECTED_REDUCTION (and the
  control's required label), or require every report of a gate A label to carry "(lower crash under the treatments;
  adaptation not supported)" whenever the rule fails.

**R2-4 (MINOR): text consistency** (fix before the commit; none changes a rule).
- `:364` "Tests not yet run … 26 OK, pins FAILS … PUBLIC_CPU has not run" contradicts "Done before the commit" item 3
  (27 pass, which I confirmed).
- `:257-259`: a blank line between the N and I rows splits the gate table.
- `:277-279`: the controls table has two header rows.
- `:164`: the stray bullet "navrl_quad's own 4 × 2 N, with its own T/W equal to 8 / (m g)" belongs under the record
  bullet.
- `:423` "with both controls as required": there are five controls now.
- The Approval paragraph has a broken bullet nesting.
- `eval_navrl_camk2e_arms.sh:7` says "raised to T/W 4.2". v2 is 4.0. Fix it and re-pin `LAUNCHER_SHA`, or disclose it.
  The launcher is SHA-pinned and its reconstruction test only covers the body.

**R2-5 (LOW): gate I has a synthetic test only.**
- Gate I has no control on real data. Its 4-arm contrast on 6 blocks (df 4) is likely wider than gate N's null
  (t half-width 1.44 pp).
- Disclose that INTERACTION_NEGATIVE has no floor, so a small negative interaction suffices, and that its resolution is
  not measured before the run.

## One line per finding

- R2-1 MAJOR: the rate gate's 160 s threshold is below CAM-K1-E's own 205-bar identity walls (169.6 s mean; 181 s
  for the reproduced cell), so the eval refuses. Measure overhead against the same cells.
- R2-2 MINOR: rename gate A's ADAPTATION labels, or qualify them, so the word is not used without gate I.
- R2-3 MINOR: "Found before launch": 0.61–0.79 m should read ≈0.70–0.89 m (my depth-probe review's error too); the
  0.71/0.73 figures are crash fractions; there is no barprobe parser.
- R2-4 MINOR: stale "Tests not yet run", broken tables, a stray bullet, "both controls", and the launcher's "T/W 4.2"
  comment.
- R2-5 LOW: gate I's resolution is unmeasured and it has no floor; disclose.
- Pins, the acceptance of the real reports and manifests, the treatment env in both modes, the telemetry path in play
  (by reading), the controls and the gate logic: PASS.
