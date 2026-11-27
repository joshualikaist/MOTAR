# CAM-K2-E v1: round-3 check (2026-11-08)

- **Reviewer:** Claude Opus 5.5 (fresh context), read-only. This file is the only one written. No GPU.
- **Worktree:** `MOTAR-camonly`, HEAD `ceab4a0`; the CAM-K2-E files are still uncommitted.
- **Tests:** `tests/test_camk2e.py` with the required CPU command form on cores 12 and 13: **Ran 27, OK**. I ran it
  twice: the first run's summary line was cut off by my `tail`, so I reran it to a log file.
- **Scope:** the R2-1 gate repair, its call site, and the R2-2..R2-5 text repairs.

## Verdict: **PASS_WITH_LIMITS**

The gate is correct. Some leftover text still states the old gate or contradicts the repairs (L1–L4 below). None of
it changes a rule, but fix it before the commit.

## R2-1: the rate gate (`tools/camk2e_run.py:93, 367-390`)

- **Logic:** the gate computes ratio = mean(4 identity walls) / mean(CAM-K1-E identity walls). It refuses if
  60 × mean(CAM-K1-E eval walls) × ratio × 1.10 > 3.5 h. It fails closed in two cases:
  - fewer or more than 4 walls, or any wall that is not positive;
  - CAM-K1-E reference files that cannot be read (OSError, ValueError, KeyError, or an empty list).

  `wall_s` comes from `gr_run.run_cell` (`gr_run.py:197`).
- **Real CAM-K1-E files** (`~/motar_tmp/camk1e_20261108/raw/`):
  - `identity_01`: IDENTITY_PASS, 3 cells, all 205 bars / 4501, walls 173.1, 156.6 and 179.2 s, mean **169.64 s**;
  - `eval_01`: COMPLETE, 36 cells, mean **158.00 s**.

  | identity mean | ratio | projected | result |
  | --- | --- | --- | --- |
  | 170 s | 1.002 | 2.90 h | PASS |
  | 175 s | 1.032 | 2.99 h | PASS |
  | 180 s | 1.061 | 3.07 h | PASS |
  | 185 s | 1.091 | 3.16 h | PASS |
  | 190 s | 1.120 | 3.24 h | PASS |
  | 200 s | 1.179 | 3.42 h | PASS |
  | 210 s | 1.238 | 3.59 h | REFUSE |
  | 300 s | 1.768 | 5.12 h | REFUSE |

  - The threshold is an identity mean of 205.0 s, that is ratio 1.208. It matches R2's "r ≲ 1.21".
  - A normal run (170–190 s) passes with 0.26–0.60 h of headroom.
  - A pathologically slow run (300 s) refuses.
  - At the threshold, the projection without the margin is 3.18 h, so a passing run should fit the 3.5 h cap.
- **Call site** (`:487-489`):
  - `rate_problems(irep, k1e)` receives the parsed eval-stage identity report and `k1e`, the resolved `--camk1e-raw`
    path.
  - Before that call, the runner checks that `irep["camk1e_raw"] == str(k1e)`. So the reference really is the same
    CAM-K1-E folder the identity stage used.
  - Preflight already requires `eval_01/eval_run_report.json` there. The gate also reads `identity_01/`, and a missing
    `identity_01/` fails closed.
- **Tests:**
  - `:222` pins (60, 1.10).
  - `:554-558` checks three cases on the real folder: 150 s passes, realistic 175 s passes, 300 s refuses.
  - `:584` checks the end-to-end refusal of the slow mode.

  A caveat: when the CAM-K1-E folder is absent the gate cannot be checked here, because `K1E` is the real folder.

## Text repairs

- **R2-2:** done as a disclosure: the prereg says gate A alone means "lower crash under the treatments", and the word
  needs `adaptation_supported` (prereg `:51-54`, `:275`). The labels were not renamed. The gate table row still reads
  "**A** (adaptation, gated)" (`:268`), which is acceptable given the rule.
- **R2-3:** done.
  - 0.70–0.89 m is corrected. The inline "(corrected in review r2 R2-3 …)" note sits mid-sentence before "bars." on
    the next line; move it.
  - "crash fraction … 0.71 against 0.73; unpaired" is fixed.
  - `barprobe v2` / `speedgov` are NOT_RECORDED unless read post hoc and labelled (`:48-50`).
- **R2-4:** partly done.
  - Done: "Tests not yet run" is gone; "all five controls" (`:430`); the launcher's T/W 4.2 is disclosed (`:57`), and
    the launcher is left pinned.
  - **Not done:** see L3.
- **R2-5:** done (`:55`).

## Leftovers (LOW, text only; fix before the commit)

- **L1:** the prereg Budget still reads "about 64 cells × 2.6 min ≈ 2.8 h" (`:218`). R2-1 asked for this fix, because
  the line mixes the two stages and the cap is per stage. Say identity ≈ 4 × 3 min, and eval ≈ 60 × 158 s ≈ 2.6 h.
- **L2:** the docstring in `tools/camk2e_run.py:31-32` still states the old rule: "RATE_CELLS (63) x the mean identity
  cell wall time x RATE_MARGIN (1.25)". The comment at `tests/test_camk2e.py:584` also still reads
  "63 x 300 s x 1.25". Update both to the ratio rule.
- **L3:** two R2-4 items remain in the prereg:
  - the blank line between the gate N and gate I rows still splits the gate table;
  - the controls table still has two header rows (`:291-294`).

  I did not re-check the stray bullet at `:174` or the Approval nesting.
- **L4:** move the R2-3 parenthesis, as noted above.
