# Repository reorganisation record (2026-09-24)

Branch `cleanup/repository-docs-site-20260924`, created from `7c40d5d`, which is `main` and one local
commit ahead of `origin/main` (`7fb547d`).
- No experiment, training or GPU evaluation was run.
- No measured value, preregistration, result record or frozen contract was changed.
- Wording that misstated a record was corrected, as listed below.

This file replaces the first pass's separate plan, delete-candidate and archive-candidate notes. The
companion files are:
- [`new_file_budget.md`](new_file_budget.md): why each net-new tracked file exists.
- [`preserved_contract_paths.md`](preserved_contract_paths.md): paths that never move.
- [`inventory_2026-09-24.json`](inventory_2026-09-24.json): the census at the base commit, plus every
  Markdown file with the action taken.

## 1. Model: one job per canonical file

| Question | Canonical file |
| --- | --- |
| What is MOTAR? | `README.md` |
| What are we researching? | `PROJECT.md` |
| How does the system work? | `ARCHITECTURE.md` |
| What results are established? | `docs/EVIDENCE.md` |
| How do I reproduce it? | `docs/REPRODUCIBILITY.md` |
| How do I operate it? | `docs/OPERATIONS.md` |
| What happened historically? | `docs/HISTORY.md` |
| How should the site and figures look? | `DESIGN_SYSTEM.md` |
| Where is everything? | `docs/README.md` (map and source-of-truth table) |

That is nine reader-facing canonical documents. `AGENTS.md` holds the agent rules, and `CLAUDE.md`
imports it; neither is part of the reader path.

## 2. First pass (moves, deletions, test contracts)

- **Nothing path-bound moved.** Every candidate was checked with `git grep` for its full path and its
  basename.
- **Archived with `git mv`:** nine documents, three tools and the Cursor `research-status` skill. Each
  had no binding reference. The old-to-new table is in [`docs/archive/README.md`](../archive/README.md).
- **Kept in place with a status banner:** `VERIFICATION.md`, `OPERATIONS.md`, `RESEARCH_PLAN.md` and
  four dated documents. Tests or receipts parse them, and a SHA-256 search found no pin on any of them.
- **Deleted (untracked, regenerable only):**
  - 39 `__pycache__/` directories outside `results/` (16 MB);
  - eight `aerial_gym/examples/*_100.{png,gif}` renders (364 KB) with zero references.
- **Code:** the dead `_latest_barprobe()` was removed from `tools/update_status_snapshot.py`.
- **No tracked file was deleted.**
- **Site:** `docs/status/index.html` became the project page. The earlier page moved to
  `docs/status/evidence.html` and keeps its tables and contracts. The frozen browser preview
  `GT_BROWSER_EPISODE_V1` is embedded unchanged except for its figure label.

| Test or checker | Change | Why |
| --- | --- | --- |
| `tools/check_public_docs.py` | New README headings. README numbers are allowed only where each one is bound to a record | Binding every number is stricter than banning them |
| `tests/test_public_docs_consistency.py` | The README may show two images from `docs/assets/paper/final/` | Graphical abstract and system map |
| `tests/test_research_overview.py`, `test_site_*`, `test_repository_claims.py` | Detailed-page contracts retargeted to `evidence.html`; guards cover both pages | The page moved; the guards got stricter |
| `tests/test_public_status_manifest.js` | Expects TM_E2 `COMPLETED / E2_NO_DEGRADATION_VS_H` | Already failing at `7c40d5d`: the registry had recorded the result and the test was never updated |

## 3. Second pass (this revision)

| Change | Files | Why |
| --- | --- | --- |
| PNG and PDF renders of the final figures are no longer tracked | −14 | The generator rebuilds them byte-exactly and `manifest.json` pins their hashes. `.gitignore` excludes them, and `--check` verifies them |
| Six maintenance notes consolidated into this record, the budget and one inventory | −6, +3 | Temporary reports minimised |
| Verbatim copies of the old `CLAUDE.md` and `docs/README.md` removed | −2 | Identical at `origin/main`: `git show 7fb547d:CLAUDE.md` and `git show 7fb547d:docs/README.md` |
| `tools/archive/README.md` folded into `docs/archive/README.md` | −1 | One archive index |
| Test profiles (`tools/run_tests.py`) | +1 | The documented default command must end with 0 failures and 0 errors on a CPU-only machine |
| Figures 1–6 redrawn at their printed width (7 in) | 0 | The smallest text had printed at 5.0–6.0 pt. Figure 1's evidence chain became five full-width rows, Figure 6 moved method families under the system names, and notes moved out of crowded panels. The printed minimum is now 7.5 pt, and 7.0 pt only for decorative grid labels in Figure 1's schematic. No value changed; `tests/test_paper_figures_final.py` enforces the sizes |
| `results/MANIFEST.json` rebuilt | 0 | 201 → 207 entries; see section 4 |

Counts before and after are in [`new_file_budget.md`](new_file_budget.md).

## 4. P0 claim audit (2026-09-24)

Every current surface was searched: the canonical docs, both site pages, the paper outline, spine and
claim matrix of 2026-09-19, the registries and the final figures. Guards in
`tests/test_canonical_docs.py` (`ClaimConsistencyTest`) now keep each finding fixed.

| Item | Finding | Resolution |
| --- | --- | --- |
| Manifest count | `results/MANIFEST.json` → `counts.results` = **201** (build of 2026-09-14). The paper drafts said 204; no committed manifest ever held 204, since it was typed in `a4badde`. "204 indexed results **with receipts**" also overstated receipts: 31 of 201 directories have one | **Rebuilt on 2026-09-24** with `tools/build_result_manifest.py`, after a dry run to the scratchpad matched: **207** entries. The six new entries are the 09-17/09-18 directories, including the target-motion evaluation. The only change to an existing entry is D8b `files` 442 → 443, from an untracked, ignored `__pycache__/audit.cpython-313.pyc` written on 2026-09-17 when `tests/test_d8b_archive.py` imported `audit.py` under Python 3.13. No tracked result file changed. Current surfaces say 207 with the build date, and a test binds both to the manifest |
| "Twelve contrasts" | The preregistration (Amendment 1 §A1.3) defines the twelve as the **exploratory per-density contrasts** E0/E1/E2 − H. Only **10 of 12** are 3/3 sign-consistent: E2 − H is mixed at 115 bars (−0.11 / +2.10 / +1.57) and at 160 bars (+0.49 / +1.03 / −0.25). Across capture, crash and timeout, 17 of the 18 arm-level contrasts are consistent; E2 − H crash is mixed | "All twelve" removed. Current wording: "All six reported arm-level capture contrasts were sign-consistent across the three evaluation seeds", with the mixed crash and per-density contrasts named |
| Single policy | Two frozen checkpoints carry the results. **Policy F** (ep25000 + riskcap, `f7022139…`) is used for the P10 frozen cost, safety geometry and target motion. **Policy R** (ref5in D1 ep1900, `197ea269…`) is used for D8b. The outline said "single frozen policy" and "the **same** frozen policy loses −48.967 pp"; the matrix called the D8b loss "an order larger than" the policy-F cost | F and R are named everywhere, and cross-policy magnitude comparisons were removed |
| Inferential units | **E3-S 6.2 %** is a median of 9 block medians from one flight, so the 3,107 frames are not samples. **Safety:** the headline interval pools 15 seed × density cells with episode-binomial errors, while the seed-level interval over 3 seeds is **[−2.42, −0.57]**. **P10 frozen cost:** the interval is episode-level over two evaluation seeds, and the three campaigns are deterministic re-runs. **D8b:** paired evaluation seed, n = 3. **Target motion:** evaluation seed, n = 3 | Each headline row states its unit, in the README, EVIDENCE, the site, the registry (`ci95_unit`) and the figure legends |
| Other overstatements | The spine said readaptation "recovers part of" the cost, and README/site said "every comparison is preregistered". The registry's P10 limitation named "three training seeds" for a measurement that involves none. `docs/REPRODUCIBILITY.md` and `docs/OPERATIONS.md` said CUDA tests "skip themselves" when two of them fail | Corrected |

**Not changed, reported instead:**
- `docs/audits/target_motion_protocol_deviations_2026-09-19.md` line 152 ("all twelve contrasts … are
  unchanged") is a dated audit record whose "twelve" is undefined. Its claim is non-change, not sign
  consistency.
- `docs/status/status.json` labels the 15 safety cells "REPLICATION". The field is rendered nowhere,
  and the frozen browser preview reads that file.
- The pinned 2026-09-18 positioning figure says "15/15 cells".

**New finding for the replication.**
- The evaluator stops when the cumulative count reaches `n_games`. It then drops the episodes still
  running on each environment, and those are disproportionately long timeouts.
- That can shift absolute rates by an arm-dependent amount. A CPU toy model, not a measurement, gives
  about 0.4–1.9 pp for timeout rates of 1–9 %.
- The 09-18 result's worst-case bound (≤ 0.018 pp) covers only the +15 extra records, not this effect.
- The replication preregistration therefore uses a fixed per-environment quota and records both
  estimators.

## 5. Documentation drift left for the user

These lines are in records, where editing is a research decision.

| Where | Drift | Correct statement |
| --- | --- | --- |
| `docs/research_status_registry.json` | TM_E0/TM_E1 evidence points at code | Their measured −3.58 / −2.82 pp are in the target-motion result |
| Registry `temporal_perception_selector` | Pairs the P7 Transformer with the S4 utility 0.4701 | S4 tested the later P7c-v2 |
| `external_baseline_selection_2026-09-19.{json,md}` | Elastic Tracker licence `NEEDS_CONFIRMATION`; B0 cause "disk at 97 %" | GPL-3.0 (B0 record); the disk was recovered on 09-21, and the blocker is `sudo` |
| `WORKLOG.md` 2026-09-19 tables | E0 timeout 7.04 %, E1 crash 10.49 %, E2 crash 9.61 % | 7.11 / 10.53 / 9.60 % (`canonical_summary.json`) |
| `docs/prereg_2026-09-17_target_motion_complexity_e0_e1_e2.md` header | `PREREGISTERED / NOT_RUN` | A frozen preregistration; the result document records the run |
| `tools/build_target_motion_figures.py` | No `--check` | The final figures read `canonical_summary.json` |
| `.claude/skills/navrl/SKILL.md` | Session-start text predates the 09-18 evaluation | `AGENTS.md` is canonical |
