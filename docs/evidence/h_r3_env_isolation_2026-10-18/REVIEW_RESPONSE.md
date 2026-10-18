# Review response and measured cause qualification

Fable5.1 supports narrowly scoped Mechanics GO; no BLOCKER/HIGH. Review source scope and no raw/test execution are explicit. No production/source change or additional GPU follows this review.

- Accepted LOW: transform binding qualified name is not in process metadata. Existing source resolves the exact production callable and forced selected-env parity is bitwise PASS, so this is not a result blocker. Add binding provenance only in a separately approved future runner source; do not alter the frozen GPU source.
- Overlap detection also has frame/reset/geometry/root-write chronology checks, all PASS. No gate is relaxed.
- SHA-field max_abs_delta is only a digest-byte diagnostic and has no physical meaning. Physical sensor deltas come from numeric RGB/depth/LiDAR fields.
- N16 blindspot specifically involved natural resets also occurring in the baseline. ISO-Q02 demonstrates an empty baseline reset set at2, force127 only, and a positive legacy N128 negative control. Do not generalize the review wording to claim every small-N design is inherently blind.

## Post-review read-only root measurement

The CPU quaternion normalization fixture is a controlled example, **not a measured explanation of the actual GPU root drift**. The reviewer correctly requires this distinction. Saved legacy-normal initial versus step2 mapped static actor roots (excluding127) contain38100 actors:19364 positions differ (max1.52587890625e-5m), quaternion changes=0, linear/angular velocities=0. Across304800 mapped vertex rows,298488 vertex delta rows exactly equal the associated root translation delta; maximum residual1.9073486328125e-6m is consistent with float32 transform arithmetic. This check does not introduce a PASS tolerance or replace the original exact equality gates.

Therefore observed GPU evidence supports **small static-root translation representation drift plus unrelated global geometry refresh**, not a claim that GPU quaternion normalization was measured. The precise PhysX/internal stage producing the translation drift is not localized; no unique world-origin quantization cause is claimed. Saved roots at the same step remain identical across the legacy normal/forced pair: only the extra reset refreshes the unselected cached geometry and triggers sensor divergence. Selected-only intervention removes this refresh and preserves exact state/sensor histories. The global-write/selected-refit mechanism is qualified in this diagnostic; the internal rounding stage and historical14 P02 causes remain unproven.

Reproduction: root_representation_check.py reads only existing ISO-Q02 NPZ frames and writes scratch JSON, never raw. See root_representation_check.json for exact measurements. This post-review check was not performed by Fable and is not silently added to its verbatim review.

H-R3 remains PROVISIONAL / CONFIRMATION HOLD. B4/B5 next work is design only. No P03, confirmation, PPO, Planner or main merge follows Mechanics GO.

## Publication contract

Preliminary read-only full unpublished-history scan of e6814a6..be4514b covered4 commits/28 new blob versions and all commit headers/messages, with no size skips: no secret candidates, private credential paths, binary/raw/checkpoint payload, new symlink/gitlink/results paths or main-WIP block matches.296 inherited symlinks are already identical on the remote branch. Context-reviewed absolute paths are public project/interpreter/Warp-cache/reproduction provenance, test fixtures or schema fragments; no private external file content is included. This finite scan does not prove absence of arbitrary encoded secrets.

The final review/evidence commit must also pass the same full-history scan against advertised origin/codex/mt-p1 e6814a6 before ordinary single-ref push. Exact final-HEAD scan, manual classifications and remote acknowledgement will be preserved outside Git in a fresh publication receipt/manifest (avoids a self-referential commit hash). Expected remote git@github.com:joshualikaist/MOTAR.git, destination HEAD:refs/heads/codex/mt-p1 only. Main/force/history rewriting remain prohibited. A failed scan blocks push.
