# MT precision results: preservation, approval deviation and reporting boundaries

Status: follow-up record only. No historical result, preregistration or authority statement is
rewritten by this note. No new GPU execution, training, merge or push is authorized.

## Approval confirmation for KROC27-A-M02

The committed amendment 2 states that the agent proceeded under a delegation after the
MT-P2-C01 canary. That statement remains unchanged at
`docs/plans/prereg_kroc2027_A_v1_amendment2_2026-10-16.md`, committed in `ea67728`.

The user's subsequent instruction explicitly identifies a procedural omission: **a separate,
run-specific GPU approval confirmation for the A3/A4 main run was missing**. Record the status as
`SEPARATE_RUN_APPROVAL_CONFIRMATION_MISSING`. A committed amendment, a runner receipt, a general
delegation and a successful numerical outcome do not by themselves document that separate
confirmation. This follow-up acknowledgement is not a retroactive claim that the original
approval procedure was complete.

Keep the original raw data, manifests, launch receipts, amendment and analysis unchanged. The
approval deviation is a separate provenance annotation; it neither erases the observations nor
upgrades their evidential independence. Future GPU work requires a new explicit run approval and
a committed preregistration before execution, as required by `AGENTS.md`.

## Git and backup handling

- `main` remains at `cb644bbe46456b1900020177617a13040a3f1552`.
- The reviewed working branch starts this follow-up at `c038ba5`. Git reports **four** commits
  ahead of main (`c4880be`, `4bba883`, `ea67728`, `c038ba5`), rather than the five stated in the
  preceding narrative. No commit, merge, push, stash operation or WIP rearrangement is part of
  this follow-up.
- `results/mt_p1_precision_v1` in the worktree is an untracked symlink to raw results in the
  primary checkout. A merge does not stage or commit either the symlink target or untracked raw
  files. Do not stage that symlink as a substitute for evidence.
- Proposed compact evidence is kept as ordinary files under
  `docs/evidence/mt_precision_review_2026-10-16/`. Its index binds byte-identical summaries,
  analysis, raw manifests and backup digests to their original paths. Preparing these files does
  not commit them.
- The existing archives are on the same filesystem as raw results. They are verified local
  copies, not an external or independently durable backup. Preserve raw, archives, all manifest
  versions that remain, WIP and stash until the user chooses an external destination and its
  completed copy has been verified. No cleanup or deletion is authorized.

## Wording for subsequent paper analysis

Use the audited compact numerical record in
`docs/evidence/mt_precision_review_2026-10-16/numerical_audit.json` for exact estimates and
intervals. The approved qualitative decisions are H-A `SUPPORTED`, H-A2 `INCONCLUSIVE`, and
H-B `INCONCLUSIVE`.

Suggested scope statement:

> The initial-observation orientation contrast was supported under the tested static and
> E2_direct_l2_v1 moving-target conditions. Hidden starts were outside the initial field of view,
> with initially clear line of sight; they were not post-acquisition occlusion trials. The
> target-motion interaction was inconclusive and does not establish equivalence. The A1/A2 cells
> and their contrast had already been observed before amendment 2 and were reused unchanged;
> consequently the combined analysis was not fully blinded or a new independent replication of
> the static contrast. The moving cells were collected after amendment 2 under the new precision
> lineage. Neither Experiment A nor B measures recovery after a previously acquired target is
> occluded for a long interval.

Retain the numerical H-A finding while disclosing the amendment timing, prior knowledge and
reuse in the same place. Do not describe Holm correction as restoring blinding or independence.
The interaction confidence interval is neither an equivalence test nor proof of no interaction.
An inconclusive H-B result is not evidence that every active recovery method is ineffective.

All recorded motion gates passing means compliance with their declared numerical tolerances.
It does not establish substantial physical headroom: the audited acceleration and turn rate
reach essentially their nominal caps. Use "within the preregistered numerical tolerances";
do not claim a strict real-number bound or a general collision-safety guarantee from finite runs.

The new E2_direct_l2_v1 observations do not repair or retroactively validate legacy E1/E2/H
trajectories. Historical comparisons remain conditional on their recorded executor until the
separate read-only TF32 audit establishes their permissible reuse. Preserve the old paper and
result records; use this dated note as a reporting qualification rather than silently replacing
their claims.
